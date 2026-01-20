## Telink SWire protocol decoder.
## Copyright (C) 2026 Stefan Hacker <mail@hacst.net>
##
## This program is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 2 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program; if not, see <http://www.gnu.org/licenses/>.
##

'''
Telink SWire protocol decoder.

Protocol: 5 units per bit
- Bit 0: 1 unit LOW, 4 units HIGH
- Bit 1: 4 units LOW, 1 unit HIGH

Master frame: CMD (1 bit) + DATA (8 bits) + end unit
Slave frame: DATA (8 bits) + end unit (no CMD bit)
Read trigger: 1 unit LOW from master before each slave byte

- CMD=1, DATA=0x5A: START
- CMD=1, DATA=0xFF: END
- CMD=0: data byte

Transaction: START + ADDR + RW_ID + DATA... + END
- RW_ID bit7: 1=read, 0=write
- RW_ID bit6-0: slave ID
'''

import sigrokdecode as srd

# Annotation indices
ANN_BIT = 0
ANN_CMD_BIT = 1
ANN_BYTE = 2
ANN_START = 3
ANN_ADDR = 4
ANN_RW = 5
ANN_DATA_M = 6
ANN_DATA_S = 7
ANN_END = 8
ANN_TRIG = 9
ANN_ERROR = 10

# States
ST_IDLE = 0             # Waiting for START
ST_ADDR_0 = 1           # First address byte
ST_ADDR_N = 2           # Remaining address bytes
ST_RW_ID = 3            # Read/Write + slave ID byte
ST_DATA = 4             # Master write data
ST_READ_TRIG = 5        # Waiting for master trigger pulse
ST_READ_DATA = 6        # Accumulating slave data bits
ST_SLAVE_END = 7        # Slave end unit
ST_READ_TRIG_OR_END = 8 # Next: trigger or END byte?

class Decoder(srd.Decoder):
    api_version = 3
    id = 'swire'
    name = 'SWire'
    longname = 'Telink SWire'
    desc = 'Telink SWire interface used in Telink BLE SoCs (TLSR825x, TLSR826x, etc.).'
    license = 'gplv2+'
    inputs = ['logic']
    outputs = []
    tags = ['Debug/trace']
    channels = ({'id': 'swire', 'name': 'SWire', 'desc': 'Single wire data'},)
    options = (
        {'id': 'bitrate', 'desc': 'Bit rate', 'default': 960000},
        {'id': 'addr_bytes', 'desc': 'Address bytes', 'default': 3, 'values': (2, 3)},
    )
    annotations = (
        ('bit', 'Bit'),
        ('cmd-bit', 'CMD Bit'),
        ('byte', 'Byte'),
        ('start', 'Start'),
        ('addr', 'Address'),
        ('rw', 'R/W'),
        ('data-m', 'Data (Master)'),
        ('data-s', 'Data (Slave)'),
        ('end', 'End'),
        ('trig', 'Read Trigger'),
        ('error', 'Error'),
    )
    annotation_rows = (
        ('bits', 'Bits', (0, 1)),
        ('bytes', 'Bytes', (2,)),
        ('master', 'Master', (3, 4, 5, 6, 8, 9, 10)),
        ('slave', 'Slave', (7,)),
    )

    def __init__(self):
        self.reset()

    def reset(self):
        self.samplerate = None
        self.bits = []  # List of (bit_value, ss, es) tuples for current byte
        self.state = ST_IDLE
        self.addr = 0
        self.addr_start = 0
        self.addr_bytes_remaining = 0
        self.is_read = False
        self.errors = 0

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def metadata(self, key, value):
        if key == srd.SRD_CONF_SAMPLERATE:
            self.samplerate = value

    def _resync(self):
        self.bits = []
        self.errors = 0
        self.state = ST_IDLE
        self.is_read = False

    def _put_ann(self, ss, es, idx, data):
        self.put(ss, es, self.out_ann, [idx, data])

    def _decode_slave_byte(self):
        if len(self.bits) < 8:
            return False
        byte_val = 0
        for i in range(8):
            byte_val = (byte_val << 1) | self.bits[i][0]
        ss, es = self.bits[0][1], self.bits[7][2]
        self._put_ann(ss, es, ANN_BYTE, ['0x%02X' % byte_val])
        self._put_ann(ss, es, ANN_DATA_S, ['0x%02X' % byte_val])
        self.bits = []
        return True

    def _decode_byte(self):
        if len(self.bits) < 9:
            return False
        cmd = self.bits[0][0]
        byte_val = 0
        for i in range(8):
            byte_val = (byte_val << 1) | self.bits[i + 1][0]
        ss, es = self.bits[0][1], self.bits[8][2]

        # START always resyncs, even mid-transaction
        if cmd == 1 and byte_val == 0x5A:
            if self.state != ST_IDLE:
                self._put_ann(ss, es, ANN_ERROR, ['Unexpected START', 'RESYNC'])
            self._put_ann(ss, es, ANN_BYTE, ['0x%02X' % byte_val])
            self._put_ann(ss, es, ANN_START, ['START', 'S'])
            self.state = ST_ADDR_0
            self.addr_bytes_remaining = self.options['addr_bytes']
            self.addr = 0
            self.bits = []
            self.errors = 0
            return True

        self._put_ann(ss, es, ANN_BYTE, ['0x%02X' % byte_val])

        if cmd == 1:
            if byte_val == 0xFF:
                self._put_ann(ss, es, ANN_END, ['END', 'E'])
                self.state = ST_IDLE
                self.is_read = False
            else:
                self._put_ann(ss, es, ANN_ERROR, ['Invalid CMD: 0x%02X' % byte_val, 'BAD CMD'])
                self.state = ST_IDLE
                self.is_read = False
        elif self.state == ST_IDLE:
            self._put_ann(ss, es, ANN_ERROR, ['Missing START', 'NO START'])
            self.bits = []
            return True
        elif self.state == ST_ADDR_0:
            self.addr = byte_val
            self.addr_start = ss
            self.addr_bytes_remaining -= 1
            if self.addr_bytes_remaining > 0:
                self.state = ST_ADDR_N
            else:
                self._put_ann(ss, es, ANN_ADDR, ['ADDR: 0x%02X' % self.addr, '0x%02X' % self.addr])
                self.state = ST_RW_ID
        elif self.state == ST_ADDR_N:
            self.addr = (self.addr << 8) | byte_val
            self.addr_bytes_remaining -= 1
            if self.addr_bytes_remaining == 0:
                fmt = '0x%0' + str(self.options['addr_bytes'] * 2) + 'X'
                self._put_ann(self.addr_start, es, ANN_ADDR, ['ADDR: ' + fmt % self.addr, fmt % self.addr])
                self.state = ST_RW_ID
        elif self.state == ST_RW_ID:
            self.is_read = (byte_val & 0x80) != 0
            slave_id = byte_val & 0x7F
            rw = 'R' if self.is_read else 'W'
            self._put_ann(ss, es, ANN_RW, ['%s ID:%d' % (rw, slave_id), rw])
            self.state = ST_READ_TRIG if self.is_read else ST_DATA
        else:
            self._put_ann(ss, es, ANN_DATA_M, ['0x%02X' % byte_val])

        self.bits = []
        return True

    def decode(self):
        if not self.samplerate:
            raise srd.SamplerateError('Cannot decode without samplerate.')

        unit = self.samplerate / (self.options['bitrate'] * 5)
        thresh = unit * 2.5
        max_errors = 3
        max_bit_gap = unit * 10  # Max gap within a byte (~2 bit times)

        last_rise = None

        while True:
            self.wait({0: 'f'})
            fall = self.samplenum

            # Detect framing error: gap too long within a byte or during read sequence
            in_frame = len(self.bits) > 0 or self.state in (ST_READ_DATA, ST_SLAVE_END)
            if last_rise and in_frame and (fall - last_rise) > max_bit_gap:
                self._put_ann(last_rise, fall, ANN_ERROR, ['Frame gap', 'GAP'])
                self._resync()

            self.wait({0: 'r'})
            rise = self.samplenum
            low_time = rise - fall
            last_rise = rise

            # Check timing validity
            is_valid_0 = unit * 0.5 <= low_time <= unit * 1.5
            is_valid_1 = unit * 3.5 <= low_time <= unit * 4.5

            if not (is_valid_0 or is_valid_1):
                self._put_ann(fall, rise, ANN_ERROR, ['Bad timing', 'TIME'])
                self.errors += 1
                if self.errors >= max_errors:
                    self._resync()
                continue

            self.errors = 0
            bit = 1 if low_time > thresh else 0
            bit_end = int(fall + unit * 5)

            # Read operation states
            if self.state == ST_READ_TRIG:
                self._put_ann(fall, rise, ANN_TRIG, ['Trigger', 'T', '>'])
                self.state = ST_READ_DATA
                continue

            if self.state == ST_READ_DATA:
                self._put_ann(fall, bit_end, ANN_BIT, ['%d' % bit])
                self.bits.append((bit, fall, bit_end))
                if len(self.bits) == 8:
                    self._decode_slave_byte()
                    self.state = ST_SLAVE_END
                continue

            if self.state == ST_SLAVE_END:
                self.state = ST_READ_TRIG_OR_END
                continue

            if self.state == ST_READ_TRIG_OR_END:
                if bit == 1:
                    # CMD=1: END byte starting, switch to master byte decoding
                    self._put_ann(fall, bit_end, ANN_CMD_BIT, ['%d' % bit])
                    self.bits.append((bit, fall, bit_end))
                    self.state = ST_DATA
                else:
                    # Another read trigger
                    self._put_ann(fall, rise, ANN_TRIG, ['Trigger', 'T', '>'])
                    self.state = ST_READ_DATA
                continue

            # Master byte handling
            if len(self.bits) == 9:
                self._decode_byte()
                continue

            ann_idx = ANN_CMD_BIT if len(self.bits) == 0 else ANN_BIT
            self._put_ann(fall, bit_end, ann_idx, ['%d' % bit])
            self.bits.append((bit, fall, bit_end))
