# Telink SWire Protocol Decoder for [PulseView/sigrok](https://sigrok.org/)

A protocol decoder for the Telink Single Wire (SWire) interface used in older Telink BLE SoCs (TLSR825x, TLSR826x, etc.).

Protocol description based on [TLSRPGM documentation](https://github.com/pvvx/TLSRPGM/tree/main/TelinkSWire).

**Note:** This decoder has received very limited testing. Verify results before relying on them.

## Installation

Copy the `swire` folder to your [PulseView decoders](https://sigrok.org/wiki/Protocol_decoders) directory:

**Windows:**
```
C:\Program Files\sigrok\PulseView\share\libsigrokdecode\decoders\
```

**Linux:**
```bash
cp -r swire ~/.local/share/libsigrokdecode/decoders/
# or system-wide:
sudo cp -r swire /usr/share/libsigrokdecode/decoders/
```

Restart PulseView after installation.

## Usage

1. Click "Add protocol decoder" and search for SWire
2. Configure options (see below)

## Options

| Option | Default | Description |
|--------|---------|-------------|
| SWire | - | Signal wire data source |
| Bit rate | 960000 | SWire clock rate in bits/second |
| Address bytes | 3 | Address width: 3 for TLSR825x (24-bit), 2 for TLSR826x (16-bit) |

## Decoder Output

| Row | Annotations |
|-----|-------------|
| Bits | Individual bits (CMD bit highlighted) |
| Bytes | Hex values |
| Master | START, ADDR, R/W, write data, triggers, END, errors |
| Slave | Read data |

## Recommended Sample Frequency

- 960 kbps: 20 MHz minimum
- 2 Mbps: 40 MHz minimum

(targeting 4 times the wire rate, which is 4 x 5 the bit rate)

## Protocol Overview

SWire is a single-wire half-duplex protocol with pull-up resistor (idle HIGH).

**Bit encoding** (5 units per bit):
- Bit 0: 1 unit LOW, 4 units HIGH
- Bit 1: 4 units LOW, 1 unit HIGH

**Frame structure:**
- Master bytes: CMD (1 bit) + DATA (8 bits) + END unit = 10 pulses
- Slave bytes: DATA (8 bits) + END unit = 9 pulses (no CMD bit)

**Transaction:**
```
START → ADDR (2-3 bytes) → RW_ID → DATA... → END
```

- START: CMD=1, DATA=0x5A
- END: CMD=1, DATA=0xFF
- RW_ID bit 7: Read (1) / Write (0)
- RW_ID bits 6-0: Slave ID

**Read operations:**

After starting a transaction with RW_ID read bit set the master sends a 1-unit pulse before each slave response byte as a trigger.

