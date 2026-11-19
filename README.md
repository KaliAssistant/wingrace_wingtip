# Wingtip light controller (ATtiny85)

Stand-alone right-wingtip light controller in AVR assembly, driven by a
one-wire serial signal, with a half-duplex bootloader so the firmware can be
updated and read back over the same wire (BLHeli style). Includes a C host tool
that finds the serial port by itself and a Makefile for Linux and macOS.

## Files

| File | What it is |
|---|---|
| `wingtip_ctrl.asm` | Application firmware (lights + RX-only protocol) |
| `wingtip_boot.asm` / `wingtip_boot.hex` | Bootloader (words `0x0E00..0x0FFF`) |
| `blflash.c` | Host flasher (Linux / macOS, plain C, no dependencies) |
| `Makefile` | Builds everything, flashes, installs the loader via ISP |

## Hardware

ATtiny85-20SUR, 5 V, internal RC oscillator at 8 MHz.

| Pin | Function | Notes |
|---|---|---|
| PB0 | `SIG` signal line | 9600 8N1, idle high, external ~10k pull-up |
| PB1 | `STB_B` back strobe, 6700k | active high |
| PB2 | `NAV` green, 3000 mcd | active high |
| PB3 | `BAKL` back light, 4000k | active high |
| PB4 | `STB_F` front strobe, 6700k | active high |
| PB5 | RESET | kept as RESET so ISP keeps working |

LEDs are 5 mm straw hat, 20 mA at 3.1 V.

PB0, PB1 and PB2 are also the ISP MOSI, MISO and SCK lines. The lights may
flicker while programming; disconnect the flight controller's signal output
while using ISP.

**Fuses:** `lfuse = 0xE2` (8 MHz RC, no CKDIV8), `hfuse = 0xDF` (default).

## Application protocol

9600 8N1, one command per byte. Unknown bytes are ignored (`0xF0..0xFF` are
reserved for future use).

| Byte | Action |
|---|---|
| `A2` / `B2` | NAV on / off |
| `A3` / `B3` | Back light on / off |
| `A4` / `B4` | Front strobe steady on / off |
| `C4` | Front strobe: one flash of the stored high time |
| `D4 <code> E4` | Set front strobe high time (see below) |
| `A5` / `B5` | Back strobe steady on / off |
| `C5` | Back strobe: one flash |
| `D5 <code> E5` | Set back strobe high time |
| `F0 B0 07` | Enter the bootloader (warm entry) |

- `TRIG` gives **one** flash; nothing repeats. The high time is only changed
  by `D4`/`D5`. Triggers are ignored while a strobe is held on with `A4`/`A5`.
- `D4 <code> E4`: both following bytes must arrive within about 100 ms or the
  whole command is dropped and the old value is kept.
- Power-up default: all outputs off, high time 50 ms.

High-time code:

| Code | High time |
|---|---|
| `0x00` | 50 ms (default) |
| `0x01` / `0x02` / `0x03` / `0x04` / `0x05` | 1 µs / 10 µs / 50 µs / 100 µs / 500 µs |
| `0x06..0xFF` | `code - 5` ms, i.e. 1..250 ms |

Timing notes:

- ms pulses are timed by Timer0 (front) and Timer1 (back) with a 1 ms tick, so
  they never block the receiver and the two strobes are independent.
- Pulses of 500 µs or less are generated with interrupts off. After such a
  `C4`/`C5`, leave at least 1 ms before the next byte.
- The RC oscillator is calibrated at 3 V and runs faster at 5 V. The link
  tolerates about ±4% baud error. On a real chip tested at 4.9..5.1 V no trim
  was needed. If yours is close to the edge, set
  `OSCCAL_ADJ` in **both** `.asm` files (a signed trim, stay away from the
  `0x7F`/`0x80` boundary).

## Flash layout

| Words | Content |
|---|---|
| `0x0000` | Always `rjmp BOOT_ADDR` (`0xCDFF`); owned by the loader |
| `0x0001..0x0DFE` | Application (vectors + code) |
| `0x0DFF` | Application's original reset vector, moved here by `blflash` |
| `0x0E00..0x0FFF` | Bootloader (~340 words used) |

The application must stay below byte `0x1BFE` and keep word 0 as a plain
`rjmp RESET` (which every avra/avr-gcc image does).

## Bootloader

One-wire, half-duplex, 9600 8N1 on PB0. The loader transmits open-drain
(pin low = output low, pin high = released with the pull-up), so the host and
the chip can never drive the line against each other. It waits about 1 ms
before each reply to give slow adapters time to turn the line around.

**Entry**

- **Cold** (power-up / reset): trims `OSCCAL`, listens for a `HELLO` for about
  0.5 s, then starts the application. This delays power-up by 0.5 s.
- **Warm** (running app receives `F0 B0 07`): listens for about 8 s.
- With no valid application it waits forever.
- The NAV LED is lit while the loader is active.

**Frames**

```
host   -> loader : B1 CMD ARG [64 data bytes if CMD = WRITE] CRCL CRCH
loader -> host   : B2 STATUS [payload if STATUS = 0]         CRCL CRCH
```

CRC is avr-libc `_crc_ccitt_update` with init `0xFFFF` (CRC-16/MCRF4XX, check
value `0x6F91` for `"123456789"`), low byte first, over `CMD ARG DATA` resp.
`STATUS payload`. Bytes inside a frame must follow each other within 100 ms.

```c
uint16_t crc_upd(uint16_t crc, uint8_t d) {
    d ^= (uint8_t)crc;
    d ^= (uint8_t)(d << 4);
    return (((uint16_t)d << 8) | (crc >> 8)) ^ (uint8_t)(d >> 4) ^ ((uint16_t)d << 3);
}
```

| CMD | Meaning | Reply payload |
|---|---|---|
| `01` HELLO | handshake | version, page bytes (64), app pages (112), 0 |
| `02` WRITE | ARG = page 0..111, 64 data bytes: erase, write, verify | none |
| `03` READ | ARG = page 0..127 | 64 bytes |
| `04` RUN | start the application | none |

STATUS: `00` ok, `01` bad CRC, `02` unknown command, `03` page out of range,
`04` verify failed, `05` no valid application.

**Safety**

- Pages 112 and up (the loader itself) cannot be written.
- Every write of page 0 forces word 0 back to `0xCDFF`, so a bad image cannot
  unlink the loader.
- `blflash` blanks page 111 first and writes it last. An interrupted flash
  therefore leaves the loader waiting instead of starting half an application.
- Flash can always be recovered with ISP.

## Build and flash

Requirements: a C compiler, [`avra`](https://github.com/Ro5bert/avra), and
`avrdude` for the one-time ISP step.

```sh
make                         # blflash + wingtip_boot.hex + wingtip_ctrl.hex
```

If avra cannot find `tn85def.inc`, pass its include directory, e.g.
`make AVRA_INC=/usr/share/avra` (Linux) or
`make AVRA_INC=$(brew --prefix)/share/avra` (macOS).

**1. One-time: install the loader with ISP**

```sh
make isp                     # PROGRAMMER=usbasp by default, override as needed
```

This runs `avrdude -p t85 -c $(PROGRAMMER) -U flash:w:wingtip_boot.hex -U lfuse:w:0xE2:m -U hfuse:w:0xDF:m`.
Do not flash the application with ISP afterwards: it would overwrite word 0
and unlink the loader. Use `blflash`.

**ISP notes (blank chips and cheap programmers)**

- A blank ATtiny85 runs at 1 MHz (CKDIV8 programmed), and ISP needs SCK below a
  quarter of the target clock, i.e. under 250 kHz. A programmer with a fixed
  fast SCK, such as a CH341A/B (about 1.3..1.7 MHz), cannot even detect a blank
  chip.
- Fix: set `lfuse = 0xE2` once with a programmer that can slow SCK (USBasp with
  its slow-SCK jumper or `-B`, an Arduino as ISP, ...). The chip then runs at
  8 MHz and accepts SCK up to 2 MHz, so a CH341 works afterwards, though close
  to the limit.
- Keep the ISP wires short, share ground, and put 100 nF across VCC/GND at the
  chip. With long wires a CH341A burned the loader correctly only about once in
  10 tries. Always verify after programming.
- ISP is only needed once. Everything after that goes through `blflash`.

**2. Flash the application over the signal wire**

```sh
make flash                   # then power-cycle the board
make flash ENTER=1           # app already running: it is sent into the loader
make flash ECHO=1            # half-duplex adapter that echoes our own bytes
make flash PORT=/dev/cu.usbserial-1410   # pin a port if auto-detect picks wrong
```

Other targets: `make ports`, `make info`, `make verify`, `make read`
(writes `dump.bin`, the whole 8 KB), `make run`, `make clean`.

## blflash

```
blflash [-p PORT|auto] [-l] [-e] [-E] [-t SEC] [-n]  info | flash FILE | verify FILE | read OUT.bin | run
```

| Option | Meaning |
|---|---|
| `-p PORT` | Serial port. Optional: without it (or `auto`, or the `BLFLASH_PORT` environment variable empty) it probes the USB serial ports and uses the first one where the loader answers |
| `-l` | List the candidate ports and exit |
| `-e` | Half-duplex adapter: discard our own echo |
| `-E` | Send `F0 B0 07` first (application must be running) |
| `-t SEC` | How long to wait for the loader (default 15) |
| `-n` | Do not start the application after flashing |

Candidate ports: Linux `/dev/ttyUSB*`, `/dev/ttyACM*`; macOS `/dev/cu.usbserial*`,
`/dev/cu.usbmodem*`, `/dev/cu.SLAB_USBtoUART*`, `/dev/cu.wchusbserial*`.
Probing sends bytes to every candidate, so pin the port with `-p` if other
serial devices are attached.

Input is Intel HEX (as written by avra) or a raw binary at address 0. The tool
moves the application's reset vector to word `0x0DFF`, patches word 0, flashes,
reads every page back to verify, and starts the application.

## Porting the host side (ATmega328P, flight controller, ...)

Implement the frame format above on any half-duplex 9600 8N1 UART:

1. Optionally send `F0 B0 07` to a running app, then wait about 50 ms (or
   power-cycle and send HELLO frames every 50 ms during the 0.5 s window).
2. Send HELLO until the loader replies.
3. Flush your own echo if the adapter echoes.
4. Wait for each reply (up to about 1.5 s for WRITE) before the next frame.

## Testing

The firmware and the host tool were tested with simavr (ATtiny85 core) and a
pty bridging the PB0 pin to a "serial port": full flash and read-back with the
real `blflash`, warm entry, echo mode, bad CRC, unknown command, out-of-range
pages, page-0 vector protection, resync after a partial frame, auto port
detection with a silent decoy port, and baud error tolerance of about ±4%.

Tested on real hardware: an ATtiny85-20SUR of unknown origin (the laser
marking suggests a clone, this part is hard to get from the usual distributors).
The app and the loader work at 4.9..5.1 V without any `OSCCAL_ADJ`.

Not tested: power loss during a flash write, and macOS.
