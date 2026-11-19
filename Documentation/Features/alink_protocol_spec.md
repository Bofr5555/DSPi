# Multi-Unit Audio Link: Line Protocol

*Protocol version: 1 (superframe byte 1). Status: normative, 2026-11-13 (WP LK1, fixup after review lk). Not yet used by any firmware
image: the sender (WP LK4), the receiver input (LK5), phase lock (LK6) and the repeater (LK8) build on it.*

*2026-11-19 (WP FWTB0, PLAN-AT): apply ops 9 `SIGGEN_START` and 10 `SIGGEN_STOP` (6.3, 6.4); the time-base contract C1-C6
that every alink time-base transport meets (section 11, normative); the carriage of the superframe on S/PDIF (section
12, normative). Specification only: the ops are WP FWTB1, the S/PDIF carriage WP FWSP1 and the S/PDIF roles WP FWSP2.*

The alink carries the audio of a whole system from one sender (the master clock) to any number of receivers (a DSPi
or a K340 with a receiver module) over optical or SFP lines, one frame per sample period, with a 64-bit frame
counter and a control sub-channel. The system design is SYS2 (`~/.cache/dspi-build/work/sys2/SPEC.md`); this
document is the normative line protocol it calls for. The control commands for the alink (`0x8000`-`0x80FF`) are in
`extension_channel_spec.md` section 2.8.

The reference implementation is bit-exact and portable: C in `firmware/DSPi/alink/`, a Python twin in
`tools/alink/`, and shared test vectors in `firmware/tests/alink_vectors.json` (section 9). Where this text and the
reference code disagree, that is a bug; report it.

Writing style note: this doc avoids em-dashes per project convention.

---

## 1. Scope

In scope: line-rate classes, the 8b/10b line code and bit order, the frame, banks and CRCs, frame length at every
sample rate (44.1 kHz padding), the 32-byte superframe with the frame counter and apply-at-frame, hop counting for
repeaters and fan-outs, the PRBS-7 test payload, and the receiver's alignment, lock, error and loss rules. Also in scope:
the time-base contract that the line and every other alink time-base transport meet (section 11) and the carriage of
the superframe in the S/PDIF user bits (section 12).

Not in scope: PIO programs and sampling (LK2), the soft VCXO servo, phase lock and the anchor procedure (SYS2 3.1,
LK6), what a unit does with an apply op (`extension_channel_spec.md` 2.8.7), muting and ramps on loss (LK5/LK6),
and hardware (SYS2 section 6).

## 2. Conventions

- **MUST**, **MUST NOT**, **SHOULD**, **MAY** as in RFC 2119.
- **Frame:** one sample period of the system. **F:** the frame counter, an unsigned 64-bit integer. **Symbol:** one
  8b/10b code group (10 line bits). **C:** channels in the frame (1..64). **Class:** the line rate (section 3).
- **Sender:** the unit that originates the stream (master clock). **Repeater / fan-out:** a unit that re-emits a
  received stream (section 7.2). **Receiver:** a unit that decodes a stream.
- **Code group bits** are named `abcdei fghj` as in IEEE 802.3 clause 36. A symbol is written as a 10-bit number
  with `a` as its most significant bit, so K28.5 with RD- is `0x0FA` = `001111 1010`. **`a` is sent first.**
- **Byte to code group:** byte bits `HGF EDCBA` (H = bit 7). `x = EDCBA` selects the 5b/6b sub-block, `y = HGF` the
  3b/4b sub-block; the byte is written `Dx.y` (data) or `Kx.y` (control).
- **Byte order:** every multi-byte field (superframe fields and every CRC on the line) is little-endian, low byte
  first. Audio samples are the exception: big-endian (MSB first), section 5.4.
- **Words:** where line bits are stored in 32-bit words (PIO FIFOs, DMA rings, test files), they are packed MSB
  first: the first bit on the line is bit 31 of the first word. This is what a PIO `out pins, 1` with autopull 32 and
  shift-left sends, and what `in pins, 1` with autopush 32 and shift-left stores. In byte strings (test vectors) the
  first bit is bit 7 of the first byte. Symbols follow each other back to back across word boundaries.
- **Polarity:** a 1 bit is a high level at the PIO pin (light on, for an optical transmitter). Parts that invert
  (AFBR-1629Z/2529Z, an SFP's TD-) are corrected with the GPIO's output or input override, not in the code.
- **Word alignment:** a frame is `10 S_F` bits (5.6). At 48 and 96 kHz that is a whole number of 32-bit words for
  every class, so a locked comma sits at a fixed bit offset inside its word. At 44.1 kHz it is not, so encoders and
  decoders keep a bit cursor that runs across frames and across a ring's wrap (the reference does: `alink_bitw_t`
  appends at any bit position, and `alink_rx_push()` takes bits in any chunks).

## 3. Line-rate classes

A class is a fixed line rate, independent of the sample rate. All are integer PIO dividers of the DSPi's 307.2 MHz
system clock and integer multiples of 48 kHz x 10.

| Class | Code | Line rate | PIO cycles per bit at 307.2 MHz | Bit period | Typical medium |
|---|---|---|---|---|---|
| L1 | 1 | 15.36 MBd | 20 | 65.1 ns | TOSLINK-class 25 Mb/s parts (PLR237 / PLT237) |
| L2 | 2 | 30.72 MBd | 10 | 32.6 ns | Broadcom 50 MBd Versatile Alink, SFP |
| L3 | 3 | 61.44 MBd | 5 | 16.3 ns | SFP 100BASE-FX (experimental) |

- Both ends run the same nominal rate; the receiver's sampler re-phases on every transition and its soft VCXO
  removes the rate error (SYS2 2.6, 2.8). The protocol has no rate negotiation: a receiver either knows the class or
  detects it by trying each sampler (SYS2 2.6) and confirms it with superframe byte 5.
- Codes 0 and 4..255 are reserved. A 38.4 MBd class (SYS2 "L2+") is not part of version 1.
- The line MUST NOT be left at a constant level while an alink is enabled: a sender sends frames from power-up, with
  zero samples until it has audio (SYS2 3.3).

## 4. Line code: 8b/10b

### 4.1 Code groups

The code is IEEE 802.3 clause 36 8b/10b (Widmer/Franaszek; US patent 4,486,739 expired 2002). The tables are
repeated here so this document stands alone. Columns are "current running disparity is RD-" and "is RD+".

5b/6b (`abcdei`):

| x | RD- | RD+ | x | RD- | RD+ | x | RD- | RD+ | x | RD- | RD+ |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 100111 | 011000 | 8 | 111001 | 000110 | 16 | 011011 | 100100 | 24 | 110011 | 001100 |
| 1 | 011101 | 100010 | 9 | 100101 | 100101 | 17 | 100011 | 100011 | 25 | 100110 | 100110 |
| 2 | 101101 | 010010 | 10 | 010101 | 010101 | 18 | 010011 | 010011 | 26 | 010110 | 010110 |
| 3 | 110001 | 110001 | 11 | 110100 | 110100 | 19 | 110010 | 110010 | 27 | 110110 | 001001 |
| 4 | 110101 | 001010 | 12 | 001101 | 001101 | 20 | 001011 | 001011 | 28 | 001110 | 001110 |
| 5 | 101001 | 101001 | 13 | 101100 | 101100 | 21 | 101010 | 101010 | 29 | 101110 | 010001 |
| 6 | 011001 | 011001 | 14 | 011100 | 011100 | 22 | 011010 | 011010 | 30 | 011110 | 100001 |
| 7 | 111000 | 000111 | 15 | 010111 | 101000 | 23 | 111010 | 000101 | 31 | 101011 | 010100 |
| K28 | 001111 | 110000 | | | | | | | | | |

3b/4b (`fghj`), selected by the running disparity *after* the 6-bit sub-block:

| y | D RD- | D RD+ | K RD- | K RD+ |
|---|---|---|---|---|
| 0 | 1011 | 0100 | 1011 | 0100 |
| 1 | 1001 | 1001 | 0110 | 1001 |
| 2 | 0101 | 0101 | 1010 | 0101 |
| 3 | 1100 | 0011 | 1100 | 0011 |
| 4 | 1101 | 0010 | 1101 | 0010 |
| 5 | 1010 | 1010 | 0101 | 1010 |
| 6 | 0110 | 0110 | 1001 | 0110 |
| 7 | 1110 (P7) | 0001 (P7) | 0111 | 1000 |

- **Alternate D.x.7 (A7):** `0111` (RD-) / `1000` (RD+) replaces P7 when RD- and x is 17, 18 or 20, or when RD+ and
  x is 11, 13 or 14. This keeps runs at 5 or fewer bits.
- **Control codes** that exist: K28.0-K28.7 and K23.7, K27.7, K29.7, K30.7 (the 6-bit sub-block of x, then the K
  column for y = 7). Version 1 sends only K28.5; every other K code is reserved, and a receiver treats it as a code
  error.

### 4.2 Running disparity

- After a sub-block with more ones than zeros RD is +; with more zeros, RD is -. A balanced sub-block leaves RD
  unchanged, except `000111` and `0011` (RD becomes +) and `111000` and `1100` (RD becomes -).
- **A sender starts its stream with RD-** and carries RD across symbols and frames without reset.
- A symbol is a valid code group for an RD when encoding some byte (D or K) at that RD produces it. 464 of the 1024
  ten-bit values are valid for at least one RD. The reference decoder returns the set of RDs a symbol is valid for.
- **Receivers do not check running disparity in version 1.** A symbol that is a valid code group for either RD
  decodes; the CRCs catch corrupted data. (Disparity errors are a cheap extra error indicator a later version may
  count.)

### 4.3 Special symbols

| Symbol | Byte | Code groups | Use |
|---|---|---|---|
| K28.5 | `0xBC` | RD- `001111 1010` (`0x0FA`), RD+ `110000 0101` (`0x305`) | comma: symbol 0 of every frame, and nowhere else |
| D21.5 | `0xB5` | `101010 1010` (`0x2AA`) in both columns | idle padding (section 5.6): neutral, a transition every bit |

The comma is singular: in a stream of valid code groups the 10-bit K28.5 patterns occur only where a K28.5 was
sent, never across symbol boundaries. Receivers find the frame and symbol alignment from it (section 8.2).

The comma is also inversion-symmetric: the complement of K28.5 RD- is K28.5 RD+. An inverted line therefore still
shows commas at the right spacing, but its heads never pass CRC-8 (the complement of a data code group is in
general another byte). A firmware receiver MAY report "inverted" as a diagnostic when commas keep arriving at frame
spacing, heads keep failing, and the complemented head symbols (`sym ^ 0x3FF`) pass; the reference receiver does
not, and a receiver MUST NOT decode an inverted line silently.

## 5. Frame

### 5.1 Layout

One frame per sample period. Symbol indices count from the comma (symbol 0).

| Symbols | Name | Content |
|---|---|---|
| 0 | comma | K28.5 |
| 1 | HDR | `F & 0xFF` |
| 2 | CTL | superframe byte `F mod 32` (section 6) |
| 3 | CRC8 | CRC-8 over HDR, CTL (section 5.5) |
| 4 + 26b .. 4 + 26b + 3k - 1 | bank b payload | channels 8b .. 8b + k - 1, 3 bytes each (section 5.4) |
| 4 + 26b + 3k, + 3k + 1 | bank b CRC | CRC-16 over the bank's 3k payload bytes, low byte first |
| layout end .. S_F - 1 | padding | D21.5, `S_F - layout` symbols (may be none) |

- **Banks.** `B = ceil(C / 8)`. Banks 0..B-2 hold 8 channels (26 symbols each); the last bank holds
  `k = C - 8 (B - 1)` channels (`3k + 2` symbols). Bank b always starts at symbol `4 + 26 b`, so a receiver decodes
  only the banks it needs.
- **Layout length** `layout(C) = 4 + sum over banks (3 k_b + 2)`. Examples: C = 8: 30; C = 16: 56; C = 18: 64;
  C = 38: 128.
- Every symbol except the comma is a data code group (Dx.y).

### 5.2 Head

HDR carries the low byte of F so that a receiver detects a slip of the frame count of up to 127 frames between
superframes. CTL carries one byte of the superframe. CRC8 protects both; a receiver uses neither HDR nor CTL when
CRC8 fails (section 8.4).

### 5.3 Banks and channel order

Channel n of the frame (the "alink slot" of `EXT_ALINK_SLOTS`) is in bank `n / 8` at position `n mod 8`. There is no
per-channel signalling: C comes from the superframe, and a sender that has fewer audio channels sends zeros in the
rest.

### 5.4 Samples

24-bit two's complement, most significant byte first (bits 23..16, 15..8, 7..0). A sender transmits bits 23..0 of
its sample; a receiver sign-extends to its pipeline's format.

### 5.5 CRCs

| CRC | Polynomial | Init | Reflect | Final XOR | Check ("123456789") | Used for |
|---|---|---|---|---|---|---|
| CRC-8 | `0x07` | `0x00` | no | none | `0xF4` | frame head: HDR, CTL |
| CRC-16/CCITT-FALSE | `0x1021` | `0xFFFF` | no | none | `0x29B1` | each bank; the superframe (bytes 0-29) |

The CRC-16 is the one `uart_control.h` uses. Both CRCs are computed over the decoded bytes, not the code groups.
The CRC-16 goes on the line low byte first, like every multi-byte field.

### 5.6 Frame length and 44.1 kHz padding

The class fixes the line rate and the rate code fixes Fs, so a frame lasts `R = baud / (10 Fs)` symbols on average.
Let `R = n / d` in lowest terms. **Frame F has**

```
S_F = floor(((F mod d) + 1) * n / d) - floor((F mod d) * n / d)
```

symbols, comma included. The sender pads every frame with D21.5 from `layout(C)` up to `S_F`.

| Class | 48 kHz | 96 kHz | 44.1 kHz (n / d) | 44.1 kHz frames |
|---|---|---|---|---|
| L1 | 32 | 16 | 5120 / 147 = 34.83 | 34 or 35 symbols |
| L2 | 64 | 32 | 10240 / 147 = 69.66 | 69 or 70 |
| L3 | 128 | 64 | 20480 / 147 = 139.32 | 139 or 140 |

- At 48 and 96 kHz `d = 1` and every frame has the same length. At 44.1 kHz the pattern repeats every 147 frames
  and the line carries exactly `147 n / d` symbols per 147 frames, so the frame rate is exactly 44.1 kHz. The L1
  pattern starts `34 35 35 35 35 34 35 35 35 35 35 34 ...` at `F mod 147 = 0`.
- The pattern is a function of F, so any receiver that knows F knows where every comma falls (section 8.5), and a
  repeater re-creates the same stream. The frame-to-frame jitter of one symbol (0.65 us at L1) is far below the phase
  servo's bandwidth (SYS2 3.1).
- The longest frame of a class is 35 / 70 / 140 symbols; a receiver that does not yet know the rate looks for the
  next comma within that many symbols (section 8.3).

### 5.7 Capacity

C MUST satisfy `layout(C) <= min S_F` at the class and rate in effect, and `1 <= C <= 64`:

| Class | 44.1 kHz | 48 kHz | 96 kHz |
|---|---|---|---|
| L1 | 8 | 8 | 3 |
| L2 | 19 | 18 | 8 |
| L3 | 41 | 38 | 18 |
| *USB-direct, RP2350 (for comparison)* | *2 (16/24-bit)* | *8 (16-bit); 4 (24-bit)* | *2 (16/24-bit)* |
| *USB-direct, RP2040 (for comparison)* | *2 (16/24-bit)* | *2 (16/24-bit)* | *2 (16/24-bit)* |

**This table is normative and is the only capacity table** (coordinator decision, 2026-11-13), except its rows
marked *for comparison* (below). Code takes it from `alink_max_channels(class, rate)` in C or `alink_codec.CAPACITY[(class, rate)]` in Python, never from a copy:
`EXT_ALINK_TX.channels` (`extension_channel_spec.md` 2.8.8), the sender (LK4), the fake device and the clients
accept exactly C = 1 .. this value. The SYS2 figures (16 on L3 at 96 kHz; the 48 kHz values at 44.1 kHz) are
superseded. Defaults are whole banks: 8 on L1, 16 on L2, 32 on L3 at 48 kHz.

The rows marked *for comparison* are not alink classes and not part of this normative table: they are the channels a
DSPi unit takes in one USB full-speed stream as an alink USB-direct unit (`extension_channel_spec.md` 2.9; the formats
in `usb_descriptors.c` of the unit build: stereo 16 or 24 bit at every rate, 4 channels 16 or 24 bit and 6/8 channels
16 bit at 48 kHz only). A `DSPI_ROLE_SENDER` build, which also has the USB-direct block, adds alt 7 (6 channels 24-bit,
48 kHz) and alt 8 (4 channels 16-bit, 96 kHz); the rows do not count those. `alink_max_channels()` and `CAPACITY` do
not know these rows. At 44.1 kHz USB-direct is rate lock only (no phase lock, 2.9.1). How many such units one computer
carries (computed, not measured; WP AUSB3, guide `usb-direct-home-theater.md` in the console repository):

| USB-direct units, for comparison | Units | Why |
|---|---|---|
| per single-TT hub (or a hub run at full speed: an isolator or a full-speed extender in front of it) | 1 | every unit reserves its endpoint's 788 B (RP2350; 582 B RP2040) per 1 ms frame whatever it streams: about 624 us (463 us) of the 900 us (90 %, USB 2.0 5.6.4) a full-speed frame allows for periodic transfers |
| per multi-TT hub | 1 per downstream port | one transaction translator, so one full-speed frame, per port |
| per USB controller (one high-speed bus, one SOF) | about 30 | about 25 us of split transactions per unit per frame of the 800 us (80 %) high-speed periodic budget, i.e. about 32; controllers check bandwidth themselves and may refuse sooner |

The figures, host delay left out: full-speed isochronous transaction time (USB 2.0 5.11.3)
`9107 + 83.54 x floor(3.167 + 1.1667 x 8 x bytes)` ns, 788 B = 624 us, 582 B = 463 us; through a transaction
translator a 788 B packet is 5 start-splits of at most 188 B, each `55 x 8 x 2.083 + 2.083 x floor(3.167 + 1.1667 x 8 x 188)`
ns = 4.6 us, about 23 us, plus about 2 us for the feedback endpoint's splits.

Units that play aligned must be on one USB controller: another controller counts other frames on another clock. On an
EHCI (USB 2.0 only) computer a full-speed unit on a root port is run by a companion UHCI/OHCI controller, another clock
than the units behind a hub; there every unit goes behind the hub.

### 5.8 PRBS-7 test payload

While superframe flag b4 is set, the payload bytes of every bank of every frame are the PRBS-7 sequence instead of
audio: generator `x^7 + x^6 + 1`, state reset to all ones (`0x7F`) at the start of each frame, one output bit per
step (`b = s6 XOR s5; s = ((s << 1) | b) & 0x7F`), bits packed MSB first into the frame's payload bytes in order
(bank 0 first; CRC bytes are not part of the sequence and stay the CRC of what is sent). The first bytes are
`02 0c 28 f2 2c ea ...`. The bench tools compare decoded payload against this sequence for BER tests (SYS2 8a). The
flag is per superframe, so it switches at a superframe boundary.

## 6. Superframe

### 6.1 Layout

The CTL bytes of 32 consecutive frames F with `F mod 32 = 0 .. 31` form one superframe, sent repeatedly. It is the
system's time base and broadcast bus.

| Byte | Field | Content |
|---|---|---|
| 0 | magic | `0xA5` |
| 1 | version | 1 |
| 2 | flags | b0 system mute; b1 rate change pending; b2 apply pending; b3 sender has a live USB stream; b4 test pattern (PRBS-7 payload, 5.8); b5 sender is slaved (fan-out); b6-b7 reserved, 0 |
| 3 | rate | rate code of this superframe's frames: 0 = 44.1 kHz, 1 = 48 kHz, 2 = 96 kHz |
| 4 | channels | C of this superframe's frames, 1..64 within 5.7 |
| 5 | class | class of this line, 1..3 |
| 6 | hops | 0 at the master sender; +1 per repeater or fan-out (7.2) |
| 7 | sender | low byte of the sender's serial number |
| 8-15 | frame | F of this superframe's first frame (a multiple of 32), u64 |
| 16-23 | apply_frame | apply target frame, u64; 0 = none |
| 24 | apply_op | 6.4; 0 = none |
| 25 | apply_arg | |
| 26-27 | apply_value | i16 |
| 28-29 | reserved | sent as 0, ignored by receivers |
| 30-31 | crc | CRC-16/CCITT-FALSE over bytes 0-29, low byte first |

Example (vector `sf/typical`): F = 2^40, 48 kHz, 8 channels, L1, hops 0, sender `0x42`, live USB stream:
`a5 01 08 01 08 01 00 42 00 00 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 d7 47`.

### 6.2 Fields

- **flags b1 and b2** are derived: b2 is set exactly when `apply_op` is not 0, b1 exactly when it is RATE. The
  sender's other flags come from its state (`EXT_ALINK_TX`).
- **rate, channels** describe the 32 frames of this superframe. They change only at a superframe boundary
  announced by a RATE apply (6.5).
- **class** is the class of the line it is sent on; a receiver rejects a superframe whose class differs from the
  class its sampler runs (8.6).
- **sender** lets a hub tell two systems apart; it has no protocol meaning.

### 6.3 Validity

A superframe is valid when, in this order: its CRC matches; magic is `0xA5`; version is 1; `frame mod 32 = 0`; class
is 1..3, rate is 0..2 and C is 1..the capacity of 5.7 for that class and rate; `apply_op` is 0..10 (0..8 before
WP FWTB1; 6.4) and `apply_op = 0` exactly when `apply_frame = 0`. The reference `alink_sf_unpack()` returns the first failing check
(`ALINK_SF_ERR_CRC` 1, `MAGIC` 2, `VERSION` 3, `FRAME` 4, `LAYOUT` 5, `APPLY` 6). Reserved bytes 28-29 are not
checked. A receiver adds the checks of 8.6.

### 6.4 Apply at frame N

Bytes 16-27 carry one pending operation for every unit on the alink: "at frame N, do op". A superframe carries the
pending apply exactly when it completes before the target, that is when `frame + 32 <= N` (the superframe's last
frame is before N). The sender MUST therefore announce at least one full superframe ahead: N at least
`next superframe start + 32`. It SHOULD announce far more (SYS2 3.3 uses 2400 frames, 50 ms, for RATE) so that a
corrupted superframe or two cost nothing. A sender carries one apply at a time.

| Op | Name | arg | value |
|---|---|---|---|
| 0 | none | 0 | 0 |
| 1 | MUTE | 0 | 0 |
| 2 | UNMUTE | 0 | 0 |
| 3 | PRESET_LOAD | slot | 0 |
| 4 | MASTER_VOLUME | 0 | dB x 100 |
| 5 | OUT_DELAY | output | samples |
| 6 | RATE | rate code | new C, 0 = unchanged (6.5) |
| 7 | CLICK | output | 0 |
| 8 | SYNC_PULSE | 0 | 0 |
| 9 | SIGGEN_START | 0 | 0 |
| 10 | SIGGEN_STOP | 0 fade, 1 now | 0 |

What a unit does with ops 1-5 and 7-10, the argument ranges and the late and refused results are specified in
`extension_channel_spec.md` 2.8.7 (ops 9 and 10: 2.8.7.1 and `test_signals_spec.md` section 15); the superframe only
transports them. Op 6 is the protocol's own (6.5).

**Ops 9 and 10 (WP FWTB1, PLAN-AT OD-ATe).** The protocol version stays 1: nothing using the alink is released, and
a version-1 receiver without the ops rejects a superframe carrying op 9 or 10 (code 6 `APPLY`), counts it in
`superframe_errors` and keeps playing on its counter (8.4), so a mixed system degrades to "the old units miss that
op". A receiver or sender build that has the ops says so with `EXT_ALINK_INFO.features` bit 6 `SIGGEN_SYNC`
(`extension_channel_spec.md` 2.8.1). A host sends op 9 or 10 in the superframe only when every unit of the alink
lists that bit; this plan's hub broadcasts only op 10 (the abort without control latency), never op 9 (each unit's
START is written per unit, through `EXT_APPLY_AT_FRAME`).

### 6.5 Layout changes (RATE)

The frame layout (rate and C) changes only at a superframe boundary N (`N mod 32 = 0`) announced with apply op RATE
(arg = new rate code, value = new C or 0 for unchanged). From frame N on, the sender uses the new layout and the
superframes describe it; receivers that saw the announcement switch at N too (8.5), so no frame is lost. Both ends
switch at the same point: at the end of frame N - 1. The reference encoder's `tx->rate` and `tx->channels` always
describe the next frame it will encode, so a sender reads them before it fills that frame's samples. A RATE op
whose new layout does not fit (5.7) MUST NOT be sent, and a receiver ignores it. A receiver that missed the
announcement loses lock at N and re-locks (8.8).

RATE is the protocol's own op: it travels only in the superframe and acts on every unit of the alink at once.
**`EXT_APPLY_AT_FRAME` (per unit, 2.8.7) refuses op 6** (coordinator decision, 2026-11-13): a receiver follows the
layout its stream carries and never re-plans for a rate the stream does not have. A host changes the rate on the
sender, through `EXT_ALINK_TX` or `EXT_ALINK_CTRL_MSG` (2.8.8, 2.8.9), where RATE takes value 0; the sender puts the
new C in `value` itself when its channel count changes. `EXT_ALINK_CTRL_MSG` follows the carriage rules of 6.4 and
this section (target at least one complete superframe ahead, RATE targets a multiple of 32, a layout that does not
fit refused), as `alink_tx_schedule()` does.

### 6.6 The frame counter

- F counts frames since an arbitrary origin chosen by the sender (power-up, typically 0) and increments by one per
  frame at every rate. It never wraps in practice (2^64 frames are 13 million years at 44.1 kHz); arithmetic is
  modulo 2^64.
- The sender MAY start a stream at any F, also in the middle of a superframe; the partial superframe it starts in
  is a valid one (bytes for frames before the start are simply never sent).
- A receiver's full F comes from a valid superframe and then advances by one per frame (8.4). HDR checks the low byte
  on every frame.

## 7. Senders, repeaters and fan-outs

### 7.1 Sender

A sender MUST: send continuously from power-up (frames of zero samples until audio is present, b3 clear); start
with RD- and carry RD; send every frame exactly `S_F` symbols long (5.6); keep C within 5.7; refresh the superframe
only at superframe boundaries; and change the layout only through RATE (6.5). All alinks of a star send the same bit
stream (SYS2 4.2).

### 7.2 Repeaters and fan-outs (the hop byte)

A repeater (a receiver with a transmit pin) or a fan-out (a sender slaved to an alink input) re-emits the stream it
receives, delayed by a whole number of frames, its **hop delay** `d_hop`, which it reports
(`extension_channel_spec.md` 2.8.2, `EXT_ALINK_STATUS.hop_delay_frames`):

- The hop delay is at least 1 frame and is the re-emitter's own: a store-and-forward re-emitter needs a whole input
  frame before its copy can start, plus its FIFOs (LK4's fan-out: 2-3 frames). It is constant while the re-emitter
  stays locked; after a re-alignment it may differ, and the re-emitter reports the new value.
- The re-emitted frame F carries the same F, the same payload and the same layout; HDR is unchanged.
- Its superframe is the received one with `hops = min(hops + 1, 255)` and `class` set to the class of the outgoing
  line, CRC recomputed (`alink_sf_repeat()`). 255 means "255 or more".
- It re-encodes (its own RD and padding), so its output is again a conforming stream. A repeater that has no valid
  superframe yet sends only D21.5 idles (no commas, so downstream units see no stream rather than a wrong one,
  and the line still never sits at a constant level).
- Receivers use hops to compute `D_local = D - hops` (SYS2 3.1), which counts one frame per hop; the hub writes it
  (`EXT_ALINK_PHASE.d_local`). The hub compensates the rest of each re-emitter's delay through
  `EXT_ALINK_PHASE.hops_offset`: a unit behind re-emitters with hop delays `d_1 .. d_h` gets
  `hops_offset = -sum(d_k - 1)`, so that its effective delay `d_local + hops_offset` is `D - sum(d_k)` and it plays
  frame F at the same moment as a unit fed directly. The hub reads each re-emitter's `hop_delay_frames` and knows from
  its own configuration which re-emitters feed which unit (the stream does not say: hops counts re-emitters, not
  frames). It re-reads them after a re-emitter lost lock and adjusts the units behind it.

## 8. Reference receiver

This section is the bit-exact behaviour of `alink_decoder.c` / `Decoder`: alignment, lock, the counters and what is
delivered for every frame. A firmware receiver MUST behave the same on the same bits. It MAY decode fewer banks
(the bank mask) and MAY be faster; what it then does with the frames (servo, mute ramps, holdover) is LK5/LK6.

### 8.1 Inputs and states

Input: the line bits in order, numbered from 0, plus the class of the sampler and a bank mask. Positions below are
bit numbers; "the comma ending at p" means bits `p - 10 .. p - 1` are a K28.5 code group (either RD).

| State | Knows | Leaves on |
|---|---|---|
| SEARCH | nothing | a comma with a good head: ALIGNED |
| ALIGNED | symbol alignment, HDR | a valid superframe: LOCKED; loss: SEARCH |
| LOCKED | F, rate, C, the position of every comma | loss: SEARCH |

Every step below is taken only once every bit it reads is present, and reads nothing else, so the output does not
depend on how the input is chunked. The reference tests feed the same streams in chunks of 1, 7, 32, 320 and 4096
bits and require identical results.

### 8.2 SEARCH

For q = 10, 11, ... (starting where the previous state left off): if the comma ends at q and the head after it is
good (8.4, without counting errors), enter ALIGNED with the frame at p = q. A head needs bits up to q + 30.

### 8.3 ALIGNED

For each frame (comma ending at p):

1. **Head.** If bad: `crc8_errors` + 1, the superframe assembly breaks (8.6); after 3 consecutive bad heads, loss
   (8.8). If good: the CTL byte goes to the assembly at index `HDR mod 32`. When index 31 completes a full assembly,
   the superframe is checked (8.6, also requiring `(frame + 31) & 0xFF = HDR`); if valid, enter LOCKED with
   F = frame + 31 for this frame and the superframe's rate and C.
2. **Next comma.** Look at symbol boundaries `p + 10 i` for i = 4 .. the class's longest frame (35 / 70 / 140): the
   first comma found starts the next frame. If none, loss.

No audio is delivered in ALIGNED.

### 8.4 LOCKED: one frame

For frame F with comma ending at p, `S = S_F` from the current rate (needs bits up to `p + 10 (S - 1)`):

1. **Head.** Good when symbols 1-3 are data code groups and CRC8 matches. A bad head: `crc8_errors` + 1, flag
   HEAD_BAD. A good head with `HDR != F & 0xFF`: `counter_errors` + 1, flag COUNTER, treated as bad from here on.
2. **Banks.** For each bank in the mask: good when all `3k + 2` symbols are data code groups and the CRC-16 matches;
   then its samples are delivered. A failed bank counts `crc16_errors` + 1 and delivers, on the first consecutive
   failure of that bank, the samples of the last delivered frame (HOLD), and from the second on, zeros (ZERO).
   Banks outside the mask are not decoded (SKIPPED) and deliver zeros.
3. **Padding.** Each symbol from `layout(C)` to `S - 1` that is not D21.5 counts `idle_errors` + 1 (nothing else).
4. **Deliver** the frame: F, flags, bank status, C samples. `frames` + 1.
5. **Superframe.** A good head adds CTL at index `F mod 32`; a bad one breaks the assembly. At index 31 the
   superframe is checked (8.6). If valid, its rate, C and apply are adopted from the next frame on; if its frame
   field disagrees with F (`frame + 31 != F`), F is corrected and `counter_errors` + 1.
6. **Failure count.** The frame failed when its comma was missing (8.5), its head was bad, or HDR mismatched. 3
   consecutive failed frames: loss (8.8). A good frame resets the count.

Single-bit damage therefore costs at most one bank of one frame (held), or one head (no control byte, no
superframe that round); a lost comma alone costs nothing but a counter.

### 8.5 LOCKED: the next comma

With `e = p + 10 S` (needs bits up to e + 2):

- F advances by one. If a RATE apply from the last valid superframe targets the new F, the layout switches now
  (when it fits, 6.5).
- **Comma at e:** the next frame starts at e.
- **Comma at e - 1, e + 1, e - 2 or e + 2** (checked in that order): a bit slip; the next frame starts there,
  `bit_slips` + 1, flag SLIP with the offset. (Because the comma is singular, at most one of these positions can hold
  one; the order is fixed for completeness.)
- **No comma within +-2 bits:** the flywheel. The next frame starts at e anyway, `comma_errors` + 1, flag
  COMMA_MISSING. A slip of 3 or more bits therefore looks like missing commas and ends in loss and re-search.
- A comma anywhere else (a stray pattern made by bit errors in the payload) is ignored.

### 8.6 Superframe acceptance

The assembly collects CTL bytes of consecutive good heads from index 0. A bad head, a non-consecutive HDR, or an
index that does not continue the run breaks it; index 0 always restarts it. At index 31 a complete assembly is
checked: 6.3 validity, then `class` equal to the receiver's class (else rejection 17). In ALIGNED also the HDR
consistency of 8.3 (else rejection 18). Rejections count `superframe_errors` + 1:

- in both states, a complete assembly that fails a check (codes 1-6, 17, 18);
- in LOCKED only, an incomplete one at index 31 (code 16), since a locked receiver knows a byte was lost.

### 8.7 Delivered audio

Every LOCKED frame delivers C samples. Per bank: OK (decoded), HOLD (last frame's samples), ZERO, or SKIPPED (zeros).
The hold buffer starts at zero on lock and is cleared on loss.

### 8.8 Loss

From ALIGNED or LOCKED, on 3 consecutive bad heads (ALIGNED), 3 consecutive failed frames (LOCKED) or no comma in
reach (ALIGNED): emit LOSS, return to SEARCH at q = p + 1 (p = the current frame's comma end), clear the assembly,
the layout, any pending RATE and the hold buffers. `loss_count` + 1 only when leaving LOCKED. Lock comes back after
the next complete valid superframe: 32 to 63 frames after the line is good again (0.7-1.3 ms at 48 kHz).

### 8.9 Counters

| Counter | Counts | `EXT_ALINK_STATUS` field (2.8.2) |
|---|---|---|
| `lock_count` | entries into LOCKED | `lock_count` |
| `loss_count` | losses from LOCKED | `loss_count` |
| `crc8_errors` | bad heads (non-data symbol or CRC-8), ALIGNED and LOCKED | `crc8_errors` |
| `crc16_errors` | failed banks (non-data symbol or CRC-16) | `crc16_errors` |
| `bit_slips` | commas found 1-2 bits off | `bit_slips` |
| `superframe_errors` | rejected superframes (8.6) | `superframe_errors` |
| `comma_errors` | flywheel frames | `EXT_ALINK_DIAG` offset 40 (u32) |
| `counter_errors` | HDR mismatches and F corrections | `EXT_ALINK_DIAG` offset 44 (u32) |
| `code_errors` | non-data symbols in heads and decoded banks | `EXT_ALINK_DIAG` offset 48 (u32) |
| `idle_errors` | non-idle padding symbols | `EXT_ALINK_DIAG` offset 52 (u32) |
| `frames` | frames delivered | |

`EXT_ALINK_DIAG` (`0x800A`, 64 bytes, `extension_channel_spec.md` 2.8.11) carries the four bench counters in its
second half, which replaces the sampling-phase histogram of SYS2 (the PIO emits only bits, so there is no
histogram). Bytes 32-63, as adopted from review lk:

| Offset | Size | Field |
|---|---|---|
| 32 | i8 | `trim_lo`: lowest clean trim step of the last scan (-9..+9) |
| 33 | i8 | `trim_hi`: highest clean trim step |
| 34 | i8 | `trim_used`: trim step in use |
| 35 | u8 | `scan_state`: 0 none, 1 running, 2 done, 3 nothing clean |
| 36 | u32 | `clean_mask`: bit i set = trim step i - 9 decoded clean |
| 40 | u32 | `comma_errors` |
| 44 | u32 | `counter_errors` |
| 48 | u32 | `code_errors` |
| 52 | u32 | `idle_errors` |
| 56 | 8 | reserved, 0 |

The trim scan is LK2's (`alink_rx_N` delay trim, 3.26 ns per step); `EXT_ALINK_STATS_RESET` clears the four counters
with the `EXT_ALINK_STATUS` ones. 2.8.11 is the normative byte layout; this table must match it.

The receiver's state maps to `EXT_ALINK_STATUS.state` as SEARCH and ALIGNED = 1 (searching), LOCKED = 2; holdover
(3) is a firmware state on top of SEARCH.

## 9. Reference implementation and test vectors

### 9.1 Files

| File | What |
|---|---|
| `firmware/DSPi/alink/alink_codec.h`, `.c` | 8b/10b, CRCs, layout and `S_F`, superframe pack/unpack/repeat, PRBS-7, bit packing, the sender's frame encoder `alink_tx_*` |
| `firmware/DSPi/alink/alink_decoder.h`, `.c` | the receiver of section 8, `alink_rx_*`, with an event callback |
| `tools/alink/alink_codec.py` | the Python twin of both, written from this document's tables; `CAPACITY` (5.7); the test helpers of 9.2 |
| `tools/alink/gen_vectors.py` | writes `firmware/tests/alink_vectors.json` |
| `tools/alink/tests/test_alink_codec.py` | Python tests: anchors, stream properties, receiver behaviour, the vector file |
| `firmware/tests/test_alink_codec.c` | C tests: the same anchors and properties, every vector, sample round trips |

The C code is portable C11 with no SDK, no heap, no floating point and no writable static data: every table is
`const` (flash), and the receiver's state (under 1 KB, `alink_rx_t`) and the encoder's (under 100 bytes) live in the caller's
memory. No firmware target compiles it yet; LK3-LK5 add it to the build (and SHOULD put the hot parts of the
receiver in RAM and replace the reference symbol decoder with a table if the cycle budget of SYS2 5.1 asks for it).

### 9.2 Vector file

`firmware/tests/alink_vectors.json` holds one flat JSON object per line (strings and integers only, so the C test
scans it without a JSON library). `type` selects the kind:

| type | Fields | Check |
|---|---|---|
| `crc8`, `crc16` | `data` hex, `crc` | CRC of data |
| `code` | `k`, `byte`, `rd`, `sym`, `rd_out` | every D (256) and K (12) code group at both RDs |
| `dectab` | `table`: 1024 x 4 hex digits | decode of every 10-bit value: `rd_mask << 9 \| k << 8 \| byte`, 0 if invalid |
| `layout` | `cls`, `rate`, `min`, `max`, `max_channels`, `lengths` (S_F for F = 0..146, 2 hex digits each), `sum`, `at_big` (S_F at F = 2^63 + 12345) | 5.6, 5.7 |
| `layout_c` | `channels`, `symbols` | `layout(C)` for C = 1..64 |
| `prbs7` | `bytes` | the first 48 PRBS-7 bytes |
| `sf` | `bytes`, `result`, `pack`; with `pack` 1 also every field | unpack result; pack of the fields gives `bytes` |
| `sf_repeat` | `bytes_in`, `cls`, `bytes_out` | 7.2 |
| `frame` | `cls`, `rate`, `channels`, `frame`, `seed`, `flags`, `symbols` (3 hex digits each), `rd_out` | one frame from a fresh encoder (RD-) |
| `stream` | see below | encoder output and receiver log |

A `stream` vector has `cls`, `bank_mask`, `bits` (length of the clean stream), `stream` (the clean stream, packed MSB
first, hex), `corrupt` (damage to apply), `log` (the receiver's output on the damaged stream) and `encode`. With
`encode` 1 it also has the encoder parameters `rate`, `channels`, `first` (F of the first frame), `frames`, `seed`,
`sender`, `flags`, `hops`, and an optional apply scheduled before frame index `sched_index` (-1 none) with
`sched_at`, `sched_op`, `sched_arg`, `sched_value`; both implementations re-encode it and compare. With `encode` 0
the stream was crafted (for example a superframe with a bad CRC) and has a `note`.

- **Test samples:** channel `ch` of frame F is `test_sample(F, ch, seed)`: with 32-bit unsigned wrap-around,
  `h = (F mod 2^32) * 0x9E3779B1; h ^= (ch + 1) * 0x85EBCA77; h ^= seed; h ^= h >> 15; h *= 0x2C1B3C6D;
  h ^= h >> 12; h *= 0x297A2D39; h ^= h >> 15`, then bits 23..0 of h sign-extended.
- **Damage (`corrupt`):** `;`-separated ops on the clean stream: `flip:P` inverts bit P, `del:P` deletes it,
  `ins:P:B` inserts bit B before it, `fill:P:N:B` sets N bits from P to B (stopping at the end of the stream; it
  never lengthens it). Positions refer to the clean stream; ops
  apply from the highest position down, equal positions in list order.
- **Log:** the receiver's events, space-separated, then the counters:
  - `A<p>` entered ALIGNED with the comma ending at bit p; `L<F>` entered LOCKED at frame F;
  - `F<F>:<flags>:<banks>:<hash>` a delivered frame. `flags` is `-` or, in this order, `c` (comma missing),
    `s<+d|-d>` (slip by d bits), `h` (bad head), `n` (HDR mismatch); `banks` has one character per bank:
    `o` OK, `h` HOLD, `z` ZERO, `.` SKIPPED; `hash` is FNV-1a 32 (offset `0x811C9DC5`, prime `0x01000193`) over
    the C delivered samples as 32-bit little-endian integers, 8 lowercase hex digits;
  - `S` a superframe accepted while LOCKED; `E<code>` a superframe rejected (6.3, 8.6); `X` loss;
  - finally `#lock,loss,crc8,crc16,slips,superframe,comma,counter,code,idle,frames`.

The streams cover every class at 44.1, 48 and 96 kHz, a start at F = 0, F above 2^32 and 2^40, 1, 2, 3, 8, 16, 18,
38 and 41 channels, PRBS-7, a bank mask, RATE changes 48 to 96 kHz (with C 16 to 8), 48 to 44.1 kHz, and two that
grow C (8 to 16 on L2; 96 kHz 3 channels to 44.1 kHz 8 on L1), an apply, and damage: a bit error in a bank, hold then
zero, a bad head, a lost comma, slips of -2, -1, +1 and +2 bits, a 3-bit slip (loss), three lost commas (loss), a
2000-bit dropout, a stray comma inside the payload, a broken superframe byte, garbage before the stream, a fill past
the end, damage at 44.1 kHz (bank, lost comma, -1 and +2 slips, three lost commas), and crafted streams: a superframe
with a bad CRC, a wrong class byte, a wrong HDR with a good CRC-8, and a superframe that claims F + 256.

### 9.3 Running the tests

```sh
# Python twin and vector file (podman, python:3.12-slim pinned by digest, no network)
tools/alink/run.sh -m unittest discover -s tests -v
tools/alink/run.sh gen_vectors.py > firmware/tests/alink_vectors.json   # after a deliberate change

# C (any C11 compiler; run.sh builds every firmware/tests/test_*.c)
podman run --rm --network none -v "$PWD/firmware:/fw:ro" <gcc image> bash /fw/tests/run.sh
```

A change to the protocol changes the vectors: regenerate them, check the diff of the logs, and make both
implementations pass.

## 10. Decisions relative to the SYS2 design

SYS2 section 2 is the design; these points make it precise or change it:

1. **Byte order of the CRCs:** every CRC-16 on the line (banks and superframe) is low byte first, as the superframe's
   other fields and `uart_control.h`.
2. **Padding** is not "0 at 48/96 kHz": every class pads from `layout(C)` to `S_F` (L1 at 48 kHz with 8 channels:
   2 idle symbols).
3. **44.1 kHz frame lengths** follow the fixed formula of 5.6 keyed to F, so receivers and repeaters can predict
   them.
4. **Capacity** is computed (5.7) and is the only capacity table. L3 at 96 kHz fits 18 channels (SYS2 said 16,
   counting whole banks) and 44.1 kHz fits 8 / 19 / 41, more than 48 kHz.
5. **A missing comma is not an immediate loss** (SYS2 2.4 said it was; 3.3 treats a short loss as recoverable): the
   locked receiver flywheels, and loss is 3 consecutive failed frames (missing comma, bad head or HDR mismatch).
6. **Bit slips** of 1-2 bits re-align without loss; larger slips go through loss and re-search.
7. **Layout changes** (rate and C) happen only at superframe boundaries through apply op RATE, whose `value` carries
   the new C (0 = unchanged). RATE is never a per-unit op: `EXT_APPLY_AT_FRAME` refuses op 6.
8. **Apply carriage:** carried in every superframe that completes before the target; at least one is required.
9. **Running disparity** is generated correctly but not checked by receivers in version 1.
10. **PRBS-7** is defined per frame (reset at each frame), so a bench receiver checks every frame on its own.
11. **Repeaters** keep F and rewrite hops (saturating at 255) and class; they send nothing until they hold a valid
    superframe.

## 11. Time bases

*Normative (WP FWTB0, PLAN-AT section 1.1, owner decisions AT1 and AT4 of 2026-11-19). The control plane is in
`extension_channel_spec.md`: 2.8.6, 2.8.7, 2.9.4, 2.10 (the S/PDIF block) and 2.11 (`EXT_TIME_BASE`,
`EXT_SIGGEN_TIMELINE`, event `0xE4`). Implemented by WP FWTB1 (line and USB-direct, 2026-11-19: `firmware/DSPi/
timebase.{c,h}`, host test `firmware/tests/test_timebase.c`) and WP FWSP2 (S/PDIF, 2026-11-19: `firmware/DSPi/alink_spdif.{c,h}`, host test
`firmware/tests/test_alink_spdif.c`, simulation `tools/alink_spdif_sim`; host-tested, HW-untested).*

A **time base** is a sample counter T shared by a set of units, the **domain**: every unit of one domain numbers the
same audio sample with the same T (or with a T a single read maps, C1). A **follower** receives a time base, a
**source** emits one. Multi-unit work (alignment, scenes, apply-at-frame, measurement with each unit's own generator
started at a sample of T) needs nothing from the transport except this contract, so it never names a transport: the
hub and the console talk to "a unit with a time base". Nothing in this section is specific to the line.

### 11.1 The contract (C1-C6)

**A transport is an alink time-base transport when a unit on it offers C1 to C6.**

- **C1. A sample counter T**, 64 bits at the host interface, shared by every unit of one domain, with a `time_base`
  id (11.3). Units MAY count from different origins only where one read resolves the difference (USB-direct: origins
  differ by multiples of `2048 * spf`, `extension_channel_spec.md` 2.9.1). T increments by exactly one per sample
  period of the domain while the time base is `LOCKED` (11.2); arithmetic is modulo 2^64.
- **C2. Fixed play-out.** A follower plays the sample carried as T exactly D samples after it arrived; D is configured
  per unit, and the measured error is reported (`phase_error`, in samples). A source plays the sample carried as T at
  the instant it emits T (D = 0 by definition). So every `LOCKED` follower with the same D plays the same T at the
  same instant, and a source plays T exactly D samples before its followers do.
- **C3. Apply at T.** A queue of ops that act on exactly the sample carried as T as it enters the pipeline, on
  programme audio and on silence alike, each completing with a result event (`0xE2`) that carries T. An op whose T has
  passed is applied at once and reported `late`, except op 9 `SIGGEN_START`, which is never late: it is refused
  (`extension_channel_spec.md` 2.8.7.1).
- **C4. Lock states and events.** The contract's states (11.2), an event for every transition (`0xE4`), and a
  **disturbance counter** that every transition out of `LOCKED`, every holdover, resume, re-anchor, whole-sample
  correction and epoch change increments (one occurrence that is several of these at once, such as a holdover that
  leaves `LOCKED`, counts once). The counter is readable together with T and `output_frames` in one atomic
  read (`EXT_TIME_BASE`), so a host can prove that nothing happened in an interval from two reads, without relying on
  events (which are rate-limited and collapse). While an entry stays `LOCKED` and its counter does not change,
  `t - output_frames` is constant.
- **C5. A domain identity** a host can compare: `EXT_TIME_BASE.domain`, 32 bits, `time_base` in byte 3. Line: byte 0
  the sender id (superframe byte 7), bytes 1-2 zero. S/PDIF: bytes 0-1 the source's serial bytes 0-1 (12.2), byte 2
  the source's epoch. USB-direct: bytes 0-2 zero (the host's bus decides; a hub compares units by their host bus and
  by `t`). An id is short and can collide, so a host also checks that the units' `t` agree in one read round
  (PLAN-AT G1): equal domains are necessary, not sufficient.
- **C6. Discovery.** `EXT_LIST` names `EXT_TIME_BASE` (`0x8030`) on every build that has a time base, and
  `EXT_SIGGEN_TIMELINE` (`0x8031`) on every build that has ops 9 and 10; each transport's own block names its own ids
  (`0x8000`-`0x800A` line, `0x8010`-`0x8012` USB-direct, `0x8020`-`0x8022` S/PDIF).

### 11.2 States

`EXT_TIME_BASE` entry `state` and event `0xE4` `state`. Each transport maps its own state onto these:

| Contract state | Meaning | Line (`EXT_ALINK_STATUS.state` / `EXT_ALINK_PHASE.state`) | USB-direct (`EXT_ALINK_USB_STATUS.state`) | S/PDIF (`EXT_ALINK_SPDIF_STATUS.state`) |
|---|---|---|---|---|
| 0 `NONE` | no usable time base | 0; or `EXT_ALINK_PHASE.mode` 0 (fill servo) | 0, 1, 3 (rate lock only) | mode 0; or the input is not S/PDIF |
| 1 `ACQUIRING` | searching, pulling in, waiting for a superframe | 1 | 2 | the receiver locking, or waiting for a valid superframe |
| 2 `ANCHORING` | drain, prefill, start, verify (muted) | 2 / 1 | 4 | anchoring (follower); outputs restarting (source) |
| 3 `LOCKED` | T valid, play-out fixed, apply-at-T works | 2 / 2 | 5 | phase-locked (follower); outputs running in sync (source) |
| 4 `HOLDOVER` | T extrapolated, muted, the timeline kept | 3 | 5 with STATUS flags bit 4 `HOLDOVER` | holdover (12.5) |
| 5 `LOCK_ERROR` | configured to lock but cannot | 2 / 3 (re-anchor pending); or mode 1 with the delay outside `d_local_min..max` (`EXT_ALINK_PHASE` flags `D_BELOW_MIN`: it runs the fill servo) | 6; or STATUS flags bit 6 `LOCK_ERROR` | no valid superframe in mode 1, D out of range, a rate it cannot lock at (12.5) |

Only `LOCKED` carries a usable T: apply-at-T, the play-out guarantee of C2 and the invariant of C4 hold only there.
In every other state `t` is the transport's own report (extrapolated in `HOLDOVER`, 0 where the transport has no T)
and MUST NOT be scheduled on.

### 11.3 Time base ids

| Id | Time base | T |
|---|---|---|
| 0 | the line | the frame counter F (6.6) |
| 1 | USB-direct | the sample index S (`extension_channel_spec.md` 2.9.1) |
| 2 | USB-direct, bus-frame form | a bus frame plus a sample in it; only in `EXT_APPLY_AT_FRAME` writes (2.9.4), never listed in `EXT_TIME_BASE`, never used by op 9 |
| 3 | S/PDIF | the source's frame counter F, carried in the user bits (section 12) |
| 4 | reserved: ADAT | |
| 5 | reserved: I2S / TDM | |
| 6 | reserved: PTP / AES67 media clock | |
| `0xFF` | none | |

Ids are never reused for another meaning.

### 11.4 The transports against the contract

| Transport | T | How T reaches a follower | Broadcast op path | C1-C6 |
|---|---|---|---|---|
| line L1-L3, fan-out, repeater | F | superframe and head byte; re-emitted with `hops` (7.2) | the superframe (sender `EXT_ALINK_CTRL_MSG`) | C1-C3, C5 exist; C4's counter and `0xE4`, C6 added by WP FWTB1 |
| USB-direct mode 2, 48 and 96 kHz | S | SOF count and the host rate | none; time base 2 gives the same bytes per unit | C1-C3, C5 exist; C4, C6 added by WP FWTB1 |
| S/PDIF between DSPi units | F | the S/PDIF superframe in the user bits, one per IEC 60958 block (section 12) | the same superframe (source `EXT_ALINK_SPDIF_CTRL`) | new: WP FWSP1, WP FWSP2 |

**No time base** (state `NONE`, single-unit work only; a host never schedules across them): **foreign S/PDIF** (any
source that is not a DSPi source in mode 2, and any S/PDIF path through equipment that re-clocks or rewrites the user
bits), **ADAT**, **I2S** (master or slave), **USB at 44.1 kHz** (no whole number of samples per bus frame), and **USB
modes 0 and 1** (rate lock without a timeline). A unit fed by them may still be rate-locked; it has no T.

### 11.5 Conformance checklist for a new transport

A new transport (ADAT, I2S/TDM, PTP) is a time-base transport when its implementation passes, in host tests through
the portable code, every check below. It then gets measurement, scenes and CLICK with no host change.

1. **Counter continuity across a relock:** after a holdover that resumes, T continues as if nothing happened
   (`RESUMED`); after a loss that ends the timeline, the next anchor places T by the received counter, never by a
   local guess, and the disturbance counter counts both.
2. **Apply at T on a split block:** an op whose T falls inside a processing block acts on exactly that sample, on
   programme audio and on silence; op 9's generator sample 0 is the sample carried as T.
3. **Late and refused:** an op whose T has passed is applied at once and reported `late`; op 9 is refused, never late.
4. **Holdover:** a short loss of the carrier is ridden through on the extrapolated T (muted), and a return whose
   counter does not confirm the extrapolation re-anchors (`REANCHOR`).
5. **The disturbance counter:** every event listed in C4 increments it exactly once, and `t - output_frames` is
   identical over 1000 atomic reads while `LOCKED` and undisturbed.
6. **The domain id:** two units of one domain report equal `domain`; a source restart (new epoch, new origin) changes
   it.
7. **The states:** every transport state maps to exactly one contract state (11.2), and every transition sends one
   `0xE4` (subject to the 250 ms merge).
8. **Discovery:** `EXT_LIST` names `0x8030` and the transport's own block; `EXT_TIME_BASE` lists one entry per time
   base the unit has, follower first.

## 12. Carriage on S/PDIF

*Normative (WP FWTB0, PLAN-AT section 1.2, OD-ATc, OD-ATs). The codec is WP FWSP1 (`firmware/DSPi/alink/
alink_spdif_codec.{c,h}`, the Python twin `tools/alink/alink_spdif.py`, vectors `firmware/tests/
alink_spdif_vectors.json`), the roles WP FWSP2 (`firmware/DSPi/alink_spdif.{c,h}`). The control plane is
`extension_channel_spec.md` 2.10.*

Between DSPi units the S/PDIF user bits are free: consumer equipment uses the channel-status bits, and the DSPi's own
receiver already reads the raw subframe words that carry the user bit. A **source** (`EXT_ALINK_SPDIF.mode` 2) puts
one **S/PDIF superframe** into the user bits of every IEC 60958 block it sends; a **follower** (mode 1) whose active
input is S/PDIF reads it and phase-locks to it with the same servo as the line and USB-direct (`alink_phase`).

### 12.1 Where the bits are

- **The block:** 192 frames, both subframes, 384 subframes, starting at the subframe with the Z (B) preamble. One
  superframe per block (`SPDIF_BLOCK_FRAMES` = 192, `SPDIF_USER_BITS` = 384).
- **The order:** user bit i of the block is the U bit (IEC 60958 time slot 29; bit 29 of the DSPi's raw subframe word,
  `SPDIF_input_spec.md` 5) of subframe i in transmission order: subframe 2k is channel A (left) of frame k, subframe
  2k + 1 channel B (right). Bit i is **bit `i mod 8` of byte `i / 8`**, least significant bit first, the order the
  firmware uses for the channel-status bits. The 384 bits are 48 bytes (`SPDIF_SUPERFRAME_LEN` = 48).
- **The block's first frame** is the frame with `F mod 192 = 0` (12.3). The receiver identifies it by the Z preamble,
  not by counting.

### 12.2 Layout

Bytes 0-31 are the line's superframe layout (6.1) with S/PDIF values; bytes 32-47 are padding.

| Byte | Field | On S/PDIF |
|---|---|---|
| 0 | magic | `0xA5` |
| 1 | version | 1 |
| 2 | flags | b0 system mute: sent 0 in this version (nothing sets it on an S/PDIF source); a follower that receives it set plays silence and refuses CLICK and op 9 as on the line (`extension_channel_spec.md` 2.8.3, 2.8.7); b2 apply pending (derived as on the line, 6.2); b3 the source has a live USB stream; b1, b4, b5, b6, b7 sent 0 (no RATE, no test pattern, no slaving on S/PDIF) and ignored by followers |
| 3 | rate | rate code of the block: 0 = 44.1 kHz, 1 = 48 kHz, 2 = 96 kHz |
| 4 | channels | 2 |
| 5 | class | **`0x10`** (`SPDIF_CARRIER_CLASS`): S/PDIF carriage. Never a line class (1..3), so a line receiver can never accept an S/PDIF superframe, nor a follower a line one |
| 6 | hops | 0 (reserved for the relay, PLAN-AT "Later"); ignored by followers in this version |
| 7 | sender | the source's serial byte 0: the last byte of its unique board id (`pico_get_unique_board_id()`, byte 7), as the line's sender byte |
| 8-15 | frame | F of the block's first frame, u64; **`F mod 192 = 0`** (replaces the line's mod-32 rule) |
| 16-23 | apply_frame | apply target F, u64; 0 = none |
| 24 | apply_op | 0 none, or one of **1 MUTE, 2 UNMUTE, 4 MASTER_VOLUME, 7 CLICK, 9 SIGGEN_START, 10 SIGGEN_STOP** (6.4); 3, 5, 6 and 8 are never carried on S/PDIF |
| 25 | apply_arg | as 6.4 |
| 26-27 | apply_value | i16, as 6.4 |
| 28 | epoch | the source's epoch (12.3), u8, wraps |
| 29 | sender2 | the source's serial byte 1: byte 6 of its unique board id |
| 30-31 | crc | CRC-16/CCITT-FALSE over bytes 0-29, low byte first, as on the line (5.5) |
| 32-47 | padding | sent 0, ignored |

The source's **id** is the u16 `sender | sender2 << 8` (`EXT_ALINK_SPDIF_STATUS.source_id`, `EXT_TIME_BASE.domain`
bytes 0-1).

**Validity**, in this order (the codec returns the first failing check with the line's codes, 6.3): the CRC matches
(1); magic is `0xA5` (2); version is 1 (3); `frame mod 192 = 0` (4); class is `0x10`, rate is 0..2 and channels is 2
(5); `apply_op` is 0 or in the S/PDIF set {1, 2, 4, 7, 9, 10}, and `apply_op = 0` exactly when `apply_frame = 0`
(6). As on the line, the codec does not judge `arg` and `value`: a superframe with a bad argument still carries a
valid F, and the unit refuses the op when it takes it (`extension_channel_spec.md` 2.8.7). The S/PDIF codec has its
own allowed-op set; it does not take `ALINK_OP_MAX` from the line's codec. A follower adds the checks of 12.5 (rate,
continuity, source and epoch).

**Apply carriage:** a superframe carries the pending apply exactly when its block completes before the target,
`frame + 192 <= N`. The source carries one broadcast apply at a time (`extension_channel_spec.md` 2.10.3).

**Example** (the vector FWSP1 MUST include as `spdif_sf/typical`): a source with serial bytes 0-1 `0x42`, `0x37`,
epoch 3, 48 kHz, the block at F = 960 000 (192 x 5000), carrying op 9 `SIGGEN_START` at N = 1 000 000. The CRC was
computed with the line's reference `crc16()` (`tools/alink/alink_codec.py`, run in the alink tools container; the same
run reproduces `sf/typical`'s `d7 47`):

```
a5 01 04 01 02 10 00 42  00 a6 0e 00 00 00 00 00  40 42 0f 00 00 00 00 00  09 00 00 00 03 37 a7 80
00 00 00 00 00 00 00 00  00 00 00 00 00 00 00 00
```

A block with no apply pending (F = 960 192, same source; were the op above still pending, this block would carry it
too): `a5 01 00 01 02 10 00 42 c0 a6 0e 00 00 00 00 00`, 12 zero bytes,
`03 37 90 dd`, then 16 zero bytes. The block's first 16 user bits, in subframe order (A0, B0, A1, B1, ...), are
`1 0 1 0 0 1 0 1 1 0 0 0 0 0 0 0` (bytes `a5 01`, LSB first): frame 0 carries A = 1, B = 0.

### 12.3 The source's counter and epochs

- **F counts the frames the source's outputs play.** F of a frame is the output-frame position at which the source's
  DAC outputs play it and its S/PDIF outputs emit it: all output slots start together
  (`start_output_slots_in_sync`), so they share one frame index. The origin is set when the outputs start in sync
  and is otherwise arbitrary, with one rule: **every IEC 60958 block the source emits starts at a frame with
  `F mod 192 = 0`** (the transmit library's block phase is 0 at the synchronized start, so an origin that is a
  multiple of 192 there meets it).
- **Epochs.** Any output restart (preset load, rate change, pipeline reset, output-type switch, a flash write that
  restarts the outputs, a USB mode-0 stream start, `usb_audio.c`) breaks the frame sequence. The source then starts a
  new **epoch**: a new origin, byte 28 incremented (mod 256), a disturbance counted, `0xE4` reason 11 `EPOCH`; every
  follower re-anchors (12.5). A source SHOULD therefore run USB-direct mode 2 or a non-USB input, whose timeline
  survives stream restarts; on USB mode 0 every "play" is a new epoch. The transmit library's frame index is 32 bits
  and today its block phase steps by 64 frames at the wrap (24.8 h of uninterrupted output at 48 kHz, 12.4 h at
  96 kHz; `audio_spdif.c`, "known limit"): WP FWSP1 either keeps the block phase continuous across the wrap or the
  source starts a new epoch there. F itself is 64 bits and never wraps.
- **The source is a member of its own domain.** Its `EXT_TIME_BASE` lists a source entry (role 1, D = 0), and its
  apply queue (`EXT_APPLY_AT_FRAME` time base 3) acts on the sample written at output position F.
- **Which slots carry it.** Every S/PDIF-type output slot, or the slots in `EXT_ALINK_SPDIF.tx_mask`. A slot that does
  not carry it, and every S/PDIF output of a unit not in mode 2, emits all-zero user bits, byte-identical to firmware
  before WP FWSP1. Consumer equipment on a stamped output sees nonzero user bits, which IEC 60958 consumer devices
  ignore; `tx_mask` leaves a slot unstamped where one does not.
- **The source's time-base slots are never in a generator mask** (the hub refuses it, PLAN-AT S-AT6): followers would
  receive the stimulus as programme.

### 12.4 How the source emits it (transmit rules)

Stamping is not monotonic: the 512-frame transmit ring (`SPDIF_TX_RING_FRAMES`, 2.7 blocks) is stamped up to one ring
ahead by the silence pump (from the main loop, and from the 1 ms timer interrupt during a flash blackout), and the
audio writer later overwrites the same frames. So:

- **(a) A block's superframe is a function of its block index** `F / 192`. The source keeps a table of
  `SPDIF_TX_TABLE_BLOCKS` = **4** entries keyed by block index (entry `index mod 4`, holding its index): the stamped
  span reaches at most 512 frames from the reader, so it touches at most four blocks. Both stampers, the silence
  stamp and the audio write, read it by the block index of the frame they stamp; an entry whose stored index is not
  that block's stamps zero user bits. The entry of block b + 4 replaces block b's, so it can be filled only once the
  reader has left block b, and it must be filled before the first frame of b + 4 is stamped: with the idle pump
  stamping a full ring ahead that window is 64 frames (1.3 ms at 48 kHz). The fill therefore runs in the same pass as
  the stamp, immediately before it (the main-loop pump and the audio give path), not as a separate periodic task. There is
  no current/next pair swapped at a boundary: a double buffer mixes two blocks' bits in one block and fails every
  follower's CRC.
- **(b) A block is frozen once any of its frames is stamped.** Its content, the apply field included, never changes
  after that. The fill interface refuses an entry whose block is already (partly) stamped. A broadcast op therefore
  enters only blocks not yet stamped, and the **shortest broadcast lead is the ring plus two blocks**,
  `SPDIF_CTRL_MIN_LEAD_FRAMES` = 512 + 2 x 192 = **896 frames** (18.7 ms at 48 kHz) from the frame the source plays
  now: the stamped frontier is at most 512 frames ahead of it, so the first block not yet stamped starts at most 703
  frames ahead and completes at most 895 frames ahead, and a target 896 frames ahead is carried by at least one
  complete superframe. `EXT_ALINK_SPDIF_CTRL` refuses a shorter lead (`extension_channel_spec.md` 2.10.3). A host
  SHOULD give more (a lost block costs the followers that op): every 192 frames beyond 896 adds one carrying block.
- **(c) Flash blackouts.** While flash is unavailable the RAM-resident stamper only reads the table, which the
  flash-resident source code has filled as far as it reaches: the reader's block and the three after it. The blackout
  pump stamps a full ring ahead, so it passes the end of the table after 64 to 256 frames (1.3 to 5.3 ms at 48 kHz)
  and from there emits all-zero user bits (an entry not filled for that block stamps zeros, and the block stays zero:
  it is frozen, b), which the followers coast over (d) and, past 8 blocks, hold over. Every real flash write (tens of
  milliseconds, and an output restart after it, which is a new epoch anyway, 12.3) therefore costs the followers a
  holdover or a re-anchor. A measurement never overlaps a flash write (the holdover is a disturbance).
- **(d) The follower's coast rule.** A block whose superframe is missing or invalid (zero user bits, a failed check)
  is ridden through on counter continuity: the follower takes F of that block as the previous F + 192, counts it in
  `sf_errors`, and changes nothing else (no disturbance, no event, the timeline unmoved). After
  `SPDIF_COAST_BLOCKS` = **8** consecutive such blocks it enters `HOLDOVER`. The next valid superframe must confirm
  the continued F (equal to the coasted value, same source id and epoch); otherwise the follower re-anchors (reason 5
  `REANCHOR`, or 11 `EPOCH` when the epoch or source changed).

### 12.5 The follower

- **Collection.** The receiver library's block interrupt already walks the 384 verified words of each block for sync,
  parity and the channel-status bits, with the block's timestamp; it collects U alongside C and hands the 48 bytes and
  the timestamp to the follower (WP FWSP1). Without a follower it costs nothing.
- **Acceptance**, after 12.2's validity: the rate code equals the rate the receiver detected; the source id and epoch
  equal the previous accepted superframe's (else a new domain: re-anchor, reason `EPOCH`); F equals the previous F +
  192 (the coast rule, 12.4 d, covers gaps).
- **Phase lock.** The same `alink_phase` servo as the line and USB-direct: `e = F_arr - (next_w - fill) - D`, with
  `F_arr` (the F arriving now) from the receive DMA position against the block start, at subframe resolution (half a
  frame), smoothed by the servo's IIR and PI as the line's quarter-frame output quantisation is. Anchor, verify,
  whole-frame corrections, PI, holdover on loss (VCXO trim held, outputs on silence, T extrapolated), re-anchor
  beyond 2 frames (`AUTO_REANCHOR`), relock placed by F. `d_min` and `d_max` are computed as on the line
  (`extension_channel_spec.md` 2.8.1) plus the receiver's in-flight margin, and reported in
  `EXT_ALINK_SPDIF_STATUS`; the hub chooses one D for the domain.
- **Holdover** lasts at most 10 s (as USB-direct's); a return within it that confirms F resumes on the same timeline
  (reason 4 `RESUMED`), anything else re-anchors. Beyond 10 s the time base ends (reason 2 `LOST`, state `ACQUIRING`).
  The 10 s are for a carrier that is gone (a pulled cable, a source restarting). While the receiver stays locked to a
  stream, the lock-error rule below ends the holdover first: 0.5 s without a valid superframe ends the time base
  (`LOST`) and the follower is in `LOCK_ERROR`.
- **Lock error** (PLAN-AT OD-ATp): a follower in mode 1 whose receiver is locked to a stream but has seen no valid
  superframe for 0.5 s, in any state (a foreign source; a DSP, AVR or switch in between that rewrites or re-clocks the user bits;
  persistent CRC failures), or whose D is outside `d_min..d_max`, or that runs at a rate it cannot lock at, is
  `LOCK_ERROR`. Its action is `EXT_ALINK_SPDIF.on_lock_error`, as on USB-direct: **0 mute** (the default) or **1 play
  unaligned** (today's rate lock). Mode 0 stays the default, so a TV on S/PDIF is never muted by a forgotten setting.
  The UI says "this S/PDIF source does not carry a DSPi time base".
- **What carries it:** a direct wire (3.3 V logic between boards, common ground), coax through a transformer, TOSLINK
  including passive optical splitters. Anything that re-clocks or rewrites the user bits does not.
- **Builds.** RP2350 builds, the K340 included (`DSPI_ALINK_SPDIF`, on by default there). RP2040 builds have no
  S/PDIF time base in this version (PLAN-AT OD-ATm). A source needs an S/PDIF-type output slot; a follower an S/PDIF
  input (the K340's RX1 default is GP5).
