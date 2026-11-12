# Multi-Unit Audio Link: Line Protocol

*Protocol version: 1 (superframe byte 1). Status: normative, 2026-11-13 (WP LK1). Not yet used by any firmware
image: the sender (WP LK4), the receiver input (LK5), phase lock (LK6) and the repeater (LK8) build on it.*

The link carries the audio of a whole system from one sender (the master clock) to any number of receivers (a DSPi
or a K340 with a receiver module) over optical or SFP lines, one frame per sample period, with a 64-bit frame
counter and a control sub-channel. The system design is SYS2 (`~/.cache/dspi-build/work/sys2/SPEC.md`); this
document is the normative line protocol it calls for. The control commands for the link (`0x8000`-`0x80FF`) are in
`extension_channel_spec.md` section 2.8.

The reference implementation is bit-exact and portable: C in `firmware/DSPi/link/`, a Python twin in
`tools/link/`, and shared test vectors in `firmware/tests/link_vectors.json` (section 9). Where this text and the
reference code disagree, that is a bug; report it.

Writing style note: this doc avoids em-dashes per project convention.

---

## 1. Scope

In scope: line-rate classes, the 8b/10b line code and bit order, the frame, banks and CRCs, frame length at every
sample rate (44.1 kHz padding), the 32-byte superframe with the frame counter and apply-at-frame, hop counting for
repeaters and fan-outs, the PRBS-7 test payload, and the receiver's alignment, lock, error and loss rules.

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
  decoders keep a bit cursor that runs across frames and across a ring's wrap (the reference does: `link_bitw_t`
  appends at any bit position, and `link_rx_push()` takes bits in any chunks).

## 3. Line-rate classes

A class is a fixed line rate, independent of the sample rate. All are integer PIO dividers of the DSPi's 307.2 MHz
system clock and integer multiples of 48 kHz x 10.

| Class | Code | Line rate | PIO cycles per bit at 307.2 MHz | Bit period | Typical medium |
|---|---|---|---|---|---|
| L1 | 1 | 15.36 MBd | 20 | 65.1 ns | TOSLINK-class 25 Mb/s parts (PLR237 / PLT237) |
| L2 | 2 | 30.72 MBd | 10 | 32.6 ns | Broadcom 50 MBd Versatile Link, SFP |
| L3 | 3 | 61.44 MBd | 5 | 16.3 ns | SFP 100BASE-FX (experimental) |

- Both ends run the same nominal rate; the receiver's sampler re-phases on every transition and its soft VCXO
  removes the rate error (SYS2 2.6, 2.8). The protocol has no rate negotiation: a receiver either knows the class or
  detects it by trying each sampler (SYS2 2.6) and confirms it with superframe byte 5.
- Codes 0 and 4..255 are reserved. A 38.4 MBd class (SYS2 "L2+") is not part of version 1.
- The line MUST NOT be left at a constant level while a link is enabled: a sender sends frames from power-up, with
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

Channel n of the frame (the "link slot" of `EXT_LINK_SLOTS`) is in bank `n / 8` at position `n mod 8`. There is no
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

These are protocol limits. A sender MAY offer less (`EXT_LINK_TX` in `extension_channel_spec.md` 2.8.8 limits C to
the 48 kHz values at 44.1 kHz and to whole banks, 16, on L3 at 96 kHz). Defaults are whole banks: 8 on L1, 16 on L2,
32 on L3 at 48 kHz.

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
  sender's other flags come from its state (`EXT_LINK_TX`).
- **rate, channels** describe the 32 frames of this superframe. They change only at a superframe boundary
  announced by a RATE apply (6.5).
- **class** is the class of the line it is sent on; a receiver rejects a superframe whose class differs from the
  class its sampler runs (8.6).
- **sender** lets a hub tell two systems apart; it has no protocol meaning.

### 6.3 Validity

A superframe is valid when, in this order: its CRC matches; magic is `0xA5`; version is 1; `frame mod 32 = 0`; class
is 1..3, rate is 0..2 and C is 1..the capacity of 5.7 for that class and rate; `apply_op` is 0..8 and
`apply_op = 0` exactly when `apply_frame = 0`. The reference `link_sf_unpack()` returns the first failing check
(`LINK_SF_ERR_CRC` 1, `MAGIC` 2, `VERSION` 3, `FRAME` 4, `LAYOUT` 5, `APPLY` 6). Reserved bytes 28-29 are not
checked. A receiver adds the checks of 8.6.

### 6.4 Apply at frame N

Bytes 16-27 carry one pending operation for every unit on the link: "at frame N, do op". A superframe carries the
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

What a unit does with ops 1-5, 7 and 8, the argument ranges and the late and refused results are specified in
`extension_channel_spec.md` 2.8.7; the superframe only transports them. Op 6 is the protocol's own (6.5).

### 6.5 Layout changes (RATE)

The frame layout (rate and C) changes only at a superframe boundary N (`N mod 32 = 0`) announced with apply op RATE
(arg = new rate code, value = new C or 0 for unchanged). From frame N on, the sender uses the new layout and the
superframes describe it; receivers that saw the announcement switch at N too (8.5), so no frame is lost. A RATE op
whose new layout does not fit (5.7) MUST NOT be sent, and a receiver ignores it. A receiver that missed the
announcement loses lock at N and re-locks (8.8).

`EXT_APPLY_AT_FRAME` and `EXT_LINK_CTRL_MSG` (2.8.7, 2.8.9) require value 0 for RATE: hosts change only the rate
there. The sender puts the new C in `value` itself when its channel count changes (`EXT_LINK_TX`).

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
only at superframe boundaries; and change the layout only through RATE (6.5). All links of a star send the same bit
stream (SYS2 4.2).

### 7.2 Repeaters and fan-outs (the hop byte)

A repeater (a receiver with a transmit pin) or a fan-out (a sender slaved to a link input) re-emits the stream it
receives, delayed by exactly one frame:

- The re-emitted frame F carries the same F, the same payload and the same layout; HDR is unchanged.
- Its superframe is the received one with `hops = min(hops + 1, 255)` and `class` set to the class of the outgoing
  line, CRC recomputed (`link_sf_repeat()`). 255 means "255 or more".
- It re-encodes (its own RD and padding), so its output is again a conforming stream. A repeater that has no valid
  superframe yet sends only D21.5 idles (no commas, so downstream units see no stream rather than a wrong one,
  and the line still never sits at a constant level).
- Receivers use hops to compute `D_local = D - hops` (SYS2 3.1); the hub writes it (`EXT_LINK_PHASE`).

## 8. Reference receiver

This section is the bit-exact behaviour of `link_decoder.c` / `Decoder`: alignment, lock, the counters and what is
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

| Counter | Counts | `EXT_LINK_STATUS` field (2.8.2) |
|---|---|---|
| `lock_count` | entries into LOCKED | `lock_count` |
| `loss_count` | losses from LOCKED | `loss_count` |
| `crc8_errors` | bad heads (non-data symbol or CRC-8), ALIGNED and LOCKED | `crc8_errors` |
| `crc16_errors` | failed banks (non-data symbol or CRC-16) | `crc16_errors` |
| `bit_slips` | commas found 1-2 bits off | `bit_slips` |
| `superframe_errors` | rejected superframes (8.6) | `superframe_errors` |
| `comma_errors` | flywheel frames | `EXT_LINK_DIAG` (bench) |
| `counter_errors` | HDR mismatches and F corrections | `EXT_LINK_DIAG` (bench) |
| `code_errors` | non-data symbols in heads and decoded banks | `EXT_LINK_DIAG` (bench) |
| `idle_errors` | non-idle padding symbols | `EXT_LINK_DIAG` (bench) |
| `frames` | frames delivered | |

The receiver's state maps to `EXT_LINK_STATUS.state` as SEARCH and ALIGNED = 1 (searching), LOCKED = 2; holdover
(3) is a firmware state on top of SEARCH.

## 9. Reference implementation and test vectors

### 9.1 Files

| File | What |
|---|---|
| `firmware/DSPi/link/link_codec.h`, `.c` | 8b/10b, CRCs, layout and `S_F`, superframe pack/unpack/repeat, PRBS-7, bit packing, the sender's frame encoder `link_tx_*` |
| `firmware/DSPi/link/link_decoder.h`, `.c` | the receiver of section 8, `link_rx_*`, with an event callback |
| `tools/link/link_codec.py` | the Python twin of both, written from this document's tables; plus the test helpers of 9.2 |
| `tools/link/gen_vectors.py` | writes `firmware/tests/link_vectors.json` |
| `tools/link/tests/test_link_codec.py` | Python tests: anchors, stream properties, receiver behaviour, the vector file |
| `firmware/tests/test_link_codec.c` | C tests: the same anchors and properties, every vector, sample round trips |

The C code is portable C11 with no SDK, no heap, no floating point and no writable static data: every table is
`const` (flash), and the receiver's state (under 1 KB, `link_rx_t`) and the encoder's (under 100 bytes) live in the caller's
memory. No firmware target compiles it yet; LK3-LK5 add it to the build (and SHOULD put the hot parts of the
receiver in RAM and replace the reference symbol decoder with a table if the cycle budget of SYS2 5.1 asks for it).

### 9.2 Vector file

`firmware/tests/link_vectors.json` holds one flat JSON object per line (strings and integers only, so the C test
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
  `ins:P:B` inserts bit B before it, `fill:P:N:B` sets N bits from P to B. Positions refer to the clean stream; ops
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
38 and 41 channels, PRBS-7, a bank mask, RATE changes 48 to 96 kHz (with C 16 to 8) and 48 to 44.1 kHz, an apply,
and damage: a bit error in a bank, hold then zero, a bad head, a lost comma, slips of -2, -1, +1 bit, a 3-bit slip
(loss), three lost commas (loss), a 2000-bit dropout, a stray comma inside the payload, a broken superframe byte,
garbage before the stream, a superframe with a bad CRC and a wrong class byte.

### 9.3 Running the tests

```sh
# Python twin and vector file (podman, python:3.12-slim pinned by digest, no network)
tools/link/run.sh -m unittest discover -s tests -v
tools/link/run.sh gen_vectors.py > firmware/tests/link_vectors.json   # after a deliberate change

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
4. **Capacity** is computed (5.7). L3 at 96 kHz fits 18 channels (SYS2 said 16, counting whole banks) and 44.1 kHz
   fits more than 48 kHz.
5. **A missing comma is not an immediate loss** (SYS2 2.4 said it was; 3.3 treats a short loss as recoverable): the
   locked receiver flywheels, and loss is 3 consecutive failed frames (missing comma, bad head or HDR mismatch).
6. **Bit slips** of 1-2 bits re-align without loss; larger slips go through loss and re-search.
7. **Layout changes** (rate and C) happen only at superframe boundaries through apply op RATE, whose `value` carries
   the new C (0 = unchanged).
8. **Apply carriage:** carried in every superframe that completes before the target; at least one is required.
9. **Running disparity** is generated correctly but not checked by receivers in version 1.
10. **PRBS-7** is defined per frame (reset at each frame), so a bench receiver checks every frame on its own.
11. **Repeaters** keep F and rewrite hops (saturating at 255) and class; they send nothing until they hold a valid
    superframe.
