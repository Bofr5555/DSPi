# USB MIDI Control Channel (`DSPI_USB_MIDI`)

*Wire format version: 1 (the `ver` byte of every message).*

An optional, class-compliant USB MIDI 1.0 function that carries the complete
vendor command surface inside SysEx. It exists for hosts that can talk to a
class-compliant MIDI device but cannot send USB vendor control requests, most
importantly iPhone apps through CoreMIDI (iOS has no public API for vendor
requests to a USB device). Any other MIDI-capable host (macOS, Windows, Linux
ALSA rawmidi, Android `android.media.midi`) can use it too.

The design reuses the UART control transport unchanged at the frame level:
one SysEx message carries exactly one UART frame (`control_interfaces_spec.md`
section 3), so framing, CRC, status codes, the one-request rule, the
notification frame and the size caps are already specified there.

Writing style note: this doc avoids em-dashes per project convention.

---

## 1. Build option

| CMake option | Default | Effect |
|---|---|---|
| `DSPI_USB_MIDI` | `OFF` | `ON` compiles `midi_control.c` + `midi_frame.c`, the MIDI descriptors and TinyUSB's MIDI class driver. `OFF` builds are byte-identical to firmware without this feature. |
| `DSPI_USB_MIDI_MFR_ID` | `0x7D` | SysEx manufacturer ID: one byte `0x01..0x7F`, or three bytes `0x00,0xNN,0xNN`. Checked at compile time. |

`0x7D` is the ID the MIDI 1.0 specification sets aside for non-commercial and
educational use. It is the development default; a product release should use
a registered manufacturer ID (see section 10). The device accepts only its own
ID and ignores every other SysEx.

Both chips build with the option on (RP2040 and RP2350). The cost is listed
in section 9.

## 2. USB descriptors

The MIDI function is appended **after** every existing interface. Nothing
before it moves:

| Interface | Function | Change |
|---|---|---|
| 0, 1 | UAC1 playback (AudioControl + AudioStreaming, IAD) | none |
| 2 | Vendor (class `0xFF`, notify EP `0x83`) | none |
| 3, 4 | Loopback capture (`DSPI_LOOPBACK` debug builds only) | none |
| 3, 4 (5, 6 with loopback) | **MIDI: AudioControl + MIDIStreaming** | new |

Every console addresses the vendor interface as 2 (`wIndex 2`), and the
Microsoft OS 2.0 descriptor set binds WinUSB to interface 2 by number; both are
unchanged, and the build asserts `ITF_NUM_VENDOR == 2`.

The function is an Interface Association Descriptor (class `0x01` Audio,
subclass `0x01` AudioControl, protocol `0x00`, two interfaces, `iFunction`
"DSPi Control") followed by TinyUSB's one-cable MIDI template
(`TUD_MIDI_DESCRIPTOR`, 92 bytes in TinyUSB 0.18.0): AudioControl interface with
a class-specific header naming the MIDIStreaming interface, MIDIStreaming
interface with one embedded and one external IN jack, one embedded and one
external OUT jack, bulk OUT endpoint `0x04` and bulk IN endpoint `0x84`, 64
bytes each. The only change to the template is the jack string index, set to
"DSPi Control" like the AudioControl `iInterface`, so a host that names ports
from jack strings and one that uses the interface string show the same name.
The configuration grows by 100 bytes; `bNumInterfaces` by 2.

**Endpoint budget.** Used before: EP1 OUT (audio), EP2 IN (feedback), EP3 IN
(notify), EP1 IN (loopback, optional). MIDI takes endpoint number 4 in both
directions. TinyUSB's RP2040/RP2350 port provides 16 endpoint numbers per
direction (`TUP_DCD_ENDPOINT_MAX 16`). USB DPRAM: the port allocates buffers
in 64-byte units from 3712 bytes and double-buffers bulk endpoints, so MIDI
uses 256 bytes; the worst case (RP2350 with loopback) is 832 (iso OUT) + 64
(feedback) + 128 (notify) + 320 (loopback) + 256 (MIDI) = 1600 bytes.

**Device descriptor.** `bcdDevice` is `0x0205` in MIDI builds (`0x0204`
otherwise) so Windows does not reuse descriptors cached for the same
VID/PID/bcdDevice. VID/PID, device class (`0xEF/0x02/0x01`, composite with
IADs) and the BOS / MS OS 2.0 descriptors are unchanged. No console matches on
`bcdDevice`.

## 3. Wire format

### 3.1 Message

```
F0 <mfr> 'D' 'S' 'P' <ver> <tag> <packed frame> F7

mfr    1 or 3 bytes, DSPI_USB_MIDI_MFR_ID (default 7D)
'DSP'  44 53 50
ver    01
tag    host -> device: 01..7F, chosen by the host, echoed in the reply
       device -> host: the request's tag; 00 = notification;
                       7F when the request's tag could not be read
packed the frame, 7-bit packed (3.2)
```

`frame` is a UART control frame (`uart_control.h`) **without the `0xA5` sync
byte**: from the type byte through the CRC16, CRC computed exactly as on UART
(CRC16-CCITT-FALSE over the type byte through the last payload byte,
transmitted little-endian):

```
SET request   01 bReq wValL wValH wIdxL wIdxH wLenL wLenH payload[wLen] crcL crcH
GET request   02 bReq wValL wValH wIdxL wIdxH wLenL wLenH               crcL crcH
SET response  81 status 00 00                                           crcL crcH
GET response  82 status lenL lenH payload[len]                          crcL crcH
Notification  40 00 lenL lenH packet[len]                               crcL crcH
```

Hosts should use tags `01..7E` and avoid `7F`, so a `7F` reply is always an
error about a message whose header could not be parsed.

### 3.2 7-bit packing

Each group of up to 7 frame bytes becomes one MSB byte followed by the low 7
bits of each byte. Bit *i* of the MSB byte is bit 7 of group byte *i*. A full
group is 8 SysEx bytes; a final partial group of *n* bytes is *n* + 1 bytes;
unused MSB bits are 0. `packed length = 8 * (n / 7) + (n % 7 ? n % 7 + 1 : 0)`.

Example: `81 02 83 04 85 06 87 FF` packs to
`55 01 02 03 04 05 06 07 01 7F` (MSB byte `0x55` = bits 0, 2, 4, 6).

### 3.3 Worked example

Read the platform (`GET 0x7F`), tag 1:

```
frame  02 7F 00 00 00 00 00 00 10 B0          (CRC 0xB010)
SysEx  F0 7D 44 53 50 01 01
       00 02 7F 00 00 00 00 00                (group 1: MSB byte 00)
       04 00 10 30                            (group 2: B0 has bit 7 -> MSB bit 2)
       F7
```

Reply (RP2350, v1.1.6, 4 outputs):

```
frame  82 00 07 00 01 01 16 04 01 06 00 C4 2F
SysEx  F0 7D 44 53 50 01 01  01 02 00 07 00 01 01 16  10 04 01 06 00 44 2F  F7
```

Set master volume to -20.0 dB (`SET 0xD2`, payload float `00 00 A0 C1`), tag 2:

```
frame  01 D2 00 00 00 00 04 00 00 00 A0 C1 CF 6D
SysEx  F0 7D 44 53 50 01 02  02 01 52 00 00 00 00 04  38 00 00 00 20 41 4F 6D  F7
reply  F0 7D 44 53 50 01 02  01 01 00 00 00 4C 2F  F7          (81 00 00 00 = OK)
```

These and more vectors (3-byte ID, 64-byte payload, the 6136-byte bulk SET, a
notification, an error reply) are in `firmware/tests/midi_vectors.json`, read
by both the firmware host test and the Python tool's tests.

### 3.4 Sizes

| Message | Frame bytes | SysEx bytes |
|---|---|---|
| GET request | 10 | 20 |
| SET request, 64-byte payload | 74 | 93 |
| SET 0xA1, 6136-byte payload (largest request) | 6146 | 7032 |
| GET 0xA0 reply, 6136 bytes (largest message) | 6142 | 7028 |
| Notification, largest v2 packet (64 B) | 70 | 88 |

No application-level chunking: USB bulk has flow control and SysEx has no
length limit. The non-bulk SET payload cap (64 bytes) and the non-bulk GET
reply cap (132 bytes, the `CsMacro`) are those of the UART transport.

### 3.5 USB-MIDI event packets

The device sends a message as 4-byte USB-MIDI event packets on cable 0: CIN
`0x4` for every packet but the last, the last packet CIN `0x5`, `0x6` or `0x7`
for 1, 2 or 3 bytes ending in `F7`, unused bytes zero. It accepts CIN `0x4` to
`0x7` and the single-byte CIN `0xF` on cable 0 and ignores every other packet
(note, controller and all other cables).

## 4. Device behaviour

### 4.1 Receiving

From the main loop (`midi_ctrl_poll()`, next to the UART and I2C polls),
packets are read out of TinyUSB's RX FIFO and fed byte by byte to the decoder
(`midi_frame.c`): header match, 7-bit unpack, then the UART frame parser.

* SysEx for another manufacturer ID, or with the right ID but not `'DSP'`, is
  ignored silently. So are bytes outside a message and stray `F7`s.
* Real-time bytes (`F8..FF`) inside a message are ignored. Any other status
  byte (`F0`, `F1..F6`) ends the message; if the header had matched, the
  message is answered `FRAME_ERROR` and `F0` starts a new one.
* `ver` other than 1, a host tag of 0, or a message ending inside the header
  after `'DSP'`: `FRAME_ERROR` with tag `7F`.
* Malformed packing (a lone MSB byte, a set MSB bit for a missing byte), an
  unknown frame type, a frame shorter than its length fields say, or bytes
  after the CRC: `FRAME_ERROR` with the request's tag.
* Bad CRC: `CRC_ERROR` with the request's tag. Nothing is dispatched.
* Non-bulk SET with `wLength` > 64: `OVERSIZE` after the message ends; the
  payload is discarded.
* Bulk SET (`0xA1`, `wLength` = `sizeof(WireBulkParams)`): the bulk lock is
  taken with `CTRL_SOURCE_MIDI` when the header arrives and the payload is
  streamed straight into `bulk_param_buf`, refreshing the lock every 64 bytes.
  If the lock is held elsewhere the payload is discarded and the message
  answered `BULK_LOCKED`.
* Errors are answered with response type `0x82` when the frame type was GET,
  `0x81` otherwise (including when the type is unknown).
* A message that stops arriving for 1 s is dropped silently (as on UART) and
  any bulk lock it held is released. Unplug / reconfiguration resets
  everything.

There is **one pending slot**. A request is parsed while the previous reply is
still being transmitted and waits in the slot; while the slot is full the
device reads nothing more, so USB flow control (NAK) holds the host back.
Nothing is dropped and no `BUSY` is generated by pipelining. Hosts should still
keep one request in flight (the UART rule); pipelined requests work but a bulk
SET sent while a bulk GET reply is streaming gets `BULK_LOCKED`.

### 4.2 Dispatch

The pending request goes through `vendor_dispatch_get/set(CTRL_SOURCE_MIDI,
...)` once no reply is being transmitted, with the UART transport's retry rule:
`BUSY` (a USB control SET in flight), and `BULK_LOCKED` on a GET, are retried
internally for 50 ms before being reported. Status codes are the shared
`CTRL_STATUS_*` set (`control_interfaces_spec.md` 3.4).

**Command set: parity with the USB vendor path.** The MIDI channel rides on
USB and is meant to behave like the normal USB vendor control channel, not like
the external UART / I2C links. Every opcode that is source-dependent in the
firmware is treated as follows:

| Opcode | USB vendor path | UART / I2C | USB MIDI | Why |
|---|---|---|---|---|
| `0xF5` `SET_UART_CONFIG` | allowed | `BLOCKED` | **allowed** | The UART/I2C refusal stops a controller locking itself out of its own link. A MIDI host is on USB and cannot; an iPhone-only user needs `0xF5` to enable an ESP32 bridge. Applied deferred and persisted exactly as over USB; read the result with `0xF9`. |
| `0xF7` `SET_I2C_CONFIG` | allowed | `BLOCKED` | **allowed** | Same reasoning as `0xF5`. |
| `0xA2` `GET_ALL_PARAMS_CHUNK` | allowed | `BLOCKED` | **allowed** | Served by `midi_control.c` with the USB semantics of `bulk_params_chunking.md`: offset 0 snapshots the whole struct under the bulk lock, later offsets read that snapshot, the final chunk releases the lock. Offset > 0 without a session: `ERROR` (USB: STALL). |
| `0xA3` `SET_ALL_PARAMS_CHUNK` | allowed | `BLOCKED` | **allowed** | Same: sequential chunks from offset 0 land in `bulk_param_buf`, the chunk completing the struct applies it like `0xA1`. Out of order or past the end: `ERROR`, restart at 0. A session ends with the first non-chunk request, after 3 s idle, or on unplug, as on USB. Lock held by another transport: `BULK_LOCKED` (USB: STALL). |
| `0x0F` `RTA_GET_BANDS_ALL` | allowed | `ERROR` (over the 132-byte cap) | **allowed** | MIDI has no reply cap; answered zero-copy out of `bulk_param_buf` under the bulk lock, like `0xA0`. |
| `0x82` `GET_CLOCK_DIAG` (`DSPI_CLOCK_DIAG` bench builds only) | full packet | zero bytes if over 132 | zero bytes if over 132 | The one remaining difference: the packet lives in a handler-local buffer the external copy path caps at 132 bytes. Bench-only; not worth a second large buffer. |
| `0xF0` `ENTER_BOOTLOADER` | allowed | allowed (reply lost) | allowed, reply flushed first (4.5) | |
| everything else | allowed | allowed | allowed | Same dispatcher. |

`0xA0` / `0xA1` remain the recommended bulk commands over MIDI (no size cap, one
message); the chunked forms exist so a client written against the USB vendor
path works unchanged.

Changes made over MIDI notify with source `PARAM_SRC_MIDI` (10), on every
consumer: the USB notify endpoint `0x83`, UART notification frames and the
MIDI channel itself.

### 4.3 Transmitting

Replies and notifications are encoded on the fly into USB-MIDI packets and
offered to TinyUSB's TX FIFO (512 bytes) from the main loop at FIFO pace; the
module never blocks or busy-waits. Small GET replies are snapshotted (132-byte
buffer) right after dispatch. The `0xA0` reply is streamed zero-copy from
`bulk_param_buf` while holding the bulk lock, refreshed every 64 bytes and
released as soon as the last payload byte is encoded. If the host stops
reading in the middle of a bulk reply for 400 ms, the device ends the SysEx
early (the host sees a malformed message and retries) and releases the lock,
so a stalled MIDI host cannot pin the bulk buffer for USB/UART/I2C.

### 4.4 Notifications

Every v2 notify packet is sent as a type-`0x40` frame with tag 0, exactly the
bytes the USB notification endpoint delivers (`notification_protocol_v2_spec.md`;
the v1 legacy master-volume packet is never sent here). There is no opt-in:
the MIDI consumer of the notify ring is active whenever the MIDI function is
configured by the host. A notification is started only between messages and
never ahead of a pending request, so it never splits or delays a reply. A host
that does not read the port backs up the TX FIFO; the ring then force-drops
this consumer's oldest entries, and the client detects that from the gap in
the packet's `seq` byte and re-reads `0xA0`. After opening the port a client
may receive a few notifications queued before it opened; treat them like any
other.

### 4.5 Bootloader entry (`GET 0xF0`)

The command handler resets into the bootloader and never returns, so the MIDI
path sends the reply itself first: it waits for the MIDI IN endpoint to be
idle, queues the reply (`82 00 01 00 01`, OK with the byte `1`, as on USB),
waits up to 200 ms for the host to take it, then dispatches. The device then
leaves the bus. An iPhone cannot flash the DSPi; an app should say so before
sending `0xF0`.

### 4.6 Flash writes

Commands that write flash blind the device for about 45 ms. Nothing special
happens on this path: USB flow control holds incoming data, the reply comes
late. Use generous timeouts (hundreds of ms) and retry idempotent GETs.

## 5. Host guidance

* Find the port by name ("DSPi Control", on a device whose USB product is
  "Weeb Labs DSPi"); check manufacturer / product / serial where the MIDI API
  exposes them.
* One request in flight; match the reply by tag; accept tag-0 notifications
  at any time between messages; treat a tag-`7F` error as the answer to the
  request in flight.
* Retry on `BUSY`, `BULK_LOCKED`, `CRC_ERROR` and timeouts.
* Throughput depends on the host's SysEx pacing (CoreMIDI's
  `kMIDIPropertyMaxSysExSpeed`); measure, and prefer the per-parameter
  commands plus notifications over repeated `0xA0` reads.
* Linux: the port is an ALSA rawmidi device (`/dev/snd/midiC<card>D0`), which
  passes SysEx through untouched. `tools/midi_ctrl/` is a standard-library
  Python client, measurement and soak tool.

## 6. Implementation map

| File | Role |
|---|---|
| `midi_frame.{c,h}` | Pure codec: CRC, 7-bit packing, encoder (frame to SysEx / packets), decoder (SysEx to request events). No SDK headers; built by `firmware/tests/run.sh`. |
| `midi_control.{c,h}` | TinyUSB glue, pending slot, dispatch, TX pump, notifications, bootloader flush. |
| `usb_descriptors.{c,h}` | Interface / endpoint numbers, IAD + MIDI template, string 4, `bcdDevice`. |
| `tusb_config.h` | `CFG_TUD_MIDI 1`, RX FIFO 256 B, TX FIFO 512 B, EP buffer 64 B. |
| `vendor_commands.{c,h}` | `CTRL_SOURCE_MIDI = 4`; notify source mapping; USB-path command set for MIDI (`CTRL_CMD_REFUSED`, `CTRL_BULK_GET`, `0x0F` zero-copy). |
| `notify.h` | `PARAM_SRC_MIDI = 10`; `NOTIFY_CONSUMER_MIDI`. |
| `firmware/tests/test_midi_frame.c`, `test_midi_control.c` | Host tests (codec; glue against TinyUSB / dispatcher mocks). |

## 7. Tests

`firmware/tests/run.sh` (host C compiler):

* `test_midi_frame`: CRC check value `0x29B1` over `123456789`; pack/unpack
  round trip for lengths 0..300, 6146 and 6147; the packing and message worked
  examples; every vector in `midi_vectors.json`; the 7032 / 7028 sizes; the
  largest request fed in every piece size 1..64 and through USB-MIDI packets;
  header cases (good, wrong ID, other product, 3-byte ID, wrong version, tag 0,
  truncated at several points); CRC error, bad type, trailing bytes, real-time
  interleave, F0 and system-common aborts, lone MSB byte, unused MSB bit,
  discarded payloads; random garbage followed by a good message.
* `test_midi_control`: mount / unmount, small GET / SET, dispatcher errors,
  `OVERSIZE`, `0xF5` reaching the dispatcher, `0x0F` larger than 132 bytes,
  chunked `0xA2` / `0xA3` sessions (order, session end, idle reap, lock
  contention), CRC and version errors, foreign SysEx, bulk SET with
  back-pressure, `BULK_LOCKED`, lock release on CRC error / timeout / unplug,
  bulk GET with a slow host and with a stalled host, `BUSY` retry window,
  pipelined requests, notification ordering, bootloader reply ordering.

`tools/midi_ctrl/` (container, no device): codec and vectors, the client
against an in-process fake device and against a fake device on a pty.

## 8. Not covered by the host tests

Enumeration on real hosts (Linux, macOS, Windows, iOS), CoreMIDI's SysEx pacing,
and coexistence with streaming audio are hardware tests (`tools/midi_ctrl/`
`regress.sh`, `measure.py`, `soak.py`).

## 9. Cost

Measured on the `usb-midi-control` branch, OFF vs ON, Arm GNU 14.2, release
build (`DSPi.bin` size; `.data` / `.bss` and free RAM as reported by
`scripts/check_ram_placement.py`):

| Chip | Flash image | `.data` (RAM code + data) | `.bss` | Free RAM |
|---|---|---|---|---|
| RP2040 | 283384 -> 290656 (+7272 B) | 71032 -> 72072 (+1040 B; budget 72704) | +1408 B | 58224 -> 55776 (-2448 B) |
| RP2350 | 312612 -> 319516 (+6904 B) | 97656 -> 98592 (+936 B) | +1408 B | 119296 -> 116952 (-2344 B) |

`.bss` is TinyUSB's MIDI FIFOs (768 B) and endpoint buffers (128 B), the
module's state and buffers (payload 64 B, reply copy 132 B, notify 64 B, codec
and chunk-session state) and one more notify-ring consumer. `.data` grows because the TinyUSB
device core and FIFO helpers, which the build keeps in RAM, gain the MIDI
driver's code paths. On RP2040 this leaves 632 B of the `.data` budget. On
RP2350 the heap start moves by 9.6 KB, not 2.3 KB, because `.bss` starts at the
next 8 KB boundary (`input_capture_arena` alignment) and the larger `.data`
crosses one; that padding is a layout effect of the existing linker script.

## 10. Decisions and open points

1. **Manufacturer ID for release builds.** `0x7D` (development) stays the
   default (decided 2026-11-03). A product may switch to a registered ID with
   `DSPI_USB_MIDI_MFR_ID`; hosts then match it. Weeb Labs' choice.
2. **Command set (decided 2026-11-03).** Parity with the USB vendor path:
   `0xF5`, `0xF7`, `0xA2`, `0xA3` and `0x0F` are allowed over MIDI (4.2). The
   only remaining difference is `0x82` on bench builds.
3. **`bcdDevice` 0x0205** for MIDI builds (decided 2026-11-03).
4. **Notification opt-in.** Always on while configured. A host that never opens
   the port leaves up to 512 bytes of stale notifications queued.
