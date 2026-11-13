# Link sender: a Pico 2 as a USB sound card driving link outputs

*Last updated: 2026-11-13 (WP LK4)*
*Status: implemented, host-tested; not yet run on hardware*
*Platforms: RP2350 only*

The link sender is the dispatcher of the multi-unit audio link: the PC sees a USB sound card, and every channel it plays
goes out on up to 8 link outputs in the link protocol (`link_protocol_spec.md`), one identical bit stream per output,
to receivers that pick their channels (SYS2 section 4). Its crystal is the system's master clock. In fan-out mode it is
instead slaved to a link input and re-sends that stream.

It is a separate firmware target, `firmware/DSPi-Sender/`, not DSPi with a switch: it shares DSPi's USB descriptors,
feedback controller, soft VCXO, flash helpers, the link codec (`firmware/DSPi/link/`) and the PIO programs
(`link_tx_N.pio`, `link_rx_N.pio`), and has no DSP pipeline, outputs or inputs.

## 1. Build

```sh
cmake -S firmware -B build-sender -G Ninja -DPICO_PLATFORM=rp2350-arm-s -DPICO_BOARD=pico2 -DDSPI_ROLE_SENDER=ON
cmake --build build-sender            # build-sender/DSPi-Sender/dspi_sender-rp2350.uf2
```

`-DDSPI_SENDER_BENCH=ON` builds `dspi_sender_bench-rp2350.uf2`. It is the same firmware with bench defaults: link 0
(GP2) sends the PRBS-7 test pattern from power-up and the SYNC pulse is on GP21. A link bench RX board can then check it
with no host software on the sender's side (`Documentation/Bench/link_sender_bench.md`). `DSPI_ROLE_SENDER` is a
top-level option: with it OFF (the default) the tree builds DSPi exactly as before. The whole image runs from RAM
(`copy_to_ram`).

## 2. USB

The device is DSPi's composite device (VID/PID `2E8B:FEAA`, product "DSPi Link Sender", `bcdDevice 0x0280`): UAC1
audio on interfaces 0/1, the vendor interface 2, no MIDI. Alternates (SYS2 4.1, Full Speed, one packet per ms, one
jitter frame reserved):

| Alt | Format | Rate | Bytes per packet |
|---|---|---|---|
| 1 / 2 | 2 ch 16 / 24-bit | 44.1, 48, 96 kHz (SET_CUR) | up to 582 |
| 3 / 4 / 5 | 4 / 6 / 8 ch 16-bit | 48 kHz | up to 784 |
| 6 | 4 ch 24-bit | 48 kHz | 588 |
| **7** | **6 ch 24-bit** | 48 kHz | 882 (wMaxPacketSize 884) |
| **8** | **4 ch 16-bit** | **96 kHz** | 776 |

Alts 7 and 8 exist only in the sender build. USB channel *i* goes to link slot *i*. Slots from the stream's channel
count up to C are sent as zeros. The host's volume and mute (the UAC1 feature unit, -96..0 dB) are applied to the
samples.

**The sender is the clock master.** DSPi's feedback controller (`usb_feedback_controller.c`) runs unchanged, with one
substitution: the "output words consumed" it measures are the link frames on the wire (one per sample, `rate_shift`
14), and its fill is the sender's sample FIFO, held at two packets. While the encoder is not playing USB samples
(another test pattern, a different rate, or no stream), the host is told the nominal rate.

## 3. Pins (the v1 carrier, SYS2 6.3)

| Link | GPIO | Drive | SFP TX_DISABLE / LOS |
|---|---|---|---|
| 0 / 1 | GP2 / GP3 | 4 mA | - |
| 2 / 3 | GP4 / GP5 | 12 mA (Broadcom AFBR-1624Z) | - |
| 4 | GP6 (SFP 1 TD+) | 4 mA | GP10 / GP12 |
| 5 | GP8 (SFP 2 TD+) | 4 mA | GP11 / GP13 |
| 6 / 7 | GP14 / GP15 | 4 mA | - |

SYNC is on any free header pin (`EXT_LINK_TX.sync_pin`; GP21 on the carrier). The fan-out input defaults to GP20
(`EXT_LINK_CONFIG.rx_pin` `0xFF`). The LED (GP25) is on while the links run. It blinks fast while USB audio plays, and
slowly while a fan-out is re-sending. A disabled link's pin is a low output. TX_DISABLE pins start high (off).

## 4. How it sends

- **One encoder, one ring, N state machines.** Core 1 encodes each frame once into a 16 KB word ring, about 0.5 ms
  ahead of the wire. Links 0-3 run LK2's one-instruction `link_tx_N` on PIO0 SM0-3, and links 4-7 on PIO1 SM0-3. Each
  has its own DMA channel reading the same ring (ring-wrapped read, endless transfer count). All 8 links and the SYNC
  machine (PIO2 SM1) start on one cycle with `pio_enable_sm_multi_mask_in_sync()`. A disabled link keeps its machine
  running, so that it stays in step, and only its GPIO function changes.
- **The encoder** (`sender_link.c`) produces exactly the bits of LK1's reference `link_tx_frame()` with the same state
  (`link_tx_t`). It is table driven, and the host tests prove it identical over 270 streams with applies, flag changes and
  RATE switches. Cost per frame, once for all links, measured on GCC's Cortex-M33 code with LK5's interpreter (the
  same code on the host gives the same ring hash), as cycles of the frame's budget at 307.2 MHz. Model A is
  optimistic and model B conservative:

  | Class, rate, C | Cycles per frame (A-B) | Budget | Share of core 1 |
  |---|---|---|---|
  | L1 48 kHz 8 | 950-1290 | 6400 | 15-20 % |
  | L1 96 kHz 3 | 630-880 | 3200 | 20-28 % |
  | L2 48 kHz 16 / 18 | 1580-2070 / 1710-2240 | 6400 | 25-35 % |
  | L3 48 kHz 32 / 38 | 2830-3630 / 3110-4010 | 6400 | 44-63 % |
  | L3 96 kHz 18 | 1710-2240 | 3200 | 53-70 % |
  | L3 44.1 kHz 41 | 3740-4890 | 6966 | 54-70 % |

  The frame that starts a superframe costs about 1600 more (the superframe CRC). The ring lead absorbs it. The
  firmware measures its own cost with the DWT cycle counter (`EXT_LINK_DIAG` offsets 4-12).
- **Test patterns** (`EXT_LINK_TX.test_pattern`): 0 USB audio, 1 PRBS-7 payload (superframe flag b4, the bench BER
  pattern), 2 idles only (no commas: receivers lose lock), 3 a 1 kHz tone at -20 dBFS on every slot.
- **SYNC** goes high on the first bit of the comma of every frame with F mod Fs = 0 (1 pps), and of the target frame of
  a `SYNC_PULSE` apply, for 10 us. It is a link-like stream from its own ring in step with the links, so the edge sits
  on the comma to the bit.
- **Superframe flags.** b0 = `EXT_LINK_TX.flags` bit 0 or `EXT_LINK_CTRL_MSG.system_flags` (system mute). b3 is set
  while USB samples are playing. b4 is set with the PRBS pattern. b1/b2 come from the pending apply.

## 5. Layout changes and applies (`extension_channel_spec.md` 2.8.8-2.8.9)

- **Rate or C** (`EXT_LINK_TX`, or the host switching the USB rate): while any link is enabled, the sender schedules
  apply op **RATE at the first multiple of 32 at least 2400 frames after the next frame it encodes**, with `value` = the
  new C (0 if unchanged). A further rate or C change is refused while that apply is pending. Receivers switch at that
  frame. A USB rate change is followed the same way, with C capped to the class's capacity at the new rate; if an
  apply is pending it is retried until the apply has ended. With no link enabled the change takes effect at once.
- **`EXT_LINK_CTRL_MSG`** OUT schedules one apply for every unit with `link_tx_schedule()`'s rules: lead of a whole
  superframe after the next one the encoder starts, RATE on a multiple of 32 with value 0 and a layout that fits, one
  at a time. The IN form shows the pending apply at once and 0 once F has reached it.
- **Class** (`EXT_LINK_TX.class`): the links restart at the new line rate (stop, reload the programs, start in sync).
  F continues. Receivers lose lock and re-detect.
- "The next frame the encoder writes" leads the wire by the ring lead (about 0.5 ms). `EXT_FRAME_COUNTER` and
  `EXT_LINK_STATUS.frame` report the frame on the wire.

## 6. Fan-out (slaved) mode

`EXT_LINK_CONFIG.enable` = 1 makes the sender a fan-out (`EXT_LINK_INFO.role` 4). It receives a link on GP20 (or
`rx_pin`) with `link_rx_N` on PIO2 SM0 into a 32 KB ring, detects the class (L1, then L2, then L3, unless forced), and
acquires with LK1's reference receiver. It then follows the frames with a word-wise parser that applies the reference's
flywheel, slip and loss rules, and re-sends each frame on all outputs at the class of `EXT_LINK_TX.class`
(`sender_fanout.c`):

- The re-sent stream has the same F, payload and layout, hops + 1, and the outgoing class. It is re-encoded with the
  fan-out's own running disparity and padding. Idles are sent until a valid superframe is held, and again after a loss.
- The superframe is re-sent byte by byte as it arrives. A byte from a bad head is replaced and the re-sent CRC is made
  wrong. So is the CRC when the received CRC fails. A bank with a non-data symbol goes out with a failing CRC.
  Corruption is never laundered into a valid frame.
- **The delay is `delay_frames` whole frames** (2 at 44.1/48 kHz, 2-3 at 96 kHz, `EXT_LINK_DIAG` byte 3), from the
  arrival of a frame's comma to its re-sent comma, to within about one 32-bit word of each class. The soft VCXO holds
  it: a feed-forward of the input bit rate measured against the crystal timer, plus a PI term on the delay error
  (`EXT_LINK_STATUS.phase_error_frames`). Past half a frame of error the fan-out goes idle and re-aligns.
- USB audio is ignored in fan-out mode (the device stays enumerated, the feedback says nominal).
- **Capacity.** The parser and the re-encoder run on core 1. On the M33 model (cycles per frame, model A-B):

  | In -> out, rate, C | Cycles per frame | Budget | Verdict |
  |---|---|---|---|
  | L1 -> L1 48 kHz 8 | 3060-4220 | 6400 | fits |
  | L3 -> L1 48 kHz 8 | 3350-4610 | 6400 | fits |
  | L2 -> L2 48 kHz 16 | 4460-5990 | 6400 | marginal |
  | L1 -> L1 96 kHz 3 | 2320-3300 | 3200 | marginal |
  | L3 -> L3 48 kHz 16 / 32 | 5030-6630 / 7280-9540 | 6400 | marginal / too slow |

  A parser that falls half a ring behind counts a loss and acquires again (`EXT_LINK_DIAG` offsets 60-61). L2/L3 at
  full channel counts need LK5's word-wise decoder (`link_fast`) in place of this parser.

## 7. Control (vendor interface 2)

`REQ_EXTENSION` (0x00) with the sender's IDs: `EXT_NOTIFY_FEATURES` (0: the sender sends no notifications), `0x8000`
`EXT_LINK_INFO` (role 2 or 4, classes L1-L3, 8 links, 64 channels, features `FAN_OUT | TEST_PATTERNS | LOS_PIN`),
`0x8001` STATUS (the fan-out input; `frame` = F on the wire; `fill_frames` = the USB FIFO), `0x8002` CONFIG (the
fan-out input), `0x8005` FRAME_COUNTER, `0x8007` TX, `0x8008` CTRL_MSG, `0x8009` STATS_RESET, `0x800A` DIAG (the
sender layout of 7.1). Also `REQ_GET_PLATFORM` (0x7F; 0 output channels), `REQ_GET_SERIAL` (0x7E),
`REQ_GET_BUILD_INFO` (0x80), `REQ_SAVE_OUTPUT_CONFIG` (0x52: stores `EXT_LINK_TX` and `EXT_LINK_CONFIG` in their own
flash sector, below DSPi's preset area), `REQ_ENTER_BOOTLOADER` (0xF0). Every other request STALLs. Defaults are those
of 2.8.8: no link enabled, L1, 8 channels, 48 kHz, audio, no SYNC, TX_DISABLE all high.

### 7.1 `EXT_LINK_DIAG` on a sender (layout `0x81`)

| Offset | Field |
|---|---|
| 0 | `0x81` (sender layout 1) |
| 1 | engine: 0 stopped, 1 master, 2 fan-out idle, 3 fan-out re-sending |
| 2 | class the links run (0 stopped) |
| 3 | fan-out delay in frames |
| 4 / 8 | encoder cycles per frame: mean of the last 1024 frames / worst since the last read |
| 12 | cycles in one frame at the current rate (the budget) |
| 16 | ring lead in words |
| 20 / 24 | `tx_late` (refills after the DMA had read the words) / `tx_stall` (PIO TX FIFO stalls) |
| 28 | USB FIFO fill (frames) |
| 32 / 36 | USB FIFO underruns / overruns (frames dropped) |
| 40 | feedback value (10.14) |
| 44 / 48 | USB packets / packet gaps of 2-50 ms |
| 52 | f32 fan-out input rate (ppm against the crystal) |
| 56 | longest core-1 pass in us since the last read |
| 60 | u16 fan-out re-alignments and overruns |
| 62 | 1 while USB samples play |

## 8. Limits and open points

- Not run on hardware. The host tests cover the encoder (bit for bit), the scheduling and refusal rules, the
  fan-out core (through LK1's decoder, with damage), and `bench.py`'s sender scenario (simulated). The PIO/DMA start, the
  USB path, the feedback and the fan-out's clock servo are compiled and reviewed only.
- The fan-out delay is 2-3 frames, not the 1 frame of `link_protocol_spec.md` 7.2. A whole input frame must arrive
  before its re-sent copy can start, plus the FIFOs. Its hop count is still hops + 1, so units behind a fan-out need
  `EXT_LINK_PHASE.hops_offset` = -(delay_frames - 1) until the spec decides.
- No UART or USB MIDI control, no notifications, no fan-out trim scan (the receive trim stays at 0).
