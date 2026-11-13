# Alink receiver: INPUT_SOURCE_ALINK on DSPi

*Status: implemented by WP LK5 (2026-11-13), fixups LK45F after review lk45; phase lock, apply at frame and holdover
by WP LK6 (section 8). RP2350 builds with `DSPI_ALINK` (on by default). Host-tested, emulated and simulated, not yet run
on hardware.*

The receiver decodes the multi-unit alink (`alink_protocol_spec.md`) on one GPIO and feeds its 8 slots to the pipeline's
8 input channels, like the 8-channel ADAT input. The code is `firmware/DSPi/alink_input.c` (its header describes
alink-up, the clock and the decoder placement) and `firmware/DSPi/alink/` (the decoder `alink_fast`, bit-exact with the
reference receiver). It is configured and observed through the alink block of the extension channel
(`extension_channel_spec.md` 2.8). Bench wiring and steps are in `Documentation/Bench/alink_gpio_bench.md` section 7.

## 1. What plays

- The unit's 8 slots (`EXT_ALINK_SLOTS`) go to the 8 input channels, each through its input preamp. A slot at or above
  the stream's C plays silence.
- **A muted or test-pattern stream plays silence.** While the superframe's flags show system mute (bit 0) or a test
  pattern (bit 4: the payload is PRBS-7, not audio, and would reach amplifiers at full scale), the receiver stays
  locked, follows the rate, counts and reports as usual (`EXT_ALINK_STATUS.sf_flags` shows the bits), but its inputs
  play silence. Audio returns with the first superframe that clears the bit. A bench PRBS test therefore loses
  nothing, and the outputs stay quiet. Accepted by the coordinator 2026-11-13.
- A loss stops delivery at once. Phase-locked (section 8), the unit enters a muted holdover and resumes on the same
  timeline. Otherwise the main loop mutes, then goes round the drain, prefill and in-sync start again once the
  receiver re-locks, as for ADAT. A `RATE` in the stream always takes the full path around the rate change. A failed
  bank holds its last samples for one frame, then plays zeros (the protocol's rule).

## 2. Main-loop stalls

The decoder reads the RX ring in place. The ring holds 32 KB of line bits, so the decode service must run at least
once per ring pass. A longer gap restarts the decoder: a loss, then re-lock (a few ms), mute and prefill.

| Class | Ring pass | Longest service gap that rides through |
|---|---|---|
| L1 (15.36 MBd) | 17.1 ms | 16.0 ms |
| L2 (30.72 MBd) | 8.5 ms | 8.0 ms |
| L3 (61.44 MBd) | 4.3 ms | 4.0 ms |

The limit is 15/16 of a pass (`alink_ring_clock.h`): the position count, the DMA write index plus counted wraps, stays
exact while passes are less than one ring pass apart, and the bits the decoder has not read yet are overwritten soon
after that anyway, so no software change can ride through more than the ring. Before LK45F the limit was half a pass
(2.1 ms at L3). The host test `test_alink_ring_clock` checks the wrap count at every class with the line up to 1000
ppm fast, and `test_alink_fast` checks that a gap at the limit costs no frame.

**Bench expectation.** A flash write (preset save, `REQ_SAVE_OUTPUT_CONFIG`) stalls the main loop for longer than a
pass at every class (a sector erase takes tens of ms), so the alink loses lock and re-locks around every save. That is
expected, not a decoder fault: the flash write resets the pipeline for every input anyway (mute, prefill), and ADAT
re-acquires the same way. Other stalls up to the table's limits must not show as losses (`loss_count`).

## 3. Clock

The sender owns the clock. The receiver measures the wire rate from the RX DMA position against the timer and feeds
it to `input_servo_apply()` as the S/PDIF and ADAT slave inputs do: 32 ms windows smoothed, then a dual-anchor window
that is used from a 2 s span and grows to 8-16 s (WP LK6: from 2 s instead of 8 s, about 1 ppm at L1 then, where the
32 ms windows wander by several ppm). In phase lock (section 8) the servo's second term is the phase error, not the
ring fill. The position and the time it was read are published together by whoever ran the decode
service, and the measurement uses that pair, never the poll's own time. That matters for the Core-1 option (section
5), where core 1 publishes at its own moments; with the poll's time every 32 ms window was off by up to a few percent
and the servo rejected it (review lk45 finding 1). `test_alink_ring_clock` checks both placements: under 100 ppm per
fast window and under 2 ppm for the long window, where the old pairing was off by more than 14000 ppm.

## 4. Decoder speed and placement

- **In RAM:** the steady state at 48 and 96 kHz with the slots in one bank (the M33 assembly clean loop and full-bank
  routine, with the CRC-16 by the DMA sniffer), the decode service and the delivery.
- **From flash (XIP):** the per-frame decode of a **44.1 kHz** alink and of a unit whose slots span **two or more
  banks**, or decode a partial bank (C not a multiple of 8 in the unit's bank). `alink_fast.c` makes that C path cold on
  M33 builds to save RAM. Correctness is unaffected; speed on XIP cache misses is unmeasured (the emulated cycle
  figures had that code in RAM). Measure with the bench image's `decode canned L3 44k1` if 44.1 kHz matters.
- **CRC mode:** at start the receiver self-tests the DMA sniffer against the table CRC. If it fails, full banks are
  decoded by the C `bank8()` from flash: slower, still correct. Only the UART log of a `DSPI_DEBUG_PRINTF` build says
  which mode was chosen (`ALINK RX: CRC-16 by the DMA sniffer` or `... table CRC`); `EXT_ALINK_DIAG` does not.
- The gate is about 10 % of core 0 for L3, 48 kHz, one bank (review lk45: credible range 540-640 cycles of 6400).
  The bench's `decode all` decides.

## 5. Core-1 option (`DSPI_ALINK_DECODE_CORE1`)

Off by default and in every shipped build (coordinator 2026-11-13). With it, the Core 1 EQ worker runs the decode
service's clean frames while it waits for work, and core 0 takes the decoder back for anything else. LK45F made it
safe to switch on when it is needed: the rate measurement uses the published pairs (section 3), and a long gap seen
by core 1 is left for core 0 to handle (core 0 restarts the decoder). It builds and passes `check_ram_placement`; it
has never run on hardware.

## 6. RAM

The alink's RAM-resident set is about 5.4 KB. `.data` against the budget of 100352 B in `check_ram_placement.py`
(after LK45F):

| Build | `.data` after LK45F | after LK6 | Headroom |
|---|---|---|---|
| RP2350 default | 98776 | 99104 | 1248 |
| RP2350 K340 (`DSPI_USB_MIDI`, 1.20 V) | 98896 | 99224 | 1128 |
| RP2350 `DSPI_ALINK_DECODE_CORE1` | 99256 | 99656 | 696 |
| RP2350 `DSPI_ALINK=OFF` | 93456 | 93480 | 6872 |

The budget stays as it is (coordinator 2026-11-13). So later alink work must fit about 1.5 KB of hot code on the
default and K340 builds:

- **LK6** (phase lock) took 328 B (296, plus 32 for the relock hold of the fixup after review lk6): the decode service's published reference (104 B), the CLICK injection in the
  pipeline's three finalize paths (about 100 B), the servo hook and a veneer. The phase servo, the anchor, the holdover
  and the apply executor run from flash (`check_ram_placement.py` whitelists them as `ALINK_COLD`), and the two alink
  event serialisers moved to flash too (`notify_push_alink_state`, `notify_push_apply_result`: 134 B back).
  `EXT_ALINK_PHASE` is stored in 6 of the V26 alink block's 7 reserved bytes (no V27).
- Note: `deliver()` (the steady-state copy into the pipeline) is inlined into `alink_input_poll()` and so runs from
  flash, as it did after LK5. `check_ram_placement.py` lists `deliver` as an optional root and warns that it is
  absent.
- **LK8** (repeater): a per-frame re-encoder in RAM (the sender's `snd_frame` is about 2-3 KB with its glue) does not
  fit. It must run from flash or live in a separate image; LK8 decides which.

## 7. Loss reasons

`0xE1` reason `COMMA_LOST` (4) means the frames that failed since the last poll that saw the decoder locked were all
flywheel frames (comma missing, no bad head); anything else is `FRAME_ERRORS` (3). Before LK45F the counters since
lock were used, so one head error early on made every later loss read `FRAME_ERRORS`.

## 8. Phase lock, apply at frame, holdover (WP LK6)

The code is `firmware/DSPi/alink/alink_phase.[ch]` (portable C, the same code in the host simulation) and its glue in
`alink_input.c`. Control: `EXT_ALINK_PHASE`, `EXT_FRAME_COUNTER`, `EXT_APPLY_AT_FRAME`, event `0xE2`
(`extension_channel_spec.md` 2.8.5-2.8.7, 2.8.12).

**The target.** In mode 1 (the default) alink frame F leaves the outputs exactly `D = d_local + hops_offset` frames after
its comma arrived, so units with the same D play the same frame at the same moment. Mode 0 is LK5's behaviour: the
fill servo, whatever latency the prefill left.

**The measurement.** The receiver writes the decoded frames into the output rings in F order. `next_w` is the F of the
next frame written, and `fill` is how many written frames have not played yet, so the frame playing now is
`next_w - fill`. The phase error is `e = F_arr - (next_w - fill) - D`, where F_arr, the frame arriving now, comes from
the RX DMA position against the decoder's last reference (frame F, comma end at bit p, published with the ring
position). Both positions are DMA counters read together with interrupts off. So e is exact to about a tenth of a frame
(one 32-bit RX word, a quarter-frame output DMA word), and it needs no model of the decode, FIFO or batch delays.
Anything that moves the audio moves `next_w` or `fill`: a block refused by the rings, an underrun's skip, inserted
silence. e shows it.

**The anchor** (SYS2 3.1, 3.3):

1. Lock (class, trim scan, superframe), then the VCXO pull-in. The drain waits until the long rate window is valid
   (a 2 s span) and the VCXO is within 1 ppm of that feed-forward (`alink_input_rate_ready()`), so the anchor neither
   drifts away while the VCXO still slews at 6.7 ppm/s nor gets pushed off when the 32 ms windows hand over to the long
   one. Meanwhile rows are dropped, so the decoder stays current. This takes 2-4 s from a small crystal offset and up
   to 10 s from 50 ppm, and the outputs stay muted. After a holdover at the same rate the VCXO is still where it was:
   no pull-in wait (review lk6 F4).
2. Drain, then prefill the rings to `D - block_cap - 16` frames (the latency option's target when that is larger).
   `output_latency_apply()` asks `alink_input_ring_target()` for the ALINK source.
3. Start the outputs when `F_arr - first = D` (first = the first frame written). The last millisecond is waited in
   place (`alink_input_anchor_ready()`), so the start lands to a fraction of a frame.
4. Verify, still muted: within half a frame the soft mute lapses and the audio fades in. A late start (a long
   main-loop pass) is corrected by whole frames, dropping rows or inserting silence, at most twice; otherwise it
   re-anchors.

**The servo**, once per 75 ms tick: `ppm = feed-forward + PI(e)` (`input_servo_apply_trim()`, slew-limited like the
fill servo). KP 4 ppm per frame of smoothed error (a time constant of about 5 s at 48 kHz, 2.6 s at 96), KI 0.01 ppm
per tick per frame, the IIR 0.3 s. The host test's plant model settles a 0.45-frame error without overshoot; the
VCXO's 0.11 ppm step leaves a limit cycle under 0.05 frame. Phase-locked, an error beyond 2 frames (an underrun's skip,
for example) re-anchors (`0xE1` reason `REANCHOR`). With `EXT_ALINK_CONFIG.flags` bit 1 clear, PHASE.state becomes 3
(pending) instead, and the servo pulls the phase in while the audio plays.

**`d_local_min` / `d_local_max`** (`EXT_ALINK_INFO` 16-19) follow the latency option and the rate: min = ring target +
block cap + 16 (SYS2 3.2), max = the rings' writable span (448 frames) - block cap. At 48 kHz: 1 ms option 81..431,
4 ms option 257..399. At 96 kHz the 1 ms option gives 145..415, so D 128 needs a larger value there. A write outside
the range is refused. If the option or the rate moves the range under the stored delay, the unit runs in mode-0
behaviour and PHASE flags `D_BELOW_MIN` (or, above the maximum, just state 0).

**Loss and holdover** (SYS2 3.3). A loss while phase-locked does not reset the pipeline:

- state 3 (holdover), `0xE1` with reason `FRAME_ERRORS` or `COMMA_LOST`, soft mute. The VCXO trim is held (no servo
  tick without a lock).
- The outputs keep running: silence keeps the rings at their fill, so the timeline (`next_w`) advances at the held
  rate. `EXT_ALINK_STATUS.frame` is extrapolated.
- On relock the stream's F is compared with the holdover's extrapolation. Within the rings' writable span (448
  frames) at the same rate, the first rows are placed by F on the held timeline: late rows are dropped, a gap is
  filled with silence. Then the verification runs. Within half a frame the audio fades in again with no re-anchor
  (the simulation resumes after 2 s and 60 s dark lines); otherwise it corrects whole frames (the correction's silence
  delays the stream without moving `next_w`) or re-anchors. Further off in either direction (a sender that was power-
  cycled restarts F at 0, another stream) or at a new rate it takes the full reset at once, and a placement gap beyond
  the span during the resync re-anchors too (review lk6 F1: before, F behind dropped rows for as long as the unit had
  been running).
- For 2 s after a relock (until the long rate window is valid again) the VCXO keeps its held trim: the 32 ms windows
  would drag it by several ppm.
- Pending apply ops still act on their frames during the holdover (on silence). When the receiver stops, they are
  cancelled (`0xE2` result 2).

**Apply at frame.** Both paths feed one 4-deep queue in frame order: `EXT_APPLY_AT_FRAME` (per unit) and the apply
that valid superframes carry (taken once; `RATE` stays the decoder's). An op acts on the samples of its frame as they
enter the pipeline. The delivery splits the block so that the frame is the first sample the pipeline processes after
the op. MUTE/UNMUTE (`user_mute`), MASTER_VOLUME, OUT_DELAY (`channel_delays_ms`) and PRESET_LOAD (the main loop's
deferred load) start there. CLICK sets a one-sample impulse at -20 dBFS (`PIPELINE_CLICK_LEVEL` 0.1) on that output,
injected after delay and limiter in the finalize step of the core that owns the output; a muted unit (`user_mute`, or
a system-muted stream) plays none and reports `refused` (coordinator's decision after review lk6). An op whose frame has already been written runs at once
and reports `late` (1). On a receiver, SYNC_PULSE reports `refused` (3): `EXT_ALINK_CONFIG` has no sync GPIO. So does
CLICK on the PDM sub (output 8), which is not finalized as PCM. Every result sends `0xE2` with the op's target frame.
`CANCEL_ALL` reports each pending op as `cancelled` (2).

**Persistence.** `EXT_ALINK_PHASE` (mode, d_local, hops_offset) is saved by `REQ_SAVE_OUTPUT_CONFIG` in the V26 alink
block (`phase_version` 1 plus 5 bytes in its reserved area); older saves read as the factory values (1, 128, 0).

**Verification on the host** (WP LK6):

- `firmware/tests/test_alink_phase.c` covers the validation, the limits, the anchor, verification and re-anchor
  decisions, holdover placement, the servo against a plant model at 44.1/48/96 kHz, and the apply queue with the 2.8.7
  examples.
- `tools/alink_phase_sim/run.sh` runs two receivers fed by LK4's encoder: `alink_fast`, `alink_phase` and a line-for-line
  copy of the glue, with a modelled line, crystals, VCXO and rings. It checks identical alignment (skew under 0.05
  frame) with different boot times, clock offsets, a 1 ms and a 2 s loss, and L3 at 96 kHz, and contrasts mode 0
  (3.7 frames apart).
- `run.sh pio` checks the simulation's line against LK2's PIO model.
- The bench procedure is `Documentation/Bench/alink_phase_bench.md`.
