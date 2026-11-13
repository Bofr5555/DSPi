# alink USB-direct units: SOF lock, USB time base, phase lock over USB

*Status: implemented by WP USB1 (2026-11-13) on RP2040 and RP2350 (`DSPI_ALINK_USB`, on by default; run-time opt-in,
off by default). Host-tested and simulated, not yet run on hardware. Control plane: `extension_channel_spec.md` 2.9.
Study: USBL1 (`~/.cache/dspi-build/work/usbl1/REPORT.md`) sections 2, 5, 7.2. Bench: `Documentation/Bench/usb_direct_bench.md`.*

A DSPi on the USB input normally runs from its own crystal and tells the host, through the asynchronous feedback
endpoint, how fast to send. As an alink USB-direct unit it does the opposite: it locks its soft VCXO to the host's
Start of Frame (SOF), which every device on the bus sees at the same instant, and in phase mode it plays every USB
sample a fixed D samples after the sample arrived. Units on one host then play sample-aligned, exactly like phase-locked
audio-link receivers (`link_receiver.md` section 8), with the USB bus as the link.

| Item of USBL1 section 5 | Where | RP2040 | RP2350 |
|---|---|---|---|
| U1 SOF-locked clock (`EXT_ALINK_USB` mode 1) | `alink_usb_core.c` service, `input_servo` | yes | yes |
| U3 USB-frame time base | `alink_usb_core.h` (`alink_usb_tb_*`), `alink_usb.c` SOF hook, `usb_audio.c` stamp hook | ISR entry time (1-4 us) | `SOF_TIMESTAMP_RAW - _LAST` against the 1 us timer |
| U4 phase lock over USB (mode 2) | `alink_usb_core.c`, `link/link_phase.c`, the anchor in `main.c` | yes | yes |
| U5 apply at frame on the USB time base | `EXT_APPLY_AT_FRAME` time base 1/2 | yes, except CLICK | yes |
| U5 SYNC_PULSE GPIO at that SOF | bench builds `-DDSPI_ALINK_USB_GPIO=<pin>` | yes | yes |
| U6 start-offset report | `EXT_ALINK_USB_STATUS` 32-47 | yes | yes |
| U2 synchronous descriptors | not in this WP: the descriptor stays asynchronous and reports nominal feedback (USBL1 2.5 v0) | | |

RP2350-only: CLICK (the RP2040's fixed-point pipeline has no click injection) and the hardware SOF timestamp.

## 1. Modes

- **Mode 0 (default)**: stock. The SOF hook still counts frames and stamps packets (a few tens of cycles per SOF and
  per packet), nothing else runs: asynchronous feedback, crystal clock, packets on the stock RAM path, no events.
  `firmware/tests/test_alink_usb.c` (`test_mode0_stock`) checks that the engine never actuates, writes, mutes or sends
  in mode 0, and a `-DDSPI_ALINK_USB=OFF` build has exactly the base image's RAM (99104 B `.data`).
- **Mode 1 (rate lock)**: the host's SOF rate against the unit's crystal (two clean SOF anchors 8-16 s apart, used from
  a 2 s span) is the VCXO's feed-forward. During pull-in (slew 0.5 ppm per 75 ms tick, as every input servo) the
  asynchronous feedback still keeps the ring fill; once the VCXO is within 2 ppm of the feed-forward the unit reports
  exactly nominal feedback and the fill servo (the S/PDIF slave's) holds the fill. All units of a host then run at the
  host's rate.
- **Mode 2 (phase lock)**: mode 1, then the anchor (section 3) and the timeline. Needs 48 or 96 kHz and
  `d_min <= D <= d_max` (153..399 at 48 kHz, 273..351 at 96 kHz for every latency option USB offers); otherwise it
  runs as mode 1 and says why (`status_flags`, event reason `CONFIG`).
- **Host range**: a host more than 295 ppm off the unit's crystal is rejected (state 6, event `HOST_RANGE`): VCXO parked,
  stock asynchronous behaviour, until it is back under 285 ppm.

## 2. The time base

`alink_usb_tb_sof()` runs first in `uac1_driver_sof()` (USB IRQ): it extends the 11-bit frame number to a 32-bit count
N (N mod 2048 = the bus frame), takes the SOF instant (RP2350: `now - (SOF_TIMESTAMP_RAW - SOF_TIMESTAMP_LAST)`, exact to the PHY's 21 ns but taken against `time_us_32()`, so each instant carries up to 1 us of read quantisation that the servo averages away;
RP2040: the interrupt entry, or the clean pair's prediction when the entry was late) and keeps the latest *clean* pair
(N, t): RP2350 when fewer than 33 SOFs were merged (the PHY counter has not wrapped), RP2040 when the interval was 1 ms
within 3 us. Time is in uint32 ticks of 1/48 us, so the ISR needs no 64-bit arithmetic. A gap of over 1 s is flagged
and ends the timeline.

Packets are stamped in the interrupt that completes them (`tud_event_hook_cb()` in `usb_audio.c`, TinyUSB's per-event
hook): TinyUSB runs `xfer_cb` later from `tud_task()`, up to a main-loop pass after the packet, so the stamp cannot be
taken there. The OUT buffer is FULL from its completion until the next arm, which happens with interrupts off and a
"waiting" flag set, so a FULL buffer seen while waiting is this packet. A completion less than 100 us after the latest
SOF belongs to the frame before (a packet of 44 or more samples takes at least 117 us on the Full Speed wire, so it
cannot be that frame's own): the interrupt was held off across the SOF. The stamp travels with the packet in the USB
ring slot (`usb_audio_slot_t.frame`).

`S = spf * N + k` (spf 48 or 96). `F_arr` (the sample arriving now) comes from the clean pair extrapolated at the
measured host rate.

## 3. Anchor and timeline (mode 2)

1. Pull-in done (`rate_locked`): the main loop's USB block (`input_service_handshakes()`) mutes
   (`prepare_pipeline_reset`), waits for the DAC mute hold, drains the outputs and calls `alink_usb_on_drain()`, which
   sets the prefill target to `D - block_cap - 16` (`output_latency_apply()` asks `alink_usb_fw_ring_target()`) and
   starts the timeline at the current S.
2. The service prefills the stopped rings with **silence** (packets meanwhile are dropped, muted anyway).
3. `alink_usb_fw_anchor_ready()` waits the last millisecond in place until `F_arr - first = D`; the outputs start in
   sync; `alink_usb_on_enabled()`.
4. Verification every pass while muted (soft mute refreshed): within 0.5 sample the mute lapses and the audio fades in;
   otherwise whole-sample corrections (skip by advancing `next_w`, or insert silence without advancing it: review lk6
   F2), at most two, then a re-anchor.

From then on the outputs play the **timeline**. Every packet is placed by its S: a gap is filled with silence (a missing
packet; counted, muted 20 ms, event `GAP`), late samples are dropped, a placement more than the rings' span off
re-anchors. While no packet comes the service writes silence so the rings never run more than `D - block_cap - 16`
behind the arrival. The phase servo runs on the timeline whether or not a stream is playing, so a stream that stops and
starts again (alt 0 and back, a PipeWire suspend) is placed exactly: **constant latency across stream restarts, no
re-anchor, no pipeline reset** (the stock alt-setting resync is skipped while the timeline runs). The first packet of a
stream is muted and the pipeline's 8 ms fade-in follows.

Re-anchors: a phase error beyond 2 samples (`AUTO_REANCHOR`; otherwise phase state 3 and the servo pulls it in), a new D
or mode, any other reset of the output rings (`complete_pipeline_reset()`: preset load, flash write, output type
switch, rate change), and a time-base jump (SOF lost for 50 ms or a gap over 1 s: the estimator restarts, the held VCXO
makes the next pull-in immediate, so it re-anchors about 2.1 s after the bus returns).

The servo, once per 75 ms: `input_servo_apply_trim(host rate, PI(e))` with `e = F_arr - (next_w - fill) - D` and the
link's gains (`LINK_PHASE_KP_PPM` etc., `link_phase_servo()`). D is in samples, so the link's thresholds (0.5 to verify,
2 to re-anchor) are samples here.

## 4. Apply at frame, CLICK, SYNC_PULSE

The link's queue (`link_apply_*`), fed by `EXT_APPLY_AT_FRAME` with time base 1 (S) or 2 (bus frame + sample, next
occurrence). Ops run as the timeline reaches their sample, on packets and on silence alike; a packet is split so the op
acts on exactly its sample. CLICK is -20 dBFS, refused on a muted unit (user mute or the host's UAC mute) and on the RP2040.
SYNC_PULSE drives the bench GPIO high for one frame at the op's SOF (it fires D before the sample is heard).

## 5. Start offsets (U6)

A host that opens the units' streams one after the other puts the same content into different frames. Each unit
reports the bus frame of its stream's first packet, the frames since, and `stream_shift` (silence inserted minus late
samples dropped); the hub delays the earlier units by the difference (`extension_channel_spec.md` 2.9.3;
`tools/alink_usb/alink_usb_status.py a.bin b.bin` computes it). A custom sender that schedules frame S/spf for sample S on
every unit (usbfs `start_frame`, USBL1 6.2 model B) needs none.

## 6. RAM and placement

RAM-resident: the SOF hook (152 B RP2350 / 228 B RP2040), the packet stamp in `tud_event_hook_cb` (84 / 92 B), the
stamp in `xfer_cb` and the ring slot (+52), the drain loop's branch. Everything else is flash: the engine
(`alink_usb_core.c`), the per-packet placement (`alink_usb_fw_deliver`, whitelisted in `check_ram_placement.py` as a cold
callee of `usb_audio_drain_ring()`: it runs only while a phase timeline runs) and `notify_push_alink_usb` (linker
scripts). `usb_audio_decode_packet()` is the old decode half of `process_audio_packet()`, now callable on its own (same
RAM). Figures per build: WP USB1 report.

## 7. Verification on the host

- `firmware/tests/test_alink_usb.c` (in `firmware/tests/run.sh`): frame extension across the wrap and merged SOFs,
  clean pairs, packet stamps across a held-off SOF, the estimator with RP2040 jitter and late entries at 0, +-100,
  +300, -400 ppm and with exact timestamps, `EXT_ALINK_USB` validation and refusals, persistence, limits, ring target,
  mode 0 inert, host rejection at +290 (accepted), +300 and -400 ppm, the bus-frame time base, the queue, `CANCEL_ALL`
  and the status layout.
- `tools/alink_usb_sim/run.sh`: two units on one host with the real engine and link_phase code, modelled SOF and hub
  jitter, ISR latency, packet completion and re-arm, main-loop passes, USB ring, VCXO, rings and asynchronous feedback.
  Scenarios and figures in the WP USB1 report and `usb_direct_bench.md` section 5.
