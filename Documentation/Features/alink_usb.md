# alink USB-direct units: SOF lock, USB time base, phase lock over USB

*Status: implemented by WP USB1 (2026-11-13) on RP2040 and RP2350 (`DSPI_ALINK_USB`, on by default; run-time opt-in,
off by default). Host-tested and simulated, not yet run on hardware. Control plane: `extension_channel_spec.md` 2.9.
Study: USBL1 (`~/.cache/dspi-build/work/usbl1/REPORT.md`) sections 2, 5, 7.2. Bench: `Documentation/Bench/usb_direct_bench.md`.*

A DSPi on the USB input normally runs from its own crystal and tells the host, through the asynchronous feedback
endpoint, how fast to send. As an alink USB-direct unit it does the opposite: it locks its soft VCXO to the host's
Start of Frame (SOF), which every device on the bus sees at the same instant, and in phase mode it plays every USB
sample a fixed D samples after the sample arrived. Units on one host then play sample-aligned, exactly like phase-locked
audio-link receivers (`alink_receiver.md` section 8), with the USB bus as the alink.

| Item of USBL1 section 5 | Where | RP2040 | RP2350 |
|---|---|---|---|
| U1 SOF-locked clock (`EXT_ALINK_USB` mode 1) | `alink_usb_core.c` service, `input_servo` | yes | yes |
| U3 USB-frame time base | `alink_usb_core.h` (`alink_usb_tb_*`), `alink_usb.c` SOF hook, `usb_audio.c` stamp hook | ISR entry time (1-4 us) | `SOF_TIMESTAMP_RAW - _LAST` against the 1 us timer |
| U4 phase lock over USB (mode 2) | `alink_usb_core.c`, `alink/alink_phase.c`, the anchor in `main.c` | yes | yes |
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
  a 2 s span; before that a quick block estimate from 0.5 s, section 3a) is the VCXO's feed-forward. During pull-in
  (slew 0.5 ppm per 75 ms tick, as every input servo; mode 2's muted pull-in is faster, section 3a) the
  asynchronous feedback still keeps the ring fill; once the VCXO is within 2 ppm of the feed-forward the unit reports
  exactly nominal feedback and, while a stream plays, the fill servo (the S/PDIF slave's) holds the fill. With no
  stream (alt 0: the ring is empty) the VCXO stays on the feed-forward alone: the fill term would pull it off by the
  empty ring's error (hardware finding H1, round 3: -16 ppm under the default 4 ms option, 2 ppm per buffer plus the
  second slope beyond 2 buffers). The same holds while a stream is stalled with the alt still set (a paused
  application, a host xrun): the fill servo runs only while packets arrive (`usb_audio_stream_active()`, review usb1
  N2). All units of a host then run at the host's rate.
- **Mode 2 (phase lock)**: mode 1, then the anchor (section 3) and the timeline. Needs 48 or 96 kHz and
  `d_min <= D <= d_max` (170..399 at 48 kHz, 324..351 at 96 kHz for every latency option USB offers); otherwise it
  runs as mode 1 and says why (`status_flags`, event reason `CONFIG`). The anchor needs no stream: it prefills with
  silence, so a unit locks on the SOF alone and a later stream is placed on its timeline (round 3: before, the fill
  servo's pull on an empty ring kept `rate_ready()` false and the unit stayed `anchoring` until a stream came, H2).
- **Host range**: a host more than 295 ppm off the unit's crystal is rejected (state 6, event `HOST_RANGE`): VCXO parked,
  stock asynchronous behaviour and the stock ring target, until it is back under 285 ppm. Without a time base (no SOF)
  the stock ring target applies too (review R5).

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

**Stamp limitation, on both chips** (review usb1 F5): the PHY timestamps the SOF, not the packet. A completion whose
interrupt runs more than 100 us after the *next* SOF (interrupts held off that long: flash writes, the pipeline reset)
is stamped one frame late, on the RP2350 as on the RP2040. The packet is then placed a frame late: it is late against
the timeline (dropped) and its frame reads as lost (faded hole, section 3). Hold-offs that long come with operations that
re-anchor anyway; the simulation's probe P8c (400 us hold-offs every 100 ms) mis-stamps a few packets in 400.

**RP2040 note (review R6):** after interrupts were off for more than 16 frames (a flash write) the RP2040's SOF time
is the late interrupt entry, with no clean-pair prediction, and can read as a discontinuity: a spurious `SOF_LOST`. Such
periods reset the pipeline and re-anchor anyway.

**SOF absences: holdover (WP USB2).** The K340's hour test (dab444d) showed 8 fresh anchors in a loaded half hour
with no host suspend: each one a time-base loss (no SOF for 50 ms as the unit saw it), and with music each would have
been 2-4 s of unaligned playback and then a dropout. Now a unit with a rate estimate that is not mid-anchor keeps its
timeline through a SOF absence: `hold()` runs on the arrival extrapolated from the last clean pair at the measured host
rate, the VCXO holds, STATUS flags bit 4 and `0xE3` `HOLDOVER`. On the return the frames counted since the last SOF must
match the time at the host rate (modulo 2048-frame wraps, which are restored in the ISR's count; the clean pair only if
it was taken after the return): within the F2 limits it continues (`RESUMED`, no anchor, estimator kept: it accepts
gaps up to 12 s), otherwise, or after 10 s, the time base ends as before. All of it in the main loop (flash).

**Consistent SOF reads (WP USB2).** The SOF interrupt reads the RP2350's `SOF_TIMESTAMP_LAST` and `_RAW` first and then
the frame number again; if a SOF landed after the DCD read the frame number at the IRQ's entry (an entry about a frame
late), it takes the new SOF whole. Before, such an entry labelled frame N with SOF N+1's time (a 1 ms "discontinuity")
or read `RAW` before `LAST` moved (a 43.7 ms one), either of which ended the time base: a candidate for the hour test's
anchors. The RP2040 reads the frame number right after the time instead (a retry would cost RAM).

**`EXT_ALINK_USB_DIAG` (0x8012)** counts timeline ends by cause, USB bus events (suspend, resume, bus reset, unplug:
counted in `tud_event_hook_cb`, which TinyUSB calls for every queued event), the last absence's frames, time and
deviation, the longest main-loop pass, the longest time without a SOF interrupt entry and the latest SOF interrupt
entry. Spec 2.9.6 says how to read it.

**Bad RP2350 readings** (WP USB3b). `SOF_TIMESTAMP_RAW` and `_LAST` are 21-bit counters at 48 MHz (wrap 43.69 ms);
the ISR's difference is masked to 21 bits, and since LAST is the latest SOF the true value is under a frame plus the
interrupt latency. The K340's second bench hour read RAW 22.7 us *behind* LAST four times: the difference wrapped to
2^21 - 1088 ticks (43.67 ms), the SOF's time was 43.7 ms early, and each one ended the time base (discontinuity) and
made a fresh anchor (DIAG: `discontinuity 4`, `last absence 1 frames 44668 us, clock - frames +43668 us`, `max SOF
interrupt latency 43668 us`). Why the PHY register lags is not documented (no erratum found); the firmware does not
need to know: `alink_usb_service()` repairs a SOF whose time lies more than 1.37 ms before the interrupt entry that
took it (the entry time instead, the clean pair kept out of the estimator, DIAG `bad_sof_readings`). Done in the main
loop, so the RAM ISR keeps its size (a test in the inline time-base code grew it by 20-32 B of `.data`). A packet
completing within 100 us after such a SOF, before the main loop repaired it, can still be stamped a frame off (a faded
gap at worst).

**SOF discontinuities** (review usb1 F2): a SOF whose frame count and clock disagree by more than 100 us (RP2350) or
0.5 ms (RP2040) — a stall with the frame number frozen, a small frame jump — sets the long-gap flag: the timeline ends,
the estimator reseeds, and the unit anchors again once the rate estimate is back (instead of a poisoned estimate for
16 s, a false `HOST_RANGE` or a re-anchor loop).

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

From then on the outputs play the **timeline**. Every packet is placed by its S, and the newest placed packet is
**held back one frame**: it is written when the next one arrives, or when the hold decides that none will (review usb1
F1). So a lost packet is never a hard hole: the held packet before the gap fades out over its own 1 ms, the gap is
silence, and the packet after it fades in over its 1 ms (linear ramps on the input samples, before the DSP; a packet
between two gaps fades in over its first half and out over its second). Each gap counts once (`gap_count`, event `GAP`).
Late samples are dropped; a placement more than the rings' span off re-anchors. A packet that is late rather than lost
(more than `D - block_cap - 16 - spf` samples after its SOF: 1.65 frames at D 192) also reads as a gap: the held packet
fades out, the late one fades in, a 2 ms dip and one `gap_count` (review R2). A stream stop is not a gap: hosts send the
last packet a few frames before SET_INTERFACE 0, and a gap counted within 16 frames before the alt 0 is taken back (R3).
A new stream or a format change drops a packet still held from the old one (R4).

**Stall budget (review R1):** holding a frame costs a frame of ring fill. At D 192 the fill runs 91-134 samples after
a flush and dips to 43-86 just before the hold fires, so a main-loop stall of about **1-2 ms** underruns the rings
(which re-anchors, about 0.25 s muted). The default D 192 stays; the bench's worst main-loop pass (step 4.6) decides
whether an installation needs more; do not treat `d_min` 170 as a practical setting. While no packet comes the service first
writes the held packet (faded out), then silence, so the rings never run more than `D - block_cap - 16` behind the
arrival; a loss that long (about three frames at D 192) also mutes the stream for 20 ms, after which it fades back in
through the pipeline's 8 ms envelope. The held frame is why `d_min` is `2.2 spf + block_cap + 16` (a frame held, 1.2
frames of packet lateness before the hold calls it lost, the block cap and a floor of 16). The phase servo runs on the timeline whether or not a stream is playing, so a stream that stops and
starts again (alt 0 and back, a PipeWire suspend) is placed exactly: **constant latency across stream restarts, no
re-anchor, no pipeline reset** (the stock alt-setting resync is skipped while the timeline runs). A stream's first packet
fades in; its last one fades out when the stream stops.

**Idle load.** With no stream the timeline runs on silence and the whole DSP processes it in real time: in mode 2 the CPU
load and the heat do not drop when nothing plays (the stock firmware idles the outputs without the pipeline). For the
K340 that matters for its thermal and 1.20 V margin story; mode 1 and mode 0 idle as stock.

Re-anchors: a phase error beyond 2 samples (`AUTO_REANCHOR`; otherwise phase state 3 and the servo pulls it in), a new D
or mode, any other reset of the output rings (`complete_pipeline_reset()`: preset load, flash write, output type
switch, rate change), and a time-base jump (SOF lost for 50 ms or a gap over 1 s: the estimator restarts, the held VCXO
makes the next pull-in immediate, so it re-anchors about 0.5 s after the bus returns: the quick estimate, section 3a).

The servo, once per 75 ms: `input_servo_apply_trim(host rate, PI(e))` with `e = F_arr - (next_w - fill) - D` and the
alink's gains (`ALINK_PHASE_KP_PPM` etc., `alink_phase_servo()`). D is in samples, so the alink's thresholds (0.5 to verify,
2 to re-anchor) are samples here.

## 3a. Out of step: muted, and locked quickly (WP USB3)

**Muted while out of step** (owner decision 2026-11-14). `alink_usb_out_of_step()`: mode 2, the phase lock possible
(48/96 kHz, D in range, host not rejected) and the unit not playing on its timeline (before the first anchor after
power-up, after a time-base loss, anchoring, verifying, re-anchoring). The service then refreshes the soft mute every
pass, a stream included; the audio fades in only once phase-locked. `alink_usb.c` also asserts the DAC's hardware mute
(`dac_hw_mute`) as soon as the soft mute has reached silence (only a pin nobody else holds), so the anchor's
`prepare_pipeline_reset()` finds the hold elapsed and drains at once; `complete_pipeline_reset()` keeps the pin asserted
when a USB anchor follows (`alink_usb_fw_keep_dac_mute()`), so a preset load or flash write costs one DAC hold, not two;
the anchor releases it after the start; leaving mode 2 or becoming not out of step another way (a rejected host, a
rate or D the phase lock cannot run, with the PLAY action) releases it. Not muted: modes 0/1 and a holdover (the
timeline continues).

**Lock error** (owner decision 2026-11-14, WP USB3b). `alink_usb_lock_error()`: mode 2 and the host rejected, the phase
lock impossible at this rate or D (CONFIG), or PENDING. The per-unit action `on_lock_error` (`EXT_ALINK_USB` byte 4,
persisted in the V26 alink block's spare byte, which every earlier firmware wrote 0 and ignored: 0 is the default)
is 0 *mute* (the default; the lock error counts as out of step: soft mute, DAC mute held) or 1 *play unaligned* (as
before). Reported as STATUS flags bit 6, `0xE3` reasons 10 `LOCK_ERROR` / 11 `LOCK_OK`, DIAG `lock_errors`.

**Drained outputs** (review usb3 N2). The anchor drains and disables the outputs; if the prefill then ends without a
start (mode 2 left, the host rejected or CONFIG during those few ms), `alink_usb_outputs_orphaned()` tells `main.c`,
which restarts them (`enable_outputs_in_sync()`, a 10 ms soft mute) and releases the DAC mute unless the unit is muted
out of step.

**The anchor's pipeline reset no longer blocks 200 ms** (WP USB3b, the second bench hour's 200125 us main-loop
pass). `prepare_pipeline_reset()` settles to silence by waiting, inside the pass, until the envelope reached zero and
the rings played past the delay lines, with a 200 ms cap. USB counted as streaming while `sync_started` was latched
from an earlier stream, but the envelope moves only with processed packets: an anchor with no stream (the bench's
start, before the player) or after a time-base loss waited the full cap. Since WP IDLE1 the settle itself counts the
USB source as a producer only while packets arrive (every reset, every mode; in phase mode only while the timeline is
started and the endpoint received a packet within 50 ms, see `current_architecture.md` "Preset-Switch Mute"). Before
the anchor's reset `main.c` still marks the wire silent when the out-of-step mute has held the output at zero longer
than the delay lines plus the rings (packets may still arrive then); the settle then returns at once.

**Locked quickly.** What a lock waited for, and what it waits for now:

| Step | Before | WP USB3 |
|---|---|---|
| First rate estimate | 2 s (two single SOF anchors) | 0.5 s: the means of two 250-frame blocks of clean SOF pairs (`est_quick`); the dual-anchor estimate takes over at 2 s |
| VCXO pull-in | 6.7 ppm/s from 0 | starts at the stored host rate (below) at once; else, muted and more than 4 ppm off, 26.7 ppm/s (2 ppm per tick), then 300 ms for the outputs' receivers to settle |
| After a time-base loss | estimator restart, 2 s, and a stale pre-absence clean pair could seed it (garbage rate, a reset, 2 s more) | only fresh clean pairs (under 50 ms old) seed it: 0.5 s; the VCXO held its trim |
| DAC mute hold (K340: 100 ms) | at every anchor | already elapsed (held while out of step) |
| Phase error 3-64 samples | full re-anchor: hold, drain, prefill, start, verify, release | soft: mute, 12 ms fade, whole-sample corrections with the outputs running, verify (about 15 ms) |

**The stored host rate.** `alink_usb_seed_*`: the V27 block's three reserved bytes (int16 LE in 0.01 ppm, check byte
`0xA5 ^ lo ^ hi`; zero = none, so no directory version change and no migration). The unit writes them itself
(`alink_usb_fw_seed_persist()` in the main loop's flash bracket, like `core_vreg`'s persist) once the rate is settled
over 8 s or more, no stream has played for 10 s and the rate moved 2 ppm from the stored one; at most every 10 minutes
and three times a boot. In mode 2 the write re-anchors (muted, nothing playing). At the next start with the VCXO parked
and nothing audible (mode 2, or mode 1 without a stream) the VCXO is set to it at once (`input_servo_preset()`,
STATUS flags bit 5). "Per host" in the sense the unit can know: a USB device cannot tell hosts apart before their SOF,
and 0.5 s after it the quick estimate knows the rate anyway; what the stored value buys is a VCXO that is already there
when the SOF starts. Another host costs the slew from the stored rate.

**The audio-link receiver** (`alink_input.c`) takes the muted fast pull-in too (2 ppm per tick while pulling in with
the output silent and more than 4 ppm off, 300 ms settling): start-up 2.0-2.6 s from any offset up to 50 ppm and 4.1 s
at 100 ppm (2.0-8.6 s and 14.9 s before; `tools/alink_phase_sim`). Its long rate window (2 s) and the rest of its lock
path are unchanged: a quick estimate needs its rate measurement (`alink_ring_clock.h`, shared with the sender's tests)
changed, and a stored rate needs a directory field (the V26 alink block has one reserved byte left, so a directory
version bump and a migration); both are left for a follow-up (no RAM growth either way). Its DAC hold already runs during its pull-in (the loss asserts it).

## 4. Apply at frame, CLICK, SYNC_PULSE

The alink's queue (`alink_apply_*`), fed by `EXT_APPLY_AT_FRAME` with time base 1 (S) or 2 (bus frame + sample, next
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

The held packet's copy is a `.bss` buffer of one USB packet (788 B RP2350, 582 B RP2040; not `.data`). RAM-resident: the SOF hook (152 B RP2350 / 228 B RP2040), the packet stamp in `tud_event_hook_cb` (84 / 92 B), the
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
- `tools/alink_usb_sim/run.sh`: two units on one host with the real engine and alink_phase code, modelled SOF and hub
  jitter, ISR latency, packet completion and re-arm, main-loop passes, USB ring, VCXO, rings and asynchronous feedback.
  Scenarios and figures in the WP USB1 report and `usb_direct_bench.md` section 7; since WP USB3 every mode-2 scenario
  also checks that no stream sample is heard out of step, and the L scenarios print `LOCKTIME` lines.
