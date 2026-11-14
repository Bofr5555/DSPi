# USB-direct bench: alink USB-direct units on one host (USBL1 E0, E2, E3)

*WP USB1, 2026-11-13. Not yet run on hardware. The owner's procedure for USBL1 section 7.2 steps E0, E2 and E3 and the
single-K340 case. How the units work: `Documentation/Features/alink_usb.md`; the control plane:
`extension_channel_spec.md` 2.9.*

Two DSPi units on one USB host, each SOF-locked and phase-locked with the same D, must output every USB sample at the
same instant: **skew under one sample (20.8 us at 48 kHz), target under 10 us, constant over an hour and across stream
restarts**. The host simulation predicts 0.3-0.7 us between two RP2350 units and about 2.5 us between an RP2350 and an
RP2040 (section 5).

## 1. What you need

| Item | Notes |
|---|---|
| Units A and B | two K340s, or two stock Pico 2 DSPis with the same DAC (a Pico 1 works too, without CLICK). Images in `~/.cache/dspi-build/work/wp-USB1/out/` (section 6); the owner flashes, after the usual backup |
| Hub | a High Speed hub; for 8-channel streams a multi-TT hub (`lsusb -v`: `bDeviceProtocol 2`). For E0 also a second hub and an active 5-10 m extension |
| Host | the PC (any xHCI) |
| Scope | two channels, 100 MHz or more, for E0 |
| Audio interface | two line inputs on one clock, 48 kHz 24-bit, for E2/E3 |
| Tools | `tools/midi_ctrl/run.sh` (both images have the USB MIDI control channel, so each unit is addressed by its own rawmidi node), `tools/alink/run.sh skew_xcorr.py`, `tools/alink_usb/alink_usb_status.py` (standard library; run with `python3 -I`) |

**Safety.** No audio is needed for E0. For E2/E3: CLICK is a one-sample impulse at **-20 dBFS after** the volume
control; a muted unit plays none (it reports `refused`), so keep any programme quiet by its level, not by muting.
Connect **no speaker and no amplifier input** to the outputs you record; record line level (a Pico 2 DSPi's DAC output,
or the K340's DAC test point; an amplifier output only through a dummy load and a divider). Programme material goes
through the owner's guarded player at **80 % or less** (the test-audio rules), with the guard owning the player.

Per unit, a shell helper (one rawmidi node per unit: `tools/midi_ctrl/run.sh midi_ctrl.py list`):

```sh
A=/dev/snd/midiC3D0; B=/dev/snd/midiC4D0          # from `midi_ctrl.py list`
M() { local u=$1; shift; DSPI_MIDI_DEV=$u tools/midi_ctrl/run.sh midi_ctrl.py "$@" --port $u; }
ST() { M $1 get 0x00 --wvalue 0x8011 --out out/$2.bin >/dev/null && python3 -I tools/alink_usb/alink_usb_status.py tools/midi_ctrl/out/$2.bin; }
```

(`--out out/...` lands in `tools/midi_ctrl/out/`, the container's writable directory.)

Common set-up, on **both** units (`U=$A`, then `U=$B`):

```sh
M $U set 0xE0 --u8 0                                        # input source USB
M $U ext-list                                               # ... 8005 8006 ... 8010 8011
M $U get 0x00 --wvalue 0x8010                               # EXT_ALINK_USB: 00 01 c0 00 00 00 00 00 (off, D 192)
```

## 2. E0: SOF skew on the scope (bench images)

Images `*-bench-gp22.uf2`: GP22 goes high for one frame at every SOF whose frame number is a multiple of 128
(every 128 ms, the same SOF on every unit of the bus), and at SYNC_PULSE ops. Mode 0 is enough: the marker runs
in every mode. K340: GP22 is Pico pin 29 (free on the K340; probe the module pad). Pico 2: pin 29.

1. Both units on one hub port pair; probe GP22 on A (CH1, trigger, rising) and B (CH2), ground at each unit's GND.
2. Persistence on, 1 us/div, 1000 triggers. Read the offset of B's edge to A's (mean, and the spread).
3. Repeat with B on a second hub (A on the first), then B behind the active cable, then (if available) behind an
   ADuM4160 isolator.

| Set-up | Expected | Reading |
|---|---|---|
| same hub | mean under 0.5 us; spread 1-4 us | the edge is set in the SOF interrupt, so the spread is the two units' interrupt entry (RP2350 about 1-3 us, RP2040 a little more), not the bus: the bus part is the **mean**, tens of ns |
| second hub / active cable | a fixed extra mean offset under 1 us, same spread | the hub's repeater and TT delay; fixed per installation |
| isolator | a fixed offset of a few hundred ns | |

The number every later claim rests on: the mean offset on one hub. Under 1 us passes.

The interrupt jitter does not enter the phase lock: the RP2350 takes the SOF instant from the PHY's own timestamp
(`SOF_TIMESTAMP_RAW - _LAST`, read against the 1 us timer), the RP2040 from clean interrupt entries (within 3 us of 1 ms apart), averaged by the servo.

## 3. E2: rate lock (mode 1)

1. Both units: `M $U set 0x00 --wvalue 0x8010 --payload 0101c00000000000` (mode 1, D 192).
2. Wait until both read `state rate-locked` and `nominal-feedback 1`: `ST $A a; ST $B b`. That takes 0.5 s (the first
   rate estimate) plus the pull-in at 6.7 ppm/s (mode 1 is audible, so the normal slew): about 3.5 s at a 20 ppm host,
   15 s at 100 ppm; about 0.5 s when the unit stored this host's rate before (`seeded 1`, WP USB3). `host` shows the PC's SOF rate against each unit's
   crystal (the two differ by the units' crystals); `vcxo` converges to it.
3. PipeWire combine sink to both units (`pactl load-module module-combine-sink sink_name=both slaves=<A>,<B>`), pink
   noise through the guarded player at 80 % or less, record A left and B right (`arecord -D hw:<card> -f S24_3LE -c 2
   -r 48000 e2.wav`) for 1 hour.
4. `tools/alink/run.sh skew_xcorr.py /work/rec/e2.wav --calib /work/rec/calib.wav` (calibration as in
   `alink_phase_bench.md` section 3).

Expected: the skew is **constant within the session** (per-minute values within a few us: both run at the host's rate,
so nothing drifts and PipeWire's combine sink stops resampling) but **arbitrary**: anything up to milliseconds, and
different after every restart of the streams. That is the contrast for E3 (the simulation's mode-1 case: a mean skew of 11 samples,
up to 60, between the two units).

## 4. E3: phase lock (mode 2), CLICK, restarts, start offsets

### 4.1 Lock

1. Latency option: any; USB runs the 2 ms option or longer, where D may be 170..399 at 48 kHz (324..351 at 96 kHz).
2. Both units: `M $U set 0x00 --wvalue 0x8010 --payload 0201c00000000000` (mode 2, auto re-anchor, **D 192** = 4 ms).
   `M $U set 0x52` (REQ_SAVE_OUTPUT_CONFIG) persists it.
3. Wait for `state phase-locked`, `phase locked`, `phase error` within +-0.05 sample (`ST $A a; ST $B b`): in mode 2
   the pull-in is muted and 4x faster (26.7 ppm/s, then 0.3 s settling): about 1.5 s at a 20 ppm host, 4.5 s at
   100 ppm, 12 s at 290 ppm; then a few ms of anchoring (the DAC mute is already held). `anchors 1`, `re-anchors 0`.
   No stream is needed: the units are locked on silence.

### 4.1b Muted while out of step, the stored host rate, soft re-anchors (WP USB3)

1. **Muted out of step.** With the stream already playing (guarded player, 80 % or less), write mode 2 on A
   (`0201c00000000000`): the output goes silent at once and comes back with a fade once `state phase-locked`; no
   unaligned audio in between. Repeat with a power cycle of A while the stream plays: silent from power-up until
   phase-locked (seconds on the first boot, about 0.5 s after the SOF starts once a rate is stored, below).
2. **The stored host rate.** After A has been phase-locked for over 8 s, stop the stream and wait 10 s: A writes its
   directory once (a 45 ms flash write, a muted re-anchor while nothing plays; `anchors` +1). Power-cycle A: `ST $A a`
   shows `seeded 1` and `state phase-locked` about 0.5 s after the PC enumerates it. On another PC (another rate) the
   first lock takes the slew, and A stores the new rate 10 s after it next idles.
3. **Soft re-anchor.** A phase error of 3-64 samples (rare; the simulation forces one with a ring skip) re-anchors
   without stopping the outputs: `re-anchors` +1, `anchors` unchanged, about 15 ms muted. Errors beyond 64 samples,
   a preset save or a flash write still re-anchor in full (`anchors` +1).

### 4.2 CLICK skew (no programme needed)

Record A left and B right. Read the bus frame from one unit and schedule a CLICK 500 frames ahead on both units with
the **same** 16 bytes (`EXT_APPLY_AT_FRAME` time base 2, extension_channel_spec.md 2.9.4):

```sh
ST $A a | grep 'bus frame'                       # e.g. "S now ..., bus frame 1234, ..."
F=$(( (1234 + 500) % 2048 ))
P=$(printf '%02x%02x0000000000000700000002000000' $((F & 255)) $((F >> 8)))   # frame, op 7 CLICK, output 0, time base 2
M $A set 0x00 --wvalue 0x8006 --payload $P; M $B set 0x00 --wvalue 0x8006 --payload $P
M $A get 0x00 --wvalue 0x8006                    # 00 07 00 00 <S> ...: CLICK applied (last_result 0)
```

(Output 0 is the first output; on a K340 pick the output of the DAC test point you record.) Repeat 20 times (one click
per run), then `tools/alink/run.sh skew_xcorr.py /work/rec/clicks.wav --mode click --calib /work/rec/calib.wav`.

| Measurement | Expected | Pass |
|---|---|---|
| CLICK skew, mean | within +-1 us between two RP2350 units on one hub (simulation 0.026 sample = 0.5 us; the bench adds the E0 mean and the DACs' group-delay difference); about 3 us RP2350 vs RP2040 | under 20.8 us; target under 10 us |
| CLICK skew, spread over 20 clicks | under 0.1 sample | half a sample |
| `ST`: phase error | +-0.05 sample | |

### 4.3 Programme and restarts

1. Pink noise or music to both units (combine sink), recorded for 1 hour: `skew_xcorr.py` noise mode. If the two streams
   started in different frames the content skew is a whole number of frames plus the shifts (4.4); the **per-minute
   values must be constant to well under half a sample**.
2. Stop and restart the stream 10 times (stop the player; PipeWire suspends the sinks after a few seconds, alt 0; start
   again). After each start: `ST` shows `anchors 1`, `re-anchors 0` (the timeline was kept), the same `phase error`,
   and `M $U get 0x00 --wvalue 0x8005 --out out/fc.bin` gives the same `frame - output` (the arrival-to-output latency
   is constant across restarts).
3. With CLICKs between restarts: the same CLICK skew every time.

### 4.4 Start offsets (U6)

After a stream start read both: `ST $A a; ST $B b`, then
`python3 -I tools/alink_usb/alink_usb_status.py tools/midi_ctrl/out/a.bin tools/midi_ctrl/out/b.bin` prints how many
samples one unit's content plays ahead of the other's, from `stream_first_bus_frame`, `stream_age_frames` and
`stream_shift`. Delay the earlier unit by that much (`REQ_SET_DELAY` on its outputs, or `EXT_APPLY_AT_FRAME` op 5
`OUT_DELAY` at a bus frame). Then the noise-mode skew reads **0 +- 1 sample** (USBL1 E3: "after U6 the staggered start
reads 0 +- 1 sample"). To provoke it, start the player on A, then add B to the combine sink a moment later.

### 4.5 Gaps

`missed` and `late` in `ST` count samples of missing packets (silence placed on the timeline) and late samples dropped;
`gaps` counts the holes, and also packets that came late rather than not at all (more than 1.65 frames after their SOF
at D 192: the same faded 2 ms dip); a normal stream stop is not counted. A lost packet is not a hard hole: the packet before it fades out over 1 ms and the one after it
fades in over 1 ms (the unit holds the newest packet back one frame to be able to do that); a loss of more than about three
frames is also muted for 20 ms. Each gap sends `0xE3` reason 5 (`GAP`), at most one event per 250 ms. On a quiet PC all
three stay 0 or grow very slowly; a burst says the host or the unit's main loop skipped a frame (the unit re-arms its OUT
endpoint from the main loop, so a main-loop pass over 1 ms loses the next packet). The phase is not affected.

### 4.6 One hour of `missed` / `late` (before mode 2 goes near a listener)

The loss rate decides whether mode 2 is usable: a faded 1 ms dip is inaudible once an hour and objectionable several
times a second. Measure it on the real host with the real programme path, with no listener:

1. Both units in mode 2, phase-locked, streaming programme through the guarded player at 80 % or less (or a silent
   stream: the counters do not need sound), the Console or uconsole connected as it will be in use.
2. At the start: `ST $A a0; ST $B b0`. Every 10 minutes for an hour: `ST $A a$i; ST $B b$i` (i = 1..6), noting
   `missed`, `late`, `gaps`, `anchors`, `re-anchors`.
3. Expected on a quiet PC: `gaps` 0 to a few per hour, `late` 0, `anchors` unchanged. More than one gap a minute: find
   the cause before using mode 2 (host load, USB hub, the unit's main loop: a long command, a preset save); every preset
   save or flash write re-anchors (about 0.25 s muted), by design.
4. Read and keep `EXT_ALINK_USB_DIAG` at the start (clear it: `M $U set 0x00 --wvalue 0x8012 --payload 01`), every 10
   minutes and at the end: `M $U get 0x00 --wvalue 0x8012 --out out/diag.bin` and
   `python3 -I tools/alink_usb/alink_usb_status.py --diag tools/midi_ctrl/out/diag.bin`. Expected: `holdovers` and
   `rides` equal (every SOF absence ridden through), no timeline ends but `stop`, no USB events. If `anchors` still
   rises: the `ends` cause, `last_frames` against `last_us` (equal: the host's counter ran on), the bus events (a host
   suspend or reset), and `max pass` / `max time without a SOF interrupt` / `max SOF interrupt latency` (the unit's own
   main loop or USB interrupt starved) say which. `max time without a SOF interrupt` also grows to the length of every
   SOF absence the host caused: it points at the unit only with `holdovers` 0 and no bus event (review usb2 N2).
5. Note the worst minute; it decides whether D 192 (about 1-2 ms of main-loop stall budget: the held frame costs one
   frame of ring fill) is enough for the installation. A larger D (up to 399) buys stall budget at 1 sample per sample.

## 5. One K340 alone

Mode 2 on a single unit gives constant latency across stream restarts (useful for lip sync on one unit):

1. `M $A set 0x00 --wvalue 0x8010 --payload 0201c00000000000`, wait for `state phase-locked`.
2. Play, stop, play 10 times (guarded player, 80 % or less, or no audio at all: the counters tell it):
   `ST $A a` after each start: `anchors 1`, `re-anchors 0`, `stream: first bus frame ...` changes per start,
   `shift +0` (no missing packets) and `missed 0`; `frame - output` from 0x8005 identical.
3. Start offset: `stream_first_bus_frame` is the bus frame the host put the stream's first sample into; with the unit's
   D the first sample leaves the DAC exactly `D` samples after that frame's SOF (4 ms at D 192), every time. The host's
   own latency before the frame (ALSA, PipeWire's quantum) is outside this and varies per session.

Expected with no hub: identical figures; the host SOF is the time base whether or not a hub is in between.

## 6. Images

In `~/.cache/dspi-build/work/wp-USB1/out/`, with `SHA256SUMS`. All built from `wp/USB1` (clean tree), USB MIDI
control channel on, `DSPI_ALINK_USB` on (mode 0 at boot: they behave as the stock firmware until mode 1 or 2 is
written):

| File | Board | Notes |
|---|---|---|
| `DSPi-alink-usb-k340.uf2` | K340 | 1.20 V core (`DSPI_VREG_MV=1200`), as the K340 runs |
| `DSPi-alink-usb-k340-bench-gp22.uf2` | K340 | the same plus the GP22 SOF/SYNC_PULSE marker (E0 only) |
| `DSPi-alink-usb-pico2.uf2` | stock Pico 2 | |
| `DSPi-alink-usb-pico2-bench-gp22.uf2` | stock Pico 2 | GP22 marker |
| `DSPi-alink-usb-pico1.uf2` | Pico 1 (RP2040) | no CLICK (refused) |

## 7. What the simulation predicts

`tools/alink_usb_sim/run.sh` (two units, real engine and phase code; RP2350 SOF timestamps unless noted; host +35 ppm,
units +12 / -18 ppm, B booted 3.7 s after A, D 192, 48 kHz). Every scenario also checks that no hard edge (audio cut to
silence or silence to audio without a fade or the pipeline's mute) reaches a playing stream: 0 in all of them. The model
loses about 0.4 % of packets by itself (1 % of its main-loop passes take 900 us and miss the re-arm), so every scenario
has about 2-3 faded gaps per second; on the bench 4.6 measures the real rate.

| Scenario | max skew, last 10 s | latency A / B (samples) | notes |
|---|---|---|---|
| S1 two RP2350 | 0.008 sample (0.17 us) | 191.8 / 191.8 | locked 1.5 s / 2.6 s after boot (WP USB3; 5.3 / 9.9 s before) |
| S2 two RP2040 (ISR time, 2 % late entries) | 0.033 (0.68 us) | 191.9 / 191.9 | |
| S3 RP2350 + RP2040 | 0.14 (3.0 us) | 191.8 / 191.9 | the RP2040's interrupt entry, a fixed offset |
| S4 host +100 / -100 ppm | 0.022 / 0.044 | 191.8 | locked 4-5 s after boot (muted fast slew; 15-19 s before WP USB3) |
| S5 / P3 host +-296, +-300, +-400 ppm | | | rejected (state 6), VCXO parked; +-294 ppm locks |
| S6 / P1 0.5 % of packets skipped by the host | 0.038 | 191.8 / 191.8 | faded gaps, no re-anchor, no hard edge |
| P2 / P2b 5 or 2 packets lost in a row | 0.029 / 0.026 | 191.8 | faded (5: also muted 20 ms), no re-anchor |
| S7 stream on B stopped 2 s and restarted | 0.019 | 191.8 / 191.8 | no re-anchor; the stop fades out |
| S8 SOF gone 120 ms, frame number jumps | 0.017 | 191.8 / 191.8 | one new anchor each |
| P5 suspend 3 s / P6 port reset 15 ms (counter runs) | 0.021 / 0.019 | 191.8 | P5 one new anchor; P6 timeline kept |
| P6b / P6c SOF stall 15 ms / 1 ms, frame number frozen | 0.030 / 0.030 | 191.8 | one new anchor each, no loop, no false host range |
| P4 host 35 -> 300 -> 280 ppm | 0.026 | 191.8 | rejected in between, anchored again (3 / 2 anchors) |
| P4b host step +25 ppm while locked | 0.013 | 191.6 | two re-anchors while the estimate follows |
| P7 / P7b main-loop stall 10 / 100 ms on A | 0.020 / 0.029 | 191.8 | one re-anchor (the stall exceeds the ring fill at D 192) |
| P8 / P8b / P8c interrupts masked 300-400 us | 0.013 / 0.070 / 0.013 | 191.8 | P8c: 4 of 38830 packets mis-stamped (the 100 us rule), faded |
| P9 / P10 D 399 / D 170 | | 398.8 / 169.8 | |
| S10 CLICK on one bus frame | | | 0.026 sample apart |
| S12 96 kHz, D 336 | 0.025 | 335.8 / 335.8 | |
| H1 / H2 no stream, mode 1 / mode 2 | | | VCXO on the host rate; mode 2 anchors on silence (with a 1 s DAC mute hold) |
| S11 mode 1 (contrast) | mean 11, max 62 samples | 60 / 60 (spread 15-82) | the phase follows each packet and the fill servo |

Lock times (WP USB3, `LOCKTIME` lines; host against the unit's crystal, the model's 200 ms DAC hold):

| Case | before WP USB3 | WP USB3 | stored host rate |
|---|---|---|---|
| L2 / L3 / L4 cold boot, host +20 / +100 / +290 ppm | 4.8 / 16.9 / 45.5 s | 1.4 / 4.3-4.4 / 11.5 s | 0.51 s |
| L5 SOF lost 15 s (beyond the ride), from the SOF's return | 3.5 s | 0.51 s | |
| L6 stream restart | no relock (timeline kept) | no relock | |
| L7 phase error 10 samples (ring skip) | 0.27 s (full) | 0.08 s (soft, from the skip; 15 ms muted) | |
| L8 output rings reset (preset load, flash write), after the reset | 0.20 s (second DAC hold) | 0.004 s | |
| L9 stored rate +290, host +20 (another PC) | | 10.8 s (slew from the stored rate) | |

In every mode-2 scenario no stream sample is heard while the unit is out of step.

Latencies read about 0.2 sample short of D in the model (the output DMA's quarter-frame word in the fill reading), the
same on every unit.
