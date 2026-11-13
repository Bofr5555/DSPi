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
| Tools | `tools/midi_ctrl/run.sh` (both images have the USB MIDI control channel, so each unit is addressed by its own rawmidi node), `tools/link/run.sh skew_xcorr.py`, `tools/alink_usb/alink_usb_status.py` (standard library; run with `python3 -I`) |

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
(`SOF_TIMESTAMP_LAST`, 21 ns), the RP2040 from clean interrupt entries (within 3 us of 1 ms apart), averaged by the servo.

## 3. E2: rate lock (mode 1)

1. Both units: `M $U set 0x00 --wvalue 0x8010 --payload 0101c00000000000` (mode 1, D 192).
2. Wait until both read `state rate-locked` and `nominal-feedback 1`: `ST $A a; ST $B b`. That takes 2 s plus the pull-in
   at 6.7 ppm/s: about 5 s at a 20 ppm host, 17 s at 100 ppm. `host` shows the PC's SOF rate against each unit's
   crystal (the two differ by the units' crystals); `vcxo` converges to it.
3. PipeWire combine sink to both units (`pactl load-module module-combine-sink sink_name=both slaves=<A>,<B>`), pink
   noise through the guarded player at 80 % or less, record A left and B right (`arecord -D hw:<card> -f S24_3LE -c 2
   -r 48000 e2.wav`) for 1 hour.
4. `tools/link/run.sh skew_xcorr.py /work/rec/e2.wav --calib /work/rec/calib.wav` (calibration as in
   `link_phase_bench.md` section 3).

Expected: the skew is **constant within the session** (per-minute values within a few us: both run at the host's rate,
so nothing drifts and PipeWire's combine sink stops resampling) but **arbitrary**: anything up to milliseconds, and
different after every restart of the streams. That is the contrast for E3 (the simulation's mode-1 case: 59.7 and 57.6
samples of latency on the two units).

## 4. E3: phase lock (mode 2), CLICK, restarts, start offsets

### 4.1 Lock

1. Latency option: any; USB runs the 2 ms option or longer, where D may be 153..399 at 48 kHz.
2. Both units: `M $U set 0x00 --wvalue 0x8010 --payload 0201c00000000000` (mode 2, auto re-anchor, **D 192** = 4 ms).
   `M $U set 0x52` (REQ_SAVE_OUTPUT_CONFIG) persists it.
3. Wait for `state phase-locked`, `phase locked`, `phase error` within +-0.05 sample (`ST $A a; ST $B b`): pull-in as
   in E2, then about 0.2 s of muted anchoring. `anchors 1`, `re-anchors 0`. No stream is needed: the units are
   locked on silence.

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
per run), then `tools/link/run.sh skew_xcorr.py /work/rec/clicks.wav --mode click --calib /work/rec/calib.wav`.

| Measurement | Expected | Pass |
|---|---|---|
| CLICK skew, mean | within +-1 us between two RP2350 units on one hub (simulation 0.005 sample = 0.1 us; the bench adds the E0 mean and the DACs' group-delay difference); about 2.5 us RP2350 vs RP2040 | under 20.8 us; target under 10 us |
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

`missed` and `late` in `ST` count samples of missing packets (silence placed on the timeline, the stream muted 20 ms and
faded back in) and late samples dropped. On a quiet PC both stay 0 or grow very slowly; a burst says the host or the
unit's main loop skipped a frame (the unit re-arms its OUT endpoint from the main loop, so a main-loop pass over 1 ms
loses the next packet). The phase is not affected.

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
units +12 / -18 ppm, B booted 3.7 s after A, D 192, 48 kHz):

| Scenario | max skew, last 10 s | latency A / B (samples) | notes |
|---|---|---|---|
| two RP2350 | 0.013 sample (0.27 us) | 191.82 / 191.82 | locked 5.1 s / 9.7 s after boot |
| two RP2040 (ISR time, 2 % late entries) | 0.023 (0.48 us) | 191.93 / 191.92 | |
| RP2350 + RP2040 | 0.12 (2.5 us) | 191.82 / 191.92 | the RP2040's interrupt entry, a fixed offset |
| host +100 / -100 ppm | 0.021 / 0.008 | 191.81 / 191.83 | locked 15-19 s after boot (VCXO slew) |
| host +-300, +-400 ppm | | | rejected (state 6), VCXO parked |
| 0.5 % of packets skipped | 0.022 | 191.82 / 191.81 | silence in place, no re-anchor |
| stream on B stopped 2 s and restarted | 0.025 | 191.82 / 191.81 | no re-anchor |
| SOF gone 120 ms, frame number jumps | 0.032 | 191.82 / 191.81 | one re-anchor each |
| CLICK on one bus frame | | | 0.005 sample apart |
| 96 kHz, D 288 | 0.029 | 287.79 / 287.78 | |
| mode 1 (contrast) | | 59.7 / 57.6 | arbitrary, per start |

Latencies read about 0.2 sample short of D in the model (the output DMA's quarter-frame word in the fill reading), the
same on every unit.
