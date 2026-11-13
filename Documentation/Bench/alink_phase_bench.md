# Alink bench: two phase-locked receivers (SYS2 test (e))

*WP LK6, 2026-11-13. Not yet run on hardware. The procedure for the owner's step (e) of the SYS2 alink spec section 8.*

Two receivers play the same alink stream with the same `d_local`. They must play every alink frame at the same instant:
**skew under one sample (20.8 us at 48 kHz), target under 10 us, constant over an hour**. This guide records both
outputs into one stereo audio interface and measures the skew with `tools/alink/skew_xcorr.py`. A scope variant follows.
How the phase lock works is in `Documentation/Features/alink_receiver.md` section 8.

## 1. What you need

| Item | Notes |
|---|---|
| Sender | a Pico 2 with the LK4 sender (`dspi_sender-rp2350.uf2`, `alink_sender.md`), or a bench TX board with `alink_bench-*.uf2` (`alink_gpio_bench.md`). The sender gives both measurement modes; the bench TX only the click mode (2.2) |
| Receivers A and B | two DSPi units of the same kind (two K340s, or two Pico 2 DSPis with the same DAC), firmware with LK6 (`DSPi.uf2` from this branch, USB MIDI build for `midi_ctrl`) |
| Alinks | sender alink 0 (GP2) to A's GP0 and alink 1 (GP3) to B's GP0, the same medium and length on both (bench wiring: `alink_gpio_bench.md` section 7, 47 ohm series resistor, shared ground). With a bench TX board, `pins=2,3` drives both from one stream |
| Audio interface | one interface with two line inputs on one clock (any USB interface; both channels share its ADC clock), 48 kHz or 96 kHz, 24-bit |
| Recorder | any program that writes a 24-bit stereo WAV (`arecord -D hw:<card> -f S24_3LE -c 2 -r 48000 rec.wav`) |

**Safety.** CLICK is a one-sample impulse at **-20 dBFS after** the volume control (the master volume does not
attenuate it). A muted unit does not click: user mute or the sender's system-mute flag make the op complete as
`refused`, so keep the programme quiet by other means (2.1, 2.2), not by muting. Still connect **no speaker and no
amplifier input** to the outputs used here. Record line level: a DSPi's DAC output, or a K340's DAC test point. If only an amplifier
output is available, use a dummy load and a resistive divider to line level, never the interface directly. Noise from
the PC goes through the owner's guarded player at 80 % volume or less (the test-audio rules).

## 2. Set-up

Flash and back up as in `alink_gpio_bench.md` section 8 (the owner flashes). Then configure **both** receivers. With USB
MIDI builds, from `tools/midi_ctrl`, once with `A=/dev/snd/midiCxD0` and once with B's node:

```sh
U=/dev/snd/midiCxD0                    # A, then B
M() { DSPI_MIDI_DEV=$U ./run.sh midi_ctrl.py "$@" --port $U; }
M set 0x03 --u8 2                                          # REQ_OUTPUT_LATENCY: the 1 ms option (D 128 needs it)
M set 0x00 --wvalue 0x8002 --payload 01ff00ffffff0300      # EXT_ALINK_CONFIG: enable, GP0, auto class, mute + re-anchor
M set 0x00 --wvalue 0x8003 --payload 0001020304050607      # EXT_ALINK_SLOTS: alink channels 0-7 to inputs 0-7
M set 0x00 --wvalue 0x8004 --payload 0100800000000000      # EXT_ALINK_PHASE: phase lock, d_local 128, hops_offset 0
M set 0xE0 --u8 7                                          # input source ALINK
M ext-list                                                 # ... 8000 8001 8002 8003 8004 8005 8006 8008 8009 800a
M alink                                                     # info d_local=81..431; phase: mode=phase lock d_local=128 state=locked
```

The matrix routes input 0 (alink channel 0) to the output you record, as for any 8-channel input. At 96 kHz the 1 ms
option needs `d_local` 145 or more (`EXT_ALINK_INFO` bytes 16-19 give the range; a value below it is refused, and the
unit falls back to the fill servo with `D_BELOW_MIN` if the range moves under it).

**Lock sequence to expect** (`M alink` or `M notify`): class detection and trim scan (0.1-0.4 s), lock, then the VCXO
pull-in. The unit stays muted until its VCXO is within 1 ppm of the stream's rate, which takes up to 10 s from a 50 ppm
crystal offset and 2-4 s from a small one (the long rate window needs a 2 s span first). Then the anchor (silent, a few ms), `phase: state=locked`, and the
audio fades in. `status: phase_error` stays within a few hundredths of a frame.

### 2.1 Sender: USB audio and the click train

The sender (`alink_sender.md`): enable alinks 0 and 1 with audio from USB, class L1, 8 channels, 48 kHz. Find its node with
`lsusb -d 2e8b:feaa` (product "DSPi Audio Link Sender") and use `tools/alink/bench.sh`, which passes only that node into its
container:

```sh
SND=/dev/bus/usb/<BBB>/<DDD>
tools/alink/bench.sh sender-diag --usb $SND --pattern audio        # test_pattern 0: USB audio
```

(`EXT_ALINK_TX` `alink_enable` must have bits 0 and 1 set. `bench.sh sender` sets alink 0 only, so write `EXT_ALINK_TX` with
`03 00 01 08 01 00 00 ff ff 00 00 00 00 00 00 00`, alinks 0 and 1, L1, C 8, 48 kHz, audio, with `dspi_poke.py set 0x00
--wvalue 0x8007 --hex "..."`, which needs pyusb on the host.)

For the **click train**, the sender schedules a CLICK apply in its superframe once a second. Every phase-locked
receiver then plays a one-sample impulse at -20 dBFS on output 0 for the same alink frame, `d_local` frames after it
arrives. Record the clicks with no USB audio playing (the sender sends zeros), not with the system-mute flag, which
stops the clicks too:

```sh
tools/alink/bench.sh clicks --usb $SND --count 60 --period 1 --output 0
```

### 2.2 Bench TX board instead of a sender

The bench TX sends a valid stream, but its payload is a fixed vector (steady levels), so only the click mode works,
through the per-unit path `EXT_APPLY_AT_FRAME`: both receivers get a CLICK for the same frame F. Start the stream with
`tools/alink/bench.sh cmd <TX> 'tx L1 48k ch=8 pay=vec pins=2,3'`, then from `tools/midi_ctrl`:

```sh
A=/dev/snd/midiCaD0; B=/dev/snd/midiCbD0
for i in $(seq 30); do
  f=$(DSPI_MIDI_DEV=$A ./run.sh midi_ctrl.py get 0x00 --wvalue 0x8005 --port $A |
      awk '/^ *0000:/{print $9$8$7$6$5$4$3$2}')                        # EXT_FRAME_COUNTER: F of A
  t=$(printf '%016x' $((0x$f + 144000)))                                # 3 s ahead (two tool starts)
  p=$(echo $t | sed 's/../& /g' | awk '{for(i=8;i>=1;i--) printf "%s",$i}')0700000000000000
  for U in $A $B; do DSPI_MIDI_DEV=$U ./run.sh midi_ctrl.py set 0x00 --wvalue 0x8006 --payload $p --port $U; done
  sleep 4
done
```

The payload is F (8 bytes, little-endian), op 7 (CLICK), output 0, value 0, 4 reserved bytes. `M get 0x00 --wvalue
0x8006` shows `last_result` 0 (applied), and `M notify` shows one event `0xE2` per click with that frame. Result 1
(late) means the three seconds were not enough.

## 3. Recording and analysis

1. **Calibrate the interface** (once): split **one** receiver output into both inputs with a Y cable and record 2
   minutes of noise or 20 clicks: `calib.wav`. This measures the interface's own channel skew, which is subtracted.
2. **Record**: A on the left input, B on the right, levels a few dB below clipping on the clicks.
   - noise: play pink noise or music through the sender (guarded player, 80 % or less) and record **1 hour**: `rec.wav`;
   - clicks: run the click train of 2.1 or 2.2 while recording: `clicks.wav`.
3. **Analyse** in the alink tools container (standard library only):

```sh
tools/alink/run.sh skew_xcorr.py --selftest                 # the estimators on synthetic data
mkdir -p rec && cp calib.wav rec.wav clicks.wav rec/        # under the checkout root, which run.sh mounts at /work
tools/alink/run.sh skew_xcorr.py /work/rec/rec.wav --calib /work/rec/calib.wav
tools/alink/run.sh skew_xcorr.py /work/rec/clicks.wav --mode click --calib /work/rec/calib.wav
```

Noise mode correlates 1 s of left and right once a minute (`--every`, `--span`) over +-48 samples (`--max-lag`) and
fits a parabola through the peak. Click mode finds each impulse on both channels and takes the fraction from the slope
of the cross-spectrum's phase; it removes each channel's DC (a bench TX's vector payload) and detects at half of each
channel's peak (`--thresh` overrides), so the clicks' -20 dBFS and the interface's gain need no setting. Set the
interface gain so that the clicks reach at least -40 dBFS in the recording. A positive skew means B (right) is late.
The last line says PASS or FAIL.

## 4. Expected results

| Measurement | Expected | Pass (SYS2 3.5, 8e) |
|---|---|---|
| skew, mean | **within +-2 us (0.1 sample)**: the receivers differ only by the RX sampling grid (one line bit, 65 ns at L1), the output DMA word, the VCXO's 0.11 ppm dither and the DACs' group delay (identical boards: equal) | under one sample (20.8 us); target under 10 us |
| skew, range over an hour | a few tenths of a microsecond; no drift | constant to under half a sample |
| clicks | every click on the same frame on both: the same skew as noise mode, impulse by impulse | as above |
| `status` | `phase_error` within +-0.05 frame on both; `loss_count` 0; `phase` state 2 | |
| power cycles | the same skew after 10 power cycles of either unit in any order (each re-anchors to the same delay) | identical to within 1 sample |
| fibre pull (one unit) | silence within D + 1 ms (2.7 ms at D 128), the `0xE1` event with state 3 (holdover); after replugging, audio within 200 ms at the **same** skew, without a re-anchor when the pull was short enough for the held VCXO (seconds) | same skew afterwards |

The host simulation (`tools/alink_phase_sim/run.sh`) runs the same receivers in software, with real decoder and
phase-lock code, a modelled line, crystals, VCXO and output rings. It gives these figures. Its stereo recording of both
outputs (`ALINK_PHASE_SIM_WAV=...`) analysed by `skew_xcorr.py` reads -0.01 sample:

| Scenario (L1 48 kHz C8 unless noted, D 128) | max skew, last 10 s | latency A / B (frames) | CLICK skew |
|---|---|---|---|
| same boot, same clocks | 0.026 frame | 127.93 / 127.94 | 0.001 sample |
| B boots 3.7 s later | 0.028 | 127.91 / 127.90 | 0.024 |
| sender +20, A +45, B -38 ppm | 0.016 | 127.92 / 127.93 | 0.014 |
| 1 ms dark line on B | 0.035 | 127.90 / 127.90 | 0.012 |
| 2 s dark line on B (holdover, no re-anchor) | 0.013 | 127.92 / 127.91 | 0.009 |
| L3 96 kHz C16, D 200, offsets, boot times | 0.025 | 199.88 / 199.88 | 0.003 |
| mode 0 (fill servo), for contrast | **3.75** | 86.2 / 82.5 | |
| sender power cycle (F back to 0) during B's 2 s dark line: both re-anchor once, play 0-8 ms after the line carries the new stream | 0.037 | 127.91 / 127.90 | 0.006 |
| sender restart with F 10M ahead, likewise | 0.033 | 127.91 / 127.92 | 0.002 |
| 60 s dark line on B (holdover, no re-anchor) | 0.020 | 127.93 / 127.94 | 0.018 |
| 60 s holdover, B's held VCXO drifting 0.5 ppm (whole-frame corrections, no re-anchor) | 0.065 | 127.92 / 127.86 | 0.062 |

The latency reads 0.08 frame short of D in the model because the arrival is taken at the comma's end and to the DMA word.
The offset is the same on every unit.

## 5. Scope variant

SYS2 8e's first check. Scope channel 1 on A's output, channel 2 on B's, trigger on channel 1, run the click train of
2.1 or 2.2. Both impulses appear within one sample (20.8 us); the expected skew is a few microseconds at most. Repeat
after 10 power cycles in any order. For absolute latency (8c) the sender's SYNC pin (`EXT_ALINK_TX.sync_pin`, GP21 on the
carrier) pulses at F mod Fs = 0. A CLICK at a frame with F mod 48000 = 0 then shows D x 20.83 us plus the DAC's group
delay after the SYNC edge.

## 6. If it fails

| Symptom | Look at |
|---|---|
| `phase: state=free` | `mode` 0, or `d_local + hops_offset` outside `d_local_min..max` (`M alink`, `D_BELOW_MIN`); the 4 ms latency option needs 257 or more |
| never unmutes | the VCXO pull-in: `status: vcxo` must approach the crystal offset (`diag`: `ppm_ff`); offsets above the VCXO's authority cannot lock |
| skew of whole samples | the two units have different `d_local`, or different output delays (`out.delay`, op `OUT_DELAY`), or one sits behind a fan-out (its `hop_delay_frames` must be compensated through `hops_offset`, `alink_protocol_spec.md` 7.2) |
| skew drifts | `loss_count` / re-anchors (`0xE1` reason 5); the recording's clock is not one clock for both channels |
| click results 1 (late) | more lead (the bench TX recipe's 3 s), or the unit was not locked |
| click results 3 (refused), no impulse | the unit is muted (`REQ_SET_USER_MUTE`, op MUTE, or the sender's system-mute flag), or the output is the PDM sub |
| after a sender power cycle | every unit re-anchors once (F restarted at 0) and plays again within about 50 ms of locking; the skew is the same as before |
