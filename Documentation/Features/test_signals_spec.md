# Test Signal Generator Specification

*Status: implemented*
*Last updated: 2026-11-18 (WP FWSIGGEN: DAC hardware-mute ownership, pump timing)*

## 1. Overview

The onboard **test signal generator** ("siggen") synthesizes measurement and
diagnostic signals directly inside the DSPi output pipeline, with no host audio
stream required. It is aimed at room-correction sweeps, polarity and channel
identification, distortion and intermodulation tests, inter-sample-peak
verification, and general bring-up diagnostics driven from DSPi Console or any
control bridge.

Signals are written into the per-output mix buffers (`buf_out[]`) in
`process_input_block()`, **between the matrix-mix pass and the per-output
processing pass**. Selected (masked) output channels have their routed program
audio replaced by the generated signal; unselected channels keep playing their
program audio untouched. Because every output slot still advances by the same
`sample_count` on every block, **inter-slot sample alignment is preserved by
construction** (the injection replaces sample values in place; it never changes
how many samples a slot consumes or produces).

### Key characteristics

- **Transient only.** The generator is always off at boot, is never persisted to
  flash, and is stopped by preset load and factory reset. There is no "saved"
  siggen state.
- **Alignment-safe.** Injection happens after matrix mixing and before per-output
  processing, so the four S/PDIF instances, the I2S slots, and PDM all remain
  sample-aligned. No siggen operation can drift slots.
- **Downstream chain intact by default.** By default the generated signal passes
  through the channel's crossover, PEQ, output trim, master volume, mute, and
  delay, exactly like program audio. This means what you measure is what the
  device actually outputs through its full processing chain.
- **Optional RAW bypass.** `SIGGEN_FLAG_RAW` bypasses only the per-channel
  crossover + PEQ on generator channels. Output trim, master volume, mute, and
  delay still apply (deliberately, for safety and level control).
- **Runs with no host stream.** When no input source is streaming, the main loop
  pumps zero-input blocks through the pipeline so the generator keeps playing,
  sample-exact against the DAC clock (section 9). When a real source starts
  streaming again it takes over pacing.
- **Owns the DAC hardware mute while it plays.** An idle hold of the DAC mute pin
  (the USB phase mode waiting for a host) is opened by the generator itself and
  restored after its tail (section 9.3).
- **Full platform parity.** Both platforms expose the identical wire protocol and
  signal catalogue. RP2350 uses the float audio path; RP2040 uses the Q28
  fixed-point path. The sample kernels fork internally; the protocol does not.

### Signal-chain position

```
Input source (USB / S/PDIF / I2S)  OR  zero-input pump (no source)
    |
process_input_block():
    PASS: per-input preamp, master EQ, leveller, crossfeed, metering
    |
    PASS: matrix mixing (fan-out to output channels -> buf_out[])
    |
>>> TEST-SIGNAL INJECTION (siggen_render): masked channels overwritten <<<
    |
    PASS: per-output crossover + PEQ (skipped on channels in siggen_raw_mask)
          + output delay + output trim + master volume + mute
    |
Output encoding (S/PDIF, I2S, PDM)
```

---

## 2. Signal Catalogue

Fifteen signal types (`SiggenType`, `signal_type` field). The p1..p4 columns are
the four `float` parameters; blank means unused. "Timing" is the timing model
(section 6).

| ID | Name | Type | Timing | p1 | p2 | p3 | p4 | Notes |
|----|------|------|--------|----|----|----|----|-------|
| 0 | `SIGGEN_SINE` | sine | continuous | freq Hz (def 1000) | - | - | - | 7th-order poly kernel, THD approx -139 dB |
| 1 | `SIGGEN_SQUARE` | square | continuous | freq Hz (def 100) | - | - | - | polyBLEP band-limited |
| 2 | `SIGGEN_WHITE` | white | continuous | - | - | - | - | uniform white noise |
| 3 | `SIGGEN_PINK` | pink | continuous | - | - | - | - | -3 dB/oct; internally normalized by 1/8 of peak |
| 4 | `SIGGEN_SWEEP_LOG` | swp-log | sweep | f1 Hz (def 20) | f2 Hz (def 20000) | - | - | log (exponential) sweep; `duration_ms` = one sweep |
| 5 | `SIGGEN_SWEEP_LIN` | swp-lin | sweep | f1 Hz (def 20) | f2 Hz (def 20000) | - | - | linear sweep; `duration_ms` = one sweep |
| 6 | `SIGGEN_SWEEP_STEP` | swp-stp | sweep | f1 Hz (def 20) | f2 Hz (def 20000) | steps/oct (1..24, def 3) | dwell ms (20..10000, def 250) | stepped/discrete-tone sweep |
| 7 | `SIGGEN_IMPULSE` | impulse | pattern | period ms (10..60000, def 500) | - | - | - | single-sample unit impulses |
| 8 | `SIGGEN_CLICKS_ALT` | clk-alt | pattern | period ms (10..60000, def 500) | - | - | - | alternating-polarity single-sample clicks |
| 9 | `SIGGEN_POLARITY` | polarty | pattern | pulse width ms (1..100, def 5) | period ms (10..60000, def 500) | - | - | single positive half-sine lobe per period |
| 10 | `SIGGEN_TONE_BURST` | burst | pattern | freq Hz (def 1000) | on cycles (1..1000, def 8) | off cycles (0..1000, def 8) | edge cycles (0..100, def 2) | sine burst with raised-cosine edges |
| 11 | `SIGGEN_TONE_PAIR` | tonpair | continuous | f1 Hz (def 60) | f2 Hz (def 7000) | amp ratio A1/A2 (0.1..10, def 4) | - | IMD: SMPTE 60/7000/4, CCIF 19000/20000/1 |
| 12 | `SIGGEN_MULTITONE` | multi | continuous | tone count (2..MAX, def 10) | f_lo Hz (def 20) | f_hi Hz (def 20000) | - | log-spaced tones, Schroeder phases, sum-normalized |
| 13 | `SIGGEN_ISP` | isp | continuous | pattern 0/1 (def 0) | - | - | - | inter-sample-peak test; +3.01 / +1.25 dBTP overs |
| 14 | `SIGGEN_CHANNEL_ID` | chan-id | pattern | blip ms (30..1000, def 120) | - | - | - | (channel+1) pentatonic blips; always walks |

`SIGGEN_MULTITONE`'s tone-count ceiling is `SIGGEN_MULTITONE_MAX`: **16 on
RP2350, 8 on RP2040** (per-platform CPU budget). Requests above the ceiling are
clamped.

### 2.1 Level and normalization

`level_db` is a **peak level in dBFS**, range -120..0 (values outside are clamped;
NaN is clamped to -120). It maps to a linear peak amplitude `10^(dB/20)`.

- **Sine / square / sweeps / bursts / clicks / impulses / polarity / channel-ID**:
  the peak equals `level_db`.
- **Pink noise**: internally normalized by 1/8 of the peak (the Kellet economy
  filter peaks around 7.1 on uniform white input; the 1/8 factor keeps it below
  full scale).
- **Multitone**: the level is split across the N tones by **sum normalization**
  (`amp_tone = peak / N`), so the coherent worst case stays within the peak.
  Schroeder phases bound the realized crest factor.
- **Tone pair**: split by the amplitude ratio `p3 = A1/A2`; `A1 = peak*r/(1+r)`,
  `A2 = peak/(1+r)`.
- **ISP patterns**: sample-peak-normalized sequences with known true-peak overs.
  Pattern 0 is `+1,+1,-1,-1` (fs/4 at 45 degrees, **+3.01 dBTP** inter-sample
  over). Pattern 1 is `+1,+1,0,-1,-1,0` (fs/6 at 30 degrees, **+1.25 dBTP**).
  Set `level_db` at or below the headroom you want the over to fit into.

### 2.2 Channel-ID melody

`SIGGEN_CHANNEL_ID` announces which physical channel is which. On the current walk
channel `k` it plays **k+1** raised-cosine sine blips at that channel's pitch,
then a 700 ms tail pause, then advances the walk to the next masked channel.

- Pitches are a C-major pentatonic from C5: `C5 D5 E5 G5 A5` =
  `523.25, 587.33, 659.25, 783.99, 880.00 Hz`. Channels 5 and above use the same
  five pitches one octave up (`pitch * 2`, indexed by `channel % 5`).
- Blip length defaults to 120 ms (`p1`, 30..1000 ms), with a fixed 100 ms
  inter-blip gap and a 700 ms tail.
- **CHANNEL_ID always walks**, regardless of the `SIGGEN_FLAG_WALK` flag. So
  channel 0 gets 1 blip, channel 1 gets 2 blips, and so on; a listener counts
  blips to identify the channel and hears the pitch rise across the array.

---

## 3. Wire Structures

All structures are packed, little-endian; floats are IEEE-754 single-precision.
`SIGGEN_CFG_VERSION = 1` tags every structure's `version` byte.

### 3.1 `SiggenConfig` (36 bytes)

Payload of `REQ_SIGGEN_SET_CONFIG`; also the response of `REQ_SIGGEN_GET_CONFIG`.

| Offset | Size | Type | Field | Meaning |
|-------:|-----:|------|-------|---------|
| 0 | 1 | u8 | `version` | must be `SIGGEN_CFG_VERSION` (1); mismatched configs are rejected |
| 1 | 1 | u8 | `signal_type` | `SiggenType` 0..14 |
| 2 | 2 | u16 | `channel_mask` | output-channel select, bit i = output i; masked to valid outputs |
| 4 | 2 | u16 | `invert_mask` | polarity-inverted subset of `channel_mask` (masked to it) |
| 6 | 1 | u8 | `flags` | `SIGGEN_FLAG_*` bitmask (section 5) |
| 7 | 1 | u8 | `reserved0` | must be 0 |
| 8 | 4 | f32 | `level_db` | peak level dBFS, -120..0 (clamped) |
| 12 | 4 | u32 | `duration_ms` | timing-model dependent (section 6) |
| 16 | 2 | u16 | `repeat` | timing-model dependent (section 6) |
| 18 | 2 | u16 | `gap_ms` | inter-cycle silence, timing-model dependent |
| 20 | 4 | f32 | `p1` | per-type parameter (section 2 table) |
| 24 | 4 | f32 | `p2` | per-type parameter |
| 28 | 4 | f32 | `p3` | per-type parameter |
| 32 | 4 | f32 | `p4` | per-type parameter |

### 3.2 `SiggenStatus` (16 bytes)

Response of `REQ_SIGGEN_GET_STATUS`.

| Offset | Size | Type | Field | Meaning |
|-------:|-----:|------|-------|---------|
| 0 | 1 | u8 | `version` | `SIGGEN_CFG_VERSION` |
| 1 | 1 | u8 | `state` | `SiggenState` (section 4.3): IDLE/FADE_IN/RUN/GAP/FADE_OUT |
| 2 | 1 | u8 | `signal_type` | active or last `SiggenType` |
| 3 | 1 | u8 | `active_channel` | walk: current output channel; `0xFF` when not walking |
| 4 | 4 | u32 | `elapsed_ms` | time since start |
| 8 | 2 | u16 | `cycles_done` | completed sweeps / pattern periods (saturates at 0xFFFF) |
| 10 | 1 | u8 | `stop_reason` | `SIGGEN_STOP_*` of the last stop |
| 11 | 1 | u8 | `reserved0` | 0 |
| 12 | 4 | f32 | `current_freq` | instantaneous sweep frequency in Hz; 0 when not sweeping |

The wire `state` is derived from the internal engine plus the fade overlay: a
rising fade reports `FADE_IN`, a falling fade reports `FADE_OUT`, otherwise the
raw engine state (`RUN`/`GAP`/`IDLE`) is reported.

### 3.3 `SiggenCapsHeader` (8 bytes)

Response of `REQ_SIGGEN_GET_CAPS` with `wValue = 0xFFFF`.

| Offset | Size | Type | Field | Meaning |
|-------:|-----:|------|-------|---------|
| 0 | 1 | u8 | `version` | `SIGGEN_CFG_VERSION` |
| 1 | 1 | u8 | `type_count` | `SIGGEN_TYPE_COUNT` (15) |
| 2 | 1 | u8 | `output_channels` | `NUM_OUTPUT_CHANNELS` (5 RP2040 / 9 RP2350) |
| 3 | 1 | u8 | `multitone_max` | `SIGGEN_MULTITONE_MAX` (8 RP2040 / 16 RP2350) |
| 4 | 2 | u16 | `valid_channel_mask` | `(1 << NUM_OUTPUT_CHANNELS) - 1` |
| 6 | 2 | u16 | `reserved0` | 0 |

### 3.4 `SiggenTypeDesc` (62 bytes)

Response of `REQ_SIGGEN_GET_CAPS` with `wValue = 0..type_count-1` (a type index).
Self-describing per-type parameter metadata.

| Offset | Size | Type | Field | Meaning |
|-------:|-----:|------|-------|---------|
| 0 | 1 | u8 | `id` | `SiggenType` |
| 1 | 8 | char[8] | `name` | NUL-padded short name (matches section 2 "Name") |
| 9 | 1 | u8 | `timing_model` | `SIGGEN_TIMING_*` (0 continuous, 1 sweep, 2 pattern) |
| 10 | 52 | `SiggenParamDesc[4]` | `p[4]` | per-parameter descriptors (below) |

`SiggenParamDesc` (13 bytes each):

| Offset | Size | Type | Field | Meaning |
|-------:|-----:|------|-------|---------|
| 0 | 1 | u8 | `semantic` | `SIGGEN_PARAM_*`: 0 unused, 1 FREQ_HZ, 2 MS, 3 CYCLES, 4 COUNT, 5 RATIO, 6 PATTERN |
| 1 | 4 | f32 | `min` | minimum accepted value |
| 5 | 4 | f32 | `max` | maximum accepted value |
| 9 | 4 | f32 | `def` | default value |

A host should read the caps to render controls and validate ranges rather than
hard-coding the section 2 table; the descriptors are the authoritative source and
already reflect the running platform's `multitone_max`.

---

## 4. Vendor Commands

Five vendor opcodes, `0xA4..0xA8`. Opcodes `0xA9..0xAF` are reserved for future
generator extensions. All commands are reachable over USB EP0 (vendor interface
2), and automatically over the UART and I2C target control transports (they ride
the shared vendor dispatcher). Multi-byte fields are little-endian.

| Code | Name | Direction | wValue | Payload / Response |
|------|------|-----------|--------|--------------------|
| 0xA4 | `REQ_SIGGEN_SET_CONFIG` | OUT (0x41) | 0 | Payload: `SiggenConfig` (36 B). STALL on reject |
| 0xA5 | `REQ_SIGGEN_GET_CONFIG` | IN (0xC1) | 0 | Response: `SiggenConfig` (36 B) |
| 0xA6 | `REQ_SIGGEN_CONTROL` | write-as-read (0xC1) | `SIGGEN_CTL_*` | Response: 1 status byte (1 = accepted); STALL on reject |
| 0xA7 | `REQ_SIGGEN_GET_STATUS` | IN (0xC1) | 0 | Response: `SiggenStatus` (16 B) |
| 0xA8 | `REQ_SIGGEN_GET_CAPS` | IN (0xC1) | 0xFFFF or type index | Response: `SiggenCapsHeader` (8 B) or `SiggenTypeDesc` (62 B) |

### 4.1 `REQ_SIGGEN_SET_CONFIG` (0xA4, OUT)

Validate and stage a config from the 36-byte payload. Validation rejects (STALL):

- payload shorter than 36 bytes,
- `version != 1`,
- `signal_type >= 15`,
- an **effective channel mask of zero** (no valid output bits set), and
- a sweep type (`LOG`/`LIN`/`STEP`) with `duration_ms == 0`.

On accept, `channel_mask` is clamped to valid outputs, `invert_mask` is clamped to
`channel_mask`, and `level_db` is clamped to -120..0. **SET never auto-starts.**
If the generator is idle, the config is simply staged and becomes the config a
subsequent `START` uses. If the generator is already running, SET **restarts** it:
the engine fades out, `siggen_service()` (main loop) applies the new config, then
the engine fades back in. The interim status `stop_reason` is `SIGGEN_STOP_RECONFIG`.

### 4.2 `REQ_SIGGEN_GET_CONFIG` (0xA5, IN)

Returns the currently applied `SiggenConfig`. If no valid config has ever been
staged, returns a zeroed struct with `version = 1`.

### 4.3 `REQ_SIGGEN_CONTROL` (0xA6, write-as-read)

Parameterless action carried in `wValue` (low byte), acknowledged with a single
status byte (`1`). This is a **write-as-read** command: issue it as an IN transfer
(`bmRequestType = 0xC1`) even though it mutates state. Unknown actions, or `START`
with no valid config staged, return failure (STALL).

| `wValue` | Constant | Action |
|----------|----------|--------|
| 0 | `SIGGEN_CTL_STOP` | Fade out over `SIGGEN_FADE_MS`, then idle (`stop_reason = HOST`). A stop issued during a silent gap goes idle immediately (no fade needed). |
| 1 | `SIGGEN_CTL_START` | (Re)start with the applied config. If already running, restarts via fade-out then relaunch (`stop_reason = RECONFIG`). From idle with the DAC hardware mute held, the engine start waits until the generator has opened the DAC and its un-mute ramp has passed (section 9.3); status reports `FADE_IN` meanwhile, and `STOP`/`STOP_NOW` cancel the wait (`stop_reason = HOST`, IDLE notification). |
| 2 | `SIGGEN_CTL_STOP_NOW` | Immediate hard stop, no fade (`stop_reason = HOST`). |

### 4.4 `REQ_SIGGEN_GET_STATUS` (0xA7, IN)

Returns `SiggenStatus` (16 B). Poll this for progress (`elapsed_ms`,
`cycles_done`, `current_freq`) and to detect completion, though completion is also
pushed asynchronously (section 8).

### 4.5 `REQ_SIGGEN_GET_CAPS` (0xA8, IN)

`wValue = 0xFFFF` returns the `SiggenCapsHeader` (8 B). `wValue = 0..type_count-1`
returns that type's `SiggenTypeDesc` (62 B). Any other index returns failure.

---

## 5. Flags

`flags` (`SiggenConfig` byte 6) is a bitmask:

| Bit | Constant | Value | Effect |
|-----|----------|-------|--------|
| 0 | `SIGGEN_FLAG_RAW` | 0x01 | Bypass per-channel crossover + PEQ on generator channels. Output trim, master volume, mute, and delay still apply. |
| 1 | `SIGGEN_FLAG_DECORR` | 0x02 | Noise only (`WHITE`/`PINK`): give each masked channel an independent noise generator (decorrelated). Ignored for other types. |
| 2 | `SIGGEN_FLAG_WALK` | 0x04 | Play masked channels one at a time, advancing one channel per cycle. `CHANNEL_ID` always walks regardless of this bit. |

### 5.1 RAW mode details

RAW is intended for filter-independent measurement (feed the DAC a known signal
without the channel's EQ coloring it) while still respecting the output trim and
master volume so you cannot accidentally blast a full-scale signal. It is
implemented via `siggen_raw_mask` (bit i = output i), read once per block by the
per-output EQ gating on **both cores** (`audio_pipeline.c` and `pdm_generator.c`).
The mask is set from the channel mask when RAW is active and cleared when the
generator goes idle.

While RAW is active, the bypassed crossover/PEQ filter states on those channels
are **frozen** (they are not being run). When normal processing resumes (RAW
cleared, generator stopped, or non-RAW config applied), a brief transient from the
stale filter state can occur as the filters re-engage. This is expected and
short.

### 5.2 Invert mask

`invert_mask` flips output polarity per channel for out-of-phase and cancellation
tests. It is always a subset of `channel_mask`. For correlated signals the
generator renders a primary channel once and copies (or negates) it to the other
masked channels, so an inverted channel is the exact bit-for-bit negation of the
non-inverted one. Combined with a mono sum this gives a clean null; feeding two
channels with one inverted verifies wiring polarity.

### 5.3 Enabled intersection rule

A channel emits signal only if it is **both** in `channel_mask` **and** enabled in
the matrix mixer (`matrix_mixer.outputs[o].enabled`). The effective mask each
block is:

```
eff = channel_mask & (enabled outputs) & valid_output_mask
```

A masked-but-disabled output produces nothing. If the intersection is empty the
config is rejected at SET time only when *no valid bit* is set; a mask that is
valid but happens to hit only disabled outputs is accepted and simply renders
silence until an output is enabled.

---

## 6. Timing Models

Each type has one of three timing models (`SIGGEN_TIMING_*`), which reinterprets
`duration_ms`, `repeat`, and `gap_ms`:

### 6.1 Continuous (`SINE`, `SQUARE`, `WHITE`, `PINK`, `TONE_PAIR`, `MULTITONE`, `ISP`)

- `duration_ms` = total play time. **0 = play until stopped.**
- `repeat`, `gap_ms` unused (unless `WALK` is set, below).
- A non-zero `duration_ms` is **duration-limited**: the engine begins its tail
  fade so the fade completes exactly at the duration end.
- **With `SIGGEN_FLAG_WALK`**, `duration_ms` becomes the **per-channel dwell**
  (default 2 s if unset/too small), and `repeat` counts full passes over the mask
  (0 = infinite). Each channel's dwell gets attack/release edge windows.

### 6.2 Sweep (`SWEEP_LOG`, `SWEEP_LIN`, `SWEEP_STEP`)

- `duration_ms` = length of **one sweep** (must be > 0; enforced at SET).
- `repeat` = sweep count (**0 = infinite**).
- `gap_ms` = silence between sweeps.
- `SWEEP_STEP` additionally uses `p3` = steps per octave (1..24) and `p4` = dwell
  ms per step; its overall length is derived to cover f1..f2 at that resolution.
- Each sweep gets raised attack/release edge windows, and **per-cycle synth state
  is reset** so repeated sweeps are bit-identical. This is required for coherent
  averaging in room-correction measurement.

### 6.3 Pattern (`IMPULSE`, `CLICKS_ALT`, `POLARITY`, `TONE_BURST`, `CHANNEL_ID`)

- `repeat` = number of pattern periods (**0 = infinite**).
- `gap_ms` = extra silence appended per period.
- `duration_ms` unused.
- `CLICKS_ALT` alternates click polarity each period. `CHANNEL_ID` always walks
  (its per-cycle length depends on the walk channel's blip count), and with a
  finite `repeat` the count is `repeat * popcount(channel_mask)` cycles (one pass
  per repeat).

### 6.4 Derived-rate rows

Rate-dependent constants (oscillator increments, sweep growth, fade/window/period
sample counts, channel-ID pitch increments) are **precomputed at apply time** for
44.1, 48, and 96 kHz. The render path then does no `libm` calls and, on RP2040, no
float math. At each block the generator selects the row whose rate exactly matches
the pipeline sample rate; **any rate that is not exactly 44.1/48/96 kHz falls back
to the 48 kHz row.**

---

## 7. Fades and Windows

Two independent envelope mechanisms keep the generator click-free without
distorting measurement timing:

- **Start/stop/config-swap fade.** A 5 ms (`SIGGEN_FADE_MS`) linear ramp is applied
  on start, on stop, and on a config swap while running. It is an **overlay** on
  top of the running engine, not an inserted delay: the fade-in covers the first
  cycle's opening samples and the fade-out covers the last cycle's tail, so a
  sweep's actual frequency-vs-time trajectory is unaffected. The fade floor is at
  least 8 samples.
- **Per-cycle attack/release windows.** Sweeps and walked continuous signals get
  raised attack/release edge windows inside each cycle (attack = release = the fade
  length by default). These smooth the discontinuity at the cycle boundary while
  leaving the interior of the cycle at full level.

Because each cycle resets its per-cycle synth state (sweep phase/increment,
oscillator phases, stepped-sweep position, ISP index, multitone Schroeder phases),
**repeated cycles are deterministic and identical**. This is what makes averaging
multiple sweeps a valid noise-reduction technique for room correction.

Duration-limited continuous signals compute exactly when to begin the tail fade so
it finishes at the requested `duration_ms`.

---

## 8. Notification: `NOTIFY_EVT_SIGGEN_STATE` (0x07)

State changes (start, stop, completion, reconfigure) are pushed on the v2
notification transport (bulk IN EP 0x83, and the UART control transport when its
`notify_enable` is set). The packet is 8 bytes:

```
Offset  Size  Field         Value
------  ----  ------------  ---------------------------------------------
0       1     version       2 (NOTIFY_V2_VERSION)
1       1     event_id      0x07 (NOTIFY_EVT_SIGGEN_STATE)
2       1     flags         0
3       1     seq           monotonic sequence byte (gap = loss)
4       1     state         SiggenState (0 IDLE, 1 FADE_IN, 2 RUN, 3 GAP, 4 FADE_OUT)
5       1     reason        SIGGEN_STOP_* (see below)
6       1     signal_type   SiggenType of the (active or last) config
7       1     channel       walk channel index, or 0xFF when not walking
```

Emission is deferred to the main loop (`siggen_service()` pushes the pending
event), so the render path never calls into the notification code. A `RUN` event
with `reason = NONE` marks a start; an `IDLE` event carries the stop reason.

`SIGGEN_STOP_*` reasons:

| Value | Constant | Meaning |
|-------|----------|---------|
| 0 | `SIGGEN_STOP_NONE` | No stop (start / running) |
| 1 | `SIGGEN_STOP_HOST` | Explicit `CONTROL` stop (STOP or STOP_NOW) |
| 2 | `SIGGEN_STOP_COMPLETED` | Duration or repeat count exhausted |
| 3 | `SIGGEN_STOP_PRESET` | Preset load or factory reset stopped it |
| 4 | `SIGGEN_STOP_RECONFIG` | SET_CONFIG (or START) while running: fade-out for a restart |

---

## 9. Pump: Running Without a Host Stream

`process_input_block()` normally runs whenever an input source delivers samples.
When the generator is running and no source is streaming, nothing would call it and
the outputs would drain to silence. The main loop instead calls `siggen_pump()`
(`audio_pipeline.c`, rule in `siggen_pump.h`), which feeds zero-input blocks
through the full pipeline.

- The pump is paced by the **slot-0 ring fill level** against the output latency
  target (`output_ring_target_frames`, 2 ms by default): a block of
  `output_ring_block_cap` frames is written only while the fill is below the
  target, at most `SIGGEN_PUMP_MAX_BLOCKS` (2) per call so the main loop stays
  responsive. The fill is re-read before every block.
- Every pumped block traverses the full pipeline, so the delay-line write index and
  every output slot advance exactly as for a real source; **inter-slot alignment is
  preserved**.
- The pump refuses to run while the pipeline is owned by someone else
  (`pipeline_source_owns_rings()`, main.c): a source delivering blocks (USB
  stock stream, S/PDIF locked, I2S running, ADAT locked, alink locked or in
  holdover) or the USB phase mode while it keeps a timeline
  (`alink_usb_engaged()`: its hold writes silence between streams); also a
  pending input-source change, an output-type switch in progress, output slots
  not yet set up, and `preset_loading` on a non-USB source (there it is the
  lock/prefill handshake). On USB the pump pumps through `preset_loading`: it is
  the only block source and the mute envelope clears only on blocks.

### 9.1 Timing: sample-exact against the DAC clock

The generator's engine counts samples, not time: a cycle, a gap, a fade and a
walk dwell are integer sample counts (section 6.4), and the engine advances only
when a block is rendered. With the pump as the producer, a block is rendered only
when the DAC's own I2S/S/PDIF DMA has consumed room for it, so the DAC clock alone
paces the generator. Main-loop jitter changes **when** a block is written, never
how many frames the generator emits or where they land in the DAC's frame
sequence. Nothing resamples, and nothing drops or inserts frames while the ring
does not run dry: generator sample *k* after `START` plays at DAC frame
*anchor + k* for the whole run, gaps included (`cycles_done` counts each cycle at
its end, exactly on schedule). Host test: `firmware/tests/test_siggen_timing.c`
(impulses every 137 ms on their exact frames; log sweeps with gaps sample for
sample equal to a back-to-back rendering, walked over 3 outputs, under main-loop
gaps up to 1.9 ms).

**The one slip path** is a ring that runs dry: a main-loop stall longer than the
ring fill (about the output latency, 2 ms by default). `output_rings_pre_give()`
then re-anchors the writer (`deficit + pre-write fill` frames of silence, at least
about one block), so every later generator frame plays that many frames late. It
is never silent: each episode counts as an underrun (`overruns` / buffer stats,
the slot ring's `underrun_frames`), and the inserted frames equal exactly the
counted skip (tested). A measurement that sees no underrun had no slip. Choosing
a longer output latency (4-8 ms) widens the stall margin for long runs.

A block that the output rings refuse at admission (`output_ring_drops`, e.g. a
PDM queue that cannot take it) used to be rendered and then dropped, so the
generator played one block early from then on. Since WP FWSIGGEN the generator
renders only into admitted blocks; a refused block gets `siggen_hold()` (silence
on the generator's channels, no engine time), and its zeros fill the delay-line
slot the rings drop, so the sample-to-frame mapping holds.

With a host stream the stream paces the pipeline (USB async feedback keeps the
stream on the DAC clock; the USB phase mode phase-locks it, alink-style). A
stream that starts while the generator plays restarts the outputs
(`stream_restart_resync_pending`, a pipeline reset): the generator keeps its
schedule but the DAC frame anchor moves. For a timing measurement keep the host
side steady for the whole run: either no stream at all or one stream open from
before `START` to after the last cycle.

Measured on the K340 (2026-11-18, output 3, 5 passes): the one large residual of
the no-stream run (-81.6 us) was the analyser hopping one carrier period, not a
firmware slip (the offline analysis found no step in the recordings); the
remaining smooth +-15-50 us wander is attributed to the USB microphone's own
clock.

### 9.2 Handover

When a real source starts delivering, `pipeline_source_owns_rings()` turns true and
the pump stands down at once; injection continues in the source's blocks. When the
source stops, the pump resumes on the next main-loop pass.

### 9.3 DAC hardware mute ownership

The DAC hardware mute (`dac_hw_mute.c`, `dac_hardware_mute_spec.md`) is held in two
kinds of situation:

- **Reset brackets**: the pin is asserted before a pipeline reset stops the output
  clocks and released after the synchronized restart (plus `release_ms`). These
  always take priority.
- **Idle holds**: the clocks run and the unit only waits for a source. Today this
  is the USB phase mode out of step with no host (`alink_usb.c`
  `dac_hold_update()`, `dac_hw_mute_assert_idle()`), and the mute that
  `complete_pipeline_reset()` keeps for the phase anchor
  (`alink_usb_fw_keep_dac_mute()` marks it idle once the clocks run again).

Before WP FWSIGGEN the generator did not touch the pin. On USB in mode 0 (the
K340 default) the pin is released by every completed reset and nothing asserts it
on stream stop, so with no stream it is open and the generator was audible. In the
USB phase mode out of step with no host the idle hold kept the DAC muted: the
generator played into a muted DAC.

Now the generator owns the pin while it plays (`siggen_dac.h`, glue
`siggen_dac_service()` in main.c, once per main-loop pass):

1. `START` with the pin open (mode 0) starts at once, as before.
2. `START` with an idle hold: the generator sets the override
   (`dac_hw_mute_siggen_override(true)`); the idle hold stays latched but no
   longer drives the pin. The outputs play silence, so the edge is click-free.
   The engine starts once the pin is open, no bracket or release dwell is in
   force, and `max(hold_ms, release_ms)` has passed (the DAC's own un-mute ramp;
   100 ms on the K340), so the first cycle is not played into the ramp. Status
   reports `FADE_IN` while it waits.
3. While it plays, a reset bracket still wins at once: the pin mutes and, because
   the DAC was playing, a full `hold_ms` is re-armed before any clock stop.
4. When the generator goes idle (completed, stopped, preset load), the override
   stays for its tail: one slot ring plus the longest active output delay
   (`SIGGEN_DAC_TAIL_BASE_US` = 15 ms + delay). Then it goes off: an idle hold
   still latched mutes the pin again (the previous behaviour is back); one that
   ended meanwhile leaves it open. Releasing an idle hold that the override has
   open skips the release dwell (the DAC is already playing).

S/PDIF, ADAT, alink and I2S-slave inputs without lock keep their
mute-until-lock: that mute is a reset bracket whose completion is the prefill
handshake, and the pump refuses there (`preset_loading`), so the generator stays
silent on an unlocked non-USB source, as before. Host test:
`firmware/tests/test_siggen_dac.c` (real `dac_hw_mute.c`: open, settle, play,
tail, restore; a hold that ends meanwhile; a bracket during play; a bracket at
`START`; `STOP` while waiting; feature off; test pulse).

---

## 10. Interaction Rules

| Event | Behavior |
|-------|----------|
| **Preset load** | Generator hard-stops immediately via `siggen_stop_immediate(SIGGEN_STOP_PRESET)` (audio is already muted on the preset path, so no fade). Staged config is cleared. |
| **Factory reset** | Same as preset load: hard stop, reason `PRESET`. |
| **Input source switch** | The pump stands down while the switch is pending; the generator's samples keep being injected once a new source streams (or the pump resumes if none does). Alignment is preserved by the same block cadence. |
| **Sample-rate change** | The render path reselects the matching precomputed rate row per block; 44.1/48/96 kHz select their own row, other rates use the 48 kHz row. No reconfigure needed. |
| **Mute / master volume / output trim** | Applied downstream of injection to generator channels (both processed and RAW), so a muted or attenuated output attenuates or silences the generated signal too. Set an output's trim/master volume appropriately before measuring. |
| **Output-type switch (S/PDIF <-> I2S)** | The pump refuses to run while the switch is in progress; injection resumes on the next normal block. |
| **DAC hardware mute** | The generator opens an idle hold itself and restores it after its tail; reset brackets keep priority (section 9.3). |
| **USB stream start while running** | A pipeline reset restarts the outputs: the schedule continues, the DAC frame anchor moves (section 9.1). |
| **PDM / Core 1 EQ_WORKER** | PDM honors `siggen_raw_mask` like the S/PDIF/I2S path. In Core 1 `EQ_WORKER` mode PDM output is inactive (a platform behavior, not siggen-specific), so a masked PDM channel simply produces nothing in that mode. |

Nothing in these paths changes how many samples any slot consumes or produces, so
**no interaction can cause inter-slot drift.**

---

## 11. RAM / CPU Notes

- **BSS.** The generator adds a small amount of BSS (well under 1 KB: the applied
  and staged `SiggenConfig` copies, three 44.1/48/96 kHz derived-parameter rows,
  per-channel RNG and pink-filter state, and the multitone/channel-ID increment
  tables).
- **RAM-resident render kernels.** The synth kernels, planner, and `siggen_render`
  are marked `DSP_TIME_CRITICAL`, so they land in RAM under the XIP build (see
  "Memory Layout" in `current_architecture.md`). Control/config code
  (`compute_derived`, caps, staging) is cold and runs from flash.
- **No `libm` in the audio path.** All transcendental work (log/pow for sweeps,
  Schroeder phases, pitch tables) happens at apply time. The render path is pure
  integer/float arithmetic; on RP2040 it is entirely fixed-point.
- **Cross-core traffic.** The only cross-core datum is `siggen_raw_mask`, written by
  Core 0 between blocks and read by Core 1's per-output EQ gating.

---

## 12. Room-Correction Workflow (sketch)

1. Host (DSPi Console) reads `REQ_SIGGEN_GET_CAPS` to learn the type list, output
   channel count, and parameter ranges for the running platform.
2. For each channel to measure, the host issues `REQ_SIGGEN_SET_CONFIG` with a log
   sweep (`SIGGEN_SWEEP_LOG`), `channel_mask` = the single channel under test,
   `duration_ms` = one sweep, `repeat` = the number of sweeps to average, and a
   safe `level_db`. RAW may be set to characterize the raw electrical/acoustic path
   without the channel's current EQ.
3. Host issues `REQ_SIGGEN_CONTROL` with `START` (`wValue = 1`).
4. Host records the response with the measurement microphone (or, for electrical
   verification, uses the `DSPI_LOOPBACK` build to capture output slot 0 back to
   the host). Because per-cycle synth state resets, repeated sweeps are identical
   and can be coherently averaged.
5. Completion is announced by `NOTIFY_EVT_SIGGEN_STATE` with `state = IDLE`,
   `reason = COMPLETED`; the host can also poll `REQ_SIGGEN_GET_STATUS`
   (`cycles_done`, `current_freq`).
6. The host computes the correction filters and writes them back via the normal EQ
   / crossover vendor commands.

---

## 13. Example Command Sequences

All payloads below are the 36-byte `SiggenConfig` for `REQ_SIGGEN_SET_CONFIG`
(0xA4, OUT `0x41`, `wIndex = 2`), followed by a `REQ_SIGGEN_CONTROL` START
(0xA6, write-as-read `0xC1`, `wValue = 1`, `wIndex = 2`, 1-byte response). Bytes
are shown space-separated in transmission order (little-endian fields).

### 13.1 1 kHz sine at -20 dBFS on channels 0 and 1 (until stopped)

`type = SINE(0)`, `channel_mask = 0x0003`, `level_db = -20.0`, `duration_ms = 0`,
`p1 = 1000.0`:

```
SET_CONFIG (0xA4) payload:
01 00 03 00 00 00 00 00  00 00 A0 C1 00 00 00 00
00 00 00 00 00 00 7A 44  00 00 00 00 00 00 00 00
00 00 00 00
```

Field decode: `version=01 type=00 mask=0003 invert=0000 flags=00 rsv=00`,
`level_db=C1A00000 (-20.0)`, `duration=0`, `repeat=0 gap=0`,
`p1=447A0000 (1000.0) p2=p3=p4=0`.

```
CONTROL (0xA6) START: bmRequestType=0xC1 wValue=0x0001 wIndex=0x0002 wLength=1
```

### 13.2 Out-of-phase mono tone check (1 kHz on ch0+ch1, ch1 inverted)

Same as 13.1 but `invert_mask = 0x0002` (invert channel 1). Summed to mono this
nulls; any residual reveals a polarity or level mismatch:

```
SET_CONFIG (0xA4) payload:
01 00 03 00 02 00 00 00  00 00 A0 C1 00 00 00 00
00 00 00 00 00 00 7A 44  00 00 00 00 00 00 00 00
00 00 00 00
```

(`invert_mask = 0002`; all other fields as 13.1.) Then START as in 13.1.

### 13.3 20 Hz to 20 kHz log sweep, 10 s, on channel 2, repeated 3x

`type = SWEEP_LOG(4)`, `channel_mask = 0x0004`, `level_db = -6.0`,
`duration_ms = 10000`, `repeat = 3`, `p1 = 20.0`, `p2 = 20000.0`:

```
SET_CONFIG (0xA4) payload:
01 04 04 00 00 00 00 00  00 00 C0 C0 10 27 00 00
03 00 00 00 00 00 A0 41  00 40 9C 46 00 00 00 00
00 00 00 00
```

Field decode: `type=04 (SWEEP_LOG) mask=0004`, `level_db=C0C00000 (-6.0)`,
`duration=00002710 (10000 ms) repeat=0003 gap=0000`,
`p1=41A00000 (20.0) p2=469C4000 (20000.0)`. Then START. Watch for the completion
notification (`state=IDLE reason=COMPLETED`) after three sweeps.

### 13.4 Channel-ID walk on all channels

`type = CHANNEL_ID(14=0x0E)`, `channel_mask = 0xFFFF` (clamped to valid outputs),
`level_db = -12.0`, `repeat = 1` (one pass over the mask), `p1 = 120.0` (blip ms).
CHANNEL_ID walks regardless of the WALK flag:

```
SET_CONFIG (0xA4) payload:
01 0E FF FF 00 00 00 00  00 00 40 C1 00 00 00 00
01 00 00 00 00 00 F0 42  00 00 00 00 00 00 00 00
00 00 00 00
```

Field decode: `type=0E (CHANNEL_ID) mask=FFFF -> clamped to valid`,
`level_db=C1400000 (-12.0)`, `duration=0 (unused) repeat=0001 gap=0000`,
`p1=42F00000 (120.0 ms blip)`. Then START. Each channel plays (index+1) pentatonic
blips in turn; count the blips and hear the pitch rise to map the array.

---

## 14. RP2040 vs RP2350 Summary

| Aspect | RP2040 | RP2350 |
|--------|--------|--------|
| Audio path | Q28 fixed-point | IEEE 754 float |
| Output channels | 5 (2 S/PDIF + PDM) | 9 (4 S/PDIF + PDM) |
| `multitone_max` | 8 | 16 |
| Sine kernel | 7th-order poly (Q30), THD approx -139 dB | 7th-order poly (float), same THD |
| Square kernel | polyBLEP band-limited | polyBLEP band-limited |
| Derived rate rows | 44.1 / 48 / 96 kHz | 44.1 / 48 / 96 kHz |
| Wire protocol | identical | identical |

The wire protocol, signal catalogue, timing models, flags, fades, and
notification are identical across platforms; only the internal sample kernels and
the two platform-dependent caps fields (`output_channels`, `multitone_max`)
differ.
