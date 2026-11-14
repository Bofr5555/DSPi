# Scope marker on the shared time base (WP SCOPE1, test builds only)

A GPIO that rises at the same sample of the shared time base on every unit, so a logic analyser or MSO
(the owner's PicoScope 2000A MSO; tool: dspi-unified-console `tools/alink-scope/`) measures the audio offset
between units directly. It is a compile-time option, off in every normal build; `-DRELEASE=ON` refuses it.

## Build

```sh
cmake -S firmware -B build-scope -G Ninja -DPICO_PLATFORM=rp2350-arm-s -DPICO_BOARD=pico2 \
      -DALINK_SCOPE_MARKER_GPIO=2 -DALINK_SCOPE_MARKER_LOCK_GPIO=3        # optional: -DALINK_SCOPE_MARKER_N=4096
cmake --build build-scope            # RP2040: -DPICO_PLATFORM=rp2040 -DPICO_BOARD=pico
```

| Option | Default | Meaning |
|---|---|---|
| `ALINK_SCOPE_MARKER_GPIO` | empty (off) | marker pin |
| `ALINK_SCOPE_MARKER_LOCK_GPIO` | empty (off) | sanity pin: high while phase-locked (USB `EXT_ALINK_USB_STATUS.state` 5 with phase LOCKED; alink phase LOCKED and not in holdover) |
| `ALINK_SCOPE_MARKER_N` | 4096 | marker period in samples: a power of two, 256..32768. 4096 = 11.72 Hz at 48 kHz, 23.4 Hz at 96 kHz |

Both pins are refused as output/UART/I2C pins at run time in such a build (`is_valid_gpio_pin`).
`DSPI_ALINK_DECODE_CORE1=ON` is refused with the marker (the alink timeline would move on the other core).

**Why 4096 and not 4800 (exactly 10 Hz).** On the USB time base two units count the frame number N from origins
that differ by a multiple of 2048 frames, i.e. their S differ by a multiple of 2048 x spf = 98304 samples at 48 kHz
(2^15 x 3; 196608 at 96 kHz) (extension_channel_spec.md 2.9.1). Only a divisor of that marks the same bus instant
on every unit; 4800 does not divide 98304, 4096 does. On the alink time base every receiver sees the same F, so any
N would do; the same N keeps the tool simple.

**GPIO choice.** Default GP2 (marker) and GP3 (lock): Pico/Pico 2 header pins 4 and 5, next to GND on pin 3. No
receiver feature defaults to them (README pin tables: GP5-GP10 outputs/S/PDIF in, GP12 UART TX/ADAT, GP13-GP15
MCK/BCK/LRCLK, GP23-GP25 reserved; elsewhere: GP0 alink RX, GP1 I2S RX, GP11 DAC mute, GP16/17 UART control,
GP18/19 I2C control, GP20-22 S/PDIF RX 2-4, GP26/27 slave clocks on RP2350, GP21 MCK on RP2040). They are only used
by the alink *sender* carrier (alinks 0/1), which does not run this firmware. On the K340 GP2/GP3 are module pads 4/5,
not used by the board. Alternatives: GP22 (free unless S/PDIF RX 4 is enabled; also the
`DSPI_ALINK_USB_GPIO` SOF bench marker, so do not combine), GP28, GP4 (free unless an I2S ADC data line is put
there, as in the PCM1808 example), GP26/GP27 on RP2040.

## Where the edge is, and the fixed offset

1. **Which sample.** The main loop reads, with interrupts off, the time base's `next_w` (S or F of the next sample
   written to the output rings), slot 0's ring counters (`wr_frames`, consumed frames + word remainder, brought up to
   date from the DMA read address) and the time. The sample at the DMA is `S_dma = next_w - (wr_frames - consumed)`:
   the relation the phase servo itself regulates (`e = F_arr - (next_w - fill) - D`). The target M is the next
   multiple of N, and its ring address follows from `f_M = consumed + (M - S_dma)`.
2. **When.** The DMA read address that slot 0's channel holds right after it fetched M's first word is computed.
   About 2 DMA words + 5 us before it is due, a dedicated hardware alarm fires (lowest IRQ priority: USB and DMA
   interrupts preempt it); its handler (RAM) spins on the DMA read address and sets the pin the moment the address
   equals the target. If the spin was preempted around that moment (two timer reads more than 1 us apart), the
   address went past or the spin timed out, that edge is skipped, never placed late: the analysis treats it as a
   missing edge. Resolution: the spin loop (well under 0.1 us), undetected lateness under 2 us (0.1 sample at 48 kHz).
   The main loop lowers the pin half a period later; only the rising edge carries timing.
3. **Fixed offset to the wire.** The DMA is paced by the PIO TX FIFO, joined to 8 words in both output programs
   (`audio_i2s_clkout.pio`, `audio_spdif*.pio`). In steady state the FIFO is full, so the DMA fetches word w the
   moment the PIO pulls word w - 8 into its shift register. I2S has 2 words per frame: M's first (left) word is
   fetched as frame M - 4's left word starts shifting out, so **frame M's left-channel MSB is on the data pin 4 frame
   periods after the edge** (83.3 us at 48 kHz, 41.7 us at 96 kHz; minus up to one BCK, as autopull refills the
   shift register right after the previous word's last bit). S/PDIF has 4 words per frame: **2 frames** (41.7 us at
   48 kHz) to the start of frame M's preamble. Add the DMA's request latency (tens of ns) and, for audio at the
   speaker, the DAC's or S/PDIF receiver's own latency.
4. **So.** The offset is the same on every unit with the same output type (slot 0), rate and DAC, so the time between
   two units' edges is the offset between their audio. Mixing I2S and S/PDIF units: subtract the 2-frame difference.
   Per-output delays (`REQ_SET_DELAY`, `OUT_DELAY` ops) and filter group delay act on the audio before the rings, so
   the marker does not show them: compare units with the same DSP settings.

Cross-check once (RP2350): a CLICK op (`EXT_APPLY_AT_FRAME` time base 2, extension_channel_spec.md 2.9.4) on a
sample that is a multiple of N, with the DAC output on an analog channel, should appear the fixed offset (plus the
DAC latency) after that unit's edge.

## Clock lock without the marker

Probe I2S LRCLK (GP15 by default, BCK + 1) of each unit instead: in **mode 1** (rate lock) all units show the same
LRCLK frequency, with an arbitrary but steady phase between them (a phase that keeps sliding means the rate lock is
not working); in **mode 2** (phase lock) the phase between units is constant and their edges sit within the
measured marker offset. S/PDIF-only units have no LRCLK: use the marker there.

## When edges are absent

No timeline (mode 0 or 1, 44.1 kHz, not yet anchored, another input source): the marker stays low and the lock pin
is low. A `SOF_LOST` or re-anchor stops the edges until the next anchor. Counters (`scope_marker_stats`: edges,
skipped, missed, armed) can be read with a debugger.
