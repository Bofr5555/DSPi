# Link receiver: INPUT_SOURCE_LINK on DSPi

*Status: implemented by WP LK5 (2026-11-13), fixups LK45F after review lk45. RP2350 builds with `DSPI_LINK` (on by
default). Host-tested and emulated, not yet run on hardware.*

The receiver decodes the multi-unit link (`link_protocol_spec.md`) on one GPIO and feeds its 8 slots to the pipeline's
8 input channels, like the 8-channel ADAT input. The code is `firmware/DSPi/link_input.c` (its header describes
link-up, the clock and the decoder placement) and `firmware/DSPi/link/` (the decoder `link_fast`, bit-exact with the
reference receiver). It is configured and observed through the link block of the extension channel
(`extension_channel_spec.md` 2.8). Bench wiring and steps are in `Documentation/Bench/link_gpio_bench.md` section 7.

## 1. What plays

- The unit's 8 slots (`EXT_LINK_SLOTS`) go to the 8 input channels, each through its input preamp. A slot at or above
  the stream's C plays silence.
- **A muted or test-pattern stream plays silence.** While the superframe's flags show system mute (bit 0) or a test
  pattern (bit 4: the payload is PRBS-7, not audio, and would reach amplifiers at full scale), the receiver stays
  locked, follows the rate, counts and reports as usual (`EXT_LINK_STATUS.sf_flags` shows the bits), but its inputs
  play silence. Audio returns with the first superframe that clears the bit. A bench PRBS test therefore loses
  nothing, and the outputs stay quiet. Accepted by the coordinator 2026-11-13.
- A loss stops delivery at once and the main loop mutes, then goes round the drain, prefill and in-sync start again
  once the receiver re-locks, as for ADAT. A `RATE` in the stream does the same around the rate change. A failed bank
  holds its last samples for one frame, then plays zeros (the protocol's rule).

## 2. Main-loop stalls

The decoder reads the RX ring in place. The ring holds 32 KB of line bits, so the decode service must run at least
once per ring pass. A longer gap restarts the decoder: a loss, then re-lock (a few ms), mute and prefill.

| Class | Ring pass | Longest service gap that rides through |
|---|---|---|
| L1 (15.36 MBd) | 17.1 ms | 16.0 ms |
| L2 (30.72 MBd) | 8.5 ms | 8.0 ms |
| L3 (61.44 MBd) | 4.3 ms | 4.0 ms |

The limit is 15/16 of a pass (`link_ring_clock.h`): the position count, the DMA write index plus counted wraps, stays
exact while passes are less than one ring pass apart, and the bits the decoder has not read yet are overwritten soon
after that anyway, so no software change can ride through more than the ring. Before LK45F the limit was half a pass
(2.1 ms at L3). The host test `test_link_ring_clock` checks the wrap count at every class with the line up to 1000
ppm fast, and `test_link_fast` checks that a gap at the limit costs no frame.

**Bench expectation.** A flash write (preset save, `REQ_SAVE_OUTPUT_CONFIG`) stalls the main loop for longer than a
pass at every class (a sector erase takes tens of ms), so the link loses lock and re-locks around every save. That is
expected, not a decoder fault: the flash write resets the pipeline for every input anyway (mute, prefill), and ADAT
re-acquires the same way. Other stalls up to the table's limits must not show as losses (`loss_count`).

## 3. Clock

The sender owns the clock. The receiver measures the wire rate from the RX DMA position against the timer and feeds
it to `input_servo_apply()` as the S/PDIF and ADAT slave inputs do: 32 ms windows smoothed, then an 8-16 s
dual-anchor window. The position and the time it was read are published together by whoever ran the decode
service, and the measurement uses that pair, never the poll's own time. That matters for the Core-1 option (section
5), where core 1 publishes at its own moments; with the poll's time every 32 ms window was off by up to a few percent
and the servo rejected it (review lk45 finding 1). `test_link_ring_clock` checks both placements: under 100 ppm per
fast window and under 2 ppm for the long window, where the old pairing was off by more than 14000 ppm.

## 4. Decoder speed and placement

- **In RAM:** the steady state at 48 and 96 kHz with the slots in one bank (the M33 assembly clean loop and full-bank
  routine, with the CRC-16 by the DMA sniffer), the decode service and the delivery.
- **From flash (XIP):** the per-frame decode of a **44.1 kHz** link and of a unit whose slots span **two or more
  banks**, or decode a partial bank (C not a multiple of 8 in the unit's bank). `link_fast.c` makes that C path cold on
  M33 builds to save RAM. Correctness is unaffected; speed on XIP cache misses is unmeasured (the emulated cycle
  figures had that code in RAM). Measure with the bench image's `decode canned L3 44k1` if 44.1 kHz matters.
- **CRC mode:** at start the receiver self-tests the DMA sniffer against the table CRC. If it fails, full banks are
  decoded by the C `bank8()` from flash: slower, still correct. Only the UART log of a `DSPI_DEBUG_PRINTF` build says
  which mode was chosen (`LINK RX: CRC-16 by the DMA sniffer` or `... table CRC`); `EXT_LINK_DIAG` does not.
- The gate is about 10 % of core 0 for L3, 48 kHz, one bank (review lk45: credible range 540-640 cycles of 6400).
  The bench's `decode all` decides.

## 5. Core-1 option (`DSPI_LINK_DECODE_CORE1`)

Off by default and in every shipped build (coordinator 2026-11-13). With it, the Core 1 EQ worker runs the decode
service's clean frames while it waits for work, and core 0 takes the decoder back for anything else. LK45F made it
safe to switch on when it is needed: the rate measurement uses the published pairs (section 3), and a long gap seen
by core 1 is left for core 0 to handle (core 0 restarts the decoder). It builds and passes `check_ram_placement`; it
has never run on hardware.

## 6. RAM

The link's RAM-resident set is about 5.4 KB. `.data` against the budget of 100352 B in `check_ram_placement.py`
(after LK45F):

| Build | `.data` | Headroom |
|---|---|---|
| RP2350 default | 98776 | 1576 |
| RP2350 K340 (`DSPI_USB_MIDI`, 1.20 V) | 98896 | 1456 |
| RP2350 `DSPI_LINK_DECODE_CORE1` | 99256 | 1096 |
| RP2350 `DSPI_LINK=OFF` | 93456 | 6896 |

The budget stays as it is (coordinator 2026-11-13). So later link work must fit about 1.5 KB of hot code on the
default and K340 builds:

- **LK6** (phase lock): keep the hot part small (the apply-at-frame executor and the output-position sampling; the
  servo tick and events stay in flash like `input_servo`). Store `EXT_LINK_PHASE` in the 7 reserved bytes of the V26
  link block (`flash_storage.c`) rather than bumping the directory to V27.
- **LK8** (repeater): a per-frame re-encoder in RAM (the sender's `snd_frame` is about 2-3 KB with its glue) does not
  fit. It must run from flash or live in a separate image; LK8 decides which.

## 7. Loss reasons

`0xE1` reason `COMMA_LOST` (4) means the frames that failed since the last poll that saw the decoder locked were all
flywheel frames (comma missing, no bad head); anything else is `FRAME_ERRORS` (3). Before LK45F the counters since
lock were used, so one head error early on made every later loss read `FRAME_ERRORS`.
