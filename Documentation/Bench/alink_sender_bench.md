# Alink bench: an alink sender (Pico 2) to a bench RX board

*WP LK4, 2026-11-13.  The sender is `Documentation/Features/alink_sender.md`; the RX board and its rules are
`alink_gpio_bench.md` (LK3).*

This checks a spare Pico 2 running the alink sender against a Pico or Pico 2 running the LK3 bench image as RX board,
wired GPIO to GPIO. It covers the sender's bit stream (BER, lock, layout, the trim scan on the wire), its frame counter,
the encoder's cycle cost, RATE scheduling, one-apply-at-a-time, a class change, and the SYNC pin on a scope.

## 1. Images

| File | Board | What it does from power-up |
|---|---|---|
| `dspi_sender_bench-rp2350.uf2` | the sender (Pico 2) | alink 0 (GP2) sends L1 48 kHz 8 channels with the PRBS-7 test pattern; SYNC on GP21 |
| `dspi_sender-rp2350.uf2` | the sender (Pico 2) | no alink sends until a host enables one (`EXT_ALINK_TX`, the spec defaults) |
| `alink_bench-rp2350.uf2` / `alink_bench-rp2040.uf2` | the RX board | the LK3 bench console |

Build the sender with `-DDSPI_ROLE_SENDER=ON` (bench image: also `-DDSPI_SENDER_BENCH=ON`), see
`alink_sender.md` section 1. Flash exactly as `alink_gpio_bench.md` section 8 says: first a backup with `picotool save -a`,
compared with a second read. Then copy the UF2 onto the `RP2350` drive.

## 2. Wiring

The same wiring as two bench boards (`alink_gpio_bench.md` sections 4-6): 3.3 V only, one powered hub for both boards
(or the grounds joined first), 47 Ω at the driver, a ground wire twisted with the data wire.

```
   sender (Pico 2, alink sender)                 RX board (LK3 bench image)
   GP2  (pin 4)  ─R(47 Ω)═══════════════════════ GP0 (pin 1)       alink 0 -> ALINK_RX
   GND  (pin 3)  ─═══════════════════════════════ GND (pin 3)
   GP21 (pin 27) ── scope channel 1 (SYNC, 1 pps)          (optional)
   GP2  (pin 4)  ── scope channel 2 (the alink)             (optional)
```

Lengths as in `alink_gpio_bench.md` section 5: L1 up to 2 m, L2 up to 1 m and L3 up to 30 cm on a twisted pair. Another
sender alink goes the same way: GP3 = alink 1, GP4/GP5 = alinks 2/3 (12 mA drive, meant for Broadcom transmitters, so keep
the 47 Ω), GP6 = 4, GP8 = 5, GP14/GP15 = 6/7. Give `--alink n` on the command line.

## 3. Commands

The ports: the RX board's console is `/dev/serial/by-id/usb-Raspberry_Pi_DSPi_alink_bench_<serial>-if00`. The sender is a
USB device `2e8b:feaa` with the product "DSPi Audio Link Sender". Its node is `/dev/bus/usb/BBB/DDD`, from
`lsusb -d 2e8b:feaa`. If a DSPi is plugged in too, `lsusb -v -d 2e8b:feaa | grep -E '^Bus|iProduct'` tells them apart.
The udev rule that gives DSPi's vendor interface to the user (`60-dspi.rules`, `2e8b:feaa`) covers the sender. `bench.sh` passes
only the named device nodes into its container.

**(a) The bench image, no USB control** (the sender just sends; 11 minutes = 1e10 bits at L1):

```sh
tools/alink/bench.sh sender --rx /dev/serial/by-id/usb-Raspberry_Pi_DSPi_alink_bench_<serial>-if00 --minutes 11
```

**(b) Either image, controlled over USB** (configures alink 0 with PRBS-7, then BER, encoder budget, RATE, apply, class
change):

```sh
RX=/dev/serial/by-id/usb-Raspberry_Pi_DSPi_alink_bench_<serial>-if00
SND=/dev/bus/usb/<BBB>/<DDD>
tools/alink/bench.sh sender --rx $RX --usb $SND --cls L1 --minutes 11
tools/alink/bench.sh sender --rx $RX --usb $SND --cls L2 --minutes 6
tools/alink/bench.sh sender --rx $RX --usb $SND --cls L3 --minutes 3        # 30 cm or less
tools/alink/bench.sh sender --rx $RX --usb $SND --cls L1 --rate 96k --minutes 3
tools/alink/bench.sh sender --rx $RX --usb $SND --cls L1 --rate 44k1 --minutes 3
tools/alink/bench.sh sender-diag --usb $SND                                  # settings, superframe, DIAG
```

`--no-changes` skips the RATE, apply and class-change steps; `--no-scan` skips the trim scan; `--unplug` asks for a
manual unplug. The tool leaves the sender sending PRBS-7 on the chosen alink.

**(c) USB audio (owner's listening rules apply: guarded player, at most 80 % volume).** Play a 6-channel 24-bit
48 kHz file to the "DSPi Audio Link Sender" card. Then set the test pattern to audio, which `bench.sh sender` leaves at PRBS-7,
and read the result:

```sh
tools/alink/bench.sh sender-diag --usb $SND --pattern audio
tools/alink/bench.sh sender-diag --usb $SND                  # again after a few seconds
```
 Expected in `diag`: `usb_playing` 1,
`fifo_fill` near 96 frames, `usb_underruns` and `usb_overruns` 0 after the first second, `feedback_10_14` within a few
ppm of 786432 (48 kHz). The RX board cannot check audio bit for bit, because its checker knows only PRBS-7 and its
fixed vector, so `stat` shows `sync=0` there. That is expected.

## 4. Expected results

| Step | Expected | Proves |
|---|---|---|
| sender firmware / EXT_ALINK_TX written | role 2, IDs `0003 8000 8001 8002 8005 8007 8008 8009 800A`; the IN form shows what was written | the vendor interface and the alink block |
| lock | `sync=1` within 1 s; layout `L1 48k C=8 flags=0x10` | the sender's stream decodes with LK1's receiver on real edges |
| trim scan | centred within ±1 on a short twisted pair (a WARN, not a FAIL, on two boards) | the edge timing on the wire |
| BER | 0 errors in 1e10 bits, no slip or loss, `tx_late=0 tx_stall=0` | the encoder (bit-identical to LK1's), the DMA ring and the in-sync start at line rate |
| frame counter | the sender's F and the RX board's F agree within the transit (tens of frames) | F travels in the stream; the sender's wire tracking |
| encoder within budget | L1 48 kHz C8: about 1000-1300 cycles per frame of 6400, worst under 3200; L2 C16: about 1600-2100; L3 C32: about 2800-3600 | the encoder cost on silicon against the M33 model (`alink_sender.md` 4) |
| RATE scheduled | op 6 at a multiple of 32, 2400-2431 frames after F, value = the new C (3 for L1 at 96 kHz); a second change refused | SYS2 3.3 and 2.8.8 |
| receiver follows | the RX board re-syncs at the new rate and C within a second (its checker stops at the apply and acquires after it) | receivers switch at that frame |
| CTRL_MSG apply | carried, a second one refused, gone once F reaches it | 2.8.9 |
| class change | the RX board (in `cls=auto`) locks at the other class within a few seconds | the restart at the new line rate |
| scope (SYNC) | a 10 us pulse once a second on GP21, its rising edge on the first bit of a comma on GP2 | 1 pps for the latency measurement (SYS2 3.5) |
