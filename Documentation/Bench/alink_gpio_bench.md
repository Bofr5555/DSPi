# Alink bench: spare Picos wired GPIO to GPIO

*WP LK3, 2026-11-13. Bench build for SYS2 bring-up steps 8(a) and 8(b) without any optical parts.*

The audio link (`Documentation/Features/alink_protocol_spec.md`) is meant for fibre, but its line code is plain 3.3 V NRZ. So
the first real-world tests use bare Pico (RP2040) and Pico 2 (RP2350) boards with jumper wires between their GPIOs. They
run a **bench image**, `alink_bench`, which is not DSPi. It reuses LK1's codec and reference receiver and LK2's
`alink_tx_N` / `alink_rx_N` PIO programs, both unchanged, and adds a line-rate bit checker, a trim scan and impairment
injection. The host tool `tools/alink/bench.py` runs the tests and prints PASS or FAIL.

Contents: [1 What the image does](#1-what-the-image-does) · [2 Clocks and classes](#2-clocks-and-classes) ·
[3 Pins](#3-pins) · [4 Electrical rules](#4-electrical-rules) · [5 Wire length](#5-wire-length-per-class) ·
[6 Diagrams](#6-wiring-diagrams) · [7 An LK5 DSPi receiver on the same wires](#7-a-pico-2-running-the-lk5-dspi-receiver) ·
[8 Flashing](#8-flashing) · [9 Running the tests](#9-running-the-tests) · [10 Console reference](#10-console-reference) ·
[11 Expected results](#11-expected-results-and-what-each-test-proves) · [12 Limits](#12-limits)

## 1. What the image does

One image per chip (`alink_bench-rp2350.uf2`, `alink_bench-rp2040.uf2`). You pick the role at run time over the board's USB
serial console:

| Role | Command | What runs |
|---|---|---|
| TX | `tx L1 48k pins=2,3` | A bench stream on 1 to 7 pins (RP2040: 5). Each pin has an `alink_tx_N` state machine fed by its own DMA channel pair from one shared 16 KB word ring. All the state machines start on the same cycle, across PIO blocks on the RP2350. The payload is PRBS-7 (superframe flag b4) or a fixed test vector (`pay=vec`), and the frame counter runs from `f0`. |
| RX | `rx pin=0` | `alink_rx_N` writes into a 32 KB raw ring through two DMA channels: one moves data, the other reloads the address. This is the ring LK5 will use, so the bench exercises it on hardware first. LK1's reference receiver acquires the stream from a snapshot (lock, frame F, disparity, superframe). From then on a bit error checker regenerates the expected stream and compares **every received bit**. It follows slips of up to 3 bits and declares loss after 16 bad frames. The reference receiver also runs on the live ring as fast as the CPU allows, for the spec 8.9 counters, and counts a gap whenever it falls a ring behind. `scan` measures the sampling trim. |
| LOOP | `loop L1 48k` | TX and RX on one board, with a jumper from GP2 to GP0 (step 8(a)). |
| IMPAIR | `impair in=0 out=2 div=1` | A PIO pass-through re-times a passing stream to this board's clock: one SM clock of quantisation, so `div=d` adds up to d sys cycles of edge jitter. It also corrupts the stream in timed bursts: `impair flip <cycles> <period_ms>` inverts the line, and `impair drop <us> <period_ms>` holds it low. |
| TX impairments | `inject comma 3` | The transmitter damages exact frames, so you can predict the counters. `comma n` damages n commas (the flywheel; 3 in a row means loss). `flip n` flips one payload bit in n frames (CRC-16, HOLD). `slip d` adds or drops \|d\| bits before a comma (1 to 2 bits is a slip, 3 is loss). `idle n` sends n frames as idles. |

Core 1 runs the real-time work: TX ring refill and the bit checker. Core 0 runs the console, acquisition, the reference
receiver, the trim scan and IMPAIR bursts. The image runs from RAM, never writes flash and needs nothing from DSPi.

## 2. Clocks and classes

The class dividers assume a 307.2 MHz system clock: 20, 10 or 5 cycles per bit for L1, L2 or L3. The image boots at
**307.2 MHz with the core at 1.20 V on both chips**. DSPi itself runs the RP2040 at 307.2 MHz too, so a Pico 1 is no worse
off here than in a DSPi. If a Pico 1 is unstable there (it is rated for 133 MHz), use `clock 153` or `clock 76`. The line
rate is `sys_clk / N`, so a lower clock carries the same classes on the faster programs:

| `clock` | L1 15.36 MBd | L2 30.72 MBd | L3 61.44 MBd | Notes |
|---|---|---|---|---|
| 307 (default, both chips) | `alink_*_20`, window ±29.2 ns, trim ±9 | `alink_*_10`, ±13.0 ns, trim ±4 | `alink_*_5`, ±6.5 ns, trim ±1 | the DSPi configuration; one trim step = 3.26 ns |
| 153 (153.6 MHz) | `alink_*_10`, ±26 ns, trim ±4 | `alink_*_5`, ±13 ns, trim ±1 | not possible | a Pico 1 inside its 200 MHz SDK rating; trim step 6.5 ns |
| 76 (76.8 MHz) | `alink_*_5`, ±26 ns, trim ±1 | not possible | not possible | below the RP2040's stock clock; trim step 13 ns |

At its stock 125 or 133 MHz an RP2040 can do no class: 15.36 MBd needs a system clock of N × 15.36 MHz with N = 5, 10 or 20.
A fractional PIO divider would add a sys cycle of jitter to every edge, so the bench never uses one. Both ends of a wire
must run the same class, not the same clock: a Pico 1 at `clock 153` sending L1 and a Pico 2 at 307.2 MHz receiving L1 is
a valid pair.

CPU budget. The checker and generator cost about 1 to 2 cycles per bit on core 1, and LOOP runs both. LOOP at L3 is the
heaviest case (61 Mbit/s), and on an RP2040 core it may not keep up. If it falls behind, `bench_skipped` or `tx_late` in
`stat` rise, and `bench.py` reports FAIL "checker kept up". The reference receiver needs about 40 to 50 cycles per bit,
so at every class it covers only part of the stream: `ref_bits` against `bert_bits`, with `ref_gaps` counting the jumps.
Its counters are exact for the part it covers. The checker covers everything.

## 3. Pins

Pico and Pico 2 have the same header, so these are the same physical pins on both:

| Signal | GPIO | Header pin | Role |
|---|---|---|---|
| alink RX data in | **GP0** | 1 | RX, LOOP (`rxpin=`), IMPAIR input (`in=`) |
| alink TX 1 out | **GP2** | 4 | TX, LOOP (`txpins=`), IMPAIR output (`out=`) |
| alink TX 2-4 | GP3, GP4, GP5 | 5, 6, 7 | extra TX alinks (`pins=2,3,4,5`) |
| alink TX 5-8 | GP6, GP7, GP8, GP9 | 9, 10, 11, 12 | more TX alinks (RP2350 up to 7 alinks, RP2040 up to 5) |
| GND | - | 3, 8, 13, 18, 23, 28, 33, 38 | one GND next to each data pin: pin 3 next to GP0/GP2, pin 8 next to GP5/GP6 |
| LED | GP25 | (on board) | slow 0.5 s blink: TX or IMPAIR running; fast blink: RX searching; steady: RX in sync; brief flash every 2 s: idle |

Any header GPIO (GP0-22, GP26-28) can be chosen with `pins=`, `pin=`, `in=` and `out=`. GP23-25 and GP29 are internal and
refused. GP0 and GP2 match the SYS2 receiver default (GP0, section 6.2) and the step 8(a) jumper (GP2 to GP0). On Pico W
and Pico 2 W the LED is on the wireless chip, so it stays dark, but everything else works.

## 4. Electrical rules

1. **3.3 V logic only.** Never connect VBUS (pin 40), VSYS (pin 39) or any 5 V signal to a GPIO. Never connect two
   boards' 3V3 pins (pin 36) together: each board has its own regulator.
2. **A common ground for every pair.** Every data wire has its own ground wire from the same two boards, twisted with it:
   two jumpers twisted a few turns per 10 cm, or one pair of a ribbon or Cat5 cable. One shared ground wire for a bundle
   of data wires is not enough at L2 and L3.
3. **A series resistor at the driver**, 33-100 Ω, in the data wire right at the transmitting pin. Start with **47 Ω**. It
   damps the ringing of a loose wire (series termination), and it limits the current if two outputs ever meet. The
   receiving end needs nothing: the RX pin is a high-impedance Schmitt input.
4. **Never connect two outputs together.** An output is any pin in `pins=` of a TX, `txpins=` of a LOOP, or `out=` of an
   IMPAIR board. Each wire joins exactly one output to one input. `stop` (and power-up) returns every pin to an input.
   Configure the role before you plug in a wire you are unsure about.
5. **Drive strength** (`drive=2|4|8|12` mA, default 4 mA, fast slew). Raise it only for longer cables at L3. More current
   means faster edges and more ringing.
6. **Powering several boards.** The simplest setup puts every board on **one powered USB hub** on one PC: the boards then
   share the hub's ground, and the console tool sees them all. If boards are powered from different PCs or chargers,
   connect a ground wire between them **before** any data wire. Keep the supplies on the same mains strip. Do not join
   two laptops on separate chargers through the bench ground: their supplies can sit tens of volts apart (Y-capacitor
   leakage) until the ground wire is in. Never feed a board from two sources at once (USB and VSYS).

## 5. Wire length per class

A loose wire is an unterminated transmission line. Propagation is about 5 ns/m. The RP2040/RP2350 GPIO edges are 1 to 3 ns
at 4 mA. Below about 30 cm the wire behaves as a lumped load. Above that, the series resistor has to absorb the
reflections. What eats the receiver's edge window (SYS2 2.7, LK2) is pulse-width distortion: unequal rise and fall times,
ringing that crosses the Schmitt threshold late, and crosstalk between neighbouring wires.

| Class | Bit | Edge window | Twisted pair + ground, 47 Ω at the driver | Single loose jumper, no resistor |
|---|---|---|---|---|
| L1 15.36 MBd | 65.1 ns | ±29.2 ns | up to **2 m** | 20-30 cm |
| L2 30.72 MBd | 32.6 ns | ±13.0 ns | up to **1 m** | 10-20 cm |
| L3 61.44 MBd | 16.3 ns | ±6.5 ns | **30 cm**, and expect a narrow trim range | about 10 cm (the step 8(a) jumper) |

These are starting points, not guarantees. Be honest about L3 at 61 MBd on loose wires: ±6.5 ns is the whole budget for
PWD plus jitter, and one breadboard row of ringing can use it up. The trim scan gives the verdict. A clean range of 3
steps (−1..+1) at L3 is a healthy short wire. A range of 1 step, or a centre that is not 0, means the wiring is eating
the margin. Shorten it, twist it, change the resistor, before you suspect the firmware. A Cat5 pair with 47-68 Ω at the
driver carries L1 and L2 much further than these numbers. The alink is meant for fibre, though, and these wires only
stand in for it on the bench.

## 6. Wiring diagrams

`R` is the series resistor (47 Ω) at the driving pin; every `====` is a data wire twisted with its ground wire.

**1 board: loopback (step 8(a))**, `bench.py loop`:

```
   Pico or Pico 2
  ┌───────────────┐
  │ GP2 (pin 4) ──R──┐   10 cm jumper (L3: keep it this short)
  │ GP0 (pin 1) ─────┘
  │ GND (pin 3)   │      (same board: no ground wire needed)
  └───────────────┘
```

**2 boards: TX to RX (step 8(b) without the optics)**, `bench.py pair`:

```
   board A (TX)                          board B (RX)
  ┌─────────────┐                       ┌─────────────┐
  │ GP2 (pin 4) ─R═══════════════════════─ GP0 (pin 1) │
  │ GND (pin 3) ─════════════════════════─ GND (pin 3) │
  └──────┬──────┘                       └──────┬──────┘
         └────────── one powered USB hub ───────┘
```

**3 boards: TX, IMPAIR, RX**, `bench.py impair`:

```
   A (TX)                  I (IMPAIR, a Pico 1 is fine)        B (RX)
  GP2 ─R══════════════════ GP0 (in)    GP2 (out) ─R══════════════ GP0
  GND ─═══════════════════ GND         GND ─══════════════════════ GND
```

**4 boards: one sender, three alinks, three receivers** (the star of SYS2, on wires). Start the TX with
`tx L1 48k pins=2,3,4` and run `bench.py pair --tx A --rx B`, then `--rx C`, then `--rx D` (one receiver at a time; the
others keep running):

```
                    ┌─ GP2 (pin 4) ─R═══════════════ GP0  board B (RX)
   board A (TX)     │  GND (pin 3) ─════════════════ GND
   tx ... pins=2,3,4├─ GP3 (pin 5) ─R═══════════════ GP0  board C (RX)
                    │  GND (pin 8) ─════════════════ GND
                    └─ GP4 (pin 6) ─R═══════════════ GP0  board D (RX)
                       GND (pin 13) ─═══════════════ GND
```

All four boards go on one powered hub. Because the three alinks start on the same cycle, a scope on GP2/GP3/GP4 shows
identical edges, which is the property the LK4 sender needs.

## 7. A Pico 2 running the LK5 DSPi receiver

The LK5 receiver is part of every RP2350 DSPi build (`DSPI_ALINK`, on by default): input source 7, `INPUT_SOURCE_ALINK`.
It reads the alink on **GP0**, the default of `EXT_ALINK_CONFIG.rx_pin`, which no other DSPi default uses, with the same
Schmitt input as the bench RX. So a DSPi Pico 2 (or a K340's Pico 2, at its GP0 pad) is wired **exactly like board B
above**:

```
   bench TX board (Pico or Pico 2)            DSPi Pico 2 running the LK5 build
   GP2 (pin 4) ─R(47 Ω)══════════════════════ GP0 (pin 1)    = ALINK_RX
   GND (pin 3) ─═════════════════════════════ GND (pin 3)
```

The LK4 alink sender (`alink_sender_bench.md`) can stand in for the bench TX here: its alink 0 is GP2 too. Without it, the bench TX stands in for the sender: `tx L1 48k ch=8 pay=vec` (or L2/L3, 44k1/96k). Its stream is a
valid protocol-v1 stream: superframe, frame counter, CRCs. With `pay=prbs` the superframe carries the test-pattern
flag, and the receiver then **plays silence** (the payload is not audio) while still locking and counting. With
`pay=vec` it plays the fixed vector samples (`bench_vector_sample()`): steady DC levels on all 8 inputs, visible on
the input meters. **Connect no amplifier or speaker to the DSPi outputs** for this test. Power both from one hub, or join the
grounds first (rule 6), because a K340 may share its supply with amplifiers.

The receiver is configured over the extension channel (`extension_channel_spec.md` 2.8). With a USB MIDI build
(`-DDSPI_USB_MIDI=ON`), `tools/midi_ctrl` does it, `DSPI_MIDI_DEV` naming the unit's rawmidi node:

```sh
M="./run.sh midi_ctrl.py"              # in tools/midi_ctrl, DSPI_MIDI_DEV=/dev/snd/midiCxD0 exported
$M ext-list                             # lists 0x8000-0x8006 0x8008 0x8009 0x800a (0x8004-0x8006: LK6)
$M set 0x00 --wvalue 0x8002 --payload 01ff00ffffff0300   # EXT_ALINK_CONFIG: enable, GP0, class auto
$M set 0x00 --wvalue 0x8003 --payload 0001020304050607   # EXT_ALINK_SLOTS: alink channels 0-7 -> inputs 0-7
$M set 0xE0 --u8 7                      # REQ_SET_INPUT_SOURCE: ALINK
$M get 0x00 --wvalue 0x8001             # EXT_ALINK_STATUS (52 bytes): state 2 = locked
$M get 0x00 --wvalue 0x800A             # EXT_ALINK_DIAG: trim scan at offsets 32-39, bench counters 40-55
$M set 0x52                             # REQ_SAVE_OUTPUT_CONFIG: keep the alink settings across reboots
```

`dspi_poke.py` does the same over the vendor interface (`set 0x00 --wvalue 0x8002 --hex "01 ff 00 ff ff ff 03 00"`).
On alink-up the receiver detects the class (about 12 ms per class tried), runs the trim scan (a few hundred frames
per trim step: L1 19 steps, L2 9, L3 3, about 0.1-0.4 s), locks, matches the pipeline to the stream's rate, prefills
and plays. Its STATUS counters and its trim scan should match what a bench RX board reports on the same wire.
With LK6, the default `EXT_ALINK_PHASE` (phase lock, `d_local` 128) needs the 1 ms latency option
(`set 0x03 --u8 2`); under the default 4 ms option 128 is below `d_local_min` and the unit runs the fill servo
(`D_BELOW_MIN`). Two phase-locked receivers: `alink_phase_bench.md`.
A save (`REQ_SAVE_OUTPUT_CONFIG`, a preset save) stalls the main loop longer than the receiver's ring holds (4 ms at
L3, 16 ms at L1), so expect one loss and re-lock per flash write; other main-loop stalls up to those limits must not
show as losses (`Documentation/Features/alink_receiver.md` section 2).

## 8. Flashing

The owner flashes; nothing here is done by an agent. Which file goes where:

| Board | Marking | File | BOOTSEL drive name |
|---|---|---|---|
| Pico (RP2040) | "Raspberry Pi Pico", chip RP2-B2 | `alink_bench-rp2040.uf2` | `RPI-RP2` |
| Pico 2 (RP2350) | "Raspberry Pi Pico 2", chip RP2350 | `alink_bench-rp2350.uf2` | `RP2350` |

A wrong-chip file does no harm: the bootrom ignores blocks for the other family, and the board stays in BOOTSEL.

**Back up first** (once per board, if it holds anything you want back):

1. Hold BOOTSEL, plug in USB, release. Wait about 5 s and run `udevadm settle`. On an RP2350, picotool races the
   kernel's mass-storage probe if it runs too soon.
2. `picotool info -a` (note the serial and what is on it).
3. `picotool save -a backup-<serial>.bin`. This takes the whole flash: 2 MB on a Pico, 4 MB on a Pico 2.
4. Check the file is the full size, then read the flash again and compare:
   `picotool save -a check.bin && cmp backup-<serial>.bin check.bin && echo backup verified`.
   If the comparison fails, do not flash; make a new backup.

**Flash**: with the board in BOOTSEL, copy the `.uf2` onto the drive (`cp alink_bench-rp2350.uf2 /run/media/$USER/RP2350/`).
The board reboots into the bench image. The image only overwrites the first ~64 KB of flash. Everything above that (a
DSPi's saved presets, for example) is left as it was, but you need the backup to get the program back.

**Find the console**: `ls /dev/serial/by-id/` shows `usb-Raspberry_Pi_DSPi_alink_bench_<serial>-if00`, the board's unique
ID, the same as `serial=` in the `id` reply. Label the boards with it.

**Back to normal**: BOOTSEL and copy the board's usual `.uf2`, or restore the backup with
`picotool load -v -x backup-<serial>.bin -t bin -o 0x10000000`. Shortcut into BOOTSEL without the button: open the
console at **1200 baud** (`stty -F /dev/ttyACMx 1200`), which reboots the board into BOOTSEL. `bench.py` never uses that
rate.

## 9. Running the tests

`tools/alink/bench.sh` runs `bench.py` in its container venv (rootless podman, no network). It passes only the tty
devices named on the command line, so the container sees nothing else. Typical session:

```sh
B=tools/alink/bench.sh
$B id /dev/serial/by-id/usb-Raspberry_Pi_DSPi_alink_bench_*                # who is who
$B loop /dev/serial/by-id/...A --cls L1 --rate 48k                        # step 8(a), 11 min = 1e10 bits
$B loop /dev/serial/by-id/...A --cls L3 --minutes 3                       # L3: 1e10 bits in 3 min
$B matrix /dev/serial/by-id/...A --minutes 1 --no-impair                  # every class and rate, 1 min each
$B pair --tx /dev/serial/by-id/...A --rx /dev/serial/by-id/...B --cls L2  # two boards
$B impair --tx ...A --imp ...I --rx ...B --cls L1                         # IMPAIR in the middle
$B cmd /dev/serial/by-id/...B 'diag'                                      # EXT_ALINK_DIAG layout
```

Options: `--minutes`, `--ch`, `--txpin`, `--rxpin`, `--drive`, `--no-scan`, `--no-impair`, `--strict-scan` (on two boards
an off-centre scan is a WARN by default, because it describes the wire), `--unplug` (asks you to pull and replug the
jumper: loss and re-lock on real hardware), `-v` (shows the console traffic). Without the container:
`python3 tools/alink/bench.py ...` works on any Linux host with Python 3.8 or later.

## 10. Console reference

One command per line; replies are single lines that start with a tag and carry `key=value` tokens. Turn on local echo
in a terminal program; the image does not echo.

| Command | Reply |
|---|---|
| `id` | `ID fw=alink_bench ver=<git> chip=rp2350 clock=307200000 serial=<id> max_alinks=7 classes=L1,L2,L3` |
| `help` | `OK` lines listing the commands |
| `clock 307\|153\|76` | stops every role, sets the clock, prints `ID` |
| `tx <L1\|L2\|L3> <44k1\|48k\|96k> [ch=C] [pay=prbs\|vec] [pins=2,3] [f0=N] [sender=N] [drive=4]` | `OK` / `ERR <why>` |
| `rx [pin=0] [cls=auto\|L1\|L2\|L3]` | `OK`; `auto` tries the classes in turn, like the SYS2 2.6 class detection |
| `loop <cls> <rate> [ch=] [pay=] [txpins=2] [rxpin=0] [drive=]` | `OK` |
| `inject comma\|flip\|idle <n>`, `inject slip <-3..3>` | `OK` (TX roles) |
| `trim <t>` | `OK`: forces sampling trim t (RX roles) |
| `scan [dwell_ms]` | one `SCANSTEP trim= sync= clean= errors= bits=` per trim, then `SCAN state= lo= hi= used= mask= cls=`; the trim in use afterwards is `used` |
| `impair [in=0] [out=2] [div=1..255] [drive=]`, `impair flip <cycles> <period_ms>`, `impair drop <us> <period_ms>`, `impair off` | `OK` |
| `stat` | `STAT ...`, below |
| `diag` | `DIAG <128 hex digits>`, the 64-byte `EXT_ALINK_DIAG` layout v1 (extension_channel_spec.md 2.8.11), then `DIAGF` with its fields decoded |
| `xstatus` | `XSTATUS <104 hex digits>`, the 52-byte `EXT_ALINK_STATUS` layout (2.8.2) |
| `reset` | `OK`: clears the checker, TX-impairment and reference counters (the reference receiver restarts, so its `lock` becomes 1 again) |
| `stop` | `OK`: every role off, every pin an input |
| (asynchronous) | `EVT sync cls= rate= ch= frame=`, `EVT loss frame=` |

`STAT` fields (present for the roles that are running):

- **TX:** `tx_cls tx_rate tx_ch tx_pay tx_alinks`; `tx_frame` is the frame now on the wire; `tx_words` is the words
  sent. `tx_late` counts times the DMA caught up with the refill (stale words sent). `tx_stall` counts times a TX FIFO
  ran dry (a stretched bit). `imp_comma imp_flip imp_slip imp_slip_bits imp_idle` count the impairments applied.
- **RX:** `rx_pin rx_cls trim`. `sync`, `frame`, `rate`, `ch`, `sf_flags` and `sender` come from the acquired stream.
  `rx_words` counts ring words, `rx_stall` RX FIFO overflows, and `acq_tries` acquisition attempts.
- **Checker:** `bert_bits` and `bert_errors` (BER = errors / bits). `bert_frames`, `bert_err_frames` and
  `bert_bad_frames` (more than 1/8 of the bits wrong). `bert_skipped` counts frames that left the ring before they were
  compared. `bert_slips` and `bert_slip_bits` count re-alignments of 1-3 bits. `bert_syncs` counts acquisitions taken
  and `bert_losses` losses of sync.
- **Reference receiver** (spec 8.9 names): `lock loss crc8 crc16 slips sf_err comma counter code idle`. `samples_bad`
  counts banks that passed the CRC but decoded to the wrong samples. `ref_bits` and `ref_frames` give its coverage, and
  `ref_gaps` its restarts.
- **LOOP:** `fdelta = tx_frame - frame`, small and steady.
- **IMPAIR:** `imp_in imp_out imp_div imp_flips imp_drops`.

## 11. Expected results and what each test proves

| Test (bench.py step) | Expected | What it proves for the alink |
|---|---|---|
| lock | `sync=1` within about 0.1-0.5 s; the superframe reports the class, rate, C and PRBS flag that were sent | LK2's RX program samples real GPIO edges; LK1's receiver finds commas, heads and a valid superframe on a real stream; the 32 KB two-channel DMA ring works |
| trim scan (LOOP) | L1 `lo=-9..-8 hi=8..9 used=0`, L2 `-4..-3 / 3..4 / 0`, L3 `-1 / 1 / 0`, the same as the LK2 model for a clean line | the trim patch and reload work on hardware; the sampling window is centred, i.e. the GPIO path has no pulse-width distortion worth a step; this is the step 8(a) trim criterion |
| trim scan (two boards) | centred within ±1 on short twisted wires; it moves off 0 and narrows as wires get longer | measures the wire's PWD plus jitter in 3.26 ns steps: the eye opening the PLR237/AFBR parts will later be judged by |
| BER | 0 errors in at least 1e10 bits (11 min at L1, 6 min at L2, 3 min at L3), no slip, no loss, `bert_skipped=0`, no `tx_late`/`tx_stall`/`rx_stall` | the TX bit grid (one instruction per bit, DMA-fed, never starved) and the RX re-anchoring are error-free at line rate on real silicon; the 10^10-bit pass criterion of step 8(a) |
| frame counter | LOOP `fdelta` steady at a few tens of frames; two boards: TX and RX frame counters agree within 5 % of a second after the USB read times are allowed for; frame rate = Fs within 1 % | F travels in the stream and the receiver's counter is the sender's; the counter on which phase lock (LK6) will rest |
| reference counters | all zero during the BER run (for the part it covered) | the spec 8 receiver produces no false errors on a clean real stream |
| comma ×1, ×2 | checker: exactly 1 / 2 bit errors, no loss; reference: `comma` +1 / +2, no loss | the flywheel: a missing comma costs only a counter (spec 8.5) |
| comma ×3 | checker: 3 errors; reference: `comma` +3, `loss` +1, `lock` +1 | loss is 3 consecutive failed frames, and re-lock follows on the next superframe (spec 8.8) |
| flip ×5 | checker: 5 errors; reference: `crc16` +5, no loss | CRC-16 per bank catches single-bit damage; HOLD keeps the frame |
| slip ±1, ±2 | checker: one slip of the right size, at most \|d\| errors; reference: `slips` +1, no loss | the ±2-bit slip window re-aligns without loss |
| slip +3 | checker: one slip of 3; reference: `comma` +3, `loss` +1, `lock` +1 | a slip outside the window is a loss and a re-search, not a silent misalignment |
| idle ×2 / ×20 | ×2: errors, no loss; ×20: checker loss +1 then re-sync, reference `loss` +1 | dark-line behaviour and re-lock after a long gap |
| unplug (manual) | checker loss and re-sync; reference `loss` +1 | the same on a physically broken line |
| IMPAIR div sweep | clean up to `div` about 8 at L1, 3 at L2, 1 at L3 (edge quantisation below the window); errors above | the edge window of 2.7, measured on hardware, with another board's crystal in the path |
| IMPAIR flip bursts | errors, no loss | burst errors are contained to the frames hit |
| IMPAIR dropouts (2 ms) | one checker loss and one re-sync per dropout; reference `loss` per dropout it saw | loss and re-lock through an independent clock domain |

The reference-receiver rows are judged only when the reference receiver covered the window without a gap. Otherwise
`bench.py` reports N/A and the checker's result stands. Coverage is partial at L2/L3 by design (section 2).

Two boards bring real crystal offsets: each Pico's 12 MHz crystal is a few tens of ppm off. The step 8(a) stress of
"TX divider ±300 / ±1000 ppm" cannot be done on one board, because the PIO divider's fraction is 1/256 (3906 ppm per step
at divider 1). Two boards give the real case instead, and the LK2 model, driven through the bench logic in the host
tests, covers ±1000 ppm.

## 12. Limits

- Not an audio path: nothing reaches a DAC, and there is no soft VCXO servo (step 8(b)'s servo lock needs LK5/LK6 code).
- Bench payloads only (PRBS-7 or the fixed vector). The checker regenerates the bench stream, so another sender's stream,
  with applies or live audio, is decoded by the reference receiver but not bit-checked.
- No LOS or TX_DISABLE pins (STATUS `los` reads 0xFF).
- The RP2040 starts alinks on pio0 and pio1 a few cycles apart, a constant skew; the RP2350 starts all blocks on one cycle.
- `scan` and `trim` reload the RX program, which leaves a seam in the raw stream. The checker re-acquires after it, but
  the reference receiver counts what it saw. Run `reset` before a measurement.
- Host tests: `firmware/alink_bench/tests/run.sh` (the bench logic alone, then driven through LK2's cycle-accurate PIO
  model) and `tools/alink/tests/test_bench_tool.py` (the tool against simulated boards and a pty).
