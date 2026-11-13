# Extension Channel (`REQ_EXTENSION`, 0x00)

*Last updated: 2026-11-12*
*Status: implemented on the bofr fork, HW-untested; the link block (2.8) is specified, not implemented*
*Platforms: RP2040 + RP2350*

The vendor opcode space is almost full, and features developed outside Weeb Labs should not claim opcodes that upstream may assign later. The extension channel puts all of them behind a single opcode, `0x00`, and selects the feature with `wValue`. The fork's owner is asking Weeb Labs to reserve `0x00` for this purpose.

---

## 1. Request format

`REQ_EXTENSION` is an ordinary vendor request on the vendor interface, the same as every other command in `Documentation/commands.md`:

| Field | Value |
|---|---|
| `bmRequestType` | `0xC1` (IN, vendor, interface) to read; `0x41` (OUT, vendor, interface) to write |
| `bRequest` | `0x00` |
| `wValue` | extension sub-command ID (uint16) |
| `wIndex` | the vendor interface number, as for every command (`2`) |
| `wLength` | IN: reply buffer size. OUT: the exact payload length of the sub-command |

The direction comes from `bmRequestType`: an IN request reads the sub-command's value and an OUT request writes it. Multi-byte values are little-endian.

### 1.1 Refusals

The device refuses the request in these cases:

- the sub-command ID is unknown;
- the sub-command has no form for that direction (for example an OUT `EXT_LIST`);
- an OUT payload is not exactly the sub-command's length;
- the sub-command itself rejects the payload (for example an invalid core-voltage request; see 3.2).

Over USB a refusal is a **STALL**, the same as an unknown opcode. Length and ID problems STALL at the SETUP stage. A rejected payload STALLs the status stage. Over UART, I2C and USB MIDI the device answers with status `ERROR`.

Stock firmware has no `0x00` handler. An IN `0x00` request therefore STALLs (or gets `ERROR` on UART / I2C / MIDI), and a client should read that as "no extensions". Detecting support only needs one `EXT_LIST` read.

IN replies are truncated to `wLength` like every other DSPi GET. On UART / I2C / MIDI a `wLength` of 0 means "the whole reply".

### 1.2 Sub-command ID ranges

| Range | Use |
|---|---|
| `0x0000` | `EXT_LIST` (discovery) |
| `0x0001`-`0x7FFF` | Features coordinated with Weeb Labs. Every assigned ID is listed in section 2. An ID is never reused for a different meaning. |
| `0x8000`-`0x80FF` | This fork's link block (2.8), assigned by the fork; an ID is never reused for a different meaning. Fork firmware without the link and stock firmware refuse them. |
| `0x8100`-`0xFFFF` | Experimental or private. Never shipped in a release build; hosts must not depend on them. |

## 2. Sub-commands

| ID | Name | IN (read) | OUT (write) |
|---|---|---|---|
| `0x0000` | `EXT_LIST` | supported IDs as uint16 LE, ascending, `0x0000` itself excluded, truncated to `wLength` | none (STALL) |
| `0x0001` | `EXT_CROSSFEED_PAIR_MAP` | `uint8[4]` live map, normalized | `uint8[4]` map (exactly 4 bytes) |
| `0x0002` | `EXT_CORE_VREG` | `CoreVregStatus`, 16 bytes | `{op, value}` (exactly 2 bytes) |
| `0x0003` | `EXT_NOTIFY_FEATURES` | `uint32` LE notification-semantics flags | none (STALL) |
| `0x0004` | `EXT_IDENTIFY` | `IdentifyStatus`, 56 bytes | `IdentifyRequest` (exactly 4 bytes) |
| `0x0005` | `EXT_RESET_INFO` | `ResetInfo`, 20 bytes | none (STALL) |
| `0x0006` | `EXT_FIRMWARE_FIXES` | `uint32` LE fixed-bug flags | none (STALL) |
| `0x8000`-`0x800A` | link block | section 2.8; which IDs and directions depend on the build (receiver or sender) | section 2.8 |

Current firmware answers `EXT_LIST` with `01 00 02 00 03 00 04 00 05 00 06 00`.

### 2.1 `EXT_LIST` (0x0000)

The IN-only discovery read. A host can read 64 bytes (room for 32 IDs) and use the reply length to count the IDs. On stock firmware this read STALLs.

- **Order.** The IDs come in ascending order, so a link build lists `0x8000`-`0x800A` after `0x0006`. A receiver build answers `01 00 02 00 03 00 04 00 05 00 06 00 00 80 01 80 02 80 03 80 04 80 05 80 06 80 08 80 09 80 0a 80` (16 IDs, 32 bytes); a sender build lists the IDs of the table in 2.8.
- **The list is the contract.** An ID that is not listed STALLs (`ERROR` on UART / I2C / MIDI) in both directions, including the link IDs on firmware without the link. A host never probes unlisted IDs, and it reads a feature's own flags (`EXT_NOTIFY_FEATURES`, `EXT_LINK_INFO.features`) before relying on the details.
- **Limit.** The firmware builds the reply in its 64-byte response buffer, so the list holds at most 32 IDs. Adding a 33rd needs a bigger buffer first.

### 2.2 `EXT_CROSSFEED_PAIR_MAP` (0x0001)

This command chooses which two outputs each crossfeed pair processes. It exists for boards whose left and right outputs are not on neighbouring slots: the K340 has one DAC per ear, so its L/R pairs are outputs 2/0 and 3/1. It was opcode `0x02` before the extension channel and keeps exactly the same semantics.

- **Encoding:** byte `p` describes pair `p`. `0x00` means the default pairing (`2p`, `2p+1`). Any other value is `(right << 4) | left` (`crossfeed_map.h`).
- **Valid maps:** every pair gets two distinct stereo outputs below `2 * NUM_SPDIF_INSTANCES`, never the PDM sub, and no output belongs to two pairs. That keeps the pair mask (`0xFC` / `0xFD`) independent of the map.
- **OUT:** sets the whole map at once. A valid map is normalized (explicit default bytes become 0) and stored in one 32-bit write. An invalid map is **ignored**: the request is ACKed and nothing changes, like an out-of-range crossfeed preset. Read the map back to confirm.
- **Notification:** a successful write sends `PARAM_CHANGED` for `WireBulkParams.crossfeed.pair_map` (bulk offset 44, 4 bytes).
- **IN:** returns the live map, normalized. The default map reads as `00 00 00 00`.
- **Persistence:** the map is also carried in the bulk parameters (`WireCrossfeedParams.pair_map`, the former `reserved2`) and in preset slots, in the former `tube_reserved_f` bytes (slot version V39 unchanged).
- **Example (K340):** woofers L=2 / R=0 and tweeters L=3 / R=1 give the map `02 13 00 00`.

### 2.3 `EXT_CORE_VREG` (0x0002)

This command manages the core voltage at the fixed 307.2 MHz clock (`core_vreg.h`). It replaces the old opcodes `0x4F` (SET) and `0x6F` (GET).

**IN** returns `CoreVregStatus`, 16 bytes, little-endian:

| Offset | Size | Field | Meaning |
|---|---|---|---|
| 0 | 1 | `live_vreg` | voltage currently programmed (vreg code) |
| 1 | 1 | `stored_sel` | persisted selection; 0 = build default |
| 2 | 1 | `default_vreg` | build default (`DSPI_VREG_MV`), which is also the floor |
| 3 | 1 | `flags` | bit 0 `AUTO_RAISED` (the firmware raised the stored voltage itself), bit 1 `AUTO_DISABLED` (the user opted out of automatic raising) |
| 4 | 1 | `hang_count` | watchdog hangs in this power session that have not been acted on yet |
| 5 | 1 | `trial_active` | 1 while a trial voltage is applied |
| 6 | 1 | `trial_vreg` | the trial voltage (valid while `trial_active` is 1) |
| 7 | 1 | `failed_trial_vreg` | 0 = none; otherwise the trial voltage whose run ended in a hang |
| 8 | 4 | `failed_trial_uptime_s` | how long that trial ran before the hang |
| 12 | 4 | `uptime_s` | uptime of the current run |

Vreg codes are the same on both chips: `0x0C` = 1.15 V, `0x0D` = 1.20 V, `0x0E` = 1.25 V, `0x0F` = 1.30 V.

**OUT** takes `{op, value}`:

| op | Name | value | Effect |
|---|---|---|---|
| 0 | `SET` | 0 = build default, or a vreg code in [build default, `0x0F`] | Persisted (flash directory). Applied at once, glitch-free. Clears `AUTO_RAISED` and the hang count. |
| 1 | `TRIAL` | a vreg code in [`0x0C`, `0x0F`] | RAM only. A hang or reboot returns to the persisted voltage, and the next boot reports the failure in `failed_trial_*`. |
| 2 | `TRIAL_END` | ignored | Return to the persisted voltage. |
| 3 | `AUTO` | 1 = enable automatic raising (the default), 0 = disable | Persisted. |
| 4 | `CLEAR` | ignored | Forget the hang count and the last trial failure. |

- **Refused (STALL / `ERROR`):** an unknown op, a voltage outside 1.15-1.30 V, or a `SET` below the build default. A device built with `DSPI_VREG_MV=1200` (K340) never boots below 1.20 V, even if an earlier build stored 1.15 V.
- **Automatic raising:** after two watchdog hangs in one power session the device raises its stored voltage by one step (+50 mV), up to 1.25 V.

### 2.4 `EXT_NOTIFY_FEATURES` (0x0003)

IN only: a `uint32` (little-endian) of flags that describe how the firmware fills in notifications. The notification packets themselves do not change; the flags tell a host which rules it may rely on. Reading this needs no new opcode, and an older build simply does not list `0x0003`.

| Bit | Name | Meaning when set |
|---|---|---|
| 0 | `NOTIFY_FEAT_BULK_SOURCE` | A bulk SET's `BULK_INVALIDATED` carries the SET's dispatch source (`HOST_SET` for USB vendor, `UART`, `I2C`, `MIDI`) instead of `BULK_SET`. An invalidation merged from causes with different sources carries `UNKNOWN`. A pending invalidation absorbs a new one only while nothing is queued after it. See `notification_protocol_v2_spec.md` §3.5.1. |
| 1 | `NOTIFY_FEAT_CLIP_EVENT` | The device sends `NOTIFY_EVT_EXT_CLIP` (`0xE0`) when a clip flag sets (section 2.4.1). |
| 2 | `NOTIFY_FEAT_LINK_STATE` | The device sends `NOTIFY_EVT_LINK_STATE` (`0xE1`) on link lock, loss and re-anchor (2.8.12). Set only by link builds. |
| 3 | `NOTIFY_FEAT_APPLY_RESULT` | The device sends `NOTIFY_EVT_APPLY_RESULT` (`0xE2`) for each completed apply-at-frame op (2.8.12). Set only by receiver builds with `APPLY_AT_FRAME`. |
| 4-31 | reserved | zero |

A host that sees bit 0 can recognise its own bulk write from the source byte and drop the time-window echo heuristic it needs for older firmware.

**Fork event IDs.** Event IDs `0xE0`-`0xEF` are the fork's, taken from the top of the discrete range because upstream assigns IDs upward from `0x01`. Each one is announced by a bit here, and a host decodes it only when that bit is set, so a later upstream use of the same ID on stock firmware cannot be misread.

#### 2.4.1 `NOTIFY_EVT_EXT_CLIP` (0xE0)

Sent on every notification consumer (USB EP 0x83, UART, USB MIDI) when one or more clip flags set. 12 bytes:

| Offset | Size | Field | Meaning |
|---|---|---|---|
| 0 | 1 | `version` | 2 |
| 1 | 1 | `event_id` | `0xE0` |
| 2 | 1 | `flags` | 0 |
| 3 | 1 | `seq` | the v2 sequence number |
| 4 | 1 | `channel` | lowest channel in `mask` |
| 5 | 1 | `count` | number of channels in `mask` |
| 6 | 2 | reserved | zero |
| 8 | 4 | `mask` | uint32 LE, the channels that clipped since the previous clip event |

- **Bit space:** the same as the polled clip flags (`REQ_CLEAR_CLIPS`, `REQ_GET_STATUS` 9): inputs first, then outputs (`CH_OUT_1` = `NUM_INPUT_CHANNELS`), the PDM sub last. RP2040 has 7 channels (2 in + 5 out), RP2350 17 (8 in + 9 out).
- **Rate limit:** at most one event per 250 ms (`NOTIFY_CLIP_MIN_INTERVAL_MS`). Clips in between collect into the next event. A sustained clip therefore gives one event every 250 ms, and if no consumer drains the endpoint the newest unsent clip event absorbs the new channels instead of queueing more, so a clip storm cannot fill the ring.
- **Polling is unchanged.** The event has its own latch, set by the same metering code. The polled flags stay sticky and are still cleared only by `REQ_CLEAR_CLIPS`; a clear does not affect the event, and the event does not clear the flags.
- **Precision:** the latch is taken from the main loop. A clip on a core-1 output in the few cycles between the load and the store of that take is lost for its block; a clip that lasts longer than one audio block is always reported.

### 2.5 `EXT_IDENTIFY` (0x0004)

Finds a unit on the bench or in a rack: the on-board LED blinks fast for a while, and on request one short quiet tone plays on one output. The IN form also says what the unit is (`identify.h`).

**OUT** takes `IdentifyRequest`, exactly 4 bytes:

| Offset | Size | Field | Meaning |
|---|---|---|---|
| 0 | 1 | `seconds` | blink time; 0 stops a running blink; above 120 is shortened to 120 |
| 1 | 1 | `tone_output` | output index for the tone, or `0xFF` for no tone (the default a host should send) |
| 2 | 2 | reserved | zero |

- **LED:** GPIO25, the Pico / Pico 2 module LED (the K340 board has no LED of its own; the module's is the one to look for). It blinks at 4 Hz (125 ms on, 125 ms off) instead of the loop-rate heartbeat, and the heartbeat resumes afterwards. A new request replaces a running one.
- **Tone (off unless asked for):** a 1 kHz sine, -40 dBFS peak, 400 ms, through the test signal generator in normal mode, so the output's crossover, PEQ, gain, master volume, mute and limiter all apply. It is never louder than a -40 dBFS sine of program material on that output, and a tweeter output gets its high-pass as usual. It replaces the generator's configuration (read it back before restarting a measurement), sends the usual `SIGGEN_STATE` notifications, and ends by itself.
- **Refused (STALL / `ERROR`):** `tone_output` neither `0xFF` nor below the output count, nonzero reserved bytes, or a tone while the signal generator is running (an identify never takes over a measurement). A blink-only request is never refused.
- Applied from the main loop within one iteration; nothing is persisted.

**IN** returns `IdentifyStatus`, 56 bytes, little-endian:

| Offset | Size | Field | Meaning |
|---|---|---|---|
| 0 | 1 | `remaining_s` | blink time left, rounded up; 0 = not blinking |
| 1 | 1 | `tone_output` | output of the last tone asked for since boot, `0xFF` if none |
| 2 | 1 | `chip` | 0 = RP2040, 1 = RP2350 |
| 3 | 1 | `chip_rev` | the SDK's `rp2040_chip_version()` (1-3 = B0-B2) or `rp2350_chip_version()` (2 = A2; A3/A4 report what the SDK decodes) |
| 4 | 4 | `chip_id` | the raw `SYSINFO.CHIP_ID` register |
| 8 | 16 | `board` | the build's `PICO_BOARD` (`pico`, `pico2`), NUL-padded |
| 24 | 32 | `hw_rev` | the build's `DSPI_HW_REV` string, NUL-padded, at most 31 characters; empty when the build does not set it |

`hw_rev` is a build-time string: a board build sets it with `-DDSPI_HW_REV="..."`. The firmware cannot sense the board revision itself, so the string is only as right as the build that set it.

### 2.6 `EXT_RESET_INFO` (0x0005)

IN only: why the unit last reset and how long it has been up (`reset_info.h`). The cause registers are latched first thing in `main()`.

**IN** returns `ResetInfo`, 20 bytes, little-endian:

| Offset | Size | Field | Meaning |
|---|---|---|---|
| 0 | 1 | `reason` | see the table below |
| 1 | 1 | `flags` | bit 0 `PREV_UPTIME`: `prev_uptime_s` is valid |
| 2 | 1 | `hang_count` | the same value as `CoreVregStatus.hang_count` (2.3) |
| 3 | 1 | `chip` | 0 = RP2040, 1 = RP2350 |
| 4 | 4 | `uptime_s` | this run's uptime |
| 8 | 4 | `prev_uptime_s` | how long the run before this reset lasted, in 1 s steps (the core-voltage uptime breadcrumb); 0 when bit 0 is clear |
| 12 | 4 | `chip_reset` | raw chip reset-cause register at boot: `VREG_AND_CHIP_RESET.CHIP_RESET` (RP2040) or `POWMAN.CHIP_RESET` (RP2350) |
| 16 | 4 | `watchdog_reason` | raw `WATCHDOG.REASON` at boot (bit 0 timer, bit 1 force); bit 8 set when the SDK marker of our own `watchdog_enable()` was present |

| `reason` | Name | RP2040 | RP2350 |
|---|---|---|---|
| 0 | `UNKNOWN` | no known cause bit | no known cause bit |
| 1 | `POWER_ON` | `HAD_POR`: power-on **or brown-out**; the RP2040 does not tell them apart | `HAD_POR` |
| 2 | `BROWNOUT` | never | `HAD_BOR` |
| 3 | `RUN_PIN` | `HAD_RUN` | `HAD_RUN_LOW` |
| 4 | `WATCHDOG` | our 8 s watchdog fired: a hang (also counted in `hang_count`) | same |
| 5 | `SOFTWARE` | a requested watchdog reboot: host-requested reboot, the BOOTSEL round trip | same, plus the bootrom starting the app after a UF2 copy, and any other watchdog reset in `CHIP_RESET` |
| 6 | `DEBUGGER` | `HAD_PSM_RESTART` | `HAD_DP_RESET_REQ`, `HAD_HZD_SYS_RESET_REQ`, `HAD_RESCUE` |
| 7 | `GLITCH` | never | `HAD_GLITCH_DETECT` |

- **Order:** a set `WATCHDOG.REASON` bit decides first (it is zero after any hardware reset, while the RP2040's `CHIP_RESET` keeps showing the power-on that preceded a watchdog reset). Our own watchdog timeout (bit 8) is `WATCHDOG`, any other watchdog reset `SOFTWARE`. Otherwise the cause bits are taken in the order of the table.
- **Power-on and brown-out clear the scratch registers**, so `PREV_UPTIME` is clear and `hang_count` starts at 0 after them. After a watchdog or software reset `prev_uptime_s` says how long the previous run lasted.
- **STM32:** the tree has no STM32 build; the platforms are RP2040 and RP2350.
- The raw registers are there for diagnosis; a host should show `reason` and keep the raw words for a support report.

### 2.7 `EXT_FIRMWARE_FIXES` (0x0006)

IN only: a `uint32` LE of fixed firmware bugs a host may rely on (`config.h` `FW_FIX_*`, the value `FW_FIXES`). A host that would otherwise work around a bug checks the bit first. A host reads this only when `EXT_LIST` names `0x0006`; when it is not listed, the read fails, or the bit is clear, the host must assume the bug is present.

| Bit | Name | Meaning |
|---|---|---|
| 0 | `FW_FIX_BULK_DB_POWF` | A bulk SET (`0xA1` / `0xA3`) converts preamp, crosspoint and output gains from dB with `powf`. Older firmware, and upstream, used a 4-term Taylor series there: -60 dB played at about +35 dB, -40 dB at +19.5 dB, -20 dB at -6.3 dB, until the next preset load or reboot. |
| 1-31 | reserved | zero |

Current firmware answers `01 00 00 00`.

### 2.8 Link block (`0x8000`-`0x80FF`)

*Status: specified 2026-11-12 (WP LK7). The sender side is implemented by WP LK4 (`firmware/DSPi-Sender`, `Documentation/Features/link_sender.md`; host-tested, not yet run on hardware); the receiver side comes with WP LK5/LK6. Firmware at de9614c lists none of these IDs and refuses all of them. The design is in the SYS2 link spec, section 5.3; `link_protocol_spec.md` (WP LK1) will hold the line protocol.*

These sub-commands configure and observe the multi-unit audio link. One sender (the master clock) sends an 8b/10b frame stream to optical receivers, and every receiver plays link frame F a configured number of frames, D, after F arrives. The IDs are in this fork's private range (1.2), so upstream will not assign them. Everything below applies to every link ID.

- **Discovery.** A host uses the link only when `EXT_LIST` names `0x8000`. It then reads `EXT_LINK_INFO` and checks `role` and `protocol_version`. On stock firmware the `EXT_LIST` read itself STALLs, and fork firmware without the link leaves these IDs out of the list. In both cases every ID below STALLs (`ERROR` on UART / I2C / MIDI), the same as any unknown sub-command.
- **Per build.** A receiver build (any DSPi or K340) and a sender build list different IDs:

  | ID | Receiver build (roles 1, 3) | Sender build (roles 2, 4) |
  |---|---|---|
  | `0x8000` `EXT_LINK_INFO` | IN | IN |
  | `0x8001` `EXT_LINK_STATUS` | IN | IN |
  | `0x8002` `EXT_LINK_CONFIG` | IN / OUT | IN / OUT (the fan-out input) |
  | `0x8003` `EXT_LINK_SLOTS` | IN / OUT | not listed |
  | `0x8004` `EXT_LINK_PHASE` | IN / OUT | not listed |
  | `0x8005` `EXT_FRAME_COUNTER` | IN | IN |
  | `0x8006` `EXT_APPLY_AT_FRAME` | IN / OUT | not listed |
  | `0x8007` `EXT_LINK_TX` | not listed | IN / OUT |
  | `0x8008` `EXT_LINK_CTRL_MSG` | IN | IN / OUT |
  | `0x8009` `EXT_LINK_STATS_RESET` | OUT | OUT |
  | `0x800A` `EXT_LINK_DIAG` | IN | IN |

  An ID that is not listed, or a direction a build does not have, is refused like an unknown sub-command.
- **Layouts.** Every multi-byte field is little-endian and packed, with no padding. The offsets below are normative. IN replies are truncated to `wLength` like every other GET. A later version may append fields to an IN reply but never moves existing ones, so a host reads at least the length given here. An OUT form needs exactly its length (1.1).
- **Reserved fields.** A reserved field reads 0. In an OUT payload it must be 0, or the write is refused. Fields marked *IN only* are ignored on OUT, so a host can write back a value it has read.
- **Refused writes change nothing.** A write that is refused leaves every field as it was, and the write is all-or-nothing.
- **Persistence.** `EXT_LINK_CONFIG`, `EXT_LINK_SLOTS`, `EXT_LINK_PHASE` and `EXT_LINK_TX` take effect at once and are device-global. They are not part of the bulk parameters or the presets, so `WireBulkParams` stays byte-compatible with upstream tools (SYS2 5.4). `REQ_SAVE_OUTPUT_CONFIG` (`0x52`) stores them with the rest of the IO block, and preset loads and factory resets never touch them.
- **No `PARAM_CHANGED`.** None of these writes sends a parameter notification. A host that shares a unit with other writers re-reads the value. In a system the hub is normally the only writer.
- **Access.** The hub keeps its rule that `REQ_EXTENSION` is admin-only, so link configuration is an admin action.

#### 2.8.1 `EXT_LINK_INFO` (0x8000)

IN only, 32 bytes. Says what the unit is and what its link can do. The values are constant for a build, except `role`, `rx_pin` and `tx_pin`, which follow `EXT_LINK_CONFIG`.

| Offset | Size | Field | Meaning |
|---|---|---|---|
| 0 | 1 | `role` | 0 none, 1 receiver, 2 sender, 3 repeater (a receiver whose `tx_pin` is set), 4 fan-out (a sender whose link input is enabled). A host treats 0 as no link support. |
| 1 | 1 | `protocol_version` | the frame and superframe version the unit speaks; 1 (superframe byte 1) |
| 2 | 1 | `class_mask` | line classes the build can run: bit 0 L1 (15.36 MBd), bit 1 L2 (30.72), bit 2 L3 (61.44) |
| 3 | 1 | `max_links` | sender: link outputs the build drives (8 on the v1 carrier, at most 12). Receiver: 1 |
| 4 | 1 | `max_channels` | the largest frame channel count C it decodes or encodes (64) |
| 5 | 1 | `slots_per_unit` | receiver: matrix inputs fed from the link (8). Sender: 0 |
| 6 | 2 | `flags` | reserved, 0 |
| 8 | 1 | `rx_pin` | GPIO in use for the link input (`EXT_LINK_CONFIG.rx_pin` resolved: the build default when that is `0xFF`), `0xFF` when the build has no input |
| 9 | 1 | `tx_pin` | repeater output GPIO, `0xFF` none |
| 10 | 2 | reserved | 0 |
| 12 | 4 | `features` | bit 0 `PHASE_LOCK` (`EXT_LINK_PHASE` mode 1), bit 1 `APPLY_AT_FRAME`, bit 2 `REPEATER`, bit 3 `FAN_OUT`, bit 4 `TEST_PATTERNS`, bit 5 `LOS_PIN`; bits 6-31 reserved, 0 |
| 16 | 2 | `d_local_min` | receiver: the smallest effective `D_local` accepted under the current output latency option (SYS2 3.2). Sender: 0 |
| 18 | 2 | `d_local_max` | receiver: the largest effective `D_local` the output rings can hold. Sender: 0 |
| 20 | 12 | reserved | 0 |

`d_local_min` changes when the output latency option (`REQ_OUTPUT_LATENCY` `0x03`) changes. Re-read it after changing the option.

Example (the fake device: a receiver build on GP0, the 1 ms latency option):
`01 01 07 01 40 08 00 00 00 ff 00 00 23 00 00 00 70 00 00 04 00 ...` (12 more zero bytes): receiver, v1, L1-L3, 1 link, 64 channels, 8 slots, input GP0, no repeater, `PHASE_LOCK | APPLY_AT_FRAME | LOS_PIN`, `D_local` 112..1024.

#### 2.8.2 `EXT_LINK_STATUS` (0x8001)

IN only, 52 bytes. On a receiver it describes the link input. On a sender, offsets 0-7 and the counters describe the fan-out input (`state` 0 while the sender is master), and `frame` is the counter the sender is sending.

| Offset | Size | Field | Meaning |
|---|---|---|---|
| 0 | 1 | `state` | 0 off, 1 searching (class detection, comma search, waiting for a superframe), 2 locked, 3 holdover (long loss: muted, VCXO trim held, outputs running on silence) |
| 1 | 1 | `class` | detected line class 1-3; 0 while unknown |
| 2 | 1 | `rate_code` | from the superframe: 0 = 44.1 kHz, 1 = 48 kHz, 2 = 96 kHz; `0xFF` unknown |
| 3 | 1 | `channels` | C, from the superframe; 0 unknown |
| 4 | 1 | `hops` | the superframe's hop count (0 = fed directly by the master sender) |
| 5 | 1 | `los` | the LOS pin: 0 clear, 1 asserted, `0xFF` no LOS pin configured |
| 6 | 1 | `sender_id` | superframe byte 7 |
| 7 | 1 | `sf_flags` | the superframe's flags byte (byte 2) as last received: bit 0 system mute, bit 1 rate change pending, bit 2 apply pending, bit 3 sender has a live USB stream, bit 4 test pattern, bit 5 sender is slaved |
| 8 | 8 | `frame` | link frame counter F (u64); valid in states 2 and 3 (in 3 it is extrapolated), 0 otherwise |
| 16 | 4 | `lock_count` | entries into LOCKED (state 2) |
| 20 | 4 | `loss_count` | losses from LOCKED (to state 1 or 3); a loss while still searching is not counted |
| 24 | 4 | `crc8_errors` | bad frame heads, aligned (state 1) and locked: a non-data symbol in the head or a CRC-8 failure |
| 28 | 4 | `crc16_errors` | failed banks: a non-data symbol in a decoded bank or a CRC-16 failure |
| 32 | 4 | `bit_slips` | commas found 1-2 bits away from the expected position. A comma that is missing altogether is not a slip; it is a flywheel frame, counted in `EXT_LINK_DIAG` `comma_errors` |
| 36 | 4 | `superframe_errors` | rejected superframes (`link_protocol_spec.md` 8.6): a complete assembly that fails validity (codes 1-6), the class check (17) or, while aligned, the counter check (18); and, while locked, an assembly still incomplete at index 31 (16) |
| 40 | 4 | `vcxo_ppm` | f32, the soft VCXO's current correction |
| 44 | 4 | `phase_error_frames` | f32, the phase error e in frames (SYS2 3.1); 0 unless `EXT_LINK_PHASE.state` is 2 |
| 48 | 2 | `fill_frames` | output ring fill in frames (the guard term) |
| 50 | 2 | reserved | 0 |

The counters are those of `link_protocol_spec.md` 8.9, which defines exactly what each one counts. They run from boot or from the last `EXT_LINK_STATS_RESET` and wrap at 2^32. The four bench counters of 8.9 (`comma_errors`, `counter_errors`, `code_errors`, `idle_errors`) are in `EXT_LINK_DIAG` (2.8.11).

#### 2.8.3 `EXT_LINK_CONFIG` (0x8002)

IN / OUT, 8 bytes.

| Offset | Field | Meaning | Default |
|---|---|---|---|
| 0 | `enable` | 0 / 1 | 0 |
| 1 | `rx_pin` | link input GPIO, `0xFF` = build default (receiver GP0, sender GP20) | `0xFF` |
| 2 | `class` | 0 auto-detect, 1-3 force L1-L3 | 0 |
| 3 | `tx_pin` | repeater output GPIO, `0xFF` = no repeater (receivers only; a sender refuses anything else) | `0xFF` |
| 4 | `los_pin` | SFP LOS input GPIO (pulled up), `0xFF` none | `0xFF` |
| 5 | `txdis_pin` | SFP TX_DISABLE output for the repeater, `0xFF` none | `0xFF` |
| 6 | `flags` | bit 0 mute on loss, bit 1 automatic re-anchor; bits 2-7 reserved, 0 | `0x03` |
| 7 | reserved | 0 | 0 |

- **Receiver:** the link input runs while `enable` is 1 *and* the input source is LINK (`REQ_SET_INPUT_SOURCE 0xE0`, value 7; stock firmware refuses 7). It uses the S/PDIF receiver's PIO and DMA, which are free only then. Under any other source, or with `enable` 0, `state` reads 0, and LINK as the source plays silence. Clearing `enable` while locked sends `0xE1` with reason `DISABLED`.
- **Sender:** `enable` 1 slaves the sender to its link input (fan-out mode, `role` 4).
- **Refused (STALL / `ERROR`):** `enable` above 1, `class` above 3, undefined flag bits or a nonzero reserved byte, or a pin that is neither `0xFF` nor a header GPIO (0-22, 26-28; 23-25 and 29 are internal on a Pico 2). Also refused: two set pins that are equal, `tx_pin` set on a sender or on a build without `REPEATER`, `los_pin` on a build without `LOS_PIN`, and a pin that the live configuration already uses for something else (an output slot, the PDM sub, ADAT, UART, I2C, an enabled S/PDIF input). The firmware checks that last rule against its pin table at the time of the write.

Example: `01 00 01 16 15 ff 01 00`: enabled, input GP0, forced L1, repeater on GP22, LOS on GP21, mute on loss, no automatic re-anchor.

#### 2.8.4 `EXT_LINK_SLOTS` (0x8003)

IN / OUT, 8 bytes: `slot[i]` is the link channel (0..C-1) that feeds matrix input `i`, or `0xFF` for silence. Receivers only. The default is the identity `00 01 02 03 04 05 06 07`.

- A value from 64 to `0xFE` is refused. A slot at or above the stream's current C is accepted and plays silence until C grows, so a table can be written before the sender is reconfigured.
- With the input source LINK, all 8 matrix inputs are active, as with the 8-channel ADAT input. The matrix then routes them as usual (SYS2 5.2).
- Each bank of 8 slots that is used costs about 7 % of core 0 at 48 kHz. Keep a unit's slots inside one bank (0-7, 8-15, ...) where possible.

#### 2.8.5 `EXT_LINK_PHASE` (0x8004)

IN / OUT, 8 bytes. Receivers only.

| Offset | Size | Field | Meaning | Default |
|---|---|---|---|---|
| 0 | 1 | `mode` | 0 fill servo (today's behaviour), 1 phase lock (SYS2 3.1) | 1 |
| 1 | 1 | reserved | 0 | 0 |
| 2 | 2 | `d_local` | frames between the arrival of link frame F and its samples leaving the outputs (SYS2 3.2: `D - hops`, written by the hub) | 128 |
| 4 | 2 | `hops_offset` | i16, normally 0 (`d_local` already includes the hops); a trim for measured offsets | 0 |
| 6 | 1 | `flags` | IN: bit 0 `D_BELOW_MIN` (the effective delay is below `d_local_min`, for example after the latency option changed; the unit then runs in fill-servo behaviour until a valid value is written). OUT: must be 0 | 0 |
| 7 | 1 | `state` | *IN only*: 0 free, 1 anchoring, 2 locked, 3 re-anchor pending | |

- The effective delay is `d_local + hops_offset`. A write is refused when the delay is outside [`d_local_min`, `d_local_max`] of `EXT_LINK_INFO`, when `mode` is above 1, when `hops_offset` is outside -64..64, or when a reserved byte or an OUT flag is nonzero.
- A write that changes `mode` or the effective delay while phase-locked re-anchors: the outputs drain, are prefilled and restart in sync, which gives 2-6 ms of silence. The unit sends `0xE1` with reason `REANCHOR`. Rewriting the same values does nothing.
- Example: `01 00 80 00 00 00 00 02`: phase lock, D_local 128, locked.

#### 2.8.6 `EXT_FRAME_COUNTER` (0x8005)

IN only, 16 bytes: `u64 frame` (F at the moment of the read; 0 unless `state` is 2 or 3; on a sender, the counter it sends), `u32 time_us` (the unit's `time_us_32()` at that moment), `u32 output_frames` (O, output frames played since the outputs started in sync, low 32 bits). The hub reads every unit in one tick and checks that the F values agree within the Link round trip (SYS2 3.5). `frame - output_frames` is constant while the unit is phase-locked.

#### 2.8.7 `EXT_APPLY_AT_FRAME` (0x8006)

Receivers only. **OUT**, 16 bytes, queues one operation for link frame `frame`:

| Offset | Size | Field |
|---|---|---|
| 0 | 8 | `frame` (u64) |
| 8 | 1 | `op` |
| 9 | 1 | `arg` |
| 10 | 2 | `value` (i16) |
| 12 | 4 | reserved, 0 |

| `op` | Name | `arg` | `value` |
|---|---|---|---|
| 0 | `CANCEL_ALL` | 0 | 0; `frame` ignored. Cancels every pending op; each sends `0xE2` with result `cancelled` |
| 1 | `MUTE` | 0 | 0 |
| 2 | `UNMUTE` | 0 | 0 |
| 3 | `PRESET_LOAD` | slot 0-9 | 0 |
| 4 | `MASTER_VOLUME` | 0 | dB x 100, -12800..0 |
| 5 | `OUT_DELAY` | output index | delay in samples, 0..`MAX_DELAY_SAMPLES` - 1 (2047 on RP2350, 1023 on RP2040) |
| 6 | `RATE` | refused here: a receiver follows the rate its superframes describe (`link_protocol_spec.md` 6.5). Only the sender schedules RATE (2.8.8, 2.8.9) | |
| 7 | `CLICK` | output index | 0: a one-sample full-scale impulse on that output, for the scope (SYS2 3.5) |
| 8 | `SYNC_PULSE` | 0 | 0: toggles the unit's sync GPIO |

- **When.** The op acts on the samples of link frame `frame`, so it is heard `D_local` frames after that frame arrives, at the same moment on every phase-locked unit. `MUTE`, `UNMUTE`, `MASTER_VOLUME` and `CLICK` act on exactly that sample. `PRESET_LOAD` and `OUT_DELAY` start there and then proceed as their opcodes do (mute, ramp, refill). A `RATE` that arrives in the superframe is not an op of this queue: the receiver switches its layout at the frame boundary (`link_protocol_spec.md` 6.5) and re-plans its dividers as `perform_rate_change` does.
- **Queue.** At most 4 ops are pending. An op whose frame has already passed is applied at once and reported `late`.
- **Refused (STALL / `ERROR`):** an unknown `op`, op 6 `RATE`, an `arg` or `value` outside the table (unused fields must be 0), a nonzero reserved byte, a full queue, or `EXT_LINK_STATUS.state` other than 2 (without a lock there is no F).
- **Two delivery paths.** Ops that should reach every unit (and `RATE`, which only exists there) go through the sender's superframe (`EXT_LINK_CTRL_MSG` OUT on the sender), which has no control-path latency. This sub-command is the per-unit path for values that differ between units.

**IN**, 16 bytes: `u8 pending` (ops queued), `u8 last_op`, `u8 last_result` (0 applied, 1 late, 2 cancelled, 3 refused at apply time, for example an empty preset slot), `u8` reserved, `u64 last_frame` (the target frame of the last op), `u32` reserved. All zero until the first op completes.

Example: MASTER_VOLUME -20 dB at frame 1 002 400: `a0 4b 0f 00 00 00 00 00 04 00 30 f8 00 00 00 00`. Once it has been applied, IN reads `00 04 00 00 a0 4b 0f 00 00 00 00 00 00 00 00 00`.

#### 2.8.8 `EXT_LINK_TX` (0x8007)

Sender builds only. IN / OUT, 16 bytes.

| Offset | Size | Field | Meaning | Default |
|---|---|---|---|---|
| 0 | 2 | `link_enable` | bit n = link output n sends (bits at or above `max_links` must be 0) | 0 |
| 2 | 1 | `class` | 1-3, must be in `class_mask` | 1 |
| 3 | 1 | `channels` | C, 1..`link_max_channels(class, rate_code)`, the protocol limit of `link_protocol_spec.md` 5.7 (the only capacity table: for example L3 at 96 kHz takes 18, L1/L2/L3 at 44.1 kHz take 8/19/41) | 8 |
| 4 | 1 | `rate_code` | 0 = 44.1, 1 = 48, 2 = 96 kHz; normally follows the USB stream, and a change is announced with apply op `RATE` | 1 |
| 5 | 1 | `test_pattern` | 0 audio, 1 PRBS-7 payload, 2 idle only, 3 1 kHz tone on all slots | 0 |
| 6 | 1 | `slot_map_id` | 0 identity (USB channel i to slot i); others reserved | 0 |
| 7 | 1 | `sync_pin` | 1 pps / `SYNC_PULSE` output GPIO, `0xFF` none | `0xFF` |
| 8 | 2 | `txdis_mask` | bit n set drives link n's SFP TX_DISABLE high (off) | `0x00FF` |
| 10 | 2 | `los_mask` | *IN only*: bit n = LOS asserted on link n's return fibre | |
| 12 | 1 | `usb_alt` | *IN only*: the active USB audio alternate setting, 0 = not streaming | |
| 13 | 1 | `flags` | bit 0 system mute (superframe flag bit 0); bits 1-7 reserved, 0 | 0 |
| 14 | 2 | reserved | 0 | 0 |

- **`class` change.** Not announced in-band (the protocol has no op for it, because it changes the line rate itself). While links run, the sender restarts every link output at the new line rate. The receivers lose lock (`0xE1`), re-detect the class and lock again. That is a dropout of tens of milliseconds, so a host changes the class only with the system muted.
- **`rate_code` or `channels` change.** Carried in-band: while links run, the sender schedules apply op `RATE` at the first superframe boundary (a multiple of 32) at least 2400 frames (50 ms at 48 kHz) ahead, with `value` = the new C, or 0 when C is unchanged (`link_protocol_spec.md` 6.5). Receivers switch at that frame. The IN form shows the requested values at once; the superframe (`EXT_LINK_CTRL_MSG` IN) shows the old ones until the switch.
- **Refused (STALL / `ERROR`):** `channels` above `link_max_channels` for the class and rate, a `class` outside `class_mask`, `rate_code` above 2, `test_pattern` above 3, a nonzero `slot_map_id`, enable or TX_DISABLE bits at or above `max_links`, a bad `sync_pin`, undefined flag bits or a nonzero reserved byte. Also refused: a `rate_code` or `channels` change while links run and an apply is already pending (the sender carries one apply at a time; wait for `EXT_LINK_CTRL_MSG` IN to show it gone).

#### 2.8.9 `EXT_LINK_CTRL_MSG` (0x8008)

**OUT** (sender only), 16 bytes: `u64 frame`, `u8 op`, `u8 arg`, `i16 value`, `u8 system_flags` (bit 0 system mute; other bits 0), `u8[3]` reserved. It schedules one apply for every unit on the link, following `link_protocol_spec.md` 6.4 and 6.5 (the reference is `link_tx_schedule()`):

- **Carriage.** The op travels in superframe bytes 16-27 of every superframe that *completes* before the target, that is when `superframe start + 32 <= frame`. The pending apply ends once the sender's F reaches `frame`.
- **Lead.** `frame` must be at least `next superframe start + 32`, so that at least one full superframe carries it, where the next superframe start is the sender's current F rounded up to a multiple of 32. Use far more lead in practice (SYS2 3.3 uses 2400 frames) so that one or two corrupted superframes cost nothing.
- **One at a time.** While an apply is pending, a second one is refused. It is not replaced, and there is no cancel. Op 0 ("none") schedules nothing: the write only sets `system_flags` and leaves a pending apply alone.
- **Ops.** 1-5, 7 and 8 take the arguments of 2.8.7. Op 6 `RATE`: `arg` = the new rate code, `value` must be 0 (hosts change only the rate; the sender fills in C itself when `EXT_LINK_TX.channels` changes), `frame` must be a multiple of 32, and the current C must fit the new rate (`link_max_channels(class, arg)`).
- **Refused (STALL / `ERROR`):** an unknown op, arguments outside those rules, too little lead, a RATE off a superframe boundary or one that does not fit, a pending apply, undefined `system_flags` bits or a nonzero reserved byte. The sender needs no lock.

**IN** (every build), 24 bytes. On a receiver it is a snapshot of the last valid superframe received. On a sender it is the superframe content it is sending now, so a pending apply shows here as soon as it is accepted, and bytes 16-27 read 0 once the apply has ended:

| Offset | Size | Field |
|---|---|---|
| 0 | 6 | superframe bytes 2-7: flags, rate code, C, class, hops, sender id |
| 6 | 2 | reserved, 0 |
| 8 | 12 | superframe bytes 16-27: apply frame (u64), op, arg, value (i16) |
| 20 | 4 | F of that superframe's first frame, low 32 bits |

All zero before the first superframe.

#### 2.8.10 `EXT_LINK_STATS_RESET` (0x8009)

OUT only, 1 byte: 1 clears the counters of `EXT_LINK_STATUS` (offsets 16-39) and the bench counters of `EXT_LINK_DIAG` (offsets 40-55). Any other value is refused. Lock state, F, the trim-scan result and configuration are untouched.

#### 2.8.11 `EXT_LINK_DIAG` (0x800A)

IN only, 64 bytes, for bench tools (`link_check.py`, SYS2 8a). Byte 0 is a layout version, and the rest may change when it changes, so apps must not parse it. The reply cannot grow beyond 64 bytes (the firmware's response buffer). The receiver's PIO program emits only bits, so there is no sampling-phase histogram. Instead the trim scan at link-up says how wide the clean sampling window is: a healthy link has a clean range centred on 0 within one step. Offsets 40-55 are the four bench counters of `link_protocol_spec.md` 8.9. Version 1:

| Offset | Size | Field |
|---|---|---|
| 0 | 1 | `diag_version` (1) |
| 1 | 1 | comma bit offset in the 32-bit DMA words (0-31) |
| 2 | 2 | reserved |
| 4 | 12 | f32 servo terms: `ppm_ff` (rate feed-forward), `ppm_phase` (Kp x e), `integral` |
| 16 | 16 | u32 RX ring write position (bytes), parser read position (bytes), output DMA words consumed, RX DMA transfer count |
| 32 | 1 | i8 `trim_lo`: lowest clean sampling trim of the last link-up scan, -9..+9 (one step moves the sampling window 3.26 ns; LK2) |
| 33 | 1 | i8 `trim_hi`: highest clean trim |
| 34 | 1 | i8 `trim_used`: the trim in use |
| 35 | 1 | u8 `scan_state`: 0 no scan, 1 running, 2 done, 3 done but no trim was clean |
| 36 | 4 | u32 `clean_mask`: bit i set = trim i - 9 was clean |
| 40 | 4 | u32 `comma_errors`: flywheel frames (a comma missing where expected) |
| 44 | 4 | u32 `counter_errors`: HDR mismatches and F corrections by a superframe |
| 48 | 4 | u32 `code_errors`: non-data symbols in heads and decoded banks |
| 52 | 4 | u32 `idle_errors`: non-idle padding symbols |
| 56 | 8 | reserved |

A sender build (WP LK4) answers with its own layout, byte 0 = `0x81`: engine state, encoder cycles per frame against
the frame's budget, ring lead, FIFO and USB counters, feedback value, fan-out delay and input rate
(`Documentation/Features/link_sender.md` 7.1).

#### 2.8.12 Link events `0xE1`, `0xE2`

Both events are v2 packets of 12 bytes. They are sent on every notification consumer (USB EP 0x83, UART, USB MIDI), like the clip event (2.4.1). Each is announced by an `EXT_NOTIFY_FEATURES` bit (2.4).

**`NOTIFY_EVT_LINK_STATE` (`0xE1`)**, bit 2 `NOTIFY_FEAT_LINK_STATE`:

| Offset | Size | Field |
|---|---|---|
| 0 | 1 | `version` 2 |
| 1 | 1 | `event_id` `0xE1` |
| 2 | 1 | `flags` 0 |
| 3 | 1 | `seq` |
| 4 | 1 | `state` (as `EXT_LINK_STATUS.state`) |
| 5 | 1 | `class` (0 unknown) |
| 6 | 1 | `reason`: 0 `NONE`, 1 `LOCKED` (anchored and playing), 2 `LOS` (the LOS pin), 3 `FRAME_ERRORS` (loss after 3 consecutive bad heads or failed frames, any mix, `link_protocol_spec.md` 8.8), 4 `COMMA_LOST` (no comma in reach while aligned, or all three failed frames were flywheel frames), 5 `REANCHOR` (phase error beyond 0.5 frame after a loss, or a new `EXT_LINK_PHASE`), 6 `VCXO_LIMIT` (VCXO authority exceeded), 7 `DISABLED` (`enable` cleared or the input source changed) |
| 7 | 1 | reserved 0 |
| 8 | 4 | F, low 32 bits (u32 LE), at the transition |

The event is sent on every change of `state` and on every re-anchor, even when the state stays 2. It is rate-limited to one per 250 ms. Transitions inside that window collapse into one event that carries the latest state and reason, following the clip event's merge rules (an entry a consumer has peeked is never changed). A host that needs every transition reads `lock_count` / `loss_count`.

Example: `02 e1 00 00 02 01 01 00 40 42 0f 00`: locked on L1, reason `LOCKED`, F_low32 = 1 000 000.

**`NOTIFY_EVT_APPLY_RESULT` (`0xE2`)**, bit 3 `NOTIFY_FEAT_APPLY_RESULT`:

| Offset | Size | Field |
|---|---|---|
| 0-3 | 4 | `2, 0xE2, 0, seq` |
| 4 | 1 | `op` |
| 5 | 1 | `result` (as `EXT_APPLY_AT_FRAME.last_result`) |
| 6 | 2 | reserved 0 |
| 8 | 4 | the op's *target* frame, low 32 bits (u32 LE), so the host can match the event to its request |

One event per op that was applied, applied late, cancelled or refused, whether it came through `EXT_APPLY_AT_FRAME` or the superframe. The queue holds at most 4 ops and every op costs the host one write, so these events cannot storm and are not rate-limited. If the notification ring is full, `EXT_APPLY_AT_FRAME` IN still shows the last result.

Example: `02 e2 00 01 04 00 00 00 a0 4b 0f 00`: MASTER_VOLUME applied at frame 1 002 400.

#### 2.8.13 Host recipe

1. `EXT_LIST` (read 64 bytes): STALL or no `0x8000` means no link. Today the list has at most 16 IDs, 32 bytes.
2. `EXT_LINK_INFO`: `role` 0 or an unknown `protocol_version` means no link.
3. `EXT_NOTIFY_FEATURES`: decode `0xE1` / `0xE2` only when bit 2 / bit 3 is set. Without them, poll `EXT_LINK_STATUS`.
4. Configure: `EXT_LINK_CONFIG`, `EXT_LINK_SLOTS`, `EXT_LINK_PHASE` (within the `d_local` limits), input source 7, then `REQ_SAVE_OUTPUT_CONFIG` to persist.
5. Wait for `0xE1` with state 2, or poll. Then use `EXT_FRAME_COUNTER` and `EXT_APPLY_AT_FRAME`.

The `midi_ctrl` tool's fake device (`tools/midi_ctrl/fake_device.py`, `FakeDevice(link="receiver" | "sender")`) implements this control plane: validation, refusals, the per-build ID lists, the apply queue and both events. Its tests (`tests/test_midi_ctrl.py`, class `LinkBlock`) check the example bytes in this section, so host code can be built against it before the firmware exists. `midi_ctrl.py link` prints a unit's link state, read only.

## 3. Transports

| Transport | Reaches `0x00`? | Refusal |
|---|---|---|
| USB vendor (EP0) | yes | STALL |
| USB MIDI (`DSPI_USB_MIDI`) | yes; same dispatcher, no special casing | `ERROR` |
| UART / I2C control interfaces | yes; same dispatcher | `ERROR` |

The pair map sends a `PARAM_CHANGED` for its bulk-parameter offset. The clip event (2.4.1) reaches every notification consumer. `EXT_NOTIFY_FEATURES` only describes notification behaviour; it sends nothing itself.

## 4. Implementation map

| File | Role |
|---|---|
| `config.h` | `REQ_EXTENSION`, `EXT_*` IDs |
| `vendor_commands.c` | `ext_ids[]` (the `EXT_LIST` table), `ext_set_len()`, `ext_handle_set()`, `ext_handle_get()`; the SETUP-stage length check and the status-stage STALL in `tud_vendor_control_xfer_cb` |
| `notify.h` | `NOTIFY_FEAT_*`, `NOTIFY_FEATURES` (the `EXT_NOTIFY_FEATURES` value), `NOTIFY_EVT_EXT_CLIP` |
| `notify.c` | `notify_push_clip()`, `notify_clip_service()` (rate limit, merge into an unsent entry) |
| `config.h` | `CLIP_LATCH()`, used by the metering sites in `audio_pipeline.c` and `pdm_generator.c` |
| `crossfeed_map.h` | pair-map validation, normalization and encoding |
| `core_vreg.{c,h}` | core voltage selection, trials, hang accounting |
| `identify.{c,h}` | identify blink and tone, hardware description; `DSPI_HW_REV` in `CMakeLists.txt` |
| `reset_info.{c,h}` | reset-cause capture and decoding; `reset_info_capture()` is the first call in `main()` |
| `config.h` | `FW_FIX_*`, `FW_FIXES` (the `EXT_FIRMWARE_FIXES` value) |
| planned (LK4-LK6) | `EXT_LINK_*` IDs in `config.h`, the link sub-commands in `ext_ids[]` / `ext_set_len()` / the handlers, `NOTIFY_FEAT_LINK_STATE` / `NOTIFY_FEAT_APPLY_RESULT` and the `0xE1` / `0xE2` pushers in `notify.{c,h}`, `link_rx.c` / `link_tx.c` |
| `tools/midi_ctrl/fake_device.py` | `FakeLink`: the control-plane model of 2.8 (host development and tests) |
| `firmware/DSPi-Sender/sender_vendor.c` | the sender build's link block (WP LK4): 0x0003, 0x8000-0x8002, 0x8005, 0x8007-0x800A; `sender_link.c` holds the `EXT_LINK_TX` / `EXT_LINK_CTRL_MSG` parsing and the RATE scheduling |

**Adding a sub-command:** pick the next free ID (`0x0001`-`0x7FFF` only after agreeing it with Weeb Labs; a link sub-command takes the next free ID from `0x800B`). Add it to `config.h`, to `ext_ids[]` in ascending order and to `ext_set_len()` if it has an OUT form, then add cases to `ext_handle_set()` / `ext_handle_get()` and a row to section 2.

## 5. History

- 2026-11-06: introduced. Replaces the fork's squatted opcodes `0x02` (`REQ_CROSSFEED_PAIR_MAP`), `0x4F` (`REQ_SET_CORE_VREG`) and `0x6F` (`REQ_GET_CORE_VREG`), which are removed with no aliases.
- 2026-11-06: `0x0003` `EXT_NOTIFY_FEATURES` added (bit 0 `NOTIFY_FEAT_BULK_SOURCE`).
- 2026-11-12: `EXT_NOTIFY_FEATURES` bit 1 `NOTIFY_FEAT_CLIP_EVENT` and the clip event `0xE0`; fork event IDs `0xE0`-`0xEF`.
- 2026-11-12: `0x0004` `EXT_IDENTIFY` added.
- 2026-11-12: `0x0005` `EXT_RESET_INFO` added.
- 2026-11-12: `0x0006` `EXT_FIRMWARE_FIXES` added (bit 0 `FW_FIX_BULK_DB_POWF`).
- 2026-11-13: the sender side of the link block implemented (WP LK4, `firmware/DSPi-Sender`); `EXT_LINK_DIAG` sender layout `0x81`.
- 2026-11-13: link block aligned with `link_protocol_spec.md` (review lk): counter definitions, `EXT_LINK_DIAG` offsets 32-63 (trim scan and bench counters instead of a histogram), per-unit `RATE` refused, `EXT_LINK_CTRL_MSG` scheduling rules, capacity from `link_max_channels()`, `class` changes not announced in-band, 0xE1 reason 3 renamed `FRAME_ERRORS`.
- 2026-11-12: link block `0x8000`-`0x800A` specified (WP LK7, SYS2 5.3), with `EXT_NOTIFY_FEATURES` bits 2 and 3 and events `0xE1` / `0xE2`; `0x8000`-`0x80FF` becomes the fork's link range. No firmware implements it yet.
