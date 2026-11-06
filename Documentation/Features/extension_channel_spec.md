# Extension Channel (`REQ_EXTENSION`, 0x00)

*Last updated: 2026-11-06*
*Status: implemented on the bofr fork, HW-untested*
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
| `0x8000`-`0xFFFF` | Experimental or private. Never shipped in a release build; hosts must not depend on them. |

## 2. Sub-commands

| ID | Name | IN (read) | OUT (write) |
|---|---|---|---|
| `0x0000` | `EXT_LIST` | supported IDs as uint16 LE, ascending, `0x0000` itself excluded, truncated to `wLength` | none (STALL) |
| `0x0001` | `EXT_CROSSFEED_PAIR_MAP` | `uint8[4]` live map, normalized | `uint8[4]` map (exactly 4 bytes) |
| `0x0002` | `EXT_CORE_VREG` | `CoreVregStatus`, 16 bytes | `{op, value}` (exactly 2 bytes) |
| `0x0003` | `EXT_NOTIFY_FEATURES` | `uint32` LE notification-semantics flags | none (STALL) |

Current firmware answers `EXT_LIST` with `01 00 02 00 03 00`.

### 2.1 `EXT_LIST` (0x0000)

The IN-only discovery read. A host can read 64 bytes (room for 32 IDs) and use the reply length to count the IDs. On stock firmware this read STALLs.

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
| 1-31 | reserved | zero |

A host that sees bit 0 can recognise its own bulk write from the source byte and drop the time-window echo heuristic it needs for older firmware.

## 3. Transports

| Transport | Reaches `0x00`? | Refusal |
|---|---|---|
| USB vendor (EP0) | yes | STALL |
| USB MIDI (`DSPI_USB_MIDI`) | yes; same dispatcher, no special casing | `ERROR` |
| UART / I2C control interfaces | yes; same dispatcher | `ERROR` |

Notifications are unchanged. Only the pair map sends one, as a `PARAM_CHANGED` for its bulk-parameter offset. `EXT_NOTIFY_FEATURES` only describes notification behaviour; it sends nothing itself.

## 4. Implementation map

| File | Role |
|---|---|
| `config.h` | `REQ_EXTENSION`, `EXT_*` IDs |
| `vendor_commands.c` | `ext_ids[]` (the `EXT_LIST` table), `ext_set_len()`, `ext_handle_set()`, `ext_handle_get()`; the SETUP-stage length check and the status-stage STALL in `tud_vendor_control_xfer_cb` |
| `notify.h` | `NOTIFY_FEAT_*`, `NOTIFY_FEATURES` (the `EXT_NOTIFY_FEATURES` value) |
| `crossfeed_map.h` | pair-map validation, normalization and encoding |
| `core_vreg.{c,h}` | core voltage selection, trials, hang accounting |

**Adding a sub-command:** pick the next free ID (`0x0001`-`0x7FFF` only after agreeing it with Weeb Labs). Add it to `config.h`, to `ext_ids[]` in ascending order and to `ext_set_len()` if it has an OUT form, then add cases to `ext_handle_set()` / `ext_handle_get()` and a row to section 2.

## 5. History

- 2026-11-06: introduced. Replaces the fork's squatted opcodes `0x02` (`REQ_CROSSFEED_PAIR_MAP`), `0x4F` (`REQ_SET_CORE_VREG`) and `0x6F` (`REQ_GET_CORE_VREG`), which are removed with no aliases.
- 2026-11-06: `0x0003` `EXT_NOTIFY_FEATURES` added (bit 0 `NOTIFY_FEAT_BULK_SOURCE`).
