# midi_ctrl: host tool for the DSPi USB MIDI control channel

Standard-library Python 3 over the ALSA rawmidi node; runs in a podman
container (`Containerfile`, official `python:3.12-slim` pinned by digest,
nothing installed). Wire format:
`Documentation/Features/usb_midi_control_spec.md`. Needs firmware built with
`-DDSPI_USB_MIDI=ON`.

| File | Purpose |
|---|---|
| `midi_ctrl.py` | codec, `RawMidiPort`, `Client`, CLI (`list`, `get`, `set`, `ext-list`, `notify`, `enter-bootloader`) |
| `measure.py` | RTT, bulk 0xA0/0xA1 throughput, notification latency, optional vendor-interface concurrency; Markdown table |
| `soak.py` | long run: meter 10 Hz, 0xA0 every 10 s, random volume in a given range, notification seq check, underrun counters |
| `regress.sh` | before/after a flash: interfaces 0-2 identical, state image identical, ALSA view |
| `fake_device.py` | device emulator (in process, or `--pty` for a separate process) |
| `usb_vendor.py`, `usb_desc.py` | read-only vendor GETs over usbfs; descriptor snapshot from sysfs |
| `gen_vectors.py` | regenerates `firmware/tests/midi_vectors.json` (shared with the C tests) |

```sh
./run.sh -m unittest discover -s tests -v        # no device
./run.sh measure.py --fake                        # no device
./run.sh midi_ctrl.py list                        # rawmidi ports (no device node needed)
DSPI_MIDI_DEV=/dev/snd/midiC2D0 ./run.sh midi_ctrl.py get 0x7F --port /dev/snd/midiC2D0
DSPI_MIDI_DEV=/dev/snd/midiC2D0 ./run.sh midi_ctrl.py ext-list --port /dev/snd/midiC2D0
DSPI_MIDI_DEV=/dev/snd/midiC2D0 ./run.sh midi_ctrl.py get 0x00 --wvalue 0x0002 --port /dev/snd/midiC2D0
```

Extension sub-commands (`Documentation/Features/extension_channel_spec.md`) are
plain `get` / `set` with `req` 0x00 and the sub-command in `--wvalue`;
`ext-list` prints the supported IDs (stock firmware: `ERROR`).

`run.sh` passes at most one rawmidi node (`DSPI_MIDI_DEV`) and optionally one
USB node (`DSPI_USB_DEV`, for `--vendor`), no network. Nothing here plays
audio: audio during `soak.py` comes from the coordinator's guarded player.
`set 0xA1` / `set 0xA3`, `measure.py --allow-bulk-set` and `enter-bootloader`
change the device; use them only on a unit the owner designated. The MIDI
channel follows the USB vendor path, so `set 0xF5` / `set 0xF7` (UART / I2C
interface config, persisted to flash) are accepted too: never send them to a
unit; their acceptance is covered by the firmware host test
(`test_midi_control`) and the fake device.
