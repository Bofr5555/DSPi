# alink: reference codec for the multi-unit audio link

Python twin of the C reference codec in `firmware/DSPi/alink/`, bit-exact with
it. Standard-library Python 3; runs in a podman container (`Containerfile`,
official `python:3.12-slim` pinned by digest, nothing installed, no network,
no devices). Protocol: `Documentation/Features/alink_protocol_spec.md`.

| File | Purpose |
|---|---|
| `alink_codec.py` | 8b/10b, CRCs, frame layout and 44.1 kHz lengths, the capacity table `CAPACITY` (the only one; spec 5.7), superframe, PRBS-7, `Encoder`, the reference receiver `Decoder`, the log format and test helpers of spec section 9.2 |
| `gen_vectors.py` | regenerates `firmware/tests/alink_vectors.json` (shared with `firmware/tests/test_alink_codec.c`) |
| `tests/test_alink_codec.py` | anchors, stream properties, receiver behaviour, and the vector file (current and passing) |
| `bench.py` | drives alink bench Picos (`firmware/alink_bench`, WP LK3) over their USB consoles: lock, trim scan, BER, impairments, PASS/FAIL summary; `Documentation/Bench/alink_gpio_bench.md` |
| `bench.sh`, `Containerfile.bench` | runs `bench.py` in a container venv with only the named tty devices passed in |
| `tests/test_bench_tool.py` | `bench.py` against simulated bench boards and a pty |
| `sender_usb.py` | the alink sender's extension block over Linux usbfs (WP LK4): `EXT_ALINK_TX`, `EXT_ALINK_CTRL_MSG`, `EXT_FRAME_COUNTER`, `EXT_ALINK_DIAG`; used by `bench.py sender` / `sender-diag` (`Documentation/Bench/alink_sender_bench.md`) |
| `skew_xcorr.py` | test (e) of SYS2 8 (WP LK6): the skew between two receivers from one stereo recording, by cross-correlation (programme material) or by impulse (the CLICK train), sub-sample, with the interface's own channel skew calibrated out; PASS under one alink sample; `--selftest`; `Documentation/Bench/alink_phase_bench.md` |
| `bench.py clicks` | test (e): one CLICK apply a second through the sender's superframe (WP LK6) |
| `tests/test_skew_xcorr.py`, `tests/test_bench_clicks.py` | the two above on synthetic recordings and a stub sender |
| `tests/test_sender_tool.py` | `bench.py sender` against a simulated RX board and LK7's `FakeAlink("sender")` |

```sh
./run.sh -m unittest discover -s tests -v
./run.sh gen_vectors.py > ../../firmware/tests/alink_vectors.json   # after a deliberate protocol change
```

Decode a captured bit stream (bytes, MSB first) with the reference receiver:

```python
import alink_codec as lc
d = lc.Decoder(cls=1)                       # L1
d.push(lc.bytes_to_bits(data, 8 * len(data)))
print(lc.log_of(d))                         # A<bit> L<F> F<F>:<flags>:<banks>:<hash> ... #counters
```
