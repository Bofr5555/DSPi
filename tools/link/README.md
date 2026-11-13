# link: reference codec for the multi-unit audio link

Python twin of the C reference codec in `firmware/DSPi/link/`, bit-exact with
it. Standard-library Python 3; runs in a podman container (`Containerfile`,
official `python:3.12-slim` pinned by digest, nothing installed, no network,
no devices). Protocol: `Documentation/Features/link_protocol_spec.md`.

| File | Purpose |
|---|---|
| `link_codec.py` | 8b/10b, CRCs, frame layout and 44.1 kHz lengths, the capacity table `CAPACITY` (the only one; spec 5.7), superframe, PRBS-7, `Encoder`, the reference receiver `Decoder`, the log format and test helpers of spec section 9.2 |
| `gen_vectors.py` | regenerates `firmware/tests/link_vectors.json` (shared with `firmware/tests/test_link_codec.c`) |
| `tests/test_link_codec.py` | anchors, stream properties, receiver behaviour, and the vector file (current and passing) |
| `bench.py` | drives link bench Picos (`firmware/link_bench`, WP LK3) over their USB consoles: lock, trim scan, BER, impairments, PASS/FAIL summary; `Documentation/Bench/link_gpio_bench.md` |
| `bench.sh`, `Containerfile.bench` | runs `bench.py` in a container venv with only the named tty devices passed in |
| `tests/test_bench_tool.py` | `bench.py` against simulated bench boards and a pty |
| `sender_usb.py` | the link sender's extension block over Linux usbfs (WP LK4): `EXT_LINK_TX`, `EXT_LINK_CTRL_MSG`, `EXT_FRAME_COUNTER`, `EXT_LINK_DIAG`; used by `bench.py sender` / `sender-diag` (`Documentation/Bench/link_sender_bench.md`) |
| `tests/test_sender_tool.py` | `bench.py sender` against a simulated RX board and LK7's `FakeLink("sender")` |

```sh
./run.sh -m unittest discover -s tests -v
./run.sh gen_vectors.py > ../../firmware/tests/link_vectors.json   # after a deliberate protocol change
```

Decode a captured bit stream (bytes, MSB first) with the reference receiver:

```python
import link_codec as lc
d = lc.Decoder(cls=1)                       # L1
d.push(lc.bytes_to_bits(data, 8 * len(data)))
print(lc.log_of(d))                         # A<bit> L<F> F<F>:<flags>:<banks>:<hash> ... #counters
```
