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
