#!/usr/bin/env python3
"""DSPi link protocol v1: bit-exact Python twin of firmware/DSPi/link/.

Normative text: Documentation/Features/link_protocol_spec.md.  Standard
library only.  Written from the spec's tables, not translated from the C
code, so the shared vectors (firmware/tests/link_vectors.json) compare two
independent implementations.

Conventions as in link_codec.h: a symbol is an int with code-group bit 'a'
in bit 9, sent first; RD is 0 for RD- and 1 for RD+; line bits are a list of
0/1 ints in line order (packed MSB first into bytes or 32-bit words when
stored).
"""

from fractions import Fraction
import struct

PROTOCOL_VERSION = 1

CLASS_BAUD = {1: 15_360_000, 2: 30_720_000, 3: 61_440_000}
RATE_HZ = {0: 44_100, 1: 48_000, 2: 96_000}

BANK_CHANNELS = 8
HEAD_SYMBOLS = 4
MAX_CHANNELS = 64
SF_LEN = 32
SF_MAGIC = 0xA5

K28_5_RDN = 0b0011111010
K28_5_RDP = 0b1100000101
SYM_IDLE = 0b1010101010
BYTE_IDLE = 0xB5
BYTE_K28_5 = 0xBC

FLAG_MUTE, FLAG_RATE_PEND, FLAG_APPLY_PEND = 0x01, 0x02, 0x04
FLAG_USB_LIVE, FLAG_PRBS, FLAG_SLAVED = 0x08, 0x10, 0x20
FLAGS_DEFINED = 0x3F

OP_NONE, OP_MUTE, OP_UNMUTE, OP_PRESET_LOAD, OP_MASTER_VOLUME = 0, 1, 2, 3, 4
OP_OUT_DELAY, OP_RATE, OP_CLICK, OP_SYNC_PULSE = 5, 6, 7, 8
OP_MAX = 8

SF_OK, SF_ERR_CRC, SF_ERR_MAGIC, SF_ERR_VERSION = 0, 1, 2, 3
SF_ERR_FRAME, SF_ERR_LAYOUT, SF_ERR_APPLY = 4, 5, 6
RX_SF_INCOMPLETE, RX_SF_CLASS, RX_SF_COUNTER = 16, 17, 18

# ---------------------------------------------------------------------------
# Layout
# ---------------------------------------------------------------------------


def frame_ratio(cls, rate):
    """Symbols per frame as an exact fraction, or None."""
    if cls not in CLASS_BAUD or rate not in RATE_HZ:
        return None
    return Fraction(CLASS_BAUD[cls], 10 * RATE_HZ[rate])


def frame_symbols(cls, rate, frame):
    r = frame_ratio(cls, rate)
    if r is None:
        return 0
    n, d = r.numerator, r.denominator
    m = frame % d
    return ((m + 1) * n) // d - (m * n) // d


def frame_symbols_min(cls, rate):
    r = frame_ratio(cls, rate)
    return 0 if r is None else r.numerator // r.denominator


def frame_symbols_max(cls, rate):
    r = frame_ratio(cls, rate)
    return 0 if r is None else -(-r.numerator // r.denominator)


def bank_count(channels):
    return -(-channels // BANK_CHANNELS)


def bank_channels(channels, bank):
    return max(0, min(BANK_CHANNELS, channels - bank * BANK_CHANNELS))


def bank_symbol(bank):
    return HEAD_SYMBOLS + bank * (3 * BANK_CHANNELS + 2)


def layout_symbols(channels):
    return HEAD_SYMBOLS + sum(3 * bank_channels(channels, b) + 2 for b in range(bank_count(channels)))


def max_channels(cls, rate):
    room = frame_symbols_min(cls, rate)
    c = 0
    while c < MAX_CHANNELS and layout_symbols(c + 1) <= room:
        c += 1
    return c


# The protocol's capacity table (spec 5.7), the one source for senders, tools and fakes:
# CAPACITY[(cls, rate)] is the largest C, from max_channels() as link_max_channels() in C.
CAPACITY = {(c, r): max_channels(c, r) for c in sorted(CLASS_BAUD) for r in sorted(RATE_HZ)}

# ---------------------------------------------------------------------------
# 8b/10b, IEEE 802.3 clause 36 tables as printed (abcdei / fghj, RD- then RD+)
# ---------------------------------------------------------------------------


_6B = """
100111 011000|011101 100010|101101 010010|110001 110001|110101 001010|101001 101001|011001 011001|111000 000111
111001 000110|100101 100101|010101 010101|110100 110100|001101 001101|101100 101100|011100 011100|010111 101000
011011 100100|100011 100011|010011 010011|110010 110010|001011 001011|101010 101010|011010 011010|111010 000101
110011 001100|100110 100110|010110 010110|110110 001001|001110 001110|101110 010001|011110 100001|101011 010100
"""
_4B_D = "1011 0100|1001 1001|0101 0101|1100 0011|1101 0010|1010 1010|0110 0110|1110 0001"
_4B_K = "1011 0100|0110 1001|1010 0101|1100 0011|1101 0010|0101 1010|1001 0110|0111 1000"
_6B_K28 = ("001111", "110000")
_4B_A7 = ("0111", "1000")

ENC6 = [tuple(int(c, 2) for c in e.split()) for e in _6B.replace("\n", "|").strip("|").split("|")]
ENC4D = [tuple(int(c, 2) for c in e.split()) for e in _4B_D.split("|")]
ENC4K = [tuple(int(c, 2) for c in e.split()) for e in _4B_K.split("|")]
assert len(ENC6) == 32 and len(ENC4D) == 8 and len(ENC4K) == 8

K_CODES = [(y << 5) | 28 for y in range(8)] + [(7 << 5) | x for x in (23, 27, 29, 30)]


def _rd_after(bits, width, rd):
    ones = bin(bits).count("1")
    half = width // 2
    if ones > half:
        return 1
    if ones < half:
        return 0
    if width == 6 and bits in (0b000111, 0b111000):
        return 1 if bits == 0b000111 else 0
    if width == 4 and bits in (0b0011, 0b1100):
        return 1 if bits == 0b0011 else 0
    return rd


def encode_8b10b(byte, k, rd):
    """Return (symbol, rd_after), or (None, rd) for a K code that does not exist."""
    x, y = byte & 0x1F, byte >> 5
    if k:
        if byte not in K_CODES:
            return None, rd
        s6 = int(_6B_K28[rd], 2) if x == 28 else ENC6[x][rd]
        r = _rd_after(s6, 6, rd)
        s4 = ENC4K[y][r]
    else:
        s6 = ENC6[x][rd]
        r = _rd_after(s6, 6, rd)
        alt = y == 7 and ((r == 0 and x in (17, 18, 20)) or (r == 1 and x in (11, 13, 14)))
        s4 = int(_4B_A7[r], 2) if alt else ENC4D[y][r]
    return (s6 << 4) | s4, _rd_after(s4, 4, r)


def rd_after(sym, rd):
    return _rd_after(sym & 0xF, 4, _rd_after(sym >> 4, 6, rd))


def _build_decode():
    table = {}
    for k in (False, True):
        for byte in (K_CODES if k else range(256)):
            for rd in (0, 1):
                sym, _ = encode_8b10b(byte, k, rd)
                mask, _, _ = table.get(sym, (0, byte, k))
                table[sym] = (mask | (1 << rd), byte, k)
    return table


DECODE = _build_decode()


def decode_8b10b(sym):
    """Return (rd_mask, byte, k); rd_mask 0 when sym is not a code group."""
    return DECODE.get(sym, (0, 0, False))


def is_comma(sym):
    return sym in (K28_5_RDN, K28_5_RDP)

# ---------------------------------------------------------------------------
# CRCs, samples, PRBS-7
# ---------------------------------------------------------------------------


def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def put_s24(sample):
    u = sample & 0xFFFFFF
    return bytes(((u >> 16) & 0xFF, (u >> 8) & 0xFF, u & 0xFF))


def get_s24(b):
    u = (b[0] << 16) | (b[1] << 8) | b[2]
    return u - 0x1000000 if u & 0x800000 else u


def prbs7(n):
    s, out = 0x7F, bytearray()
    for _ in range(n):
        v = 0
        for _ in range(8):
            nb = ((s >> 6) ^ (s >> 5)) & 1
            s = ((s << 1) | nb) & 0x7F
            v = (v << 1) | nb
        out.append(v)
    return bytes(out)

# ---------------------------------------------------------------------------
# Superframe
# ---------------------------------------------------------------------------


class Superframe:
    FIELDS = ("flags", "rate", "channels", "cls", "hops", "sender", "frame",
              "apply_frame", "apply_op", "apply_arg", "apply_value")

    def __init__(self, **kw):
        for f in self.FIELDS:
            setattr(self, f, kw.get(f, 0))

    def __eq__(self, other):
        return all(getattr(self, f) == getattr(other, f) for f in self.FIELDS)

    def __repr__(self):
        return "Superframe(%s)" % ", ".join("%s=%r" % (f, getattr(self, f)) for f in self.FIELDS)

    def copy(self):
        return Superframe(**{f: getattr(self, f) for f in self.FIELDS})

    def pack(self):
        b = struct.pack("<BBBBBBBBQQBBh", SF_MAGIC, PROTOCOL_VERSION, self.flags, self.rate,
                        self.channels, self.cls, self.hops, self.sender, self.frame,
                        self.apply_frame, self.apply_op, self.apply_arg, self.apply_value)
        b += b"\x00\x00"
        return b + struct.pack("<H", crc16(b))

    @classmethod
    def unpack(cls_, data):
        """Return (error, Superframe or None); the Superframe is set once CRC, magic, version pass."""
        data = bytes(data)
        if crc16(data[:30]) != struct.unpack_from("<H", data, 30)[0]:
            return SF_ERR_CRC, None
        if data[0] != SF_MAGIC:
            return SF_ERR_MAGIC, None
        if data[1] != PROTOCOL_VERSION:
            return SF_ERR_VERSION, None
        v = struct.unpack_from("<BBBBBBQQBBh", data, 2)
        sf = Superframe(**dict(zip(cls_.FIELDS, v)))
        if sf.frame % SF_LEN:
            return SF_ERR_FRAME, sf
        if (sf.cls not in CLASS_BAUD or sf.rate not in RATE_HZ or sf.channels < 1
                or sf.channels > max_channels(sf.cls, sf.rate)):
            return SF_ERR_LAYOUT, sf
        if sf.apply_op > OP_MAX or (sf.apply_op == OP_NONE) != (sf.apply_frame == 0):
            return SF_ERR_APPLY, sf
        return SF_OK, sf

    def repeat(self, out_cls):
        r = self.copy()
        r.hops = min(255, self.hops + 1)
        r.cls = out_cls
        return r

# ---------------------------------------------------------------------------
# Encoder
# ---------------------------------------------------------------------------


class Encoder:
    def __init__(self, cls, rate, channels, first, sender=0):
        if cls not in CLASS_BAUD or rate not in RATE_HZ or not 1 <= channels <= max_channels(cls, rate):
            raise ValueError("bad layout")
        self.cls, self.rate, self.channels = cls, rate, channels
        self.rd = 0
        self.frame = first
        self.flags = 0
        self.sender = sender
        self.hops = 0
        self.apply = None           # (frame, op, arg, value)
        self.sf = None              # packed at the first frame

    def _pack_sf(self, first):
        flags = self.flags & FLAGS_DEFINED & ~(FLAG_RATE_PEND | FLAG_APPLY_PEND)
        at, op, arg, value = self.apply or (0, 0, 0, 0)
        if op:
            flags |= FLAG_APPLY_PEND
        if op == OP_RATE:
            flags |= FLAG_RATE_PEND
        self.sf = Superframe(flags=flags, rate=self.rate, channels=self.channels, cls=self.cls,
                             hops=self.hops, sender=self.sender, frame=first, apply_frame=at,
                             apply_op=op, apply_arg=arg, apply_value=value).pack()

    def schedule(self, at, op, arg=0, value=0):
        next_sf = -(-self.frame // SF_LEN) * SF_LEN
        if not 1 <= op <= OP_MAX or self.apply or at < next_sf + SF_LEN:
            return False
        if op == OP_RATE:
            c = value or self.channels
            if at % SF_LEN or value < 0 or arg not in RATE_HZ or not 1 <= c <= max_channels(self.cls, arg):
                return False
        self.apply = (at, op, arg, value)
        return True

    def _hdr(self, f):
        """HDR of frame f (a hook the crafted test vectors override)."""
        return f & 0xFF

    def frame_symbols(self, samples):
        """Encode frame self.frame; return its symbols and advance."""
        f = self.frame
        if f % SF_LEN == 0 or self.sf is None:
            first = f - f % SF_LEN
            if self.apply and first + SF_LEN > self.apply[0]:
                self.apply = None
            self._pack_sf(first)
        out = []

        def put(byte, k=False):
            sym, self.rd = encode_8b10b(byte, k, self.rd)
            out.append(sym)

        put(BYTE_K28_5, True)
        hdr, ctl = self._hdr(f), self.sf[f % SF_LEN]
        for b in (hdr, ctl, crc8(bytes((hdr, ctl)))):
            put(b)
        c = self.channels
        if self.sf[2] & FLAG_PRBS:
            payload = prbs7(3 * c)
        else:
            payload = b"".join(put_s24(samples[ch]) for ch in range(c))
        for b in range(bank_count(c)):
            chunk = payload[3 * BANK_CHANNELS * b: 3 * BANK_CHANNELS * b + 3 * bank_channels(c, b)]
            for v in chunk + struct.pack("<H", crc16(chunk)):
                put(v)
        for _ in range(layout_symbols(c), frame_symbols(self.cls, self.rate, f)):
            put(BYTE_IDLE)
        self.frame = f + 1
        # RATE: the layout of the target frame is set before the caller builds its samples.
        if self.apply and self.apply[1] == OP_RATE and self.frame == self.apply[0]:
            self.rate = self.apply[2]
            if self.apply[3]:
                self.channels = self.apply[3]
        return out


def symbols_to_bits(symbols):
    bits = []
    for s in symbols:
        bits.extend((s >> (9 - i)) & 1 for i in range(10))
    return bits


def bits_to_bytes(bits):
    out = bytearray((len(bits) + 7) // 8)
    for i, b in enumerate(bits):
        if b:
            out[i >> 3] |= 0x80 >> (i & 7)
    return bytes(out)


def bytes_to_bits(data, nbits):
    return [(data[i >> 3] >> (7 - (i & 7))) & 1 for i in range(nbits)]

# ---------------------------------------------------------------------------
# Reference receiver (spec section 8)
# ---------------------------------------------------------------------------

SEARCH, ALIGNED, LOCKED = 0, 1, 2
SLIP_MAX = 2
FAIL_LIMIT = 3
RXF_COMMA_MISSING, RXF_SLIP, RXF_HEAD_BAD, RXF_COUNTER = 0x01, 0x02, 0x04, 0x08
BANK_SKIPPED, BANK_OK, BANK_HOLD, BANK_ZERO = 0, 1, 2, 3
STAT_NAMES = ("lock_count", "loss_count", "crc8_errors", "crc16_errors", "bit_slips",
              "superframe_errors", "comma_errors", "counter_errors", "code_errors",
              "idle_errors", "frames")


class Decoder:
    """Events are tuples: ("align", bitpos), ("lock", F), ("frame", F, flags, slip,
    bank_status list, samples list of C), ("sf", F), ("sferr", code), ("loss",)."""

    def __init__(self, cls, bank_mask=0xFF):
        self.cls, self.bank_mask = cls, bank_mask
        self.bits = []
        self.events = []
        self.stats = dict.fromkeys(STAT_NAMES, 0)
        self.p = 0
        self.frame = 0
        self.rate = self.channels = 0
        self.sf = None
        self._search(10)

    # -- helpers
    def _clear(self):
        self.fails = 0
        self.prev_hdr = None
        self.sf_bytes = bytearray(SF_LEN)
        self.sf_count = 0
        self.apply_pending = False
        self.bank_fails = [0] * 8
        self.samples = [0] * MAX_CHANNELS

    def _search(self, q):
        self.state, self.q = SEARCH, q
        self._clear()

    def _loss(self):
        if self.state == LOCKED:
            self.stats["loss_count"] += 1
        self.events.append(("loss",))
        self._search(self.p + 1)

    def sym(self, end):
        v = 0
        for b in self.bits[end - 10:end]:
            v = (v << 1) | b
        return v

    def _data(self, end, count=True):
        mask, byte, k = decode_8b10b(self.sym(end))
        if mask and not k:
            return byte
        if count:
            self.stats["code_errors"] += 1
        return None

    def _head(self, p, count=True):
        b = [self._data(p + 10 * (i + 1), count) for i in range(3)]
        if None in b or crc8(bytes(b[:2])) != b[2]:
            return None
        return b[0], b[1]

    def _collect(self, idx, ctl, hdr):
        cont = self.prev_hdr is not None and hdr == (self.prev_hdr + 1) & 0xFF
        if idx == 0:
            self.sf_bytes[0], self.sf_count = ctl, 1
        elif cont and self.sf_count == idx:
            self.sf_bytes[idx] = ctl
            self.sf_count += 1
        else:
            self.sf_count = 0
        self.prev_hdr = hdr

    def _sf_error(self, err):
        self.stats["superframe_errors"] += 1
        self.events.append(("sferr", err))

    def _sf_check(self):
        err, sf = Superframe.unpack(self.sf_bytes)
        if err == SF_OK and sf.cls != self.cls:
            err = RX_SF_CLASS
        return err, sf

    def _adopt(self, sf):
        self.sf, self.rate, self.channels = sf, sf.rate, sf.channels
        self.apply_pending = sf.apply_op == OP_RATE and sf.apply_frame > self.frame

    # -- steps
    def _step(self):
        if self.state == SEARCH:
            return self._step_search()
        if self.state == ALIGNED:
            return self._next_aligned() if self.body_done else self._body_aligned()
        return self._next_locked() if self.body_done else self._body_locked()

    def _step_search(self):
        self.q = max(self.q, 10)
        while self.q + 30 <= len(self.bits):
            if is_comma(self.sym(self.q)) and self._head(self.q, count=False):
                self.state, self.p = ALIGNED, self.q
                self.body_done, self.comma_ok, self.slip = False, True, 0
                self._clear()
                self.events.append(("align", self.p))
                return True
            self.q += 1
        return False

    def _body_aligned(self):
        if self.p + 30 > len(self.bits):
            return False
        head = self._head(self.p)
        if head is None:
            self.stats["crc8_errors"] += 1
            self.prev_hdr, self.sf_count = None, 0
            self.fails += 1
            if self.fails >= FAIL_LIMIT:
                self._loss()
                return True
            self.body_done = True
            return True
        self.fails = 0
        hdr, ctl = head
        idx = hdr % SF_LEN
        self._collect(idx, ctl, hdr)
        if idx == SF_LEN - 1 and self.sf_count == SF_LEN:
            err, sf = self._sf_check()
            if err == SF_OK and (sf.frame + SF_LEN - 1) & 0xFF != hdr:
                err = RX_SF_COUNTER
            if err != SF_OK:
                self._sf_error(err)
            else:
                self.state = LOCKED
                self.frame = sf.frame + SF_LEN - 1
                self._adopt(sf)
                self.bank_fails = [0] * 8
                self.samples = [0] * MAX_CHANNELS
                self.stats["lock_count"] += 1
                self.events.append(("lock", self.frame))
        self.body_done = True
        return True

    def _banks(self):
        c = self.channels
        status = [BANK_SKIPPED] * 8
        for b in range(bank_count(c)):
            k = bank_channels(c, b)
            base = BANK_CHANNELS * b
            if not (self.bank_mask >> b) & 1:
                self.samples[base:base + k] = [0] * k
                continue
            s0 = bank_symbol(b)
            vals = [self._data(self.p + 10 * (s0 + i)) for i in range(3 * k + 2)]
            ok = None not in vals and vals[3 * k] | (vals[3 * k + 1] << 8) == crc16(bytes(vals[:3 * k]))
            if ok:
                status[b] = BANK_OK
                self.bank_fails[b] = 0
                self.samples[base:base + k] = [get_s24(vals[3 * i:3 * i + 3]) for i in range(k)]
            else:
                self.stats["crc16_errors"] += 1
                self.bank_fails[b] = min(255, self.bank_fails[b] + 1)
                if self.bank_fails[b] == 1:
                    status[b] = BANK_HOLD
                else:
                    status[b] = BANK_ZERO
                    self.samples[base:base + k] = [0] * k
        self.samples[c:] = [0] * (MAX_CHANNELS - c)
        return status

    def _body_locked(self):
        s = frame_symbols(self.cls, self.rate, self.frame)
        if self.p + 10 * (s - 1) > len(self.bits):
            return False
        flags = 0
        if not self.comma_ok:
            flags |= RXF_COMMA_MISSING
        if self.slip:
            flags |= RXF_SLIP
        head = self._head(self.p)
        good = head is not None
        if not good:
            self.stats["crc8_errors"] += 1
            flags |= RXF_HEAD_BAD
        elif head[0] != self.frame & 0xFF:
            self.stats["counter_errors"] += 1
            flags |= RXF_COUNTER
            good = False
        status = self._banks()
        for i in range(layout_symbols(self.channels), s):
            if self.sym(self.p + 10 * i) != SYM_IDLE:
                self.stats["idle_errors"] += 1
        self.stats["frames"] += 1
        self.events.append(("frame", self.frame, flags, self.slip,
                            status[:bank_count(self.channels)], self.samples[:self.channels]))
        idx = self.frame % SF_LEN
        if good:
            self._collect(idx, head[1], head[0])
        else:
            self.sf_count, self.prev_hdr = 0, None
        if idx == SF_LEN - 1:
            if self.sf_count == SF_LEN:
                err, sf = self._sf_check()
            else:
                err, sf = RX_SF_INCOMPLETE, None
            if err != SF_OK:
                self._sf_error(err)
            else:
                if sf.frame + SF_LEN - 1 != self.frame:
                    self.stats["counter_errors"] += 1
                    self.frame = sf.frame + SF_LEN - 1
                self._adopt(sf)
                self.events.append(("sf", self.frame))
        if flags & (RXF_COMMA_MISSING | RXF_HEAD_BAD | RXF_COUNTER):
            self.fails += 1
            if self.fails >= FAIL_LIMIT:
                self._loss()
                return True
        else:
            self.fails = 0
        self.body_done = True
        return True

    def _next_aligned(self):
        for i in range(HEAD_SYMBOLS, frame_symbols_max(self.cls, 0) + 1):
            end = self.p + 10 * i
            if end > len(self.bits):
                return False
            if is_comma(self.sym(end)):
                self.p, self.body_done, self.comma_ok, self.slip = end, False, True, 0
                return True
        self._loss()
        return True

    def _next_locked(self):
        e = self.p + 10 * frame_symbols(self.cls, self.rate, self.frame)
        if e + SLIP_MAX > len(self.bits):
            return False
        found = None
        for d in (0, -1, 1, -2, 2):
            if is_comma(self.sym(e + d)):
                found = d
                break
        self.frame += 1
        if self.apply_pending and self.frame == self.sf.apply_frame:
            c = self.sf.apply_value or self.channels
            if (self.sf.apply_arg in RATE_HZ and self.sf.apply_value >= 0
                    and 1 <= c <= max_channels(self.cls, self.sf.apply_arg)):
                self.rate, self.channels = self.sf.apply_arg, c
            self.apply_pending = False
        if found is not None:
            self.p, self.comma_ok, self.slip = e + found, True, found
            if found:
                self.stats["bit_slips"] += 1
        else:
            self.p, self.comma_ok, self.slip = e, False, 0
            self.stats["comma_errors"] += 1
        self.body_done = False
        return True

    # -- entry point
    def push(self, bits):
        self.bits.extend(bits)
        while self._step():
            pass


def fnv1a32(data):
    h = 0x811C9DC5
    for b in data:
        h = ((h ^ b) * 0x01000193) & 0xFFFFFFFF
    return h


def log_of(dec):
    """The canonical log string of a decoder's events and counters (spec section 9.2)."""
    out = []
    for ev in dec.events:
        t = ev[0]
        if t == "align":
            out.append("A%d" % ev[1])
        elif t == "lock":
            out.append("L%d" % ev[1])
        elif t == "frame":
            _, f, flags, slip, status, samples = ev
            fl = ""
            if flags & RXF_COMMA_MISSING:
                fl += "c"
            if flags & RXF_SLIP:
                fl += "s%+d" % slip
            if flags & RXF_HEAD_BAD:
                fl += "h"
            if flags & RXF_COUNTER:
                fl += "n"
            banks = "".join(".ohz"[s] for s in status)
            h = fnv1a32(b"".join(struct.pack("<i", s) for s in samples))
            out.append("F%d:%s:%s:%08x" % (f, fl or "-", banks, h))
        elif t == "sf":
            out.append("S")
        elif t == "sferr":
            out.append("E%d" % ev[1])
        elif t == "loss":
            out.append("X")
    out.append("#" + ",".join(str(dec.stats[n]) for n in STAT_NAMES))
    return " ".join(out)

# ---------------------------------------------------------------------------
# Test stream helpers (shared with the C test through the vectors)
# ---------------------------------------------------------------------------


def test_sample(frame, ch, seed):
    """Deterministic 24-bit test sample (spec section 9.2)."""
    m = 0xFFFFFFFF
    h = ((frame & m) * 0x9E3779B1) & m
    h ^= ((ch + 1) * 0x85EBCA77) & m
    h ^= seed & m
    h ^= h >> 15
    h = (h * 0x2C1B3C6D) & m
    h ^= h >> 12
    h = (h * 0x297A2D39) & m
    h ^= h >> 15
    u = h & 0xFFFFFF
    return u - 0x1000000 if u & 0x800000 else u


def encode_stream(cls, rate, channels, first, frames, seed=0, sender=0, flags=0, hops=0,
                  schedule=None):
    """Bits of `frames` frames from `first`; schedule = (frame_index, at, op, arg, value)."""
    enc = Encoder(cls, rate, channels, first, sender)
    enc.flags, enc.hops = flags, hops
    syms = []
    for i in range(frames):
        if schedule and schedule[0] == i:
            if not enc.schedule(*schedule[1:]):
                raise ValueError("schedule refused")
        f = enc.frame
        syms += enc.frame_symbols([test_sample(f, ch, seed) for ch in range(enc.channels)])
    return symbols_to_bits(syms)


def corrupt(bits, ops):
    """Apply "flip:P", "del:P", "ins:P:B", "fill:P:N:B" ops (';'-separated, positions in
    the clean stream), highest position first."""
    bits = list(bits)
    parsed = []
    for op in filter(None, ops.split(";")):
        f = op.split(":")
        parsed.append((int(f[1]), f[0], [int(v) for v in f[2:]]))
    for pos, kind, args in sorted(parsed, key=lambda t: -t[0]):
        if kind == "flip":
            bits[pos] ^= 1
        elif kind == "del":
            del bits[pos]
        elif kind == "ins":
            bits.insert(pos, args[0])
        elif kind == "fill":
            n = max(0, min(args[0], len(bits) - pos))     # clamped at the end, as the C helper
            bits[pos:pos + n] = [args[1]] * n
        else:
            raise ValueError(kind)
    return bits
