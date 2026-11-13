#!/usr/bin/env python3
"""gen_link_fast_m33.py - write firmware/DSPi/link/link_fast_m33.S (WP LK5)

The bank decoder of link_fast.c (bank8 / bank8_dma) as unrolled Cortex-M33
Thumb-2, generated here so that every symbol's bit position, register and
shift is computed rather than typed:

    bool lf_bank8_sw_m33 (const uint32_t *r, unsigned o, int32_t *out, const lf_m33_ctx_t *c);
    bool lf_bank8_dma_m33(const uint32_t *r, unsigned o, int32_t *out, const lf_m33_ctx_t *c);

r points at the word holding the bank's first bit, o is that bit's offset
(0..31); r[0..9] are read.  The 26 symbols are realigned on the fly (three
rotating registers, one load and three ALU operations per word), extracted
with constant UBFX/shift pairs, decoded through c->dec, joined three at a
time (e2 | e1 << 8 | e0 << 16, bit 31 = "not a data code group") and
checked against the bank's CRC-16: by the 256-entry table (sw), or by the
DMA sniffer on channel registers c->dma_ch (dma), which runs while the
samples are written.  Returns 1 when all 26 symbols are data code groups
and the CRC matches; out[0..7] receive the samples either way.  Exactly
the semantics of the C functions, which stay the reference (host tests)
and the fallback.

usage: gen_link_fast_m33.py > firmware/DSPi/link/link_fast_m33.S
"""

CTX_DEC, CTX_CRC16, CTX_DMA = 0, 4, 8      # lf_m33_ctx_t offsets (link_fast.h asserts them)
SNIFF_DATA = 0x50000458
DMA_BUSY = 0x04000000

A = ["r2", "r4", "r5"]      # rotating realigned words: a_k lives in A[k % 3]
T = ["r6", "r7", "r8"]      # symbol / entry temporaries
# r0 raw pointer, r1 o, ip 32 - o, r3 dec, lr raw word, r9 err, r10 crc, r11 crc table / scratch
FRAME = 68                  # u[8] @0, w[6] @32, out @56, ctx @60, pad: 36 + 68 = 104, 8-aligned
U, W, OUT, CTX = 0, 32, 56, 60


import re

REG_RE = re.compile(r"\b(r1[0-2]|r[0-9]|ip|lr|sp)\b")
ALIAS = {"ip": "r12"}


def regs_of(txt):
    return [ALIAS.get(x, x) for x in REG_RE.findall(txt)]


def deps(ins):
    """(defs, uses, kind) of one instruction line; kind 'load', 'store' or 'alu'."""
    op, _, rest = ins.strip().partition(" ")
    rest = rest.strip()
    base = op.split(".")[0]
    if base in ("str", "strh", "strb"):
        rs = regs_of(rest)
        d = {rs[1]} if "], #" in rest or "]!" in rest else set()     # writeback
        return d, set(rs), "store"
    if base == "stm":
        rn, _, lst = rest.partition(",")
        d = set(regs_of(rn)) if "!" in rn else set()                # writeback
        return d, set(regs_of(rest)), "store"
    if base in ("ldr", "ldrh", "ldrb"):
        rs = regs_of(rest)
        d = {rs[0]}
        u = set(rs[1:])
        if "], #" in rest or "]!" in rest:      # writeback
            d.add(rs[1])
        return d, u, "load"
    if base == "ldm":
        rn, _, lst = rest.partition(",")
        return set(regs_of(lst)), set(regs_of(rn)), "load"
    rs = regs_of(rest)
    return {rs[0]} if rs else set(), set(rs[1:]), "alu"


def schedule(block):
    """List-schedule a straight-line block: loads have latency 2, everything
    else 1; register RAW/WAR/WAW and memory order (loads after earlier
    stores, stores after earlier loads and stores) are kept."""
    n = len(block)
    info = [deps(x) for x in block]
    preds = [set() for _ in range(n)]
    for i in range(n):
        di, ui, ki = info[i]
        for j in range(i):
            dj, uj, kj = info[j]
            if (ui & dj) or (di & uj) or (di & dj):
                preds[i].add(j)
            if ki != "alu" and kj != "alu" and (ki == "store" or kj == "store"):
                preds[i].add(j)
    lat = [2 if info[i][2] == "load" else 1 for i in range(n)]
    succ = [[] for _ in range(n)]
    for i in range(n):
        for j in preds[i]:
            succ[j].append(i)
    prio = [0] * n
    for i in reversed(range(n)):
        prio[i] = lat[i] + max((prio[k] for k in succ[i]), default=0)
    ready_at = [0] * n
    done = [False] * n
    out, cycle = [], 0
    while len(out) < n:
        cand = [i for i in range(n) if not done[i] and all(done[j] for j in preds[i]) and ready_at[i] <= cycle]
        if not cand:
            cycle += 1
            continue
        i = max(cand, key=lambda k: (prio[k], -k))
        done[i] = True
        out.append(block[i])
        for k in succ[i]:
            # a successor that reads a load's result waits one more cycle
            r = cycle + (lat[i] if (info[k][1] & info[i][0]) else 1)
            ready_at[k] = max(ready_at[k], r)
        cycle += 1
    return out


class Gen:
    def __init__(self):
        self.lines = []
        self.done = -1          # realigned words complete: a_0 .. a_done

    def e(self, s):
        self.lines.append("    " + s)

    def need(self, k):
        """Complete realigned words up to a_k (each step also starts a_{k+1})."""
        while self.done < k:
            j = self.done + 1
            self.e("ldr     lr, [r0], #4")
            self.e("lsl     %s, lr, r1" % A[(j + 1) % 3])
            self.e("lsr     lr, lr, ip")
            self.e("orr     %s, %s, lr" % (A[j % 3], A[j % 3]))
            self.done = j

    def sym(self, j, t):
        """Symbol j of the bank (bits 10j .. 10j+9 of the realigned stream) into t."""
        bit = 10 * j
        w, off = bit >> 5, bit & 31
        if off <= 22:
            self.need(w)
            if off == 0:
                self.e("lsr     %s, %s, #22" % (t, A[w % 3]))
            else:
                self.e("ubfx    %s, %s, #%d, #10" % (t, A[w % 3], 22 - off))
        else:
            self.need(w + 1)
            self.e("lsl     %s, %s, #%d" % (t, A[w % 3], off - 22))
            self.e("orr     %s, %s, %s, lsr #%d" % (t, t, A[(w + 1) % 3], 54 - off))
            self.e("ubfx    %s, %s, #0, #10" % (t, t))

    def body(self, crc_sw):
        self.e("push    {r4-r11, lr}")
        self.mark = len(self.lines)
        self.e("sub     sp, #%d" % FRAME)
        self.e("str     r2, [sp, #%d]" % OUT)
        self.e("str     r3, [sp, #%d]" % CTX)
        if crc_sw:
            self.e("ldr     r11, [r3, #%d]" % CTX_CRC16)
            self.e("movw    r10, #0xFFFF")
        self.e("ldr     r3, [r3, #%d]" % CTX_DEC)
        self.e("rsb     ip, r1, #32")
        self.e("ldr     lr, [r0], #4")
        self.e("lsl     %s, lr, r1" % A[0])
        for i in range(8):
            for k in range(3):
                self.sym(3 * i + k, T[k])
            for k in range(3):
                self.e("ldr     %s, [r3, %s, lsl #2]" % (T[k], T[k]))
            self.e("orr     r8, r8, r7, lsl #8")
            self.e("orr     r8, r8, r6, lsl #16")
            self.e(("mov     r9, r8") if i == 0 else "orr     r9, r9, r8")
            self.e("str     r8, [sp, #%d]" % (U + 4 * i))
            if crc_sw:
                self.e("eor     r6, r10, r8, lsr #8")
                self.e("ubfx    r6, r6, #8, #8")
                self.e("ldrh    r6, [r11, r6, lsl #1]")
                self.e("eor     r10, r6, r10, lsl #8")
                self.e("eor     r6, r10, r8")
                self.e("ubfx    r6, r6, #8, #8")
                self.e("ldrh    r6, [r11, r6, lsl #1]")
                self.e("eor     r10, r6, r10, lsl #8")
                self.e("eor     r6, r8, r10, lsr #8")
                self.e("uxtb    r6, r6")
                self.e("ldrh    r6, [r11, r6, lsl #1]")
                self.e("eor     r10, r6, r10, lsl #8")
        # The CRC symbols: rx = byte24 | byte25 << 8 (low byte first), their
        # validity into err.
        self.sym(24, "r6")
        self.sym(25, "r7")
        self.e("ldr     r6, [r3, r6, lsl #2]")
        self.e("ldr     r7, [r3, r7, lsl #2]")
        self.e("orr     r9, r9, r6")
        self.e("orr     r9, r9, r7")
        self.e("uxtb    r6, r6")
        self.e("uxtb    r7, r7")
        self.e("orr     r6, r6, r7, lsl #8")
        if crc_sw:
            self.e("eor     r10, r10, r6")
        else:
            self.e("mov     r10, r6")
            # Pack the 24 payload bytes MSB first into w[0..5] and start the sniffer.
            for g in range(2):
                self.e("add     r6, sp, #%d" % (U + 16 * g))
                self.e("ldm     r6, {r2, r4, r5, r6}")
                self.e("ubfx    r8, r4, #16, #8")
                self.e("orr     r8, r8, r2, lsl #8")
                self.e("ubfx    r11, r5, #8, #16")
                self.e("orr     r11, r11, r4, lsl #16")
                self.e("ubfx    lr, r6, #0, #24")
                self.e("orr     lr, lr, r5, lsl #24")
                self.e("add     r6, sp, #%d" % (W + 12 * g))
                self.e("stm     r6, {r8, r11, lr}")
            self.e("dmb")
            self.e("ldr     r4, [sp, #%d]" % CTX)
            self.e("ldr     r4, [r4, #%d]" % CTX_DMA)
            self.e("movw    r5, #0x%04x" % (SNIFF_DATA & 0xFFFF))
            self.e("movt    r5, #0x%04x" % (SNIFF_DATA >> 16))
            self.e("movw    r6, #0xFFFF")
            self.e("str     r6, [r5]")
            self.e("add     r6, sp, #%d" % W)
            self.e("str     r6, [r4]")
            self.e("movs    r6, #6")
            self.e("str     r6, [r4, #0x1c]")
        # Samples: sign-extend from bit 23 into out[0..7] (while the DMA runs).
        self.e("ldr     r0, [sp, #%d]" % OUT)
        for g in range(2):
            self.e("add     r1, sp, #%d" % (U + 16 * g))
            self.e("ldm     r1, {r1, r2, r3, r8}")
            for rr in ("r1", "r2", "r3", "r8"):
                self.e("sbfx    %s, %s, #0, #24" % (rr, rr))
            self.e("stm     r0!, {r1, r2, r3, r8}")
        if not crc_sw:
            self.sched_end_dma = len(self.lines)
            self.lines.append("1:")
            self.e("ldr     r6, [r4, #0x10]")
            self.e("tst     r6, #0x%08x" % DMA_BUSY)
            self.e("bne     1b")
            self.e("ldr     r6, [r5]")
            self.e("eor     r10, r10, r6")
        # ok = crc matches (low 16 bits) and no symbol was bad (bit 31 of err).
        self.sched_end = len(self.lines)
        self.e("movs    r0, #0")
        self.e("lsls    r10, r10, #16")
        self.e("bne     2f")
        self.e("cmp     r9, #0")
        self.e("it      ge")
        self.e("movge   r0, #1")
        self.lines.append("2:")
        self.e("add     sp, #%d" % FRAME)
        self.e("pop     {r4-r11, pc}")


def func(name, crc_sw):
    g = Gen()
    g.body(crc_sw)
    end = g.sched_end if crc_sw else g.sched_end_dma
    # The DMA start (dmb, register writes) must stay after the word stores and
    # before the sample writes: schedule the two parts around it separately.
    body = g.lines[g.mark:end]
    if not crc_sw:
        k = body.index("    dmb")
        kk = k + 9      # dmb + 8 instructions that program the sniffer
        body = schedule(body[:k]) + body[k:kk] + schedule(body[kk:])
    else:
        body = schedule(body)
    g.lines = g.lines[:g.mark] + body + g.lines[end:]
    return "\n".join([
        # One section per function: a build that never calls one (DSPi uses
        # only the DMA-CRC decoder) drops it with --gc-sections.
        '    .section .time_critical.link_m33.%s, "ax", %%progbits' % name,
        "    .global %s" % name,
        "    .type   %s, %%function" % name,
        "    .thumb_func",
        "    .p2align 2",
        "%s:" % name,
    ] + g.lines + ["    .size   %s, . - %s" % (name, name), ""])


def main():
    print("""@ link_fast_m33.S - DSPi link receiver: bank decoder for the Cortex-M33 (WP LK5)
@
@ GENERATED by tools/link/gen_link_fast_m33.py; edit the generator, not this file.
@ The C functions bank8() / bank8_dma() in link_fast.c are the reference: same
@ arguments' meaning, same result, same out[0..7].  Placed in .time_critical
@ (RAM) like the hot C code.

    .syntax unified
    .cpu    cortex-m33
    .thumb
""")
    print(func("lf_bank8_sw_m33", True))
    print(func("lf_bank8_dma_m33", False))


if __name__ == "__main__":
    main()
