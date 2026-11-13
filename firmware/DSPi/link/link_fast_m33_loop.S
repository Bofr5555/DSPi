@ link_fast_m33_loop.S - DSPi link receiver: clean-frame loop for the Cortex-M33 (WP LK5)
@
@   uint32_t lf_clean_m33(lf_clean_ctx_t *c);
@
@ The loop of lf_clean_fixed() in link_fast.c for its common case: 48/96 kHz,
@ one decoded bank that is a full bank, delivery of a whole bank in order (or
@ none).  lf_clean_fixed() computes everything that is constant for the call
@ into *c (link_fast.h, lf_clean_ctx_t, offsets asserted there) and the number
@ of frames n that lie before any boundary; this loop checks each frame's
@ content exactly as the C loop does and stops at the first frame that is not
@ clean, before touching anything of it.  Returns the frames taken; c->flo,
@ c->sf_count, c->prev_hdr and c->out are updated.  The C loop is the
@ reference (host tests) and the fallback for every other case.

    .syntax unified
    .cpu    cortex-m33
    .thumb
    .section .time_critical.link_m33, "ax", %progbits

    .equ C_WP,      0       @ word holding the first frame's comma end
    .equ C_N,       4       @ frames allowed
    .equ C_STEP,    8       @ bytes per frame
    .equ C_O,       12      @ comma end's bit offset in its word
    .equ C_RO,      16      @ 32 - o
    .equ C_XW,      20      @ next comma window: byte offset from wp
    .equ C_XS,      24      @   its bit offset
    .equ C_XRO,     28      @   32 - that
    .equ C_IW0,     32      @ padding: first word (byte offset)
    .equ C_IW1,     36      @   last word (byte offset)
    .equ C_FM,      40      @   first word mask (already ANDed with LM when one word)
    .equ C_LM,      44      @   last word mask (0 when one word or no padding)
    .equ C_PAT,     48      @   D21.5 pattern at this phase
    .equ C_MID,     52      @   words between first and last
    .equ C_DEC,     56
    .equ C_CRC8,    60
    .equ C_BW,      64      @ bank: byte offset of its first word from wp
    .equ C_BO,      68      @   bit offset
    .equ C_BANKFN,  72      @   lf_bank8_sw_m33 or lf_bank8_dma_m33
    .equ C_M33,     76      @   their context
    .equ C_SAMPLES, 80      @   &samples[8 b]
    .equ C_FAILS,   84      @   &bank_fails[b]
    .equ C_OUT,     88      @ next output row, 0 = no delivery (updated)
    .equ C_SRC,     92      @ delivery source: &samples[slot_run]
    .equ C_FLO,     96      @ low half of F (updated)
    .equ C_SFB,     100     @ sf_bytes
    .equ C_SFCNT,   104     @ sf_count (updated)
    .equ C_PREV,    108     @ prev_hdr (updated)

    .equ S_TMP,     0       @ the bank's samples
    .equ S_HDR,     32
    .equ S_CTL,     36
    .equ FRAME,     44      @ 36 pushed + 44 = 80, 8-aligned

    .global lf_clean_m33
    .type   lf_clean_m33, %function
    .thumb_func
    .p2align 2
lf_clean_m33:
    push    {r4-r11, lr}
    sub     sp, #FRAME
    mov     r4, r0                  @ c
    ldr     r5, [r4, #C_WP]         @ wp
    movs    r6, #0                  @ done
    ldr     r7, [r4, #C_N]
    ldr     r8, [r4, #C_FLO]
    ldr     r9, [r4, #C_SFCNT]
    ldr     r10, [r4, #C_PREV]
    ldr     r11, [r4, #C_OUT]

.Lframe:
    cmp     r6, r7
    bhs     .Lexit
    @ Head: bits p .. p + 29 -> HDR, CTL, CRC8 must be data, CRC-8 good, HDR = F.
    ldr     r0, [r4, #C_O]
    ldr     r2, [r5]
    ldr     r3, [r5, #4]
    ldr     r1, [r4, #C_RO]
    lsl     r2, r2, r0
    lsr     r3, r3, r1
    ldr     r12, [r4, #C_DEC]
    orr     r2, r2, r3
    lsr     r0, r2, #22
    ubfx    r1, r2, #12, #10
    ubfx    r3, r2, #2, #10
    ldr     r0, [r12, r0, lsl #2]
    ldr     r1, [r12, r1, lsl #2]
    ldr     r3, [r12, r3, lsl #2]
    ldr     r12, [r4, #C_CRC8]
    orr     r2, r0, r1
    orrs    r2, r2, r3
    bmi     .Lexit
    uxtb    r2, r0
    ldrb    r2, [r12, r2]
    eor     r2, r2, r1
    uxtb    r2, r2
    ldrb    r2, [r12, r2]
    uxtb    r3, r3
    cmp     r2, r3
    bne     .Lexit
    eor     r2, r0, r8
    lsls    r2, r2, #24
    bne     .Lexit
    str     r0, [sp, #S_HDR]
    str     r1, [sp, #S_CTL]
    @ Next comma exactly at e.
    ldr     r2, [r4, #C_XW]
    add     r2, r5, r2
    ldr     r3, [r2]
    ldr     r2, [r2, #4]
    ldr     r12, [r4, #C_XS]
    ldr     r1, [r4, #C_XRO]
    lsl     r3, r3, r12
    lsr     r2, r2, r1
    orr     r3, r3, r2
    ubfx    r3, r3, #20, #10
    cmp     r3, #0xFA
    beq     1f
    movw    r2, #0x305
    cmp     r3, r2
    bne     .Lexit
1:  @ Padding: all D21.5.
    ldr     r12, [r4, #C_PAT]
    ldr     r2, [r4, #C_IW0]
    ldr     r0, [r4, #C_FM]
    ldr     r2, [r5, r2]
    ldr     r3, [r4, #C_IW1]
    ldr     r1, [r4, #C_LM]
    ldr     r3, [r5, r3]
    eor     r2, r2, r12
    eor     r3, r3, r12
    and     r2, r2, r0
    and     r3, r3, r1
    orr     r2, r2, r3
    ldr     r3, [r4, #C_MID]
    cbz     r3, 3f
    ldr     lr, [r4, #C_IW0]
    add     lr, r5, lr
    add     lr, lr, #4
2:  ldr     r1, [lr], #4
    eor     r1, r1, r12
    orr     r2, r2, r1
    subs    r3, r3, #1
    bne     2b
3:  cmp     r2, #0
    bne     .Lexit
    @ The bank, into the temporaries.
    ldr     r0, [r4, #C_BW]
    ldr     r1, [r4, #C_BO]
    add     r0, r5, r0
    mov     r2, sp
    ldr     r3, [r4, #C_M33]
    ldr     r12, [r4, #C_BANKFN]
    blx     r12
    cmp     r0, #0
    beq     .Lexit
    @ Commit: samples, bank_fails = 0, delivery, superframe byte, F.
    ldr     r12, [r4, #C_SAMPLES]
    ldm     sp, {r0-r3}
    stm     r12!, {r0-r3}
    add     lr, sp, #16
    ldm     lr, {r0-r3}
    stm     r12, {r0-r3}
    ldr     r12, [r4, #C_FAILS]
    movs    r0, #0
    strb    r0, [r12]
    cmp     r11, #0
    beq     4f
    ldr     r12, [r4, #C_SRC]
    ldm     r12!, {r0-r3}
    stm     r11!, {r0-r3}
    ldm     r12, {r0-r3}
    stm     r11!, {r0-r3}
4:  ldr     r0, [sp, #S_HDR]
    ldr     r1, [sp, #S_CTL]
    ldr     r3, [r4, #C_SFB]
    uxtb    r0, r0
    ands    r2, r8, #31
    bne     5f
    strb    r1, [r3]
    mov     r9, #1
    b       6f
5:  cmp     r10, #0
    blt     7f
    add     r12, r10, #1
    uxtb    r12, r12
    cmp     r12, r0
    bne     7f
    cmp     r9, r2
    bne     7f
    strb    r1, [r3, r2]
    add     r9, r9, #1
    b       6f
7:  mov     r9, #0
6:  mov     r10, r0
    add     r8, r8, #1
    adds    r6, r6, #1
    ldr     r12, [r4, #C_STEP]
    add     r5, r5, r12
    b       .Lframe

.Lexit:
    str     r8, [r4, #C_FLO]
    str     r9, [r4, #C_SFCNT]
    str     r10, [r4, #C_PREV]
    str     r11, [r4, #C_OUT]
    mov     r0, r6
    add     sp, #FRAME
    pop     {r4-r11, pc}
    .size   lf_clean_m33, . - lf_clean_m33
