/*
 * link_8b10b_shim.h; minimal 8b/10b encoder for the link PIO bench model.
 *
 * Test-only. It exists so the PIO model (WP LK2) can run before the
 * reference codec (WP LK1) lands; it produces line-realistic streams (run
 * length <= 5, bounded disparity, K28.5 commas) and nothing more. Swap it for
 * LK1's encoder by providing the same function: the model needs only
 * link_shim_encode() and the bit-order contract below.
 *
 * Bit-order contract (must match LK1): a 10-bit code group is held as
 * abcdei fghj with bit 'a' as bit 9; 'a' goes on the wire first. The model
 * packs code groups MSB first into 32-bit words, which is what link_tx_N
 * (OUT shift left) transmits and link_rx_N (IN shift left) returns.
 */
#ifndef LINK_8B10B_SHIM_H
#define LINK_8B10B_SHIM_H

#include <stdbool.h>
#include <stdint.h>

// 5b/6b for RD-, indexed by EDCBA; bit 6 flags "RD+ uses the complement".
#define ALT6 0x40u
static const uint8_t link_shim_6b[32] = {
    0x27 | ALT6, 0x1D | ALT6, 0x2D | ALT6, 0x31,        0x35 | ALT6, 0x29,        0x19,        0x38 | ALT6,
    0x39 | ALT6, 0x25,        0x15,        0x34,        0x0D,        0x2C,        0x1C,        0x17 | ALT6,
    0x1B | ALT6, 0x23,        0x13,        0x32,        0x0B,        0x2A,        0x1A,        0x3A | ALT6,
    0x33 | ALT6, 0x26,        0x16,        0x36 | ALT6, 0x0E,        0x2E | ALT6, 0x1E | ALT6, 0x2B | ALT6,
};
// 3b/4b for RD-, indexed by HGF (7 = primary P7); bit 4 flags the complement.
#define ALT4 0x10u
static const uint8_t link_shim_4b[8] = {
    0x0B | ALT4, 0x09, 0x05, 0x0C | ALT4, 0x0D | ALT4, 0x0A, 0x06, 0x0E | ALT4,
};
#define LINK_SHIM_A7 (0x07 | ALT4)

static inline int link_shim_ones(uint32_t v) { return __builtin_popcount(v); }

#define LINK_K28_5_RDN 0x0FAu   // 001111 1010
#define LINK_K28_5_RDP 0x305u   // 110000 0101

// Encode one byte (k = true: only K28.5 is supported). *rd_pos is the
// running disparity (false = RD-). Returns the 10-bit code group.
static inline uint32_t link_shim_encode(uint8_t byte, bool k, bool *rd_pos) {
    if (k) {
        uint32_t cg = *rd_pos ? LINK_K28_5_RDP : LINK_K28_5_RDN;
        *rd_pos = !*rd_pos;   // 6b part unbalanced, 4b part balanced
        return cg;
    }
    uint32_t x = byte & 0x1Fu, y = byte >> 5;
    uint32_t e6 = link_shim_6b[x];
    uint32_t c6 = e6 & 0x3Fu;
    if (*rd_pos && (e6 & ALT6)) c6 ^= 0x3Fu;
    if (link_shim_ones(c6) != 3) *rd_pos = !*rd_pos;
    uint32_t e4 = link_shim_4b[y];
    if (y == 7 && ((!*rd_pos && (x == 17 || x == 18 || x == 20)) ||
                   (*rd_pos && (x == 11 || x == 13 || x == 14))))
        e4 = LINK_SHIM_A7;
    uint32_t c4 = e4 & 0x0Fu;
    if (*rd_pos && (e4 & ALT4)) c4 ^= 0x0Fu;
    if (link_shim_ones(c4) != 2) *rd_pos = !*rd_pos;
    return (c6 << 4) | c4;
}

#endif
