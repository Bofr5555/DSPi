/*------------------------------------------------------/
/ Copyright (c) 2023, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

// DSPi patch (WP FWSP1): the user-bit collector of _check_block, pure so the
// host test firmware/tests/test_spdif_ubits.c runs the same code.  The U bit of
// a raw subframe word is bit 29 (IEC 60958 time slot 29); user bit i of the
// block is subframe i's, stored as bit i mod 8 of byte i / 8, LSB first
// (alink_protocol_spec.md 12.1), the order of the channel-status bytes.

#ifndef SPDIF_RX_UBITS_H
#define SPDIF_RX_UBITS_H

#include <stdint.h>

#define SPDIF_RX_UBITS_LEN 48   // 384 subframes, one U bit each

static inline void spdif_rx_collect_ubits(const uint32_t *words, uint8_t out[SPDIF_RX_UBITS_LEN]) {
    for (int i = 0; i < SPDIF_RX_UBITS_LEN; i++) {
        uint32_t b = 0;
        for (int j = 0; j < 8; j++) b |= ((words[i * 8 + j] >> 29) & 1u) << j;
        out[i] = (uint8_t)b;
    }
}

#endif // SPDIF_RX_UBITS_H
