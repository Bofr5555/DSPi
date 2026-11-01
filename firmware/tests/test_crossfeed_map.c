// Host unit test for the crossfeed output pair map (firmware/DSPi/crossfeed_map.h).
//
//   firmware/tests/run.sh
//
// (cc -std=c11 -Wall -Wextra -Werror -I firmware/DSPi <this file>, then run.)  Pure functions only: no SDK.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "crossfeed_map.h"

static int failures = 0;
#define CHECK(cond) do { if (!(cond)) { \
    fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); failures++; } } while (0)

static uint32_t map4(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
    return (uint32_t)b0 | ((uint32_t)b1 << 8) | ((uint32_t)b2 << 16) | ((uint32_t)b3 << 24);
}

int main(void) {
    uint8_t l, r;

    // Zero = default pairing on every pair.
    for (int p = 0; p < 4; p++) {
        crossfeed_map_pair(0, p, &l, &r);
        CHECK(l == 2 * p && r == 2 * p + 1);
    }
    CHECK(crossfeed_map_valid(0, 4));   // RP2350: 4 pairs
    CHECK(crossfeed_map_valid(0, 2));   // RP2040: 2 pairs

    // Encoding: low nibble left, high nibble right.
    CHECK(crossfeed_map_encode_pair(2, 0) == 0x02);
    CHECK(crossfeed_map_encode_pair(3, 1) == 0x13);
    crossfeed_map_pair(map4(0x02, 0x13, 0, 0), 0, &l, &r);
    CHECK(l == 2 && r == 0);
    crossfeed_map_pair(map4(0x02, 0x13, 0, 0), 1, &l, &r);
    CHECK(l == 3 && r == 1);

    // K340: woofers (2,0) + tweeters (3,1), valid on both platforms.
    uint32_t k340 = map4(0x02, 0x13, 0, 0);
    CHECK(k340 == 0x00001302u);
    CHECK(crossfeed_map_valid(k340, 4));
    CHECK(crossfeed_map_valid(k340, 2));
    CHECK(crossfeed_map_normalize(k340) == k340);

    // Same output twice in one pair.
    CHECK(!crossfeed_map_valid(map4(0x11, 0, 0, 0), 4));
    // Output shared between pairs (pair 0 = (2,0) while pair 1 keeps default (2,3)).
    CHECK(!crossfeed_map_valid(map4(0x02, 0x00, 0, 0), 4));
    // Output shared, both explicit.
    CHECK(!crossfeed_map_valid(map4(0x10, 0x20, 0, 0), 4));
    // Out of range: RP2040 has outputs 0..3 only (4 = PDM sub).
    CHECK(!crossfeed_map_valid(map4(0x40, 0x00, 0, 0), 2));
    CHECK(!crossfeed_map_valid(map4(0x01, 0x42, 0, 0), 2));
    // RP2350: outputs 0..7, 8 = PDM sub, 15 never.
    CHECK(!crossfeed_map_valid(map4(0x80, 0, 0, 0), 4));
    CHECK(!crossfeed_map_valid(map4(0xF0, 0, 0, 0), 4));
    CHECK(crossfeed_map_valid(map4(0x76, 0x54, 0x32, 0x10), 4));   // reversed pairs
    // Bytes beyond num_pairs must be zero.
    CHECK(!crossfeed_map_valid(map4(0, 0, 0x54, 0), 2));
    // A pair may swap its own outputs.
    CHECK(crossfeed_map_valid(map4(0x01, 0, 0, 0), 4));
    // Bad pair count.
    CHECK(!crossfeed_map_valid(0, 0));
    CHECK(!crossfeed_map_valid(0, 5));

    // Normalization: explicit default bytes become 0, others untouched.
    CHECK(crossfeed_map_normalize(map4(0x10, 0x32, 0x54, 0x76)) == 0);
    CHECK(crossfeed_map_normalize(map4(0x10, 0x13, 0x02, 0x76)) == map4(0, 0x13, 0x02, 0));
    CHECK(crossfeed_map_valid(map4(0x10, 0x32, 0x54, 0x76), 4));

    // Exhaustive: every valid 2-pair map assigns the 4 outputs to two ordered
    // pairs: 4*3 choices for pair 0, 2 for pair 1 = 24 decoded configurations.
    // A pair in its default orientation has two encodings (0x00 and the
    // explicit byte): 2 configs have pair 0 = (0,1), 2 have pair 1 = (2,3),
    // 1 has both, so 24 + 2 + 2 + 1 = 29 valid byte combinations.
    int valid2 = 0;
    for (uint32_t b0 = 0; b0 < 256; b0++)
        for (uint32_t b1 = 0; b1 < 256; b1++) {
            uint32_t m = map4((uint8_t)b0, (uint8_t)b1, 0, 0);
            if (!crossfeed_map_valid(m, 2)) continue;
            valid2++;
            uint32_t n = crossfeed_map_normalize(m);
            CHECK(crossfeed_map_valid(n, 2));
            for (int p = 0; p < 2; p++) {   // normalization preserves the decoded pairs
                uint8_t l1, r1, l2, r2;
                crossfeed_map_pair(m, p, &l1, &r1);
                crossfeed_map_pair(n, p, &l2, &r2);
                CHECK(l1 == l2 && r1 == r2);
            }
        }
    CHECK(valid2 == 29);

    // Core ownership (RP2350 Core 1 = outputs 2..7, RP2040 = 2..3).
    CHECK(crossfeed_pair_owner(0, 1, 2, 7) == XF_OWNER_CORE0);
    CHECK(crossfeed_pair_owner(2, 3, 2, 7) == XF_OWNER_CORE1);
    CHECK(crossfeed_pair_owner(6, 7, 2, 7) == XF_OWNER_CORE1);
    CHECK(crossfeed_pair_owner(2, 0, 2, 7) == XF_OWNER_SPLIT);   // K340 woofers
    CHECK(crossfeed_pair_owner(3, 1, 2, 3) == XF_OWNER_SPLIT);   // K340 tweeters (RP2040)
    CHECK(crossfeed_pair_owner(1, 0, 2, 3) == XF_OWNER_CORE0);
    // Default pairs never split.
    for (int p = 0; p < 4; p++)
        CHECK(crossfeed_pair_owner((uint8_t)(2 * p), (uint8_t)(2 * p + 1), 2, 7) != XF_OWNER_SPLIT);

    // Preset slot round trip through the former tube_reserved_f bytes
    // (flash_storage.c collect_live_state / apply_slot_to_live).
    {
        uint8_t field[4];
        // Upstream V38/V39 firmware writes 0.0f there: the default map.
        float zero = 0.0f;
        memcpy(field, &zero, sizeof field);
        CHECK(crossfeed_map_load(field, 38, 4) == 0);
        CHECK(crossfeed_map_load(field, 39, 2) == 0);
        // Default map saves as four zero bytes (byte-identical to upstream).
        crossfeed_map_to_bytes(0, field);
        CHECK(field[0] == 0 && field[1] == 0 && field[2] == 0 && field[3] == 0);
        CHECK(memcmp(field, &zero, 4) == 0);
        CHECK(crossfeed_map_load(field, 39, 4) == 0);
        // K340 map saves as 02 13 00 00 and loads back on both platforms.
        crossfeed_map_to_bytes(k340, field);
        CHECK(field[0] == 0x02 && field[1] == 0x13 && field[2] == 0 && field[3] == 0);
        CHECK(crossfeed_map_from_bytes(field) == k340);
        CHECK(crossfeed_map_load(field, 39, 4) == k340);
        CHECK(crossfeed_map_load(field, 39, 2) == k340);
        CHECK(crossfeed_map_load(field, 38, 4) == k340);
        // Slots older than V38 have no such field: default, whatever the bytes.
        CHECK(crossfeed_map_load(field, 37, 4) == 0);
        CHECK(crossfeed_map_load(field, 21, 4) == 0);
        // Erased / garbage / invalid bytes: default.
        uint8_t ff[4] = {0xFF, 0xFF, 0xFF, 0xFF};
        CHECK(crossfeed_map_load(ff, 39, 4) == 0);
        uint8_t dup[4] = {0x02, 0x00, 0x00, 0x00};
        CHECK(crossfeed_map_load(dup, 39, 4) == 0);
        uint8_t wide[4] = {0x00, 0x00, 0x45, 0x00};   // RP2350-only pair (5,4) on RP2040
        CHECK(crossfeed_map_load(wide, 39, 2) == 0);
        CHECK(crossfeed_map_load(wide, 39, 4) == map4(0, 0, 0x45, 0));
        // Explicit default bytes load normalized.
        uint8_t expl[4] = {0x10, 0x32, 0x54, 0x76};
        CHECK(crossfeed_map_load(expl, 39, 4) == 0);
    }

    if (failures) {
        fprintf(stderr, "test_crossfeed_map: %d failure(s)\n", failures);
        return EXIT_FAILURE;
    }
    printf("test_crossfeed_map: OK\n");
    return EXIT_SUCCESS;
}
