// Host test for config.h gain_db_clamp() (FW2 gain clamps): finite values
// clamp into the range, NaN and both infinities land on the quiet end.
#include "../DSPi/config.h"

#include <math.h>
#include <stdio.h>

static int failures;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

int main(void) {
    const float lo = OUTPUT_GAIN_MIN_DB, hi = OUTPUT_GAIN_MAX_DB;
    CHECK(gain_db_clamp(0.0f, lo, hi) == 0.0f);
    CHECK(gain_db_clamp(-6.5f, lo, hi) == -6.5f);
    CHECK(gain_db_clamp(lo, lo, hi) == lo);
    CHECK(gain_db_clamp(hi, lo, hi) == hi);
    CHECK(gain_db_clamp(-80.0f, lo, hi) == lo);
    CHECK(gain_db_clamp(35.0f, lo, hi) == hi);
    CHECK(gain_db_clamp(3.4e38f, lo, hi) == hi);
    CHECK(gain_db_clamp(NAN, lo, hi) == lo);
    CHECK(gain_db_clamp(-NAN, lo, hi) == lo);
    CHECK(gain_db_clamp(INFINITY, lo, hi) == lo);
    CHECK(gain_db_clamp(-INFINITY, lo, hi) == lo);
    // Host build = RP2040 limits: +18 dB keeps the Q28 preamp multiplier in
    // range (10^(18/20) = 7.943, times 2^28 = 2.13e9 < 2^31).
    CHECK(PREAMP_MAX_DB == 18.0f);
    if (failures) { printf("test_gain_clamp: %d failure(s)\n", failures); return 1; }
    printf("test_gain_clamp: OK\n");
    return 0;
}
