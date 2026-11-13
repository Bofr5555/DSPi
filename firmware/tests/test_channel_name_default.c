// Host test: channel_name_is_default() (review alink2 finding 1).
#include <stdio.h>
#include "channel_name_default.h"

static int checks, failures;
#define CHECK(c, m) do { checks++; if (!(c)) { failures++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, m); } } while (0)

int main(void) {
    CHECK(channel_name_is_default("ALINK 1", "ALINK 1"), "current alink default");
    CHECK(channel_name_is_default("LINK 1", "ALINK 1"), "legacy LINK 1 is the default");
    CHECK(channel_name_is_default("LINK 8", "ALINK 8"), "legacy LINK 8 is the default");
    CHECK(!channel_name_is_default("LINK 2", "ALINK 1"), "legacy name of another channel");
    CHECK(!channel_name_is_default("Kick", "ALINK 1"), "a user name stays");
    CHECK(!channel_name_is_default("LINK 1", "USB 1"), "legacy spelling only against the alink default");
    CHECK(!channel_name_is_default("SB 1", "USB 1"), "no prefix stripping for other sources");
    CHECK(channel_name_is_default("USB 1", "USB 1"), "other defaults unchanged");
    CHECK(!channel_name_is_default("ALINK 1", "USB 1"), "an alink name is not the USB default");
    printf("test_channel_name_default: %d checks, %d failures\n", checks, failures);
    return failures != 0;
}
