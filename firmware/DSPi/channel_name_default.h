/*
 * channel_name_default.h - does a stored channel name still follow its default?
 *
 * A channel name equal to the current default is relabelled when the source or
 * the output type changes; any other name is the user's and stays.  WP ALINK2
 * renamed the input-7 default from "LINK n" to "ALINK n", so a preset saved
 * under older firmware holds the legacy spelling: it still counts as the
 * default (review alink2 finding 1).  Pure C, no SDK (host-tested).
 */
#ifndef CHANNEL_NAME_DEFAULT_H
#define CHANNEL_NAME_DEFAULT_H

#include <stdbool.h>
#include <string.h>

static inline bool channel_name_is_default(const char *name, const char *dflt) {
    if (strcmp(name, dflt) == 0) return true;
    // Legacy alink default: "LINK n" for the current "ALINK n".
    return strncmp(dflt, "ALINK ", 6) == 0 && strcmp(name, dflt + 1) == 0;
}

#endif
