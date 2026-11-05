/*
 * dspi_no_printf.h; compiles printf()/puts() diagnostics out of the firmware.
 *
 * Force-included (gcc -include) into DSPi's own sources and the vendored
 * pico-extras audio/SPDIF-RX sources when DSPI_DEBUG_PRINTF is OFF (the
 * default).  No stdio driver is linked (UART stdio is disabled and there is
 * no USB CDC), so these calls formatted text that went nowhere.  The
 * replacements are empty inline functions: arguments are still evaluated
 * (any side effects are kept), the format strings are still checked by
 * -Wformat, and the calls, their strings and the pico_stdio/printf
 * machinery drop out of the link.  snprintf() into buffers (display text,
 * channel names) is untouched.
 */
#ifndef DSPI_NO_PRINTF_H
#define DSPI_NO_PRINTF_H

#include <stdio.h>   // declare the real ones first; the macros only affect later uses

static inline __attribute__((format(printf, 1, 2)))
int dspi_printf_nop(const char *fmt, ...) { (void)fmt; return 0; }

static inline int dspi_puts_nop(const char *s) { (void)s; return 0; }

#define printf dspi_printf_nop
#define puts   dspi_puts_nop

#endif // DSPI_NO_PRINTF_H
