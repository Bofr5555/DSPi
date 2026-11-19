// Host-test stand-in for the Pico SDK's hardware/pio.h: only the PIO handle
// type the S/PDIF instance struct names (WP FWSP1, test_spdif_ubits.c).
#pragma once
#include <stdint.h>
typedef struct { volatile uint32_t ctrl; volatile uint32_t txf[4]; } pio_hw_t;
typedef pio_hw_t *PIO;
