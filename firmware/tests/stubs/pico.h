// Host-test stand-in for the Pico SDK's pico.h (WP FWSP1, test_spdif_ubits.c).
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef unsigned int uint;
static inline uint32_t __mul_instruction(uint32_t a, uint32_t b) { return a * b; }
