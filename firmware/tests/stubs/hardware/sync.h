// Host-test stand-in for the Pico SDK's hardware/sync.h (single-threaded).
#pragma once
#include <stdint.h>
static inline uint32_t save_and_disable_interrupts(void) { return 0; }
static inline void restore_interrupts(uint32_t f) { (void)f; }
static inline void __dmb(void) {}
