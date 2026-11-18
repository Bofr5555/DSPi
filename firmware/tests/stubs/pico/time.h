// Host-test stand-in for the Pico SDK's pico/time.h: a test-driven clock.
#pragma once
#include <stdint.h>
extern uint64_t stub_time_us;
static inline uint64_t time_us_64(void) { return stub_time_us; }
static inline uint32_t time_us_32(void) { return (uint32_t)stub_time_us; }
