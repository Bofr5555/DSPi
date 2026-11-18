// Host-test stand-in for the Pico SDK's hardware/gpio.h: the test records
// pin levels through stub_gpio_put() (WP FWSIGGEN, test_siggen_dac.c).
#pragma once
#include <stdbool.h>
#include <stdint.h>
#define GPIO_IN  0
#define GPIO_OUT 1
void stub_gpio_put(unsigned pin, bool level);
static inline void gpio_init(unsigned pin) { (void)pin; }
static inline void gpio_set_dir(unsigned pin, int out) { (void)pin; (void)out; }
static inline void gpio_put(unsigned pin, bool level) { stub_gpio_put(pin, level); }
