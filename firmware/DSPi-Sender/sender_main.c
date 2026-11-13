/*
 * sender_main.c - DSPi link sender (WP LK4): boot and the core 0 loop
 *
 * SYS2 section 4: a Pico 2 as a USB sound card driving link outputs.  Boot:
 * 307.2 MHz from the crystal (the class dividers 20 / 10 / 5 and the 48 kHz
 * frame of 6400 cycles are exact there), core at 1.20 V, the soft VCXO
 * parked (it moves only in fan-out mode), the link engine on core 1, USB on
 * core 0.  The sender sends from power-up (frames of zero samples until USB
 * audio plays, spec 7.1); which links send is EXT_LINK_TX.link_enable.
 *
 * Bench build (DSPI_SENDER_BENCH=ON): link 0 (GP2) enabled with the PRBS-7
 * test pattern and the SYNC pulse on GP21 from boot, so a link bench RX
 * board (LK3) checks it with no host software on the sender's side.
 */

#include <string.h>

#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"

#include "flash_clkdiv.h"
#include "sender.h"
#include "soft_vcxo.h"
#include "usb_descriptors.h"

sender_t snd;
spin_lock_t *snd_lock;
snd_tables_t snd_tables;

void sender_defaults(void) {
    snd_txcfg_default(&snd.cfg);
#ifdef DSPI_SENDER_BENCH
    snd.cfg.link_enable = 0x0001;
    snd.cfg.test_pattern = SND_TP_PRBS;
    snd.cfg.sync_pin = 21;
#endif
    snd.channels_req = snd.cfg.channels;
    static const uint8_t cfg[8] = { 0, 0xFF, 0, 0xFF, 0xFF, 0xFF, 0x03, 0 };
    memcpy(snd.config, cfg, 8);
}

static void led_service(void) {
    // On: links running.  Fast blink: playing USB audio.  Slow blink: fan-out
    // re-sending.  Off: links stopped.
    uint32_t ms = to_ms_since_boot(get_absolute_time());
    bool on;
    if (snd.engine == SND_ENG_OFF) on = false;
    else if (snd.usb_playing) on = (ms / 100u) & 1u;
    else if (snd.engine == SND_ENG_FANOUT) on = (ms / 500u) & 1u;
    else on = true;
    gpio_put(SND_LED_PIN, on);
}

int main(void) {
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    busy_wait_ms(2);
    if (!set_sys_clock_hz(307200000, false)) set_sys_clock_hz(150000000, false);
    dspi_flash_apply_clkdiv();

    gpio_init(23);                      // RT6150 forced PWM, as DSPi
    gpio_set_dir(23, GPIO_OUT);
    gpio_put(23, 1);
    gpio_init(SND_LED_PIN);
    gpio_set_dir(SND_LED_PIN, GPIO_OUT);

    pico_get_unique_board_id_string(usb_descriptor_str_serial, 17);
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);
    snd.sender_id = id.id[PICO_UNIQUE_BOARD_ID_SIZE_BYTES - 1];

    // The VCXO claims its fixed DMA channels before the engine claims any.
    soft_vcxo_init();
    snd_tables_init(&snd_tables);
    snd_lock = spin_lock_init(spin_lock_claim_unused(true));
    sender_defaults();
    sender_store_load();

    snd.restart_req = 1;                // core 1 starts the links
    multicore_launch_core1(sender_engine_core1);
    sender_usb_init();

    for (;;) {
        sender_usb_task();
        sender_follow_usb_rate();
        if (snd.save_req) {
            snd.save_req = false;
            sender_store_save();
        }
        led_service();
    }
}
