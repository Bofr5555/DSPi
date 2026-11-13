/*
 * sender_store.c - DSPi link sender (WP LK4): persisted link settings
 *
 * EXT_LINK_TX and EXT_LINK_CONFIG persist through REQ_SAVE_OUTPUT_CONFIG
 * (0x52), as on a receiver (extension_channel_spec.md 2.8), in one flash
 * sector of their own just below the area DSPi's presets use (the last 21
 * sectors on RP2350), so a board that later goes back to DSPi firmware finds
 * its preset area untouched.  The image runs from RAM (copy_to_ram), so core
 * 1 keeps sending through the erase; core 0 has its interrupts off for it
 * (about 50 ms: USB audio drops a few packets, so save while idle).
 */

#include <string.h>

#include "hardware/flash.h"
#include "hardware/sync.h"

#include "flash_clkdiv.h"
#include "sender.h"

#define STORE_OFFSET    (PICO_FLASH_SIZE_BYTES - 22u * FLASH_SECTOR_SIZE)
#define STORE_MAGIC     0x4B4C5331u     // "LKS1"

typedef struct {
    uint32_t magic;
    uint8_t  tx[SND_TX_LEN];        // EXT_LINK_TX IN form (IN-only fields 0)
    uint8_t  config[8];             // EXT_LINK_CONFIG
    uint8_t  channels_req;
    uint8_t  pad[1];
    uint16_t crc;                   // CRC-16/CCITT-FALSE over the bytes before it
} store_t;

_Static_assert(sizeof(store_t) <= FLASH_PAGE_SIZE, "one flash page");

void sender_store_load(void) {
    const store_t *st = (const store_t *)(XIP_BASE + STORE_OFFSET);
    if (st->magic != (uint32_t)(STORE_MAGIC)) return;
    if (link_crc16((const uint8_t *)st, offsetof(store_t, crc)) != st->crc) return;
    snd_txcfg_t c;
    if (snd_txcfg_parse(st->tx, SND_MAX_LINKS, &c) == 0) {
        snd.cfg = c;
        snd.channels_req = st->channels_req >= 1 && st->channels_req <= LINK_MAX_CHANNELS ? st->channels_req
                                                                                          : c.channels;
    }
    const uint8_t *p = st->config;
    if (p[0] <= 1 && p[2] <= 3 && !(p[6] & ~3u) && !p[7] && p[3] == 0xFF && snd_pin_ok(p[1]) &&
        snd_pin_ok(p[4]) && snd_pin_ok(p[5]))
        memcpy(snd.config, p, 8);
}

bool sender_store_save(void) {
    static uint8_t page[FLASH_PAGE_SIZE] __attribute__((aligned(4)));
    store_t st;
    memset(&st, 0, sizeof(st));
    st.magic = (uint32_t)(STORE_MAGIC);
    uint32_t s = snd_lock_take();
    snd_txcfg_pack(&snd.cfg, 0, 0, st.tx);
    memcpy(st.config, snd.config, 8);
    st.channels_req = snd.channels_req;
    snd_lock_give(s);
    st.crc = link_crc16((const uint8_t *)&st, offsetof(store_t, crc));
    memset(page, 0xFF, sizeof(page));
    memcpy(page, &st, sizeof(st));
    uint32_t irq = save_and_disable_interrupts();
    dspi_flash_range_erase(STORE_OFFSET, FLASH_SECTOR_SIZE);
    dspi_flash_range_program(STORE_OFFSET, page, FLASH_PAGE_SIZE);
    restore_interrupts(irq);
    return memcmp((const void *)(XIP_BASE + STORE_OFFSET), page, sizeof(st)) == 0;
}
