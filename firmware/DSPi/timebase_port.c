/*
 * timebase_port.c - the firmware's port of timebase.c (WP FWTB1): interrupt
 * masking, the clock, slot 0's ring counters and the 0xE4 push.  Flash.
 */
#include "timebase.h"

#include "audio_pipeline.h"
#include "notify.h"
#include "hardware/sync.h"
#include "hardware/timer.h"

uint32_t timebase_port_irq_save(void) { return save_and_disable_interrupts(); }
void timebase_port_irq_restore(uint32_t s) { restore_interrupts(s); }
uint32_t timebase_port_now_us(void) { return time_us_32(); }
bool timebase_port_ring(uint32_t *wr, uint32_t *played) { return slot_ring_counters(0, wr, played); }
void timebase_port_event(uint8_t tb, uint8_t state, uint8_t reason, uint32_t t_low) {
    notify_push_time_base(tb, state, reason, t_low);
}
