/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026 Diego Asanza <f.asanza@gmail.com> */

/* Simple, portable-ish timer implementation for the micro:bit timer
 * peripheral used in this chapter. The implementation intentionally
 * keeps the public API tiny (see hal_timer.h) and the platform file
 * contains only the minimal code required for the examples.
 */

#include <assert.h>
#include <hal_timer.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#define TIMER0_BASE (0x40008000U)
#define TIMER1_BASE (0x40009000U)
#define TIMER2_BASE (0x4000A000U)

#define TIMER0_IRQN (8U)

/* Timer register offsets (only what we use) */
#define TASK_START_OFFSET     (0x00u)
#define TASK_STOP_OFFSET      (0x04u)
#define TASK_CLEAR_OFFSET     (0x0Cu)
#define EVENT_COMPARE0_OFFSET (0x140u)

#define INTENSET_OFFSET  (0x304u)
#define MODE_OFFSET      (0x504u)
#define BITMODE_OFFSET   (0x508u)
#define PRESCALER_OFFSET (0x510u)
#define CC0_OFFSET       (0x540u)

#define TIMER_MODE_TIMER        (0u)
#define TIMER_BITMODE_32BIT     (3u)
#define TIMER_PRESCALER_16      (4u)
#define TIMER_INTENSET_COMPARE0 (1u << 16)

#define NVIC_ISER0 (0xE000E100u)

#define REG(addr) (*(volatile uint32_t *)(addr))

static inline void __wfi(void) { __asm volatile("wfi" : : : "memory"); }


static inline void NVIC_EnableIRQ(uint32_t IRQn) {
    if ((int32_t)IRQn >= 0) {
        REG(NVIC_ISER0) |= (1u << (IRQn & 0x1Fu));
    }
}

/* Minimal timer descriptor retained for future implementation. For now
 * functions below provide skeletal/no-op behaviour matching the public
 * API in hal_timer.h so the microbit platform compiles alongside the
 * Nucleo implementation.
 */
struct timer_desc {
    uint32_t base;
    bool one_shot;
    uint32_t period_us;
    hal_event_mask_t event;
};

static struct timer_desc timers[] = {
    {TIMER0_BASE, false, 0, 0},
};

/* Match the public API: hal_timer_init(int, bool, hal_event_mask_t) */
void hal_timer_init(int timer_instance, bool one_shoot, hal_event_mask_t event) {
    (void)one_shoot;
    (void)event;
    if ((unsigned)timer_instance >= (sizeof(timers) / sizeof(timers[0]))) return;
    timers[timer_instance].one_shot = one_shoot;
    timers[timer_instance].event = event;
    /* Clear pending event and enable IRQ in NVIC to mirror behaviour.
     * The detailed peripheral setup is intentionally left for later.
     */
    REG(timers[timer_instance].base + EVENT_COMPARE0_OFFSET) = 0u;
    REG(timers[timer_instance].base + INTENSET_OFFSET) |= TIMER_INTENSET_COMPARE0;
    NVIC_EnableIRQ(TIMER0_IRQN + (uint32_t)timer_instance);
}

/* Match public API: hal_timer_start(int, uint32_t) */
void hal_timer_start(int timer_instance, uint32_t period_us) {
    if ((unsigned)timer_instance >= (sizeof(timers) / sizeof(timers[0]))) return;
    timers[timer_instance].period_us = period_us;
    /* Skeleton: program CC0 and start the timer minimally so examples can
     * build and link. Full implementation deferred. */
    uint32_t base = timers[timer_instance].base;
    REG(base + CC0_OFFSET) = (period_us == 0) ? 1u : period_us;
    REG(base + EVENT_COMPARE0_OFFSET) = 0u;
    REG(base + TASK_START_OFFSET) = 1u;
}

/* hal_timer_stop already matched the public API; keep behaviour but keep it
 * minimal to avoid surprising side-effects in examples. */
void hal_timer_stop(int timer_instance) {
    if ((unsigned)timer_instance >= (sizeof(timers) / sizeof(timers[0]))) return;
    uint32_t base = timers[timer_instance].base;
    REG(base + TASK_STOP_OFFSET) = 1u;
    /* clear internal state */
    timers[timer_instance].period_us = 0;
}

/* IRQ handler for Timer0. Startup file aliases the other handlers to
 * the default handler when not implemented. Keep this handler small
 * to avoid complex logic inside interrupts. */
void Timer0_IRQHandler(void) {
    uint32_t base = timers[0].base;
    if (REG(base + EVENT_COMPARE0_OFFSET)) {
        REG(base + EVENT_COMPARE0_OFFSET) = 0u;
        if (timers[0].one_shot) {
            REG(base + TASK_STOP_OFFSET) = 1u;
        }

        /* If the timer is periodic, clear the counter so the compare
         * will trigger again after the configured period. This emulates
         * the COMPARE[n] -> CLEAR short present in Nordic timers. */
        if (!timers[0].one_shot) {
            REG(base + TASK_CLEAR_OFFSET) = 1u;
        }

        if (timers[0].event) {
            hal_event_set_mask(timers[0].event);
        }
    }
}

