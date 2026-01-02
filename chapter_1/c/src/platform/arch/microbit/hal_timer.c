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
#define TIMER_PRESCALER_128     (7u)
#define TIMER_INTENSET_COMPARE0 (1u << 16)

#define NVIC_ISER0 (0xE000E100u)

#define REG(addr) (*(volatile uint32_t *)(addr))

static inline void __wfi(void) { __asm volatile("wfi" : : : "memory"); }

static inline void NVIC_EnableIRQ(uint32_t IRQn) {
    if ((int32_t)IRQn >= 0) {
        REG(NVIC_ISER0) |= (1u << (IRQn & 0x1Fu));
    }
}

struct timer_desc {
    uint32_t base;
    bool one_shot;
    volatile uint32_t fired;
};

static struct timer_desc timers[] = {
    {TIMER0_BASE, false, 0},
};

void hal_timer_init(int timer_instance, bool one_shot) {
    assert((unsigned)timer_instance < (sizeof(timers) / sizeof(timers[0])));

    timers[timer_instance].one_shot = one_shot;

    /* Clear any pending event, enable CC[0] interrupt and NVIC */
    REG(timers[timer_instance].base + EVENT_COMPARE0_OFFSET) = 0u;
    REG(timers[timer_instance].base + INTENSET_OFFSET) |= TIMER_INTENSET_COMPARE0;
    NVIC_EnableIRQ(TIMER0_IRQN + (uint32_t)timer_instance);
}

void hal_timer_delay_ms(int timer_instance, uint32_t period_ms) {
    assert((unsigned)timer_instance < (sizeof(timers) / sizeof(timers[0])));

    uint32_t base = timers[timer_instance].base;

    /* Configure peripheral */
    REG(base + MODE_OFFSET) = TIMER_MODE_TIMER;
    REG(base + TASK_CLEAR_OFFSET) = 1u;
    REG(base + PRESCALER_OFFSET) = TIMER_PRESCALER_128;
    REG(base + BITMODE_OFFSET) = TIMER_BITMODE_32BIT;

    /* Approximate period — device clocks differ and QEMU is not
     * timing-accurate. Keep calculation here so caller uses ms.
     */
    uint32_t ticks = period_ms * 125u;
    REG(base + CC0_OFFSET) = ticks;

    /* Start and wait for compare event. We use a simple spin with
     * WFI — this is sufficient for the examples in the book. */
    /* Ensure previous events are cleared, arm the fired flag, then start */
    REG(base + EVENT_COMPARE0_OFFSET) = 0u;
    timers[timer_instance].fired = 0u;
    REG(base + TASK_START_OFFSET) = 1u;

    /* Wait for IRQ to set the fired flag */
    while (!timers[timer_instance].fired) {
        __wfi();
    }

    if (timers[timer_instance].one_shot) {
        REG(base + TASK_STOP_OFFSET) = 1u;
    }
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
        timers[0].fired = 1u;
    }
}

