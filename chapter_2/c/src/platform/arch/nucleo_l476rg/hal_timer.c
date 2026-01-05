// Interrupt-driven STM32L476 timer HAL using TIM2 peripheral; no vendor
// libs/CMSIS.
#include <assert.h>
#include <hal_timer.h>
#include <stdint.h>
#include <stdlib.h>

#define RCC_BASE     0x40021000UL
#define RCC_APB1ENR1 (*(volatile uint32_t *)(RCC_BASE + 0x58))
#define RCC_APB1RSTR (*(volatile uint32_t *)(RCC_BASE + 0x38))

#define TIM2_BASE 0x40000000UL
#define TIM2_CR1  (*(volatile uint32_t *)(TIM2_BASE + 0x00))
#define TIM2_DIER (*(volatile uint32_t *)(TIM2_BASE + 0x0C))
#define TIM2_SR   (*(volatile uint32_t *)(TIM2_BASE + 0x10))
#define TIM2_EGR  (*(volatile uint32_t *)(TIM2_BASE + 0x14))
#define TIM2_CNT  (*(volatile uint32_t *)(TIM2_BASE + 0x24))
#define TIM2_PSC  (*(volatile uint32_t *)(TIM2_BASE + 0x28))
#define TIM2_ARR  (*(volatile uint32_t *)(TIM2_BASE + 0x2C))

#define TIM3_BASE 0x40000400UL
#define TIM3_CR1  (*(volatile uint32_t *)(TIM3_BASE + 0x00))
#define TIM3_DIER (*(volatile uint32_t *)(TIM3_BASE + 0x0C))
#define TIM3_SR   (*(volatile uint32_t *)(TIM3_BASE + 0x10))
#define TIM3_EGR  (*(volatile uint32_t *)(TIM3_BASE + 0x14))
#define TIM3_CNT  (*(volatile uint32_t *)(TIM3_BASE + 0x24))
#define TIM3_PSC  (*(volatile uint32_t *)(TIM3_BASE + 0x28))
#define TIM3_ARR  (*(volatile uint32_t *)(TIM3_BASE + 0x2C))

#define TIM4_BASE 0x40000800UL
#define TIM4_CR1  (*(volatile uint32_t *)(TIM4_BASE + 0x00))
#define TIM4_DIER (*(volatile uint32_t *)(TIM4_BASE + 0x0C))
#define TIM4_SR   (*(volatile uint32_t *)(TIM4_BASE + 0x10))
#define TIM4_EGR  (*(volatile uint32_t *)(TIM4_BASE + 0x14))
#define TIM4_CNT  (*(volatile uint32_t *)(TIM4_BASE + 0x24))
#define TIM4_PSC  (*(volatile uint32_t *)(TIM4_BASE + 0x28))
#define TIM4_ARR  (*(volatile uint32_t *)(TIM4_BASE + 0x2C))

#define TIM_CR1_CEN  (1UL << 0)
#define TIM_DIER_UIE (1UL << 0)
#define TIM_SR_UIF   (1UL << 0)
#define TIM_EGR_UG   (1UL << 0)

#define NVIC_ISER0   (*(volatile uint32_t *)0xE000E100UL)
#define TIM2_IRQ_BIT (1UL << 28) // TIM2 IRQn = 28
#define TIM3_IRQ_BIT (1UL << 29) // TIM3 IRQn = 29
#define TIM4_IRQ_BIT (1UL << 30) // TIM4 IRQn = 30

/* Assume default 4 MHz MSI unless startup changes it. Adjust if needed. */
#define TIMER_CLOCK_HZ 4000000UL

struct priv_timer {
    volatile uint32_t waiting;
    bool one_shoot;
    uint32_t period_us;
    hal_event_mask_t event;
};

static struct priv_timer s_timer_instances[3] = {
    { .waiting = 0, .one_shoot = false, .period_us = 1000U, .event = 0 },
    { .waiting = 0, .one_shoot = false, .period_us = 1000U, .event = 0 },
    { .waiting = 0, .one_shoot = false, .period_us = 1000U, .event = 0 },
};

static inline void __WFI(void) { __asm volatile("wfi" ::: "memory"); }

static void tim_config_period_us(int timer_instance, uint32_t period_us) {
    const uint32_t prescaler = (TIMER_CLOCK_HZ / 1000000UL) - 1UL;
    volatile uint32_t *CR1, *DIER, *SR, *PSC, *ARR, *CNT, *EGR;
    /* Map public instances 2..4 to internal indices 0..2 */
    int idx = timer_instance - 2;
    if (idx == 0) {
        CR1 = &TIM2_CR1;
        DIER = &TIM2_DIER;
        SR = &TIM2_SR;
        PSC = &TIM2_PSC;
        ARR = &TIM2_ARR;
        CNT = &TIM2_CNT;
        EGR = &TIM2_EGR;
    } else if (idx == 1) {
        CR1 = &TIM3_CR1;
        DIER = &TIM3_DIER;
        SR = &TIM3_SR;
        PSC = &TIM3_PSC;
        ARR = &TIM3_ARR;
        CNT = &TIM3_CNT;
        EGR = &TIM3_EGR;
    } else {
        CR1 = &TIM4_CR1;
        DIER = &TIM4_DIER;
        SR = &TIM4_SR;
        PSC = &TIM4_PSC;
        ARR = &TIM4_ARR;
        CNT = &TIM4_CNT;
        EGR = &TIM4_EGR;
    }

    /* Stop timer and disable interrupts while configuring */
    *CR1 = 0;
    *DIER = 0;
    *SR = 0; /* clear pending flags */
    *PSC = prescaler; /* prescale to 1 MHz */
    *ARR = (period_us == 0) ? 1UL : period_us; /* avoid zero ARR */
    *CNT = 0;
    *EGR = TIM_EGR_UG; /* load prescaler immediately (may set UIF) */
    *SR &= ~TIM_SR_UIF; /* clear the UIF set by UG */
}

void hal_timer_init(int timer_instance, bool one_shoot, hal_event_mask_t event) {
    /* Public instances are 2..4 mapping to TIM2..TIM4 */
    assert(timer_instance >= 2 && timer_instance <= 4);
    int idx = timer_instance - 2;

    s_timer_instances[idx].one_shoot = one_shoot;
    s_timer_instances[idx].waiting = 0;
    /* leave period until start, but configure to default now */
    s_timer_instances[idx].period_us = s_timer_instances[idx].period_us;
    s_timer_instances[idx].event = event;

    /* Enable TIMx clock on APB1. TIM2EN bit = 0, TIM3EN = 1, TIM4EN = 2 */
    RCC_APB1ENR1 |= (1UL << (uint32_t)idx);
    (void)RCC_APB1ENR1; /* readback as barrier */

    /* Reset TIMx to a known state, then release reset (APB1RSTR). */
    RCC_APB1RSTR |= (1UL << (uint32_t)idx);
    RCC_APB1RSTR &= ~(1UL << (uint32_t)idx);

    /* Configure timer with current default period (not started) */
    tim_config_period_us(timer_instance, (s_timer_instances[idx].period_us == 0)
                                             ? 1U
                                             : s_timer_instances[idx].period_us);

    /* Enable timer IRQ in NVIC for the corresponding timer. */
    if (timer_instance == 2)
        NVIC_ISER0 |= TIM2_IRQ_BIT;
    else if (timer_instance == 3)
        NVIC_ISER0 |= TIM3_IRQ_BIT;
    else
        NVIC_ISER0 |= TIM4_IRQ_BIT;
}

void hal_timer_start(int timer_instance, uint32_t period_us) {
    assert(timer_instance >= 2 && timer_instance <= 4);
    int idx = timer_instance - 2;

    s_timer_instances[idx].period_us = period_us;
    tim_config_period_us(timer_instance, (s_timer_instances[idx].period_us == 0)
                                             ? 1U
                                             : s_timer_instances[idx].period_us);

    s_timer_instances[idx].waiting = 1;

    if (timer_instance == 2) {
        TIM2_DIER = TIM_DIER_UIE;
        TIM2_CR1 = TIM_CR1_CEN;
    } else if (timer_instance == 3) {
        TIM3_DIER = TIM_DIER_UIE;
        TIM3_CR1 = TIM_CR1_CEN;
    } else {
        TIM4_DIER = TIM_DIER_UIE;
        TIM4_CR1 = TIM_CR1_CEN;
    }
}

void hal_timer_stop(int timer_instance) {
    assert(timer_instance >= 2 && timer_instance <= 4);
    int idx = timer_instance - 2;
    if (timer_instance == 2) {
        TIM2_CR1 &= ~TIM_CR1_CEN;
        TIM2_DIER = 0;
    } else if (timer_instance == 3) {
        TIM3_CR1 &= ~TIM_CR1_CEN;
        TIM3_DIER = 0;
    } else {
        TIM4_CR1 &= ~TIM_CR1_CEN;
        TIM4_DIER = 0;
    }
    s_timer_instances[idx].waiting = 0;
}

/* TIM2 ISR: clear update flag, stop timer in one-shot mode, and release wait.
 */
void TIM2_IRQHandler(void) {
    if (TIM2_SR & TIM_SR_UIF) {
        TIM2_SR &= ~TIM_SR_UIF; // clear UIF
        if (s_timer_instances[0].one_shoot) {
            TIM2_CR1 &= ~TIM_CR1_CEN;
        }
        s_timer_instances[0].waiting = 0;
        if (s_timer_instances[0].event) {
            hal_event_set_mask(s_timer_instances[0].event);
        }
    }
}

void TIM3_IRQHandler(void) {
    if (TIM3_SR & TIM_SR_UIF) {
        TIM3_SR &= ~TIM_SR_UIF;
        if (s_timer_instances[1].one_shoot) {
            TIM3_CR1 &= ~TIM_CR1_CEN;
        }
        s_timer_instances[1].waiting = 0;
        if (s_timer_instances[1].event) {
            hal_event_set_mask(s_timer_instances[1].event);
        }
    }
}

void TIM4_IRQHandler(void) {
    if (TIM4_SR & TIM_SR_UIF) {
        TIM4_SR &= ~TIM_SR_UIF;
        if (s_timer_instances[2].one_shoot) {
            TIM4_CR1 &= ~TIM_CR1_CEN;
        }
        s_timer_instances[2].waiting = 0;
        if (s_timer_instances[2].event) {
            hal_event_set_mask(s_timer_instances[2].event);
        }
    }
}
