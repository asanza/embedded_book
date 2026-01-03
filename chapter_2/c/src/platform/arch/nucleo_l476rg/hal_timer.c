// Interrupt-driven STM32L476 timer HAL using TIM2 peripheral; no vendor
// libs/CMSIS.
#include <assert.h>
#include <hal_timer.h>
#include <stdint.h>
#include <stdlib.h>

#define RCC_BASE 0x40021000UL
#define RCC_APB1ENR1 (*(volatile uint32_t *)(RCC_BASE + 0x58))

#define TIM2_BASE 0x40000000UL
#define TIM2_CR1 (*(volatile uint32_t *)(TIM2_BASE + 0x00))
#define TIM2_DIER (*(volatile uint32_t *)(TIM2_BASE + 0x0C))
#define TIM2_SR (*(volatile uint32_t *)(TIM2_BASE + 0x10))
#define TIM2_EGR (*(volatile uint32_t *)(TIM2_BASE + 0x14))
#define TIM2_CNT (*(volatile uint32_t *)(TIM2_BASE + 0x24))
#define TIM2_PSC (*(volatile uint32_t *)(TIM2_BASE + 0x28))
#define TIM2_ARR (*(volatile uint32_t *)(TIM2_BASE + 0x2C))

#define TIM_CR1_CEN (1UL << 0)
#define TIM_DIER_UIE (1UL << 0)
#define TIM_SR_UIF (1UL << 0)
#define TIM_EGR_UG (1UL << 0)

#define NVIC_ISER0 (*(volatile uint32_t *)0xE000E100UL)
#define TIM2_IRQ_BIT (1UL << 28) // TIM2 IRQn = 28

/* Assume default 4 MHz MSI unless startup changes it. Adjust if needed. */
#define TIMER_CLOCK_HZ 4000000UL

static volatile uint32_t s_waiting = 0;
static bool s_one_shot = false;
static uint32_t s_period_us = 1000U;
static hal_timer_cb s_cb = NULL;
static void* s_cb_arg = NULL;

static inline void __WFI(void) { __asm volatile("wfi" ::: "memory"); }

static void tim2_config_period_us(uint32_t period_us) {
    /* Use prescaler to get 1 MHz tick: PSC = (f_clk/1MHz) - 1. */
    const uint32_t prescaler = (TIMER_CLOCK_HZ / 1000000UL) - 1UL;
    TIM2_CR1 = 0;         // stop timer
    TIM2_DIER = 0;        // disable interrupts while configuring
    TIM2_SR = 0;          // clear pending flags
    TIM2_PSC = prescaler; // prescale to 1 MHz
    TIM2_ARR = (period_us == 0) ? 1UL : period_us; // avoid zero ARR
    TIM2_CNT = 0;
    TIM2_EGR = TIM_EGR_UG; // load prescaler immediately
}

void hal_timer_init(int timer_instance, uint32_t period_us, bool one_shoot) {
    assert(timer_instance == 0);

    s_one_shot = one_shoot;
    s_waiting = 0;
    s_period_us = period_us;
    s_cb = NULL;
    s_cb_arg = NULL;

    /* Enable TIM2 clock on APB1. */
    RCC_APB1ENR1 |= (1UL << 0); // TIM2EN bit
    (void)RCC_APB1ENR1;         // simple barrier

    tim2_config_period_us((s_period_us == 0) ? 1U : s_period_us);

    /* Enable TIM2 interrupt in NVIC without clobbering other enables. */
    NVIC_ISER0 |= TIM2_IRQ_BIT;
}

void hal_timer_set_period(int timer_instance, uint32_t period_us) {
    assert(timer_instance == 0);
    s_period_us = period_us;
    tim2_config_period_us((s_period_us == 0) ? 1U : s_period_us);
}

void hal_timer_start(int timer_instance, hal_timer_cb cb, void* arg) {
    assert(timer_instance == 0);
    s_cb = cb;
    s_cb_arg = arg;

    s_waiting = 1;

    /* Enable update interrupt and start timer. */
    TIM2_DIER = TIM_DIER_UIE;
    TIM2_CR1 = TIM_CR1_CEN;

    /* If caller wants to block until timer fires, they can wait on s_waiting
     * externally; for compatibility this function does not block.
     */
}

void hal_timer_stop(int timer_instance) {
    assert(timer_instance == 0);
    TIM2_CR1 &= ~TIM_CR1_CEN;
    TIM2_DIER = 0;
    s_cb = NULL;
    s_cb_arg = NULL;
}

/* TIM2 ISR: clear update flag, stop timer in one-shot mode, and release wait.
 */
void TIM2_IRQHandler(void) {
    if (TIM2_SR & TIM_SR_UIF) {
        TIM2_SR &= ~TIM_SR_UIF; // clear UIF
        if (s_one_shot) {
            TIM2_CR1 &= ~TIM_CR1_CEN;
        }
        s_waiting = 0;
        if (s_cb) {
            s_cb(s_cb_arg);
        }
    }
}
