// Interrupt-driven STM32L476 timer HAL using TIM2 peripheral; no vendor
// libs/CMSIS.
#include <assert.h>
#include <hal_timer.h>
#include <stdint.h>

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

static inline void __WFI(void) { __asm volatile("wfi" ::: "memory"); }

static void tim2_config_period_ms(uint32_t period_ms) {
    /* Use prescaler to get 1 kHz tick: PSC = (f_clk/1kHz) - 1. */
    const uint32_t prescaler = (TIMER_CLOCK_HZ / 1000UL) - 1UL;
    TIM2_CR1 = 0;         // stop timer
    TIM2_DIER = 0;        // disable interrupts while configuring
    TIM2_SR = 0;          // clear pending flags
    TIM2_PSC = prescaler; // prescale to 1 kHz
    TIM2_ARR = (period_ms == 0) ? 1UL : period_ms; // avoid zero ARR
    TIM2_CNT = 0;
    TIM2_EGR = TIM_EGR_UG; // load prescaler immediately
}

void hal_timer_init(int timer_instance, bool one_shoot) {
    assert(timer_instance == 0);

    s_one_shot = one_shoot;
    s_waiting = 0;

    /* Enable TIM2 clock on APB1. */
    RCC_APB1ENR1 |= (1UL << 0); // TIM2EN bit
    (void)RCC_APB1ENR1;         // simple barrier

    tim2_config_period_ms(1);

    /* Enable TIM2 interrupt in NVIC without clobbering other enables. */
    NVIC_ISER0 |= TIM2_IRQ_BIT;
}

void hal_timer_delay_ms(int timer_instance, uint32_t period_ms) {
    assert(timer_instance == 0);

    tim2_config_period_ms(period_ms);

    s_waiting = 1;

    /* Enable update interrupt and start timer. */
    TIM2_DIER = TIM_DIER_UIE;
    TIM2_CR1 = TIM_CR1_CEN;

    while (s_waiting) {
        __WFI();
    }
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
    }
}
