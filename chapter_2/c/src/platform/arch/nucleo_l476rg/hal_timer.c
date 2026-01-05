// Interrupt-driven STM32L476 timer HAL using TIM2 peripheral; no vendor
// libs/CMSIS.
#include <assert.h>
#include <hal_timer.h>
#include <stdint.h>
#include <stdlib.h>

#define RCC_BASE     0x40021000UL
#define RCC_APB1ENR1 (*(volatile uint32_t *)(RCC_BASE + 0x58))

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

/* Support 3 timer instances: 0=TIM2,1=TIM3,2=TIM4 */
static volatile uint32_t s_waiting[3] = {0, 0, 0};
static bool s_one_shot[3] = {false, false, false};
static uint32_t s_period_us_arr[3] = {1000U, 1000U, 1000U};
static hal_timer_cb s_cb_arr[3] = {NULL, NULL, NULL};
static void *s_cb_arg_arr[3] = {NULL, NULL, NULL};

static inline void __WFI(void) { __asm volatile("wfi" ::: "memory"); }

static void tim_config_period_us(int timer_instance, uint32_t period_us) {
    const uint32_t prescaler = (TIMER_CLOCK_HZ / 1000000UL) - 1UL;
    volatile uint32_t *CR1, *DIER, *SR, *PSC, *ARR, *CNT, *EGR;
    if (timer_instance == 0) {
        CR1 = &TIM2_CR1;
        DIER = &TIM2_DIER;
        SR = &TIM2_SR;
        PSC = &TIM2_PSC;
        ARR = &TIM2_ARR;
        CNT = &TIM2_CNT;
        EGR = &TIM2_EGR;
    } else if (timer_instance == 1) {
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

    *CR1 = 0;         /* stop timer */
    *DIER = 0;        /* disable interrupts while configuring */
    *SR = 0;          /* clear pending flags */
    *PSC = prescaler; /* prescale to 1 MHz */
    *ARR = (period_us == 0) ? 1UL : period_us; /* avoid zero ARR */
    *CNT = 0;
    *EGR = TIM_EGR_UG; /* load prescaler immediately */
}

void hal_timer_init(int timer_instance, uint32_t period_us, bool one_shoot) {
    assert(timer_instance >= 0 && timer_instance <= 2);

    s_one_shot[timer_instance] = one_shoot;
    s_waiting[timer_instance] = 0;
    s_period_us_arr[timer_instance] = period_us;
    s_cb_arr[timer_instance] = NULL;
    s_cb_arg_arr[timer_instance] = NULL;

    /* Enable TIMx clock on APB1. TIM2EN bit = 0, TIM3EN = 1, TIM4EN = 2 */
    RCC_APB1ENR1 |= (1UL << (uint32_t)timer_instance);
    (void)RCC_APB1ENR1;

    tim_config_period_us(timer_instance, (s_period_us_arr[timer_instance] == 0)
                                             ? 1U
                                             : s_period_us_arr[timer_instance]);

    /* Enable timer IRQ in NVIC for the corresponding timer. */
    if (timer_instance == 0)
        NVIC_ISER0 |= TIM2_IRQ_BIT;
    else if (timer_instance == 1)
        NVIC_ISER0 |= TIM3_IRQ_BIT;
    else
        NVIC_ISER0 |= TIM4_IRQ_BIT;
}

void hal_timer_set_period(int timer_instance, uint32_t period_us) {
    assert(timer_instance >= 0 && timer_instance <= 2);
    s_period_us_arr[timer_instance] = period_us;
    tim_config_period_us(timer_instance, (s_period_us_arr[timer_instance] == 0)
                                             ? 1U
                                             : s_period_us_arr[timer_instance]);
}

void hal_timer_start(int timer_instance, hal_timer_cb cb, void *arg) {
    assert(timer_instance >= 0 && timer_instance <= 2);
    s_cb_arr[timer_instance] = cb;
    s_cb_arg_arr[timer_instance] = arg;

    s_waiting[timer_instance] = 1;

    if (timer_instance == 0) {
        TIM2_DIER = TIM_DIER_UIE;
        TIM2_CR1 = TIM_CR1_CEN;
    } else if (timer_instance == 1) {
        TIM3_DIER = TIM_DIER_UIE;
        TIM3_CR1 = TIM_CR1_CEN;
    } else {
        TIM4_DIER = TIM_DIER_UIE;
        TIM4_CR1 = TIM_CR1_CEN;
    }
}

void hal_timer_stop(int timer_instance) {
    assert(timer_instance >= 0 && timer_instance <= 2);
    if (timer_instance == 0) {
        TIM2_CR1 &= ~TIM_CR1_CEN;
        TIM2_DIER = 0;
    } else if (timer_instance == 1) {
        TIM3_CR1 &= ~TIM_CR1_CEN;
        TIM3_DIER = 0;
    } else {
        TIM4_CR1 &= ~TIM_CR1_CEN;
        TIM4_DIER = 0;
    }
    s_cb_arr[timer_instance] = NULL;
    s_cb_arg_arr[timer_instance] = NULL;
}

/* TIM2 ISR: clear update flag, stop timer in one-shot mode, and release wait.
 */
void TIM2_IRQHandler(void) {
    if (TIM2_SR & TIM_SR_UIF) {
        TIM2_SR &= ~TIM_SR_UIF; // clear UIF
        if (s_one_shot[0]) {
            TIM2_CR1 &= ~TIM_CR1_CEN;
        }
        s_waiting[0] = 0;
        if (s_cb_arr[0]) {
            s_cb_arr[0](s_cb_arg_arr[0]);
        }
    }
}

void TIM3_IRQHandler(void) {
    if (TIM3_SR & TIM_SR_UIF) {
        TIM3_SR &= ~TIM_SR_UIF;
        if (s_one_shot[1]) {
            TIM3_CR1 &= ~TIM_CR1_CEN;
        }
        s_waiting[1] = 0;
        if (s_cb_arr[1]) {
            s_cb_arr[1](s_cb_arg_arr[1]);
        }
    }
}

void TIM4_IRQHandler(void) {
    if (TIM4_SR & TIM_SR_UIF) {
        TIM4_SR &= ~TIM_SR_UIF;
        if (s_one_shot[2]) {
            TIM4_CR1 &= ~TIM_CR1_CEN;
        }
        s_waiting[2] = 0;
        if (s_cb_arr[2]) {
            s_cb_arr[2](s_cb_arg_arr[2]);
        }
    }
}
