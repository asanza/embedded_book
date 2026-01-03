// Minimal STM32L476 GPIO HAL for Nucleo boards, no vendor libs or CMSIS.
#include <assert.h>
#include <hal_gpio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/* GPIO register offsets for one port (RM0351, STM32L4x6). */
#define GPIO_MODER_OFFSET   0x00u
#define GPIO_OTYPER_OFFSET  0x04u
#define GPIO_OSPEEDR_OFFSET 0x08u
#define GPIO_PUPDR_OFFSET   0x0Cu
#define GPIO_IDR_OFFSET     0x10u
#define GPIO_ODR_OFFSET     0x14u
#define GPIO_BSRR_OFFSET    0x18u
#define GPIO_LCKR_OFFSET    0x1Cu
#define GPIO_AFRL_OFFSET    0x20u
#define GPIO_AFRH_OFFSET    0x24u
#define GPIO_BRR_OFFSET     0x28u

#define RCC_BASE 0x40021000UL

/* Generic register access macro. */
#define REG(addr) (*(volatile uint32_t *)(addr))

#define RCC_AHB2ENR REG(RCC_BASE + 0x4CUL)

#define GPIOA_BASE 0x48000000UL
#define GPIOB_BASE 0x48000400UL
#define GPIOC_BASE 0x48000800UL
#define GPIOD_BASE 0x48000C00UL
#define GPIOE_BASE 0x48001000UL
#define GPIOF_BASE 0x48001400UL
#define GPIOG_BASE 0x48001800UL
#define GPIOH_BASE 0x48001C00UL

static const uintptr_t gpio_bases[] = {GPIOA_BASE, GPIOB_BASE, GPIOC_BASE,
                                       GPIOD_BASE, GPIOE_BASE, GPIOF_BASE,
                                       GPIOG_BASE, GPIOH_BASE};

static inline uintptr_t gpio_from_pin(int pin, uint32_t *mask,
                                      unsigned *port_idx) {
    if (pin < 0) {
        return 0;
    }
    unsigned port = (unsigned)pin / 16U;
    unsigned bit = (unsigned)pin % 16U;
    if (port >= (sizeof(gpio_bases) / sizeof(gpio_bases[0]))) {
        return 0;
    }
    *mask = 1UL << bit;
    *port_idx = port;
    return gpio_bases[port];
}

static inline void enable_gpio_clock(unsigned port_idx) {
    RCC_AHB2ENR |= (1UL << port_idx);
    (void)RCC_AHB2ENR; // simple barrier to ensure the write posts
}

void hal_gpio_init_out(int pin, bool od, int value) {
    uint32_t mask = 0;
    unsigned port_idx = 0;
    uintptr_t base = gpio_from_pin(pin, &mask, &port_idx);
    assert(base != 0);

    enable_gpio_clock(port_idx);

    /* Set MODER to output (01b) for this pin without disturbing others. */
    unsigned shift = ((unsigned)pin % 16U) * 2U;
    uint32_t m = REG(base + GPIO_MODER_OFFSET);
    m &= ~(0x3UL << shift);
    m |= (0x1UL << shift);
    REG(base + GPIO_MODER_OFFSET) = m;

    /* Configure output type: push-pull (0) or open-drain (1). */
    uint32_t o = REG(base + GPIO_OTYPER_OFFSET);
    if (od) {
        o |= mask;
    } else {
        o &= ~mask;
    }
    REG(base + GPIO_OTYPER_OFFSET) = o;

    /* Initialize output level atomically via BSRR. */
    if (value) {
        REG(base + GPIO_BSRR_OFFSET) = mask; /* set pin high */
    } else {
        REG(base + GPIO_BSRR_OFFSET) = (mask << 16); /* set pin low */
    }
}

void hal_gpio_write(int pin, int value) {
    uint32_t mask = 0;
    unsigned port_idx = 0;
    uintptr_t base = gpio_from_pin(pin, &mask, &port_idx);
    assert(base != 0);
    (void)port_idx; /* unused here, but kept for symmetry */

    if (value) {
        REG(base + GPIO_BSRR_OFFSET) = mask; /* set pin high */
    } else {
        REG(base + GPIO_BSRR_OFFSET) = (mask << 16); /* reset pin low */
    }
}

int hal_gpio_init_in(int pin, enum hal_gpio_pull pull) {
    uint32_t mask = 0;
    unsigned port_idx = 0;
    uintptr_t base = gpio_from_pin(pin, &mask, &port_idx);
    if (base == 0) {
        return -1;
    }

    enable_gpio_clock(port_idx);

    /* Set MODER to input (00b) for this pin without disturbing others. */
    unsigned shift = ((unsigned)pin % 16U) * 2U;
    uint32_t m = REG(base + GPIO_MODER_OFFSET);
    m &= ~(0x3UL << shift);
    REG(base + GPIO_MODER_OFFSET) = m;

    /* Configure pull-up/pull-down in PUPDR (2 bits per pin). */
    unsigned pshift = ((unsigned)pin % 16U) * 2U;
    uint32_t p = REG(base + GPIO_PUPDR_OFFSET);
    p &= ~(0x3UL << pshift);
    if (pull == HAL_GPIO_PULLUP) {
        p |= (0x1UL << pshift); /* 01 = pull-up */
    } else if (pull == HAL_GPIO_PULLDOWN) {
        p |= (0x2UL << pshift); /* 10 = pull-down */
    }
    REG(base + GPIO_PUPDR_OFFSET) = p;

    return 0;
}

int hal_gpio_read(int pin) {
    uint32_t mask = 0;
    unsigned port_idx = 0;
    uintptr_t base = gpio_from_pin(pin, &mask, &port_idx);
    if (base == 0) {
        return -1;
    }
    return (REG(base + GPIO_IDR_OFFSET) & mask) ? 1 : 0;
}
