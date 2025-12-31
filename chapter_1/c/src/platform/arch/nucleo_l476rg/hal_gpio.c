// Minimal STM32L476 GPIO HAL for Nucleo boards, no vendor libs or CMSIS.
#include <hal_gpio.h>
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/* Register layout for one GPIO port (RM0351, STM32L4x6). */
struct gpio_regs {
	volatile uint32_t MODER;   // 0x00: mode register
	volatile uint32_t OTYPER;  // 0x04: output type
	volatile uint32_t OSPEEDR; // 0x08: output speed (unused here)
	volatile uint32_t PUPDR;   // 0x0C: pull-up/down (unused here)
	volatile uint32_t IDR;     // 0x10: input data
	volatile uint32_t ODR;     // 0x14: output data
	volatile uint32_t BSRR;    // 0x18: bit set/reset
	volatile uint32_t LCKR;    // 0x1C: lock (unused here)
	volatile uint32_t AFRL;    // 0x20: alternate function low (unused)
	volatile uint32_t AFRH;    // 0x24: alternate function high (unused)
	volatile uint32_t BRR;     // 0x28: bit reset register
};

#define RCC_BASE        0x40021000UL
#define RCC_AHB2ENR     (*(volatile uint32_t *)(RCC_BASE + 0x4CUL))

#define GPIOA_BASE 0x48000000UL
#define GPIOB_BASE 0x48000400UL
#define GPIOC_BASE 0x48000800UL
#define GPIOD_BASE 0x48000C00UL
#define GPIOE_BASE 0x48001000UL
#define GPIOF_BASE 0x48001400UL
#define GPIOG_BASE 0x48001800UL
#define GPIOH_BASE 0x48001C00UL

static const uintptr_t gpio_bases[] = {
	GPIOA_BASE, GPIOB_BASE, GPIOC_BASE, GPIOD_BASE,
	GPIOE_BASE, GPIOF_BASE, GPIOG_BASE, GPIOH_BASE
};

static inline struct gpio_regs *gpio_from_pin(int pin, uint32_t *mask, unsigned *port_idx) {
	if (pin < 0) {
		return NULL;
	}
	unsigned port = (unsigned)pin / 16U;
	unsigned bit  = (unsigned)pin % 16U;
	if (port >= (sizeof(gpio_bases) / sizeof(gpio_bases[0]))) {
		return NULL;
	}
	*mask = 1UL << bit;
	*port_idx = port;
	return (struct gpio_regs *)gpio_bases[port];
}

static inline void enable_gpio_clock(unsigned port_idx) {
	RCC_AHB2ENR |= (1UL << port_idx);
	(void)RCC_AHB2ENR; // simple barrier to ensure the write posts
}

void hal_gpio_init_out(int pin, bool od, int value) {
	uint32_t mask = 0;
	unsigned port_idx = 0;
	struct gpio_regs *gpio = gpio_from_pin(pin, &mask, &port_idx);
	assert(gpio != NULL);

	enable_gpio_clock(port_idx);

	// Set MODER to output (01b) for this pin without disturbing others.
	unsigned shift = ((unsigned)pin % 16U) * 2U;
	uint32_t moder = gpio->MODER;
	moder &= ~(0x3UL << shift);
	moder |=  (0x1UL << shift);
	gpio->MODER = moder;

	// Configure output type: push-pull (0) or open-drain (1).
	uint32_t otyper = gpio->OTYPER;
	if (od) {
		otyper |= mask;
	} else {
		otyper &= ~mask;
	}
	gpio->OTYPER = otyper;

	// Initialize output level atomically via BSRR.
	if (value) {
		gpio->BSRR = mask;           // set pin high
	} else {
		gpio->BSRR = (mask << 16);   // set pin low
	}
}

void hal_gpio_write(int pin, int value) {
	uint32_t mask = 0;
	unsigned port_idx = 0;
	struct gpio_regs *gpio = gpio_from_pin(pin, &mask, &port_idx);
	assert(gpio != NULL);
	(void)port_idx; // unused here, but kept for symmetry

	if (value) {
		gpio->BSRR = mask;           // set pin high
	} else {
		gpio->BSRR = (mask << 16);   // reset pin low
	}
}
