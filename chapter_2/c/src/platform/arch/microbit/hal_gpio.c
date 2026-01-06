/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026 Diego Asanza <f.asanza@gmail.com> */

#include <hal_gpio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/**
 * @brief General purpose input and output. (GPIO)
 */

#define GPIO_BASE   (0x50000000)
#define GPIO_OUTSET (GPIO_BASE + 0x508)
#define GPIO_OUTCLR (GPIO_BASE + 0x50C)
#define GPIO_DIRSET (GPIO_BASE + 0x518)
#define GPIO_DIRCLR (GPIO_BASE + 0x51C)
#define GPIO_IN     (GPIO_BASE + 0x510)

/* PIN_CNF base (nRF-style): one 32-bit config word per pin starting here. */
#define GPIO_PIN_CNF_BASE (GPIO_BASE + 0x700)

#define REG(addr) (*(volatile uint32_t *)(addr))

void hal_gpio_init_out(int pin, bool od, int value) {
    (void)od;
    REG(GPIO_DIRSET) = 1U << pin;
    if (value) {
        REG(GPIO_OUTSET) = 1u << pin;
    } else {
        REG(GPIO_OUTCLR) = 1u << pin;
    }
}

void hal_gpio_write(int pin, int value) {
    if (value) {
        REG(GPIO_OUTSET) = 1u << pin;
    } else {
        REG(GPIO_OUTCLR) = 1u << pin;
    }
    printf("GPIO pin %d = %d\n", pin, value);
}

int hal_gpio_init_in(int pin, enum hal_gpio_pull pull) {
    if (pin < 0 || pin >= 32) {
        return -1;
    }
    /* Clear direction bit to configure as input */
    REG(GPIO_DIRCLR) = 1u << pin;

    /* Configure pull via PIN_CNF (nRF-style). PULL field bits are at bit 2..3.
     * Values: 0 = disabled, 1 = pull-down, 3 = pull-up (common mapping).
     */
    uintptr_t cfg = GPIO_PIN_CNF_BASE + (unsigned)pin * 4U;
    uint32_t v = REG(cfg) & ~(0x3u << 2);
    if (pull == HAL_GPIO_PULLDOWN) {
        v |= (0x1u << 2);
    } else if (pull == HAL_GPIO_PULLUP) {
        v |= (0x3u << 2);
    }
    REG(cfg) = v;

    return 0;
}

int hal_gpio_read(int pin) {
    if (pin < 0 || pin >= 32) {
        return -1;
    }
    return (REG(GPIO_IN) & (1u << pin)) ? 1 : 0;
}

/* Minimal interrupt support for microbit platform. The real implementation
 * would configure GPIOTE/EXTI equivalent and map to event masks. For now
 * provide skeletons so the application can link; they do nothing except
 * validate arguments.
 */
void hal_gpio_enable_interrupt(int pin, enum hal_gpio_edge e, hal_event_mask_t evt) {
    (void)pin;
    (void)e;
    (void)evt;
    /* No-op: platform interrupt support for GPIO not implemented yet. */
}

void hal_gpio_disable_interrupt(int pin) {
    (void)pin;
    /* No-op for now */
}
