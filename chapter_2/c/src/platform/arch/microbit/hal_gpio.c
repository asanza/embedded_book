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
