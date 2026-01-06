/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026 Diego Asanza <f.asanza@gmail.com> */

#include <hal_gpio.h>
#include <hal_event.h>
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

static inline void NVIC_EnableIRQ(uint32_t IRQn) {
    if ((int32_t)IRQn >= 0) {
        REG(0xE000E100u) |= (1u << (IRQn & 0x1Fu));
    }
}

/* Per-pin event masks for enabled interrupts (0 = none). */
static hal_event_mask_t gpio_event_masks[32] = {0};

/* GPIOTE peripheral base (nRF-style) and EVENTS_IN offsets. QEMU's
 * micro:bit model uses these addresses for pin event reporting.
 */
#define GPIOTE_BASE (0x40006000u)
#define GPIOTE_EVENTS_IN_OFFSET (0x100u)
#define GPIOTE_IRQN (6u)

/* PIN_CNF.SENSE field assumed at bit 16 occupying 2 bits (value: 0=Disabled, 1=High, 2=Low)
 * Use these helpers to set/clear the sense field.
 */
#define PIN_CNF_SENSE_SHIFT (16u)
#define PIN_CNF_SENSE_MASK (0x3u << PIN_CNF_SENSE_SHIFT)
#define PIN_CNF_SENSE_HIGH (0x1u << PIN_CNF_SENSE_SHIFT)
#define PIN_CNF_SENSE_LOW  (0x2u << PIN_CNF_SENSE_SHIFT)

/* Software shadows to track pin direction and output state. QEMU's
 * peripheral model for the micro:bit may not reflect writes to the
 * output registers in the input register; maintain shadow state so
 * hal_gpio_read behaves as expected for pins configured as outputs.
 */
static uint32_t gpio_dir_out_mask = 0u; /* 1 = configured as output */
static uint32_t gpio_out_state = 0u;   /* 1 = logical high last written */

void hal_gpio_init_out(int pin, bool od, int value) {
    (void)od;
    gpio_dir_out_mask |= (1u << pin);
    if (value) {
        REG(GPIO_OUTSET) = 1u << pin;
        gpio_out_state |= (1u << pin);
    } else {
        REG(GPIO_OUTCLR) = 1u << pin;
        gpio_out_state &= ~(1u << pin);
    }
}

void hal_gpio_write(int pin, int value) {
    if (value) {
        REG(GPIO_OUTSET) = 1u << pin;
        gpio_out_state |= (1u << pin);
    } else {
        REG(GPIO_OUTCLR) = 1u << pin;
        gpio_out_state &= ~(1u << pin);
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

    /* Clear any sense configuration by default when configuring as input. */
    uint32_t c = REG(cfg) & ~PIN_CNF_SENSE_MASK;
    REG(cfg) = c;

    return 0;
}

int hal_gpio_read(int pin) {
    if (pin < 0 || pin >= 32) {
        return -1;
    }
    /* If configured as output, return the last driven value from the
     * software shadow. Otherwise read the peripheral input register. */
    uint32_t mask = 1u << pin;
    if (gpio_dir_out_mask & mask) {
        return (gpio_out_state & mask) ? 1 : 0;
    }
    return (REG(GPIO_IN) & mask) ? 1 : 0;
}


/* Minimal interrupt support for microbit platform. The real implementation
 * would configure GPIOTE/EXTI equivalent and map to event masks. For now
 * provide skeletons so the application can link; they do nothing except
 * validate arguments.
 */
void hal_gpio_enable_interrupt(int pin, enum hal_gpio_edge e, hal_event_mask_t evt) {
    if (pin < 0 || pin >= 32) return;
    uintptr_t cfg = GPIO_PIN_CNF_BASE + (unsigned)pin * 4U;

    /* Save event mask for use by IRQ handler */
    gpio_event_masks[pin] = evt;

    /* Configure the PIN_CNF SENSE field to generate events on the requested edge.
     * For rising edge, configure SENSE to HIGH (event when pin is driven high).
     * For falling edge, configure SENSE to LOW. For BOTH, enable both by
     * setting SENSE to trigger on HIGH and rely on the IRQ handler to detect
     * subsequent transitions as needed.
     */
    uint32_t v = REG(cfg) & ~PIN_CNF_SENSE_MASK;
    if (e == HAL_GPIO_EDGE_RISING) {
        v |= PIN_CNF_SENSE_HIGH;
    } else if (e == HAL_GPIO_EDGE_FALLING) {
        v |= PIN_CNF_SENSE_LOW;
    } else {
        /* For 'both', enable high sense (will at least catch one edge); the
         * software may sample input in the handler to detect the other edge.
         */
        v |= PIN_CNF_SENSE_HIGH;
    }
    REG(cfg) = v;

    /* Enable GPIOTE IRQ in NVIC so hardware events are delivered. */
    NVIC_EnableIRQ(GPIOTE_IRQN);
}

void hal_gpio_disable_interrupt(int pin) {
    if (pin < 0 || pin >= 32) return;
    uintptr_t cfg = GPIO_PIN_CNF_BASE + (unsigned)pin * 4U;
    /* Clear sense to disable hardware event generation */
    REG(cfg) &= ~PIN_CNF_SENSE_MASK;
    gpio_event_masks[pin] = 0;
}

/* GPIOTE IRQ handler: check EVENTS_IN[n] and forward to hal_event_set_mask.
 * The handler is intentionally small: it clears events and sets the user
 * supplied event mask so the application can handle the input in thread
 * context. This mirrors the approach used for the Nucleo EXTI handler.
 */
void GPIOTE_IRQHandler(void) {
    for (unsigned pin = 0; pin < 32; ++pin) {
        uintptr_t ev = GPIOTE_BASE + GPIOTE_EVENTS_IN_OFFSET + (pin * 4u);
        if (REG(ev)) {
            REG(ev) = 0u; /* clear event */
            hal_event_mask_t m = gpio_event_masks[pin];
            if (m) {
                hal_event_set_mask(m);
            }
        }
    }
}
