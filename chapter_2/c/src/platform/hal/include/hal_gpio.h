
#ifndef HAL_GPIO_H
#define HAL_GPIO_H

#include <stdbool.h>

/**
 * @file hal_gpio.h
 * @brief Minimal HAL GPIO interface.
 *
 * This header exposes a tiny, platform-neutral GPIO API intended for
 * simple embedded projects. Implementations are platform-specific and
 * should interpret the integer `pin` identifier according to the
 * platform's pin numbering scheme.
 */

enum hal_gpio_pull {
    HAL_GPIO_PULLUP,
    HAL_GPIO_PULLDOWN,
    HAL_GPIO_PULLNONE,
};

/**
 * @brief Initialize a GPIO pin.
 *
 * Configure a pin for output and set its initial value.
 *
 * @param pin Numeric pin identifier (platform-specific).
 * @param od If true configure as open-drain; if false configure as push-pull.
 * @param value Initial output value: 0 for low, non-zero for high.
 *
 * @note Implementations should validate `pin` and may return early or
 * assert on invalid values. This API intentionally uses simple types to
 * avoid dependence on larger HAL frameworks.
 */
void hal_gpio_init_out(int pin, bool od, int value);

/**
 * @brief Drive a GPIO pin to a logic level.
 *
 * Set the output state of a previously-initialized pin.
 *
 * @param pin Numeric pin identifier (platform-specific).
 * @param value Output value: 0 for low, non-zero for high.
 */
void hal_gpio_write(int pin, int value);

/**
 * @brief Initialize a GPIO pin for input.
 *
 * Configure a pin as an input with the specified pull resistor setting.
 * Implementations should interpret the integer `pin` according to the
 * platform's numbering scheme.
 *
 * @param pin Numeric pin identifier (platform-specific).
 * @param pull Pull configuration: HAL_GPIO_PULLUP, HAL_GPIO_PULLDOWN, or HAL_GPIO_PULLNONE.
 * @return 0 on success, non-zero on error.
 */
int hal_gpio_init_in(int pin, enum hal_gpio_pull pull);


/**
 * @brief Read the current logic level of a GPIO pin.
 *
 * @param pin Numeric pin identifier (platform-specific).
 * @return 0 if the pin is driving a logical low, non-zero if logical high.
 *         Implementations may return a negative value to indicate an error.
 */
int hal_gpio_read(int pin);

#endif // HAL_GPIO_H

