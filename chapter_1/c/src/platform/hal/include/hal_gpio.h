
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

#endif // HAL_GPIO_H

