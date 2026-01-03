
#ifndef HAL_TIMER_H
#define HAL_TIMER_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @file hal_timer.h
 * @brief Minimal HAL timer interface.
 *
 * Provides a tiny, platform-neutral timer API. Implementations are
 * platform-specific and should interpret the integer `timer instance`
 * identifier according to the platform's timer numbering scheme.
 */

/**
 * @brief Initialize a hardware timer instance.
 *
 * @param timer_instance Numeric timer instance identifier (platform-specific).
 * @param one_shoot If true the timer operates in one-shot mode; if false it
 *                  operates in periodic mode (retriggering).
 */
void hal_timer_init(int timer_instance, bool one_shoot);

/**
 * @brief Block (busy or sleep depending on implementation) for a period.
 *
 * Implementations may use the specified hardware timer instance to
 * implement the delay. The units are milliseconds.
 *
 * @param timer_instance Numeric timer instance identifier (platform-specific).
 * @param period_ms Delay period in milliseconds.
 */
void hal_timer_delay_ms(int timer_instance, uint32_t period_ms);

#endif // HAL_TIMER_H

