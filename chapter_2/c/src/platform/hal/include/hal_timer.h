
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
 * @brief Timer callback type.
 *
 * Called from the timer context (interrupt or dedicated thread, platform
 * specific) when the timer fires. The provided `arg` pointer is the same
 * value passed to `hal_timer_start` and may be NULL.
 */
typedef void (*hal_timer_cb)(void* arg);


/**
 * @brief Initialize a hardware timer instance.
 *
 * Configure a hardware timer for use. `timer_instance` is a numeric,
 * platform-specific identifier (for example a peripheral index). `period_us`
 * sets the initial period in microseconds. If `one_shoot` is true the timer
 * will fire once and stop when started; otherwise it should operate in
 * periodic mode and retrigger automatically.
 *
 * Implementations should prepare the timer but need not start it; use
 * `hal_timer_start` to begin operation. Validation failures or unsupported
 * parameters may be handled via asserts or returned as runtime errors by the
 * platform implementation (this API uses `void` for simplicity).
 *
 * @param timer_instance Platform-specific timer identifier.
 * @param period_us Initial timer period, in microseconds.
 * @param one_shoot true for one-shot mode, false for periodic mode.
 */
void hal_timer_init(int timer_instance, uint32_t period_us, bool one_shoot);


/**
 * @brief Change the timer period.
 *
 * Update the timer period while the timer is stopped or running. Behavior
 * when changing the period while the timer is active is platform-dependent:
 * implementations may apply the new period immediately or on the next cycle.
 *
 * @param timer_instance Platform-specific timer identifier.
 * @param period_us New period in microseconds.
 */
void hal_timer_set_period(int timer_instance, uint32_t period_us);


/**
 * @brief Start the timer and register a callback.
 *
 * When the timer fires it must invoke `cb(arg)`. The callback may be called
 * from an interrupt context; keep handlers short and use deferred work for
 * lengthy processing. If `cb` is NULL the timer should still run but no
 * callback will be invoked.
 *
 * @param timer_instance Platform-specific timer identifier.
 * @param cb Callback to invoke on timer expiry (may be NULL).
 * @param arg Opaque pointer passed to the callback (may be NULL).
 */
void hal_timer_start(int timer_instance, hal_timer_cb cb, void* arg);


/**
 * @brief Stop the timer.
 *
 * Stop the timer if it is running. If a callback is pending or currently
 * running, semantics are platform-dependent; implementations should document
 * whether the callback may still run after `hal_timer_stop` returns.
 *
 * @param timer_instance Platform-specific timer identifier.
 */
void hal_timer_stop(int timer_instance);

#endif // HAL_TIMER_H

