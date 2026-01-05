#ifndef HAL_EVENT_H_A0F0E86C_EB07_4963_9829_97FC85F73577
#define HAL_EVENT_H_A0F0E86C_EB07_4963_9829_97FC85F73577

/* Single 32-bit event mask API.
 * The system supports up to 32 distinct events. Each event is a single
 * bit in a 32-bit mask. Interrupts must set bits; foreground code polls
 * and clears them atomically.
 */

#include <stdint.h>
#include <stddef.h>

typedef uint32_t hal_event_mask_t;  /* single 32-bit event mask */

/* Initialize the event subsystem. Call once at startup. */
void hal_event_init(void);

/* Set one or more event bits (callable from interrupt context). */
void hal_event_set_mask(hal_event_mask_t mask);

/* Poll and atomically clear a single event bit. Returns 0 if none.
 * If multiple bits are set, the implementation will prefer the lowest
 * set bit and return a mask with that single bit set.
 */
hal_event_mask_t hal_event_poll(void);

/* Atomically read-and-clear the entire event mask, returning the previous
 * mask (0 if none). Useful for draining all pending events.
 */
hal_event_mask_t hal_event_poll_all(void);

/* Clear specific bits in the mask (callable from thread context). */
void hal_event_clear_mask(hal_event_mask_t mask);


/* --- Backwards compatibility helpers ---
 * Provide macros to map legacy `HAL_EVENT_<TYPE>_<N>` names to masks.
 */

/* Helper to build per-index masks: HAL_EVENT(n) -> (1u << n) */
#define HAL_EVENT(n) ((hal_event_mask_t)(1u << (n)))

#endif /* HAL_EVENT_H_A0F0E86C_EB07_4963_9829_97FC85F73577 */
