/* hal_event.c
 * Simple event subsystem: fixed number of groups of 32-bit masks.
 * Designed to be safe to call from interrupt context.
 */


#include "include/hal_event.h"
#include <stdatomic.h>
#include <stdint.h>

static atomic_uint_fast32_t events = 0u;

void hal_event_init(void)
{
    atomic_store(&events, 0u);
}

void hal_event_set_mask(hal_event_mask_t mask)
{
    if (mask == 0u) return;
    atomic_fetch_or(&events, mask);
}

hal_event_mask_t hal_event_poll(void)
{
    uint32_t curr = atomic_load(&events);
    if (curr == 0u) return 0u;

    /* lowest set bit */
    uint32_t bit = curr & (uint32_t)(-(int32_t)curr);

    /* try to clear that bit */
    uint32_t expected = curr;
    uint32_t desired = curr & ~bit;
    if (atomic_compare_exchange_strong(&events, &expected, desired)) {
        return bit;
    }

    /* CAS failed — best-effort clear using fetch_and */
    atomic_fetch_and(&events, ~bit);
    return bit;
}

hal_event_mask_t hal_event_poll_all(void)
{
    return atomic_exchange(&events, 0u);
}

void hal_event_clear_mask(hal_event_mask_t mask)
{
    if (mask == 0u) return;
    atomic_fetch_and(&events, ~mask);
}
