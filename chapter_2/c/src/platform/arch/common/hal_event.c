/* hal_event.c
 * Simple event subsystem: single 32-bit mask protected by disabling IRQs.
 * This avoids libatomic and works on single-core Cortex-M devices.
 */

#include <hal_event.h>
#include <stdint.h>

static volatile hal_event_mask_t events = 0u;

static inline uint32_t primask_save_and_disable(void) {
    uint32_t prim;
    __asm volatile("mrs %0, PRIMASK" : "=r" (prim) :: "memory");
    __asm volatile("cpsid i" ::: "memory");
    return prim;
}

static inline void primask_restore(uint32_t prim) {
    __asm volatile("msr PRIMASK, %0" :: "r" (prim) : "memory");
}

void hal_event_init(void)
{
    uint32_t p = primask_save_and_disable();
    events = 0u;
    primask_restore(p);
}

void hal_event_set_mask(hal_event_mask_t mask)
{
    if (mask == 0u) return;
    uint32_t p = primask_save_and_disable();
    events |= mask;
    primask_restore(p);
}

hal_event_mask_t hal_event_poll(void)
{
    uint32_t p = primask_save_and_disable();
    uint32_t curr = events;
    if (curr == 0u) {
        primask_restore(p);
        return 0u;
    }

    uint32_t bit = curr & (uint32_t)(-(int32_t)curr);
    events = curr & ~bit;
    primask_restore(p);
    return bit;
}

hal_event_mask_t hal_event_poll_all(void)
{
    uint32_t p = primask_save_and_disable();
    uint32_t prev = events;
    events = 0u;
    primask_restore(p);
    return prev;
}

void hal_event_clear_mask(hal_event_mask_t mask)
{
    if (mask == 0u) return;
    uint32_t p = primask_save_and_disable();
    events &= ~mask;
    primask_restore(p);
}
