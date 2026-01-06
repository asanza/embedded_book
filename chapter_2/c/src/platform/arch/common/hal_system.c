#include <hal_system.h>

void hal_wait_for_interrupt( void )
{
    __asm volatile("wfi" ::: "memory");
}
