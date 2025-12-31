#include <stdio.h>
#include <stdbool.h>
#include <hal_timer.h>
#include <hal_gpio.h>


int main(void)
{
    hal_timer_init(0, false);
    hal_gpio_init_out(5, false, 1);
    while(1)
    {
        hal_gpio_write(5, 1);
        hal_timer_delay_ms(0, 1000);
        hal_gpio_write(5, 0);
        hal_timer_delay_ms(0, 1000);
    }
    return 0;
}