#include <hal_gpio.h>
#include <hal_timer.h>
#include <stdbool.h>
#include <stdio.h>

#define DELAY_MS       (500u * 1000)
#define LED_GPIO       (5)
#define TIMER_INSTANCE (0)

static void timer_callback(void* args)
{
    bool* fired = (bool*) args;
    *fired = true;
}

int main(void) {

    static volatile bool fired = false;

    hal_timer_init(TIMER_INSTANCE, DELAY_MS,false);
    hal_gpio_init_out(LED_GPIO, false, 1);
    hal_timer_start(TIMER_INSTANCE, timer_callback, (void*) &fired);

    while (1) {

        while(!fired);
        hal_gpio_write(LED_GPIO, 0);
        fired = false;

        while(!fired);
        hal_gpio_write(LED_GPIO, 1);
        fired = false;
    }

    return 0;
}
