#include <hal_gpio.h>
#include <hal_timer.h>
#include <stdbool.h>
#include <stdio.h>

#define DELAY_MS       (500u)
#define LED_GPIO       (5)
#define TIMER_INSTANCE (0)

int main(void) {

    hal_timer_init(TIMER_INSTANCE, false);
    hal_gpio_init_out(5, false, 1);

    while (1) {
        hal_gpio_write(LED_GPIO, 1);
        hal_timer_delay_ms(TIMER_INSTANCE, DELAY_MS);
        hal_gpio_write(LED_GPIO, 0);
        hal_timer_delay_ms(TIMER_INSTANCE, DELAY_MS);
    }

    return 0;
}
