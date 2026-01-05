#include <hal_gpio.h>
#include <hal_timer.h>
#include <stdbool.h>
#include <stdio.h>

#define DELAY_MS       (500u * 1000)
#define LED_GPIO       (5)
#define TIMER_INSTANCE (2)

static void timer_callback(void* args)
{
    bool* fired = (bool*) args;
    *fired = true;
}

int main(void) {

    hal_event_init();

    /* Use event bit 0 for the timer */
    const hal_event_mask_t TIMER_EVT = HAL_EVENT(0);

    hal_timer_init(TIMER_INSTANCE, false, TIMER_EVT);
    hal_gpio_init_out(LED_GPIO, false, 1);
    hal_timer_start(TIMER_INSTANCE, DELAY_MS);

    bool led_state = true;
    while (1) {
        /* wait for timer event */
        while (hal_event_poll() == 0) {
            ;
        }
        led_state = !led_state;
        hal_gpio_write(LED_GPIO, led_state ? 1 : 0);
    }

    return 0;
}
