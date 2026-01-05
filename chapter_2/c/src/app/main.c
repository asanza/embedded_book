#include <hal_gpio.h>
#include <hal_timer.h>
#include <stdbool.h>
#include <stdio.h>

#define DELAY_MS       (500u * 1000)
#define LED_GPIO       (5)
#define BLINK_TIMER (2)
#define DEBOUNCE_TIMER (3)

int main(void) {

    hal_event_init();

    /* Use event bit 0 for the timer */
    const hal_event_mask_t BLINK_EVT = HAL_EVENT(0);
    const hal_event_mask_t DEBNC_EVT = HAL_EVENT(1);
    const hal_event_mask_t GPIO_EVT = HAL_EVENT(2);

    hal_timer_init(BLINK_TIMER, false, BLINK_EVT);
    hal_timer_init(DEBOUNCE_TIMER, true, DEBNC_EVT);
    hal_gpio_init_out(LED_GPIO, false, 1);
    hal_timer_start(BLINK_TIMER, DELAY_MS);

    hal_gpio_init_in(45, HAL_GPIO_PULLUP);
    hal_gpio_enable_interrupt(45, HAL_GPIO_EDGE_FALLING, GPIO_EVT);

    bool led_state = true;
    bool running = false;
    bool fast = false;
    while (1) {
        hal_event_mask_t evt = hal_event_poll();
        /* wait for timer event */
        if ( evt & BLINK_EVT ) {
            led_state = !led_state;
            hal_gpio_write(LED_GPIO, led_state ? 1 : 0);
        } else if(evt & GPIO_EVT && !running ) {
            hal_timer_start(DEBOUNCE_TIMER, 10000);
            running = true;
        } else if(evt & DEBNC_EVT && running) {
            hal_timer_stop(BLINK_TIMER);
            if (fast) {
                hal_timer_start(BLINK_TIMER, 50000);
            } else {
                hal_timer_start(BLINK_TIMER, DELAY_MS);
            }
            running = false;
            fast = !fast;
        }
    }

    return 0;
}
