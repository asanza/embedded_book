#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

mod arch;
mod hal;

use crate::hal::hal_gpio::{ConfigurablePin, Pull, Edge};
use crate::hal::hal_timer::Timer as TimerTrait;
use crate::hal::utils::{poll_all, Event};
use arch::{GpioImpl, TimerImpl};

#[entry]
fn main() -> ! {
    const DELAY_US: u32 = 500_000;
    // Initialize board resources via grouped cfg blocks to avoid repeating attributes
    let timers = TimerImpl::new();

    let blink_timer_opt: Option<_>;
    let debounce_timer_opt: Option<_>;
    let led_opt: Option<_>;
    let but_opt: Option<_>;

    #[cfg(feature = "nucleo")]
    {
        blink_timer_opt = Some(timers.t2.into_periodic());
        debounce_timer_opt = Some(timers.t3.into_oneshot());
        let gpio = GpioImpl::new();
        led_opt = Some(gpio.p5.into_output(false, true));
        but_opt = Some(gpio.p45.into_input(Pull::Up));
    }

    #[cfg(feature = "qemu")]
    {
        blink_timer_opt = Some(timers.t0.into_periodic());
        debounce_timer_opt = Some(timers.t1.into_oneshot());
        let gpio = GpioImpl::new();
        led_opt = Some(gpio.p5.into_output(false, true));
        but_opt = Some(gpio.p0.into_input(Pull::None));
    }

    let mut blink_timer = blink_timer_opt.expect("timer not configured");
    let mut debounce_timer = debounce_timer_opt.expect("debounce timer not configured");
    let mut led = led_opt.expect("led not configured");
    let mut but = but_opt.expect("button not configured");

    // Create events directly (no factory) — simple, static zero-sized events.
    let blink_evt = Event::<0>::new();
    let gpio_evt = Event::<1>::new();
    let debnc_evt = Event::<2>::new();

    // Associate event and start the blink timer
    blink_timer.enable_interrupt(blink_evt);
    blink_timer.start(DELAY_US);

    debounce_timer.enable_interrupt(debnc_evt);

    // Configure input interrupt to set bit 2 (GPIO event)
    but.enable_interrupt(Edge::Falling, gpio_evt);

    let mut running = false;
    let mut fast = false;

    loop {
        let evt = poll_all();

        // Handle all set event bits independently instead of an else-if chain.
        if evt & Event::<0>::mask() != 0 {
            let state = !led.read();
            led.write(state);
        }

        if evt & Event::<1>::mask() != 0 && !running {
            debounce_timer.start(20_000);
            running = true;
        }

        if evt & Event::<2>::mask() != 0 && running {
            // Toggle blink speed immediately on GPIO event (debounce omitted)
            blink_timer.stop();
            if !fast {
                blink_timer.start(50_000);
            } else {
                blink_timer.start(DELAY_US);
            }
            fast = !fast;
            running = false;
        }

        cortex_m::asm::wfi();
    }
}
