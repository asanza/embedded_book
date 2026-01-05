#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

mod arch;
mod hal;

use crate::hal::hal_gpio::{ConfigurablePin, Pull};
use crate::hal::hal_timer::Timer as TimerTrait;
use crate::hal::utils::{poll_all, Event};
use arch::{GpioImpl, TimerImpl};

#[entry]
fn main() -> ! {
    const DELAY_US: u32 = 500_000;
    // Construct timer collection and acquire timer 0 as a running timer.
    let timers = TimerImpl::new();
    let mut blink_timer = timers.t2.into_periodic();

    // Use t3 as a one-shot timer placeholder (not used — debouncer removed).
    let mut debounce_timer = timers.t3.into_oneshot();

    // Construct the board GPIO collection once and move individual pins out.
    let gpio = GpioImpl::new();
    let mut led = gpio.p5.into_output(false, true);

    #[cfg(feature = "nucleo")]
    let mut but = gpio.p45.into_input(Pull::Up);

    #[cfg(feature = "qemu")]
    let but = gpio.p0.into_input(Pull::None);

    // Create events via a value-level factory (static wiring but ergonomic)
    let factory = crate::hal::utils::EventFactory::new();
    let (blink_evt, factory) = factory.create::<0>().expect("alloc blink");
    let (gpio_evt, _factory) = factory.create::<1>().expect("alloc gpio");
    let (debnc_evt, factory) = factory.create::<2>().expect("alloc debnc");

    // Initialize global events storage (if needed)
    crate::hal::utils::signal_mask(0); // no-op, ensures module linked

    // Associate event and start the blink timer
    blink_timer.enable_interrupt(blink_evt);
    blink_timer.start(DELAY_US);

    debounce_timer.enable_interrupt(debnc_evt);

    // Configure input interrupt to set bit 2 (GPIO event)
    #[cfg(feature = "nucleo")]
    but.enable_interrupt(crate::hal::hal_gpio::Edge::Falling, gpio_evt);

    let mut running = false;
    let mut fast = false;

    loop {
        let evt = poll_all();

        if evt & Event::<0>::mask() != 0 {
            let state = !led.read();
            led.write(state);
        } else if evt & Event::<1>::mask() != 0 && !running {
            debounce_timer.start(20_000);
            running = true;
        } else if evt & Event::<2>::mask() != 0 && running {
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
