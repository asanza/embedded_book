#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

mod arch;
mod hal;

use crate::hal::hal_gpio::{ConfigurablePin, Pull};
use crate::hal::hal_timer::Timer as TimerTrait;
use crate::hal::utils::{Event, poll_all};
use arch::{GpioImpl, TimerImpl};

#[entry]
fn main() -> ! {
    const DELAY_US: u32 = 500_000;
    // Construct timer collection and acquire timer 0 as a running timer.
    let timers = TimerImpl::new();
    let mut blink_timer = timers.t2.into_periodic();

    // Use t3 as a one-shot timer placeholder (not used — debouncer removed).
    let mut _debounce_timer = timers.t3.into_oneshot();

    // Construct the board GPIO collection once and move individual pins out.
    let gpio = GpioImpl::new();
    let mut led = gpio.p5.into_output(false, true);

    #[cfg(feature = "nucleo")]
    let mut but = gpio.p45.into_input(Pull::Up);

    #[cfg(feature = "qemu")]
    let but = gpio.p0.into_input(Pull::None);

    // Use const-generic events (ZST) mapped to bits 0..n.
    type BlinkEvt = Event<0>;
    type DebncEvt = Event<1>;
    type GpioEvt = Event<2>;

    let blink_evt = BlinkEvt::new();
    let _debnc_evt = DebncEvt::new();
    let _gpio_evt = GpioEvt::new();

    // Initialize global events storage (if needed)
    crate::hal::utils::signal_mask(0); // no-op, ensures module linked

    // Start the blink timer
    blink_timer.start(DELAY_US, blink_evt);

    // Configure input interrupt to set bit 2 (GPIO event)
    but.enable_interrupt(crate::hal::hal_gpio::Edge::Falling, Event::<2>::mask());

    let _running = false;
    let mut fast = false;

    loop {
        let evt = poll_all();

        if evt & BlinkEvt::mask() != 0 {
            let state = !led.read();
            led.write(state);
        } else if evt & GpioEvt::mask() != 0 {
            // Toggle blink speed immediately on GPIO event (debounce omitted)
            blink_timer.stop();
            if !fast {
                blink_timer.start(50_000, blink_evt);
            } else {
                blink_timer.start(DELAY_US, blink_evt);
            }
            fast = !fast;
        }

        cortex_m::asm::wfi();
    }
}
