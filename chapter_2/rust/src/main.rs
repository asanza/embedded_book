#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

mod arch;
mod hal;

use crate::hal::hal_gpio::{ConfigurablePin, Pull};
use crate::hal::hal_timer::Timer as TimerTrait;
use crate::hal::utils::{DebounceEdge, Debouncer};
use arch::{GpioImpl, TimerImpl};

#[entry]
fn main() -> ! {
    const DELAY_US: u32 = 500_000;
    // Construct timer collection and acquire timer 0 as a running timer.
    let timers = TimerImpl::new();
    let mut timer = timers.t0.into_periodic();

    // Use t1 as a one-shot timer for debouncing.
    let one_shot = timers.t1.into_oneshot();

    // Construct the board GPIO collection once and move individual pins out.
    let gpio = GpioImpl::new();
    let mut led = gpio.p5.into_output(false, true);

    #[cfg(feature = "nucleo")]
    let but = gpio.p45.into_input(Pull::Up);

    #[cfg(feature = "qemu")]
    let but = gpio.p0.into_input(Pull::None);

    // Create an event (static storage) and start the blink timer.
    let ev = crate::make_event!();
    timer.start(DELAY_US, ev.clone());

    // The DebouncerOneShot will register the pin interrupt itself.

    let mut state = false;
    // Logical toggle state: false = slow (500ms), true = fast (100ms)
    let mut fast = false;
    // Create a Debouncer that owns the input pin and the one-shot timer.
    // The constructor auto-detects the pressed polarity and uses the given timeout.
    let mut deb = Debouncer::new(but, one_shot, 20_000);

    loop {
        if ev.poll() {
            state = !state;
            led.write(state);
        }

        // One-shot debouncer: poll the input event and arm timer as needed.
        if let Some(edge) = deb.poll() {
            match edge {
                DebounceEdge::Pressed => {
                    fast = !fast;
                    timer.stop();
                    if fast {
                        timer.start(50_000, ev.clone());
                    } else {
                        timer.start(500_000, ev.clone());
                    }
                }
                DebounceEdge::Released => {
                    // no-op
                }
            }
        }

        cortex_m::asm::wfi();
    }
}
