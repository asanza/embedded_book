#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

mod arch;
mod hal;

use crate::hal::hal_gpio::{ConfigurablePin, Pull, Edge};
use crate::hal::hal_timer::Timer as TimerTrait;
use crate::hal::utils::{DebounceEdge, DebouncerOneShot};
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
    let mut but = gpio.p45.into_input(Pull::Up);

    #[cfg(feature = "qemu")]
    let mut but = gpio.p0.into_input(Pull::None);

    // Create an event (static storage) and start the blink timer.
    let ev = crate::make_event!();
    timer.start(DELAY_US, ev.clone());

    // lets create an event for the gpio and bind the interrupt to it.
    let gev = crate::make_event!();
    but.enable_interrupt(crate::Edge::Rising, gev);

    let mut state = false;
    // Read initial button state to compute pressed polarity.
    let pressed_level = !but.read();
    // Logical toggle state: false = slow (500ms), true = fast (100ms)
    let mut fast = false;
    // Create a DebouncerOneShot using a closure reader over `but`.
    let mut deb = DebouncerOneShot::new(|| but.read(), one_shot, pressed_level);

    loop {
        if ev.poll() {
            state = !state;
            led.write(state);
        }

        // One-shot debouncer: poll the one-shot event and update state.
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
