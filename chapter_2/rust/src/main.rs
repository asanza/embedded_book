#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

mod hal;
mod arch;
mod event;
mod debouncer;

use crate::hal::{Timer as TimerTrait, ConfigurablePin};
use arch::{GpioImpl, TimerImpl};


#[entry]
fn main() -> ! {
    const DELAY_US:u32 = 500_000;
    // Construct timer collection and acquire timer 0 as a running timer.
    let timers = TimerImpl::new();
    let mut timer = timers.t0.into_running(false);

    // Use t1 as a periodic sampling timer (not one-shot). into_running(false)
    // makes it periodic for sampling the button every few ms.
    let mut debounce = timers.t1.into_running(false);

    // Construct the board GPIO collection once and move individual pins out.
    let gpio = GpioImpl::new();
    let mut led = gpio.p5.into_output(false, true);

    #[cfg(feature="nucleo")]
    let mut but = gpio.p45.into_input(crate::hal::Pull::Up);

    #[cfg(feature="qemu")]
    let mut but = gpio.p0.into_input(crate::hal::Pull::None);

    // Create an event (static storage) and start the timer to trigger it.
    let ev = crate::make_event!();
    timer.start(DELAY_US, ev.clone()); // 2 Hz toggling

    let db = crate::make_event!();

    let mut state = false;
    // Read initial button state to track transitions.
    let mut but_prev = but.read();
    let idle_level = but_prev;
    let pressed_level = !idle_level;
    // Logical toggle state: false = slow (500ms), true = fast (100ms)
    let mut fast = false;
    // Start sampling timer at 5 ms (5000 us)
    debounce.start(5_000, db.clone());

    // Create a Debouncer that samples the `but.read()` closure.
    let mut deb = debouncer::Debouncer::new(|| but.read(), 3, pressed_level);

    loop {
        if ev.poll() {
            state = !state;
            led.write(state);
        }

        // Periodic sampling: when the sampling timer fires read the button and
        // update the signed sample counter towards pressed (+) or released (-).
        if db.poll() {
            if let Some(edge) = deb.sample() {
                match edge {
                    debouncer::Edge::Pressed => {
                        if but_prev != pressed_level {
                            fast = !fast;
                            timer.stop();
                            if fast { timer.start(100_000, ev.clone()); } else { timer.start(500_000, ev.clone()); }
                        }
                        but_prev = pressed_level;
                    }
                    debouncer::Edge::Released => {
                        but_prev = !pressed_level;
                    }
                }
            }
        }

        cortex_m::asm::wfi();
    }
}
