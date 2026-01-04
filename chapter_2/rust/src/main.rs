#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

mod hal;
mod arch;
mod event;

use crate::hal::{Timer as TimerTrait};
use arch::{GpioImpl, TimerImpl};
use crate::event::Event;


#[entry]
fn main() -> ! {
    const DELAY_US:u32 = 500_000;
    // Construct timer collection and acquire timer 0 as a running timer.
    let timers = TimerImpl::new();
    let mut timer = timers.t0.into_running(false);
    // Construct the board GPIO collection once and move individual pins out.
    let gpio = GpioImpl::new();
    let mut led = gpio.p5.into_output(false, true);

    // Create an event (static storage) and start the timer to trigger it.
    let ev = crate::make_event!();
    timer.start(DELAY_US, ev.clone()); // 2 Hz toggling

    let mut state = false;
    loop {
        if ev.poll() {
            state = !state;
            led.write(state);
        }
        cortex_m::asm::wfi();
    }
}
