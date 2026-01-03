#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

mod hal;
mod arch;

use crate::hal::{Timer as TimerTrait};
use arch::{GpioImpl, TimerImpl};

#[entry]
fn main() -> ! {
    const DELAY_MS:u32 = 500;
    // Construct timer collection and acquire timer 0 as a running timer.
    let timers = TimerImpl::new();
    let mut timer = timers.t0.into_running(false);
    // Construct the board GPIO collection once and move individual pins out.
    let gpio = GpioImpl::new();
    let mut led = gpio.p5.into_output(false, true);

    loop {
        led.write(true);
        timer.delay_ms(DELAY_MS);
        led.write(false);
        timer.delay_ms(DELAY_MS);
    }
}
