#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

mod hal;
mod arch;

use crate::hal::{Gpio as GpioTrait, Timer as TimerTrait};
use arch::{GpioImpl, TimerImpl};

#[entry]
fn main() -> ! {
    let mut timer = TimerImpl::new(0, false);
    let mut led = GpioImpl::new(5, false, true);

    loop {
        led.write(true);
        timer.delay_ms(1000);
        led.write(false);
        timer.delay_ms(1000);
    }
}
