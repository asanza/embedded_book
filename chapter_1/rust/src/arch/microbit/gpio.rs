use core::ptr::write_volatile;
#[cfg(feature = "qemu")]
use cortex_m_semihosting::hprintln;

use crate::hal::Gpio as GpioTrait;

// NRF51 (micro:bit) peripheral addresses used by the C implementation
const GPIO_BASE: u32 = 0x5000_0000;
const GPIO_OUTSET: *mut u32 = (GPIO_BASE + 0x508) as *mut u32;
const GPIO_OUTCLR: *mut u32 = (GPIO_BASE + 0x50C) as *mut u32;
const GPIO_DIRSET: *mut u32 = (GPIO_BASE + 0x518) as *mut u32;

pub struct Gpio {
    pin: u8,
}

impl Gpio {
    pub fn new(pin: u8, _open_drain: bool, initial_high: bool) -> Self {
        let pin_mask = 1u32 << pin;
        unsafe {
            write_volatile(GPIO_DIRSET, pin_mask); // configure as output
            if initial_high {
                write_volatile(GPIO_OUTSET, pin_mask);
            } else {
                write_volatile(GPIO_OUTCLR, pin_mask);
            }
        }
        Self { pin }
    }
}

impl GpioTrait for Gpio {
    fn write(&mut self, high: bool) {
        let pin_mask = 1u32 << self.pin;
        unsafe {
            if high {
                write_volatile(GPIO_OUTSET, pin_mask);
            } else {
                write_volatile(GPIO_OUTCLR, pin_mask);
            }
        }

        #[cfg(feature = "qemu")]
        {
            let _ = hprintln!("GPIO pin {} = {}", self.pin, if high { 1 } else { 0 });
        }
    }
}
