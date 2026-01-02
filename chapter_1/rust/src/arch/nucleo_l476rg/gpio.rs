use core::ptr::{read_volatile, write_volatile};

use crate::hal::Gpio as GpioTrait;

// STM32L476 register map (GPIO) for Nucleo board.
const RCC_BASE: u32 = 0x4002_1000;
const RCC_AHB2ENR: *mut u32 = (RCC_BASE + 0x4C) as *mut u32;

const GPIO_BASES: [u32; 8] = [
    0x4800_0000, // A
    0x4800_0400, // B
    0x4800_0800, // C
    0x4800_0C00, // D
    0x4800_1000, // E
    0x4800_1400, // F
    0x4800_1800, // G
    0x4800_1C00, // H
];

pub struct Gpio {
    pin: u8,
    mask: u32,
    bsrr: *mut u32,
}

impl Gpio {
    pub fn new(pin: u8, open_drain: bool, initial_high: bool) -> Self {
        let port = (pin / 16) as usize;
        assert!(port < GPIO_BASES.len());
        let bit = pin % 16;
        let mask = 1u32 << bit;

        let base = GPIO_BASES[port];
        let moder = (base + 0x00) as *mut u32;
        let otyper = (base + 0x04) as *mut u32;
        let bsrr = (base + 0x18) as *mut u32;

        unsafe {
            // Enable GPIO clock
            let mut ahb = read_volatile(RCC_AHB2ENR);
            ahb |= 1 << port;
            write_volatile(RCC_AHB2ENR, ahb);

            // Set mode to output (01)
            let shift = (bit as u32) * 2;
            let mut m = read_volatile(moder);
            m &= !(0b11 << shift);
            m |= 0b01 << shift;
            write_volatile(moder, m);

            // Configure output type
            let mut ot = read_volatile(otyper);
            if open_drain {
                ot |= mask;
            } else {
                ot &= !mask;
            }
            write_volatile(otyper, ot);

            // Initial level
            if initial_high {
                write_volatile(bsrr, mask);
            } else {
                write_volatile(bsrr, mask << 16);
            }
        }

        Self { pin, mask, bsrr }
    }
}

impl GpioTrait for Gpio {
    fn write(&mut self, high: bool) {
        unsafe {
            if high {
                write_volatile(self.bsrr, self.mask);
            } else {
                write_volatile(self.bsrr, self.mask << 16);
            }
        }
    }
}
