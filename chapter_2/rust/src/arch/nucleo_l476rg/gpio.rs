use core::marker::PhantomData;
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

pub mod typestate {
    pub struct NotConfigured;
    pub struct Output;
    pub struct Input;
}

use typestate::*;

pub struct Pin<MODE, const PIN: u8> {
    _marker: PhantomData<MODE>,
}

impl<MODE, const PIN: u8> Pin<MODE, PIN> {
    fn new() -> Self { Pin { _marker: PhantomData } }
}

impl<const PIN: u8> Pin<NotConfigured, PIN> {
    pub fn into_output(self, open_drain: bool, initial_high: bool) -> Pin<Output, PIN> {
        let port = (PIN / 16) as usize;
        assert!(port < GPIO_BASES.len());
        let bit = (PIN % 16) as u32;
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
            let shift = bit * 2;
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

        Pin::new()
    }
}

impl<const PIN: u8> Pin<Output, PIN> {
    pub fn write(&mut self, high: bool) {
        let port = (PIN / 16) as usize;
        let bit = (PIN % 16) as u32;
        let mask = 1u32 << bit;
        let base = GPIO_BASES[port];
        let bsrr = (base + 0x18) as *mut u32;
        unsafe {
            if high {
                write_volatile(bsrr, mask);
            } else {
                write_volatile(bsrr, mask << 16);
            }
        }
    }
}

impl<const PIN: u8> GpioTrait for Pin<Output, PIN> {
    fn write(&mut self, high: bool) { self.write(high); }
}

// Generate Gpio struct with p0..p31 fields (board-visible pins). Move-out enforces uniqueness.
macro_rules! make_pins {
    ($($idx:expr),*) => {
        paste::paste! {
            pub struct Gpio {
                $( pub [<p $idx>]: Pin<NotConfigured, $idx>, )*
            }

            impl Gpio {
                pub fn new() -> Self {
                    Self { $( [<p $idx>]: Pin::new(), )* }
                }
            }
        }
    }
}

make_pins!(0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31);
