use core::marker::PhantomData;
use core::ptr::write_volatile;
use cortex_m::interrupt;
#[cfg(feature = "qemu")]
use cortex_m_semihosting::hprintln;

use crate::hal::Gpio as GpioTrait;

// NRF51 (micro:bit) peripheral addresses used by the C implementation
const GPIO_BASE: u32 = 0x5000_0000;
const GPIO_OUTSET: *mut u32 = (GPIO_BASE + 0x508) as *mut u32;
const GPIO_OUTCLR: *mut u32 = (GPIO_BASE + 0x50C) as *mut u32;
const GPIO_DIRSET: *mut u32 = (GPIO_BASE + 0x518) as *mut u32;

// Global mask to track which pins have been configured as outputs. Guarded
// by a critical section when mutated to avoid races on Cortex-M.
static mut GPIO_INIT_MASK: u32 = 0;

pub mod typestate {
    pub struct NotConfigured;
    pub struct Output;
    pub struct Input;
}

use typestate::*;

pub struct Pin<MODE, const INDEX: u8> {
    _marker: PhantomData<MODE>
}

impl<MODE, const INDEX: u8> Pin<MODE, INDEX> {
    fn new() -> Self { Pin { _marker: PhantomData } }
}

impl<const INDEX: u8> Pin<NotConfigured, INDEX> {
    /// Convert this pin into an output pin, performing any required hardware init.
    pub fn into_output(self, _open_drain: bool, initial_high: bool) -> Pin<Output, INDEX> {
        let bit = 1u32 << INDEX;
        let mut do_init = false;
        interrupt::free(|_| unsafe {
            if (GPIO_INIT_MASK & bit) == 0 { GPIO_INIT_MASK |= bit; do_init = true; }
        });

        if do_init {
            unsafe { write_volatile(GPIO_DIRSET, bit); }
        }
        unsafe {
            if initial_high { write_volatile(GPIO_OUTSET, bit); } else { write_volatile(GPIO_OUTCLR, bit); }
        }

        Pin::new()
    }
}

impl<const INDEX: u8> Pin<Output, INDEX> {
    pub fn write(&mut self, high: bool) {
        let bit = 1u32 << INDEX;
        unsafe {
            if high { write_volatile(GPIO_OUTSET, bit); } else { write_volatile(GPIO_OUTCLR, bit); }
        }
        #[cfg(feature = "qemu")] { let _ = hprintln!("GPIO pin {} = {}", INDEX, if high { 1 } else { 0 }); }
    }
}

impl<const INDEX: u8> GpioTrait for Pin<Output, INDEX> {
    fn write(&mut self, high: bool) { self.write(high); }
}

// Macro-driven generation of the micro:bit pin collection and aliases.
// We only expose pins 0..=9 for the micro:bit board.
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

            // Export concise aliases P0..P9
            $( pub type [<P $idx>]<MODE> = Pin<MODE, $idx>; )*
        }
    }
}

make_pins!(0,1,2,3,4,5,6,7,8,9);

