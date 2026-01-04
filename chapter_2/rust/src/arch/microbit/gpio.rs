/*
MIT License

Copyright (c) 2026 Diego.Asanza <f.asanza@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

//! Micro:bit GPIO typestate implementation
//!
//! This module provides zero-sized, typestate-driven `Pin<MODE, INDEX>` types
//! and a generated `Gpio` collection with fields `p0..p9`. Each `Pin` is a
//! compile-time distinct type; moving a field out of the `Gpio` struct consumes
//! that pin value and prevents re-acquisition of the same pin at compile time.
//!
//! Usage summary:
//! - Construct the board collection: `let gpio = Gpio::new();`
//! - Move a pin out and configure as output: `let mut led = gpio.p5.into_output(false, true);`
//! - Use the pin: `led.write(true);`
//!
//! Rationale and safety notes:
//! - Pins are represented as zero-sized typestates, so they incur no runtime
//!   memory cost; the generated methods perform register writes directly.
//! - Hardware initialization for an output pin is performed when converting
//!   `NotConfigured` -> `Output`. For the micro:bit this is tracked via a
//!   global init mask guarded by a critical section to ensure idempotence.
//! - Moving a field out of the `Gpio` value enforces unique ownership at
//!   compile time — attempts to use the same named field twice will be a
//!   compile-time error.
//!
//! If you prefer numeric-literal ergonomics (e.g., `acquire(5, ...)`), a small
//! macro wrapper can be added to expand numeric literals to the corresponding
//! `gpio.pN` field. This pattern follows the "macro bunker" approach used in
//! the example article: https://www.ecorax.net/macro-bunker-1/

use core::marker::PhantomData;
use core::ptr::write_volatile;
#[allow(unused_imports)]
use cortex_m::interrupt;
#[cfg(feature = "qemu")]
use cortex_m_semihosting::hprintln;

use crate::hal::Gpio as GpioTrait;

// NRF51 (micro:bit) peripheral addresses used by the C implementation
const GPIO_BASE: u32 = 0x5000_0000;
const GPIO_OUTSET: *mut u32 = (GPIO_BASE + 0x508) as *mut u32;
const GPIO_OUTCLR: *mut u32 = (GPIO_BASE + 0x50C) as *mut u32;
const GPIO_DIRSET: *mut u32 = (GPIO_BASE + 0x518) as *mut u32;
const GPIO_DIRCLR: *mut u32 = (GPIO_BASE + 0x51C) as *mut u32;
const GPIO_IN: *const u32 = (GPIO_BASE + 0x510) as *const u32;

// Note: we no longer track a global init mask; DIRSET is idempotent on nRF
// (writing the same bit repeatedly is harmless), so we do a direct write.

pub mod typestate {
    /// Pin typestates.
    pub struct NotConfigured;
    pub struct Output;
    pub struct Input;
}

use typestate::*;

/// Zero-sized pin representation parameterized by typestate and index.
pub struct Pin<MODE, const INDEX: u8> {
    _marker: PhantomData<MODE>
}

impl<MODE, const INDEX: u8> Pin<MODE, INDEX> {
    fn new() -> Self { Pin { _marker: PhantomData } }
}

impl<const INDEX: u8> Pin<NotConfigured, INDEX> {
    /// Convert this pin into an output pin, performing any required hardware init.
    ///
    /// `open_drain` is kept for API compatibility (ignored on micro:bit), and
    /// `initial_high` sets the initial output level.
    pub fn into_output(self, _open_drain: bool, initial_high: bool) -> Pin<Output, INDEX> {
        let bit = 1u32 << INDEX;
        // DIRSET is write-1-to-set and idempotent; perform the set unconditionally.
        unsafe { write_volatile(GPIO_DIRSET, bit); }
        unsafe {
            if initial_high { write_volatile(GPIO_OUTSET, bit); } else { write_volatile(GPIO_OUTCLR, bit); }
        }

        Pin::new()
    }
}

impl<const INDEX: u8> Pin<Output, INDEX> {
    /// Drive the pin high or low.
    pub fn write(&mut self, high: bool) {
        let bit = 1u32 << INDEX;
        unsafe {
            if high { write_volatile(GPIO_OUTSET, bit); } else { write_volatile(GPIO_OUTCLR, bit); }
        }
        #[cfg(feature = "qemu")] { let _ = hprintln!("GPIO pin {} = {}", INDEX, if high { 1 } else { 0 }); }
    }

    /// Read the pin level from the input register (works for output pins too).
    pub fn read(&mut self) -> bool {
        let bit = 1u32 << INDEX;
        unsafe { (core::ptr::read_volatile(GPIO_IN) & bit) != 0 }
    }
}

impl<const INDEX: u8> Pin<Input, INDEX> {
    /// Read the current input level for this pin.
    pub fn read(&mut self) -> bool {
        let bit = 1u32 << INDEX;
        unsafe { (core::ptr::read_volatile(GPIO_IN) & bit) != 0 }
    }
}

impl<const INDEX: u8> Pin<NotConfigured, INDEX> {
    /// Configure this pin as input (clears direction bit).
    pub fn into_input(self) -> Pin<Input, INDEX> {
        let bit = 1u32 << INDEX;
        unsafe { core::ptr::write_volatile(GPIO_DIRCLR, bit); }
        Pin::new()
    }
}

impl<const INDEX: u8> GpioTrait for Pin<Input, INDEX> {
    fn write(&mut self, _high: bool) { /* not applicable for input */ }
    fn read(&mut self) -> bool { self.read() }
}

impl<const INDEX: u8> GpioTrait for Pin<Output, INDEX> {
    fn write(&mut self, high: bool) { self.write(high); }
    fn read(&mut self) -> bool { self.read() }
}

// Macro-driven generation of the micro:bit pin collection and aliases.
// We only expose pins 0..=9 for the micro:bit board.
macro_rules! make_pins {
    ($($idx:expr),*) => {
        paste::paste! {
            /// Board GPIO collection. Move a field out to acquire that pin.
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

