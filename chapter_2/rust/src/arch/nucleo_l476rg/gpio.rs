// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Diego Asanza <f.asanza@gmail.com>

use core::marker::PhantomData;
use core::ptr::{read_volatile, write_volatile};

use crate::hal::hal_gpio::Gpio as GpioTrait;
use crate::hal::hal_gpio::{ConfigurablePin, Edge, Pull};

// STM32L476 register map (GPIO) for Nucleo board.
const RCC_BASE: u32 = 0x4002_1000;
const RCC_AHB2ENR: *mut u32 = (RCC_BASE + 0x4C) as *mut u32;
const RCC_APB2ENR: *mut u32 = (RCC_BASE + 0x60) as *mut u32; // SYSCFG clock lives on APB2ENR (offset 0x60)

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

// IRQ handlers for EXTI lines. These are small wrappers that call the
// common handler which clears the pending bit and invokes a registered event.
#[no_mangle]
pub extern "C" fn EXTI0() {
    exti_common_handler(0);
}
#[no_mangle]
pub extern "C" fn EXTI1() {
    exti_common_handler(1);
}
#[no_mangle]
pub extern "C" fn EXTI2() {
    exti_common_handler(2);
}
#[no_mangle]
pub extern "C" fn EXTI3() {
    exti_common_handler(3);
}
#[no_mangle]
pub extern "C" fn EXTI4() {
    exti_common_handler(4);
}

#[no_mangle]
pub extern "C" fn EXTI9_5() {
    unsafe {
        let pr = read_volatile(EXTI_PR);
        let mask = pr & 0x0000_03E0; // bits 5..9
        if mask != 0 {
            for i in 5..=9 {
                if (mask & (1 << i)) != 0 {
                    exti_common_handler(i);
                }
            }
        }
    }
}

#[no_mangle]
pub extern "C" fn EXTI15_10() {
    unsafe {
        let pr = read_volatile(EXTI_PR);
        let mask = pr & 0x0000_FC00; // bits 10..15
        if mask != 0 {
            for i in 10..=15 {
                if (mask & (1 << i)) != 0 {
                    exti_common_handler(i);
                }
            }
        }
    }
}

fn exti_common_handler(line: u8) {
    let bit = 1u32 << line;
    unsafe {
        if (read_volatile(EXTI_PR) & bit) != 0 {
            // Clear pending (W1C) and invoke any registered event.
            write_volatile(EXTI_PR, bit);
            cortex_m::interrupt::free(|cs| {
                let idx = (line % 16) as usize;
                if let Some(ev) = EXTI_EVENTS[idx].borrow(cs).borrow().as_ref().cloned() {
                    ev.trigger();
                }
            });
        }
    }
}

use typestate::*;

use crate::hal::utils::Event;
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

// EXTI / SYSCFG registers (STM32L4 typical addresses)
const SYSCFG_BASE: u32 = 0x4001_0000;
const SYSCFG_EXTICR1: *mut u32 = (SYSCFG_BASE + 0x08) as *mut u32; // EXTICR1..EXTICR4 contiguous
const EXTI_BASE: u32 = 0x4001_0400;
const EXTI_IMR: *mut u32 = (EXTI_BASE + 0x00) as *mut u32;
const EXTI_RTSR: *mut u32 = (EXTI_BASE + 0x08) as *mut u32;
const EXTI_FTSR: *mut u32 = (EXTI_BASE + 0x0C) as *mut u32;
const EXTI_PR: *mut u32 = (EXTI_BASE + 0x14) as *mut u32;

// Registry for EXTI lines 0..15
static EXTI_EVENTS: [Mutex<RefCell<Option<Event>>>; 16] = [
    Mutex::new(RefCell::new(None)),
    Mutex::new(RefCell::new(None)),
    Mutex::new(RefCell::new(None)),
    Mutex::new(RefCell::new(None)),
    Mutex::new(RefCell::new(None)),
    Mutex::new(RefCell::new(None)),
    Mutex::new(RefCell::new(None)),
    Mutex::new(RefCell::new(None)),
    Mutex::new(RefCell::new(None)),
    Mutex::new(RefCell::new(None)),
    Mutex::new(RefCell::new(None)),
    Mutex::new(RefCell::new(None)),
    Mutex::new(RefCell::new(None)),
    Mutex::new(RefCell::new(None)),
    Mutex::new(RefCell::new(None)),
    Mutex::new(RefCell::new(None)),
];

// Helper to compute SYSCFG EXTICR index/shift for a pin
fn exticr_index_shift(pin: u8) -> (usize, u32) {
    let line = (pin % 16) as usize;
    let idx = line / 4; // EXTICR register index 0..3
    let shift = ((line % 4) * 4) as u32;
    (idx, shift)
}

// Map port index to SYSCFG exticr code: A => 0, B => 1, C => 2, ...
fn port_code_for_pin(pin: u8) -> u32 {
    let port = (pin / 16) as u32;
    port & 0xF
}

// Small GPIO helpers
#[inline(always)]
fn gpio_base(pin: u8) -> u32 {
    GPIO_BASES[(pin / 16) as usize]
}

#[inline(always)]
fn gpio_bit(pin: u8) -> u32 {
    1u32 << (pin % 16)
}

// NVIC helpers
const NVIC_ISER_BASE: u32 = 0xE000_E100;
fn nvic_enable(irqn: u32) {
    let idx = (irqn / 32) as usize;
    let bit = irqn % 32;
    unsafe {
        let iser = (NVIC_ISER_BASE as *mut u32).add(idx);
        core::ptr::write_volatile(iser, 1 << bit);
    }
}

// Edge enum is defined in crate::hal

pub struct Pin<MODE, const PIN: u8> {
    _marker: PhantomData<MODE>,
}

impl<MODE, const PIN: u8> Pin<MODE, PIN> {
    fn new() -> Self {
        Pin {
            _marker: PhantomData,
        }
    }
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

    /// Read the pin level from IDR (works for output pins too).
    pub fn read(&mut self) -> bool {
        let port = (PIN / 16) as usize;
        let bit = (PIN % 16) as u32;
        let mask = 1u32 << bit;
        let base = GPIO_BASES[port];
        let idr = (base + 0x10) as *const u32;
        unsafe { (core::ptr::read_volatile(idr) & mask) != 0 }
    }
}

impl<const PIN: u8> Pin<Input, PIN> {
    pub fn read(&mut self) -> bool {
        let port = (PIN / 16) as usize;
        let bit = (PIN % 16) as u32;
        let mask = 1u32 << bit;
        let base = GPIO_BASES[port];
        let idr = (base + 0x10) as *const u32;
        unsafe { (core::ptr::read_volatile(idr) & mask) != 0 }
    }

    pub fn enable_interrupt(&mut self, edge: Edge, ev: Event) {
        let pin = PIN;
        let (idx, shift) = exticr_index_shift(pin);
        let line = (pin % 16) as u32;
        let bit = 1u32 << line;

        unsafe {
            // enable SYSCFG clock so EXTICR writes take effect
            let mut apb2 = core::ptr::read_volatile(RCC_APB2ENR);
            apb2 |= 1 << 0; // SYSCFGEN
            core::ptr::write_volatile(RCC_APB2ENR, apb2);

            let exticr = (SYSCFG_EXTICR1 as *mut u32).add(idx);
            let mut v = core::ptr::read_volatile(exticr);
            v &= !(0xF << shift);
            let port_code = port_code_for_pin(pin);
            v |= (port_code & 0xF) << shift;
            core::ptr::write_volatile(exticr, v);

            // use precomputed line/bit
            // configure triggers
            match edge {
                Edge::Rising => {
                    core::ptr::write_volatile(EXTI_RTSR, core::ptr::read_volatile(EXTI_RTSR) | bit);
                    core::ptr::write_volatile(
                        EXTI_FTSR,
                        core::ptr::read_volatile(EXTI_FTSR) & !bit,
                    );
                }
                Edge::Falling => {
                    core::ptr::write_volatile(EXTI_FTSR, core::ptr::read_volatile(EXTI_FTSR) | bit);
                    core::ptr::write_volatile(
                        EXTI_RTSR,
                        core::ptr::read_volatile(EXTI_RTSR) & !bit,
                    );
                }
                Edge::Both => {
                    core::ptr::write_volatile(EXTI_RTSR, core::ptr::read_volatile(EXTI_RTSR) | bit);
                    core::ptr::write_volatile(EXTI_FTSR, core::ptr::read_volatile(EXTI_FTSR) | bit);
                }
            }

            // clear any previous pending and unmask
            core::ptr::write_volatile(EXTI_PR, bit);
            core::ptr::write_volatile(EXTI_IMR, core::ptr::read_volatile(EXTI_IMR) | bit);
        }

        cortex_m::interrupt::free(|cs| {
            EXTI_EVENTS[(PIN % 16) as usize]
                .borrow(cs)
                .replace(Some(ev));
        });

        // enable NVIC for the EXTI group covering this line
        let irq = if line <= 4 {
            6 + line // EXTI0..4 -> IRQ 6..10
        } else if line <= 9 {
            23 // EXTI9_5 -> IRQ 23
        } else {
            40 // EXTI15_10 -> IRQ 40
        };
        nvic_enable(irq);

        // Memory barrier and clear pending after NVIC enable
        unsafe {
            cortex_m::asm::dsb();
            cortex_m::asm::isb();
            core::ptr::write_volatile(EXTI_PR, bit);
        }
    }

    pub fn disable_interrupt(&mut self) {
        let pin = PIN;
        let line = (pin % 16) as u32;
        let bit = 1u32 << line;
        unsafe {
            core::ptr::write_volatile(EXTI_IMR, core::ptr::read_volatile(EXTI_IMR) & !bit);
            core::ptr::write_volatile(EXTI_PR, bit);
        }
        cortex_m::interrupt::free(|cs| {
            EXTI_EVENTS[(PIN % 16) as usize].borrow(cs).replace(None);
        });
    }
}

impl<const PIN: u8> Pin<NotConfigured, PIN> {
    // The actual constructors are provided via the `ConfigurablePin` trait
}

impl<const PIN: u8> ConfigurablePin for Pin<NotConfigured, PIN> {
    type Input = Pin<Input, PIN>;
    type Output = Pin<Output, PIN>;

    fn into_input(self, pull: Pull) -> Self::Input {
        let port = (PIN / 16) as usize;
        assert!(port < GPIO_BASES.len());
        let bit = (PIN % 16) as u32;
        let base = GPIO_BASES[port];
        let moder = (base + 0x00) as *mut u32;
        let pupdr = (base + 0x0C) as *mut u32;
        unsafe {
            // Enable GPIO clock
            let mut ahb = core::ptr::read_volatile(RCC_AHB2ENR);
            ahb |= 1 << port;
            core::ptr::write_volatile(RCC_AHB2ENR, ahb);

            // Clear mode bits to 00 (input)
            let shift = bit * 2;
            let mut m = core::ptr::read_volatile(moder);
            m &= !(0b11 << shift);
            core::ptr::write_volatile(moder, m);

            // configure pull-up/pull-down according to `pull`
            let pupdr_shift = bit * 2;
            let mut p = core::ptr::read_volatile(pupdr);
            p &= !(0b11 << pupdr_shift);
            match pull {
                Pull::None => { /* leave 00 */ }
                Pull::Up => p |= 0b01 << pupdr_shift,
                Pull::Down => p |= 0b10 << pupdr_shift,
                Pull::Both => p |= 0b11 << pupdr_shift,
            }
            core::ptr::write_volatile(pupdr, p);
        }

        Pin::new()
    }

    fn into_output(self, open_drain: bool, initial_high: bool) -> Self::Output {
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

impl<const PIN: u8> GpioTrait for Pin<Input, PIN> {
    fn write(&mut self, _high: bool) {}
    fn read(&mut self) -> bool {
        self.read()
    }
}

impl<const PIN: u8> GpioTrait for Pin<Output, PIN> {
    fn write(&mut self, high: bool) {
        self.write(high);
    }
    fn read(&mut self) -> bool {
        self.read()
    }
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

make_pins!(
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
    26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45
);
