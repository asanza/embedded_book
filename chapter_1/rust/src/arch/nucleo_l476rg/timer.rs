use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{AtomicBool, Ordering};
use core::marker::PhantomData;
use cortex_m::asm;

// TIM2 peripheral (interrupt-driven delay)
const TIM2_BASE: u32 = 0x4000_0000;
const TIM2_CR1: *mut u32 = (TIM2_BASE + 0x00) as *mut u32;
const TIM2_DIER: *mut u32 = (TIM2_BASE + 0x0C) as *mut u32;
const TIM2_SR: *mut u32 = (TIM2_BASE + 0x10) as *mut u32;
const TIM2_EGR: *mut u32 = (TIM2_BASE + 0x14) as *mut u32;
const TIM2_CNT: *mut u32 = (TIM2_BASE + 0x24) as *mut u32;
const TIM2_PSC: *mut u32 = (TIM2_BASE + 0x28) as *mut u32;
const TIM2_ARR: *mut u32 = (TIM2_BASE + 0x2C) as *mut u32;

const TIM_CR1_CEN: u32 = 1 << 0;
const TIM_DIER_UIE: u32 = 1 << 0;
const TIM_SR_UIF: u32 = 1 << 0;
const TIM_EGR_UG: u32 = 1 << 0;

const NVIC_ISER0: *mut u32 = 0xE000_E100 as *mut u32;
const TIM2_IRQ_BIT: u32 = 1 << 28; // TIM2 IRQn = 28

// Assume default 4 MHz MSI unless startup changes it. Adjust if clocks change.
const TIMER_CLOCK_HZ: u32 = 4_000_000;

static TIM2_FIRED: AtomicBool = AtomicBool::new(false);
static TIM2_ONE_SHOT: AtomicBool = AtomicBool::new(false);

use crate::hal::Timer as TimerTrait;

pub mod typestate {
    pub struct NotConfigured;
    pub struct Running;
}

use typestate::*;

/// Zero-sized timer representation parameterized by typestate.
pub struct TimerPeripheral<MODE, const IDX: u8> {
    _marker: PhantomData<MODE>,
}

impl<MODE, const IDX: u8> TimerPeripheral<MODE, IDX> {
    const fn new() -> Self { Self { _marker: PhantomData } }
}

impl TimerPeripheral<NotConfigured, 0> {
    /// Initialize TIM2 and enable its IRQ; transition to `Running`.
    pub fn into_running(self, one_shot: bool) -> TimerPeripheral<Running, 0> {
        TIM2_ONE_SHOT.store(one_shot, Ordering::Release);

        unsafe {
            // Enable TIM2 clock on APB1
            let mut apb = read_volatile((0x4002_1000 + 0x58) as *mut u32);
            apb |= 1 << 0; // TIM2EN
            write_volatile((0x4002_1000 + 0x58) as *mut u32, apb);
            let _ = read_volatile((0x4002_1000 + 0x58) as *mut u32); // barrier

            // Reset TIM2 to a known state, then release reset.
            let mut rst = read_volatile((0x4002_1000 + 0x38) as *mut u32);
            rst |= 1 << 0; // TIM2RST
            write_volatile((0x4002_1000 + 0x38) as *mut u32, rst);
            rst &= !(1 << 0);
            write_volatile((0x4002_1000 + 0x38) as *mut u32, rst);

            // Enable TIM2 interrupt in NVIC
            let iser = read_volatile(NVIC_ISER0);
            write_volatile(NVIC_ISER0, iser | TIM2_IRQ_BIT);
        }

        TimerPeripheral::new()
    }
}

impl TimerPeripheral<Running, 0> {
    fn arm_delay(&self, ms: u32) {
        unsafe {
            TIM2_FIRED.store(false, Ordering::Release);

            // Configure TIM2 for 1 kHz tick and desired period
            let prescaler = (TIMER_CLOCK_HZ / 1_000).saturating_sub(1);
            write_volatile(TIM2_CR1, 0);
            write_volatile(TIM2_DIER, 0);
            write_volatile(TIM2_SR, 0);
            write_volatile(TIM2_PSC, prescaler);
            let arr = if ms == 0 { 1 } else { ms };
            write_volatile(TIM2_ARR, arr);
            write_volatile(TIM2_CNT, 0);
            write_volatile(TIM2_EGR, TIM_EGR_UG); // load prescaler immediately (sets UIF)
            write_volatile(TIM2_SR, 0);           // clear the UIF raised by UG so we don't fire instantly

            write_volatile(TIM2_DIER, TIM_DIER_UIE);
            write_volatile(TIM2_CR1, TIM_CR1_CEN);
        }
    }

    fn fired_flag(&self) -> &AtomicBool {
        &TIM2_FIRED
    }
}

impl TimerTrait for TimerPeripheral<Running, 0> {
    fn delay_ms(&mut self, ms: u32) {
        self.arm_delay(ms);
        while !self.fired_flag().load(Ordering::Acquire) {
            asm::wfi();
        }
    }
}

// Generate a simple Timer collection with t0 field.
macro_rules! make_timers {
    ($($idx:expr),*) => {
        paste::paste! {
            pub struct Timer {
                $( pub [<t $idx>]: TimerPeripheral<NotConfigured, $idx>, )*
            }

            impl Timer {
                pub fn new() -> Self {
                    Self { $( [<t $idx>]: TimerPeripheral::new(), )* }
                }
            }
        }
    }
}

make_timers!(0);

#[no_mangle]
pub extern "C" fn TIM2() {
    unsafe {
        if read_volatile(TIM2_SR) & TIM_SR_UIF != 0 {
            write_volatile(TIM2_SR, read_volatile(TIM2_SR) & !TIM_SR_UIF);
            if TIM2_ONE_SHOT.load(Ordering::Relaxed) {
                write_volatile(TIM2_CR1, read_volatile(TIM2_CR1) & !TIM_CR1_CEN);
            }
        }
    }
    TIM2_FIRED.store(true, Ordering::Release);
}

#[no_mangle]
pub unsafe extern "C" fn default_handler() {
    loop {
        asm::bkpt();
    }
}
