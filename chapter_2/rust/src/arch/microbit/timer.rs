use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{AtomicBool, Ordering};
use core::marker::PhantomData;
use cortex_m::asm;

use crate::hal::Timer as TimerTrait;

// Timer peripheral and NVIC constants (from original file)
const TIMER_BASES: [u32; 3] = [0x4000_8000, 0x4000_9000, 0x4000_A000];
const TIMER_IRQS: [u32; 3] = [8, 9, 10];

const TASKS_START_OFFSET: u32 = 0x000;
const TASKS_STOP_OFFSET: u32 = 0x004;
const TASKS_CLEAR_OFFSET: u32 = 0x00C;
const EVENTS_COMPARE0_OFFSET: u32 = 0x140;
const INTENSET_OFFSET: u32 = 0x304;
const MODE_OFFSET: u32 = 0x504;
const BITMODE_OFFSET: u32 = 0x508;
const PRESCALER_OFFSET: u32 = 0x510;
const CC0_OFFSET: u32 = 0x540;

const TIMER_MODE_TIMER: u32 = 0;
const TIMER_BITMODE_32BIT: u32 = 3; // 32-bit counter
const TIMER_PRESCALER_256: u32 = 8; // divide 16 MHz by 2^8 ≈ 62.5 kHz
const TIMER_INTENSET_COMPARE0: u32 = 1 << 16; // enable compare0 interrupt

// NVIC register block (only what we need)
const NVIC_ISER0: *mut u32 = 0xE000_E100 as *mut u32;

static TIMER_FIRED: [AtomicBool; 3] = [
    AtomicBool::new(false),
    AtomicBool::new(false),
    AtomicBool::new(false),
];

static TIMER_ONE_SHOT: [AtomicBool; 3] = [
    AtomicBool::new(false),
    AtomicBool::new(false),
    AtomicBool::new(false),
];

pub mod typestate {
    pub struct NotConfigured;
    pub struct Running;
}

use typestate::*;

/// Zero-sized timer representation parameterized by typestate and index.
pub struct TimerPeripheral<MODE, const IDX: u8> {
    _marker: PhantomData<MODE>,
}

impl<MODE, const IDX: u8> TimerPeripheral<MODE, IDX> {
    const fn new() -> Self { Self { _marker: PhantomData } }
}

impl<const IDX: u8> TimerPeripheral<NotConfigured, IDX> {
    /// Configure the timer peripheral (enable IRQ and compare0). This mirrors
    /// previous runtime setup but is a typestate transition to `Running`.
    pub fn into_running(self, one_shot: bool) -> TimerPeripheral<Running, IDX> {
        let idx = IDX as usize;
        TIMER_ONE_SHOT[idx].store(one_shot, Ordering::Release);

        // enable compare0 interrupt
        unsafe {
            let inten = (TIMER_BASES[idx] + INTENSET_OFFSET) as *mut u32;
            let current = read_volatile(inten);
            write_volatile(inten, current | TIMER_INTENSET_COMPARE0);

            // enable NVIC
            let mask = 1u32 << TIMER_IRQS[idx];
            let val = read_volatile(NVIC_ISER0);
            write_volatile(NVIC_ISER0, val | mask);
        }

        TimerPeripheral::new()
    }
}

impl<const IDX: u8> TimerPeripheral<Running, IDX> {
    fn reg(&self, offset: u32) -> *mut u32 {
        (TIMER_BASES[IDX as usize] + offset) as *mut u32
    }

    fn arm_delay(&self, period: u32) {
        let fired = &TIMER_FIRED[IDX as usize];
        fired.store(false, Ordering::Release);

        unsafe {
            write_volatile(self.reg(TASKS_STOP_OFFSET), 1);
            write_volatile(self.reg(TASKS_CLEAR_OFFSET), 1);
            write_volatile(self.reg(MODE_OFFSET), TIMER_MODE_TIMER);
            write_volatile(self.reg(BITMODE_OFFSET), TIMER_BITMODE_32BIT);
            write_volatile(self.reg(PRESCALER_OFFSET), TIMER_PRESCALER_256);

            write_volatile(self.reg(CC0_OFFSET), period);
            write_volatile(self.reg(EVENTS_COMPARE0_OFFSET), 0);
            write_volatile(self.reg(TASKS_START_OFFSET), 1);
        }
    }

    fn fired_flag(&self) -> &AtomicBool { &TIMER_FIRED[IDX as usize] }
}

impl<const IDX: u8> TimerTrait for TimerPeripheral<Running, IDX> {
    fn delay_ms(&mut self, ms: u32) {
        let period = ms.saturating_mul(50);
        self.arm_delay(period);

        let fired = self.fired_flag();
        while !fired.load(Ordering::Acquire) {
            asm::wfi();
        }
    }
}

// Generate a small board-level `Timer` collection with fields `t0..t2`.
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

make_timers!(0,1,2);

#[inline(always)]
unsafe fn handle_timer_irq(idx: usize) {
    debug_assert!(idx < TIMER_BASES.len());

    let events = (TIMER_BASES[idx] + EVENTS_COMPARE0_OFFSET) as *mut u32;
    let stop = (TIMER_BASES[idx] + TASKS_STOP_OFFSET) as *mut u32;

    write_volatile(events, 0);

    if TIMER_ONE_SHOT[idx].load(Ordering::Relaxed) {
        write_volatile(stop, 1);
    }

    TIMER_FIRED[idx].store(true, Ordering::Release);
}

// Interrupt handlers for TIMER0/1/2 compare0
#[no_mangle]
pub extern "C" fn TIMER0() {
    unsafe { handle_timer_irq(0) }
}

#[no_mangle]
pub extern "C" fn TIMER1() {
    unsafe { handle_timer_irq(1) }
}

#[no_mangle]
pub extern "C" fn TIMER2() {
    unsafe { handle_timer_irq(2) }
}

#[no_mangle]
pub unsafe extern "C" fn default_handler() {
    loop {
        asm::bkpt();
    }
}
