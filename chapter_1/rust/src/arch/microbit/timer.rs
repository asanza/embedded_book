use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{AtomicBool, Ordering};
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

type Handler = unsafe extern "C" fn();

pub struct Timer {
    peripheral: TimerPeripheral,
}

impl Timer {
    pub fn new(instance: u8, one_shot: bool) -> Self {
        let idx = instance as usize;
        assert!(idx < TIMER_BASES.len());

        TIMER_ONE_SHOT[idx].store(one_shot, Ordering::Release);

        let peripheral = TimerPeripheral::new(idx);
        peripheral.enable_compare0_interrupt();
        peripheral.enable_irq();

        Self { peripheral }
    }
}

impl TimerTrait for Timer {
    fn delay_ms(&mut self, ms: u32) {
        let period = ms.saturating_mul(50);
        self.peripheral.arm_delay(period);

        let fired = self.peripheral.fired_flag();
        while !fired.load(Ordering::Acquire) {
            asm::wfi();
        }
    }
}

struct TimerPeripheral {
    idx: usize,
}

impl TimerPeripheral {
    const fn new(idx: usize) -> Self {
        Self { idx }
    }

    fn reg(&self, offset: u32) -> *mut u32 {
        (TIMER_BASES[self.idx] + offset) as *mut u32
    }

    fn enable_irq(&self) {
        unsafe {
            let mask = 1u32 << TIMER_IRQS[self.idx];
            let val = read_volatile(NVIC_ISER0);
            write_volatile(NVIC_ISER0, val | mask);
        }
    }

    fn enable_compare0_interrupt(&self) {
        unsafe {
            let inten = self.reg(INTENSET_OFFSET);
            let current = read_volatile(inten);
            write_volatile(inten, current | TIMER_INTENSET_COMPARE0);
        }
    }

    fn arm_delay(&self, period: u32) {
        let fired = self.fired_flag();
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

    fn fired_flag(&self) -> &AtomicBool {
        &TIMER_FIRED[self.idx]
    }
}

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
