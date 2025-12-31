#![allow(dead_code)]

use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{AtomicBool, Ordering};
use cortex_m::asm;

use crate::hal::{Gpio as GpioTrait, Timer as TimerTrait};
#[cfg(feature = "qemu")]
use cortex_m_semihosting::hprintln;

// NRF51 (micro:bit) peripheral addresses used by the C implementation
const GPIO_BASE: u32 = 0x5000_0000;
const GPIO_OUTSET: *mut u32 = (GPIO_BASE + 0x508) as *mut u32;
const GPIO_OUTCLR: *mut u32 = (GPIO_BASE + 0x50C) as *mut u32;
const GPIO_DIRSET: *mut u32 = (GPIO_BASE + 0x518) as *mut u32;

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
const TIMER_PRESCALER_256: u32 = 8;  // divide 16 MHz by 2^8 ≈ 62.5 kHz
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

// Minimal interrupt vector: 32 peripheral IRQs on nRF51; only TIMER0 is wired.
#[link_section = ".vector_table.interrupts"]
#[no_mangle]
pub static __INTERRUPTS: [Handler; 32] = [
    default_handler, // 0 POWER_CLOCK
    default_handler, // 1 RADIO
    default_handler, // 2 UARTE0 / UART0
    default_handler, // 3 SPI0_TWI0
    default_handler, // 4 SPI1_TWI1
    default_handler, // 5 Reserved
    default_handler, // 6 GPIOTE
    default_handler, // 7 ADC
    TIMER0,          // 8 TIMER0
    TIMER1,          // 9 TIMER1
    TIMER2,          // 10 TIMER2
    default_handler, // 11 RTC0
    default_handler, // 12 TEMP
    default_handler, // 13 RNG
    default_handler, // 14 ECB
    default_handler, // 15 CCM_AAR
    default_handler, // 16 WDT
    default_handler, // 17 RTC1
    default_handler, // 18 QDEC
    default_handler, // 19 LPCOMP
    default_handler, // 20 SWI0
    default_handler, // 21 SWI1
    default_handler, // 22 SWI2
    default_handler, // 23 SWI3
    default_handler, // 24 SWI4
    default_handler, // 25 SWI5
    default_handler, // 26 Reserved
    default_handler, // 27 Reserved
    default_handler, // 28 Reserved
    default_handler, // 29 Reserved
    default_handler, // 30 Reserved
    default_handler, // 31 Reserved
];

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
