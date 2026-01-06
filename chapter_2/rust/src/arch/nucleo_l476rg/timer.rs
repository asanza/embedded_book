// Event import not required in this module
use core::marker::PhantomData;
use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{AtomicBool, Ordering};
use cortex_m::asm;
use cortex_m::interrupt::Mutex;
use core::cell::RefCell;

// TIM2 peripheral (interrupt-driven delay)
const TIM2_BASE: u32 = 0x4000_0000;
const TIM2_CR1: *mut u32 = (TIM2_BASE + 0x00) as *mut u32;
const TIM2_DIER: *mut u32 = (TIM2_BASE + 0x0C) as *mut u32;
const TIM2_SR: *mut u32 = (TIM2_BASE + 0x10) as *mut u32;
const TIM2_EGR: *mut u32 = (TIM2_BASE + 0x14) as *mut u32;
const TIM2_CNT: *mut u32 = (TIM2_BASE + 0x24) as *mut u32;
const TIM2_PSC: *mut u32 = (TIM2_BASE + 0x28) as *mut u32;
const TIM2_ARR: *mut u32 = (TIM2_BASE + 0x2C) as *mut u32;

// TIM3 peripheral (same register layout offset)
const TIM3_BASE: u32 = 0x4000_0400;
const TIM3_CR1: *mut u32 = (TIM3_BASE + 0x00) as *mut u32;
const TIM3_DIER: *mut u32 = (TIM3_BASE + 0x0C) as *mut u32;
const TIM3_SR: *mut u32 = (TIM3_BASE + 0x10) as *mut u32;
const TIM3_EGR: *mut u32 = (TIM3_BASE + 0x14) as *mut u32;
const TIM3_CNT: *mut u32 = (TIM3_BASE + 0x24) as *mut u32;
const TIM3_PSC: *mut u32 = (TIM3_BASE + 0x28) as *mut u32;
const TIM3_ARR: *mut u32 = (TIM3_BASE + 0x2C) as *mut u32;

// TIM4 peripheral
const TIM4_BASE: u32 = 0x4000_0800;
const TIM4_CR1: *mut u32 = (TIM4_BASE + 0x00) as *mut u32;
const TIM4_DIER: *mut u32 = (TIM4_BASE + 0x0C) as *mut u32;
const TIM4_SR: *mut u32 = (TIM4_BASE + 0x10) as *mut u32;
const TIM4_EGR: *mut u32 = (TIM4_BASE + 0x14) as *mut u32;
const TIM4_CNT: *mut u32 = (TIM4_BASE + 0x24) as *mut u32;
const TIM4_PSC: *mut u32 = (TIM4_BASE + 0x28) as *mut u32;
const TIM4_ARR: *mut u32 = (TIM4_BASE + 0x2C) as *mut u32;

const TIM_CR1_CEN: u32 = 1 << 0;
const TIM_DIER_UIE: u32 = 1 << 0;
const TIM_SR_UIF: u32 = 1 << 0;
const TIM_EGR_UG: u32 = 1 << 0;

const NVIC_ISER0: *mut u32 = 0xE000_E100 as *mut u32;
const TIM2_IRQ_BIT: u32 = 1 << 28; // TIM2 IRQn = 28
const TIM3_IRQ_BIT: u32 = 1 << 29; // TIM3 IRQn = 29
const TIM4_IRQ_BIT: u32 = 1 << 30; // TIM4 IRQn = 30

// Assume default 4 MHz MSI unless startup changes it. Adjust if clocks change.
const TIMER_CLOCK_HZ: u32 = 4_000_000;

static TIM2_FIRED: AtomicBool = AtomicBool::new(false);
static TIM2_ONE_SHOT: AtomicBool = AtomicBool::new(false);
/* store the event mask (u32) so ISRs can signal without knowing concrete Event type */
static TIM2_EVENT_MASK: Mutex<RefCell<Option<u32>>> = Mutex::new(RefCell::new(None));
static TIM3_FIRED: AtomicBool = AtomicBool::new(false);
static TIM3_ONE_SHOT: AtomicBool = AtomicBool::new(false);
static TIM3_EVENT_MASK: Mutex<RefCell<Option<u32>>> = Mutex::new(RefCell::new(None));
static TIM4_FIRED: AtomicBool = AtomicBool::new(false);
static TIM4_ONE_SHOT: AtomicBool = AtomicBool::new(false);
static TIM4_EVENT_MASK: Mutex<RefCell<Option<u32>>> = Mutex::new(RefCell::new(None));

use crate::hal::hal_timer::Timer as TimerTrait;

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
    const fn new() -> Self {
        Self {
            _marker: PhantomData,
        }
    }
}

impl TimerPeripheral<NotConfigured, 2> {
    /// Initialize TIM2 as a periodic timer and enable its IRQ; transition to `Running`.
    pub fn into_periodic(self) -> TimerPeripheral<Running, 2> {
        TIM2_ONE_SHOT.store(false, Ordering::Release);
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

    /// Initialize TIM2 as a one-shot timer and enable its IRQ; transition to `Running`.
    pub fn into_oneshot(self) -> TimerPeripheral<Running, 2> {
        TIM2_ONE_SHOT.store(true, Ordering::Release);
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

impl TimerPeripheral<NotConfigured, 3> {
    /// Initialize TIM3 as periodic and enable its IRQ; transition to `Running`.
    pub fn into_periodic(self) -> TimerPeripheral<Running, 3> {
        TIM3_ONE_SHOT.store(false, Ordering::Release);
        unsafe {
            // Enable TIM3 clock on APB1 (TIM3EN is bit 1)
            let mut apb = read_volatile((0x4002_1000 + 0x58) as *mut u32);
            apb |= 1 << 1; // TIM3EN
            write_volatile((0x4002_1000 + 0x58) as *mut u32, apb);
            let _ = read_volatile((0x4002_1000 + 0x58) as *mut u32);

            // Reset TIM3 then release
            let mut rst = read_volatile((0x4002_1000 + 0x38) as *mut u32);
            rst |= 1 << 1; // TIM3RST
            write_volatile((0x4002_1000 + 0x38) as *mut u32, rst);
            rst &= !(1 << 1);
            write_volatile((0x4002_1000 + 0x38) as *mut u32, rst);

            // Enable TIM3 IRQ in NVIC
            let iser = read_volatile(NVIC_ISER0);
            write_volatile(NVIC_ISER0, iser | TIM3_IRQ_BIT);
        }
        TimerPeripheral::new()
    }

    /// Initialize TIM3 as one-shot and enable its IRQ; transition to `Running`.
    pub fn into_oneshot(self) -> TimerPeripheral<Running, 3> {
        TIM3_ONE_SHOT.store(true, Ordering::Release);
        unsafe {
            // Enable TIM3 clock on APB1 (TIM3EN is bit 1)
            let mut apb = read_volatile((0x4002_1000 + 0x58) as *mut u32);
            apb |= 1 << 1; // TIM3EN
            write_volatile((0x4002_1000 + 0x58) as *mut u32, apb);
            let _ = read_volatile((0x4002_1000 + 0x58) as *mut u32);

            // Reset TIM3 then release
            let mut rst = read_volatile((0x4002_1000 + 0x38) as *mut u32);
            rst |= 1 << 1; // TIM3RST
            write_volatile((0x4002_1000 + 0x38) as *mut u32, rst);
            rst &= !(1 << 1);
            write_volatile((0x4002_1000 + 0x38) as *mut u32, rst);

            // Enable TIM3 IRQ in NVIC
            let iser = read_volatile(NVIC_ISER0);
            write_volatile(NVIC_ISER0, iser | TIM3_IRQ_BIT);
        }
        TimerPeripheral::new()
    }
}

impl TimerPeripheral<NotConfigured, 4> {
    /// Initialize TIM4 as periodic and enable its IRQ; transition to `Running`.
    pub fn into_periodic(self) -> TimerPeripheral<Running, 4> {
        TIM4_ONE_SHOT.store(false, Ordering::Release);
        unsafe {
            // Enable TIM4 clock on APB1 (TIM4EN is bit 2)
            let mut apb = read_volatile((0x4002_1000 + 0x58) as *mut u32);
            apb |= 1 << 2; // TIM4EN
            write_volatile((0x4002_1000 + 0x58) as *mut u32, apb);
            let _ = read_volatile((0x4002_1000 + 0x58) as *mut u32);

            // Reset TIM4 then release
            let mut rst = read_volatile((0x4002_1000 + 0x38) as *mut u32);
            rst |= 1 << 2; // TIM4RST
            write_volatile((0x4002_1000 + 0x38) as *mut u32, rst);
            rst &= !(1 << 2);
            write_volatile((0x4002_1000 + 0x38) as *mut u32, rst);

            // Enable TIM4 IRQ in NVIC
            let iser = read_volatile(NVIC_ISER0);
            write_volatile(NVIC_ISER0, iser | TIM4_IRQ_BIT);
        }
        TimerPeripheral::new()
    }

    /// Initialize TIM4 as one-shot and enable its IRQ; transition to `Running`.
    pub fn into_oneshot(self) -> TimerPeripheral<Running, 4> {
        TIM4_ONE_SHOT.store(true, Ordering::Release);
        unsafe {
            // Enable TIM4 clock on APB1 (TIM4EN is bit 2)
            let mut apb = read_volatile((0x4002_1000 + 0x58) as *mut u32);
            apb |= 1 << 2; // TIM4EN
            write_volatile((0x4002_1000 + 0x58) as *mut u32, apb);
            let _ = read_volatile((0x4002_1000 + 0x58) as *mut u32);

            // Reset TIM4 then release
            let mut rst = read_volatile((0x4002_1000 + 0x38) as *mut u32);
            rst |= 1 << 2; // TIM4RST
            write_volatile((0x4002_1000 + 0x38) as *mut u32, rst);
            rst &= !(1 << 2);
            write_volatile((0x4002_1000 + 0x38) as *mut u32, rst);

            // Enable TIM4 IRQ in NVIC
            let iser = read_volatile(NVIC_ISER0);
            write_volatile(NVIC_ISER0, iser | TIM4_IRQ_BIT);
        }
        TimerPeripheral::new()
    }
}

impl TimerPeripheral<Running, 2> {
    fn arm_delay(&self, us: u32) {
        unsafe {
            TIM2_FIRED.store(false, Ordering::Release);

            // Configure TIM2 for 1 MHz tick (1 us per tick) and desired period
            let prescaler = (TIMER_CLOCK_HZ / 1_000_000).saturating_sub(1);
            write_volatile(TIM2_CR1, 0);
            write_volatile(TIM2_DIER, 0);
            write_volatile(TIM2_SR, 0);
            write_volatile(TIM2_PSC, prescaler);

            // ARR is the number of ticks (in microseconds)
            let arr = if us == 0 { 1 } else { us };
            write_volatile(TIM2_ARR, arr);
            write_volatile(TIM2_CNT, 0);
            write_volatile(TIM2_EGR, TIM_EGR_UG); // load prescaler immediately (sets UIF)
            write_volatile(TIM2_SR, 0); // clear the UIF raised by UG so we don't fire instantly

            write_volatile(TIM2_DIER, TIM_DIER_UIE);
            cortex_m::asm::dsb();
            cortex_m::asm::isb();
            write_volatile(TIM2_DIER, TIM_DIER_UIE);
            write_volatile(TIM2_CR1, TIM_CR1_CEN);
        }
    }

    fn fired_flag(&self) -> &AtomicBool {
        &TIM2_FIRED
    }
}

impl TimerPeripheral<Running, 3> {
    fn arm_delay(&self, us: u32) {
        unsafe {
            TIM3_FIRED.store(false, Ordering::Release);

            let prescaler = (TIMER_CLOCK_HZ / 1_000_000).saturating_sub(1);
            write_volatile(TIM3_CR1, 0);
            write_volatile(TIM3_DIER, 0);
            write_volatile(TIM3_SR, 0);
            write_volatile(TIM3_PSC, prescaler);

            let arr = if us == 0 { 1 } else { us };
            write_volatile(TIM3_ARR, arr);
            write_volatile(TIM3_CNT, 0);
            write_volatile(TIM3_EGR, TIM_EGR_UG);
            write_volatile(TIM3_SR, 0);

            write_volatile(TIM3_DIER, TIM_DIER_UIE);
            cortex_m::asm::dsb();
            cortex_m::asm::isb();
            write_volatile(TIM3_DIER, TIM_DIER_UIE);
            write_volatile(TIM3_CR1, TIM_CR1_CEN);
        }
    }

    fn fired_flag(&self) -> &AtomicBool {
        &TIM3_FIRED
    }
}

impl TimerPeripheral<Running, 4> {
    fn arm_delay(&self, us: u32) {
        assert!(us <= 65535);
        unsafe {
            TIM4_FIRED.store(false, Ordering::Release);

            let prescaler = (TIMER_CLOCK_HZ / 1_000_000).saturating_sub(1);
            write_volatile(TIM4_CR1, 0);
            write_volatile(TIM4_DIER, 0);
            write_volatile(TIM4_SR, 0);
            write_volatile(TIM4_PSC, prescaler);

            let arr = if us == 0 { 1 } else { us };
            write_volatile(TIM4_ARR, arr);
            write_volatile(TIM4_CNT, 0);
            write_volatile(TIM4_EGR, TIM_EGR_UG);
            write_volatile(TIM4_SR, 0);

            write_volatile(TIM4_DIER, TIM_DIER_UIE);
            cortex_m::asm::dsb();
            cortex_m::asm::isb();
            write_volatile(TIM4_DIER, TIM_DIER_UIE);
            write_volatile(TIM4_CR1, TIM_CR1_CEN);
        }
    }

    fn fired_flag(&self) -> &AtomicBool {
        &TIM4_FIRED
    }
}

impl TimerTrait for TimerPeripheral<Running, 2> {
    fn enable_interrupt<E>(&mut self, event: E)
    where
        E: crate::hal::utils::Trigger,
    {
        let mask = event.mask();
        cortex_m::interrupt::free(|cs| *TIM2_EVENT_MASK.borrow(cs).borrow_mut() = Some(mask));
    }

    fn start(&mut self, us: u32) {
        // configure hardware timer to desired frequency (omitted)
        self.arm_delay(us);
    }

    fn stop(&mut self) {
        unsafe {
            write_volatile(TIM2_CR1, read_volatile(TIM2_CR1) & !TIM_CR1_CEN);
            write_volatile(TIM2_DIER, 0);
        }
        // Keep the registered event mask so start() can re-enable interrupts
        // later if needed. Do not clear TIM2_EVENT_MASK here.
    }

    fn is_running(&self) -> bool {
        // crude: running if counter enabled
        unsafe { read_volatile(TIM2_CR1) & TIM_CR1_CEN != 0 }
    }
}

impl TimerTrait for TimerPeripheral<Running, 3> {
    fn enable_interrupt<E>(&mut self, event: E)
    where
        E: crate::hal::utils::Trigger,
    {
        let mask = event.mask();
        cortex_m::interrupt::free(|cs| *TIM3_EVENT_MASK.borrow(cs).borrow_mut() = Some(mask));
    }

    fn start(&mut self, us: u32) {
        self.arm_delay(us);
    }

    fn stop(&mut self) {
        unsafe {
            write_volatile(TIM3_CR1, read_volatile(TIM3_CR1) & !TIM_CR1_CEN);
            write_volatile(TIM3_DIER, 0);
        }
        // Keep the registered event mask so start() can re-enable interrupts
        // later if needed. Do not clear TIM3_EVENT_MASK here.
    }

    fn is_running(&self) -> bool {
        unsafe { read_volatile(TIM3_CR1) & TIM_CR1_CEN != 0 }
    }
}

impl TimerTrait for TimerPeripheral<Running, 4> {
    fn enable_interrupt<E>(&mut self, event: E)
    where
        E: crate::hal::utils::Trigger,
    {
        let mask = event.mask();
        cortex_m::interrupt::free(|cs| *TIM4_EVENT_MASK.borrow(cs).borrow_mut() = Some(mask));
    }

    fn start(&mut self, us: u32) {
        self.arm_delay(us);
    }

    fn stop(&mut self) {
        unsafe {
            write_volatile(TIM4_CR1, read_volatile(TIM4_CR1) & !TIM_CR1_CEN);
            write_volatile(TIM4_DIER, 0);
        }
        // Keep the registered event mask so start() can re-enable interrupts
        // later if needed. Do not clear TIM4_EVENT_MASK here.
    }

    fn is_running(&self) -> bool {
        unsafe { read_volatile(TIM4_CR1) & TIM_CR1_CEN != 0 }
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

make_timers!(2, 3, 4);

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
    // trigger event if registered
    cortex_m::interrupt::free(|cs| {
        if let Some(mask) = *TIM2_EVENT_MASK.borrow(cs).borrow() {
            crate::arch::events::signal_mask(mask);
        }
    });
}

#[no_mangle]
pub extern "C" fn TIM3() {
    unsafe {
        if read_volatile(TIM3_SR) & TIM_SR_UIF != 0 {
            write_volatile(TIM3_SR, read_volatile(TIM3_SR) & !TIM_SR_UIF);
            if TIM3_ONE_SHOT.load(Ordering::Relaxed) {
                write_volatile(TIM3_CR1, read_volatile(TIM3_CR1) & !TIM_CR1_CEN);
            }
        }
    }
    TIM3_FIRED.store(true, Ordering::Release);
    cortex_m::interrupt::free(|cs| {
        if let Some(mask) = *TIM3_EVENT_MASK.borrow(cs).borrow() {
            crate::arch::events::signal_mask(mask);
        }
    });
}

#[no_mangle]
pub extern "C" fn TIM4() {
    unsafe {
        if read_volatile(TIM4_SR) & TIM_SR_UIF != 0 {
            write_volatile(TIM4_SR, read_volatile(TIM4_SR) & !TIM_SR_UIF);
            if TIM4_ONE_SHOT.load(Ordering::Relaxed) {
                write_volatile(TIM4_CR1, read_volatile(TIM4_CR1) & !TIM_CR1_CEN);
            }
        }
    }
    TIM4_FIRED.store(true, Ordering::Release);
    cortex_m::interrupt::free(|cs| {
        if let Some(mask) = *TIM4_EVENT_MASK.borrow(cs).borrow() {
            crate::arch::events::signal_mask(mask);
        }
    });
}

#[no_mangle]
pub unsafe extern "C" fn default_handler() {
    loop {
        asm::bkpt();
    }
}
