#![allow(dead_code)]

use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{AtomicBool, Ordering};
use cortex_m::asm;

use crate::hal::{Gpio as GpioTrait, Timer as TimerTrait};

// STM32L476 register map (GPIO + TIM2) for Nucleo board.
const RCC_BASE: u32 = 0x4002_1000;
const RCC_AHB2ENR: *mut u32 = (RCC_BASE + 0x4C) as *mut u32;
const RCC_APB1ENR1: *mut u32 = (RCC_BASE + 0x58) as *mut u32;
const RCC_APB1RSTR1: *mut u32 = (RCC_BASE + 0x38) as *mut u32;

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

type Handler = unsafe extern "C" fn();

#[cfg(feature = "nucleo")]
#[link_section = ".vector_table.interrupts"]
#[no_mangle]
pub static __INTERRUPTS: [Handler; 64] = [
    default_handler, // 0 WWDG
    default_handler, // 1 PVD/PVM
    default_handler, // 2 RTC_TAMP_STAMP
    default_handler, // 3 RTC_WKUP
    default_handler, // 4 FLASH
    default_handler, // 5 RCC
    default_handler, // 6 EXTI0
    default_handler, // 7 EXTI1
    default_handler, // 8 EXTI2
    default_handler, // 9 EXTI3
    default_handler, // 10 EXTI4
    default_handler, // 11 DMA1_CH1
    default_handler, // 12 DMA1_CH2
    default_handler, // 13 DMA1_CH3
    default_handler, // 14 DMA1_CH4
    default_handler, // 15 DMA1_CH5
    default_handler, // 16 DMA1_CH6
    default_handler, // 17 DMA1_CH7
    default_handler, // 18 ADC1_2
    default_handler, // 19 CAN1_TX
    default_handler, // 20 CAN1_RX0
    default_handler, // 21 CAN1_RX1
    default_handler, // 22 CAN1_SCE
    default_handler, // 23 EXTI9_5
    default_handler, // 24 TIM1_BRK_TIM15
    default_handler, // 25 TIM1_UP_TIM16
    default_handler, // 26 TIM1_TRG_COM_TIM17
    default_handler, // 27 TIM1_CC
    TIM2,            // 28 TIM2
    default_handler, // 29 TIM3
    default_handler, // 30 TIM4
    default_handler, // 31 I2C1_EV
    default_handler, // 32 I2C1_ER
    default_handler, // 33 I2C2_EV
    default_handler, // 34 I2C2_ER
    default_handler, // 35 SPI1
    default_handler, // 36 SPI2
    default_handler, // 37 USART1
    default_handler, // 38 USART2
    default_handler, // 39 USART3
    default_handler, // 40 EXTI15_10
    default_handler, // 41 RTC_ALARM
    default_handler, // 42 DFSDM1_FLT3
    default_handler, // 43 TIM8_BRK
    default_handler, // 44 TIM8_UP
    default_handler, // 45 TIM8_TRG_COM
    default_handler, // 46 TIM8_CC
    default_handler, // 47 ADC3
    default_handler, // 48 FMC
    default_handler, // 49 SDMMC1
    default_handler, // 50 TIM5
    default_handler, // 51 SPI3
    default_handler, // 52 UART4
    default_handler, // 53 UART5
    default_handler, // 54 TIM6_DACUNDER
    default_handler, // 55 TIM7
    default_handler, // 56 DMA2_CH1
    default_handler, // 57 DMA2_CH2
    default_handler, // 58 DMA2_CH3
    default_handler, // 59 DMA2_CH4
    default_handler, // 60 DMA2_CH5
    default_handler, // 61 DFSDM1_FLT0
    default_handler, // 62 DFSDM1_FLT1
    default_handler, // 63 DFSDM1_FLT2
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

pub struct Timer {
    instance: u8,
}

impl Timer {
    pub fn new(instance: u8, one_shot: bool) -> Self {
        assert!(instance == 0);

        TIM2_ONE_SHOT.store(one_shot, Ordering::Release);

        unsafe {
            // Enable TIM2 clock on APB1
            let mut apb = read_volatile(RCC_APB1ENR1);
            apb |= 1 << 0; // TIM2EN
            write_volatile(RCC_APB1ENR1, apb);
            let _ = read_volatile(RCC_APB1ENR1); // simple barrier so subsequent TIM writes stick

            // Reset TIM2 to a known state, then release reset.
            let mut rst = read_volatile(RCC_APB1RSTR1);
            rst |= 1 << 0; // TIM2RST
            write_volatile(RCC_APB1RSTR1, rst);
            rst &= !(1 << 0);
            write_volatile(RCC_APB1RSTR1, rst);

            // Enable TIM2 interrupt in NVIC
            let iser = read_volatile(NVIC_ISER0);
            write_volatile(NVIC_ISER0, iser | TIM2_IRQ_BIT);
        }
        Self { instance }
    }
}

impl TimerTrait for Timer {
    fn delay_ms(&mut self, ms: u32) {
        let _ = self.instance; // document timer selection even though only instance 0 is valid
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

            while !TIM2_FIRED.load(Ordering::Acquire) {
                asm::wfi();
            }
        }
    }
}

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
