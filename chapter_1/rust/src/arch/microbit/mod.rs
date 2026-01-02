#![allow(dead_code)]

mod gpio;
mod timer;

pub use gpio::Gpio;
pub use timer::Timer;

type Handler = unsafe extern "C" fn();

// Minimal interrupt vector: 32 peripheral IRQs on nRF51; only TIMER0/1/2 are wired.
#[link_section = ".vector_table.interrupts"]
#[no_mangle]
pub static __INTERRUPTS: [Handler; 32] = [
    timer::default_handler, // 0 POWER_CLOCK
    timer::default_handler, // 1 RADIO
    timer::default_handler, // 2 UARTE0 / UART0
    timer::default_handler, // 3 SPI0_TWI0
    timer::default_handler, // 4 SPI1_TWI1
    timer::default_handler, // 5 Reserved
    timer::default_handler, // 6 GPIOTE
    timer::default_handler, // 7 ADC
    timer::TIMER0,          // 8 TIMER0
    timer::TIMER1,          // 9 TIMER1
    timer::TIMER2,          // 10 TIMER2
    timer::default_handler, // 11 RTC0
    timer::default_handler, // 12 TEMP
    timer::default_handler, // 13 RNG
    timer::default_handler, // 14 ECB
    timer::default_handler, // 15 CCM_AAR
    timer::default_handler, // 16 WDT
    timer::default_handler, // 17 RTC1
    timer::default_handler, // 18 QDEC
    timer::default_handler, // 19 LPCOMP
    timer::default_handler, // 20 SWI0
    timer::default_handler, // 21 SWI1
    timer::default_handler, // 22 SWI2
    timer::default_handler, // 23 SWI3
    timer::default_handler, // 24 SWI4
    timer::default_handler, // 25 SWI5
    timer::default_handler, // 26 Reserved
    timer::default_handler, // 27 Reserved
    timer::default_handler, // 28 Reserved
    timer::default_handler, // 29 Reserved
    timer::default_handler, // 30 Reserved
    timer::default_handler, // 31 Reserved
];

