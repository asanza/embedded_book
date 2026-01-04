#![allow(dead_code)]

mod gpio;
mod timer;

pub use gpio::Gpio;
pub use timer::Timer;

type Handler = unsafe extern "C" fn();

// Interrupts: vector table for STM32L476 (only a subset wired here).
// Keep the vector entries referring to functions implemented in `timer` or
// aliased to `default_handler` in `timer`.
#[cfg(feature = "nucleo")]
#[link_section = ".vector_table.interrupts"]
#[no_mangle]
pub static __INTERRUPTS: [Handler; 64] = [
    timer::default_handler, // 0 WWDG
    timer::default_handler, // 1 PVD/PVM
    timer::default_handler, // 2 RTC_TAMP_STAMP
    timer::default_handler, // 3 RTC_WKUP
    timer::default_handler, // 4 FLASH
    timer::default_handler, // 5 RCC
    timer::default_handler, // 6 EXTI0
    timer::default_handler, // 7 EXTI1
    timer::default_handler, // 8 EXTI2
    timer::default_handler, // 9 EXTI3
    timer::default_handler, // 10 EXTI4
    timer::default_handler, // 11 DMA1_CH1
    timer::default_handler, // 12 DMA1_CH2
    timer::default_handler, // 13 DMA1_CH3
    timer::default_handler, // 14 DMA1_CH4
    timer::default_handler, // 15 DMA1_CH5
    timer::default_handler, // 16 DMA1_CH6
    timer::default_handler, // 17 DMA1_CH7
    timer::default_handler, // 18 ADC1_2
    timer::default_handler, // 19 CAN1_TX
    timer::default_handler, // 20 CAN1_RX0
    timer::default_handler, // 21 CAN1_RX1
    timer::default_handler, // 22 CAN1_SCE
    timer::default_handler, // 23 EXTI9_5
    timer::default_handler, // 24 TIM1_BRK_TIM15
    timer::default_handler, // 25 TIM1_UP_TIM16
    timer::default_handler, // 26 TIM1_TRG_COM_TIM17
    timer::default_handler, // 27 TIM1_CC
    timer::TIM2,            // 28 TIM2
    timer::TIM3,            // 29 TIM3
    timer::TIM4,            // 30 TIM4
    timer::default_handler, // 31 I2C1_EV
    timer::default_handler, // 32 I2C1_ER
    timer::default_handler, // 33 I2C2_EV
    timer::default_handler, // 34 I2C2_ER
    timer::default_handler, // 35 SPI1
    timer::default_handler, // 36 SPI2
    timer::default_handler, // 37 USART1
    timer::default_handler, // 38 USART2
    timer::default_handler, // 39 USART3
    timer::default_handler, // 40 EXTI15_10
    timer::default_handler, // 41 RTC_Alarm
    timer::default_handler, // 42 DFSDM1_FLT3
    timer::default_handler, // 43 TIM8_BRK
    timer::default_handler, // 44 TIM8_UP
    timer::default_handler, // 45 TIM8_TRG_COM
    timer::default_handler, // 46 TIM8_CC
    timer::default_handler, // 47 ADC3
    timer::default_handler, // 48 FMC
    timer::default_handler, // 49 SDMMC1
    timer::default_handler, // 50 TIM5
    timer::default_handler, // 51 SPI3
    timer::default_handler, // 52 UART4
    timer::default_handler, // 53 UART5
    timer::default_handler, // 54 TIM6_DAC
    timer::default_handler, // 55 TIM7
    timer::default_handler, // 56 DMA2_Channel1
    timer::default_handler, // 57 DMA2_Channel2
    timer::default_handler, // 58 DMA2_Channel3
    timer::default_handler, // 59 DMA2_Channel4
    timer::default_handler, // 60 DMA2_Channel5
    timer::default_handler, // 61 DFSDM1_FLT0
    timer::default_handler, // 62 DFSDM1_FLT1
    timer::default_handler, // 63 DFSDM1_FLT2
];
