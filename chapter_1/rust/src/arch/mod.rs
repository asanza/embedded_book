#[cfg(feature = "qemu")]
pub mod microbit;

#[cfg(feature = "nucleo")]
pub mod nucleo_l476rg;

#[cfg(feature = "qemu")]
pub use microbit::{Gpio as GpioImpl, Timer as TimerImpl};

#[cfg(feature = "nucleo")]
pub use nucleo_l476rg::{Gpio as GpioImpl, Timer as TimerImpl};
