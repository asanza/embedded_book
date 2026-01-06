#[cfg(feature = "qemu")]
pub mod microbit;

pub mod events;

#[cfg(feature = "nucleo")]
pub mod nucleo_l476rg;

#[cfg(feature = "qemu")]
pub use microbit::{Gpio as GpioImpl, Timer as TimerImpl};
// microbit exposes typed `Gpio` with per-pin fields; no `Pins` owner exported.

#[cfg(feature = "nucleo")]
pub use nucleo_l476rg::{Gpio as GpioImpl, Timer as TimerImpl};
