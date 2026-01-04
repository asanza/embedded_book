#![allow(dead_code)]
use crate::hal::utils::Event;

/// Basic GPIO read/write trait.
pub trait Gpio {
    fn write(&mut self, high: bool);
    fn read(&mut self) -> bool;
}

/// Edge selection for GPIO interrupts.
pub enum Edge {
    Rising,
    Falling,
    Both,
}

/// Pull-up / Pull-down configuration for inputs.
pub enum Pull {
    None,
    Up,
    Down,
    Both,
}

/// Trait implemented by pins that can be configured from not-configured state
/// into input or output modes. Associated types let implementations return
/// their concrete typestate `Pin` types.
pub trait ConfigurablePin {
    type Input;
    type Output;

    fn into_input(self, pull: Pull) -> Self::Input;
    fn into_output(self, open_drain: bool, initial_high: bool) -> Self::Output;
}

/// Optional extension trait implemented by input pins that support interrupts.
pub trait InputInterrupt {
    fn enable_interrupt(&mut self, edge: Edge, ev: Event);
    fn disable_interrupt(&mut self);
}
