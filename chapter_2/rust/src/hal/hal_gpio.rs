#![allow(dead_code)]

/// Basic GPIO read/write trait
pub trait Gpio {
    fn write(&mut self, high: bool);
    fn read(&mut self) -> bool;
}

/// Edge selection for GPIO interrupts
pub enum Edge {
    Rising,
    Falling,
    Both,
}

/// Pull-up / Pull-down configuration for inputs
pub enum Pull {
    None,
    Up,
    Down,
    Both,
}

/// Trait for pins that can be configured into input or output modes
pub trait ConfigurablePin {
    type Input;
    type Output;

    fn into_input(self, pull: Pull) -> Self::Input;
    fn into_output(self, open_drain: bool, initial_high: bool) -> Self::Output;
}

/// Extension trait for input pins that support interrupts
pub trait InputInterrupt {
    fn enable_interrupt<E>(&mut self, edge: Edge, event: E)
    where
        E: crate::hal::utils::Trigger;

    fn disable_interrupt(&mut self);
}
