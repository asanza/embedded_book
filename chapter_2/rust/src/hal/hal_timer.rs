#![allow(dead_code)]

pub trait Timer {
    /// Associate an event with the timer and enable its interrupt.
    fn enable_interrupt<E>(&mut self, event: E)
    where
        E: crate::hal::utils::Trigger;

    /// Start the timer with a given period in microseconds.
    fn start(&mut self, period_us: u32);

    /// Stop the timer.
    fn stop(&mut self);

    /// Check if timer is running.
    fn is_running(&self) -> bool;
}
