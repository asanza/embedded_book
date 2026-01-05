#![allow(dead_code)]
use crate::hal::utils::Trigger;

pub trait Timer {
    /// Start the timer with a given period in microseconds and an event.
    fn start<E>(&mut self, period_us: u32, event: E)
    where
        E: Trigger + Copy;

    /// Stop the timer.
    fn stop(&mut self);

    /// Check if timer is running.
    fn is_running(&self) -> bool;
}
