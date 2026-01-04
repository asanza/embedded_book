#![allow(dead_code)]
use crate::hal::utils::Event;

pub trait Timer {
    /// Start the timer with a given period in microseconds and an event.
    fn start(&mut self, period_us: u32, event: Event);

    /// Stop the timer.
    fn stop(&mut self);

    /// Check if timer is running.
    fn is_running(&self) -> bool;
}
