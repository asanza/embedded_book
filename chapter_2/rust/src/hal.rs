#![allow(dead_code)]
use crate::event::Event;

pub trait Gpio {
    fn write(&mut self, high: bool);
    fn read(&mut self) -> bool;
}

pub trait Timer {
    /// Start the timer with a given frequency and event
    fn start(&mut self, frequency_us: u32, event: Event);

    /// Stop the timer
    fn stop(&mut self);

    /// Check if timer is running
    fn is_running(&self) -> bool;
}
