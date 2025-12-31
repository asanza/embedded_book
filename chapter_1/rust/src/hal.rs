#![allow(dead_code)]

pub trait Gpio {
    fn write(&mut self, high: bool);
}

pub trait Timer {
    fn delay_ms(&mut self, ms: u32);
}
