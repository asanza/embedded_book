// Minimal HAL utilities: Event, macro, and simple debouncers.
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

pub struct Event {
    flag: &'static Mutex<RefCell<bool>>,
}

impl Event {
    pub(crate) const fn from_static(flag: &'static Mutex<RefCell<bool>>) -> Self {
        Event { flag }
    }

    pub fn poll(&self) -> bool {
        cortex_m::interrupt::free(|cs| {
            let mut f = self.flag.borrow(cs).borrow_mut();
            let v = *f;
            *f = false;
            v
        })
    }

    pub(crate) fn trigger(&self) {
        cortex_m::interrupt::free(|cs| {
            *self.flag.borrow(cs).borrow_mut() = true;
        })
    }
}

impl Clone for Event {
    fn clone(&self) -> Self {
        Event { flag: self.flag }
    }
}

#[macro_export]
macro_rules! make_event {
    () => {{
        static FLAG: cortex_m::interrupt::Mutex<core::cell::RefCell<bool>> =
            cortex_m::interrupt::Mutex::new(core::cell::RefCell::new(false));
        crate::hal::utils::Event::from_static(&FLAG)
    }};
}

pub enum DebounceEdge {
    Pressed,
    Released,
}

/// One-shot timer-backed debouncer with KISS API.
/// Call `Debouncer::new(pin, timer, debounce_us)` and then `poll()`.
pub struct Debouncer<T, P>
where
    T: crate::hal::hal_timer::Timer,
    P: crate::hal::hal_gpio::InputInterrupt + crate::hal::hal_gpio::Gpio,
{
    pin: P,
    timer: T,
    debounce_us: u32,
    pin_event: Event,
    tim_event: Event,
    armed: bool,
    last_state: bool,
    pressed_level: bool,
}

impl<T, P> Debouncer<T, P>
where
    T: crate::hal::hal_timer::Timer,
    P: crate::hal::hal_gpio::InputInterrupt + crate::hal::hal_gpio::Gpio,
{
    /// Create a debouncer that auto-detects pressed polarity by sampling
    /// the pin a few times and uses the provided debounce timeout.
    pub fn new(mut pin: P, timer: T, debounce_us: u32) -> Self {
        let mut ones = 0u8;
        let samples = 3u8;
        for _ in 0..samples {
            if pin.read() {
                ones += 1;
            }
            for _ in 0..1000 {
                cortex_m::asm::nop();
            }
        }

        let idle = ones * 2 >= samples;
        let pressed_level = !idle;

        let cur = pin.read();
        let ev = crate::make_event!();
        let tev = crate::make_event!();
        pin.enable_interrupt(crate::hal::hal_gpio::Edge::Both, ev.clone());

        Debouncer {
            pin,
            timer,
            debounce_us,
            pin_event: ev,
            tim_event: tev,
            armed: false,
            last_state: cur,
            pressed_level,
        }
    }

    /// Poll once: read ISR event, arm timer if a raw change happened, and
    /// return a stable DebounceEdge when the one-shot timer fires.
    pub fn poll(&mut self) -> Option<DebounceEdge> {
        // If ISR set the event, arm the timer if needed
        if self.pin_event.poll() {
            let cur = self.pin.read();
            if cur != self.last_state && !self.armed {
                self.timer.start(self.debounce_us, self.tim_event.clone());
                self.armed = true;
            }
        }

        // If timer fired, evaluate stable state.
        if self.armed && self.tim_event.poll() {
            self.armed = false;
            let s = self.pin.read();
            if s == self.pressed_level && self.last_state != self.pressed_level {
                self.last_state = s;
                return Some(DebounceEdge::Pressed);
            }
            if s != self.pressed_level && self.last_state == self.pressed_level {
                self.last_state = s;
                return Some(DebounceEdge::Released);
            }
        }

        None
    }
}

