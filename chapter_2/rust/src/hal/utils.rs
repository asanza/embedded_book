use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

pub struct Event {
    flag: &'static Mutex<RefCell<bool>>,
}

impl Event {
    /// Create from static storage (internal use)
    pub(crate) const fn from_static(flag: &'static Mutex<RefCell<bool>>) -> Self {
        Event { flag }
    }

    /// Check and clear the event flag
    pub fn poll(&self) -> bool {
        cortex_m::interrupt::free(|cs| {
            let mut flag = self.flag.borrow(cs).borrow_mut();
            let was_set = *flag;
            *flag = false;
            was_set
        })
    }

    /// Trigger the event (called from interrupt)
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

/// Macro to create events with static storage
#[macro_export]
macro_rules! make_event {
    () => {{
        static FLAG: cortex_m::interrupt::Mutex<core::cell::RefCell<bool>> =
            cortex_m::interrupt::Mutex::new(core::cell::RefCell::new(false));
        crate::hal::utils::Event::from_static(&FLAG)
    }};
}

// Small, no_std-friendly debouncer utility.
// Uses a majority counter policy: sample pushes counter toward pressed or released.
use core::marker::PhantomData;

pub enum DebounceEdge {
    Pressed,
    Released,
}

pub struct Debouncer<P> {
    pin: P,
    counter: i8,
    threshold: i8,
    pressed_level: bool,
    _marker: PhantomData<P>,
}

impl<P> Debouncer<P>
where
    P: FnMut() -> bool,
{
    pub fn new(pin: P, threshold: i8, pressed_level: bool) -> Self {
        Debouncer { pin, counter: 0, threshold, pressed_level, _marker: PhantomData }
    }

    /// Call this on each sampling tick; returns Some(Edge) when a stable edge is detected.
    pub fn sample(&mut self) -> Option<DebounceEdge> {
        let s = (self.pin)();
        if s == self.pressed_level {
            if self.counter < self.threshold { self.counter += 1; }
        } else {
            if self.counter > -self.threshold { self.counter -= 1; }
        }

        if self.counter >= self.threshold {
            self.counter = self.threshold; // clamp
            return Some(DebounceEdge::Pressed);
        }

        if self.counter <= -self.threshold {
            self.counter = -self.threshold;
            return Some(DebounceEdge::Released);
        }

        None
    }
}
