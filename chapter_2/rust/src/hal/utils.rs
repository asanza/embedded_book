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
        Debouncer {
            pin,
            counter: 0,
            threshold,
            pressed_level,
            _marker: PhantomData,
        }
    }

    /// Call this on each sampling tick; returns Some(Edge) when a stable edge is detected.
    pub fn sample(&mut self) -> Option<DebounceEdge> {
        let s = (self.pin)();
        if s == self.pressed_level {
            if self.counter < self.threshold {
                self.counter += 1;
            }
        } else {
            if self.counter > -self.threshold {
                self.counter -= 1;
            }
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

/// One-shot timer-backed debouncer.
///
/// Usage: create with a pin reader closure and a hardware timer implementing
/// the `Timer` trait. Call `on_change(debounce_us)` when you observe a raw
/// transition (e.g. from an interrupt or poll). Then call `poll()` regularly
/// in the main loop; when the timer fires `poll()` will return a
/// `Some(DebounceEdge)` if a stable edge was detected.
pub struct DebouncerOneShot<T, P>
where
    P: FnMut() -> bool,
    T: crate::hal::hal_timer::Timer,
{
    pin: P,
    timer: T,
    event: Event,
    armed: bool,
    last_state: bool,
    pressed_level: bool,
}

impl<T, P> DebouncerOneShot<T, P>
where
    P: FnMut() -> bool,
    T: crate::hal::hal_timer::Timer,
{
    pub fn new(mut pin: P, timer: T, pressed_level: bool) -> Self {
        let cur = (pin)();
        let ev = crate::make_event!();
        DebouncerOneShot {
            pin,
            timer,
            event: ev,
            armed: false,
            last_state: cur,
            pressed_level,
        }
    }

    /// Arm the one-shot timer if the pin changed since last observed state.
    /// If already armed this is a no-op.
    pub fn on_change(&mut self, debounce_us: u32) {
        let cur = (self.pin)();
        if cur != self.last_state && !self.armed {
            self.timer.start(debounce_us, self.event.clone());
            self.armed = true;
        }
    }

    /// Poll the debouncer; returns `Some(DebounceEdge)` when the one-shot timer
    /// fired and a stable edge was detected.
    pub fn poll(&mut self) -> Option<DebounceEdge> {
        if self.armed && self.event.poll() {
            self.armed = false;
            let s = (self.pin)();
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
