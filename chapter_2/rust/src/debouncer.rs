// Small, no_std-friendly debouncer utility.
// Uses a majority counter policy: sample pushes counter toward pressed or released.
use core::marker::PhantomData;

pub enum Edge {
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
    pub fn sample(&mut self) -> Option<Edge> {
        let s = (self.pin)();
        if s == self.pressed_level {
            if self.counter < self.threshold { self.counter += 1; }
        } else {
            if self.counter > -self.threshold { self.counter -= 1; }
        }

        if self.counter >= self.threshold {
            self.counter = self.threshold; // clamp
            return Some(Edge::Pressed);
        }

        if self.counter <= -self.threshold {
            self.counter = -self.threshold;
            return Some(Edge::Released);
        }

        None
    }
}
