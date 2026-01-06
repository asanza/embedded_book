use cortex_m::interrupt;

/// Global event mask (single 32-bit word) used by `Event`
static mut EVENTS: u32 = 0;

/// Trait representing a triggerable event (ISR-safe).
pub trait Trigger {
    fn trigger(&self);
    fn mask(&self) -> u32;
}

/// Zero-sized event type; `Event<0>` maps to bit 0
pub struct Event<const BIT: u8>;

impl<const BIT: u8> Event<BIT> {
    pub const fn new() -> Self {
        Event
    }

    pub const fn mask() -> u32 {
        1u32 << BIT
    }
}

impl<const BIT: u8> Trigger for Event<BIT> {
    fn trigger(&self) {
        // set bit under critical section
        interrupt::free(|_| unsafe { EVENTS |= Self::mask() });
    }
    fn mask(&self) -> u32 { Self::mask() }
}

/// Signal an event mask from other modules/ISRs
pub fn signal_mask(mask: u32) {
    interrupt::free(|_| unsafe { EVENTS |= mask });
}

/// Atomically read and clear the global event mask.
pub fn poll_all() -> u32 {
    interrupt::free(|_| {
        let v = unsafe { EVENTS };
        unsafe { EVENTS = 0 };
        v
    })
}

// Main loop polls the global mask and handles events

