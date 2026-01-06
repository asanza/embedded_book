// Minimal HAL utilities: Event, macro, and simple debouncers.
use core::sync::atomic::{AtomicU32, Ordering};

// Minimal HAL utilities: Event and factories.

/// Global event mask (single 32-bit word) used by const-generic `Event`.
static EVENTS: AtomicU32 = AtomicU32::new(0);

/// Trait representing a triggerable event (ISR-safe).
pub trait Trigger {
    fn trigger(&self);
    fn mask(&self) -> u32;
}

// (No owned trigger trait — zero-sized `Event` values are created via `Event::new()`.)

/// Zero-sized, compile-time event type. `Event<0>` maps to bit 0, etc.
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
        EVENTS.fetch_or(Self::mask(), Ordering::Release);
    }
    fn mask(&self) -> u32 { Self::mask() }
}
// No factory: the app creates `Event::<N>::new()` values directly.

// (Removed compile-time factory and legacy BoolEvent to keep utils minimal.)

/// Signal an event mask from other modules/ISRs.
pub fn signal_mask(mask: u32) {
    // Use Release ordering to pair with `poll_all()` Acquire read.
    EVENTS.fetch_or(mask, Ordering::Release);
}

/// Atomically read and clear the global event mask.
pub fn poll_all() -> u32 {
    EVENTS.swap(0, Ordering::Acquire)
}

// Debouncer removed — main uses mask-backed events and superloop.

