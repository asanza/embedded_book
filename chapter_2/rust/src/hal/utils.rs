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

/// Trigger that can be owned and (re)instantiated by value.
pub trait OwnableTrigger: Trigger + Sized {
    fn new_owned() -> Self;
}

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
        EVENTS.fetch_or(Self::mask(), Ordering::Relaxed);
    }
    fn mask(&self) -> u32 { Self::mask() }
}

impl<const BIT: u8> OwnableTrigger for Event<BIT> {
    fn new_owned() -> Self {
        Event
    }
}

/// Simple value-level EventFactory that tracks which bits are already used.
/// It returns a new `Event<BIT>` and a new `EventFactory` with the bit marked.
#[derive(Copy, Clone)]
pub struct EventFactory {
    used: u32,
}

impl EventFactory {
    pub const fn new() -> Self {
        EventFactory { used: 0 }
    }

    pub fn create<const BIT: u8>(self) -> Result<(Event<BIT>, EventFactory), &'static str> {
        if BIT >= 32 {
            return Err("bit out of range");
        }
        let mask = 1u32 << (BIT as u32);
        if (self.used & mask) != 0 {
            return Err("event bit already allocated");
        }
        let new_factory = EventFactory { used: self.used | mask };
        Ok((Event::<BIT>::new(), new_factory))
    }
}

// (Removed compile-time factory and legacy BoolEvent to keep utils minimal.)

/// Signal an event mask from other modules/ISRs.
pub fn signal_mask(mask: u32) {
    EVENTS.fetch_or(mask, Ordering::Relaxed);
}

/// Atomically read and clear the global event mask.
pub fn poll_all() -> u32 {
    EVENTS.swap(0, Ordering::Acquire)
}

// Debouncer removed — main uses mask-backed events and superloop.

