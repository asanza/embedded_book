use cortex_m::interrupt;

/// Trait representing a triggerable event (ISR-safe).
pub trait Trigger {
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
    fn mask(&self) -> u32 {
        Self::mask()
    }
}

static mut EVENTS: u32 = 0;

/// Signal an event mask from arch modules/ISRs.
/// This is visible to the crate so arch modules can call it, but it is
/// intentionally not re-exported by `hal::utils`.
pub(crate) fn signal_mask(mask: u32) {
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
