// Minimal HAL utilities: Event, macro, and simple debouncers.
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use core::sync::atomic::{AtomicU32, Ordering};

/// Legacy boolean-backed Event kept for compatibility.
#[derive(Copy, Clone)]
pub struct BoolEvent {
    flag: &'static Mutex<RefCell<bool>>,
}

impl BoolEvent {
    pub(crate) const fn from_static(flag: &'static Mutex<RefCell<bool>>) -> Self {
        BoolEvent { flag }
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


// BoolEvent derives Copy/Clone above; no manual impl needed.

impl Trigger for BoolEvent {
    fn trigger(&self) {
        cortex_m::interrupt::free(|cs| {
            *self.flag.borrow(cs).borrow_mut() = true;
        })
    }
    fn mask(&self) -> u32 {
        0
    }
}

#[macro_export]
macro_rules! make_event {
    () => {{
        static FLAG: cortex_m::interrupt::Mutex<core::cell::RefCell<bool>> =
            cortex_m::interrupt::Mutex::new(core::cell::RefCell::new(false));
        crate::hal::utils::BoolEvent::from_static(&FLAG)
    }};
}

/// Global event mask (single 32-bit word) used by const-generic `Event`.
static EVENTS: AtomicU32 = AtomicU32::new(0);

/// Trait representing a triggerable event (ISR-safe).
pub trait Trigger {
    fn trigger(&self);
    fn mask(&self) -> u32;
}

/// Zero-sized, compile-time event type. `Event<0>` maps to bit 0, etc.
#[derive(Copy, Clone)]
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

/// Signal an event mask from other modules/ISRs.
pub fn signal_mask(mask: u32) {
    EVENTS.fetch_or(mask, Ordering::Relaxed);
}

/// Atomically read and clear the global event mask.
pub fn poll_all() -> u32 {
    EVENTS.swap(0, Ordering::Acquire)
}

// Debouncer removed — main uses mask-backed events and superloop.

