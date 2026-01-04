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
        crate::event::Event::from_static(&FLAG)
    }};
}
