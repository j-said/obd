use portable_atomic::{AtomicBool, Ordering};

/// false = 11-bit (Standard), true = 29-bit (Extended)
static IS_EXTENDED: AtomicBool = AtomicBool::new(false);
/// Гарантує, що протокол зафіксовано після успішного Discovery
static CONFIG_INITIALIZED: AtomicBool = AtomicBool::new(false);

/// Встановлює тип протоколу. Викликається після успішного пінгу ECU.
pub fn set_protocol(extended: bool) -> Result<(), &'static str> {
    if CONFIG_INITIALIZED
        .compare_exchange(false, true, Ordering::SeqCst, Ordering::SeqCst)
        .is_ok()
    {
        IS_EXTENDED.store(extended, Ordering::SeqCst);
        Ok(())
    } else {
        Err("Protocol already locked")
    }
}

/// Скидає стан конфігурації для нового циклу пошуку (напр. при перепідключенні)
pub fn reset_protocol() {
    CONFIG_INITIALIZED.store(false, Ordering::SeqCst);
}

#[inline(always)]
pub fn is_extended() -> bool {
    IS_EXTENDED.load(Ordering::Relaxed)
}
