//! Модуль керування CAN-шиною (TWAI).
//!
//! Цей модуль виступає шлюзом до апаратного забезпечення. Він зберігає
//! глобальний стан конфігурації (Standard/Extended) та надає безпечний
//! доступ до шини через CanManager.

pub mod iso_tp;
pub mod obd2;

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

use esp_hal::Async;
use esp_hal::twai::{EspTwaiFrame, TwaiRx, TwaiTx};

// --- Re-exports для зручності ---
pub use iso_tp::IsoTpHandler;
pub use obd2::Obd2Service;

/// Окремі Mutex-и для Rx та Tx, щоб дозволити одночасний доступ
/// (наприклад, одна задача слухає, інша відправляє).
pub type SharedTwaiRx<'a> = Mutex<CriticalSectionRawMutex, TwaiRx<'a, Async>>;
pub type SharedTwaiTx<'a> = Mutex<CriticalSectionRawMutex, TwaiTx<'a, Async>>;

// --- Глобальна конфігурація ---

/// Глобальний прапорець режиму адресації.
/// false = Standard ID (11-bit)
/// true = Extended ID (29-bit)
pub static IS_EXTENDED: AtomicBool = AtomicBool::new(false);

/// Helper: Чи увімкнено режим розширених ID (29-bit)?
#[inline(always)]
pub fn is_extended() -> bool {
    IS_EXTENDED.load(Ordering::Relaxed)
}

/// Helper: Встановити режим адресації (викликати в main при старті).
pub fn set_extended_mode(mode: bool) {
    IS_EXTENDED.store(mode, Ordering::Relaxed);
}

// --- Менеджер шини ---

/// Обгортка над Mutex-ами для зручної передачі в сервіси (IsoTp, OBD2).
pub struct CanManager<'a> {
    tx: &'a SharedTwaiTx<'a>,
    rx: &'a SharedTwaiRx<'a>,
}

impl<'a> CanManager<'a> {
    pub fn new(tx: &'a SharedTwaiTx<'a>, rx: &'a SharedTwaiRx<'a>) -> Self {
        Self { tx, rx }
    }

    /// Асинхронна відправка кадру.
    /// Блокує Tx-м'ютекс лише на час запису в регістри.
    pub async fn transmit(&self, frame: &EspTwaiFrame) -> Result<(), esp_hal::twai::EspTwaiError> {
        let mut tx = self.tx.lock().await;
        tx.transmit_async(frame).await
    }

    /// Асинхронне отримання кадру.
    /// Блокує Rx-м'ютекс, поки не прийде повідомлення.
    pub async fn receive(&self) -> Result<EspTwaiFrame, esp_hal::twai::EspTwaiError> {
        let mut rx = self.rx.lock().await;
        rx.receive_async().await
    }
}
