//! Модуль керування апаратним рівнем CAN (TWAI).
//!
//! Ціль: Надати потокобезпечний асинхронний інтерфейс до периферії TWAI
//! для спільного використання різними підсистемами (Scanner, WiFi, BLE).

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use esp_hal::Async;
use esp_hal::twai::{EspTwaiFrame, TwaiRx, TwaiTx};

/// Використовує CriticalSectionRawMutex для безпеки в no_std середовищі.
/// Окремі Mutex-и для Rx та Tx
pub type SharedTwaiRx<'a> = Mutex<CriticalSectionRawMutex, TwaiRx<'a, Async>>;
pub type SharedTwaiTx<'a> = Mutex<CriticalSectionRawMutex, TwaiTx<'a, Async>>;

/// Менеджер CAN-шини.
/// Інкапсулює логіку блокування ресурсу та низькорівневої передачі кадрів.
pub struct CanManager<'a> {
    tx: &'a SharedTwaiTx<'a>,
    rx: &'a SharedTwaiRx<'a>,
}

impl<'a> CanManager<'a> {
    pub fn new(tx: &'a SharedTwaiTx<'a>, rx: &'a SharedTwaiRx<'a>) -> Self {
        Self { tx, rx }
    }
    pub async fn transmit(&self, frame: &EspTwaiFrame) -> Result<(), esp_hal::twai::EspTwaiError> {
        // Тут відновлення складніше, бо ми не володіємо "цілим" драйвером.
        // У esp-hal v1.0 split-частини не мають методу .recover(),
        // тому "fail-fast" логіку треба робити обережніше або перезапускати драйвер зовні.
        
        let mut tx = self.tx.lock().await;
        tx.transmit_async(frame).await
    }

    /// Асинхронне отримання (не блокує відправку)
    pub async fn receive(&self) -> Result<EspTwaiFrame, esp_hal::twai::EspTwaiError> {
        let mut rx = self.rx.lock().await;
        rx.receive_async().await
    }
}
