/// Модуль керування CAN-шиною.
///
/// Містить абстракцію AsyncCanDriver та реалізацію для ESP32.
/// Цей модуль виступає шлюзом до апаратного забезпечення. Він зберігає
/// глобальний стан конфігурації (Standard/Extended) та надає безпечний
/// доступ до шини через EspCanManager.
pub mod iso_tp;
pub mod obd2;

use core::sync::atomic::{AtomicBool, Ordering};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embedded_can::Frame;

use esp_hal::Async;
use esp_hal::twai::{EspTwaiFrame, TwaiRx, TwaiTx};

/// --- Re-exports для зручності ---
pub use iso_tp::IsoTpHandler;
pub use obd2::Obd2Service;

/// --- Абстракція  ---
///
/// D::Frame дозволяє використовувати EspTwaiFrame, Stm32CanFrame або MockFrame.
#[allow(async_fn_in_trait)]
pub trait AsyncCanDriver {
    type Frame: Frame;
    /// Note: Будь-який тип, що реалізує embedded_can::Frame
    type Error: core::fmt::Debug;

    async fn transmit(&self, frame: &Self::Frame) -> Result<(), Self::Error>;
    async fn receive(&self) -> Result<Self::Frame, Self::Error>;
}

/// --- Глобальна конфігурація ---
///
/// Глобальний прапорець режиму адресації.
/// false = Standard ID (11-bit)
/// true = Extended ID (29-bit)
pub static IS_EXTENDED: AtomicBool = AtomicBool::new(false);

/// Helper funcs
#[inline(always)]
pub fn is_extended() -> bool {
    IS_EXTENDED.load(Ordering::Relaxed)
}

pub fn set_extended_mode(mode: bool) {
    IS_EXTENDED.store(mode, Ordering::Relaxed);
}

/// --- Реалізація для ESP32 ---

pub type SharedTwaiRx<'a> = Mutex<CriticalSectionRawMutex, TwaiRx<'a, Async>>;
pub type SharedTwaiTx<'a> = Mutex<CriticalSectionRawMutex, TwaiTx<'a, Async>>;
pub struct EspCanManager<'a> {
    tx: &'a SharedTwaiTx<'a>,
    rx: &'a SharedTwaiRx<'a>,
}

impl<'a> EspCanManager<'a> {
    pub fn new(tx: &'a SharedTwaiTx<'a>, rx: &'a SharedTwaiRx<'a>) -> Self {
        Self { tx, rx }
    }
}

impl<'a> AsyncCanDriver for EspCanManager<'a> {
    type Frame = EspTwaiFrame;
    type Error = esp_hal::twai::EspTwaiError;

    async fn transmit(&self, frame: &EspTwaiFrame) -> Result<(), esp_hal::twai::EspTwaiError> {
        let mut tx = self.tx.lock().await;
        tx.transmit_async(frame).await
    }

    async fn receive(&self) -> Result<EspTwaiFrame, esp_hal::twai::EspTwaiError> {
        let mut rx = self.rx.lock().await;
        rx.receive_async().await
    }
}
