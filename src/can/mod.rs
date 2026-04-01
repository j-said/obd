pub mod iso_tp;
pub mod obd2;
pub mod dtc;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embedded_can::Frame;

use esp_hal::Async;
use esp_hal::twai::{EspTwaiFrame, TwaiRx, TwaiTx};

pub use iso_tp::IsoTpHandler;
pub use obd2::Obd2Service;

#[allow(async_fn_in_trait)]
pub trait AsyncCanDriver {
    type Frame: Frame;
    // Note: Будь-який тип, що реалізує embedded_can::Frame
    type Error: core::fmt::Debug;

    async fn transmit(&self, frame: &Self::Frame) -> Result<(), Self::Error>;
    async fn receive(&self) -> Result<Self::Frame, Self::Error>;
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
