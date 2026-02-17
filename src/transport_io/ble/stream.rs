use super::error::BleError;
use super::types::{BleChannel, BlePacket};
use embedded_io_async::{ErrorType, Read, Write};

pub struct BleStream<'a> {
    rx: &'a BleChannel,
    tx: &'a BleChannel,
    // TODO: add store field
}

impl<'a> BleStream<'a> {
    pub fn new(rx: &'a BleChannel, tx: &'a BleChannel) -> Self {
        Self { rx, tx }
    }
}

impl ErrorType for BleStream<'_> {
    type Error = BleError;
}

impl Read for BleStream<'_> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        // Чекаємо на пакет з каналу RX
        let packet = self.rx.receive().await;

        // Перевіряємо, чи влізе пакет у буфер читача
        if packet.len() > buf.len() {
            // TODO: logic for storing
            return Err(BleError::MtuExceeded);
        }

        // Копіюємо дані
        buf[..packet.len()].copy_from_slice(&packet);
        Ok(packet.len())
    }
}

impl Write for BleStream<'_> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let mut packet = BlePacket::new();
        
        packet
            .extend_from_slice(buf)
            .map_err(|_| BleError::MtuExceeded)?;

        self.tx.send(packet).await;

        Ok(buf.len())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        // TODO: in the distant future scedule the flush
        Ok(())
    }
}
