// TODO: Пізніше мігрувати до transport_io::mod, як абстракція для бізнес логіки
use super::{BleError, BleChannel, BlePacket, MTU_SIZE};
use embedded_io_async::{ErrorType, Read, Write};

pub struct BleStream {
    rx: &'static BleChannel,
    tx: &'static BleChannel,
    store: BlePacket,
    store_offset: usize,
}

impl BleStream {
    pub fn new(rx: &'static BleChannel, tx: &'static BleChannel) -> Self {
        Self {
            rx,
            tx,
            store: BlePacket::new(),
            store_offset: 0,
        }
    }
}

impl ErrorType for BleStream {
    type Error = BleError;
}

impl Read for BleStream {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }

        // Якщо внутрішній буфер вичитано, чекаємо новий пакет
        if self.store.is_empty() || self.store_offset >= self.store.len() {
            self.store = self.rx.receive().await;
            self.store_offset = 0;
        }

        let available = self.store.len() - self.store_offset;
        let to_copy = core::cmp::min(available, buf.len());

        buf[..to_copy].copy_from_slice(&self.store[self.store_offset..self.store_offset + to_copy]);
        self.store_offset += to_copy;

        if self.store_offset >= self.store.len() {
            self.store.clear();
            self.store_offset = 0;
        }

        Ok(to_copy)
    }
}

impl Write for BleStream {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }

        let mut offset = 0;

        // Фрагментація великих масивів під MTU радіоканалу
        while offset < buf.len() {
            let chunk_size = core::cmp::min(buf.len() - offset, MTU_SIZE);
            let mut packet = BlePacket::new();

            packet
                .extend_from_slice(&buf[offset..offset + chunk_size])
                .map_err(|_| BleError::MtuExceeded)?;

            self.tx.send(packet).await;
            offset += chunk_size;
        }

        Ok(buf.len())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
