//! Модуль Application layer OBD2
//!
//! Відповідає за бізнес-логіку діагностики: які байти відправити (PID)
//! та як інтерпретувати отримані дані (наприклад, парсинг VIN).
//! Вся транспортна магія (адресація, Flow Control) схована в IsoTpHandler.

use super::iso_tp::{EcuResponse, IsoTpError, IsoTpHandler};
use heapless::Vec;

pub struct Obd2Service<'a> {
    tp: IsoTpHandler<'a>,
}

impl<'a> Obd2Service<'a> {
    pub fn new(tp: IsoTpHandler<'a>) -> Self {
        Self { tp }
    }

    /// Отримання Live Data від усіх ECU (Broadcast)
    /// Зазвичай mode = 0x01
    pub async fn get_broadcast_livedata(
        &self,
        mode: u8,
        pid: u8,
    ) -> Result<Vec<EcuResponse, 8>, IsoTpError> {
        // Відправляємо функціональний запит (всім блокам)
        self.tp.send_functional_request(&[mode, pid]).await
    }

    /// Mode 09: Отримання VIN (Physical Addressing)
    /// Запит направляється конкретному ECU за його ID.
    pub async fn get_vin(&self, ecu_id: u32) -> Result<Vec<u8, 64>, IsoTpError> {
        // Формуємо запит Service 09, PID 02
        let raw = self.tp.send_physical_request(ecu_id, &[0x09, 0x02]).await?;
        
        // Парсинг відповіді.
        // Очікуваний формат відповіді (позитивний): [0x49, 0x02, 0x01 (count), VIN bytes...]
        // Ми пропускаємо перші 3 байти заголовків.
        let mut vin = Vec::new();
        if raw.len() > 3 {
            vin.extend_from_slice(&raw[3..]).ok();
        }
        Ok(vin)
    }

    /// Mode 03: Запит збережених помилок (DTC)
    pub async fn get_stored_dtcs(&self) -> Result<Vec<EcuResponse, 8>, IsoTpError> {
        self.tp.send_functional_request(&[0x03]).await
    }

    /// Mode 04: Очищення помилок (Clear DTCs)
    pub async fn clear_dtcs(&self) -> Result<(), IsoTpError> {
        // Ігноруємо результат, оскільки ECU може не відповісти на команду очищення
        let _ = self.tp.send_functional_request(&[0x04]).await?;
        Ok(())
    }

    /// Mode 07: Помилки, що очікують підтвердження (Pending DTCs)
    pub async fn get_pending_dtcs(&self) -> Result<Vec<EcuResponse, 8>, IsoTpError> {
        self.tp.send_functional_request(&[0x07]).await
    }
}