//! Модуль прикладного рівня OBD2.
//! Використовує IsoTpHandler для виконання діагностичних команд.

use super::iso_tp::{EcuResponse, IsoTpError, IsoTpHandler};
use heapless::Vec;

pub struct Obd2Service<'a> {
    tp: IsoTpHandler<'a>,
}

impl<'a> Obd2Service<'a> {
    pub fn new(tp: IsoTpHandler<'a>) -> Self {
        Self { tp }
    }

    /// Отримання сирих даних від усіх ECU. Декодування виконується на стороні App.
    pub async fn get_broadcast_livedata(
        &self,
        mode: u8,
        pid: u8,
    ) -> Result<Vec<EcuResponse, 8>, IsoTpError> {
        self.tp.send_functional_request(&[mode, pid]).await
    }

    /// Отримання VIN. Використовує Physical Addressing через IsoTpHandler.
    pub async fn get_vin(&self, ecu_id: u32) -> Result<Vec<u8, 64>, IsoTpError> {
        let raw = self.tp.send_request(ecu_id, &[0x09, 0x02]).await?;
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
        let _ = self.tp.send_functional_request(&[0x04]).await?;
        Ok(())
    }

    /// Mode 07: Помилки, що очікують підтвердження
    pub async fn get_pending_dtcs(&self) -> Result<Vec<EcuResponse, 8>, IsoTpError> {
        self.tp.send_functional_request(&[0x07]).await
    }
}
