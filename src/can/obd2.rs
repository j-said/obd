/// Модуль Application layer OBD2
///
/// Generic implementation.
/// Відповідає за бізнес-логіку діагностики: які байти відправити (PID)
/// та як інтерпретувати отримані дані (наприклад, парсинг VIN).
/// Вся транспортна магія (адресація, Flow Control) схована в IsoTpHandler.
use super::iso_tp::{EcuResponse, IsoTpError, IsoTpHandler};
use super::{AsyncCanDriver, is_extended};
use heapless::Vec;

pub const OBD_FUNC_REQ_STD: u32 = 0x7DF;
pub const OBD_FUNC_REQ_EXT: u32 = 0x18DB33F1;
pub const ECU_ENGINE_TX_ID: u32 = 0x7E0;

const POSITIVE_RESPONSE_OFFSET: u8 = 0x40;

pub struct Obd2Service<D> {
    tp: IsoTpHandler<D>,
}

#[repr(u8)]
pub enum Obd2Mode {
    LiveData = 0x01,
    ShowDtcs = 0x03,
    ClearDtcs = 0x04,
    PendingDtcs = 0x07,
    VehicleInfo = 0x09,
}

impl<D: AsyncCanDriver> Obd2Service<D> {
    pub fn new(tp: IsoTpHandler<D>) -> Self {
        Self { tp }
    }

    fn get_functional_id() -> u32 {
        if is_extended() {
            OBD_FUNC_REQ_EXT
        } else {
            OBD_FUNC_REQ_STD
        }
    }

    /// Отримання Live Data від усіх ECU (Broadcast)
    /// Зазвичай mode = 0x01
    pub async fn get_broadcast_livedata(&self, pid: u8) -> Result<Vec<EcuResponse, 8>, IsoTpError> {
        self.tp
            .send_functional_request(Self::get_functional_id(), &[Obd2Mode::LiveData as u8, pid])
            .await
    }

    /// Mode 09: Отримання VIN (Physical Addressing)
    pub async fn get_vin(&self, ecu_id: u32) -> Result<Vec<u8, 64>, IsoTpError> {
        let mode = Obd2Mode::VehicleInfo as u8;
        // VIN PID
        let pid = 0x02 as u8;
        let raw = self.tp.send_physical_request(ecu_id, &[mode, pid]).await?;

        if raw.len() > 3 && raw[0] == mode + POSITIVE_RESPONSE_OFFSET && raw[1] == pid {
            let mut vin = Vec::new();
            vin.extend_from_slice(&raw[3..]).ok();
            return Ok(vin);
        }
        Err(IsoTpError::InvalidSequence)
    }

    /// Mode 03: Запит збережених помилок (DTC)
    pub async fn get_stored_dtcs(&self) -> Result<Vec<EcuResponse, 8>, IsoTpError> {
        self.tp
            .send_functional_request(Self::get_functional_id(), &[Obd2Mode::ShowDtcs as u8])
            .await
    }

    /// Mode 04: Очищення помилок (Clear DTCs)
    pub async fn clear_dtcs(&self) -> Result<(), IsoTpError> {
        let _ = self
            .tp
            .send_functional_request(Self::get_functional_id(), &[Obd2Mode::ClearDtcs as u8])
            .await?;
        Ok(())
    }

    /// Mode 07: Помилки, що очікують підтвердження (Pending DTCs)
    pub async fn get_pending_dtcs(&self) -> Result<Vec<EcuResponse, 8>, IsoTpError> {
        self.tp
            .send_functional_request(Self::get_functional_id(), &[Obd2Mode::PendingDtcs as u8])
            .await
    }
}
