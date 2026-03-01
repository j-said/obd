use super::iso_tp::{EcuResponse, IsoTpError, IsoTpHandler};
use super::{AsyncCanDriver, is_extended};
use automotive_diag::obd2::*;
use heapless::Vec;

pub const OBD_FUNC_REQ_STD: u32 = 0x7DF;
pub const OBD_FUNC_REQ_EXT: u32 = 0x18DB33F1;
pub const ECU_ENGINE_TX_ID: u32 = 0x7E0;

pub struct Obd2Service<D> {
    tp: IsoTpHandler<D>,
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

    pub async fn get_broadcast_livedata(&self, pid: u8) -> Result<Vec<EcuResponse, 8>, IsoTpError> {
        self.tp
            .send_functional_request(
                Self::get_functional_id(),
                &[Obd2Command::Service01 as u8, pid],
            )
            .await
    }

    pub async fn clear_dtcs(&self) -> Result<(), IsoTpError> {
        let _ = self.tp.send_functional_request(
            Self::get_functional_id(), 
            &[Obd2Command::Service04 as u8]
        ).await?;
        Ok(())
    }

    pub async fn get_vin(&self, ecu_id: u32) -> Result<Vec<u8, 64>, IsoTpError> {
        let mode = Obd2Command::Service09 as u8;
        let pid = Service09Pid::Vin as u8;
        let raw = self.tp.send_physical_request(ecu_id, &[mode, pid]).await?;

        if raw.len() > 3 && raw[0] == (mode + 0x40) && raw[1] == pid {
            let mut vin = Vec::new();
            vin.extend_from_slice(&raw[3..]).ok();
            return Ok(vin);
        }
        Err(IsoTpError::InvalidSequence)
    }
}
