use super::AsyncCanDriver;
use super::iso_tp::{EcuResponse, IsoTpError, IsoTpHandler};
use automotive_diag::obd2::{Obd2Command, Service09Pid};
use core::sync::atomic::{AtomicBool, Ordering};
use embedded_can::{ExtendedId, Id, StandardId};
use heapless::Vec;

pub const OBD_FUNC_REQ_STD: u32 = 0x7DF;
pub const OBD_FUNC_REQ_EXT: u32 = 0x18DB33F1;
pub const ECU_ENGINE_TX_ID: u32 = 0x7E0;

pub struct Obd2Service<D> {
    tp: IsoTpHandler<D>,
    is_extended: AtomicBool,
}

impl<D: AsyncCanDriver> Obd2Service<D> {
    pub fn new(tp: IsoTpHandler<D>, is_extended: bool) -> Self {
        Self {
            tp,
            is_extended: AtomicBool::new(is_extended),
        }
    }

    pub fn set_extended(&self, extended: bool) {
        self.is_extended.store(extended, Ordering::Relaxed);
    }

    fn is_extended(&self) -> bool {
        self.is_extended.load(Ordering::Relaxed)
    }

    fn get_functional_id(&self) -> Id {
        if self.is_extended() {
            Id::Extended(ExtendedId::new(OBD_FUNC_REQ_EXT).unwrap())
        } else {
            Id::Standard(StandardId::new(OBD_FUNC_REQ_STD as u16).unwrap())
        }
    }

    fn to_id(&self, raw: u32) -> Id {
        if self.is_extended() {
            Id::Extended(ExtendedId::new(raw).unwrap())
        } else {
            Id::Standard(StandardId::new(raw as u16).unwrap())
        }
    }

    pub async fn get_broadcast_livedata(&self, pid: u8) -> Result<Vec<EcuResponse, 8>, IsoTpError> {
        self.tp
            .send_functional_request(
                self.get_functional_id(),
                &[Obd2Command::Service01 as u8, pid],
            )
            .await
    }

    pub async fn clear_dtcs(&self) -> Result<(), IsoTpError> {
        self.tp
            .send_functional_request(self.get_functional_id(), &[Obd2Command::Service04 as u8])
            .await?;
        Ok(())
    }

    pub async fn get_stored_dtcs(&self) -> Result<Vec<EcuResponse, 8>, IsoTpError> {
        self.tp
            .send_functional_request(self.get_functional_id(), &[Obd2Command::Service03 as u8])
            .await
    }

    pub async fn get_vin(&self, ecu_id: u32) -> Result<Vec<u8, 64>, IsoTpError> {
        let mode = Obd2Command::Service09 as u8;
        let pid = Service09Pid::Vin as u8;

        let raw = self
            .tp
            .send_physical_request(self.to_id(ecu_id), &[mode, pid])
            .await?;

        if raw.len() > 3 && raw[0] == (mode + 0x40) && raw[1] == pid {
            let mut vin = Vec::new();
            vin.extend_from_slice(&raw[3..]).ok();
            return Ok(vin);
        }
        Err(IsoTpError::InvalidSequence)
    }
}

// TODO: Add more OBD-II services as needed, such as Service 0x09 for more PIDs, Service 0x0A for permanent DTCs, etc.
// TODO: Implement support for sending physical requests to specific ECUs, not just functional requests.
// TODO: Add error handling for cases where the ECU does not respond or returns an error code.
// TODO: Consider adding support for OBD-II over CAN FD if needed in the future.
// TODO: Add unit tests for the Obd2Service methods, possibly using a mock IsoTpHandler to simulate ECU responses.
// TODO: Implement logging for debugging purposes.

// TODO: Add the feuture flag to support SPI-based CAN drivers
