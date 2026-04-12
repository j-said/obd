use super::AsyncCanDriver;
use super::iso_tp::{EcuResponse, IsoTpError, IsoTpHandler};
use automotive_diag::obd2::{Obd2Command, Service09Pid};
use defmt::{error, info};
use embedded_can::{ExtendedId, Id, StandardId};
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

    /// Switch to Extended (29-bit) addressing mode at runtime.
    pub fn to_extended_adr(&self, target_addr: u8) {
        info!("enter: Obd2Service::to_extended_adr target_addr=0x{:02X}", target_addr);
        self.tp.to_extended_adr(target_addr);
        info!("return: Obd2Service::to_extended_adr");
    }

    /// Switch to Normal (11-bit) addressing mode at runtime.
    pub fn to_normal_addr(&self) {
        info!("enter: Obd2Service::to_normal_addr");
        self.tp.to_normal_addr();
        info!("return: Obd2Service::to_normal_addr");
    }

    fn get_functional_id(&self) -> Id {
        if self.tp.is_extended_addressing() {
            Id::Extended(ExtendedId::new(OBD_FUNC_REQ_EXT).unwrap())
        } else {
            Id::Standard(StandardId::new(OBD_FUNC_REQ_STD as u16).unwrap())
        }
    }

    fn to_id(&self, raw: u32) -> Id {
        if self.tp.is_extended_addressing() {
            Id::Extended(ExtendedId::new(raw).unwrap())
        } else {
            Id::Standard(StandardId::new(raw as u16).unwrap())
        }
    }

    pub async fn get_broadcast_livedata(&self, pid: u8) -> Result<Vec<EcuResponse, 8>, IsoTpError> {
        info!("enter: Obd2Service::get_broadcast_livedata pid=0x{:02X}", pid);
        let result = self
            .tp
            .send_functional_request(
                self.get_functional_id(),
                &[Obd2Command::Service01 as u8, pid],
            )
            .await;
        match &result {
            Ok(_) => info!("return ok: Obd2Service::get_broadcast_livedata"),
            Err(e) => error!("return err: Obd2Service::get_broadcast_livedata {:?}", e),
        }
        result
    }

    pub async fn clear_dtcs(&self) -> Result<(), IsoTpError> {
        info!("enter: Obd2Service::clear_dtcs");
        let result = self
            .tp
            .send_functional_request(self.get_functional_id(), &[Obd2Command::Service04 as u8])
            .await
            .map(|_| ());
        match &result {
            Ok(_) => info!("return ok: Obd2Service::clear_dtcs"),
            Err(e) => error!("return err: Obd2Service::clear_dtcs {:?}", e),
        }
        result
    }

    pub async fn get_stored_dtcs(&self) -> Result<Vec<EcuResponse, 8>, IsoTpError> {
        info!("enter: Obd2Service::get_stored_dtcs");
        let result = self
            .tp
            .send_functional_request(self.get_functional_id(), &[Obd2Command::Service03 as u8])
            .await;
        match &result {
            Ok(_) => info!("return ok: Obd2Service::get_stored_dtcs"),
            Err(e) => error!("return err: Obd2Service::get_stored_dtcs {:?}", e),
        }
        result
    }

    pub async fn get_vin(&self, ecu_id: u32) -> Result<Vec<u8, 256>, IsoTpError> {
        info!("enter: Obd2Service::get_vin ecu_id=0x{:08X}", ecu_id);
        let mode = Obd2Command::Service09 as u8;
        let pid = Service09Pid::Vin as u8;

        let raw = self
            .tp
            .send_physical_request(self.to_id(ecu_id), &[mode, pid])
            .await?;

        if raw.len() > 3 && raw[0] == (mode + 0x40) && raw[1] == pid {
            let mut vin = Vec::new();
            vin.extend_from_slice(&raw[3..]).ok();
            info!("return ok: Obd2Service::get_vin");
            return Ok(vin);
        }
        error!("return err: Obd2Service::get_vin DriverError (unexpected response)");
        Err(IsoTpError::DriverError)
    }
}

// TODO: Add more OBD-II services as needed, such as Service 0x09 for more PIDs, Service 0x0A for permanent DTCs, etc.
// TODO: Implement support for sending physical requests to specific ECUs, not just functional requests.
// TODO: Add error handling for cases where the ECU does not respond or returns an error code.
// TODO: Consider adding support for OBD-II over CAN FD if needed in the future.
// TODO: Add unit tests for the Obd2Service methods, possibly using a mock IsoTpHandler to simulate ECU responses.

// TODO: Add the feuture flag to support SPI-based CAN drivers
