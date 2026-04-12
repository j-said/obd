pub mod protocol;

use crate::can::{AsyncCanDriver, Obd2Service, iso_tp::IsoTpError};
use defmt::info;
use embedded_can::{Frame, Id};
use embedded_io_async::{Read, Write};
use heapless::Vec;
use protocol::{DebugMsg, Response, Status};

pub async fn handle_client<S, D>(mut stream: S, obd_service: &Obd2Service<D>)
where
    S: Read + Write,
    D: AsyncCanDriver,
{
    let mut out_buf = [0u8; 1024];
    let mut id: u32 = 0;

    info!("Started loop");

    loop {
        let ser_result = match obd_service.debug_sniffer().await {
            Ok(frame) => {
                let can_id = match frame.id() {
                    Id::Standard(sid) => sid.as_raw() as u32,
                    Id::Extended(eid) => eid.as_raw(),
                };
                let mut raw: Vec<u8, 13> = Vec::new();
                raw.extend_from_slice(&can_id.to_le_bytes()).ok();
                raw.push(frame.dlc() as u8).ok();
                raw.extend_from_slice(frame.data()).ok();
                serde_json_core::to_slice(
                    &Response {
                        id,
                        status: Status::Ok,
                        data: Some(raw),
                        debug: None,
                    },
                    &mut out_buf,
                )
            }
            Err(e) => serde_json_core::to_slice(
                &Response::<Vec<u8, 13>> {
                    id,
                    status: Status::Error,
                    data: None,
                    debug: Some(iso_tp_to_debug(e)),
                },
                &mut out_buf,
            ),
        };
        id += 1;
        if let Ok(len) = ser_result {
            let _ = stream.write_all(&out_buf[..len]).await;
        }
    }
}

fn iso_tp_to_debug(e: IsoTpError) -> DebugMsg {
    match e {
        IsoTpError::TimeoutA => DebugMsg::IsoTpTimeoutA,
        IsoTpError::TimeoutBs => DebugMsg::IsoTpTimeoutBs,
        IsoTpError::TimeoutCr => DebugMsg::IsoTpTimeoutCr,
        IsoTpError::WrongSn => DebugMsg::IsoTpWrongSn,
        IsoTpError::InvalidFs => DebugMsg::IsoTpInvalidFs,
        IsoTpError::WftOverrun => DebugMsg::IsoTpWftOverrun,
        IsoTpError::BufferOverflow => DebugMsg::IsoTpBufferOverflow,
        IsoTpError::DriverError => DebugMsg::IsoTpDriverError,
        IsoTpError::InvalidId => DebugMsg::IsoTpInvalidId,
    }
}

// TODO: Put here logic to store dtc localy
// TODO: Add support for autonomous DTC monitoring and reporting via BLE notifications if needed in the future.
