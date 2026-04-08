pub mod protocol;

use crate::can::{AsyncCanDriver, Obd2Service, obd2::ECU_ENGINE_TX_ID};
use defmt::{info, warn};
use embedded_io_async::{Read, Write};
use protocol::{Command, DebugMsg, Request, Response, Status};

pub async fn handle_client<S, D>(mut stream: S, obd_service: &Obd2Service<D>)
where
    S: Read + Write,
    D: AsyncCanDriver,
{
    let mut in_buf = [0u8; 1024];
    let mut out_buf = [0u8; 1024];

    info!("Client handler task started");

    loop {
        let Ok(n) = stream.read(&mut in_buf).await else {
            warn!("Stream read error");
            break;
        };
        if n == 0 {
            info!("Stream closed by client");
            break;
        }

        if let Ok(raw_str) = core::str::from_utf8(&in_buf[..n]) {
            info!("RX ({} bytes): {}", n, raw_str);
        } else {
            warn!("RX ({} bytes): Decoding failed", n);
        }

        if let Ok((req, _)) = serde_json_core::from_slice::<Request>(&in_buf[..n]) {
            let id = req.id;
            info!("Parsed request ID: {}", id);

            let ser_result = match req.cmd {
                Command::GetVin => match obd_service.get_vin(ECU_ENGINE_TX_ID).await {
                    Ok(vin) => serde_json_core::to_slice(
                        &Response {
                            id,
                            status: Status::Ok,
                            data: Some(&*vin),
                            debug: None,
                        },
                        &mut out_buf,
                    ),
                    Err(_) => serde_json_core::to_slice(
                        &Response::<()> {
                            id,
                            status: Status::Error,
                            data: None,
                            debug: Some(DebugMsg::ObdTimeout),
                        },
                        &mut out_buf,
                    ),
                },
                Command::GetLiveData { pid } => match obd_service.get_broadcast_livedata(pid).await
                {
                    Ok(data) => serde_json_core::to_slice(
                        &Response {
                            id,
                            status: Status::Ok,
                            data: Some(&data),
                            debug: None,
                        },
                        &mut out_buf,
                    ),
                    Err(_) => serde_json_core::to_slice(
                        &Response::<()> {
                            id,
                            status: Status::Error,
                            data: None,
                            debug: Some(DebugMsg::LiveDataFailed),
                        },
                        &mut out_buf,
                    ),
                },
                Command::ClearDtcs => {
                    let _ = obd_service.clear_dtcs().await;
                    serde_json_core::to_slice(
                        &Response::<()> {
                            id,
                            status: Status::Ok,
                            data: None,
                            debug: None,
                        },
                        &mut out_buf,
                    )
                }
                Command::GetStoredDtcs => match obd_service.get_stored_dtcs().await {
                    Ok(data) => serde_json_core::to_slice(
                        &Response {
                            id,
                            status: Status::Ok,
                            data: Some(&data),
                            debug: None,
                        },
                        &mut out_buf,
                    ),
                    Err(_) => serde_json_core::to_slice(
                        &Response::<()> {
                            id,
                            status: Status::Error,
                            data: None,
                            debug: Some(DebugMsg::GetStoredDtcsFailed),
                        },
                        &mut out_buf,
                    ),
                },
            };

            if let Ok(len) = ser_result {
                if let Ok(raw_str) = core::str::from_utf8(&out_buf[..len]) {
                    info!("TX ({} bytes): {}", len, raw_str);
                } else {
                    warn!("TX ({} bytes): Decoding failed", len);
                }
                let _ = stream.write_all(&out_buf[..len]).await;
            }
        } else {
            if let Ok(len) = serde_json_core::to_slice(
                &Response::<()> {
                    id: 0,
                    status: Status::Error,
                    data: None,
                    debug: Some(DebugMsg::InvalidFormat),
                },
                &mut out_buf,
            ) {
                info!("TX ({} bytes): {}", len, "Invalid request format");
                let _ = stream.write_all(&out_buf[..len]).await;
            }
        }
    }
}

// TODO: Put here logic to store dtc localy
// TODO: Add support for autonomous DTC monitoring and reporting via BLE notifications if needed in the future.
