// L2CAP CoC implementation (replaced by GATT/NUS — kept for reference)
/*
use super::{BleChannel, BleError, BlePacket, DEVICE_NAME, MTU_SIZE, ObdStack};
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, with_timeout};
use trouble_host::advertise::{AdStructure, Advertisement};
use trouble_host::prelude::*;

pub async fn run_connection(
    stack: &'static ObdStack,
    conn: &Connection<'_, super::MyPacketPool>,
    tx_channel: &'static BleChannel,
    rx_channel: &'static BleChannel,
) -> Result<(), BleError> {
    let l2cap = match with_timeout(
        Duration::from_secs(5),
        trouble_host::l2cap::L2capChannel::accept(
            stack,
            conn,
            &[0x0080],
            &trouble_host::l2cap::L2capChannelConfig::default(),
        ),
    )
    .await
    {
        Ok(Ok(c)) => c,
        Ok(Err(_)) => return Err(BleError::L2capError),
        Err(_) => return Err(BleError::Timeout),
    };

    let (mut tx_socket, mut rx_socket) = l2cap.split();

    let rx_task = async {
        let mut buf = [0u8; MTU_SIZE];
        loop {
            let len = rx_socket
                .receive(stack, &mut buf)
                .await
                .map_err(|_| BleError::L2capError)?;
            if len == 0 {
                return Err(BleError::ChannelClosed);
            }

            let mut packet = BlePacket::new();
            packet
                .extend_from_slice(&buf[..len])
                .map_err(|_| BleError::MtuExceeded)?;
            rx_channel.send(packet).await;
        }
        #[allow(unreachable_code)]
        Ok::<(), BleError>(())
    };

    let tx_task = async {
        loop {
            let packet = tx_channel.receive().await;
            tx_socket
                .send(stack, &packet)
                .await
                .map_err(|_| BleError::L2capError)?;
        }
        #[allow(unreachable_code)]
        Ok::<(), BleError>(())
    };

    match select(rx_task, tx_task).await {
        Either::First(res) => res,
        Either::Second(res) => res,
    }
}
*/

use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use heapless::Vec;
use trouble_host::advertise::{AdStructure, Advertisement};
use trouble_host::prelude::*;

use super::{BleChannel, BleError, BlePacket, DEVICE_NAME, MTU_SIZE, NUS_SERVICE_UUID};

// ==========================================
// GATT SERVICES
// ==========================================

/// Nordic UART Service — standard BLE GATT profile for UART-style communication.
/// Enables connection from any standard BLE central (laptop, phone, nRF Connect, etc.)
#[gatt_service(uuid = "6e400001-b5b3-f393-e0a9-e50e24dcca9e")]
pub struct NusService {
    /// RX: central writes data to peripheral (NUS spec: WriteWithoutResponse)
    #[characteristic(uuid = "6e400002-b5b3-f393-e0a9-e50e24dcca9e", write_without_response)]
    pub rx: Vec<u8, MTU_SIZE>,

    /// TX: peripheral notifies central with response data
    #[characteristic(uuid = "6e400003-b5b3-f393-e0a9-e50e24dcca9e", notify)]
    pub tx: Vec<u8, MTU_SIZE>,
}

/// Public key exchange service (custom UUID).
/// Allows future X25519 key exchange; encryption not enforced yet.
#[gatt_service(uuid = "deadbeef-1234-5678-90ab-cdef01234567")]
pub struct PubKeyService {
    /// 32-byte X25519 public key: central writes its key, reads device key
    #[characteristic(uuid = "deadbeef-1234-5678-90ab-cdef01234568", read, write)]
    pub key: [u8; 32],
}

/// Composite GATT server: GAP + NUS + PubKey.
/// CriticalSectionRawMutex is required because the server is shared between
/// the concurrent gatt_task and notify_task futures.
#[gatt_server(mutex_type = CriticalSectionRawMutex)]
pub struct ObdGattServer {
    pub nus: NusService,
    pub pubkey: PubKeyService,
}

// ==========================================
// ADVERTISEMENT
// ==========================================

/// Build advertisement + scan response.
/// adv_data: Flags + complete name (14 bytes — fits 31-byte limit).
/// scan_data: NUS service UUID 128-bit (18 bytes — fits 31-byte limit).
pub fn create_advertisement<'a>(
    adv_buf: &'a mut [u8],
    scan_buf: &'a mut [u8],
) -> Result<Advertisement<'a>, BleError> {
    let adv_len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::CompleteLocalName(DEVICE_NAME.as_bytes()),
        ],
        adv_buf,
    )
    .map_err(|_| BleError::AdvertisingError)?;

    let scan_len = AdStructure::encode_slice(
        &[AdStructure::ServiceUuids128(&[NUS_SERVICE_UUID])],
        scan_buf,
    )
    .map_err(|_| BleError::AdvertisingError)?;

    Ok(Advertisement::ConnectableScannableUndirected {
        adv_data: &adv_buf[..adv_len],
        scan_data: &scan_buf[..scan_len],
    })
}

// ==========================================
// GATT CONNECTION HANDLER
// ==========================================

/// Run the GATT connection, bridging BLE GATT events to the BleChannel queues
/// consumed by BleStream / handle_client.
///
/// - NUS RX writes → rx_channel (picked up by BleStream::read)
/// - tx_channel packets → NUS TX notify (sent from BleStream::write)
/// - PubKey and CCCD writes are auto-accepted (stored in attribute table)
pub async fn run_gatt_connection(
    server: &'static ObdGattServer<'static>,
    conn: Connection<'_, super::MyPacketPool>,
    tx_channel: &'static BleChannel,
    rx_channel: &'static BleChannel,
) -> Result<(), BleError> {
    let gatt_conn = conn
        .with_attribute_server(server)
        .map_err(|_| BleError::GattError)?;

    // Receive GATT events; forward NUS RX writes to rx_channel.
    let gatt_task = async {
        loop {
            match gatt_conn.next().await {
                GattConnectionEvent::Disconnected { .. } => {
                    return Err(BleError::ChannelClosed);
                }
                GattConnectionEvent::Gatt {
                    event: GattEvent::Write(event),
                } => {
                    if event.handle() == server.nus.rx.handle {
                        let data = event.data();
                        let mut packet = BlePacket::new();
                        let _ = packet.extend_from_slice(&data[..data.len().min(MTU_SIZE)]);
                        // WriteEvent drops here → auto-accept (no ATT response for WriteWithoutResponse)
                        rx_channel.send(packet).await;
                    }
                    // PubKey writes and CCCD updates: drop → auto-accept, stored in attribute table
                }
                _ => {} // PhyUpdated, DataLengthUpdated, etc. — no action needed
            }
        }
        #[allow(unreachable_code)]
        Ok::<(), BleError>(())
    };

    // Drain tx_channel and send BLE notifications on NUS TX characteristic.
    // notify() is a silent no-op if the central hasn't subscribed (CCCD not set).
    let notify_task = async {
        loop {
            let packet = tx_channel.receive().await;
            let _ = server.nus.tx.notify(&gatt_conn, &packet).await;
        }
        #[allow(unreachable_code)]
        Ok::<(), BleError>(())
    };

    match select(gatt_task, notify_task).await {
        Either::First(res) => res,
        Either::Second(res) => res,
    }
}
