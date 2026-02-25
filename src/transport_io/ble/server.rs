use super::{BleChannel, BleError, BlePacket, ObdStack, DEVICE_NAME, MTU_SIZE};
use trouble_host::advertise::{AdStructure, Advertisement};
use embassy_futures::join::join;
use trouble_host::prelude::*;

pub fn create_advertisement<'a>(buf: &'a mut [u8]) -> Result<Advertisement<'a>, BleError> {
    let structures = [
        AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
        AdStructure::CompleteLocalName(DEVICE_NAME.as_bytes()),
    ];

    let len = AdStructure::encode_slice(&structures, buf).map_err(|_| BleError::AdvertisingError)?;

    Ok(Advertisement::ConnectableScannableUndirected {
        adv_data: &buf[..len],
        scan_data: &[],
    })
}

pub async fn run_connection(
    stack: &'static ObdStack, 
    conn: &Connection<'_, super::MyPacketPool>,
    tx_channel: &'static BleChannel,
    rx_channel: &'static BleChannel,
) -> Result<(), BleError> {
    let l2cap = trouble_host::l2cap::L2capChannel::accept(
        stack,
        conn,
        &[0x0080],
        &trouble_host::l2cap::L2capChannelConfig::default(),
    )
    .await
    .map_err(|_| BleError::L2capError)?;

    let (mut tx_socket, mut rx_socket) = l2cap.split();

    let rx_task = async {
        let mut buf = [0u8; MTU_SIZE];
        loop {
            let len = rx_socket.receive(stack, &mut buf).await.map_err(|_| BleError::L2capError)?;
            if len == 0 { return Err(BleError::ChannelClosed); }
            
            let mut packet = BlePacket::new();
            packet.extend_from_slice(&buf[..len]).map_err(|_| BleError::MtuExceeded)?;
            rx_channel.send(packet).await;
        }
        #[allow(unreachable_code)]
        Ok::<(), BleError>(())
    };

    let tx_task = async {
        loop {
            let packet = tx_channel.receive().await;
            tx_socket.send(stack, &packet).await.map_err(|_| BleError::L2capError)?;
        }
        #[allow(unreachable_code)]
        Ok::<(), BleError>(())
    };

    match join(rx_task, tx_task).await {
        (Err(e), _) | (_, Err(e)) => Err(e),
        _ => Ok(()),
    }
}