use super::ad_table::create_advertisement;
use super::error::BleError;
use super::types::{BleChannel, BlePacket, MyBleStack, MTU_SIZE};
use embassy_futures::join::join;
use trouble_host::prelude::*;

pub struct BleOrchestrator<'a, C: Controller> {
    stack: &'a MyBleStack<'a, C>,
    tx_channel: &'a BleChannel,
    rx_channel: &'a BleChannel,
}

impl<'a, C: Controller> BleOrchestrator<'a, C> {
    pub fn new(
        stack: &'a MyBleStack<'a, C>,
        tx_channel: &'a BleChannel,
        rx_channel: &'a BleChannel,
    ) -> Self {
        Self {
            stack,
            tx_channel,
            rx_channel,
        }
    }

    pub async fn run(&mut self) -> Result<(), BleError> {
        loop {
            let mut ad_buf = [0u8; 31];
            let advertisement = create_advertisement(&mut ad_buf)?;
            let params = trouble_host::prelude::AdvertisementParameters::default();

            let conn = {
                let mut peripheral = self.stack.build().peripheral;

                #[allow(unused_mut)] // TODO: check if has to be mutable
                let mut advertiser = peripheral
                    .advertise(&params, advertisement)
                    .await
                    .map_err(|_| BleError::AdvertisingError)?;

                advertiser
                    .accept()
                    .await
                    .map_err(|_| BleError::ConnectionFailed)?
            };

            let _ = self.run_l2cap(&conn).await;
        }
    }

    async fn run_l2cap(
        &self,
        conn: &Connection<'_, super::types::MyPacketPool>,
    ) -> Result<(), BleError> {
        #[allow(unused_mut)] // TODO: check if has to be mutable
        let mut l2cap = trouble_host::l2cap::L2capChannel::accept(
            self.stack,
            conn,
            &[0x0080],
            &trouble_host::l2cap::L2capChannelConfig::default(),
        )
        .await
        .map_err(|_| BleError::L2capError)?;

        let (mut tx_socket, mut rx_socket) = l2cap.split();

        match join(self.pump_in(&mut rx_socket), self.pump_out(&mut tx_socket)).await {
            (Err(e), _) | (_, Err(e)) => Err(e),
            _ => Ok(()),
        }
    }

    async fn pump_in(
        &self,
        socket: &mut L2capChannelReader<'_, super::types::MyPacketPool>,
    ) -> Result<(), BleError> {
        let mut buf = [0u8; MTU_SIZE];
        loop {
            let len = socket
                .receive(&self.stack, &mut buf)
                .await
                .map_err(|_| BleError::L2capError)?;
            if len == 0 {
                return Err(BleError::ChannelClosed);
            }

            let mut packet = BlePacket::new();
            packet
                .extend_from_slice(&buf[..len])
                .map_err(|_| BleError::MtuExceeded)?;
            self.rx_channel.send(packet).await;
        }
    }

    async fn pump_out(
        &self,
        socket: &mut L2capChannelWriter<'_, super::types::MyPacketPool>,
    ) -> Result<(), BleError> {
        loop {
            let packet = self.tx_channel.receive().await;
            socket
                .send(&self.stack, &packet)
                .await
                .map_err(|_| BleError::L2capError)?;
        }
    }
}
