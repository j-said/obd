use super::AsyncCanDriver;
use embassy_time::{Duration, with_timeout};
use embedded_can::{ExtendedId, Frame, Id, StandardId};
use heapless::Vec;

const PADDING_BYTE: u8 = 0xAA;
const FC_PCI_BYTE: u8 = 0x30;

const TIMEOUT_SINGLE: Duration = Duration::from_millis(1000);
const TIMEOUT_INTER_FRAME: Duration = Duration::from_millis(100);
const TIMEOUT_TOTAL: Duration = Duration::from_millis(500);

#[derive(Debug)]
pub enum IsoTpError {
    Timeout,
    BufferOverflow,
    InvalidSequence,
    DriverError,
    InvalidId,
}

#[derive(Debug, Clone, serde::Serialize)]
pub struct EcuResponse {
    pub id: u32,
    pub data: Vec<u8, 64>,
}

#[repr(u8)]
enum PciType {
    SingleFrame = 0,
    FirstFrame = 1,
    ConsecutiveFrame = 2,
    FlowControl = 3,
}

impl PciType {
    fn from_byte(b: u8) -> Option<Self> {
        match b >> 4 {
            0 => Some(Self::SingleFrame),
            1 => Some(Self::FirstFrame),
            2 => Some(Self::ConsecutiveFrame),
            3 => Some(Self::FlowControl),
            _ => None,
        }
    }
}

pub struct IsoTpHandler<D> {
    driver: D,
}

impl<D: AsyncCanDriver> IsoTpHandler<D> {
    pub fn new(driver: D) -> Self {
        Self { driver }
    }

    pub async fn send_physical_request(
        &self,
        target_id: Id,
        data: &[u8],
    ) -> Result<Vec<u8, 64>, IsoTpError> {
        self.transmit_sf(target_id, data).await?;

        let resp_id = match target_id {
            Id::Standard(s) => Id::Standard(StandardId::new(s.as_raw() + 8).unwrap()),
            Id::Extended(e) => {
                let id = e.as_raw();
                Id::Extended(
                    ExtendedId::new((id & 0xFFFF0000) | ((id & 0xFF) << 8) | ((id >> 8) & 0xFF))
                        .unwrap(),
                )
            }
        };

        self.receive_single(resp_id).await
    }

    pub async fn send_functional_request(
        &self,
        target_id: Id,
        data: &[u8],
    ) -> Result<Vec<EcuResponse, 8>, IsoTpError> {
        self.transmit_sf(target_id, data).await?;
        self.collect_multiple(
            matches!(target_id, Id::Extended(_)),
            TIMEOUT_INTER_FRAME,
            TIMEOUT_TOTAL,
        )
        .await
    }

    async fn transmit_sf(&self, id: Id, data: &[u8]) -> Result<(), IsoTpError> {
        if data.len() > 7 {
            return Err(IsoTpError::BufferOverflow);
        }

        let mut tx = [PADDING_BYTE; 8];
        tx[0] = data.len() as u8;
        tx[1..1 + data.len()].copy_from_slice(data);

        let frame = D::Frame::new(id, &tx).ok_or(IsoTpError::DriverError)?;

        self.driver
            .transmit(&frame)
            .await
            .map_err(|_| IsoTpError::DriverError)
    }

    async fn receive_single(&self, target_id: Id) -> Result<Vec<u8, 64>, IsoTpError> {
        let mut full_data: Vec<u8, 64> = Vec::new();
        let mut expected_len = 0;
        let mut next_sn = 1;

        with_timeout(TIMEOUT_SINGLE, async {
            loop {
                let frame = self
                    .driver
                    .receive()
                    .await
                    .map_err(|_| IsoTpError::DriverError)?;

                if frame.id() != target_id {
                    continue;
                }

                let d = frame.data();
                if d.is_empty() {
                    continue;
                }

                match PciType::from_byte(d[0]) {
                    Some(PciType::SingleFrame) => {
                        let len = (d[0] & 0x0F) as usize;
                        if len > 0 && len <= 7 {
                            full_data.extend_from_slice(&d[1..1 + len]).ok();
                            return Ok(full_data);
                        }
                    }
                    Some(PciType::FirstFrame) => {
                        expected_len = (((d[0] & 0x0F) as usize) << 8) | (d[1] as usize);
                        full_data.extend_from_slice(&d[2..]).ok();

                        let fc_id = match target_id {
                            Id::Standard(s) => {
                                Id::Standard(StandardId::new(s.as_raw() - 8).unwrap())
                            }
                            Id::Extended(_) => target_id,
                        };
                        self.send_flow_control(fc_id).await?;
                    }
                    Some(PciType::ConsecutiveFrame) => {
                        if (d[0] & 0x0F) != next_sn {
                            continue;
                        }
                        let to_copy = core::cmp::min(expected_len - full_data.len(), 7);
                        full_data.extend_from_slice(&d[1..1 + to_copy]).ok();

                        if full_data.len() >= expected_len {
                            return Ok(full_data);
                        }
                        next_sn = (next_sn + 1) % 16;
                    }
                    _ => continue,
                }
            }
        })
        .await
        .map_err(|_| IsoTpError::Timeout)?
    }

    async fn collect_multiple(
        &self,
        is_ext_mode: bool,
        inter_frame: Duration,
        total_guard: Duration,
    ) -> Result<Vec<EcuResponse, 8>, IsoTpError> {
        let mut responses: Vec<EcuResponse, 8> = Vec::new();

        let _ = with_timeout(total_guard, async {
            loop {
                if let Ok(Ok(frame)) = with_timeout(inter_frame, self.driver.receive()).await {
                    let (id, is_ext) = match frame.id() {
                        Id::Standard(s) => (s.as_raw() as u32, false),
                        Id::Extended(e) => (e.as_raw(), true),
                    };

                    if is_ext_mode != is_ext {
                        continue;
                    }

                    let valid_resp = if is_ext_mode {
                        (id & 0xFFFF0000) == 0x18DA0000
                    } else {
                        (0x7E8..=0x7EF).contains(&id)
                    };

                    if valid_resp {
                        let d = frame.data();
                        if !d.is_empty() {
                            let len = (d[0] & 0x0F) as usize;
                            if len <= 7
                                && PciType::from_byte(d[0])
                                    .map_or(false, |p| matches!(p, PciType::SingleFrame))
                            {
                                let mut entry = Vec::new();
                                entry.extend_from_slice(&d[1..1 + len]).ok();
                                if !responses.iter().any(|r| r.id == id) {
                                    responses.push(EcuResponse { id, data: entry }).ok();
                                }
                            }
                        }
                    }
                } else {
                    break;
                }
            }
        })
        .await;

        if responses.is_empty() {
            Err(IsoTpError::Timeout)
        } else {
            Ok(responses)
        }
    }

    async fn send_flow_control(&self, target_id: Id) -> Result<(), IsoTpError> {
        let fc = [
            FC_PCI_BYTE,
            0x00,
            0x00,
            PADDING_BYTE,
            PADDING_BYTE,
            PADDING_BYTE,
            PADDING_BYTE,
            PADDING_BYTE,
        ];
        let frame = D::Frame::new(target_id, &fc).ok_or(IsoTpError::DriverError)?;
        self.driver
            .transmit(&frame)
            .await
            .map_err(|_| IsoTpError::DriverError)
    }
}
