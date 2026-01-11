//! Модуль реалізації транспортного рівня ISO-TP (ISO 15765-2).
//! Підтримує роботу з декількома ECU та сегментовані повідомлення.

use super::config;
use super::mod_::CanManager;
use embassy_time::{Duration, with_timeout};
use esp_hal::twai::EspTwaiFrame;
use heapless::Vec;

#[derive(Debug)]
pub enum IsoTpError {
    Timeout,
    BufferOverflow,
    InvalidSequence,
    CanError,
}

/// Відповідь від конкретного електронного блоку керування.
#[derive(Debug, Clone)]
pub struct EcuResponse {
    pub id: u32,
    pub data: Vec<u8, 64>,
}

pub struct IsoTpHandler<'a> {
    manager: &'a CanManager<'a>,
}
impl<'a> IsoTpHandler<'a> {
    pub fn new(manager: &'a CanManager<'a>) -> Self {
        Self { manager }
    }

    /// Універсальний метод для Physical Addressing
    pub async fn send_request(&self, id: u32, data: &[u8]) -> Result<Vec<u8, 64>, IsoTpError> {
        let ext = config::is_extended();
        self.transmit_sf(id, data, ext).await?;

        // Автоматично розраховуємо ID відповіді
        let resp_id = if ext {
            // Extended: Swap Source (Byte 0) and Target (Byte 1)
            // Request: 0x18DA[Target][Source] -> Response: 0x18DA[Source][Target]
            let target = (id >> 8) & 0xFF;
            let source = id & 0xFF;
            (id & 0xFFFF0000) | (source << 8) | target
        } else {
            // Standard: +8 logic
            id + 8
        };
        self.receive_single(resp_id).await
    }

    /// Універсальний метод для Functional Addressing
    pub async fn send_functional_request(
        &self,
        data: &[u8],
    ) -> Result<Vec<EcuResponse, 8>, IsoTpError> {
        let ext = config::is_extended();
        let id = if ext {
            config::ID_FUNCTIONAL_EXTENDED
        } else {
            config::ID_FUNCTIONAL_STANDARD
        };
        self.transmit_sf(id, data, ext).await?;
        self.collect_multiple(Duration::from_millis(100), Duration::from_millis(500))
            .await
    }

    /// Приватний метод відправки (PCI + Data + Padding)
    async fn transmit_sf(&self, id: u32, data: &[u8], ext: bool) -> Result<(), IsoTpError> {
        let mut tx = [0xAA; 8];
        tx[0] = data.len() as u8;
        tx[1..1 + data.len()].copy_from_slice(data);

        let frame = if ext {
            EspTwaiFrame::new_extended(id, &tx).unwrap()
        } else {
            EspTwaiFrame::new_standard(id, &tx).unwrap()
        };
        self.manager
            .transmit(&frame)
            .await
            .map_err(|_| IsoTpError::CanError)
    }
    /// Отримання одиночної відповіді. Підтримує Standard/Extended та Multi-frame.
    async fn receive_single(&self, target_id: u32) -> Result<Vec<u8, 64>, IsoTpError> {
        let mut full_data: Vec<u8, 64> = Vec::new();
        let mut expected_len = 0;
        let mut next_sn = 1;

        with_timeout(Duration::from_millis(1000), async {
            loop {
                let frame = self
                    .manager
                    .receive()
                    .await
                    .map_err(|_| IsoTpError::CanError)?;

                // Визначаємо ID згідно з глобальним конфігом
                let id = if config::is_extended() {
                    frame.id().as_extended().unwrap_or(0)
                } else {
                    frame.id().as_standard().unwrap_or(0)
                };

                if id != target_id {
                    continue;
                }

                let d = frame.data();
                match d[0] >> 4 {
                    0 => {
                        // Single Frame
                        let len = (d[0] & 0x0F) as usize;
                        full_data.extend_from_slice(&d[1..1 + len]).ok();
                        return Ok(full_data);
                    }
                    1 => {
                        // First Frame
                        expected_len = (((d[0] & 0x0F) as usize) << 8) | (d[1] as usize);
                        full_data.extend_from_slice(&d[2..]).ok();

                        // Розрахунок ID для Flow Control
                        let fc_id = if config::is_extended() {
                            target_id ^ 0x8
                        } else {
                            target_id - 8
                        };
                        self.send_flow_control(fc_id).await?;
                    }
                    2 => {
                        // Consecutive Frame
                        if (d[0] & 0x0F) != next_sn {
                            return Err(IsoTpError::InvalidSequence);
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

    /// Збір відповідей від декількох блоків (Rolling Timeout)
    async fn collect_multiple(
        &self,
        inter_frame: Duration,
        total_guard: Duration,
    ) -> Result<Vec<EcuResponse, 8>, IsoTpError> {
        let mut responses: Vec<EcuResponse, 8> = Vec::new();
        let _ = with_timeout(total_guard, async {
            loop {
                match with_timeout(inter_frame, self.manager.receive()).await {
                    Ok(Ok(frame)) => {
                        let id = if config::is_extended() {
                            frame.id().as_extended().unwrap_or(0)
                        } else {
                            frame.id().as_standard().unwrap_or(0)
                        };
                        if (0x7E8..=0x7EF).contains(&id) || (id & 0x18DA0000 == 0x18DA0000) {
                            let d = frame.data();
                            let mut entry = Vec::new();
                            let len = (d[0] & 0x0F) as usize;
                            entry.extend_from_slice(&d[1..1 + len]).ok();
                            responses.push(EcuResponse { id, data: entry }).ok();
                        }
                    }
                    _ => break,
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

    /// Відправка Flow Control кадру
    async fn send_flow_control(&self, request_id: u32) -> Result<(), IsoTpError> {
        let fc = [0x30, 0x00, 0x00, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA];
        let frame = if config::is_extended() {
            EspTwaiFrame::new_extended(request_id, &fc).unwrap()
        } else {
            EspTwaiFrame::new_standard(request_id, &fc).unwrap()
        };
        self.manager
            .transmit(&frame)
            .await
            .map_err(|_| IsoTpError::CanError)
    }
}
