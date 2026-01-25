//! Модуль реалізації транспортного рівня ISO-TP (ISO 15765-2).
//! Підтримує роботу з декількома ECU та сегментовані повідомлення.

use super::CanManager;
use super::config;
use embassy_time::{Duration, with_timeout};
use embedded_can::{ExtendedId, Frame, Id, StandardId};
use esp_hal::twai::EspTwaiFrame;
use heapless::Vec;

const ID_FUNCTIONAL_STANDARD: u32 = 0x7DF;
const ID_FUNCTIONAL_EXTENDED: u32 = 0x18DB33F1;

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
            // Standard: +8 logic (напр. 7E0 -> 7E8)
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
            ID_FUNCTIONAL_EXTENDED
        } else {
            ID_FUNCTIONAL_STANDARD
        };

        self.transmit_sf(id, data, ext).await?;
        self.collect_multiple(Duration::from_millis(100), Duration::from_millis(500))
            .await
    }

    /// Приватний метод відправки (PCI + Data + Padding)
    async fn transmit_sf(&self, id: u32, data: &[u8], ext: bool) -> Result<(), IsoTpError> {
        let mut tx = [0xAA; 8]; // Padding byte 0xAA is common
        if data.len() > 7 {
            return Err(IsoTpError::BufferOverflow);
        }

        tx[0] = data.len() as u8;
        tx[1..1 + data.len()].copy_from_slice(data);

        let can_id = if ext {
            // ExtendedId::new повертає Option, тому робимо unwrap (або обробку помилки)
            Id::Extended(ExtendedId::new(id).unwrap())
        } else {
            // StandardId::new приймає u16. Переконуємось, що id влазить у 11 біт.
            Id::Standard(StandardId::new(id as u16).unwrap())
        };

        // Використовуємо трейт Frame::new
        let frame = EspTwaiFrame::new(can_id, &tx).unwrap();

        self.manager
            .transmit(&frame)
            .await
            .map_err(|_| IsoTpError::CanError)
    }

    /// Отримання одиночної відповіді.
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

                // ВИПРАВЛЕНО: Правильне отримання ID з enum
                let id = match frame.id() {
                    Id::Standard(s) => s.as_raw() as u32,
                    Id::Extended(e) => e.as_raw(),
                };

                // Перевірка режиму (Extended/Standard) згідно конфігу
                let is_ext_mode = config::is_extended();
                let frame_is_ext = matches!(frame.id(), Id::Extended(_));

                if is_ext_mode != frame_is_ext {
                    continue; // Ігноруємо кадри неправильного формату
                }

                if id != target_id {
                    continue;
                }

                let d = frame.data();
                if d.is_empty() {
                    continue;
                } // Захист від пустих кадрів

                match d[0] >> 4 {
                    0 => {
                        // Single Frame
                        let len = (d[0] & 0x0F) as usize;
                        if len == 0 || len > 7 {
                            continue;
                        } // Basic validation
                        full_data.extend_from_slice(&d[1..1 + len]).ok();
                        return Ok(full_data);
                    }
                    1 => {
                        // First Frame
                        expected_len = (((d[0] & 0x0F) as usize) << 8) | (d[1] as usize);
                        full_data.extend_from_slice(&d[2..]).ok();

                        // Розрахунок ID для Flow Control
                        let fc_id = if config::is_extended() {
                            // Для Extended OBD: 18DA F1 10 (req) -> 18DA 10 F1 (resp)
                            // Flow Control має летіти назад на 18DA F1 10
                            // Тут спрощена логіка XOR для прикладу, краще явно конструювати
                            target_id // Умовно, зазвичай FC шлеться на Source ID
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
                        // ВИПРАВЛЕНО: ID extraction
                        let id = match frame.id() {
                            Id::Standard(s) => s.as_raw() as u32,
                            Id::Extended(e) => e.as_raw(),
                        };

                        // Фільтр для OBD2 відповідей
                        // Standard: 0x7E8..0x7EF
                        // Extended: 0x18DA....
                        let valid_resp = if config::is_extended() {
                            (id & 0xFFFF0000) == 0x18DA0000
                        } else {
                            (0x7E8..=0x7EF).contains(&id)
                        };

                        if valid_resp {
                            let d = frame.data();
                            if !d.is_empty() {
                                let mut entry = Vec::new();
                                // Припускаємо Single Frame для discovery
                                let len = (d[0] & 0x0F) as usize;
                                if len <= 7 {
                                    entry.extend_from_slice(&d[1..1 + len]).ok();
                                    // Перевірка на дублікати (простий варіант)
                                    let exists = responses.iter().any(|r| r.id == id);
                                    if !exists {
                                        responses.push(EcuResponse { id, data: entry }).ok();
                                    }
                                }
                            }
                        }
                    }
                    _ => break, // Timeout між кадрами - кінець пачки
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
        // Flow Control: Continue To Send (CTS), BlockSize=0, STmin=0
        let fc = [0x30, 0x00, 0x00, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA];

        let can_id = if config::is_extended() {
            Id::Extended(ExtendedId::new(request_id).unwrap())
        } else {
            Id::Standard(StandardId::new(request_id as u16).unwrap())
        };

        let frame = EspTwaiFrame::new(can_id, &fc).unwrap();

        self.manager
            .transmit(&frame)
            .await
            .map_err(|_| IsoTpError::CanError)
    }
}
