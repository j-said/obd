///! Модуль транспортного рівня ISO-TP (ISO 15765-2)
///!
///! Hardware Agnostic реалізація.
///! Підтримує роботу з множиною CAN-фреймів, використовуючи глобальний
///! прапорець режиму адресації (Standard/Extended).
use super::{AsyncCanDriver, is_extended};
use embassy_time::{Duration, with_timeout};
use embedded_can::{ExtendedId, Frame, Id, StandardId};
use heapless::Vec;

const ID_FUNCTIONAL_STANDARD: u32 = 0x7DF;
const ID_FUNCTIONAL_EXTENDED: u32 = 0x18DB33F1;
const PADDING_BYTE: u8 = 0xAA;

///! Помилки транспортного рівня
#[derive(Debug)]
pub enum IsoTpError {
    Timeout,
    BufferOverflow,
    InvalidSequence,
    DriverError,
    InvalidId,
}

#[derive(Debug, Clone)]
pub struct EcuResponse {
    pub id: u32,
    pub data: Vec<u8, 64>,
}

///! Generic handler. D - це драйвер (ESP32, STM32, Mock).
pub struct IsoTpHandler<D> {
    driver: D,
}

impl<D: AsyncCanDriver> IsoTpHandler<D> {
    pub fn new(driver: D) -> Self {
        Self { driver }
    }

    /// Метод для Physical Addressing (запит до конкретного ECU)
    pub async fn send_physical_request(
        &self,
        id: u32,
        data: &[u8],
    ) -> Result<Vec<u8, 64>, IsoTpError> {
        let ext = is_extended();
        self.transmit_sf(id, data, ext).await?;

        // Автоматично розраховуємо очікуваний ID відповіді
        let resp_id = if ext {
            // Extended ID: Змінюємо Source (Byte 0) і Target (Byte 1)
            // Запит: 0x18DA[Target][Source] -> Відповідь: 0x18DA[Source][Target]
            let target = (id >> 8) & 0xFF;
            let source = id & 0xFF;
            (id & 0xFFFF0000) | (source << 8) | target
        } else {
            // Standard: RequestID + 8 (напр. 7E0 -> 7E8)
            id + 8
        };

        self.receive_single(resp_id).await
    }

    /// Метод для Functional Addressing (запит до всіх ECU)
    pub async fn send_functional_request(
        &self,
        data: &[u8],
    ) -> Result<Vec<EcuResponse, 8>, IsoTpError> {
        let ext = is_extended();
        let id = if ext {
            ID_FUNCTIONAL_EXTENDED
        } else {
            ID_FUNCTIONAL_STANDARD
        };

        self.transmit_sf(id, data, ext).await?;
        self.collect_multiple(Duration::from_millis(100), Duration::from_millis(500))
            .await
    }

    /// Приватний метод відправки Single Frame (PCI + Data + Padding)
    async fn transmit_sf(&self, id: u32, data: &[u8], ext: bool) -> Result<(), IsoTpError> {
        let mut tx = [PADDING_BYTE; 8];

        if data.len() > 7 {
            return Err(IsoTpError::BufferOverflow);
        }

        tx[0] = data.len() as u8;
        tx[1..1 + data.len()].copy_from_slice(data);

        let can_id = if ext {
            Id::Extended(ExtendedId::new(id).ok_or(IsoTpError::InvalidId)?)
        } else {
            if id > 0x7FF {
                return Err(IsoTpError::InvalidId);
            }
            Id::Standard(StandardId::new(id as u16).ok_or(IsoTpError::InvalidId)?)
        };

        let frame = D::Frame::new(can_id, &tx).ok_or(IsoTpError::DriverError)?;

        self.driver
            .transmit(&frame)
            .await
            .map_err(|_| IsoTpError::DriverError)
    }

    /// Отримання одиночної відповіді
    async fn receive_single(&self, target_id: u32) -> Result<Vec<u8, 64>, IsoTpError> {
        let mut full_data: Vec<u8, 64> = Vec::new();
        let mut expected_len = 0;
        let mut next_sn = 1;

        with_timeout(Duration::from_millis(1000), async {
            loop {
                // Виклик через абстракцію
                let frame = self
                    .driver
                    .receive()
                    .await
                    .map_err(|_| IsoTpError::DriverError)?;

                let is_ext_mode = is_extended();
                // frame.id() - це метод трейту embedded_can
                let frame_is_ext = matches!(frame.id(), Id::Extended(_));

                if is_ext_mode != frame_is_ext {
                    continue;
                }

                let id = match frame.id() {
                    Id::Standard(s) => s.as_raw() as u32,
                    Id::Extended(e) => e.as_raw(),
                };

                if id != target_id {
                    continue;
                }

                let d = frame.data();
                if d.is_empty() {
                    continue;
                }

                match d[0] >> 4 {
                    0 => {
                        // Single Frame
                        let len = (d[0] & 0x0F) as usize;
                        if len == 0 || len > 7 {
                            continue;
                        }
                        full_data.extend_from_slice(&d[1..1 + len]).ok();
                        return Ok(full_data);
                    }
                    1 => {
                        // First Frame
                        expected_len = (((d[0] & 0x0F) as usize) << 8) | (d[1] as usize);
                        full_data.extend_from_slice(&d[2..]).ok();

                        let fc_id = if is_ext_mode {
                            target_id
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
        let is_ext_mode = is_extended();

        let _ = with_timeout(total_guard, async {
            loop {
                match with_timeout(inter_frame, self.driver.receive()).await {
                    Ok(Ok(frame)) => {
                        let frame_is_ext = matches!(frame.id(), Id::Extended(_));
                        if is_ext_mode != frame_is_ext {
                            continue;
                        }

                        let id = match frame.id() {
                            Id::Standard(s) => s.as_raw() as u32,
                            Id::Extended(e) => e.as_raw(),
                        };
                        // Фільтр діапазону відповідей OBD2
                        let valid_resp = if is_ext_mode {
                            // Для Extended OBD відповіді зазвичай мають формат 0x18DA....
                            (id & 0xFFFF0000) == 0x18DA0000
                        } else {
                            // Standard: діапазон 0x7E8..0x7EF
                            (0x7E8..=0x7EF).contains(&id)
                        };

                        if valid_resp {
                            let d = frame.data();
                            if !d.is_empty() {
                                let mut entry = Vec::new();
                                // Припускаємо Single Frame для discovery (більшість ECU відповідають коротко на broadcast)
                                let len = (d[0] & 0x0F) as usize;
                                if len <= 7 {
                                    entry.extend_from_slice(&d[1..1 + len]).ok();
                                    if !responses.iter().any(|r| r.id == id) {
                                        responses.push(EcuResponse { id, data: entry }).ok();
                                    }
                                }
                            }
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
    /// Flow Control: Continue To Send (CTS), BlockSize=0, STmin=0
    async fn send_flow_control(&self, request_id: u32) -> Result<(), IsoTpError> {
        let fc = [0x30, 0x00, 0x00, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA];
        let ext = is_extended();

        let can_id = if ext {
            Id::Extended(ExtendedId::new(request_id).ok_or(IsoTpError::InvalidId)?)
        } else {
            if request_id > 0x7FF {
                return Err(IsoTpError::InvalidId);
            }
            Id::Standard(StandardId::new(request_id as u16).ok_or(IsoTpError::InvalidId)?)
        };

        let frame = D::Frame::new(can_id, &fc).ok_or(IsoTpError::DriverError)?;
        self.driver
            .transmit(&frame)
            .await
            .map_err(|_| IsoTpError::DriverError)
    }
}
