use super::AsyncCanDriver;
use embassy_time::{Duration, with_timeout};
use embedded_can::{ExtendedId, Frame, Id, StandardId};
use heapless::Vec;

// FC = Flow Control
// PCI = Protocol Control Information
// cf = Consecutive Frame
// FF = First Frame
// SF = Single Frame
// SN = Sequence Number

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

struct TransferState {
    id: u32,
    expected_len: usize,
    next_sn: u8,
    buffer: Vec<u8, 64>,
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

    fn get_fc_id(&self, target_id: Id) -> Result<Id, IsoTpError> {
        match target_id {
            Id::Standard(std) => {
                let raw = std.as_raw();
                if raw < 8 {
                    return Err(IsoTpError::InvalidId);
                }
                Ok(Id::Standard(StandardId::new(raw - 8).unwrap()))
            }
            Id::Extended(ext) => {
                Ok(Id::Extended(ExtendedId::new(self.swap_ext_addr(ext.as_raw())).unwrap()))
            }
        }
    }

    fn swap_ext_addr(&self, id: u32) -> u32 {
        (id & 0xFFFF0000) | ((id & 0xFF) << 8) | ((id >> 8) & 0xFF)
    }

    pub async fn send_physical_request(
        &self,
        target_id: Id,
        data: &[u8],
    ) -> Result<Vec<u8, 64>, IsoTpError> {
        let resp_id = match target_id {
            Id::Standard(s) => Id::Standard(StandardId::new(s.as_raw() + 8).unwrap()),
            Id::Extended(e) => {
                Id::Extended(ExtendedId::new(self.swap_ext_addr(e.as_raw())).unwrap())
            }
        };
        self.transmit_sf(target_id, data).await?;
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
        let mut state = TransferState {
            id: 0,
            expected_len: 0,
            next_sn: 1,
            buffer: Vec::new(),
        };
        with_timeout(TIMEOUT_SINGLE, self.receive_loop(&mut state, target_id))
            .await
            .map_err(|_| IsoTpError::Timeout)?
    }

    async fn receive_loop(
        &self,
        state: &mut TransferState,
        target_id: Id,
    ) -> Result<Vec<u8, 64>, IsoTpError> {
        loop {
            let frame = self
                .driver
                .receive()
                .await
                .map_err(|_| IsoTpError::DriverError)?;
            if frame.id() == target_id && self.process_frame(state, &frame).await? {
                return Ok(core::mem::replace(&mut state.buffer, Vec::new()));
            }
        }
    }

    async fn process_frame(
        &self,
        state: &mut TransferState,
        frame: &D::Frame,
    ) -> Result<bool, IsoTpError> {
        let d = frame.data();
        if d.is_empty() {
            return Ok(false);
        }
        match PciType::from_byte(d[0]) {
            Some(PciType::SingleFrame) => self.handle_sf(state, d),
            Some(PciType::FirstFrame) => self.handle_ff(state, frame.id(), d).await,
            Some(PciType::ConsecutiveFrame) => self.handle_cf(state, d),
            _ => Ok(false),
        }
    }

    async fn handle_ff(
        &self,
        state: &mut TransferState,
        id: Id,
        d: &[u8],
    ) -> Result<bool, IsoTpError> {
        if d.len() < 3 {
            return Ok(false);
        }
        state.expected_len = (((d[0] & 0x0F) as usize) << 8) | (d[1] as usize);
        state.buffer.extend_from_slice(&d[2..]).map_err(|_| IsoTpError::BufferOverflow)?;
        self.send_flow_control(self.get_fc_id(id)?).await?;
        Ok(false)
    }

    fn handle_sf(&self, state: &mut TransferState, d: &[u8]) -> Result<bool, IsoTpError> {
        let len = (d[0] & 0x0F) as usize;
        if len == 0 || len > 7 {
            return Ok(false);
        }
        if d.len() < 1 + len {
            return Ok(false);
        }
        state.buffer.extend_from_slice(&d[1..1 + len]).map_err(|_| IsoTpError::BufferOverflow)?;
        Ok(true)
    }

    fn handle_cf(&self, state: &mut TransferState, d: &[u8]) -> Result<bool, IsoTpError> {
        if d.is_empty() || (d[0] & 0x0F) != state.next_sn {
            return Ok(false);
        }
        let to_copy = core::cmp::min(state.expected_len - state.buffer.len(), 7);
        state.buffer.extend_from_slice(&d[1..1 + to_copy]).map_err(|_| IsoTpError::BufferOverflow)?;
        state.next_sn = (state.next_sn + 1) % 16;
        Ok(state.buffer.len() >= state.expected_len)
    }

    pub async fn collect_multiple(
        &self,
        is_ext: bool,
        inter: Duration,
        total: Duration,
    ) -> Result<Vec<EcuResponse, 8>, IsoTpError> {
        let mut res: Vec<EcuResponse, 8> = Vec::new();
        let mut states: Vec<TransferState, 8> = Vec::new();
        let _ = with_timeout(
            total,
            self.collection_loop(&mut res, &mut states, is_ext, inter),
        )
        .await;
        if res.is_empty() {
            Err(IsoTpError::Timeout)
        } else {
            Ok(res)
        }
    }

    async fn collection_loop(
        &self,
        res: &mut Vec<EcuResponse, 8>,
        states: &mut Vec<TransferState, 8>,
        is_ext: bool,
        inter: Duration,
    ) {
        loop {
            let frame = with_timeout(inter, self.driver.receive()).await;
            let Ok(Ok(f)) = frame else {
                break;
            };
            self.handle_collection_step(res, states, f, is_ext).await;
        }
    }

    async fn handle_collection_step(
        &self,
        res: &mut Vec<EcuResponse, 8>,
        states: &mut Vec<TransferState, 8>,
        f: D::Frame,
        is_ext_mode: bool,
    ) {
        let (id_raw, is_f_ext) = match f.id() {
            Id::Standard(std) => (std.as_raw() as u32, false),
            Id::Extended(ext) => (ext.as_raw(), true),
        };
        if is_ext_mode != is_f_ext || !self.is_valid_resp(id_raw, is_ext_mode) {
            return;
        }
        let state = self.get_or_create_state(states, id_raw);
        if let Some(s) = state {
            if self.process_frame(s, &f).await.unwrap_or(false) {
                let data = core::mem::replace(&mut s.buffer, Vec::new());
                s.expected_len = 0;
                s.next_sn = 1;
                res.push(EcuResponse { id: id_raw, data }).ok();
            }
        }
    }

    fn get_or_create_state<'a>(
        &self,
        states: &'a mut Vec<TransferState, 8>,
        id: u32,
    ) -> Option<&'a mut TransferState> {
        if let Some(idx) = states.iter().position(|s| s.id == id) {
            return Some(&mut states[idx]);
        }
        states
            .push(TransferState {
                id,
                expected_len: 0,
                next_sn: 1,
                buffer: Vec::new(),
            })
            .ok()?;
        states.last_mut()
    }

    fn is_valid_resp(&self, id: u32, is_ext: bool) -> bool {
        if is_ext {
            (id & 0xFFFF0000) == 0x18DA0000
        } else {
            (0x7E8..=0x7EF).contains(&id)
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

// TODO: Add support for multi-frame transmission (currently only single frame is supported)
// TODO: Add support for functional requests (currently only physical requests are supported)
// TODO: Add better error handling and reporting (currently just returns a generic error)
// TODO: Add support for extended addressing (currently only standard addressing is supported)
// TODO: Add support for different flow control options (currently just sends a basic flow control frame)
// TODO: Add support for timing parameters (currently uses fixed timeouts)
// TODO: Add support for concurrent transfers (currently assumes only one transfer at a time)
// TODO: Add support for cancellation of transfers (currently no way to cancel an ongoing transfer)

// TODO: Add iso-tp feutures build flags 