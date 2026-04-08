use super::AsyncCanDriver;
use embassy_time::{Duration, Timer, with_timeout};
use embedded_can::{ExtendedId, Frame, Id, StandardId};
use heapless::Vec;

const PADDING_BYTE: u8 = 0xCC;

/// ISO 15765-2 §6.7.2 — sender TX-confirm timeout
const N_AS_TIMEOUT: Duration = Duration::from_millis(1000);
/// ISO 15765-2 §6.7.2 — sender wait-for-FC timeout
const N_BS_TIMEOUT: Duration = Duration::from_millis(1000);
/// ISO 15765-2 §6.7.2 — receiver wait-for-CF timeout
const N_CR_TIMEOUT: Duration = Duration::from_millis(1000);
/// ISO 15765-2 §6.7.6 — maximum consecutive FC.WAIT frames before abort
const N_WFTMAX: u8 = 10;

const TIMEOUT_INTER_FRAME: Duration = Duration::from_millis(100);
const TIMEOUT_TOTAL: Duration = Duration::from_millis(500);

#[repr(u8)]
enum FlowStatus {
    ContinueToSend = 0,
    Wait = 1,
    Overflow = 2,
}

impl FlowStatus {
    fn from_nibble(n: u8) -> Result<Self, IsoTpError> {
        match n & 0x0F {
            0 => Ok(Self::ContinueToSend),
            1 => Ok(Self::Wait),
            2 => Ok(Self::Overflow),
            _ => Err(IsoTpError::InvalidFs),
        }
    }
}

/// ISO 15765-2 §7 — network layer addressing modes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AddressingMode {
    /// Normal addressing: PCI at byte[0], max SF payload 7 bytes
    Normal,
    /// Extended addressing: byte[0] = N_TA, PCI at byte[1], max SF payload 6 bytes
    Extended,
}

#[derive(Debug)]
pub enum IsoTpError {
    /// N_As / N_Ar: TX or RX frame acknowledge timeout
    TimeoutA,
    /// N_Bs: no Flow Control received after First Frame was sent
    TimeoutBs,
    /// N_Cr: no Consecutive Frame received within the inter-frame window
    TimeoutCr,
    /// CF arrived with an unexpected sequence number
    WrongSn,
    /// FC carried a reserved FlowStatus value (3–F)
    InvalidFs,
    /// N_WFTmax consecutive FC.WAIT frames received
    WftOverrun,
    BufferOverflow,
    DriverError,
    InvalidId,
}

// TODO: make buffer capacity a const generic on IsoTpHandler<D, const N: usize>
//       so callers can trade memory for max PDU size (ISO-TP allows up to 4095 bytes).
#[derive(Debug, Clone, serde::Serialize)]
pub struct EcuResponse {
    pub id: u32,
    pub data: Vec<u8, 256>,
}

struct TransferState {
    id: u32,
    /// FF_DL from the First Frame header (ISO 15765-2 §9.6.2.2)
    rx_dl: usize,
    next_sn: u8,
    buffer: Vec<u8, 256>,
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
    addressing: AddressingMode,
    /// N_TA byte prepended to every frame in Extended addressing mode
    target_addr: u8,
}

impl<D: AsyncCanDriver> IsoTpHandler<D> {
    pub fn new(driver: D) -> Self {
        Self { driver, addressing: AddressingMode::Normal, target_addr: 0 }
    }

    pub fn new_extended(driver: D, target_addr: u8) -> Self {
        Self { driver, addressing: AddressingMode::Extended, target_addr }
    }

    /// Byte offset at which the PCI byte starts (0 for Normal, 1 for Extended).
    fn pci_offset(&self) -> usize {
        match self.addressing {
            AddressingMode::Normal => 0,
            AddressingMode::Extended => 1,
        }
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
            Id::Extended(ext) => Ok(Id::Extended(
                ExtendedId::new(self.swap_ext_addr(ext.as_raw())).unwrap(),
            )),
        }
    }

    fn swap_ext_addr(&self, id: u32) -> u32 {
        (id & 0xFFFF0000) | ((id & 0xFF) << 8) | ((id >> 8) & 0xFF)
    }

    pub async fn send_physical_request(
        &self,
        target_id: Id,
        data: &[u8],
    ) -> Result<Vec<u8, 256>, IsoTpError> {
        let resp_id = match target_id {
            Id::Standard(s) => Id::Standard(StandardId::new(s.as_raw() + 8).unwrap()),
            Id::Extended(e) => {
                Id::Extended(ExtendedId::new(self.swap_ext_addr(e.as_raw())).unwrap())
            }
        };
        // 5.4: route to multi-frame TX when payload exceeds SF capacity
        if data.len() <= 7 {
            self.transmit_sf(target_id, data).await?;
        } else {
            self.transmit_multi(target_id, resp_id, data).await?;
        }
        self.receive_single(resp_id).await
    }

    pub async fn send_functional_request(
        &self,
        target_id: Id,
        data: &[u8],
    ) -> Result<Vec<EcuResponse, 8>, IsoTpError> {
        // 5.5: functional addressing only supports SF (ISO 15765-2 §9.2)
        if data.len() > 7 {
            return Err(IsoTpError::BufferOverflow);
        }
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

    /// Transmit a multi-frame message via FF + CF sequence (ISO 15765-2 §9.6).
    /// `tx_id`  — CAN ID we send on.
    /// `fc_id`  — CAN ID we expect FC frames from (usually `resp_id`).
    async fn transmit_multi(&self, tx_id: Id, fc_id: Id, data: &[u8]) -> Result<(), IsoTpError> {
        let len = data.len();
        // ISO 15765-2 §9.6.2.2: FF_DL is 12-bit, max 4095
        if len > 0xFFF {
            return Err(IsoTpError::BufferOverflow);
        }

        // 5.2: build and transmit FF
        let mut ff = [PADDING_BYTE; 8];
        ff[0] = (PciType::FirstFrame as u8) << 4 | ((len >> 8) as u8 & 0x0F);
        ff[1] = (len & 0xFF) as u8;
        ff[2..8].copy_from_slice(&data[..6]);
        let frame = D::Frame::new(tx_id, &ff).ok_or(IsoTpError::DriverError)?;
        with_timeout(N_AS_TIMEOUT, self.driver.transmit(&frame))
            .await
            .map_err(|_| IsoTpError::TimeoutA)?
            .map_err(|_| IsoTpError::DriverError)?;

        // 5.2: await FC(CTS) — receive_fc handles WAIT/OVFLW/reserved internally
        let mut wft_count = 0u8;
        let (mut bs, st_min) = self.receive_fc(fc_id, &mut wft_count).await?;
        let mut st_min_dur = Self::st_min_duration(st_min);

        // 5.3: transmit CF sequence
        let mut sn: u8 = 1;
        let mut block_count: u8 = 0;
        let mut offset: usize = 6; // bytes already sent in FF

        while offset < len {
            let mut cf = [PADDING_BYTE; 8];
            cf[0] = (PciType::ConsecutiveFrame as u8) << 4 | (sn & 0x0F);
            let chunk_end = (offset + 7).min(len);
            cf[1..1 + (chunk_end - offset)].copy_from_slice(&data[offset..chunk_end]);

            let frame = D::Frame::new(tx_id, &cf).ok_or(IsoTpError::DriverError)?;
            with_timeout(N_AS_TIMEOUT, self.driver.transmit(&frame))
                .await
                .map_err(|_| IsoTpError::TimeoutA)?
                .map_err(|_| IsoTpError::DriverError)?;

            sn = (sn + 1) % 16;
            offset += 7;
            block_count += 1;

            if bs > 0 && block_count == bs {
                // Block exhausted — await next FC before continuing
                let mut wft_count = 0u8;
                let (new_bs, new_st_min) = self.receive_fc(fc_id, &mut wft_count).await?;
                bs = new_bs;
                st_min_dur = Self::st_min_duration(new_st_min);
                block_count = 0;
            } else if offset < len {
                // STmin separation between consecutive CFs (§6.5.4)
                Timer::after(st_min_dur).await;
            }
        }

        Ok(())
    }

    /// Convert FC STmin byte to a `Duration` (ISO 15765-2 §9.6.5.5).
    ///
    /// | Value      | Meaning                          |
    /// |------------|----------------------------------|
    /// | 0x00       | 0 ms                             |
    /// | 0x01–0x7F  | 1–127 ms (1 ms resolution)       |
    /// | 0xF1–0xF9  | 100–900 µs (100 µs resolution)   |
    /// | 0x80–0xF0, 0xFA–0xFF | reserved → 127 ms (max) |
    fn st_min_duration(st_min: u8) -> Duration {
        match st_min {
            0x00 => Duration::from_millis(0),
            v @ 0x01..=0x7F => Duration::from_millis(v as u64),
            v @ 0xF1..=0xF9 => Duration::from_micros((v - 0xF0) as u64 * 100),
            _ => Duration::from_millis(127), // reserved — use maximum per spec
        }
    }

    async fn receive_single(&self, target_id: Id) -> Result<Vec<u8, 256>, IsoTpError> {
        let mut state = TransferState {
            id: 0,
            rx_dl: 0,
            next_sn: 1,
            buffer: Vec::new(),
        };
        self.receive_loop(&mut state, target_id).await
    }

    async fn receive_loop(
        &self,
        state: &mut TransferState,
        target_id: Id,
    ) -> Result<Vec<u8, 256>, IsoTpError> {
        loop {
            let frame = with_timeout(N_CR_TIMEOUT, self.driver.receive())
                .await
                .map_err(|_| IsoTpError::TimeoutCr)?
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
        let o = self.pci_offset();
        if d.len() <= o {
            return Ok(false);
        }
        match PciType::from_byte(d[o]) {
            Some(PciType::SingleFrame) => self.handle_sf(state, &d[o..]),
            Some(PciType::FirstFrame) => self.handle_ff(state, frame.id(), &d[o..]).await,
            Some(PciType::ConsecutiveFrame) => self.handle_cf(state, &d[o..]),
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
        let ff_dl = (((d[0] & 0x0F) as usize) << 8) | (d[1] as usize);
        // 4.2.1: FF_DL must be ≥ 8 for normal addressing (ISO 15765-2 §9.6.2.2)
        if ff_dl < 8 {
            return Ok(false);
        }
        let fc_id = self.get_fc_id(id)?;
        // 4.2.2: FF_DL exceeds our buffer — send FC(OVFLW) and abort
        if ff_dl > 256 {
            self.send_flow_control(fc_id, FlowStatus::Overflow, 0, 0).await?;
            return Err(IsoTpError::BufferOverflow);
        }
        state.rx_dl = ff_dl;
        state.next_sn = 1; // 4.2.3
        state.buffer.clear();
        // 4.2.5: copy exactly 6 payload bytes for normal addressing (bytes 2–7 of classic CAN frame)
        let payload_end = d.len().min(8);
        state
            .buffer
            .extend_from_slice(&d[2..payload_end])
            .map_err(|_| IsoTpError::BufferOverflow)?;
        // 4.2.6: N_Cr timer starts after FC is sent; per-iteration timeout in receive_loop covers this
        // 7.3: FC(CTS, bs=0, stmin=0) — no block limit, no separation time
        self.send_flow_control(fc_id, FlowStatus::ContinueToSend, 0, 0)
            .await?;
        Ok(false)
    }

    fn handle_sf(&self, state: &mut TransferState, d: &[u8]) -> Result<bool, IsoTpError> {
        let len = (d[0] & 0x0F) as usize;
        // Normal addressing: SF_DL ∈ [1, 7]; extended addressing would cap at 6 (not yet supported)
        if len == 0 || len > 7 {
            return Ok(false);
        }
        // 4.1.2: N_UNEXP_PDU — frame too short to contain the declared payload
        if d.len() < 1 + len {
            return Ok(false);
        }
        state
            .buffer
            .extend_from_slice(&d[1..1 + len])
            .map_err(|_| IsoTpError::BufferOverflow)?;
        Ok(true)
    }

    fn handle_cf(&self, state: &mut TransferState, d: &[u8]) -> Result<bool, IsoTpError> {
        // 4.3.2: need at least PCI byte + 1 data byte before indexing d[1]
        if d.len() < 2 {
            return Ok(false);
        }
        if (d[0] & 0x0F) != state.next_sn {
            return Err(IsoTpError::WrongSn);
        }
        let to_copy = core::cmp::min(state.rx_dl - state.buffer.len(), 7);
        state
            .buffer
            .extend_from_slice(&d[1..1 + to_copy])
            .map_err(|_| IsoTpError::BufferOverflow)?;
        state.next_sn = (state.next_sn + 1) % 16;
        Ok(state.buffer.len() >= state.rx_dl)
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
            Err(IsoTpError::TimeoutCr)
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
                s.rx_dl = 0;
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
                rx_dl: 0,
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

    /// Await an FC frame on `fc_id` with N_Bs timeout, handling WAIT/OVFLW/reserved FS.
    /// Returns `(BS, STmin)` on FC(CTS).
    async fn receive_fc(&self, fc_id: Id, wft_count: &mut u8) -> Result<(u8, u8), IsoTpError> {
        loop {
            let frame = with_timeout(N_BS_TIMEOUT, self.driver.receive())
                .await
                .map_err(|_| IsoTpError::TimeoutBs)?
                .map_err(|_| IsoTpError::DriverError)?;
            if frame.id() != fc_id {
                continue;
            }
            let d = frame.data();
            let o = self.pci_offset();
            if d.len() < o + 3 || d[o] >> 4 != PciType::FlowControl as u8 {
                continue;
            }
            match FlowStatus::from_nibble(d[o])? {
                FlowStatus::ContinueToSend => return Ok((d[o + 1], d[o + 2])),
                FlowStatus::Wait => {
                    *wft_count += 1;
                    if *wft_count >= N_WFTMAX {
                        return Err(IsoTpError::WftOverrun);
                    }
                    // N_Bs timer restarts implicitly on the next loop iteration
                }
                FlowStatus::Overflow => return Err(IsoTpError::BufferOverflow),
            }
        }
    }

    async fn send_flow_control(
        &self,
        target_id: Id,
        fs: FlowStatus,
        bs: u8,
        stmin: u8,
    ) -> Result<(), IsoTpError> {
        let fc = [
            (PciType::FlowControl as u8) << 4 | fs as u8,
            bs,
            stmin,
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
