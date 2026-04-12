use super::AsyncCanDriver;
use defmt::{error, info};

use esp_println as _;
use esp_backtrace as _;

use core::sync::atomic::{AtomicU8, Ordering};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_can::{ExtendedId, Frame, Id, StandardId};
use heapless::Vec;

const PADDING_BYTE: u8 = 0xCC;

///  Sender TX-confirm timeout
const N_AS_TIMEOUT: Duration = Duration::from_millis(5000);
///  Sender wait-for-FC timeout
const N_BS_TIMEOUT: Duration = Duration::from_millis(1000);
///  Receiver wait-for-CF timeout
const N_CR_TIMEOUT: Duration = Duration::from_millis(1000);
///  Maximum consecutive FC.WAIT frames before abort
const N_WFTMAX: u8 = 10;

const TIMEOUT_INTER_FRAME: Duration = Duration::from_millis(100);
const TIMEOUT_TOTAL: Duration = Duration::from_millis(500);

/// FC response block size: 0 = unlimited — safe because process_first_frame rejects ff_dl > 256.
const FC_BS: u8 = 0;
/// FC response STmin: 25 ms between consecutive frames — enough headroom for the embassy
/// executor to drain the RX queue without dropping frames on typical MCU clock speeds.
const FC_STMIN: u8 = 25;

#[repr(u8)]
enum FlowStatus {
    ContinueToSend = 0,
    Wait = 1,
    Overflow = 2,
}

impl FlowStatus {
    /// Parse from the full FC PCI byte (lower nibble = FS).
    fn from_pci_byte(b: u8) -> Result<Self, IsoTpError> {
        info!("enter: FlowStatus::from_pci_byte");
        match b & 0x0F {
            0 => Ok(Self::ContinueToSend),
            1 => Ok(Self::Wait),
            2 => Ok(Self::Overflow),
            _ => Err(IsoTpError::InvalidFs),
        }
    }
}

/// Network layer addressing modes
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum AddressingMode {
    /// Normal addressing: PCI at byte[0], max SF payload 7 bytes
    Normal = 0,
    /// Extended addressing: byte[0] = N_TA, PCI at byte[1], max SF payload 6 bytes
    Extended = 1,
}

impl AddressingMode {
    fn from_u8(v: u8) -> Self {
        if v == 0 { Self::Normal } else { Self::Extended }
    }
}

#[derive(Debug, defmt::Format)]
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
fn serialize_can_id<S: serde::Serializer>(id: &Id, ser: S) -> Result<S::Ok, S::Error> {
    info!("enter: serialize_can_id");
    match id {
        Id::Standard(s) => ser.serialize_u32(s.as_raw() as u32),
        Id::Extended(e) => ser.serialize_u32(e.as_raw()),
    }
}

#[derive(Debug, Clone, serde::Serialize)]
pub struct EcuResponse {
    #[serde(serialize_with = "serialize_can_id")]
    pub id: Id,
    pub data: Vec<u8, 256>,
}

struct TransferState {
    id: Id,
    /// FF_DL from the First Frame header (ISO 15765-2 §9.6.2.2)
    rx_dl: usize,
    next_sn: u8,
    buffer: Vec<u8, 256>,
    /// Timestamp of the last received frame for this ECU (for per-ECU N_Cr tracking)
    last_frame_at: Option<Instant>,
}

impl TransferState {
    fn new(id: Id) -> Self {
        info!("enter: TransferState::new");
        Self {
            id,
            rx_dl: 0,
            next_sn: 1,
            buffer: Vec::new(),
            last_frame_at: None,
        }
    }

    /// Apply a Single Frame payload (d starts at the PCI byte).
    /// Returns true when the message is complete.
    fn apply_single_frame(&mut self, data: &[u8]) -> Result<bool, IsoTpError> {
        info!("enter: TransferState::apply_single_frame");
        let len = (data[0] & 0x0F) as usize;
        if len == 0 || len > 7 || data.len() < 1 + len {
            return Ok(false);
        }
        self.buffer
            .extend_from_slice(&data[1..1 + len])
            .map_err(|_| IsoTpError::BufferOverflow)?;
        Ok(true)
    }

    /// Apply a First Frame payload (d starts at the PCI byte, after FF validation).
    /// Caller is responsible for sending FC(CTS) after this returns Ok.
    fn apply_first_frame(&mut self, data: &[u8], ff_dl: usize) -> Result<(), IsoTpError> {
        info!("enter: TransferState::apply_first_frame");
        self.rx_dl = ff_dl;
        self.next_sn = 1;
        self.buffer.clear();
        let payload_end = data.len().min(8);
        self.buffer
            .extend_from_slice(&data[2..payload_end])
            .map_err(|_| IsoTpError::BufferOverflow)
    }

    /// Apply a Consecutive Frame payload (d starts at the PCI byte).
    /// Returns true when all expected bytes have been received.
    fn apply_consecutive_frame(&mut self, data: &[u8]) -> Result<bool, IsoTpError> {
        info!("enter: TransferState::apply_consecutive_frame");
        if data.len() < 2 {
            return Ok(false);
        }
        if (data[0] & 0x0F) != self.next_sn {
            return Err(IsoTpError::WrongSn);
        }
        let to_copy = core::cmp::min(self.rx_dl - self.buffer.len(), data.len() - 1);
        self.buffer
            .extend_from_slice(&data[1..1 + to_copy])
            .map_err(|_| IsoTpError::BufferOverflow)?;
        self.next_sn = (self.next_sn + 1) % 16;
        Ok(self.buffer.len() >= self.rx_dl)
    }
}

#[repr(u8)]
enum PciType {
    SingleFrame = 0,
    FirstFrame = 1,
    ConsecutiveFrame = 2,
    FlowControl = 3,
}

impl PciType {
    /// Parse from the full PCI byte (upper nibble = type).
    fn from_pci_byte(b: u8) -> Option<Self> {
        info!("enter: PciType::from_pci_byte");
        match b >> 4 {
            0 => Some(Self::SingleFrame),
            1 => Some(Self::FirstFrame),
            2 => Some(Self::ConsecutiveFrame),
            3 => Some(Self::FlowControl),
            _ => None,
        }
    }
}

/// Pluggable CAN ID mapping strategy for ISO-TP.
///
/// Separates OBD-II/UDS-specific ID logic from the generic framing layer.
/// Implement this trait to support non-standard ID assignments.
pub trait IdMapper {
    /// Given a request CAN ID, return the expected response CAN ID.
    /// Used in `send_physical_request` to know which frames to accept.
    fn response_id_for_request(&self, request_id: Id) -> Result<Id, IsoTpError>;
    /// Given the source ID of a received FF, return the CAN ID
    /// on which to send Flow Control frames back to that ECU.
    fn derive_fc_sender_id(&self, response_id: Id) -> Result<Id, IsoTpError>;
    /// Return true if `id` is a valid response frame ID to collect.
    fn is_valid_response_id(&self, id: Id) -> bool;
}

pub struct IsoTpHandler<D, M: IdMapper = Obd2IdMapper> {
    driver: D,
    /// Addressing mode stored atomically for runtime switching
    addressing: AtomicU8,
    /// N_TA byte prepended to every frame in Extended addressing mode
    target_addr: AtomicU8,
    mapper: M,
}

impl<D: AsyncCanDriver> IsoTpHandler<D, Obd2IdMapper> {
    pub fn new(driver: D) -> Self {
        info!("enter: IsoTpHandler::new");
        Self {
            driver,
            addressing: AtomicU8::new(AddressingMode::Normal as u8),
            target_addr: AtomicU8::new(0),
            mapper: Obd2IdMapper::new(AddressingMode::Normal),
        }
    }
    /// Construct in Extended addressing mode with the given target address byte.
    pub fn with_target_addr(driver: D, target_addr: u8) -> () {
        info!("enter: IsoTpHandler::with_target_addr");
        self.target_addr.store(target_addr, Ordering::Relaxed);
    }

    /// Switch to Extended addressing mode at runtime.
    pub fn to_extended_adr(&self, target_addr: u8) {
        info!("enter: IsoTpHandler::to_extended_adr target_addr=0x{:02X}", target_addr);
        self.addressing.store(AddressingMode::Extended as u8, Ordering::Relaxed);
        self.target_addr.store(target_addr, Ordering::Relaxed);
        self.mapper.to_extended_adr();
        info!("return: IsoTpHandler::to_extended_adr");
    }

    /// Switch to Normal (11-bit) addressing mode at runtime.
    pub fn to_normal_addr(&self) {
        info!("enter: IsoTpHandler::to_normal_addr");
        self.addressing.store(AddressingMode::Normal as u8, Ordering::Relaxed);
        self.mapper.to_normal_addr();
        info!("return: IsoTpHandler::to_normal_addr");
    }
}

impl<D: AsyncCanDriver, M: IdMapper> IsoTpHandler<D, M> {
    pub fn with_mapper(driver: D, addressing: AddressingMode, target_addr: u8, mapper: M) -> Self {
        info!("enter: IsoTpHandler::with_mapper");
        Self {
            driver,
            addressing: AtomicU8::new(addressing as u8),
            target_addr: AtomicU8::new(target_addr),
            mapper,
        }
    }

    fn addressing_mode(&self) -> AddressingMode {
        AddressingMode::from_u8(self.addressing.load(Ordering::Relaxed))
    }

    /// Returns true when the handler is currently in Extended addressing mode.
    pub fn is_extended_addressing(&self) -> bool {
        self.addressing_mode() == AddressingMode::Extended
    }

    /// Byte offset at which the PCI byte starts (0 for Normal, 1 for Extended).
    fn pci_byte_offset(&self) -> usize {
        info!("enter: IsoTpHandler::pci_byte_offset");
        match self.addressing_mode() {
            AddressingMode::Normal => 0,
            AddressingMode::Extended => 1,
        }
    }

    pub async fn send_physical_request(
        &self,
        target_id: Id,
        data: &[u8],
    ) -> Result<Vec<u8, 256>, IsoTpError> {
        info!("enter: IsoTpHandler::send_physical_request");
        // Derive the expected response ID (mapper reverses the OBD-II/UDS offset convention)
        let resp_id = self.mapper.response_id_for_request(target_id)?;
        let max_sf = 7 - self.pci_byte_offset();
        if data.len() <= max_sf {
            self.send_single_frame(target_id, data).await?;
        } else {
            self.send_multi_frame(target_id, resp_id, data).await?;
        }
        let result = self.receive_response(resp_id).await;
        match &result {
            Ok(_) => info!("return ok: IsoTpHandler::send_physical_request"),
            Err(e) => error!("return err: IsoTpHandler::send_physical_request {:?}", e),
        }
        result
    }

    pub async fn send_functional_request(
        &self,
        target_id: Id,
        data: &[u8],
    ) -> Result<Vec<EcuResponse, 8>, IsoTpError> {
        info!("enter: IsoTpHandler::send_functional_request");
        // functional addressing only supports SF
        let max_sf = 7 - self.pci_byte_offset();
        if data.len() > max_sf {
            error!("return err: IsoTpHandler::send_functional_request BufferOverflow");
            return Err(IsoTpError::BufferOverflow);
        }
        self.send_single_frame(target_id, data).await?;
        let result = self
            .receive_functional_responses(TIMEOUT_INTER_FRAME, TIMEOUT_TOTAL)
            .await;
        match &result {
            Ok(_) => info!("return ok: IsoTpHandler::send_functional_request"),
            Err(e) => error!("return err: IsoTpHandler::send_functional_request {:?}", e),
        }
        result
    }

    async fn send_single_frame(&self, id: Id, data: &[u8]) -> Result<(), IsoTpError> {
        info!("enter: IsoTpHandler::send_single_frame");
        let o = self.pci_byte_offset();
        let max_payload = 7 - o;
        if data.len() > max_payload {
            error!("return err: IsoTpHandler::send_single_frame BufferOverflow");
            return Err(IsoTpError::BufferOverflow);
        }

        let mut tx = [PADDING_BYTE; 8];
        if o == 1 {
            tx[0] = self.target_addr.load(Ordering::Relaxed);
        }
        tx[o] = data.len() as u8;
        tx[o + 1..o + 1 + data.len()].copy_from_slice(data);

        let frame = D::Frame::new(id, &tx).ok_or(IsoTpError::DriverError)?;

        let result = with_timeout(N_AS_TIMEOUT, self.driver.transmit(&frame))
            .await
            .map_err(|_| IsoTpError::TimeoutA)?
            .map_err(|_| IsoTpError::DriverError);
        match &result {
            Ok(_) => info!("return ok: IsoTpHandler::send_single_frame"),
            Err(e) => error!("return err: IsoTpHandler::send_single_frame {:?}", e),
        }
        result
    }

    /// Transmit a multi-frame message via FF + CF sequence (ISO 15765-2 §9.6).
    /// `tx_id`  — CAN ID we send on.
    /// `fc_id`  — CAN ID we expect FC frames from (usually `resp_id`).
    async fn send_multi_frame(&self, tx_id: Id, fc_id: Id, data: &[u8]) -> Result<(), IsoTpError> {
        info!("enter: IsoTpHandler::send_multi_frame");
        let len = data.len();
        // FF_DL is 12-bit, max 4095
        if len > 0xFFF {
            error!("return err: IsoTpHandler::send_multi_frame BufferOverflow");
            return Err(IsoTpError::BufferOverflow);
        }

        // build and transmit FF
        let o = self.pci_byte_offset();
        let ff_payload = 6 - o; // 6 bytes normal, 5 bytes extended
        let mut ff = [PADDING_BYTE; 8];
        if o == 1 {
            ff[0] = self.target_addr.load(Ordering::Relaxed);
        }
        ff[o] = (PciType::FirstFrame as u8) << 4 | ((len >> 8) as u8 & 0x0F);
        ff[o + 1] = (len & 0xFF) as u8;
        ff[o + 2..o + 2 + ff_payload].copy_from_slice(&data[..ff_payload]);
        let frame = D::Frame::new(tx_id, &ff).ok_or(IsoTpError::DriverError)?;
        with_timeout(N_AS_TIMEOUT, self.driver.transmit(&frame))
            .await
            .map_err(|_| IsoTpError::TimeoutA)?
            .map_err(|_| IsoTpError::DriverError)?;

        // await FC(CTS) — await_flow_control handles WAIT/OVFLW/reserved internally
        let mut wft_count = 0u8;
        let (mut bs, st_min) = self.await_flow_control(fc_id, &mut wft_count).await?;
        let mut st_min_dur = Self::decode_stmin(st_min);

        // transmit CF sequence
        let cf_payload = 7 - o; // 7 bytes normal, 6 bytes extended
        let mut sn: u8 = 1;
        let mut block_count: u8 = 0;
        let mut offset: usize = ff_payload; // bytes already sent in FF

        while offset < len {
            let mut cf = [PADDING_BYTE; 8];

            if o == 1 {
                cf[0] = self.target_addr.load(Ordering::Relaxed);
            }
            cf[o] = (PciType::ConsecutiveFrame as u8) << 4 | (sn & 0x0F);
            let chunk_end = (offset + cf_payload).min(len);
            cf[o + 1..o + 1 + (chunk_end - offset)].copy_from_slice(&data[offset..chunk_end]);

            let frame = D::Frame::new(tx_id, &cf).ok_or(IsoTpError::DriverError)?;
            with_timeout(N_AS_TIMEOUT, self.driver.transmit(&frame))
                .await
                .map_err(|_| IsoTpError::TimeoutA)?
                .map_err(|_| IsoTpError::DriverError)?;

            sn = (sn + 1) % 16;
            offset += cf_payload;
            block_count += 1;

            if bs > 0 && block_count == bs && offset < len {
                // Block exhausted and more data remains — await next FC before continuing
                let mut wft_count = 0u8;
                let (new_bs, new_st_min) = self.await_flow_control(fc_id, &mut wft_count).await?;
                bs = new_bs;
                st_min_dur = Self::decode_stmin(new_st_min);
                block_count = 0;
                // STmin applies between all consecutive CFs, including across block boundaries
                Timer::after(st_min_dur).await;
            } else if offset < len {
                // STmin separation between consecutive CFs (§6.5.4)
                Timer::after(st_min_dur).await;
            }
        }

        info!("return ok: IsoTpHandler::send_multi_frame");
        Ok(())
    }

    /// Decode FC STmin byte to a `Duration` (ISO 15765-2 §9.6.5.5).
    ///
    /// | Value      | Meaning                           |
    /// |------------|-----------------------------------|
    /// | 0x00       | 0 ms                              |
    /// | 0x01–0x7F  | 1–127 ms (1 ms resolution)        |
    /// | 0xF1–0xF9  | 100–900 µs (100 µs resolution)    |
    /// | 0x80–0xF0, 0xFA–0xFF | reserved → 127 ms (max) |
    fn decode_stmin(st_min: u8) -> Duration {
        info!("enter: IsoTpHandler::decode_stmin");
        match st_min {
            0x00 => Duration::from_millis(0),
            v @ 0x01..=0x7F => Duration::from_millis(v as u64),
            v @ 0xF1..=0xF9 => Duration::from_micros((v - 0xF0) as u64 * 100),
            _ => Duration::from_millis(127), // reserved — use maximum per spec
        }
    }

    async fn receive_response(&self, target_id: Id) -> Result<Vec<u8, 256>, IsoTpError> {
        info!("enter: IsoTpHandler::receive_response");
        let mut state = TransferState::new(target_id);
        // Phase 1: wait for first frame (SF or FF) — N_Cr timeout
        let complete = self.await_initial_frame(&mut state, target_id).await?;
        if complete {
            let result = core::mem::replace(&mut state.buffer, Vec::new());
            info!("return ok: IsoTpHandler::receive_response (single-frame)");
            return Ok(result);
        }
        // Phase 2: FF received, FC sent — wait for each CF with per-CF N_Cr timeout
        let result = self.receive_consecutive_frames(&mut state, target_id).await;
        match &result {
            Ok(_) => info!("return ok: IsoTpHandler::receive_response"),
            Err(e) => error!("return err: IsoTpHandler::receive_response {:?}", e),
        }
        result
    }

    /// Phase 1: wait for SF or FF; returns true on SF complete (done), false on FF received (more CFs expected).
    async fn await_initial_frame(
        &self,
        state: &mut TransferState,
        target_id: Id,
    ) -> Result<bool, IsoTpError> {
        info!("enter: IsoTpHandler::await_initial_frame");
        let deadline = Instant::now() + N_CR_TIMEOUT;
        loop {
            let now = Instant::now();
            if now >= deadline {
                error!("return err: IsoTpHandler::await_initial_frame TimeoutCr");
                return Err(IsoTpError::TimeoutCr);
            }
            let frame = with_timeout(deadline - now, self.driver.receive())
                .await
                .map_err(|_| IsoTpError::TimeoutCr)?
                .map_err(|_| IsoTpError::DriverError)?;
            if frame.id() == target_id {
                let result = self.dispatch_frame(state, &frame).await;
                match &result {
                    Ok(complete) => info!("return ok: IsoTpHandler::await_initial_frame complete={}", complete),
                    Err(e) => error!("return err: IsoTpHandler::await_initial_frame {:?}", e),
                }
                return result;
            }
        }
    }

    /// Phase 2: collect CFs. N_Cr deadline restarts only on valid target frames;
    /// frames from other IDs are discarded without resetting the timer.
    async fn receive_consecutive_frames(
        &self,
        state: &mut TransferState,
        target_id: Id,
    ) -> Result<Vec<u8, 256>, IsoTpError> {
        info!("enter: IsoTpHandler::receive_consecutive_frames");
        let mut deadline = Instant::now() + N_CR_TIMEOUT;
        loop {
            let now = Instant::now();
            if now >= deadline {
                error!("return err: IsoTpHandler::receive_consecutive_frames TimeoutCr");
                return Err(IsoTpError::TimeoutCr);
            }
            let frame = with_timeout(deadline - now, self.driver.receive())
                .await
                .map_err(|_| IsoTpError::TimeoutCr)?
                .map_err(|_| IsoTpError::DriverError)?;
            if frame.id() == target_id {
                if self.dispatch_frame(state, &frame).await? {
                    info!("return ok: IsoTpHandler::receive_consecutive_frames");
                    return Ok(core::mem::replace(&mut state.buffer, Vec::new()));
                }
                deadline = Instant::now() + N_CR_TIMEOUT;
            }
        }
    }

    /// Route incoming frame to the appropriate handler by PCI type.
    async fn dispatch_frame(
        &self,
        state: &mut TransferState,
        frame: &D::Frame,
    ) -> Result<bool, IsoTpError> {
        info!("enter: IsoTpHandler::dispatch_frame");
        let d = frame.data();
        let o = self.pci_byte_offset();
        if d.len() <= o {
            return Ok(false);
        }
        match PciType::from_pci_byte(d[o]) {
            Some(PciType::SingleFrame) => self.process_single_frame(state, &d[o..]),
            Some(PciType::FirstFrame) => self.process_first_frame(state, frame.id(), &d[o..]).await,
            Some(PciType::ConsecutiveFrame) => self.process_consecutive_frame(state, &d[o..]),
            _ => Ok(false),
        }
    }

    async fn process_first_frame(
        &self,
        state: &mut TransferState,
        id: Id,
        d: &[u8],
    ) -> Result<bool, IsoTpError> {
        info!("enter: IsoTpHandler::process_first_frame");
        if d.len() < 3 {
            return Ok(false);
        }
        let ff_dl = (((d[0] & 0x0F) as usize) << 8) | (d[1] as usize);
        // Minimum FF_DL depends on addressing: Normal SF holds 7 bytes, Extended SF holds 6.
        let min_ff_dl = 8 - self.pci_byte_offset();
        if ff_dl < min_ff_dl {
            return Ok(false);
        }
        let fc_id = self.mapper.derive_fc_sender_id(id)?;
        if ff_dl > 256 {
            self.transmit_flow_control(fc_id, FlowStatus::Overflow, 0, 0)
                .await?;
            error!("return err: IsoTpHandler::process_first_frame BufferOverflow");
            return Err(IsoTpError::BufferOverflow);
        }
        state.apply_first_frame(d, ff_dl)?;
        self.transmit_flow_control(fc_id, FlowStatus::ContinueToSend, FC_BS, FC_STMIN)
            .await?;
        info!("return ok: IsoTpHandler::process_first_frame (FF accepted, awaiting CFs)");
        Ok(false)
    }

    fn process_single_frame(
        &self,
        state: &mut TransferState,
        d: &[u8],
    ) -> Result<bool, IsoTpError> {
        info!("enter: IsoTpHandler::process_single_frame");
        state.apply_single_frame(d)
    }

    fn process_consecutive_frame(
        &self,
        state: &mut TransferState,
        d: &[u8],
    ) -> Result<bool, IsoTpError> {
        info!("enter: IsoTpHandler::process_consecutive_frame");
        state.apply_consecutive_frame(d)
    }

    async fn receive_functional_responses(
        &self,
        inter: Duration,
        total: Duration,
    ) -> Result<Vec<EcuResponse, 8>, IsoTpError> {
        info!("enter: IsoTpHandler::receive_functional_responses");
        let mut res: Vec<EcuResponse, 8> = Vec::new();
        let mut states: Vec<TransferState, 8> = Vec::new();
        let _ = with_timeout(
            total,
            self.run_collection_loop(&mut res, &mut states, inter),
        )
        .await;
        if res.is_empty() {
            error!("return err: IsoTpHandler::receive_functional_responses TimeoutCr (no responses)");
            Err(IsoTpError::TimeoutCr)
        } else {
            info!("return ok: IsoTpHandler::receive_functional_responses count={}", res.len());
            Ok(res)
        }
    }

    async fn run_collection_loop(
        &self,
        res: &mut Vec<EcuResponse, 8>,
        states: &mut Vec<TransferState, 8>,
        inter: Duration,
    ) {
        info!("enter: IsoTpHandler::run_collection_loop");
        loop {
            let frame = with_timeout(inter, self.driver.receive()).await;
            match frame {
                Ok(Ok(f)) => self.process_collection_frame(res, states, f).await,
                _ => {
                    let any_active = states.iter().any(|s| s.rx_dl > 0);
                    if !any_active {
                        break;
                    }
                }
            }
        }
    }

    async fn process_collection_frame(
        &self,
        res: &mut Vec<EcuResponse, 8>,
        states: &mut Vec<TransferState, 8>,
        f: D::Frame,
    ) {
        info!("enter: IsoTpHandler::process_collection_frame");
        let frame_id = f.id();
        if !self.mapper.is_valid_response_id(frame_id) {
            return;
        }
        states.retain(|s| {
            s.rx_dl == 0 || s.last_frame_at.map_or(true, |t| t.elapsed() < N_CR_TIMEOUT)
        });

        let state = self.find_or_insert_ecu_state(states, frame_id);
        if let Some(s) = state {
            s.last_frame_at = Some(Instant::now());
            match self.dispatch_frame(s, &f).await {
                Ok(true) => {
                    let data = core::mem::replace(&mut s.buffer, Vec::new());
                    s.rx_dl = 0;
                    s.next_sn = 1;
                    res.push(EcuResponse { id: frame_id, data }).ok();
                }
                Ok(false) => {}
                Err(_) => {
                    states.retain(|st| st.id != frame_id);
                }
            }
        }
    }

    fn find_or_insert_ecu_state<'a>(
        &self,
        states: &'a mut Vec<TransferState, 8>,
        id: Id,
    ) -> Option<&'a mut TransferState> {
        info!("enter: IsoTpHandler::find_or_insert_ecu_state");
        if let Some(idx) = states.iter().position(|s| s.id == id) {
            return Some(&mut states[idx]);
        }
        states.push(TransferState::new(id)).ok()?;
        states.last_mut()
    }

    /// Await an FC frame on `fc_id` with N_Bs timeout, handling WAIT/OVFLW/reserved FS.
    /// Non-matching frames are discarded without resetting the deadline.
    /// FC.WAIT resets N_Bs per spec (§9.6.5.4).
    /// Returns `(BS, STmin)` on FC(CTS).
    async fn await_flow_control(
        &self,
        fc_id: Id,
        wft_count: &mut u8,
    ) -> Result<(u8, u8), IsoTpError> {
        info!("enter: IsoTpHandler::await_flow_control");
        let mut deadline = Instant::now() + N_BS_TIMEOUT;
        loop {
            let now = Instant::now();
            if now >= deadline {
                error!("return err: IsoTpHandler::await_flow_control TimeoutBs");
                return Err(IsoTpError::TimeoutBs);
            }
            let frame = with_timeout(deadline - now, self.driver.receive())
                .await
                .map_err(|_| IsoTpError::TimeoutBs)?
                .map_err(|_| IsoTpError::DriverError)?;
            if frame.id() != fc_id {
                continue;
            }
            let d = frame.data();
            let o = self.pci_byte_offset();
            if d.len() < o + 3 || d[o] >> 4 != PciType::FlowControl as u8 {
                continue;
            }
            match FlowStatus::from_pci_byte(d[o])? {
                FlowStatus::ContinueToSend => {
                    info!("return ok: IsoTpHandler::await_flow_control CTS bs={} stmin={}", d[o + 1], d[o + 2]);
                    return Ok((d[o + 1], d[o + 2]));
                }
                FlowStatus::Wait => {
                    *wft_count += 1;
                    if *wft_count >= N_WFTMAX {
                        error!("return err: IsoTpHandler::await_flow_control WftOverrun");
                        return Err(IsoTpError::WftOverrun);
                    }
                    deadline = Instant::now() + N_BS_TIMEOUT;
                }
                FlowStatus::Overflow => {
                    error!("return err: IsoTpHandler::await_flow_control Overflow");
                    return Err(IsoTpError::BufferOverflow);
                }
            }
        }
    }

    async fn transmit_flow_control(
        &self,
        target_id: Id,
        fs: FlowStatus,
        bs: u8,
        stmin: u8,
    ) -> Result<(), IsoTpError> {
        info!("enter: IsoTpHandler::transmit_flow_control");
        let o = self.pci_byte_offset();
        let mut fc = [PADDING_BYTE; 8];
        if o == 1 {
            fc[0] = self.target_addr.load(Ordering::Relaxed);
        }
        fc[o] = (PciType::FlowControl as u8) << 4 | fs as u8;
        fc[o + 1] = bs;
        fc[o + 2] = stmin;
        let frame = D::Frame::new(target_id, &fc).ok_or(IsoTpError::DriverError)?;
        let result = with_timeout(N_AS_TIMEOUT, self.driver.transmit(&frame))
            .await
            .map_err(|_| IsoTpError::TimeoutA)?
            .map_err(|_| IsoTpError::DriverError);
        match &result {
            Ok(_) => info!("return ok: IsoTpHandler::transmit_flow_control"),
            Err(e) => error!("return err: IsoTpHandler::transmit_flow_control {:?}", e),
        }
        result
    }
}

// ==========================================
// OBD-II / UDS default ID mapper
// ==========================================

/// Default `IdMapper` for OBD-II (SAE J1979 / ISO 15031-5) and UDS over J1939.
///
/// Standard 11-bit IDs:  request 0x7E0–0x7E7 → response +8 (0x7E8–0x7EF)
/// Extended 29-bit IDs:  0x18DA_TA_SA ↔ 0x18DA_SA_TA (TA/SA byte swap)
pub struct Obd2IdMapper {
    /// Addressing mode stored atomically for runtime switching
    addressing: AtomicU8,
}

impl Obd2IdMapper {
    pub fn new(addressing: AddressingMode) -> Self {
        info!("enter: Obd2IdMapper::new");
        Self { addressing: AtomicU8::new(addressing as u8) }
    }

    /// Switch to Extended (29-bit) addressing mode at runtime.
    pub fn to_extended_adr(&self) {
        info!("enter: Obd2IdMapper::to_extended_adr");
        self.addressing.store(AddressingMode::Extended as u8, Ordering::Relaxed);
        info!("return: Obd2IdMapper::to_extended_adr");
    }

    /// Switch to Normal (11-bit) addressing mode at runtime.
    pub fn to_normal_addr(&self) {
        info!("enter: Obd2IdMapper::to_normal_addr");
        self.addressing.store(AddressingMode::Normal as u8, Ordering::Relaxed);
        info!("return: Obd2IdMapper::to_normal_addr");
    }

    fn addressing_mode(&self) -> AddressingMode {
        AddressingMode::from_u8(self.addressing.load(Ordering::Relaxed))
    }

    /// Swap SA/TA bytes in a UDS 29-bit extended ID (J1939 format 0x18DA_TA_SA).
    fn swap_uds_sa_ta(id: u32) -> u32 {
        info!("enter: Obd2IdMapper::swap_uds_sa_ta");
        (id & 0xFFFF0000) | ((id & 0xFF) << 8) | ((id >> 8) & 0xFF)
    }
}

impl IdMapper for Obd2IdMapper {
    fn response_id_for_request(&self, request_id: Id) -> Result<Id, IsoTpError> {
        info!("enter: Obd2IdMapper::response_id_for_request");
        let result = match request_id {
            Id::Standard(s) => Ok(Id::Standard(StandardId::new(s.as_raw() + 8).unwrap())),
            Id::Extended(e) => Ok(Id::Extended(
                ExtendedId::new(Self::swap_uds_sa_ta(e.as_raw())).unwrap(),
            )),
        };
        match &result {
            Ok(_) => info!("return ok: Obd2IdMapper::response_id_for_request"),
            Err(e) => error!("return err: Obd2IdMapper::response_id_for_request {:?}", e),
        }
        result
    }

    fn derive_fc_sender_id(&self, response_id: Id) -> Result<Id, IsoTpError> {
        info!("enter: Obd2IdMapper::derive_fc_sender_id");
        let result = match response_id {
            Id::Standard(std) => {
                let raw = std.as_raw();
                if raw < 8 {
                    return Err(IsoTpError::InvalidId);
                }
                Ok(Id::Standard(StandardId::new(raw - 8).unwrap()))
            }
            Id::Extended(ext) => Ok(Id::Extended(
                ExtendedId::new(Self::swap_uds_sa_ta(ext.as_raw())).unwrap(),
            )),
        };
        match &result {
            Ok(_) => info!("return ok: Obd2IdMapper::derive_fc_sender_id"),
            Err(e) => error!("return err: Obd2IdMapper::derive_fc_sender_id {:?}", e),
        }
        result
    }

    fn is_valid_response_id(&self, id: Id) -> bool {
        info!("enter: Obd2IdMapper::is_valid_response_id");
        match (self.addressing_mode(), id) {
            (AddressingMode::Normal, Id::Standard(s)) => (0x7E8..=0x7EF).contains(&s.as_raw()),
            (AddressingMode::Extended, Id::Extended(e)) => (e.as_raw() & 0xFFFF0000) == 0x18DA0000,
            _ => false,
        }
    }
}

// TODO: configurable timeouts — N_As/N_Bs/N_Cr are compile-time constants; see IsoTpConfig in roadmap 3.2
// TODO: large FF_DL escape sequence — FF_DL=0 with 4-byte length field (ISO 15765-2 §9.6.2.2) for PDUs > 4095 bytes
// TODO: buffer capacity as const generic on IsoTpHandler<D, const N: usize> to allow callers to trade memory for max PDU size