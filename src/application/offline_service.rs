use core::sync::atomic::{AtomicBool, Ordering};

use crate::can::{AsyncCanDriver, SharedObd2Service};
use defmt::{error, info, warn};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    mutex::Mutex,
};
use embassy_time::{Duration, Timer};
use embedded_can::Id;
use embedded_io_async::Write;
use embedded_storage::{ReadStorage, Storage};
use heapless::Vec;
use serde::{Deserialize, Serialize};
use esp_storage::FlashStorage;

use super::protocol::{Response, Status};

pub const IMPORTANT_PIDS: [u8; 8] = [0x0C, 0x0D, 0x05, 0x0F, 0x10, 0x11, 0x2F, 0x0B];

const SCAN_INTERVAL: Duration = Duration::from_secs(15);
const CACHE_MAGIC: [u8; 4] = *b"ODBC";
const CACHE_VERSION: u8 = 1;
const CACHE_HEADER_LEN: usize = 12;
const CACHE_JSON_CAPACITY: usize = 8192;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct StoredEcuResponse {
    pub ecu_id: u32,
    pub data: Vec<u8, 256>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct StoredPidSample {
    pub pid: u8,
    pub responses: Vec<StoredEcuResponse, 8>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct StoredPidSnapshot {
    pub sequence: u32,
    pub samples: Vec<StoredPidSample, 8>,
}

impl StoredPidSnapshot {
    pub fn empty() -> Self {
        Self {
            sequence: 0,
            samples: Vec::new(),
        }
    }
}

#[derive(Debug, defmt::Format)]
pub enum TelemetryError {
    Storage,
    Serialization,
    TooLarge,
}

pub struct PidTelemetryStore<'a> {
    flash: FlashStorage<'a>,
    cache_offset: u32,
    cache_size: usize,
    sequence: u32,
    scratch: [u8; CACHE_JSON_CAPACITY],
}

pub type SharedPidCache = Mutex<CriticalSectionRawMutex, PidTelemetryStore<'static>>;

impl<'a> PidTelemetryStore<'a> {
    pub fn new(flash: FlashStorage<'a>, cache_offset: u32, cache_size: usize) -> Self {
        Self {
            flash,
            cache_offset,
            cache_size,
            sequence: 0,
            scratch: [0u8; CACHE_JSON_CAPACITY],
        }
    }

    pub fn load_snapshot(&mut self) -> Result<StoredPidSnapshot, TelemetryError> {
        let mut header = [0u8; CACHE_HEADER_LEN];
        self.flash
            .read(self.cache_offset, &mut header)
            .map_err(|_| TelemetryError::Storage)?;

        if header[..4] != CACHE_MAGIC || header[4] != CACHE_VERSION {
            return Ok(StoredPidSnapshot::empty());
        }

        let payload_len = u32::from_le_bytes(header[8..12].try_into().unwrap()) as usize;
        if payload_len == 0
            || payload_len > self.cache_size.saturating_sub(CACHE_HEADER_LEN)
            || payload_len > self.scratch.len()
        {
            return Ok(StoredPidSnapshot::empty());
        }

        self.flash
            .read(
                self.cache_offset + CACHE_HEADER_LEN as u32,
                &mut self.scratch[..payload_len],
            )
            .map_err(|_| TelemetryError::Storage)?;

        let (snapshot, _) = serde_json_core::from_slice::<StoredPidSnapshot>(
            &self.scratch[..payload_len],
        )
        .map_err(|_| TelemetryError::Serialization)?;

        self.sequence = snapshot.sequence;
        Ok(snapshot)
    }

    pub fn save_snapshot(&mut self, mut snapshot: StoredPidSnapshot) -> Result<(), TelemetryError> {
        self.sequence = self.sequence.wrapping_add(1);
        snapshot.sequence = self.sequence;

        let payload_len = serde_json_core::to_slice(&snapshot, &mut self.scratch)
            .map_err(|_| TelemetryError::Serialization)?;

        if payload_len + CACHE_HEADER_LEN > self.cache_size {
            return Err(TelemetryError::TooLarge);
        }

        let mut header = [0u8; CACHE_HEADER_LEN];
        header[..4].copy_from_slice(&CACHE_MAGIC);
        header[4] = CACHE_VERSION;
        header[8..12].copy_from_slice(&(payload_len as u32).to_le_bytes());

        self.flash
            .write(self.cache_offset, &header)
            .map_err(|_| TelemetryError::Storage)?;
        self.flash
            .write(self.cache_offset + CACHE_HEADER_LEN as u32, &self.scratch[..payload_len])
            .map_err(|_| TelemetryError::Storage)?;

        Ok(())
    }
}

pub async fn collect_snapshot<D>(obd_service: &'static SharedObd2Service<D>) -> StoredPidSnapshot
where
    D: AsyncCanDriver,
{
    info!("Collecting offline PID snapshot");

    let mut samples = Vec::new();
    for &pid in &IMPORTANT_PIDS {
        let result = {
            let service = obd_service.lock().await;
            service.get_broadcast_livedata(pid).await
        };

        let mut responses = Vec::new();
        match result {
            Ok(ecu_responses) => {
                for response in &ecu_responses {
                    responses.push(StoredEcuResponse {
                        ecu_id: can_id_to_u32(response.id),
                        data: response.data.clone(),
                    })
                    .ok();
                }
            }
            Err(e) => warn!("PID 0x{:02X} scan failed: {:?}", pid, e),
        }

        samples.push(StoredPidSample { pid, responses }).ok();
    }

    StoredPidSnapshot {
        sequence: 0,
        samples,
    }
}

pub async fn run_offline_scanner<D>(
    connected: &'static AtomicBool,
    obd_service: &'static SharedObd2Service<D>,
    cache: &'static SharedPidCache,
)
where
    D: AsyncCanDriver,
{
    info!("Offline PID scanner task started");

    loop {
        if connected.load(Ordering::Acquire) {
            Timer::after_secs(1).await;
            continue;
        }

        let snapshot = collect_snapshot(obd_service).await;

        let save_result = {
            let mut store = cache.lock().await;
            store.save_snapshot(snapshot)
        };

        if let Err(e) = save_result {
            error!("Failed to persist PID snapshot: {:?}", e);
        }

        Timer::after(SCAN_INTERVAL).await;
    }
}

pub async fn send_cached_snapshot<S>(
    stream: &mut S,
    cache: &'static SharedPidCache,
) -> Result<(), TelemetryError>
where
    S: Write,
{
    let snapshot = {
        let mut store = cache.lock().await;
        match store.load_snapshot() {
            Ok(snapshot) => snapshot,
            Err(e) => {
                warn!("Failed to load cached PID snapshot: {:?}", e);
                StoredPidSnapshot::empty()
            }
        }
    };

    let mut out_buf = [0u8; 4096];
    let len = serde_json_core::to_slice(
        &Response {
            id: 0,
            status: Status::Ok,
            data: Some(&snapshot),
            debug: None,
        },
        &mut out_buf,
    )
    .map_err(|_| TelemetryError::Serialization)?;

    stream
        .write_all(&out_buf[..len])
        .await
        .map_err(|_| TelemetryError::Storage)?;

    Ok(())
}

fn can_id_to_u32(id: Id) -> u32 {
    match id {
        Id::Standard(s) => s.as_raw() as u32,
        Id::Extended(e) => e.as_raw(),
    }
}
