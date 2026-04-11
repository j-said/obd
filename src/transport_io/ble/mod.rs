pub mod server;
pub mod stream;

use bt_hci::controller::ExternalController;
use core::fmt;
use defmt::Format;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use esp_radio::ble::controller::BleConnector;
use heapless::Vec;
use trouble_host::prelude::*;

// ==========================================
// TYPES
// ==========================================

pub const DEVICE_NAME: &str = "WROOM-OBD";
pub const MTU_SIZE: usize = trouble_host::config::DEFAULT_PACKET_POOL_MTU;

pub type ObdController = ExternalController<BleConnector<'static>, 10>;

pub type MyPacketPool = DefaultPacketPool;
// GATT does not use L2CAP CoC channels → CHANNELS = 0
pub type BleResources = HostResources<MyPacketPool, 1, 0, 1>;

pub type ObdStack = Stack<'static, ObdController, MyPacketPool>;
pub type ObdHost = Host<'static, ObdController, MyPacketPool>;
pub type ObdPeripheral = Peripheral<'static, ObdController, MyPacketPool>;
pub type ObdRunner = Runner<'static, ObdController, MyPacketPool>;

pub type BlePacket = Vec<u8, MTU_SIZE>;
pub type BleChannel = Channel<CriticalSectionRawMutex, BlePacket, 10>;

// NUS Service UUID: 6E400001-B5B3-F393-E0A9-E50E24DCCA9E (little-endian bytes)
// [u8; 16] to match AdStructure::ServiceUuids128(&[[u8; 16]])
pub const NUS_SERVICE_UUID: [u8; 16] = [
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xB3, 0xB5, 0x01, 0x00, 0x40, 0x6E,
];

// ==========================================
// ERRORS
// ==========================================

#[derive(Debug, Format, Clone, Copy, PartialEq)]
pub enum BleError {
    AdvertisingError,
    ConnectionFailed,
    GattError,
    MtuExceeded,
    ChannelClosed,
    Timeout,
    Other,
}

impl embedded_io_async::Error for BleError {
    fn kind(&self) -> embedded_io_async::ErrorKind {
        match self {
            BleError::MtuExceeded => embedded_io_async::ErrorKind::OutOfMemory,
            BleError::ChannelClosed => embedded_io_async::ErrorKind::BrokenPipe,
            BleError::Timeout => embedded_io_async::ErrorKind::TimedOut,
            BleError::ConnectionFailed => embedded_io_async::ErrorKind::NotConnected,
            _ => embedded_io_async::ErrorKind::Other,
        }
    }
}

impl core::error::Error for BleError {}

impl fmt::Display for BleError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            BleError::AdvertisingError => write!(f, "Advertising failed"),
            BleError::ConnectionFailed => write!(f, "Connection failed"),
            BleError::GattError => write!(f, "GATT error"),
            BleError::MtuExceeded => write!(f, "MTU exceeded"),
            BleError::ChannelClosed => write!(f, "Channel closed"),
            BleError::Timeout => write!(f, "Timeout"),
            BleError::Other => write!(f, "Other error"),
        }
    }
}
