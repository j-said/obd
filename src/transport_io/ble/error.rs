use core::fmt;
use defmt::Format;

#[derive(Debug, Format, Clone, Copy, PartialEq)]
pub enum BleError {
    AdvertisingError,
    ConnectionFailed,
    L2capError,
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
            BleError::L2capError => write!(f, "L2CAP error"),
            BleError::MtuExceeded => write!(f, "MTU exceeded"),
            BleError::ChannelClosed => write!(f, "Channel closed"),
            BleError::Timeout => write!(f, "Timeout"),
            BleError::Other => write!(f, "Other error"),
        }
    }
}