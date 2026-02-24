use super::super::parse_usize;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use heapless::Vec;
use trouble_host::prelude::*;

pub const DEVICE_NAME: &str = env!("MY_BLE_DEVICE_NAME");
pub const MTU_SIZE: usize = trouble_host::config::DEFAULT_PACKET_POOL_MTU;

pub(crate) type MyPacketPool = DefaultPacketPool;

pub type BleResources = HostResources<
    MyPacketPool,
    { parse_usize(env!("MY_BLE_MAX_CONNECTIONS")) },
    { parse_usize(env!("MY_BLE_L2CAP_CHANNELS")) },
    { parse_usize(env!("MY_BLE_AD_HANDLES")) },
>;

pub type MyBleStack<'a, C> = Stack<'a, C, MyPacketPool>;                    // Білдер
pub type MyBleHost<'a, C> = Host<'a, C, MyPacketPool>;                      // Результат build()
pub type MyBlePeripheral<'a, C> = Peripheral<'a, C, MyPacketPool>;          // GAP інтерфейс
pub type MyBleRunner<'a, C> = Runner<'a, C, MyPacketPool>;                  // Процес стека

pub type BlePacket = Vec<u8, MTU_SIZE>;
pub type BleChannel =
    Channel<CriticalSectionRawMutex, BlePacket, { parse_usize(env!("MY_BLE_CHANNEL_CAPACITY")) }>;
