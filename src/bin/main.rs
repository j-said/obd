#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_sync::mutex::Mutex;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    interrupt::software,
    timer::timg::TimerGroup,
    twai::{BaudRate, TwaiConfiguration, TwaiMode},
};
use esp_radio::ble::controller::BleConnector;
use static_cell::StaticCell;
use trouble_host::prelude::*;

use obd_rust::can::{EspCanManager, IsoTpHandler, Obd2Service, SharedTwaiRx, SharedTwaiTx};
use obd_rust::transport_io::ble::{BleResources, ObdRunner, ObdStack, BleChannel, ObdPeripheral};

static TX_CHANNEL: BleChannel = BleChannel::new();
static RX_CHANNEL: BleChannel = BleChannel::new();

static BLE_STACK_RESOURCES: StaticCell<BleResources> = StaticCell::new();
static BLE_STACK: StaticCell<ObdStack> = StaticCell::new();

static TWAI_TX: StaticCell<SharedTwaiTx> = StaticCell::new();
static TWAI_RX: StaticCell<SharedTwaiRx> = StaticCell::new();

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    // ESP init
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let software_interrupt = software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

    esp_rtos::start(timg0.timer0, software_interrupt.software_interrupt0);
    esp_alloc::heap_allocator!(size: 72 * 1024);

    // TWAI init
    let (rx, tx) = TwaiConfiguration::new(
        peripherals.TWAI0,
        peripherals.GPIO4,
        peripherals.GPIO5,
        BaudRate::B500K,
        TwaiMode::Normal,
    )
    .into_async()
    .start()
    .split();

    let tx_shared = TWAI_TX.init(Mutex::new(tx));
    let rx_shared = TWAI_RX.init(Mutex::new(rx));

    let can_manager = EspCanManager::new(tx_shared, rx_shared);
    let iso_tp = IsoTpHandler::new(can_manager);
    let _obd2 = Obd2Service::new(iso_tp);

    // Ble init with esp-radio
    let connector = BleConnector::new(peripherals.BT, Default::default()).unwrap();
    let controller = ExternalController::new(connector);

    let resources = BLE_STACK_RESOURCES.init(BleResources::new());
    let stack = BLE_STACK.init_with(|| trouble_host::new(controller, resources));
    let _host = stack.build();
    let _peripheral = _host.peripheral;
    let runner = _host.runner;

    spawner.spawn(ble_runner_task(runner)).unwrap();
    // spawner.spawn().unwrap();

    // -- End

    loop {}
}

#[embassy_executor::task]
async fn ble_runner_task(mut runner: ObdRunner) {
    // Керує подіями HCI.
    runner.run().await.unwrap();
}

// #[embassy_executor::task]
// async fn ble_service_task(stack: ObdStack, peripheral: ObdPeripheral) {}
