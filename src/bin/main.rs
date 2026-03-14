#![no_std]
#![no_main]

// Error pack's
use defmt::{error, info};
// use defmt_rtt as _;
use esp_alloc as _;
use esp_backtrace as _;
use esp_println as _;

// Embassy core
use core::future::pending;
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_sync::mutex::Mutex;
use esp_hal::{
    clock::CpuClock,
    interrupt::software,
    // rng::{Trng, TrngSource},
    timer::timg::TimerGroup,
    twai::{BaudRate, TwaiConfiguration, TwaiMode},
};
use static_cell::StaticCell;

// Trouble + chip ble driver
use esp_radio::ble::controller::BleConnector;
use trouble_host::prelude::*;

// Self imports
use obd_rust::application::handle_client;
use obd_rust::can::{EspCanManager, IsoTpHandler, Obd2Service, SharedTwaiRx, SharedTwaiTx};
use obd_rust::transport_io::ble::{
    BleChannel, BleResources, ObdPeripheral, ObdRunner, ObdStack,
    server::{create_advertisement, run_connection},
    stream::BleStream,
};

static STREAM_TX: BleChannel = BleChannel::new();
static STREAM_RX: BleChannel = BleChannel::new();

static BLE_STACK_RESOURCES: StaticCell<BleResources> = StaticCell::new();
static BLE_STACK: StaticCell<ObdStack> = StaticCell::new();

static TWAI_TX: StaticCell<SharedTwaiTx> = StaticCell::new();
static TWAI_RX: StaticCell<SharedTwaiRx> = StaticCell::new();
static OBD_SERVICE: StaticCell<Obd2Service<EspCanManager<'static>>> = StaticCell::new();

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    info!("Starting OBD-BLE Bridge initialization...");

    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let software_interrupt = software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

    esp_rtos::start(timg0.timer0, software_interrupt.software_interrupt0);
    // let _trng_source = TrngSource::new(peripherals.RNG, peripherals.ADC1);
    // let mut trng = Trng::try_new().unwrap();
    esp_alloc::heap_allocator!(size: 72 * 1024);
    info!("System and allocator initialized");

    info!("CAN initialization");
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
    let obd2 = OBD_SERVICE.init(Obd2Service::new(iso_tp, false));
    info!("OBD2 services configured");

    let connector = match BleConnector::new(peripherals.BT, Default::default()) {
        Ok(c) => c,
        Err(_e) => {
            error!("BLE Connector init failed");
            panic!("BLE Init Error");
        }
    };
    let controller = ExternalController::new(connector);

    let resources = BLE_STACK_RESOURCES.init(BleResources::new());
    let stack = BLE_STACK.init_with(|| trouble_host::new(controller, resources));
    let host = stack.build();
    info!("BLE stack built successfully");
    let peripheral = host.peripheral;
    let runner = host.runner;

    spawner.spawn(ble_runner_task(runner)).unwrap();
    info!("BLE runner task started");

    spawner
        .spawn(ble_service_task(stack, peripheral, obd2))
        .unwrap();
    info!("BLE servise task started");

    // -- End

    pending::<()>().await;
    info!("All tasks spawned. Entering pending state.");
}

// Керує подіями HCI.
#[embassy_executor::task]
async fn ble_runner_task(mut runner: ObdRunner) {
    info!("BLE Runner task is running...");
    if let Err(e) = runner.run().await {
        error!("BLE Runner exited with error: {:?}", e);
    }
}

#[embassy_executor::task]
async fn ble_service_task(
    stack: &'static ObdStack,
    mut peripheral: ObdPeripheral,
    obd_service: &'static Obd2Service<EspCanManager<'static>>,
) {
    info!("BLE Service task started");
    let mut adv_data = [0; 31];

    let adv = match create_advertisement(&mut adv_data) {
        Ok(a) => a,
        Err(e) => {
            error!("Failed to create advertisement: {:?}", e);
            return;
        }
    };

    loop {
        info!("Starting advertisement...");
        match peripheral.advertise(&Default::default(), adv).await {
            Ok(advertiser) => {
                info!("Advertising. Waiting for connection...");
                match advertiser.accept().await {
                    Ok(conn) => {
                        info!("Device connected!");

                        let mut stream = BleStream::new(&STREAM_TX, &STREAM_RX);

                        let l2cap_task = run_connection(stack, &conn, &STREAM_TX, &STREAM_RX);
                        let app_task = handle_client(&mut stream, obd_service);

                        match select(l2cap_task, app_task).await {
                            Either::First(Err(e)) => {
                                error!("Connection closed with error: {:?}", e)
                            }
                            Either::First(Ok(_)) => info!("Connection closed normally (L2CAP)"),
                            Either::Second(_) => info!("App task finished"),
                        }
                    }
                    Err(e) => error!("Accept error: {:?}", e),
                }
            }
            Err(e) => {
                error!("Advertise error: {:?}", e);
                embassy_time::Timer::after_secs(1).await;
            }
        }
    }
}
