#![no_std]
#![no_main]

extern crate alloc;

use bt_hci::controller::ExternalController;
use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::twai::{self, TwaiConfiguration, TwaiMode};
use esp_radio::ble::controller::BleConnector;
use static_cell::StaticCell;
use trouble_host::prelude::*;

// Імпорти з нашої бібліотеки
use obd_rust::can::{CanManager, SharedTwaiRx, SharedTwaiTx};
// use obd_rust::can::obd2::Obd2Service; // Розкоментуйте, коли будете використовувати

// --- Глобальні канали ---
static TX_CHANNEL: Channel<CriticalSectionRawMutex, heapless::Vec<u8, 128>, 2> = Channel::new();
static RX_CHANNEL: Channel<CriticalSectionRawMutex, heapless::Vec<u8, 128>, 2> = Channel::new();

// --- Статичні ресурси (пам'ять для драйверів) ---
static BLE_RESOURCES: StaticCell<HostResources<DefaultPacketPool, 1, 2>> = StaticCell::new();
static BLE_STACK: StaticCell<Stack<ExternalController<BleConnector<'static>, 1>>> = StaticCell::new();

static CAN_TX_MUTEX: StaticCell<SharedTwaiTx> = StaticCell::new();
static CAN_RX_MUTEX: StaticCell<SharedTwaiRx> = StaticCell::new();

// --- UUIDs ---
const SERVICE_UUID: Uuid = Uuid::new_long([
    0xAB, 0xCD, 0xEF, 0x12, 0x34, 0x56, 0x78, 0x90, 0xAB, 0xCD, 0xEF, 0x12, 0x34, 0x56, 0x78, 0x90,
]);
const RX_CHAR_UUID: Uuid = Uuid::new_long([
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x01,
]);
const TX_CHAR_UUID: Uuid = Uuid::new_long([
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x02,
]);

// --- Задача 1: BLE (Transport Layer) ---
#[embassy_executor::task]
async fn ble_task(stack: &'static Stack<ExternalController<BleConnector<'static>, 1>>) {
    let mut table = AttributeTable::new();
    let mut service = table.add_service(Service::new(SERVICE_UUID));

    // Буфери (повинні жити протягом життя сервісу)
    let mut rx_storage = [0u8; 128];
    let mut tx_storage = [0u8; 128];

    // --- ВИПРАВЛЕННЯ BORROW CHECKER ---
    // Ми одразу викликаємо .handle(), щоб "відпустити" service
    
    let rx_props = [CharacteristicProp::Write, CharacteristicProp::WriteWithoutResponse];
    let rx_handle = service.add_characteristic(
        RX_CHAR_UUID,
        &rx_props,
        &[],
        &mut rx_storage,
    ).handle();

    let tx_props = [CharacteristicProp::Notify, CharacteristicProp::Read];
    let tx_handle = service.add_characteristic(
        TX_CHAR_UUID, 
        &tx_props, 
        &[], 
        &mut tx_storage
    ).handle();

    let server = AttributeServer::new(stack, &mut table);

    loop {
        info!("BLE: Advertising...");
        let mut advertiser = match stack.advertise().await {
            Ok(a) => a,
            Err(e) => {
                error!("BLE: Advertise error: {:?}", e);
                Timer::after(Duration::from_secs(1)).await;
                continue;
            }
        };

        let conn = match advertiser.accept().await {
            Ok(c) => c,
            Err(e) => {
                error!("BLE: Connection error: {:?}", e);
                continue;
            }
        };

        info!("BLE: Connected!");

        // Обробка подій
        let server_future = server.run(&conn, |event| {
            match event {
                AttributeServerEvent::Write(handle, data) => {
                    if handle == rx_handle {
                        if let Ok(vec) = heapless::Vec::from_slice(data) {
                            let _ = RX_CHANNEL.try_send(vec);
                        }
                    }
                }
                _ => {}
            }
        });

        let tx_sender_future = async {
            loop {
                let data = TX_CHANNEL.receive().await;
                if let Err(e) = conn.notify(tx_handle, &data).await {
                    warn!("BLE: Notify failed: {:?}", e);
                }
            }
        };

        use embassy_futures::select::{select3, Either3};
        match select3(server_future, conn.disconnected(), tx_sender_future).await {
            Either3::First(Err(e)) => error!("BLE: Server error: {:?}", e),
            Either3::Second(_) => info!("BLE: Disconnected"),
            Either3::Third(_) => {}, // TX loop shouldn't exit
            _ => {},
        }
    }
}

// --- Задача 2: Application Logic ---
#[embassy_executor::task]
async fn app_logic_task(can_manager: &'static CanManager<'static>) {
    info!("App: Task started");
    
    // Тут ви можете створити сервіси, використовуючи can_manager
    // let iso_tp = obd_rust::can::iso_tp::IsoTpHandler::new(can_manager);
    // let obd_service = obd_rust::can::obd2::Obd2Service::new(iso_tp);

    loop {
        let cmd = RX_CHANNEL.receive().await;
        info!("App: Command: {:x}", cmd.as_slice());

        // Приклад: Просто ехо-відповідь + статус CAN
        let mut response: heapless::Vec<u8, 128> = heapless::Vec::new();
        
        // Тут буде ваша логіка обробки OBD...
        // let res = obd_service.get_vin().await;
        
        // Для тесту відправляємо назад те, що отримали
        if response.extend_from_slice(&cmd).is_ok() {
            TX_CHANNEL.send(response).await;
        }
    }
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    use panic_rtt_target as _;
    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 66320);
    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    info!("System: Init...");

    // 1. Ініціалізація CAN (TWAI)
    // Налаштуйте піни під вашу плату (тут GPIO4/GPIO5 як приклад для ESP32-C3 SuperMini)
    let twai_config = TwaiConfiguration::default();
    let (tx_pin, rx_pin) = (peripherals.GPIO4, peripherals.GPIO5); 
    
    let twai = twai::Twai::new(peripherals.TWAI0, rx_pin, tx_pin, &twai_config);
    let (tx, rx) = twai.split();

    // Створюємо глобальні м'ютекси для CAN
    let can_tx = CAN_TX_MUTEX.init(Mutex::new(tx));
    let can_rx = CAN_RX_MUTEX.init(Mutex::new(rx));
    
    // Створюємо менеджер (він буде жити вічно на стеку main, але ми передамо посилання)
    // Увага: оскільки app_logic_task вимагає 'static, ми "витікаємо" CanManager або створюємо його як static
    static CAN_MANAGER: StaticCell<CanManager> = StaticCell::new();
    let can_manager = CAN_MANAGER.init(CanManager::new(can_tx, can_rx));

    // 2. Ініціалізація BLE
    let radio_init = esp_radio::init().expect("Radio init failed");
    let transport = BleConnector::new(&radio_init, peripherals.BT, Default::default()).unwrap();
    let ble_controller = ExternalController::<_, 1>::new(transport);

    let ble_resources = BLE_RESOURCES.init(HostResources::new());
    let ble_stack = BLE_STACK.init(trouble_host::new(ble_controller, ble_resources));

    // 3. Запуск задач
    spawner.spawn(ble_task(ble_stack)).unwrap();
    spawner.spawn(app_logic_task(can_manager)).unwrap();

    info!("System: Ready!");
    
    loop {
        Timer::after(Duration::from_secs(10)).await;
    }
}