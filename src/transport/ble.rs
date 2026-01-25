use super::TransportAdapter;
use defmt::{error, info, warn};
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use trouble_host::prelude::*;

/// UUID сервісу (приклад: 128-бітний рандомний UUID)
const SERVICE_UUID: Uuid = Uuid::new_long([
    0xAB, 0xCD, 0xEF, 0x12, 0x34, 0x56, 0x78, 0x90, 0xAB, 0xCD, 0xEF, 0x12, 0x34, 0x56, 0x78, 0x90,
]);

/// UUID характеристики для запису (RX - отримання від телефону)
const RX_CHAR_UUID: Uuid = Uuid::new_long([
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x01,
]);

/// UUID характеристики для сповіщень (TX - відправка на телефон)
const TX_CHAR_UUID: Uuid = Uuid::new_long([
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x02,
]);

pub struct BleTransport<'a, C> {
    stack: &'a Stack<C>,
    connection: Option<Connection<'a>>,
    rx_channel: Channel<CriticalSectionRawMutex, heapless::Vec<u8, 256>, 2>,
    tx_handle: Option<u16>,
}

impl<'a, C> BleTransport<'a, C>
where
    C: Controller,
{
    pub fn new(stack: &'a Stack<C>) -> Self {
        Self {
            stack,
            connection: None,
            rx_channel: Channel::new(),
            tx_handle: None,
        }
    }

    pub async fn run(&mut self) {
        let mut table = AttributeTable::new();

        // Ці буфери повинні жити стільки ж, скільки живе сервіс.
        // Оскільки run() крутиться у вічному циклі, стек-пам'ять тут підходить.
        let mut rx_storage = [0u8; 256];
        let mut tx_storage = [0u8; 256];

        // Створюємо сервіс
        let mut service = table.add_service(Service::new(SERVICE_UUID));

        // --- Додавання RX Характеристики (Write) ---
        // Використовуємо сигнатуру: add_characteristic(uuid, props, value, store)
        let rx_props = [
            CharacteristicProp::Write,
            CharacteristicProp::WriteWithoutResponse,
        ];

        let rx_builder = service.add_characteristic(
            RX_CHAR_UUID,
            &rx_props,
            &[],             // Початкове значення (порожнє)
            &mut rx_storage, // Мутабельний буфер
        );
        let rx_char_handle = rx_builder.handle(); // Отримуємо u16 handle з білдера

        // --- Додавання TX Характеристики (Notify) ---
        let tx_props = [CharacteristicProp::Notify, CharacteristicProp::Read];

        let tx_builder = service.add_characteristic(TX_CHAR_UUID, &tx_props, &[], &mut tx_storage);
        let tx_char_handle = tx_builder.handle();

        // Зберігаємо handle для відправки
        self.tx_handle = Some(tx_char_handle);

        // Реєструємо сервер
        let server = AttributeServer::new(table);

        loop {
            info!("BLE: Запуск реклами...");
            let mut advertiser = match self.stack.advertise().await {
                Ok(a) => a,
                Err(e) => {
                    error!("BLE: Помилка запуску реклами: {:?}", e);
                    Timer::after(Duration::from_secs(1)).await;
                    continue;
                }
            };

            let conn = match advertiser.accept().await {
                Ok(c) => c,
                Err(e) => {
                    error!("BLE: Помилка підключення: {:?}", e);
                    continue;
                }
            };

            info!("BLE: Клієнт підключено!");
            self.connection = Some(conn.clone());

            // Обробка GATT подій
            let process_gatt = server.run(&conn, |event| match event {
                AttributeServerEvent::Write(handle, data) => {
                    if handle == rx_char_handle {
                        info!("BLE: Отримано дані ({} байт)", data.len());
                        if let Ok(vec) = heapless::Vec::from_slice(data) {
                            if let Err(_) = self.rx_channel.try_send(vec) {
                                warn!("BLE: Буфер переповнено");
                            }
                        }
                    }
                }
                _ => {}
            });

            match select(process_gatt, conn.disconnected()).await {
                Either::First(Err(e)) => error!("BLE: Помилка GATT: {:?}", e),
                Either::Second(_) => info!("BLE: Клієнт відключився"),
                _ => {}
            }

            self.connection = None;
        }
    }
}

impl<'a, C: Controller> TransportAdapter for BleTransport<'a, C> {
    type Error = ();

    async fn wait_for_connection(&mut self) -> Result<(), Self::Error> {
        while self.connection.is_none() {
            Timer::after(Duration::from_millis(100)).await;
        }
        Ok(())
    }

    async fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Self::Error> {
        let packet = self.rx_channel.receive().await;
        let len = packet.len().min(buffer.len());
        buffer[0..len].copy_from_slice(&packet[0..len]);
        Ok(len)
    }

    async fn write(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        if let Some(conn) = &self.connection {
            if let Some(handle) = self.tx_handle {
                match conn.notify(handle, data).await {
                    Ok(_) => Ok(()),
                    Err(e) => {
                        warn!("BLE Notify error: {:?}", e);
                        Err(())
                    }
                }
            } else {
                Err(())
            }
        } else {
            Err(())
        }
    }

    fn is_connected(&self) -> bool {
        self.connection.is_some()
    }
}
