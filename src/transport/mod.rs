//! Модуль абстракції транспортного рівня.
//! Дозволяє підміняти реалізацію (BLE, WiFi, Serial) без зміни логіки програми.

/// Трейт адаптера транспорту.
/// Відповідає рівню L4 (Transport Layer) моделі OSI.

pub mod ble;

pub trait TransportAdapter {
    ///! Абстракція

    type Error;

    /// Очікування встановлення з'єднання
    async fn wait_for_connection(&mut self) -> Result<(), Self::Error>;

    /// Повертає кількість прочитаних байт
    async fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Self::Error>;

    /// Відправка даних
    async fn write(&mut self, data: &[u8]) -> Result<(), Self::Error>;

    /// Перевірка статусу з'єднання
    fn is_connected(&self) -> bool;
}