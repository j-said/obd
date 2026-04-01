///! Модуль-директорія реалізацій транспортного рівня.
///!
///! Дозволяє підміняти реалізацію (BLE, WiFi, Serial) без зміни логіки програми у main.rs
pub mod ble;

// TODO: Add support for WiFi and Serial transport layers 