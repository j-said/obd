///! Модуль-директорія реалізацій транспортного рівня.
///!
///! Дозволяє підміняти реалізацію (BLE, WiFi, Serial) без зміни логіки програми у main.rs
pub mod ble;



// Костиль для конфігурації через env. Виконується ТІЛЬКИ під час компіляції.
pub(crate) const fn parse_usize(s: &str) -> usize {
    let bytes = s.as_bytes();
    let mut res = 0;
    let mut i = 0;

    while i < bytes.len() {
        let b = bytes[i];
        assert!(b.is_ascii_digit(), "Invalid number in env");
        res = res * 10 + (b - b'0') as usize;
        i += 1;
    }
    res
}
