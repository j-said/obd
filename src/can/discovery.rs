use super::config;
use super::iso_tp::IsoTpHandler;
use embassy_time::{Duration, Timer};

pub struct DiscoveryService<'a> {
    tp: IsoTpHandler<'a>,
}

impl<'a> DiscoveryService<'a> {
    pub fn new(tp: IsoTpHandler<'a>) -> Self {
        Self { tp }
    }

    /// Пробує пінгувати машину різними типами ID, доки не отримає відповідь
    pub async fn run_discovery(&self) {
        loop {
            // Пінгуємо стандартним ID
            if self.tp.send_request(0x7E0, &[0x01, 0x00]).await.is_ok() {
                let _ = config::set_protocol(false);
                break;
            }
            // Пінгуємо розширеним ID (Functional Extended)
            if self
                .tp.send_request(0x18DB33F1, &[0x01, 0x00]).await.is_ok()
            {
                let _ = config::set_protocol(true);
                break;
            }
            Timer::after(Duration::from_secs(2)).await;
        }
    }
}
