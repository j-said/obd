#![no_std]
#![no_main]

use embassy_executor::Spawner;
use panic_rtt_target as _;


#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    loop {
        embassy_time::Timer::after_millis(1000).await;
    }
}
