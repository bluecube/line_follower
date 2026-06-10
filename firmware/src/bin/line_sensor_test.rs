#![no_std]
#![no_main]

use core::fmt::Write as _;

use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use esp_backtrace as _;
use heapless::String;
use lf_hal::line_sensor::{LedIndex, LineSensor};
use line_follower::line_detection::detect_line;

const LED_SCAN_INTERVAL: Duration = Duration::from_millis(50);

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    let mut hal = line_follower::init!(spawner);

    loop {
        let buf = hal.line_sensor.read().await;
        let mut s: String<64> = String::new();
        let _ = write!(s, "Raw:");
        for v in buf.values {
            let _ = write!(s, " {v:4}");
        }
        log::info!("{s}");

        let detections = detect_line(&buf);
        if detections.is_empty() {
            log::info!("Detections: no line");
        } else {
            let mut s: String<128> = String::new();
            let _ = write!(s, "Detections:");
            for d in &detections {
                let _ = write!(s, " {:+.3}({:.2})", d.position, d.strength);
            }
            log::info!("{s}");
        }

        let buf = hal.line_sensor.read_raw();
        let mut s: String<64> = String::new();
        let _ = write!(s, "Ambient:");
        for v in buf {
            let _ = write!(s, " {v:4}");
        }
        log::info!("{s}");

        led_scan(&mut hal.line_sensor, LED_SCAN_INTERVAL).await;
    }
}

async fn led_scan(line_sensor: &mut LineSensor<'_>, interval: Duration) {
    let mut ticker = Ticker::every(interval);
    for i in [0usize, 1, 2, 3, 4, 5, 4, 3, 2, 1] {
        line_sensor.enable_led(LedIndex::new(i).unwrap());
        ticker.next().await;
    }
}
