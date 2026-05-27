#![no_std]
#![no_main]

extern crate alloc;

use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use esp_backtrace as _;
use esp_hal::delay::Delay;
use esp_println::{print, println};
use lf_hal::line_sensor::{LedIndex, LineSensor};
use line_follower::line_detection::detect_line;

const LED_SCAN_INTERVAL: Duration = Duration::from_millis(50);

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    let mut hal = line_follower::init!();
    let delay = Delay::new();

    loop {
        let buf = hal.line_sensor.read(&delay);
        print!("Raw:");
        for v in buf.values {
            print!(" {:4}", v);
        }
        println!();

        let detections = detect_line(&buf);
        println!("Detections:");
        if detections.is_empty() {
            println!(" no line");
        }
        for d in &detections {
            print!(" {:+.3}({:.2})", d.position, d.strength);
        }
        println!();

        let buf = hal.line_sensor.read_raw();
        print!("Ambient:");
        for v in buf {
            print!(" {:4}", v);
        }
        println!();

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
