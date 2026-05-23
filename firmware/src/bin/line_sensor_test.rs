#![no_std]
#![no_main]

extern crate alloc;

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::delay::Delay;
use esp_println::{print, println};
use lf_hal::{button::ButtonEvent, line_sensor::LedIndex};
use line_follower::line_detection::detect_line;

const LED_SCAN_INTERVAL: Duration = Duration::from_millis(50);

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    let mut hal = line_follower::init!();
    let delay = Delay::new();

    let led_scan_sequence = [0usize, 1, 2, 3, 4, 5, 4, 3, 2, 1];
    let mut led_on = false;

    loop {
        if let Some(ButtonEvent::Release(_)) = hal.boot_button.poll() {
            led_on = !led_on;
            hal.set_led(led_on);
        }

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

        // Using the LED scan animation as a delay.
        for led_index in led_scan_sequence {
            hal.line_sensor
                .enable_led(LedIndex::new(led_index).unwrap());
            Timer::after(LED_SCAN_INTERVAL).await;
        }
    }
}
