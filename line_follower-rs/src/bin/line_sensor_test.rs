#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{delay::Delay, main, time::Duration};
use esp_println::{print, println};
use lf_hal::{Hal, line_sensor::LedIndex};

const LED_SCAN_INTERVAL: Duration = Duration::from_millis(50);

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let p = esp_hal::init(esp_hal::Config::default());
    let mut hal = Hal::new(p);
    let delay = Delay::new();

    let led_scan_sequence = [0usize, 1, 2, 3, 4, 5, 4, 3, 2, 1];

    loop {
        let buf = hal.line_sensor.read(&delay);
        print!("Raw:");
        for v in buf.values {
            print!(" {:4}", v);
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
            delay.delay(LED_SCAN_INTERVAL);
        }
    }
}
