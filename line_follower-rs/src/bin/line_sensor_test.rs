#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{delay::Delay, main, time::Duration};
use esp_println::println;
use line_follower::hal::Hal;

const STEP_TIME: Duration = Duration::from_millis(100);

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let p = esp_hal::init(esp_hal::Config::default());
    let mut hal = Hal::new(p);
    let delay = Delay::new();

    println!("Line sensor test — scanning LEDs");

    let sequence = [0, 1, 2, 3, 4, 5, 4, 3, 2, 1];

    loop {
        for &led in &sequence {
            println!("LED {}", led);
            hal.line_sensor.enable_led(led);
            delay.delay(STEP_TIME);
        }
    }
}
