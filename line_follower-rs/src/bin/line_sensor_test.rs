#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{delay::Delay, main, time::Duration};
use esp_println::{print, println};
use line_follower::hal::{Hal, button::ButtonEvent};

const STEP_TIME: Duration = Duration::from_millis(100);
const READ_INTERVAL: Duration = Duration::from_millis(100);

#[derive(Clone, Copy)]
enum Mode {
    LedScan,
    FullRead,
    RawRead,
}

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let p = esp_hal::init(esp_hal::Config::default());
    let mut hal = Hal::new(p);
    let delay = Delay::new();

    let mut mode = Mode::LedScan;
    println!("Line sensor test - press button to switch mode");
    println!("Mode: LED scan");

    let sequence = [0usize, 1, 2, 3, 4, 5, 4, 3, 2, 1];
    let mut seq_idx = 0;

    loop {
        if let Some(ButtonEvent::Press) = hal.deck_button.poll() {
            mode = match mode {
                Mode::LedScan => {
                    hal.line_sensor.disable_leds();
                    println!("Mode: full read");
                    Mode::FullRead
                }
                Mode::FullRead => {
                    hal.line_sensor.disable_leds();
                    println!("Mode: raw read");
                    Mode::RawRead
                }
                Mode::RawRead => {
                    println!("Mode: LED scan");
                    Mode::LedScan
                }
            };
        }

        match mode {
            Mode::LedScan => {
                hal.line_sensor.enable_led(sequence[seq_idx]);
                seq_idx = (seq_idx + 1) % sequence.len();
                delay.delay(STEP_TIME);
            }
            Mode::FullRead => {
                let buf = hal.line_sensor.read(&delay);
                for v in buf {
                    print!(" {:4}", v);
                }
                println!();

                delay.delay(READ_INTERVAL);
            }
            Mode::RawRead => {
                let buf = hal.line_sensor.read_raw();
                for v in buf {
                    print!(" {:4}", v);
                }
                println!();
                delay.delay(READ_INTERVAL);
            }
        }
    }
}
