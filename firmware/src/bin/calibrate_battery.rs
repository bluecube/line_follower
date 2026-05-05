#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{delay::Delay, main};
use esp_println::println;
use lf_hal::{Hal, button::ButtonEvent};

esp_bootloader_esp_idf::esp_app_desc!();

const VOLTAGES: [f32; 2] = [8.0, 12.0];
const SAMPLE_COUNT: u32 = 500;

fn wait_for_release(hal: &mut Hal<'_>) {
    loop {
        if let Some(ButtonEvent::Release(_)) = hal.deck_button.poll() {
            return;
        }
    }
}

fn print_voltages(hal: &mut Hal<'_>, delay: &Delay) {
    loop {
        if let Some(ButtonEvent::Release(_)) = hal.deck_button.poll() {
            break;
        }
        let v = hal.read_battery();
        println!("voltage = {:.2} V (raw = {})", v.voltage(), v.raw);
        delay.delay_millis(500);
    }
}

fn sample_raw_average(hal: &mut Hal<'_>, delay: &Delay) -> f32 {
    (0..SAMPLE_COUNT)
        .map(|_| {
            delay.delay_millis(5);
            hal.read_battery().raw as u32
        })
        .sum::<u32>() as f32
        / SAMPLE_COUNT as f32
}

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let p = esp_hal::init(esp_hal::Config::default());
    let mut hal = Hal::new(p);
    let delay = Delay::new();

    print_voltages(&mut hal, &delay);

    let mut raws = [0f32; 2];
    for (i, &voltage) in VOLTAGES.iter().enumerate() {
        println!(
            "Set battery to {:.0} V, then press the deck button.",
            voltage
        );
        wait_for_release(&mut hal);
        raws[i] = sample_raw_average(&mut hal, &delay);
        println!("  raw avg = {:.1}", raws[i]);
    }

    let k = (VOLTAGES[1] - VOLTAGES[0]) / (raws[1] - raws[0]);
    let a = VOLTAGES[0] - k * raws[0];

    println!("--- result ---");
    println!("const K: f32 = {:.6};", k);
    println!("const A: f32 = {:.6};", a);

    loop {}
}
