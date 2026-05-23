#![no_std]
#![no_main]

extern crate alloc;

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_println::println;
use lf_hal::{Hal, button::ButtonEvent};

esp_bootloader_esp_idf::esp_app_desc!();

const VOLTAGES: [f32; 2] = [8.0, 12.0];
const SAMPLE_COUNT: u32 = 500;
const PRINT_INTERVAL: Duration = Duration::from_millis(500);
const SAMPLE_INTERVAL: Duration = Duration::from_millis(5);

async fn wait_for_release(hal: &mut Hal<'_>) {
    loop {
        if let Some(ButtonEvent::Release(_)) = hal.deck_button.poll() {
            return;
        }
        Timer::after_millis(10).await;
    }
}

async fn print_voltages(hal: &mut Hal<'_>) {
    loop {
        if let Some(ButtonEvent::Release(_)) = hal.deck_button.poll() {
            break;
        }
        let v = hal.read_battery();
        println!("voltage = {:.2} V (raw = {})", v.voltage(), v.raw);
        Timer::after(PRINT_INTERVAL).await;
    }
}

async fn sample_raw_average(hal: &mut Hal<'_>) -> f32 {
    let mut sum = 0u32;
    for _ in 0..SAMPLE_COUNT {
        Timer::after(SAMPLE_INTERVAL).await;
        sum += hal.read_battery().raw as u32;
    }
    sum as f32 / SAMPLE_COUNT as f32
}

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    let mut hal = line_follower::init!();

    print_voltages(&mut hal).await;

    let mut raws = [0f32; 2];
    for (i, &voltage) in VOLTAGES.iter().enumerate() {
        println!(
            "Set battery to {:.0} V, then press the deck button.",
            voltage
        );
        wait_for_release(&mut hal).await;
        raws[i] = sample_raw_average(&mut hal).await;
        println!("  raw avg = {:.1}", raws[i]);
    }

    let k = (VOLTAGES[1] - VOLTAGES[0]) / (raws[1] - raws[0]);
    let a = VOLTAGES[0] - k * raws[0];

    println!("--- result ---");
    println!("const K: f32 = {:.6};", k);
    println!("const A: f32 = {:.6};", a);

    loop {}
}
