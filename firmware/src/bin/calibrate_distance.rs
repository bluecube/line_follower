#![no_std]
#![no_main]

extern crate alloc;

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_println::println;
use lf_hal::{Hal, button::ButtonEvent};

esp_bootloader_esp_idf::esp_app_desc!();

const DISTANCES_CM: [f32; 2] = [10.0, 50.0];
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

async fn print_readings(hal: &mut Hal<'_>) {
    loop {
        if let Some(ButtonEvent::Release(_)) = hal.deck_button.poll() {
            break;
        }
        let r = hal.read_range();
        println!("long = {} (raw = {})", r.distance_long(), r.raw);
        Timer::after(PRINT_INTERVAL).await;
    }
}

async fn sample_raw_average(hal: &mut Hal<'_>) -> f32 {
    let mut sum = 0u32;
    for _ in 0..SAMPLE_COUNT {
        Timer::after(SAMPLE_INTERVAL).await;
        sum += hal.read_range().raw as u32;
    }
    sum as f32 / SAMPLE_COUNT as f32
}

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    let mut hal = line_follower::init!();

    print_readings(&mut hal).await;

    let mut raws = [0f32; 2];
    for (i, &distance_cm) in DISTANCES_CM.iter().enumerate() {
        println!(
            "Place object at {:.0} cm, then press the deck button.",
            distance_cm
        );
        wait_for_release(&mut hal).await;
        raws[i] = sample_raw_average(&mut hal).await;
        println!("  raw avg = {:.1}", raws[i]);
    }

    // Fit distance_m = a / (raw + b) using two points (distances in metres).
    // From d1*(r1+b) = d2*(r2+b):
    //   b = (d2*r2 - d1*r1) / (d1 - d2)
    //   a = d1 * (r1 + b)
    let d1 = DISTANCES_CM[0] / 100.0;
    let d2 = DISTANCES_CM[1] / 100.0;
    let r1 = raws[0];
    let r2 = raws[1];
    let b = (d2 * r2 - d1 * r1) / (d1 - d2);
    let a = d1 * (r1 + b);

    println!("--- result ---");
    println!("const A: f32 = {:.6};", a);
    println!("const B: f32 = {:.6};", b);

    loop {}
}
