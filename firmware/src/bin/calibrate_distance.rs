#![no_std]
#![no_main]

extern crate alloc;

use embassy_executor::Spawner;
use embassy_futures::select::select;
use embassy_time::{Duration, Ticker, Timer};
use esp_backtrace as _;
use lf_hal::Hal;

esp_bootloader_esp_idf::esp_app_desc!();

const DISTANCES_CM: [f32; 2] = [10.0, 50.0];
const SAMPLE_COUNT: u32 = 500;
const PRINT_INTERVAL: Duration = Duration::from_millis(500);
const SAMPLE_INTERVAL: Duration = Duration::from_millis(5);

/// Print readings until the button is pressed.
async fn print_readings(hal: &mut Hal<'_>) {
    let mut ticker = Ticker::every(PRINT_INTERVAL);
    loop {
        let r = hal.read_range();
        log::info!("long = {} (raw = {})", r.distance_long(), r.raw);
        if select(ticker.next(), hal.deck_button.released())
            .await
            .is_second()
        {
            return;
        }
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
async fn main(spawner: Spawner) {
    let mut hal = line_follower::init!(spawner);

    print_readings(&mut hal).await;

    let mut raws = [0f32; 2];
    for (i, &distance_cm) in DISTANCES_CM.iter().enumerate() {
        log::info!("Place object at {distance_cm:.0} cm, then press the deck button.");
        hal.deck_button.released().await;
        log::info!("averaging...");
        raws[i] = sample_raw_average(&mut hal).await;
        log::info!("  raw avg = {:.1}", raws[i]);
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

    log::info!("--- result ---");
    log::info!("const A: f32 = {a:.6};");
    log::info!("const B: f32 = {b:.6};");

    core::future::pending::<()>().await;
}
