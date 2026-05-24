#![no_std]
#![no_main]

extern crate alloc;

use embassy_executor::Spawner;
use embassy_futures::select::select;
use embassy_time::{Duration, Ticker, Timer};
use esp_backtrace as _;
use lf_hal::Hal;

esp_bootloader_esp_idf::esp_app_desc!();

const VOLTAGES: [f32; 2] = [8.0, 12.0];
const SAMPLE_COUNT: u32 = 500;
const PRINT_INTERVAL: Duration = Duration::from_millis(500);
const SAMPLE_INTERVAL: Duration = Duration::from_millis(5);

async fn print_voltages(hal: &mut Hal<'_>) {
    let mut ticker = Ticker::every(PRINT_INTERVAL);
    loop {
        let v = hal.read_battery();
        log::info!("voltage = {:.2} V (raw = {})", v.voltage(), v.raw);
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
        sum += hal.read_battery().raw as u32;
    }
    sum as f32 / SAMPLE_COUNT as f32
}

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    let mut hal = line_follower::init!(spawner);

    print_voltages(&mut hal).await;

    let mut raws = [0f32; 2];
    for (i, &voltage) in VOLTAGES.iter().enumerate() {
        log::info!(
            "Set battery to {:.0} V, then press the deck button.",
            voltage
        );
        hal.deck_button.released().await;
        log::info!("averaging...");
        raws[i] = sample_raw_average(&mut hal).await;
        log::info!("  raw avg = {:.1}", raws[i]);
    }

    let k = (VOLTAGES[1] - VOLTAGES[0]) / (raws[1] - raws[0]);
    let a = VOLTAGES[0] - k * raws[0];

    log::info!("--- result ---");
    log::info!("const K: f32 = {:.6};", k);
    log::info!("const A: f32 = {:.6};", a);

    loop {}
}
