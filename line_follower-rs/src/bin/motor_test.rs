#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    main,
    time::{Duration, Instant},
};
use esp_println::println;
use line_follower::hal::{
    Hal,
    motors::{PwmT, Motors},
};

const TEST_PWM: i16 = PwmT::MAX.get() * 3 / 4;
const RUN_MS: u64 = 3_000;
const SAMPLE_MS: u32 = 200;
const BRAKE_MS: u32 = 500;
const SAMPLE_SECS: f32 = SAMPLE_MS as f32 / 1000.0;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let p = esp_hal::init(esp_hal::Config::default());

    let mut hal = Hal::new(p);
    let delay = Delay::new();

    loop {
        run_phase(&mut hal.motors, &delay, TEST_PWM, 0, "LEFT  FORWARD");
        brake(&mut hal.motors, &delay);
        run_phase(&mut hal.motors, &delay, -TEST_PWM, 0, "LEFT  BACKWARD");
        brake(&mut hal.motors, &delay);
        run_phase(&mut hal.motors, &delay, 0, TEST_PWM, "RIGHT FORWARD");
        brake(&mut hal.motors, &delay);
        run_phase(&mut hal.motors, &delay, 0, -TEST_PWM, "RIGHT BACKWARD");
        brake(&mut hal.motors, &delay);
    }
}

fn run_phase(motors: &mut Motors<'_>, delay: &Delay, left: i16, right: i16, label: &str) {
    let pwm = if left != 0 { left } else { right };
    println!("\n=== {} (pwm={}) ===", label, pwm);
    motors.set(left.try_into().unwrap(), right.try_into().unwrap());

    let start = Instant::now();
    let mut last_enc = motors.encoders();

    while start.elapsed() < Duration::from_millis(RUN_MS) {
        delay.delay_millis(SAMPLE_MS);
        let enc = motors.encoders();
        let vel_l = enc.0.wrapping_sub(last_enc.0) as f32 / SAMPLE_SECS;
        let vel_r = enc.1.wrapping_sub(last_enc.1) as f32 / SAMPLE_SECS;
        println!(
            "enc=({:6},{:6})  vel=({:6.0},{:6.0}) ticks/s",
            enc.0, enc.1, vel_l, vel_r
        );
        last_enc = enc;
    }
}

fn brake(motors: &mut Motors<'_>, delay: &Delay) {
    motors.set(0.try_into().unwrap(), 0.try_into().unwrap());
    println!("--- braking ---");
    delay.delay_millis(BRAKE_MS);
}
