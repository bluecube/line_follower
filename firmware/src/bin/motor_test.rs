#![no_std]
#![no_main]

extern crate alloc;

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
use esp_println::println;
use lf_hal::{Hal, button::ButtonEvent, motors::PwmT};

const TEST_PWM: i16 = PwmT::MAX.get() * 3 / 4;
const SAMPLE_TIME: Duration = Duration::from_millis(500);
const BRAKE_TIME: Duration = Duration::from_millis(500);
const LONG_PRESS_TIME: Duration = Duration::from_millis(500);

const PHASES: &[(&str, i16, i16)] = &[
    ("LEFT FORWARD", TEST_PWM, 0),
    ("LEFT BACKWARD", -TEST_PWM, 0),
    ("RIGHT FORWARD", 0, TEST_PWM),
    ("RIGHT BACKWARD", 0, -TEST_PWM),
];

#[derive(Clone, Copy)]
enum Phase {
    Running(usize),
    Stopped,
}

enum PhaseAction {
    Advance,
    Stop,
}

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    let mut hal = line_follower::init!();

    println!("Motor test -- short press: next phase  long press: stop");

    let mut phase = Phase::Stopped;

    loop {
        let action = match phase {
            Phase::Running(i) => {
                let (label, left, right) = PHASES[i];
                run_phase(&mut hal, left, right, label).await
            }
            Phase::Stopped => run_phase(&mut hal, 0, 0, "STOPPED").await,
        };

        brake(&mut hal).await;

        phase = match action {
            PhaseAction::Advance => match phase {
                Phase::Running(i) => Phase::Running((i + 1) % PHASES.len()),
                Phase::Stopped => Phase::Running(0),
            },
            PhaseAction::Stop => Phase::Stopped,
        };
    }
}

async fn run_phase(hal: &mut Hal<'_>, left: i16, right: i16, label: &str) -> PhaseAction {
    println!("\n=== {}, pwm=({}, {}) ===", label, left, right);
    hal.motors
        .set(left.try_into().unwrap(), right.try_into().unwrap());

    let mut last_enc = hal.motors.encoders();
    let mut last_sample = Instant::now();

    loop {
        match hal.deck_button.poll_with_threshold(LONG_PRESS_TIME) {
            Some(ButtonEvent::LongPress) => return PhaseAction::Stop,
            Some(ButtonEvent::Release(_)) => return PhaseAction::Advance,
            _ => {}
        }
        let elapsed = last_sample.elapsed();
        if elapsed >= SAMPLE_TIME {
            let elapsed_secs = elapsed.as_micros() as f32 / 1e6;
            let enc = hal.motors.encoders();
            let vel_l = enc.0.wrapping_sub(last_enc.0) as f32 / elapsed_secs;
            let vel_r = enc.1.wrapping_sub(last_enc.1) as f32 / elapsed_secs;
            println!(
                "enc=({:6},{:6})  vel=({:6.0},{:6.0}) ticks/s",
                enc.0, enc.1, vel_l, vel_r
            );
            last_enc = enc;
            last_sample = Instant::now();
        }
        Timer::after_millis(1).await;
    }
}

async fn brake(hal: &mut Hal<'_>) {
    hal.motors.set(0.try_into().unwrap(), 0.try_into().unwrap());
    println!("--- braking ---");
    Timer::after(BRAKE_TIME).await;
}
