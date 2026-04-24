#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    main,
    time::{Duration, Instant},
};
use esp_println::println;
use line_follower::hal::{Hal, button::ButtonEvent, motors::PwmT};

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

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let p = esp_hal::init(esp_hal::Config::default());

    let mut hal = Hal::new(p);
    let delay = Delay::new();

    println!("Motor test — short press: next phase  long press: stop");

    let mut phase = Phase::Stopped;

    loop {
        let action = match phase {
            Phase::Running(i) => {
                let (label, left, right) = PHASES[i];
                run_phase(&mut hal, left, right, label)
            }
            Phase::Stopped => run_phase(&mut hal, 0, 0, "STOPPED"),
        };

        brake(&mut hal, &delay);

        phase = match action {
            PhaseAction::Advance => match phase {
                Phase::Running(i) => Phase::Running((i + 1) % PHASES.len()),
                Phase::Stopped => Phase::Running(0),
            },
            PhaseAction::Stop => Phase::Stopped,
        };
    }
}

fn run_phase(hal: &mut Hal<'_>, left: i16, right: i16, label: &str) -> PhaseAction {
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
    }
}

fn brake(hal: &mut Hal<'_>, delay: &Delay) {
    hal.motors.set(0.try_into().unwrap(), 0.try_into().unwrap());
    println!("--- braking ---");
    delay.delay(BRAKE_TIME);
}
