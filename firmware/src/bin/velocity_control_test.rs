#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    main,
    time::{Duration, Instant},
};
use esp_println::println;
use lf_hal::{Hal, button::ButtonEvent};
use lf_hal_types::motors::PwmT;
use line_follower::velocity_controller::{Gains, VelocityController};

const KP: f32 = 0.3;
const TI: f32 = 0.3;
const TD: f32 = 0.005;
const STICTION_PWM: PwmT = PwmT::new_static::<400>();

const PERIOD: Duration = Duration::from_millis(10);
const LONG_PRESS: Duration = Duration::from_millis(500);

const SETPOINTS: &[i16] = &[
    0, 300, 1000, 2000, 4000, 8000, 4000, 2000, 1000, 0, -1000, -2000, -4000, -8000, -2000, -1000,
];

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let p = esp_hal::init(esp_hal::Config::default());

    let mut hal = Hal::new(p);
    let delay = Delay::new();

    let enc = hal.motors.encoders();
    let gains = Gains::from_standard(KP, TI, TD)
        .unwrap()
        .with_stiction(STICTION_PWM);
    println!("gains: {:?}", &gains);

    let mut controller = VelocityController::new(gains, enc);

    let mut sp_index: usize = 0;
    let mut current_setpoint = SETPOINTS[sp_index];

    println!("# deck short: cycle setpoints  deck long: stop  boot short: reverse");

    let mut last_t = Instant::now();

    let mut i = 0;
    loop {
        delay.delay(PERIOD);

        match hal.deck_button.poll_with_threshold(LONG_PRESS) {
            Some(ButtonEvent::LongPress) => {
                sp_index = 0;
                current_setpoint = 0;
                controller.reset(hal.motors.encoders());
                println!("reset");
            }
            Some(ButtonEvent::Release(_)) => {
                sp_index = (sp_index + 1) % SETPOINTS.len();
                current_setpoint = SETPOINTS[sp_index];
            }
            _ => {}
        }

        let enc = hal.motors.encoders();
        let now = Instant::now();
        let dt_us = (now - last_t).as_micros() as u32;
        last_t = now;
        let (l, r) = controller.update(enc, (current_setpoint, current_setpoint), dt_us);
        hal.motors.set(l.motor_pwm, r.motor_pwm);

        if i == 10 {
            println!(
                "setpoints: {}, velocity: ({},{}), pwm: ({},{}), dt: {} ms",
                current_setpoint,
                l.velocity,
                r.velocity,
                l.motor_pwm.get(),
                r.motor_pwm.get(),
                dt_us / 1000
            );
            i = 0;
        } else {
            i += 1;
        }
    }
}
