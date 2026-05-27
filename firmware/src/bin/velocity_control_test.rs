#![no_std]
#![no_main]

extern crate alloc;

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, Ticker};
use esp_backtrace as _;
use esp_println::println;
use lf_hal::button::ButtonEvent;
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

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    let mut hal = line_follower::init!();

    let enc = hal.motors.encoders();
    let gains = Gains::from_standard(KP, TI, TD)
        .unwrap()
        .with_stiction(STICTION_PWM);
    println!("gains: {:?}", &gains);

    let mut controller = VelocityController::new(gains, enc);

    let mut sp_index: usize = 0;
    let mut current_setpoint = SETPOINTS[sp_index];

    println!("# deck short: cycle setpoints  deck long: stop  boot short: reverse");

    let mut ticker = Ticker::every(PERIOD);
    let mut last_t = Instant::now();

    let mut i = 0;
    loop {
        loop {
            match select(hal.deck_button.next_event(Some(LONG_PRESS)), ticker.next()).await {
                Either::First(ButtonEvent::LongPress) => {
                    sp_index = 0;
                    current_setpoint = 0;
                    controller.reset(hal.motors.encoders());
                    println!("reset");
                }
                Either::First(ButtonEvent::Release) => {
                    sp_index = (sp_index + 1) % SETPOINTS.len();
                    current_setpoint = SETPOINTS[sp_index];
                }
                Either::First(_) => {}
                Either::Second(_) => break,
            }
        }

        let enc = hal.motors.encoders();
        let now = Instant::now();
        let dt = now - last_t;
        last_t = now;
        let (l, r) = controller.update(enc, (current_setpoint, current_setpoint), dt);
        hal.motors.set(l.motor_pwm, r.motor_pwm);

        if i == 10 {
            println!(
                "setpoints: {}, velocity: ({},{}), pwm: ({},{}), dt: {} ms",
                current_setpoint,
                l.velocity,
                r.velocity,
                l.motor_pwm.get(),
                r.motor_pwm.get(),
                dt.as_millis()
            );
            i = 0;
        } else {
            i += 1;
        }
    }
}
