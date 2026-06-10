#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

extern crate alloc;

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, Ticker, WithTimeout as _};
use esp_backtrace as _;
use itertools::Itertools as _;
use lf_hal::{Hal, button::ButtonEvent, line_sensor::LedIndex};
use lf_hal_types::{
    RangeMeasurement,
    motors::{MAX_SPEED, METERS_PER_TICK, PwmT},
};
use line_follower::{
    line_detection::{PositionT, detect_line},
    velocity_controller::{Gain, Gains, VelocityController},
};

esp_bootloader_esp_idf::esp_app_desc!();

const BASE_SPEED: i32 = 3000;
const STEERING_KP: Gain = Gain::lit("1");
const STEERING_KD: Gain = Gain::lit("0.25");
const KP: f32 = 0.4;
const TI: f32 = 0.3;
const TD: f32 = 0.005;
const STICTION_PWM: PwmT = PwmT::new_static::<400>();
const OBSTACLE_STOP_M: f32 = 0.20;
const OBSTACLE_CLEAR_M: f32 = 0.40;
const OBSTACLE_CLEAR_DURATION: Duration = Duration::from_secs(3);
const LOST_LINE_DIST_M: f32 = 0.30;
const OBSTACLE_POLL: Duration = Duration::from_millis(100);
const LINE_CENTERED_THRESHOLD: PositionT = PositionT::lit("0.5");
const SCAN_INTERVAL: Duration = Duration::from_millis(120);

type Position32T = fixed::types::I22F10;

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    let mut hal = line_follower::init!(spawner);
    loop {
        wait_for_start(&mut hal).await;
        follow_line(&mut hal).await;
    }
}

async fn wait_for_start(hal: &mut Hal<'_>) {
    const LED_ANIMATION: [usize; 10] = [0, 1, 2, 3, 4, 5, 4, 3, 2, 1];
    log::info!("Waiting: center line on sensor, then press deck button");
    let mut ticker = Ticker::every(SCAN_INTERVAL);
    let mut animation_index: usize = 0;
    let mut line_ready = false;
    hal.line_sensor.disable_leds();

    loop {
        match select(hal.deck_button.next_event(None), ticker.next()).await {
            Either::First(ButtonEvent::Press) if line_ready => {
                hal.line_sensor.disable_leds();
                hal.deck_button.released().await;
                log::info!("Starting");
                return;
            }
            Either::Second(_) => {
                let detections = detect_line(&hal.line_sensor.read().await);
                let was_ready = line_ready;
                line_ready = detections
                    .iter()
                    .exactly_one()
                    .is_ok_and(|d| d.position.abs() <= LINE_CENTERED_THRESHOLD);
                if line_ready != was_ready {
                    if line_ready {
                        log::info!("Line centered, press deck button to start");
                    } else {
                        log::info!("Line lost, reposition");
                    }
                }
                if !line_ready {
                    let led_index = LedIndex::new(LED_ANIMATION[animation_index]).unwrap();
                    animation_index = (animation_index + 1) % LED_ANIMATION.len();
                    hal.line_sensor.enable_led(led_index);
                }
            }
            _ => {}
        }
    }
}

async fn follow_line(hal: &mut Hal<'_>) {
    let gains = Gains::from_standard(KP, TI, TD)
        .unwrap()
        .with_stiction(STICTION_PWM);
    let mut controller = VelocityController::new(gains, hal.motors.encoders());
    let mut last_position: Option<PositionT> = None;
    let mut prev_steering_pos: Option<PositionT> = None;
    let mut lost_at_enc: Option<(i16, i16)> = None;
    let mut last_t = Instant::now();
    let mut range_filter = RangeFilter::new(hal.read_range());

    loop {
        range_filter.add(hal.read_range());
        let dist = range_filter.filtered().distance_long();

        if dist < OBSTACLE_STOP_M {
            hal.motors.stop();
            log::info!("Obstacle at {:.2} m. Waiting for clear.", dist);
            if !wait_for_obstacle_clear(hal, &mut range_filter).await {
                return;
            }
            let readings = hal.line_sensor.read().await;
            let detections = detect_line(&readings);
            if detections.is_empty() {
                hal.motors.stop();
                log::info!("Obstacle cleared but no line detected. Stopping.");
                return;
            }
            log::info!("Obstacle cleared. Resuming.");
            controller.reset(hal.motors.encoders());
            prev_steering_pos = None;
            last_t = Instant::now();
        }

        // Read line sensor concurrently with button monitoring. `pressed()` resolves only on a
        // press, so a release event won't cancel an in-flight line read.
        let readings = match select(hal.deck_button.pressed(), hal.line_sensor.read()).await {
            Either::First(()) => {
                hal.motors.stop();
                log::info!("Stopped by button.");
                return;
            }
            Either::Second(r) => r,
        };

        let detections = detect_line(&readings);
        // When multiple lines are detected, follow the leftmost (most negative position).
        // Chosen for consistency for now; revisit if we need to handle forks or intersections.
        let detection = detections.iter().min_by_key(|d| d.position).copied();

        let enc = hal.motors.encoders();
        let now = Instant::now();
        let dt = now - last_t;
        last_t = now;

        let steering_line_position = if let Some(detection) = detection {
            let pos = detection.position;
            last_position = Some(pos);
            lost_at_enc = None;
            pos
        } else if let Some(last_position) = last_position {
            if last_position.abs() > LINE_CENTERED_THRESHOLD {
                hal.motors.stop();
                log::info!("Line lost in outer half (pos {}), stopping.", last_position);
                return;
            }
            if lost_at_enc.is_none() {
                log::info!(
                    "Line lost in inner half (pos {}), continuing on arc",
                    last_position
                );
            }
            // TODO: rules allow the line to resume anywhere within a 30 degree cone from the
            // interruption point. For now we just continue in the same arc rely on the search distance.
            let start = *lost_at_enc.get_or_insert(enc);
            let dl = enc.0.wrapping_sub(start.0) as i32;
            let dr = enc.1.wrapping_sub(start.1) as i32;
            let dist = (dl + dr) / 2;
            if dist.abs() > (LOST_LINE_DIST_M / METERS_PER_TICK) as i32 {
                hal.motors.stop();
                log::info!(
                    "No line after {:.2} m, stopping.",
                    dist as f32 / METERS_PER_TICK
                );
                return;
            }
            last_position
        } else {
            hal.motors.stop();
            log::info!("No line on start, stopping.");
            return;
        };

        let speeds = calculate_steering(BASE_SPEED, steering_line_position, prev_steering_pos);
        prev_steering_pos = Some(steering_line_position);

        let (l, r) = controller.update(enc, speeds, dt);
        hal.motors.set(l.motor_pwm, r.motor_pwm);
    }
}

fn calculate_steering(
    speed: i32,
    line_position: PositionT,
    prev_position: Option<PositionT>,
) -> (i16, i16) {
    // Widen the i16-backed position into i32, scale is unchanged.
    let pos = Position32T::from_num(line_position);
    let d_pos = prev_position.map_or(Position32T::ZERO, |prev| pos - Position32T::from_num(prev));

    // steering = speed * (KP * pos + KD * d_pos)
    let factor = Gain::ZERO
        .add_prod(STEERING_KP, pos)
        .add_prod(STEERING_KD, d_pos);
    let steering = (factor * speed).to_num::<i32>();
    log::debug!("steering: factor={factor} total={steering}");

    const MAX: i32 = MAX_SPEED as i32;
    let left = (speed + steering).clamp(-MAX, MAX) as i16;
    let right = (speed - steering).clamp(-MAX, MAX) as i16;

    log::debug!("left, right = ({left}, {right})");

    (left, right)
}

// Returns true when the path is clear, false if stopped by button press.
async fn wait_for_obstacle_clear(hal: &mut Hal<'_>, range_filter: &mut RangeFilter) -> bool {
    let mut clear_since: Option<Instant> = None;
    loop {
        range_filter.add(hal.read_range());
        let dist = range_filter.filtered().distance_long();
        if dist > OBSTACLE_CLEAR_M {
            let now = Instant::now();
            let since = *clear_since.get_or_insert(now);
            if now - since >= OBSTACLE_CLEAR_DURATION {
                return true;
            }
        } else {
            if clear_since.take().is_some() {
                log::info!("Obstacle returned at {:.2} m, resetting timer.", dist);
            }
        }

        match hal.deck_button.pressed().with_timeout(OBSTACLE_POLL).await {
            Ok(_) => {
                log::info!("Stopped by button during obstacle wait.");
                return false;
            }
            Err(_) => {}
        }
    }
}

struct RangeFilter {
    measurements: [u16; 7],
    i: usize,
}

impl RangeFilter {
    fn new(first: RangeMeasurement) -> RangeFilter {
        RangeFilter {
            measurements: [first.raw; 7],
            i: 0,
        }
    }

    fn add(&mut self, measurement: RangeMeasurement) {
        self.measurements[self.i] = measurement.raw;
        self.i = (self.i + 1) % self.measurements.len();
    }

    fn filtered(&self) -> RangeMeasurement {
        let mut copied = self.measurements;
        copied.sort();
        RangeMeasurement {
            raw: copied[copied.len() / 2],
        }
    }
}
