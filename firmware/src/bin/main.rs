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
use embassy_time::{Duration, Instant, Ticker};
use esp_backtrace as _;
use fixed::types::I9F23;
use itertools::Itertools as _;
use lf_hal::{
    Hal, RangeSensor,
    button::ButtonEvent,
    line_sensor::{LedIndex, LineSensor},
    motors::Motors,
};
use lf_hal_types::{
    RangeMeasurement,
    motors::{MAX_SPEED, METERS_PER_TICK, PwmT},
};
use line_follower::{
    line_detection::{LineDetection, PositionT, detect_line},
    velocity_controller::{Gain, Gains, VelocityController},
};

esp_bootloader_esp_idf::esp_app_desc!();

const BASE_SPEED: i32 = 1000;
const STEERING_KP: Gain = Gain::lit("0.75");
const STEERING_LEAD_TICKS: i32 = 256; // Derivative look-ahead distance along the path (~75 mm)
const STEERING_DERIV_LENGTH_TICKS: i32 = 64; // Distance over which the steering derivative is averaged (~15 mm)
const KP: f32 = 0.4;
const TI: f32 = 0.3;
const TD: f32 = 0.005;
const STICTION_PWM: PwmT = PwmT::new_static::<400>();
const OBSTACLE_STOP_M: f32 = 0.15;
const OBSTACLE_CLEAR_M: f32 = 0.30;
const OBSTACLE_CLEAR_DURATION: Duration = Duration::from_secs(3);
const LOST_LINE_DIST_M: f32 = 0.30;
const MAIN_LOOP_PERIOD: Duration = Duration::from_millis(10);
const OBSTACLE_POLL: Duration = Duration::from_millis(100);
const LINE_CENTERED_THRESHOLD: PositionT = PositionT::lit("0.5");
const SCAN_INTERVAL: Duration = Duration::from_millis(120);

type Position32T = fixed::types::I6F26;

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    let mut hal = line_follower::init!(spawner);
    loop {
        wait_for_start(&mut hal).await;

        if let Either::Second(()) = select(
            follow_line(&mut hal.motors, &mut hal.line_sensor, &hal.range),
            hal.deck_button.pressed(),
        )
        .await
        {
            hal.motors.stop();
            log::info!("Stopped by button.");
        }
    }
}

async fn wait_for_start(hal: &mut Hal<'_>) {
    const LED_ANIMATION: [usize; 10] = [0, 1, 2, 3, 4, 5, 4, 3, 2, 1];
    const STARTING_LINE_DETECTION_COUNT_THRESHOLD: usize = 3;
    log::info!("Waiting: center line on sensor, then press deck button");
    let mut ticker = Ticker::every(SCAN_INTERVAL);
    hal.line_sensor.disable_leds();

    loop {
        let mut animation_index: usize = 0;
        let mut line_detected_count = 0;
        loop {
            if let Some(d) = read_line_for_start(hal).await {
                line_detected_count += 1;

                if line_detected_count >= STARTING_LINE_DETECTION_COUNT_THRESHOLD {
                    log::info!(
                        "Line centered (pos {}, strength {}), press deck button to start",
                        d.position,
                        d.strength
                    );
                    break;
                }
            } else {
                line_detected_count = 0;
            }

            let led_index = LedIndex::new(LED_ANIMATION[animation_index]).unwrap();
            animation_index = (animation_index + 1) % LED_ANIMATION.len();
            hal.line_sensor.enable_led(led_index);

            ticker.next().await;
        }

        loop {
            match select(hal.deck_button.next_event(None), ticker.next()).await {
                Either::First(ButtonEvent::Press) => {
                    hal.line_sensor.disable_leds();
                    hal.deck_button.released().await;
                    log::info!("Starting");
                    return;
                }
                Either::Second(_) => {
                    if read_line_for_start(hal).await.is_none() {
                        log::info!("Line lost, reposition");
                        break;
                    }
                }
                _ => {}
            }
        }
    }
}

async fn follow_line(
    motors: &mut Motors<'_>,
    line_sensor: &mut LineSensor<'_>,
    range: &RangeSensor,
) {
    let gains = Gains::from_standard(KP, TI, TD)
        .unwrap()
        .with_stiction(STICTION_PWM);

    // Initial values
    let initial_readings = line_sensor.read().await;
    let Some(initial_pos) = detect_line(&initial_readings)
        .iter()
        .next()
        .map(|d| d.position)
    else {
        log::error!("follow_line without line detected.");
        return;
    };
    let initial_enc = motors.encoders();
    let now = Instant::now();

    // Controllers
    let mut velocity = VelocityController::new(gains, initial_enc);
    let mut steering = SteeringController::new(motors.encoders(), initial_pos);

    // Mutable loop state
    let mut line_lost_at: Option<(i16, i16)> = None;
    let mut last_good_line_position = initial_pos;
    let mut last_t = now;
    let mut range_filter = RangeFilter::new(range.read());

    let mut ticker = Ticker::every(MAIN_LOOP_PERIOD);

    loop {
        range_filter.add(range.read());
        let dist = range_filter.filtered().distance_long();

        if dist < OBSTACLE_STOP_M {
            motors.stop();
            log::info!("Obstacle at {dist:.2} m. Waiting for clear.");
            wait_for_obstacle_clear(range, &mut range_filter).await;
            let readings = line_sensor.read().await;
            let detections = detect_line(&readings);
            if detections.is_empty() {
                log::info!("Obstacle cleared but no line detected. Stopping.");
                return;
            } else {
                log::info!("Obstacle cleared. Resuming.");
                let resume_enc = motors.encoders();
                last_t = Instant::now();
                velocity.reset(resume_enc);
                ticker.reset();
            }
        }

        let readings = line_sensor.read().await;

        let detections = detect_line(&readings);
        // When multiple lines are detected, follow the leftmost (most negative position).
        // Chosen for consistency for now; revisit if we need to handle forks or intersections.
        let detection = detections.iter().min_by_key(|d| d.position).copied();

        let enc = motors.encoders();
        let now = Instant::now();
        let dt = now - last_t;
        last_t = now;

        let steering_line_position = if let Some(detection) = detection {
            let pos = detection.position;
            last_good_line_position = pos;
            line_lost_at = None;
            pos
        } else {
            if let Some(line_lost_at) = line_lost_at {
                // We were already driving with lost line, verify that we did not drive away too far.

                let dl = enc.0.wrapping_sub(line_lost_at.0) as i32;
                let dr = enc.1.wrapping_sub(line_lost_at.1) as i32;
                let dist = (dl + dr) / 2;
                if dist.abs() > (LOST_LINE_DIST_M / METERS_PER_TICK) as i32 {
                    motors.stop();
                    log::info!(
                        "No line after {:.2} m, stopping.",
                        dist as f32 * METERS_PER_TICK
                    );
                    return;
                }
            } else {
                // Line lost for the first time

                if last_good_line_position.abs() > LINE_CENTERED_THRESHOLD {
                    motors.stop();
                    log::info!(
                        "Line lost in outer half (pos {}), stopping.",
                        last_good_line_position
                    );
                    return;
                } else {
                    line_lost_at = Some(enc);
                    log::info!(
                        "Line lost in inner half (pos {}), continuing on arc",
                        last_good_line_position
                    );
                }
            }

            // TODO: rules allow the line to resume anywhere within a 30 degree cone from the
            // interruption point. For now we just continue in the same arc rely on the search distance.
            last_good_line_position
        };

        let speeds = steering.update(BASE_SPEED, steering_line_position, enc);
        let (l, r) = velocity.update(enc, speeds, dt);
        motors.set(l.motor_pwm, r.motor_pwm);

        ticker.next().await;
    }
}

/// Proportional steering with a distance-based derivative look-ahead.
///
/// Holds a first-order spatial low-pass of the line position whose lag estimates the path
/// derivative `d(position)/d(distance)` without dividing by the per-loop distance.
struct SteeringController {
    /// Distance-based EWMA of the line position.
    /// This approximates the line position as it was `STEERING_DERIV_LENGTH_TICKS` ago.
    filtered_pos: Position32T,
    /// Encoder reading at the previous update, for differentiating distance travelled.
    prev_enc: (i16, i16),
}

impl SteeringController {
    fn new(enc: (i16, i16), line_position: PositionT) -> Self {
        Self {
            filtered_pos: Position32T::from_num(line_position),
            prev_enc: enc,
        }
    }

    /// Returns the `(left, right)` motor speeds for a given base `speed`, the current line
    /// `line_position`, and the current encoder reading `enc` (the distance travelled since the
    /// last update is differentiated internally).
    fn update(&mut self, speed: i32, line_position: PositionT, enc: (i16, i16)) -> (i16, i16) {
        // Distance travelled since the last update, averaged across both wheels.
        let ds = (enc.0.wrapping_sub(self.prev_enc.0) as i32
            + enc.1.wrapping_sub(self.prev_enc.1) as i32)
            / 2;
        debug_assert!(
            ds < 150,
            "Guaranteed by maximum wheel speed and >= 100Hz update rate"
        );
        debug_assert!(
            ds >= 0,
            "This steering controller only works when driving forward"
        );
        self.prev_enc = enc;

        // Widen the i16-backed position into i32, scale is unchanged.
        let line_position = Position32T::from_num(line_position);

        // Smoothing weight for the EWMA distance filter depends on distance driven
        // The formula is obtained by forcing the result of this filter to estimate exactly
        // the derivative in case the input position changes linearly.
        let alpha = I9F23::from_num(ds) / (STEERING_DERIV_LENGTH_TICKS + ds);

        // Updating the smoothing EWMA
        self.filtered_pos += (line_position - self.filtered_pos).mul_add(alpha, Position32T::ZERO);

        // Effectively we calculate
        // `derivative = (line_position - self.filtered_pos) / STEERING_DERIV_LENGTH_TICKS` (in line sensor units per encoder tick)
        // `lookahead = derivative * STEERING_LEAD_RATIO` (in line sensor units),
        // but without the rounding in the middle
        const {
            assert!(
                STEERING_LEAD_TICKS % STEERING_DERIV_LENGTH_TICKS == 0,
                "Steering lead ratio should be an exact integer for this trick"
            );
        };
        const STEERING_LEAD_RATIO: i32 = STEERING_LEAD_TICKS / STEERING_DERIV_LENGTH_TICKS;
        let lookahead = (line_position - self.filtered_pos) * STEERING_LEAD_RATIO;

        // Extrapolate a fixed look-ahead distance along the path.
        let extrapolated = line_position + lookahead;

        let factor = STEERING_KP.mul_add(extrapolated, Gain::ZERO);

        let steering = (factor * speed).to_num::<i32>();
        log::debug!("{steering} = {speed} * {STEERING_KP} * ({line_position} + {lookahead})");

        const MAX: i32 = MAX_SPEED as i32;
        let left = (speed + steering).clamp(-MAX, MAX) as i16;
        let right = (speed - steering).clamp(-MAX, MAX) as i16;

        (left, right)
    }
}

// Returns once the path has been clear for `OBSTACLE_CLEAR_DURATION`.
async fn wait_for_obstacle_clear(range: &RangeSensor, range_filter: &mut RangeFilter) {
    let mut unblock_at: Option<Instant> = None;
    let mut ticker = Ticker::every(OBSTACLE_POLL);
    loop {
        ticker.next().await;

        range_filter.add(range.read());
        let dist = range_filter.filtered().distance_long();
        if dist > OBSTACLE_CLEAR_M {
            let now = Instant::now();
            if let Some(unblock_at) = unblock_at {
                if now >= unblock_at {
                    return;
                }
            } else {
                unblock_at = Some(now + OBSTACLE_CLEAR_DURATION);
            }
        } else if unblock_at.take().is_some() {
            log::info!("Obstacle returned at {dist:.2} m, resetting timer.");
        }
    }
}

/// Reads the line sensor and verifies that there is only one line detected
/// centered on the sensor.
async fn read_line_for_start(hal: &mut Hal<'_>) -> Option<LineDetection> {
    let detections = detect_line(&hal.line_sensor.read().await);

    detections
        .iter()
        .exactly_one()
        .ok()
        .filter(|d| d.position.abs() <= LINE_CENTERED_THRESHOLD)
        .cloned()
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
        copied.sort_unstable();
        RangeMeasurement {
            raw: copied[copied.len() / 2],
        }
    }
}
