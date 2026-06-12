#![no_std]
#![no_main]

extern crate alloc;

use core::fmt::Write as _;

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, Timer, WithTimeout as _};
use esp_backtrace as _;
use heapless::String;
use lf_hal::{
    Hal,
    button::{Button, ButtonEvent},
    line_sensor::LedIndex,
    motors::PwmT,
};
use lf_hal_types::{BatteryMeasurement, RangeMeasurement};
use line_follower::line_detection::detect_line;

esp_bootloader_esp_idf::esp_app_desc!();

macro_rules! log_ok {
    () => {{
        log::info!("\x1b[32m[OK]\x1b[0m");
    }};
    ($fmt:literal $(, $arg:expr)*) => {{
        log::info!(concat!("\x1b[32m[", $fmt, " OK]\x1b[0m") $(, $arg)*);
    }};
}

macro_rules! log_fail {
    () => {{
        log::info!("\x1b[31m[FAIL]\x1b[0m");
    }};
    ($fmt:literal $(, $arg:expr)*) => {{
        log::info!(concat!("\x1b[31m[", $fmt, " FAIL]\x1b[0m") $(, $arg)*);
    }};
}

macro_rules! log_skip {
    () => {{
        log::info!("\x1b[33m[SKIP]\x1b[0m");
    }};
    ($fmt:literal $(, $arg:expr)*) => {{
        log::info!(concat!("\x1b[33m[", $fmt, " SKIP]\x1b[0m") $(, $arg)*);
    }};
}

macro_rules! test_section {
    ($title:literal) => {
        log::info!(concat!("--- ", $title, " ---"));
    };
    ($title:literal, $instr:literal) => {
        log::info!(concat!("--- ", $title, " ---"));
        log::info!($instr);
    };
}

const LONG_PRESS: Duration = Duration::from_millis(800);
const BLINK_LED_ON: Duration = Duration::from_millis(200);
const BLINK_LED_OFF: Duration = Duration::from_millis(200);
const MOTOR_LOW_PERCENT: i32 = 75;
const MOTOR_HIGH_PERCENT: i32 = 100;
const MOTOR_LOW_TIME: Duration = Duration::from_secs(2);
const MOTOR_HIGH_TIME: Duration = Duration::from_secs(1);
const VELOCITY_THRESHOLD: i32 = 1; // Ticks per millisecond
const ADC_SAMPLES: u32 = 16;
const LED_SENSOR_THRESHOLD: i16 = 200;

async fn blink_n(hal: &mut Hal<'_>, count: usize, on: Duration, off: Duration) {
    for _ in 0..count {
        hal.set_led(true);
        Timer::after(on).await;
        hal.set_led(false);
        Timer::after(off).await;
    }
}

/// Wait for a deck button release. Returns `None` (triggering `?` early-return) on long press.
async fn wait_button(btn: &mut Button<'_>) -> Option<()> {
    loop {
        match btn.next_event(Some(LONG_PRESS)).await {
            ButtonEvent::Release => return Some(()),
            ButtonEvent::LongPress => {
                return None;
            }
            _ => {}
        }
    }
}

async fn wait_for_continue(hal: &mut Hal<'_>) -> Option<()> {
    log::info!("Press deck button to continue");
    wait_button(&mut hal.deck_button).await
}

/// Automatic line sensor LED check: energises each LED and verifies the adjacent
/// phototransistor(s) respond. Runs without any button interaction.
async fn test_line_leds(hal: &mut Hal<'_>) {
    test_section!("Line sensor LEDs");
    hal.line_sensor.disable_leds();
    let baseline = hal.line_sensor.read_raw();

    for i in 0..=5usize {
        hal.line_sensor.enable_led(LedIndex::new(i).unwrap());
        Timer::after(Duration::from_millis(2)).await;

        let lit = hal.line_sensor.read_raw();

        // Neighbors of LED i: phototransistor i-1 on the left, i on the right (where they exist).
        let left = (i > 0).then(|| i - 1);
        let right = (i < lit.len()).then_some(i);
        let neighbors = [left, right].into_iter().flatten();
        let ok = neighbors
            .clone()
            .any(|p| lit[p] > baseline[p] + LED_SENSOR_THRESHOLD);

        if ok {
            log_ok!("Line LED {}/6", i + 1);
        } else {
            for p in neighbors {
                log::info!("  pt{} base={} lit={}", p, baseline[p], lit[p]);
            }
            log_fail!("Line LED {}/6", i + 1);
        }
    }
    hal.line_sensor.disable_leds();
}

async fn test_battery(hal: &mut Hal<'_>) -> Option<()> {
    test_section!("Battery voltage");
    let mut sum: u32 = 0;
    for _ in 0..ADC_SAMPLES {
        sum += hal.battery.read().raw as u32;
    }
    let avg_raw = sum / ADC_SAMPLES;
    let voltage = BatteryMeasurement {
        raw: avg_raw as u16,
    }
    .voltage();
    log::info!("Battery: {voltage:.3} V  (raw avg: {avg_raw})");
    log::info!("Verify against measured value.");
    Some(())
}

async fn test_range(hal: &mut Hal<'_>) -> Option<()> {
    test_section!("Distance sensor", "Place obstacle at known distance.");
    wait_for_continue(hal).await?;
    let mut sum: u32 = 0;
    for _ in 0..ADC_SAMPLES {
        sum += hal.range.read().raw as u32;
    }
    let avg_raw = sum / ADC_SAMPLES;
    let distance = RangeMeasurement {
        raw: avg_raw as u16,
    }
    .distance_long();
    log::info!("Distance: {distance:.3} m  (raw avg: {avg_raw})");
    log::info!("Verify against measured value.");
    wait_for_continue(hal).await
}

async fn test_line_detection(hal: &mut Hal<'_>) -> Option<()> {
    test_section!(
        "Line detection",
        "Optionally place a dark line under the sensors."
    );
    wait_for_continue(hal).await?;
    let readings = hal.line_sensor.read().await;

    let mut s: String<128> = String::new();
    let _ = write!(s, "Sensors [0-9]:");
    for v in readings.values {
        let _ = write!(s, " {v:5}");
    }
    log::info!("{s}");

    let detections = detect_line(&readings);
    if detections.is_empty() {
        log::info!("No line detected.");
    } else {
        let mut s: String<128> = String::new();
        let _ = write!(s, "Detected:");
        for d in &detections {
            let _ = write!(s, " pos={:+} str={}", d.position, d.strength);
        }
        log::info!("{s}");
    }

    Some(())
}

enum PhaseOutcome {
    Completed { delta: i32 },
    Skipped,
}

/// Builds a signed PWM value from a power percentage and a direction (`+1`/`-1`).
fn motor_pwm(percent: i32, direction: i16) -> PwmT {
    PwmT::new(direction * (PwmT::MAX.get() as i32 * percent / 100) as i16).unwrap()
}

async fn test_motors(hal: &mut Hal<'_>) -> Option<()> {
    test_section!("Motors + encoders");
    log::warn!("RAISE ROBOT off the ground");
    wait_for_continue(hal).await?;

    let mut skip = false;

    const PHASES: [(&str, bool, i16); 4] = [
        ("Left forward", true, 1),
        ("Left backward", true, -1),
        ("Right forward", false, 1),
        ("Right backward", false, -1),
    ];

    for (label, left, direction) in PHASES {
        if skip {
            log_skip!("{} {}%", label, MOTOR_LOW_PERCENT);
            log_skip!("{} {}%", label, MOTOR_HIGH_PERCENT);
            continue;
        }

        let outcome_low = run_motor_phase(
            hal,
            label,
            MOTOR_LOW_PERCENT,
            left,
            motor_pwm(MOTOR_LOW_PERCENT, direction),
            MOTOR_LOW_TIME,
        )
        .await?;

        let delta_low = match outcome_low {
            PhaseOutcome::Completed { delta } => {
                if delta * direction as i32 > VELOCITY_THRESHOLD {
                    log_ok!("{} {}%", label, MOTOR_LOW_PERCENT);
                } else {
                    log_fail!("{} {}%", label, MOTOR_LOW_PERCENT);
                }
                delta
            }
            PhaseOutcome::Skipped => {
                skip = true;
                log_skip!("{} {}%", label, MOTOR_HIGH_PERCENT); // high phase gets skipped too
                continue;
            }
        };

        let outcome_high = run_motor_phase(
            hal,
            label,
            MOTOR_HIGH_PERCENT,
            left,
            motor_pwm(MOTOR_HIGH_PERCENT, direction),
            MOTOR_HIGH_TIME,
        )
        .await?;

        log::debug!("Stopping motor");
        hal.motors.stop();

        match outcome_high {
            PhaseOutcome::Completed { delta } => {
                let velocity_low =
                    (delta_low * direction as i32) / MOTOR_LOW_TIME.as_millis() as i32;
                let velocity_high = (delta * direction as i32) / MOTOR_HIGH_TIME.as_millis() as i32;
                if velocity_high > velocity_low {
                    log_ok!("{} {}%", label, MOTOR_HIGH_PERCENT);
                } else {
                    log_fail!("{} {}%", label, MOTOR_HIGH_PERCENT);
                }
            }
            PhaseOutcome::Skipped => {
                skip = true;
                continue;
            }
        }
    }

    Some(())
}

async fn run_motor_phase(
    hal: &mut Hal<'_>,
    label: &str,
    percent: i32,
    left: bool,
    pwm: PwmT,
    duration: Duration,
) -> Option<PhaseOutcome> {
    let right_pwm = if left { PwmT::new_static::<0>() } else { pwm };
    let left_pwm = if left { pwm } else { PwmT::new_static::<0>() };
    log::debug!("Setting motor PWM to {pwm}");
    hal.motors.set(left_pwm, right_pwm);

    // The hardware encoder counters are only 16 bit and can overflow several
    // times over a full motor phase. Sample them in short intervals so each
    // i16 subtraction stays within range, then accumulate into a wide i32.
    let (delta_l, delta_r) = {
        const SAMPLE_INTERVAL: Duration = Duration::from_millis(100);
        let mut prev_encoders = hal.motors.encoders();
        let mut delta_l: i32 = 0;
        let mut delta_r: i32 = 0;
        let mut next_sample = Instant::now();
        let deadline = next_sample + duration;

        while next_sample < deadline {
            next_sample = deadline.min(next_sample + SAMPLE_INTERVAL);
            match wait_button(&mut hal.deck_button)
                .with_deadline(next_sample)
                .await
            {
                Ok(None) => {
                    hal.motors.stop();
                    return None;
                }
                Ok(Some(())) => {
                    hal.motors.stop();
                    log_skip!("{} {}%", label, percent);
                    return Some(PhaseOutcome::Skipped);
                }
                _ => (), // Timed out, sample encoders and continue
            }

            let current_encoders = hal.motors.encoders();
            delta_l += i32::from(current_encoders.0.wrapping_sub(prev_encoders.0));
            delta_r += i32::from(current_encoders.1.wrapping_sub(prev_encoders.1));
            prev_encoders = current_encoders;
        }

        (delta_l, delta_r)
    };

    log::info!("enc_L={delta_l:+} enc_R={delta_r:+}");

    let delta = if left { delta_l } else { delta_r };
    Some(PhaseOutcome::Completed { delta })
}

async fn test_indicator_led(hal: &mut Hal<'_>) -> Option<()> {
    test_section!("Indicator LED", "Watch for 5 blinks.");
    wait_for_continue(hal).await?;
    blink_n(hal, 5, BLINK_LED_ON, BLINK_LED_OFF).await;
    Some(())
}

async fn test_boot_button(hal: &mut Hal<'_>) -> Option<()> {
    test_section!("Boot button", "Press BOOT to test, or deck to skip.");
    match select(
        hal.boot_button.released(),
        wait_button(&mut hal.deck_button),
    )
    .await
    {
        Either::First(_) => log_ok!(),
        Either::Second(Some(_)) => log_skip!(),
        Either::Second(None) => return None,
    }
    Some(())
}

async fn run_tests(hal: &mut Hal<'_>) -> Option<()> {
    log::info!("=== HAL Test ===");
    log::info!("Press deck button to start, long press to restart.");
    wait_button(&mut hal.deck_button).await?;

    test_line_leds(hal).await;
    test_line_detection(hal).await?;

    test_motors(hal).await?;

    test_indicator_led(hal).await?;

    test_battery(hal).await?;
    test_range(hal).await?;
    test_boot_button(hal).await?;

    log::info!("=== All tests complete ===");
    Some(())
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let mut hal = line_follower::init!(spawner);

    loop {
        if run_tests(&mut hal).await.is_some() {
            log::info!("=== DONE -- long press deck button to restart ===");

            while wait_button(&mut hal.deck_button).await.is_some() {
                log::info!("long press deck button to restart.");
            }
        }
        log::info!("=== RESTARTING ===");
    }
}
