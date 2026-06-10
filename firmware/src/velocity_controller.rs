use embassy_time::{Duration, TICK_HZ};
use fixed::types::{I20F12, I52F12};
use lf_hal_types::motors::{MAX_SPEED, PwmT};

/// Fixed-point control gain. The 12 fractional bits match the scale used
/// throughout the controller arithmetic. Gains have no inherent range, so this
/// is an unbounded fixed-point type; sane values are enforced at construction.
pub type Gain = I20F12;

#[derive(Clone, Copy, Debug)]
pub struct Gains {
    kp: Gain,
    ki: Gain,
    kd: Gain,
    stiction_pwm: i32,
}

impl Gains {
    /// Constructor, all three arguments must be between 0 and 8
    pub fn new(kp: f32, ki: f32, kd: f32) -> Option<Self> {
        if kp < 0.0 || kp > 8.0 {
            return None;
        }
        if ki < 0.0 || ki > 8.0 {
            return None;
        }
        if kd < 0.0 || kd > 8.0 {
            return None;
        }

        Some(Self {
            kp: Gain::from_num(kp),
            ki: Gain::from_num(ki),
            kd: Gain::from_num(kd),
            stiction_pwm: 0,
        })
    }

    /// Constructor from standard form PID controller parametrization
    /// <https://en.wikipedia.org/wiki/PID_controller#Standard_form>
    /// - `kp` must be between 0 and 8.
    /// - `ti` is the integral time constant in seconds
    ///   (how long the controller tolerates an error).
    ///   `kp / ti` must be between 0 and 8.
    /// - `td` is the derivative time constant in seconds
    ///   (how far forward in future does the controller look).
    ///   `kp * td` must be between 0 and 8.
    pub fn from_standard(kp: f32, ti: f32, td: f32) -> Option<Self> {
        Self::new(kp, kp / ti, kp * td)
    }

    /// Adds stiction PWM to the gains structure.
    /// This controls the minimum nonzero PWM output to be generated,
    /// helping to overcome stiction of the motors and gearboxes.
    pub fn with_stiction(mut self, pwm: PwmT) -> Self {
        self.stiction_pwm = pwm.get() as i32;
        self
    }
}

pub struct WheelUpdateResult {
    /// PWM command to send to the motor
    pub motor_pwm: PwmT,
    /// How many ticks we drove since last call to update
    pub ticks_since_last_update: i16,
    /// Measured immediate velocity (`tick_since_last_update / dt`)
    pub velocity: i16,
}

/// PID regulator for controlling PWM command for a single motor.
pub struct WheelController {
    /// The integral of the error, premultiplied by the kI gain (i.e. the
    /// integral term's contribution to the PWM output).
    integral: Gain,
    /// Last value of encoder input, used for speed calculation
    last_ticks: i16,

    last_velocity: i32,
}

impl WheelController {
    pub fn new(initial_ticks: i16) -> Self {
        Self {
            integral: Gain::ZERO,
            last_ticks: initial_ticks,
            last_velocity: 0,
        }
    }

    /// Updates the wheel velocity controller and returns the new PWM command
    /// - `dt` must be in the range (0ms, 120ms]
    /// - Velocity is limited by the measured motor maximum of +-15_000 ticks/s (= ~4.5m/s)
    /// - Acceleration is limited to 60_000 ticks/s^2 (= ~18m/s^2, a value that is more of an
    /// educated guess)
    pub fn update(
        &mut self,
        gains: &Gains,
        current_ticks: i16,
        setpoint: i16,
        dt: Duration,
    ) -> WheelUpdateResult {
        debug_assert!(dt > Duration::from_ticks(0) && dt <= Duration::from_millis(120));
        let dt_ticks = dt.as_ticks() as i32;

        let tick_delta = current_ticks.wrapping_sub(self.last_ticks) as i32;
        self.last_ticks = current_ticks;
        debug_assert!(tick_delta.abs() <= 1800);

        let velocity = tick_delta * TICK_HZ as i32 / dt_ticks;
        debug_assert!(
            velocity.abs() <= MAX_SPEED as i32,
            "Input range requirement"
        );

        let error = setpoint as i32 - velocity;
        debug_assert!(error.abs() < 48_000);

        let integral_update: Gain =
            (I52F12::from_num(gains.ki) * i64::from(error) * i64::from(dt_ticks) / TICK_HZ as i64)
                .to_num::<Gain>();
        self.integral += integral_update;

        let velocity_change = velocity - self.last_velocity;
        self.last_velocity = velocity;
        let acceleration = (velocity_change as i64 * TICK_HZ as i64 / dt_ticks as i64) as i32;
        debug_assert!(acceleration.abs() < 60_000, "Input range requirement");

        let stiction_ff = if setpoint > 0 {
            gains.stiction_pwm
        } else if setpoint < 0 {
            -gains.stiction_pwm
        } else if error > 0 {
            gains.stiction_pwm
        } else if error < 0 {
            -gains.stiction_pwm
        } else {
            0
        };

        let raw = (gains.kp * error).to_num::<i32>() + self.integral.to_num::<i32>()
            - (gains.kd * acceleration).to_num::<i32>()
            + stiction_ff;

        let clamped = raw.clamp(PwmT::MIN.get() as i32, PwmT::MAX.get() as i32);
        let clamping_error = raw - clamped;

        if clamping_error.signum() == integral_update.signum().to_num::<i32>() {
            // Undo the integral update if adding it to the raw output made it worse.
            // clamping_error == 0 && integral_update == 0: this is a no-op
            self.integral -= integral_update;
        }

        let motor_pwm = if clamped.abs() > gains.stiction_pwm {
            PwmT::new(clamped as i16).unwrap()
        } else {
            PwmT::new_static::<0>()
        };

        WheelUpdateResult {
            motor_pwm,
            ticks_since_last_update: tick_delta as i16,
            velocity: velocity as i16,
        }
    }

    /// Resets all internal state of the controller to zero, sets last ticks to given value.
    pub fn reset(&mut self, current_ticks: i16) {
        self.integral = Gain::ZERO;
        self.last_ticks = current_ticks;
        self.last_velocity = 0;
    }
}

/// Velocity controller for two wheels
pub struct VelocityController {
    gains: Gains,
    left: WheelController,
    right: WheelController,
}

impl VelocityController {
    pub fn new(gains: Gains, initial_ticks: (i16, i16)) -> Self {
        Self {
            gains,
            left: WheelController::new(initial_ticks.0),
            right: WheelController::new(initial_ticks.1),
        }
    }

    /// Updates the velocity controller and returns the new PWM commands and stats
    /// - `dt` must be in the range (0ms, 120ms].
    /// - Velocity is limited by the measured motor maximum of +-15_000 ticks/s (= ~4.5m/s)
    /// - Acceleration is limited to 60_000 ticks/s^2 (= ~18m/s^2, a value that is more of an
    /// educated guess).
    pub fn update(
        &mut self,
        encoder_ticks: (i16, i16),
        velocity_setpoints: (i16, i16),
        dt: Duration,
    ) -> (WheelUpdateResult, WheelUpdateResult) {
        (
            self.left
                .update(&self.gains, encoder_ticks.0, velocity_setpoints.0, dt),
            self.right
                .update(&self.gains, encoder_ticks.1, velocity_setpoints.1, dt),
        )
    }

    /// Resets all internal state of the controller to zero, sets last ticks to given values
    pub fn reset(&mut self, ticks: (i16, i16)) {
        self.left.reset(ticks.0);
        self.right.reset(ticks.1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embassy_time::TICK_HZ;
    use test_strategy::proptest;

    const DT_10MS: Duration = Duration::from_millis(10);

    #[test]
    fn test_wrapping_tick_delta() {
        // Normal forward
        assert_eq!(50i16.wrapping_sub(0) as i32, 50);

        // Forward wrap: traveled 11 ticks across i16::MAX
        let before = i16::MAX - 5;
        let after = i16::MIN + 5;
        assert_eq!(after.wrapping_sub(before) as i32, 11);

        // Backward motion
        assert_eq!((-5i16).wrapping_sub(5) as i32, -10);
    }

    #[test]
    fn test_zero_tracks_zero() {
        let gains = Gains::new(1.0, 1.0, 0.0).unwrap();
        let mut ctrl = WheelController::new(0);
        for _ in 0..100 {
            let out = ctrl.update(&gains, 0, 0, DT_10MS);
            assert_eq!(out.motor_pwm, PwmT::new(0).unwrap());
            assert_eq!(out.velocity, 0);
        }
    }

    #[proptest]
    fn pwm_output_clamps(
        #[strategy(10_000u64..=120_000)] dt_ticks: u64,
        #[strategy(-(MAX_SPEED as f32)..=MAX_SPEED as f32)] velocity: f32,
        #[strategy(-15000i16..=15000)] setpoint: i16,
        #[strategy(0.0f32..=0.5)] kp: f32,
    ) {
        let dt = Duration::from_ticks(dt_ticks);
        let dt_secs = dt_ticks as f32 / TICK_HZ as f32;
        // The controller starts with last_velocity=0, so on the first call acceleration =
        // velocity / dt_secs. Skip cases where that would exceed the 60_000 ticks/s^2 limit.
        proptest::prop_assume!(velocity.abs() < 60_000.0 * dt_secs);
        let ticks = (velocity * dt_secs) as i16;
        // Small nonzero ki so the integral term participates without dominating.
        let gains = Gains::new(kp, 0.025, 0.0).unwrap();
        let mut ctrl = WheelController::new(0);
        let out = ctrl.update(&gains, ticks, setpoint, dt);
        assert!(out.motor_pwm.get() >= -1024 && out.motor_pwm.get() <= 1024);
    }

    #[test]
    fn test_gains_new_range() {
        assert!(Gains::new(0.0, 0.0, 0.0).is_some());
        assert!(Gains::new(8.0, 8.0, 8.0).is_some());
        assert!(Gains::new(-0.1, 0.0, 0.0).is_none());
        assert!(Gains::new(0.0, -0.1, 0.0).is_none());
        assert!(Gains::new(0.0, 0.0, -0.1).is_none());
        assert!(Gains::new(8.001, 0.0, 0.0).is_none());
        assert!(Gains::new(0.0, 8.001, 0.0).is_none());
        assert!(Gains::new(0.0, 0.0, 8.001).is_none());
    }

    #[test]
    fn test_gains_from_standard_range() {
        // ki = kp/ti = 2/0.5 = 4.0, kd = kp*td = 2*0.1 = 0.2 — valid
        assert!(Gains::from_standard(2.0, 0.5, 0.1).is_some());
        // ki = 1/0.1 = 10.0 > 8 — invalid
        assert!(Gains::from_standard(1.0, 0.1, 0.0).is_none());
        // kd = 2*5 = 10.0 > 8 — invalid
        assert!(Gains::from_standard(2.0, 1.0, 5.0).is_none());
        // kp = 9 > 8 — invalid
        assert!(Gains::from_standard(9.0, 1.0, 0.0).is_none());
    }

    #[test]
    fn test_integral_bounded_under_saturation() {
        let gains = Gains::new(0.0, 8.0, 0.0).unwrap();
        let mut ctrl = WheelController::new(0);
        for _ in 0..10_000 {
            let out = ctrl.update(&gains, 0, 15000, DT_10MS);
            assert_eq!(out.motor_pwm, PwmT::new(1024).unwrap());
        }
    }

    #[test]
    fn test_opposes_velocity() {
        let gains = Gains::new(5.0, 0.0, 0.0).unwrap();
        let mut ctrl = WheelController::new(0);
        // 5 ticks in 10ms = 500 ticks/sec, setpoint 0 → output should be negative
        let out = ctrl.update(&gains, 5, 0, DT_10MS);
        assert!(
            out.motor_pwm.get() < 0,
            "expected negative output opposing positive velocity, got {}",
            out.motor_pwm.get()
        );
    }

    #[test]
    fn test_reset_clears_state() {
        let gains = Gains::new(1.0, 5.0, 0.0).unwrap();
        let mut ctrl = WheelController::new(0);
        for _ in 0..100 {
            ctrl.update(&gains, 0, 5000, DT_10MS);
        }
        ctrl.reset(0);
        let out = ctrl.update(&gains, 0, 0, DT_10MS);
        assert_eq!(out.motor_pwm, PwmT::new(0).unwrap());
        assert_eq!(out.velocity, 0);
    }

    #[test]
    fn test_time_invariant_p() {
        // Same velocity error at different dt → same P output
        let gains = Gains::new(0.05, 0.0, 0.0).unwrap();

        let mut ctrl_a = WheelController::new(0);
        let out_a = ctrl_a.update(&gains, 5, 0, Duration::from_millis(10)); // 5 ticks/10ms = 500/s

        let mut ctrl_b = WheelController::new(0);
        let out_b = ctrl_b.update(&gains, 10, 0, Duration::from_millis(20)); // 10 ticks/20ms = 500/s

        assert_eq!(
            out_a.motor_pwm, out_b.motor_pwm,
            "P term must be dt-independent for same velocity error"
        );
    }

    #[test]
    fn test_time_invariant_i() {
        // Two updates of 10ms vs one of 20ms at same constant velocity → same output
        let gains = Gains::new(0.0, 1.0, 0.0).unwrap();

        let mut ctrl_a = WheelController::new(0);
        ctrl_a.update(&gains, 5, 0, Duration::from_millis(10));
        let out_a = ctrl_a.update(&gains, 10, 0, Duration::from_millis(10));

        let mut ctrl_b = WheelController::new(0);
        let out_b = ctrl_b.update(&gains, 10, 0, Duration::from_millis(20));

        assert_eq!(
            out_a.motor_pwm, out_b.motor_pwm,
            "integral must accumulate proportionally to elapsed time"
        );
    }

    #[test]
    fn test_derivative_damps_acceleration() {
        // With only D term: output should oppose increasing velocity
        let gains_d = Gains::new(0.0, 0.0, 0.01).unwrap();
        let gains_no_d = Gains::new(0.0, 0.0, 0.0).unwrap();

        let mut ctrl_d = WheelController::new(0);
        let mut ctrl_no_d = WheelController::new(0);

        // First update: velocity goes from 0 to 300 ticks/s
        ctrl_d.update(&gains_d, 3, 0, DT_10MS);
        ctrl_no_d.update(&gains_no_d, 3, 0, DT_10MS);

        // Second update: velocity jumps to 600 ticks/s — D term should resist increase
        let out_d = ctrl_d.update(&gains_d, 9, 0, DT_10MS);
        let out_no_d = ctrl_no_d.update(&gains_no_d, 9, 0, DT_10MS);

        assert!(
            out_d.motor_pwm.get() < out_no_d.motor_pwm.get(),
            "D term must produce more negative output when velocity is increasing: d={}, no_d={}",
            out_d.motor_pwm.get(),
            out_no_d.motor_pwm.get()
        );
    }

    #[test]
    fn test_derivative_time_invariant() {
        // Same acceleration at different dt → same D output
        let gains = Gains::new(0.0, 0.0, 0.01).unwrap();

        // ctrl_a: 0→300 ticks/s in 10ms, then 300→600 ticks/s in 10ms (accel = 30_000 ticks/s²)
        let mut ctrl_a = WheelController::new(0);
        ctrl_a.update(&gains, 3, 0, DT_10MS);
        let out_a = ctrl_a.update(&gains, 9, 0, DT_10MS);

        // ctrl_b: 0→600 ticks/s in 20ms, then 600→1200 ticks/s in 20ms (accel = 30_000 ticks/s²)
        let mut ctrl_b = WheelController::new(0);
        ctrl_b.update(&gains, 12, 0, Duration::from_millis(20));
        let out_b = ctrl_b.update(&gains, 36, 0, Duration::from_millis(20));

        assert_eq!(
            out_a.motor_pwm, out_b.motor_pwm,
            "D term must be dt-independent for same acceleration"
        );
    }
}
