#![no_std]

pub mod motors {
    use core::f64::consts::PI;

    use deranged::RangedI16;

    /// PWM command range for a single motor. Positive = forward, negative = backward, 0 = brake.
    pub type PwmT = RangedI16<-1024, 1024>;

    /// Track width of the robot in meters
    pub const TRACK_WIDTH_M: f32 = 86e-3;

    /// Approximate wheel travel per encoder tick, assuming zero slip.
    /// Wheel diameter 40 mm, encoder 7 CPR, gear ratio 1:30, 2-channel counting.
    /// TODO: calibrate
    pub const METERS_PER_TICK: f32 = (40e-3 * PI / (7.0 * 2.0 * 30.0)) as f32;

    /// Approximate maximum speed of unloaded motors in ticks per second.
    /// Don't ask the robot to go this fast!
    pub const MAX_SPEED: u32 = 15000;
}

pub struct BatteryMeasurement {
    pub raw: u16,
}

impl BatteryMeasurement {
    pub fn voltage(&self) -> f32 {
        const K: f32 = 0.007278;
        const A: f32 = -8.963262;

        K * self.raw as f32 + A
    }
}

pub struct RangeMeasurement {
    pub raw: u16,
}

impl RangeMeasurement {
    pub fn distance_long(&self) -> f32 {
        const A: f32 = 318.913_54;
        const B: f32 = -19.318_924;

        A / (self.raw as f32 + B)
    }

    pub fn distance_short(&self) -> Option<f32> {
        todo!("implement short-range option");
    }
}

pub mod line_sensor {
    use deranged::RangedUsize;

    /// Number of line brightness readouts.
    pub const SENSOR_COUNT: usize = 10;

    /// Index of a charlieplexed LED. Index 0 is on the left side of the robot.
    pub type LedIndex = RangedUsize<0, 5>;

    /// Compensated line sensor readings from one full read cycle.
    /// Index 0 is on the left side of the robot.
    pub struct SensorReadings {
        pub values: [i16; SENSOR_COUNT],
    }

    /// Radius of the line sensor in meters.
    /// The line sensor is centered around the steering center.
    pub const RADIUS_M: f32 = 82e-3;
    /// Angle in degrees between the leftmost and rightmost measured point
    pub const ANGLE_DEG: f32 = 31.5;
}
