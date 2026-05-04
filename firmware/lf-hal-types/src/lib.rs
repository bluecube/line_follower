#![no_std]

pub mod motors {
    use deranged::RangedI16;

    /// PWM command range for a single motor. Positive = forward, negative = backward, 0 = brake.
    pub type PwmT = RangedI16<-1024, 1024>;
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
        // TODO: fit curve to measured data
        0.0
    }

    pub fn distance_short(&self) -> Option<f32> {
        // TODO: implement short-range option
        None
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
}
