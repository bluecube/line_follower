#![no_std]

pub mod motors {
    use deranged::RangedI16;

    /// PWM command range for a single motor. Positive = forward, negative = backward, 0 = brake.
    pub type PwmT = RangedI16<-1024, 1024>;
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
