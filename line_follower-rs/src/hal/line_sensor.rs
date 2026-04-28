use esp_hal::analog::adc::{Adc, AdcChannel, AdcConfig, Attenuation, RegisterAccess};
use esp_hal::delay::Delay;
use esp_hal::gpio::{AnalogPin, AnyPin, Flex};
use esp_hal::peripherals::{ADC1, ADC2};
use esp_hal::time::Duration;

const SETTLE_TIME: Duration = Duration::from_micros(1000); // TODO: Maybe we can have less time here?

const SENSOR_COUNT: usize = 5;
const LED_COUNT: usize = 6;
pub const MEASUREMENT_COUNT: usize = 2 * SENSOR_COUNT;
const ATTENUATION: Attenuation = Attenuation::_0dB;

pub type SensorBuffer = [i16; MEASUREMENT_COUNT];

// Maps LED index to (low_pin, high_pin, hiz_pin) indices into self.led_pins.
// Pins order: [0=GPIO27, 1=GPIO32, 2=GPIO26]
const LED_PATTERNS: [(usize, usize, usize); LED_COUNT] = [
    (0, 1, 2),
    (1, 0, 2),
    (0, 2, 1),
    (2, 0, 1),
    (2, 1, 0),
    (1, 2, 0),
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum AdcUnit {
    Adc1,
    Adc2,
}

// ADC low-level access via RegisterAccess
#[derive(Clone, Copy)]
struct SensorChannel {
    unit: AdcUnit,
    channel: u8,
}

impl SensorChannel {
    fn new<ADCI, P>(config: &mut AdcConfig<ADCI>, unit: AdcUnit, pin: P) -> Self
    where
        P: AdcChannel + AnalogPin,
    {
        let channel = pin.adc_channel();
        let _ = config.enable_pin(pin, ATTENUATION);
        Self { unit, channel }
    }
}

pub struct LineSensor<'d> {
    led_pins: [Flex<'d>; 3],
    sensors: [SensorChannel; SENSOR_COUNT],
}

impl<'d> LineSensor<'d> {
    /// Create a new line sensor.
    ///
    /// Sensor pins are passed in physical order (0-4). By PCB layout,
    /// even-indexed sensors (0, 2, 4) are on ADC1 and odd-indexed (1, 3)
    /// are on ADC2. The ADC peripherals are consumed for configuration
    /// and then released; subsequent reads use `RegisterAccess` directly.
    pub fn new<S0, S1, S2, S3, S4>(
        led_pins: [AnyPin<'d>; 3],
        adc1: ADC1<'d>,
        adc2: ADC2<'d>,
        sensor_pins: (S0, S1, S2, S3, S4),
    ) -> Self
    where
        S0: AdcChannel + AnalogPin,
        S1: AdcChannel + AnalogPin,
        S2: AdcChannel + AnalogPin,
        S3: AdcChannel + AnalogPin,
        S4: AdcChannel + AnalogPin,
    {
        let mut adc1_config = AdcConfig::new();
        let mut adc2_config = AdcConfig::new();

        let sensors = [
            SensorChannel::new(&mut adc1_config, AdcUnit::Adc1, sensor_pins.0),
            SensorChannel::new(&mut adc2_config, AdcUnit::Adc2, sensor_pins.1),
            SensorChannel::new(&mut adc1_config, AdcUnit::Adc1, sensor_pins.2),
            SensorChannel::new(&mut adc2_config, AdcUnit::Adc2, sensor_pins.3),
            SensorChannel::new(&mut adc1_config, AdcUnit::Adc1, sensor_pins.4),
        ];

        let _ = Adc::new(adc1, adc1_config);
        let _ = Adc::new(adc2, adc2_config);
        // Adc instances drop here, but register configuration persists.

        // esp-hal bug: Adc::new hardcodes ADC1::set_attenuation even for ADC2,
        // so ADC2 channels are left at the 0dB default. Set them explicitly.
        for s in &sensors {
            if s.unit == AdcUnit::Adc2 {
                ADC2::set_attenuation(s.channel as usize, ATTENUATION as u8);
            }
        }

        let [p0, p1, p2] = led_pins;
        Self {
            led_pins: [Flex::new(p0), Flex::new(p1), Flex::new(p2)],
            sensors,
        }
    }

    /// Enable the LED at `index` (0-5) by setting the charlieplex pattern.
    pub fn enable_led(&mut self, index: usize) {
        let (low, high, hiz) = LED_PATTERNS[index];
        self.led_pins[hiz].set_output_enable(false);
        self.led_pins[low].set_low();
        self.led_pins[low].set_output_enable(true);
        self.led_pins[high].set_high();
        self.led_pins[high].set_output_enable(true);
    }

    /// Disable all LEDs by floating all three pins.
    pub fn disable_leds(&mut self) {
        for pin in &mut self.led_pins {
            pin.set_output_enable(false);
        }
    }

    /// Full line sensor read with ambient light subtraction.
    /// Returns the 10-element buffer and (min, max) of the values.
    pub fn read(&mut self, delay: &Delay) -> (SensorBuffer, i16, i16) {
        // Phase 1: read with LEDs on
        let mut buffer = self.read_with_leds(delay);

        // Phase 2: ambient (all LEDs off), subtract from illuminated values
        let (min, max) = self.subtract_ambient(delay, &mut buffer);

        (buffer, min, max)
    }

    pub fn read_raw(&mut self) -> [i16; SENSOR_COUNT] {
        let mut buffer = [0i16; SENSOR_COUNT];
        for (i, v) in self.read_ambient_iter().enumerate() {
            buffer[i] = v;
        }

        buffer
    }

    /// Illuminate each LED in sequence and read the adjacent sensor(s).
    fn read_with_leds(&mut self, delay: &Delay) -> SensorBuffer {
        let mut buffer = [0i16; MEASUREMENT_COUNT];

        // LED 0: only reads sensor 0
        self.enable_led(0);
        delay.delay(SETTLE_TIME);
        buffer[0] = self.read_single(0);

        // LEDs in the middle: each illuminates between sensors (i-1) and i
        // Separating odd first, then even, to have the ADC1 and ADC2 ordering fixed
        for i in (1..(LED_COUNT - 2)).step_by(2) {
            self.enable_led(i);
            delay.delay(SETTLE_TIME);

            let [va, vb] = self.read_pair(i - 1, i);
            buffer[2 * i - 1] = va;
            buffer[2 * i] = vb;
        }
        for i in (2..(LED_COUNT - 1)).step_by(2) {
            self.enable_led(i);
            delay.delay(SETTLE_TIME);

            let [vb, va] = self.read_pair(i, i - 1);
            buffer[2 * i - 1] = va;
            buffer[2 * i] = vb;
        }

        // last LED: only reads last sensor
        self.enable_led(LED_COUNT - 1);
        delay.delay(SETTLE_TIME);
        buffer[MEASUREMENT_COUNT - 1] = self.read_single(SENSOR_COUNT - 1);

        buffer
    }

    /// Reads the ambient light with LEDs disabled, subtracts the values from
    /// `buffer` and returns the minimum and maximum value in the buffer after the subtraction.
    fn subtract_ambient(&mut self, delay: &Delay, buffer: &mut SensorBuffer) -> (i16, i16) {
        let mut min = i16::MAX;
        let mut max = i16::MIN;

        self.disable_leds();
        delay.delay(SETTLE_TIME);

        for (i, ambient) in self.read_ambient_iter().enumerate() {
            let mut apply = |idx: usize| {
                let value = buffer[idx] - ambient;
                if value < min {
                    min = value;
                }
                if value > max {
                    max = value;
                }
                buffer[idx] = value;
            };

            apply(2 * i);
            apply(2 * i + 1);
        }

        (min, max)
    }

    fn read_ambient_iter(&self) -> impl Iterator<Item = i16> {
        (0..SENSOR_COUNT - 1)
            .step_by(2)
            .flat_map(|i| self.read_pair(i, i + 1))
            .chain(core::iter::once_with(|| self.read_single(SENSOR_COUNT - 1)))
    }

    /// Read a single sensor (blocking).
    fn read_single(&self, index: usize) -> i16 {
        let s = self.sensors[index];
        match s.unit {
            AdcUnit::Adc1 => {
                Self::start_adc::<ADC1>(s.channel);
                // ADC2 reads inverted (high voltage -> 0),
                // but we have a pull-up on the sensors so we need to actually invert ADC1
                4095 - Self::read_adc::<ADC1>()
            }
            AdcUnit::Adc2 => {
                Self::start_adc::<ADC2>(s.channel);
                Self::read_adc::<ADC2>()
            }
        }
    }

    /// Read sensors at `index` and `index + 1` in parallel.
    /// Adjacent sensors must be on different ADC units (PCB layout guarantee),
    /// so both conversions overlap in hardware.
    fn read_pair(&self, index_a: usize, index_b: usize) -> [i16; 2] {
        let a = self.sensors[index_a];
        let b = self.sensors[index_b];
        assert!(
            matches!((a.unit, b.unit), (AdcUnit::Adc1, AdcUnit::Adc2)),
            "read_pair: sensors {} and {} must be on ADC units 1 and 2 respectively (currently {:?} and {:?})",
            index_a,
            index_b,
            a.unit,
            b.unit
        );
        // Self::start_adc::<ADC1>(a.channel);
        // let v1 = Self::read_adc::<ADC1>();
        // Self::start_adc::<ADC2>(b.channel);
        // let v2 = Self::read_adc::<ADC2>();

        // [v1, v2]
        Self::start_adc::<ADC1>(a.channel);
        Self::start_adc::<ADC2>(b.channel);

        // ADC2 reads inverted (high voltage -> 0),
        // but we have a pull-up on the sensors so we need to actually invert ADC1
        [4095 - Self::read_adc::<ADC1>(), Self::read_adc::<ADC2>()]
    }

    fn start_adc<ADC: RegisterAccess>(channel: u8) {
        ADC::set_en_pad(channel);
        ADC::clear_start_sar();
        ADC::set_start_sar();
    }

    fn read_adc<ADC: RegisterAccess>() -> i16 {
        while !ADC::read_done_sar() {}
        ADC::read_data_sar() as i16
    }
}
