use embassy_time::{Duration, Timer};
use esp_hal::analog::adc::{Adc, AdcChannel, AdcConfig, Attenuation, RegisterAccess};
use esp_hal::gpio::{AnalogPin, AnyPin, Flex};
use esp_hal::peripherals::{ADC1, ADC2, SENS};
use lf_hal_types::line_sensor::SENSOR_COUNT;
pub use lf_hal_types::line_sensor::{LedIndex, SensorReadings};

/// How much time we wait after lighting up the LED to have the signal settle.
/// This is mostly a guess, that seems to work ok.
const SETTLE_TIME: Duration = Duration::from_micros(500);

/// Attenuation used in the ADC.
const ATTENUATION: Attenuation = Attenuation::_0dB;
const PHOTOTRANSISTOR_COUNT: usize = 5;
const LED_COUNT: usize = LedIndex::MAX.get() + 1;

const _: () = assert!(
    LED_COUNT == PHOTOTRANSISTOR_COUNT + 1,
    "There's one LED on each side of every sensor."
);
const _: () = assert!(
    SENSOR_COUNT == 2 * PHOTOTRANSISTOR_COUNT,
    "Two logical sensors for every phototransistor."
);

type SensorBuffer = [i16; SENSOR_COUNT];

/// Maps LED index to (low_pin, high_pin, hiz_pin) indices into self.led_pins.
/// Pins order: [0=GPIO27, 1=GPIO32, 2=GPIO26]
const LED_PATTERNS: [(usize, usize, usize); LedIndex::MAX.get() + 1] = [
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
struct PhototransistorAdcChannel {
    unit: AdcUnit,
    channel: u8,
}

impl PhototransistorAdcChannel {
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
    adc_channels: [PhototransistorAdcChannel; PHOTOTRANSISTOR_COUNT],
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
            PhototransistorAdcChannel::new(&mut adc1_config, AdcUnit::Adc1, sensor_pins.0),
            PhototransistorAdcChannel::new(&mut adc2_config, AdcUnit::Adc2, sensor_pins.1),
            PhototransistorAdcChannel::new(&mut adc1_config, AdcUnit::Adc1, sensor_pins.2),
            PhototransistorAdcChannel::new(&mut adc2_config, AdcUnit::Adc2, sensor_pins.3),
            PhototransistorAdcChannel::new(&mut adc1_config, AdcUnit::Adc1, sensor_pins.4),
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
            adc_channels: sensors,
        }
    }

    /// Enable the LED at `index` by setting the charlieplex pattern.
    pub fn enable_led(&mut self, index: LedIndex) {
        let (low, high, hiz) = LED_PATTERNS[index.get()];
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

    /// Performs a full line sensor read with ambient light subtraction.
    /// Index 0 is on the left side of the robot.
    pub async fn read(&mut self) -> SensorReadings {
        let mut buffer = self.read_with_leds().await;
        self.subtract_ambient(&mut buffer).await;
        SensorReadings { values: buffer }
    }

    /// Performs a raw sensor read.
    /// Index 0 is on the left side of the robot.
    pub fn read_raw(&mut self) -> [i16; PHOTOTRANSISTOR_COUNT] {
        let mut buffer = [0i16; PHOTOTRANSISTOR_COUNT];
        for (i, v) in self.read_ambient_iter().enumerate() {
            buffer[i] = v;
        }

        buffer
    }

    /// Illuminate each LED in sequence and read the adjacent sensor(s).
    async fn read_with_leds(&mut self) -> SensorBuffer {
        let mut buffer = [0i16; SENSOR_COUNT];

        // LED 0: only reads sensor 0
        self.enable_led(LedIndex::MIN);
        Timer::after(SETTLE_TIME).await;
        buffer[0] = self.read_single(0);

        // LEDs in the middle: each illuminates between sensors (i-1) and i
        // Separating odd first, then even, to have the ADC1 and ADC2 ordering fixed
        for i in (1..(LED_COUNT - 2)).step_by(2) {
            self.enable_led(LedIndex::new(i).unwrap());
            Timer::after(SETTLE_TIME).await;
            let [va, vb] = self.read_pair(i - 1, i);
            buffer[2 * i - 1] = va;
            buffer[2 * i] = vb;
        }
        for i in (2..(LED_COUNT - 1)).step_by(2) {
            self.enable_led(LedIndex::new(i).unwrap());
            Timer::after(SETTLE_TIME).await;
            let [vb, va] = self.read_pair(i, i - 1);
            buffer[2 * i - 1] = va;
            buffer[2 * i] = vb;
        }

        // last LED: only reads last sensor
        self.enable_led(LedIndex::MAX);
        Timer::after(SETTLE_TIME).await;
        buffer[SENSOR_COUNT - 1] = self.read_single(PHOTOTRANSISTOR_COUNT - 1);

        buffer
    }

    /// Reads the ambient light with LEDs disabled and subtracts the values from `buffer`.
    async fn subtract_ambient(&mut self, buffer: &mut SensorBuffer) {
        self.disable_leds();
        Timer::after(SETTLE_TIME).await;

        for (i, ambient) in self.read_ambient_iter().enumerate() {
            buffer[2 * i] -= ambient;
            buffer[2 * i + 1] -= ambient;
        }
    }

    fn read_ambient_iter(&self) -> impl Iterator<Item = i16> {
        (0..PHOTOTRANSISTOR_COUNT - 1)
            .step_by(2)
            .flat_map(|i| self.read_pair(i, i + 1))
            .chain(core::iter::once_with(|| {
                self.read_single(PHOTOTRANSISTOR_COUNT - 1)
            }))
    }

    /// Read a single sensor (blocking).
    fn read_single(&self, index: usize) -> i16 {
        let s = self.adc_channels[index];
        match s.unit {
            AdcUnit::Adc1 => {
                Self::start_adc::<ADC1>(s.channel);
                Self::read_adc::<ADC1>()
            }
            // Critical section prevents the BLE PHY ISR from running a concurrent
            // ADC2 conversion (for TX power detection) and corrupting the result.
            AdcUnit::Adc2 => critical_section::with(|_| {
                Self::start_adc2(s.channel);
                Self::read_adc::<ADC2>()
            }),
        }
    }

    /// Read sensors at `index` and `index + 1` in parallel.
    /// Adjacent sensors must be on different ADC units (PCB layout guarantee),
    /// so both conversions overlap in hardware.
    fn read_pair(&self, index_a: usize, index_b: usize) -> [i16; 2] {
        let a = self.adc_channels[index_a];
        let b = self.adc_channels[index_b];
        assert!(
            matches!((a.unit, b.unit), (AdcUnit::Adc1, AdcUnit::Adc2)),
            "read_pair: sensors {} and {} must be on ADC units 1 and 2 respectively (currently {:?} and {:?})",
            index_a,
            index_b,
            a.unit,
            b.unit
        );
        // Critical section prevents the BLE PHY ISR from running a concurrent
        // ADC2 conversion (for TX power detection) and corrupting the result.
        critical_section::with(|_| {
            Self::start_adc::<ADC1>(a.channel);
            Self::start_adc2(b.channel);
            [Self::read_adc::<ADC1>(), Self::read_adc::<ADC2>()]
        })
    }

    fn start_adc<ADC: RegisterAccess>(channel: u8) {
        ADC::set_en_pad(channel);
        ADC::clear_start_sar();
        ADC::set_start_sar();
    }

    pub(super) fn start_adc2(channel: u8) {
        // BLE leaves SAR2_PWDET_FORCE=1 in SAR_READ_CTRL2 after TX power
        // detection, routing the ADC2 input to the internal PWDET signal
        // instead of the external GPIO pin. Clear it before each read.
        // Must be called inside a critical section so the PHY ISR cannot
        // observe the transient state.
        SENS::regs()
            .sar_read_ctrl2()
            .modify(|_, w| w.sar2_pwdet_force().clear_bit());
        Self::start_adc::<ADC2>(channel);
    }

    pub(super) fn read_adc<ADC: RegisterAccess>() -> i16 {
        while !ADC::read_done_sar() {}
        ADC::read_data_sar() as i16
    }
}
