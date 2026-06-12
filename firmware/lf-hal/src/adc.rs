//! Low-level ADC register access and the ADC-backed sensor types.
//!
//! Conversions use `RegisterAccess` directly rather than esp-hal's `Adc` object: the ADC
//! peripherals are consumed for configuration elsewhere and then released, and reads happen
//! through the raw register interface from here on. ADC2 is shared with the BLE PHY (TX power
//! detection) and also between line sensor, battery measurement and range sensor, so every ADC2
//! access is wrapped in a critical section.

use esp_hal::analog::adc::{AdcChannel, Attenuation, RegisterAccess};
use esp_hal::peripherals::{ADC1, ADC2, SENS};
use lf_hal_types::{BatteryMeasurement, RangeMeasurement};

/// Attenuation used by the range and battery channels (full 0-3.3V range).
const SENSOR_ATTENUATION: Attenuation = Attenuation::_11dB;

fn start_adc<ADC: RegisterAccess>(channel: u8) {
    ADC::set_en_pad(channel);
    ADC::clear_start_sar();
    ADC::set_start_sar();
}

/// Starts an ADC2 conversion, first clearing the PWDET routing left behind by the BLE PHY.
///
/// BLE leaves `SAR2_PWDET_FORCE=1` in `SAR_READ_CTRL2` after TX power detection, routing the
/// ADC2 input to the internal PWDET signal instead of the external GPIO pin. Must be called
/// inside a critical section so the PHY ISR cannot observe the transient state.
fn start_adc2(channel: u8) {
    SENS::regs()
        .sar_read_ctrl2()
        .modify(|_, w| w.sar2_pwdet_force().clear_bit());
    start_adc::<ADC2>(channel);
}

fn read_adc<ADC: RegisterAccess>() -> u16 {
    while !ADC::read_done_sar() {}
    ADC::read_data_sar()
}

/// Reads a single ADC1 channel (blocking). ADC1 isn't shared with the BLE PHY, so no critical
/// section is needed.
pub(crate) fn read_adc1(channel: u8) -> u16 {
    start_adc::<ADC1>(channel);
    read_adc::<ADC1>()
}

/// Reads a single ADC2 channel (blocking), guarded against concurrent BLE PHY access.
pub(crate) fn read_adc2(channel: u8) -> u16 {
    critical_section::with(|_| {
        start_adc2(channel);
        read_adc::<ADC2>()
    })
}

/// Reads one ADC1 and one ADC2 channel with overlapping conversions, returning
/// `[adc1_value, adc2_value]`. The shared critical section also serializes the ADC2 half
/// against the BLE PHY.
pub(crate) fn read_adc1_adc2(adc1_channel: u8, adc2_channel: u8) -> [u16; 2] {
    critical_section::with(|_| {
        start_adc::<ADC1>(adc1_channel);
        start_adc2(adc2_channel);
        [read_adc::<ADC1>(), read_adc::<ADC2>()]
    })
}

/// Sets the attenuation for an ADC2 channel.
///
/// Works around an esp-hal bug where `Adc::new` hardcodes `ADC1::set_attenuation` even for
/// ADC2, leaving ADC2 channels at the 0dB default.
pub(crate) fn set_adc2_attenuation(channel: u8, attenuation: Attenuation) {
    ADC2::set_attenuation(channel as usize, attenuation as u8);
}

/// Sharp GP2Y0A21YK0F IR range sensor on an ADC2 channel.
#[derive(Clone, Copy)]
pub struct RangeSensor {
    channel: u8,
}

impl RangeSensor {
    #[must_use]
    pub fn new(pin: impl AdcChannel) -> Self {
        let channel = pin.adc_channel();
        set_adc2_attenuation(channel, SENSOR_ATTENUATION);
        Self { channel }
    }

    /// Reads the range measurement from the Sharp GP2Y0A21YK0F sensor.
    /// The part only updates positions roughly every 40ms!
    #[must_use]
    pub fn read(&self) -> RangeMeasurement {
        RangeMeasurement {
            raw: read_adc2(self.channel),
        }
    }
}

/// Battery voltage divider on an ADC2 channel.
#[derive(Clone, Copy)]
pub struct BatterySensor {
    channel: u8,
}

impl BatterySensor {
    #[must_use]
    pub fn new(pin: impl AdcChannel) -> Self {
        let channel = pin.adc_channel();
        set_adc2_attenuation(channel, SENSOR_ATTENUATION);
        Self { channel }
    }

    #[must_use]
    pub fn read(&self) -> BatteryMeasurement {
        BatteryMeasurement {
            raw: read_adc2(self.channel),
        }
    }
}
