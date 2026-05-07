#![no_std]

pub mod button;
pub mod line_sensor;
pub mod motors;

use button::Button;
use esp_hal::analog::adc::{AdcChannel, Attenuation, RegisterAccess};
use esp_hal::gpio::{Level, Output, OutputConfig, Pin};
use esp_hal::peripherals::{ADC2, Peripherals};
use lf_hal_types::{BatteryMeasurement, RangeMeasurement};
use line_sensor::LineSensor;
use motors::{MotorChannelPins, Motors};

pub struct Hal<'d> {
    pub motors: Motors<'d>,
    pub deck_button: Button<'d>,
    pub boot_button: Button<'d>,
    pub line_sensor: LineSensor<'d>,
    led: Output<'d>,
    battery_adc_channel: u8,
    range_adc_channel: u8,
}

impl<'d> Hal<'d> {
    pub fn new(p: Peripherals) -> Self {
        // ── Remaining pin assignments (not yet implemented) ──────────
        // I2C:                     SCL=GPIO22  SDA=GPIO21
        // Accel interrupt:         GPIO18

        // Get ADC2 channel numbers before ADC2 is consumed by LineSensor.
        let battery_adc_channel = p.GPIO15.adc_channel();
        let range_adc_channel = p.GPIO12.adc_channel();
        // Drop both pins — the channel numbers are all that's needed.
        drop(p.GPIO15);
        drop(p.GPIO12);

        let line_sensor = LineSensor::new(
            [p.GPIO27.degrade(), p.GPIO32.degrade(), p.GPIO26.degrade()],
            p.ADC1,
            p.ADC2,
            (p.GPIO33, p.GPIO14, p.GPIO35, p.GPIO25, p.GPIO34),
        );

        // ADC2 is now consumed by LineSensor. Configure battery and range
        // attenuation directly via registers (same workaround used in line_sensor.rs).
        ADC2::set_attenuation(battery_adc_channel as usize, Attenuation::_11dB as u8);
        ADC2::set_attenuation(range_adc_channel as usize, Attenuation::_11dB as u8);

        Self {
            deck_button: Button::new(p.GPIO5),
            boot_button: Button::new(p.GPIO0),
            led: Output::new(p.GPIO2, Level::Low, OutputConfig::default()),
            line_sensor,
            motors: Motors::new(
                p.MCPWM0,
                p.PCNT,
                MotorChannelPins {
                    pwm_a: p.GPIO13.degrade(),
                    pwm_b: p.GPIO4.degrade(),
                    enc_a: p.GPIO16.degrade(),
                    enc_b: p.GPIO17.degrade(),
                },
                MotorChannelPins {
                    pwm_a: p.GPIO23.degrade(),
                    pwm_b: p.GPIO19.degrade(),
                    enc_a: p.GPIO36.degrade(),
                    enc_b: p.GPIO39.degrade(),
                },
            ),
            battery_adc_channel,
            range_adc_channel,
        }
    }

    pub fn set_led(&mut self, on: bool) {
        if on {
            self.led.set_high();
        } else {
            self.led.set_low();
        }
    }

    pub fn read_battery(&self) -> BatteryMeasurement {
        BatteryMeasurement {
            raw: self.read_adc2(self.battery_adc_channel),
        }
    }

    pub fn read_range(&self) -> RangeMeasurement {
        RangeMeasurement {
            raw: self.read_adc2(self.range_adc_channel),
        }
    }

    fn read_adc2(&self, channel: u8) -> u16 {
        ADC2::set_en_pad(channel);
        ADC2::clear_start_sar();
        ADC2::set_start_sar();
        while !ADC2::read_done_sar() {}
        ADC2::read_data_sar() as u16
    }
}
