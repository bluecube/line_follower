#![no_std]

pub mod button;
pub mod line_sensor;
pub mod motors;

use button::Button;
use esp_hal::{
    analog::adc::{AdcChannel, Attenuation, RegisterAccess},
    gpio::{Level, Output, OutputConfig, Pin},
    peripherals::{ADC2, Peripherals},
    timer::timg::TimerGroup,
};
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
    /// Initialize hardware and the RTOS scheduler.
    pub fn setup(p: Peripherals) -> Self {
        esp_rtos::start(TimerGroup::new(p.TIMG0).timer0);

        // ADC2 is consumed by LineSensor, extract battery and range ADC pins before that
        // and then set attenuation using direct register access
        let battery_adc_channel = p.GPIO15.adc_channel();
        let range_adc_channel = p.GPIO12.adc_channel();
        drop(p.GPIO15);
        drop(p.GPIO12);

        let line_sensor = LineSensor::new(
            [p.GPIO27.degrade(), p.GPIO32.degrade(), p.GPIO26.degrade()],
            p.ADC1,
            p.ADC2,
            (p.GPIO33, p.GPIO14, p.GPIO35, p.GPIO25, p.GPIO34),
        );

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
        self.led.set_level(on.into());
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
