pub mod button;
pub mod line_sensor;
pub mod motors;

use button::Button;
use esp_hal::{gpio::Pin, peripherals::Peripherals};
use line_sensor::LineSensor;
use motors::{MotorChannelPins, Motors};

pub struct Hal<'d> {
    pub motors: Motors<'d>,
    pub deck_button: Button<'d>,
    pub boot_button: Button<'d>,
    pub line_sensor: LineSensor<'d>,
}

impl<'d> Hal<'d> {
    pub fn new(p: Peripherals) -> Self {
        // ── Motor pins ───────────────────────────────────────────────
        // Left:  PWM A=GPIO13  PWM B=GPIO4   ENC A=GPIO16  ENC B=GPIO17
        // Right: PWM A=GPIO23  PWM B=GPIO19  ENC A=GPIO36  ENC B=GPIO39

        // ── Remaining pin assignments (not yet implemented) ──────────
        // Line sensor ADC:         GPIO33, GPIO14, GPIO35, GPIO25, GPIO34
        // Range sensor:            GPIO12
        // I2C:                     SCL=GPIO22  SDA=GPIO21
        // Accel interrupt:         GPIO18
        // Battery sense:           GPIO15
        // Indicator LED:           GPIO2

        Self {
            deck_button: Button::new(p.GPIO5),
            boot_button: Button::new(p.GPIO0),
            line_sensor: LineSensor::new([
                p.GPIO27.degrade(),
                p.GPIO32.degrade(),
                p.GPIO26.degrade(),
            ]),
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
        }
    }
}
