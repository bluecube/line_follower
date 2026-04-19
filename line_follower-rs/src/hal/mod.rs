pub mod motors;

use esp_hal::{gpio::Pin, peripherals::Peripherals};
use motors::{MotorChannelPins, Motors};

pub struct Hal<'d> {
    pub motors: Motors<'d>,
}

impl<'d> Hal<'d> {
    pub fn new(p: Peripherals) -> Self {
        // ── Motor pins ───────────────────────────────────────────────
        // Left:  PWM A=GPIO13  PWM B=GPIO4   ENC A=GPIO16  ENC B=GPIO17
        // Right: PWM A=GPIO23  PWM B=GPIO19  ENC A=GPIO36  ENC B=GPIO39

        // ── Remaining pin assignments (not yet implemented) ──────────
        // Line sensor ADC:         GPIO33, GPIO14, GPIO35, GPIO25, GPIO34
        // Line LEDs (charlieplex): GPIO27, GPIO32, GPIO26
        // Range sensor:            GPIO12
        // I2C:                     SCL=GPIO22  SDA=GPIO21
        // Accel interrupt:         GPIO18
        // Deck button:             GPIO5   Boot button: GPIO0
        // Battery sense:           GPIO15
        // Indicator LED:           GPIO2

        Self {
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
