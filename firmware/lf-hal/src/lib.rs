#![no_std]

pub mod adc;
pub mod button;
pub mod line_sensor;
pub mod motors;

pub use adc::{BatterySensor, RangeSensor};

use button::Button;
use esp_hal::{
    gpio::{Level, Output, OutputConfig, Pin},
    interrupt::software::SoftwareInterruptControl,
    peripherals::{BT, Peripherals},
    timer::timg::TimerGroup,
};
use esp_radio::ble::controller::BleConnector;
use line_sensor::LineSensor;
use motors::{MotorChannelPins, Motors};
use trouble_host::prelude::ExternalController;

/// BLE controller type produced by [`Hal::init_bt_controller`].
pub type BleController<const N: usize> = ExternalController<BleConnector<'static>, N>;

pub struct Hal<'d> {
    pub motors: Motors<'d>,
    pub deck_button: Button<'d>,
    pub boot_button: Button<'d>,
    pub line_sensor: LineSensor<'d>,
    led: Output<'d>,
    pub range: RangeSensor,
    pub battery: BatterySensor,
    bt: Option<BT<'d>>,
}

impl<'d> Hal<'d> {
    /// Initialize hardware and the RTOS scheduler.
    #[must_use]
    pub fn setup(p: Peripherals) -> Self {
        let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);
        esp_rtos::start(TimerGroup::new(p.TIMG0).timer0, sw_ints.software_interrupt0);

        // Battery and range are on ADC2, which LineSensor also consumes; the sensors own their
        // own GPIOs and read ADC2 directly via register access.
        let battery = BatterySensor::new(p.GPIO15);
        let range = RangeSensor::new(p.GPIO12);

        let line_sensor = LineSensor::new(
            [p.GPIO27.degrade(), p.GPIO32.degrade(), p.GPIO26.degrade()],
            p.ADC1,
            p.ADC2,
            (p.GPIO33, p.GPIO14, p.GPIO35, p.GPIO25, p.GPIO34),
        );

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
            range,
            battery,
            bt: Some(p.BT),
        }
    }

    /// Initialize the radio and return a BLE controller. Requires a heap.
    ///
    /// # Panics
    /// Panics if called more than once, or if the BLE connector fails to initialize.
    pub fn init_bt_controller<const N: usize>(&mut self) -> BleController<N>
    where
        'd: 'static,
    {
        ExternalController::<_, N>::new(
            BleConnector::new(
                self.bt
                    .take()
                    .expect("init_bt_controller called more than once"),
                esp_radio::ble::Config::default(),
            )
            .expect("BLE connector init failed"),
        )
    }

    pub fn set_led(&mut self, on: bool) {
        self.led.set_level(on.into());
    }
}
