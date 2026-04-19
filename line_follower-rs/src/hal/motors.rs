use core::f64::consts::PI;

use esp_hal::{
    gpio::{AnyPin, Input, InputConfig, InputPin, Pull, interconnect::PeripheralOutput},
    mcpwm::{
        McPwm, PeripheralClockConfig,
        operator::{PwmActions, PwmPin, PwmPinConfig, PwmUpdateMethod, UpdateAction},
        timer::PwmWorkingMode,
    },
    pcnt::{
        Pcnt,
        channel::{CtrlMode, EdgeMode},
        unit::{Counter, Unit},
    },
    peripherals::{MCPWM0, PCNT},
    time::Rate,
};

use deranged::RangedI16;

pub struct MotorChannelPins {
    pub pwm_a: AnyPin<'static>,
    pub pwm_b: AnyPin<'static>,
    pub enc_a: AnyPin<'static>,
    pub enc_b: AnyPin<'static>,
}

pub type PwmT = RangedI16<-1024, 1024>;

const TARGET_PWM_FREQ: Rate = Rate::from_hz(25_000);

/// Glitch filter: maximum hardware value (10-bit register, ~6.4 µs at 160 MHz APB).
const PCNT_FILTER: u16 = 1023;

/// Generator action that holds the output unconditionally LOW.
/// Applied to the inactive H-bridge leg while the other leg is
/// PWM-controlled, and to both legs when braking.
const FORCE_LOW_A: PwmActions<true> = PwmActions::empty()
    .on_up_counting_timer_equals_zero(UpdateAction::SetLow)
    .on_up_counting_timer_equals_timestamp(UpdateAction::SetLow);
/// Same as FORCE_LOW_A, but typed to work on the B channel
const FORCE_LOW_B: PwmActions<false> = PwmActions::empty()
    .on_up_counting_timer_equals_zero(UpdateAction::SetLow)
    .on_up_counting_timer_equals_timestamp(UpdateAction::SetLow);

pub struct Motors<'d> {
    left_a: PwmPin<'d, MCPWM0<'d>, 0, true>,
    left_b: PwmPin<'d, MCPWM0<'d>, 0, false>,
    right_a: PwmPin<'d, MCPWM0<'d>, 1, true>,
    right_b: PwmPin<'d, MCPWM0<'d>, 1, false>,
    left_counter: Counter<'d, 0>,
    right_counter: Counter<'d, 1>,
    left_last: i16,
    right_last: i16,
}

impl<'d> Motors<'d> {
    /// Approximate wheel travel per encoder tick, assuming zero slip.
    /// Wheel diameter 40 mm, encoder 7 CPR, gear ratio 1:30, 2-channel counting.
    /// TODO: calibrate
    pub const METERS_PER_TICK: f64 = 40e-3 * PI / (7.0 * 2.0 * 30.0);

    pub fn new(
        mcpwm: MCPWM0<'d>,
        pcnt: PCNT<'d>,
        left: MotorChannelPins,
        right: MotorChannelPins,
    ) -> Self {
        let (left_a, left_b, right_a, right_b) =
            Self::init_pwm(mcpwm, left.pwm_a, left.pwm_b, right.pwm_a, right.pwm_b);

        let (left_counter, right_counter) =
            Self::init_encoders(pcnt, left.enc_a, left.enc_b, right.enc_a, right.enc_b);

        Self {
            left_a,
            left_b,
            right_a,
            right_b,
            left_counter,
            right_counter,
            left_last: 0,
            right_last: 0,
        }
    }

    /// Set PWM for both motors. Positive = forward, negative = backward, 0 = brake.
    pub fn set(&mut self, left: PwmT, right: PwmT) {
        apply_pwm(
            &mut self.left_a,
            &mut self.left_b,
            &mut self.left_last,
            left,
        );
        apply_pwm(
            &mut self.right_a,
            &mut self.right_b,
            &mut self.right_last,
            right,
        );
    }

    pub fn encoders(&self) -> (i16, i16) {
        (self.left_counter.get(), self.right_counter.get())
    }

    fn init_pwm(
        mcpwm: MCPWM0<'d>,
        left_a: impl PeripheralOutput<'d>,
        left_b: impl PeripheralOutput<'d>,
        right_a: impl PeripheralOutput<'d>,
        right_b: impl PeripheralOutput<'d>,
    ) -> (
        PwmPin<'d, MCPWM0<'d>, 0, true>,
        PwmPin<'d, MCPWM0<'d>, 0, false>,
        PwmPin<'d, MCPWM0<'d>, 1, true>,
        PwmPin<'d, MCPWM0<'d>, 1, false>,
    ) {
        let clock_cfg = PeripheralClockConfig::with_prescaler(0);
        let mut mcpwm = McPwm::new(mcpwm, clock_cfg);

        mcpwm.operator0.set_timer(&mcpwm.timer0);
        mcpwm.operator1.set_timer(&mcpwm.timer0);

        let (la, lb) = mcpwm.operator0.with_pins(
            left_a,
            PwmPinConfig::new(FORCE_LOW_A, PwmUpdateMethod::SYNC_ON_ZERO),
            left_b,
            PwmPinConfig::new(FORCE_LOW_B, PwmUpdateMethod::SYNC_ON_ZERO),
        );
        let (ra, rb) = mcpwm.operator1.with_pins(
            right_a,
            PwmPinConfig::new(FORCE_LOW_A, PwmUpdateMethod::SYNC_ON_ZERO),
            right_b,
            PwmPinConfig::new(FORCE_LOW_B, PwmUpdateMethod::SYNC_ON_ZERO),
        );

        let timer_cfg = clock_cfg
            .timer_clock_with_frequency(
                PwmT::MAX.get() as u16,
                PwmWorkingMode::Increase,
                TARGET_PWM_FREQ,
            )
            .expect("PWM frequency unachievable");
        log::info!("PWM frequency is {}", timer_cfg.frequency());
        mcpwm.timer0.start(timer_cfg);

        (la, lb, ra, rb)
    }

    fn init_encoders(
        pcnt: PCNT<'d>,
        left_enc_a: impl InputPin + 'd,
        left_enc_b: impl InputPin + 'd,
        right_enc_a: impl InputPin + 'd,
        right_enc_b: impl InputPin + 'd,
    ) -> (Counter<'d, 0>, Counter<'d, 1>) {
        let pcnt = Pcnt::new(pcnt);
        let enc_cfg = InputConfig::default().with_pull(Pull::Up);

        setup_encoder(&pcnt.unit0, enc_cfg, left_enc_a, left_enc_b);
        let left_counter = pcnt.unit0.counter.clone();

        setup_encoder(&pcnt.unit1, enc_cfg, right_enc_a, right_enc_b);
        let right_counter = pcnt.unit1.counter.clone();

        // pcnt and its units drop here; GenericPeripheralGuards inside the
        // cloned Counters keep the PCNT peripheral clock enabled.
        (left_counter, right_counter)
    }
}

fn setup_encoder<'d, const N: usize>(
    unit: &Unit<'d, N>,
    cfg: InputConfig,
    enc_a: impl InputPin + 'd,
    enc_b: impl InputPin + 'd,
) {
    unit.set_filter(Some(PCNT_FILTER)).unwrap();

    let pin_a = Input::new(enc_a, cfg);
    let pin_b = Input::new(enc_b, cfg);
    let sig_a = pin_a.peripheral_input();
    let sig_b = pin_b.peripheral_input();

    unit.channel0.set_edge_signal(sig_a.clone());
    unit.channel0.set_ctrl_signal(sig_b.clone());
    unit.channel0
        .set_input_mode(EdgeMode::Decrement, EdgeMode::Increment);
    unit.channel0
        .set_ctrl_mode(CtrlMode::Keep, CtrlMode::Reverse);

    unit.channel1.set_edge_signal(sig_b);
    unit.channel1.set_ctrl_signal(sig_a);
    unit.channel1
        .set_input_mode(EdgeMode::Decrement, EdgeMode::Increment);
    unit.channel1
        .set_ctrl_mode(CtrlMode::Reverse, CtrlMode::Keep);

    unit.clear();
    unit.resume();
}

// Drives one H-bridge channel.
// - Brake (pwm=0): force both outputs unconditionally LOW
// - Forward (pwm>0): force B low, let A be PWM-controlled.
// - Backward (pwm<0): force A low, let B be PWM-controlled.
fn apply_pwm<PWM, const OP: u8>(
    pin_a: &mut PwmPin<'_, PWM, OP, true>,
    pin_b: &mut PwmPin<'_, PWM, OP, false>,
    last: &mut i16,
    pwm: PwmT,
) where
    PWM: esp_hal::mcpwm::PwmPeripheral,
{
    let pwm = pwm.get();
    if pwm == *last {
        return;
    }
    let v = pwm.unsigned_abs();
    if pwm == 0 {
        pin_a.set_actions(FORCE_LOW_A);
        pin_b.set_actions(FORCE_LOW_B);
    } else if pwm > 0 {
        if *last <= 0 {
            // Coming from stop or reverse: release A to PWM, keep B forced low.
            pin_a.set_actions(PwmActions::UP_ACTIVE_HIGH);
            pin_b.set_actions(FORCE_LOW_B);
        }
        pin_a.set_timestamp(v);
    } else {
        if *last >= 0 {
            // Coming from stop or forward: force A low, release B to PWM.
            pin_a.set_actions(FORCE_LOW_A);
            pin_b.set_actions(PwmActions::UP_ACTIVE_HIGH);
        }
        pin_b.set_timestamp(v);
    }
    *last = pwm;
}
