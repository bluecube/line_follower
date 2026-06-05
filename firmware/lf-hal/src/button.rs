use embassy_time::{Duration, Instant, Timer, WithTimeout as _};
use esp_hal::gpio::{Input, InputConfig, InputPin, Pull};

/// An edge that reverses within this window is treated as bounce or EMI and discarded.
const EMI_FILTER_TIME: Duration = Duration::from_millis(5);
/// Additional time after EMI_FILTER_TIME required to register an opposite action
const DEBOUNCE_TIME: Duration = Duration::from_millis(10);

pub enum ButtonEvent {
    Press,
    LongPress,
    Release,
    /// Emitted instead of `Release` when the press already produced a `LongPress`.
    SuppressedRelease,
}

enum ButtonState {
    Released,
    Pressed { press_start: Instant },
    LongPressEmitted,
}

pub struct Button<'d> {
    pin: Input<'d>,
    debounce_until: Option<Instant>,
    state: ButtonState,
}

impl<'d> Button<'d> {
    pub fn new(pin: impl InputPin + 'd) -> Self {
        let pin = Input::new(pin, InputConfig::default().with_pull(Pull::Up));
        let state = if pin.is_low() {
            ButtonState::Pressed {
                press_start: Instant::now(),
            }
        } else {
            ButtonState::Released
        };
        Self {
            pin,
            debounce_until: None,
            state,
        }
    }

    /// Waits for and returns the next button event.
    ///
    /// Events normally alternate `Press`, `Release`. If `long_press_duration` is `Some`, emits
    /// `LongPress` once the button has been held for that long; the following
    /// release is then `SuppressedRelease` instead of `Release`. If the button was already held
    /// when [`Button::new`] ran, the first event is the release (no preceding `Press`).
    pub async fn next_event(&mut self, long_press_duration: Option<Duration>) -> ButtonEvent {
        self.wait_for_debounce().await;

        match self.state {
            ButtonState::Released => self.wait_for_stable_low().await,
            ButtonState::Pressed { press_start } => {
                let deadline = long_press_duration.map(|d| press_start + d);
                if !self.wait_for_stable_high(deadline).await {
                    self.state = ButtonState::LongPressEmitted;
                    return ButtonEvent::LongPress;
                }
            }
            ButtonState::LongPressEmitted => {
                self.wait_for_stable_high(None).await;
            }
        }

        let now = Instant::now();
        self.debounce_until = Some(now + DEBOUNCE_TIME);

        match self.state {
            ButtonState::Released => {
                self.state = ButtonState::Pressed { press_start: now };
                ButtonEvent::Press
            }
            ButtonState::Pressed { .. } => {
                self.state = ButtonState::Released;
                ButtonEvent::Release
            }
            ButtonState::LongPressEmitted => {
                self.state = ButtonState::Released;
                ButtonEvent::SuppressedRelease
            }
        }
    }

    pub async fn pressed(&mut self) {
        loop {
            if let ButtonEvent::Press = self.next_event(None).await {
                return;
            }
        }
    }

    pub async fn released(&mut self) {
        debug_assert!(
            !matches!(self.state, ButtonState::LongPressEmitted),
            "released() called while button is in LongPressEmitted state; use next_event() directly"
        );
        loop {
            if let ButtonEvent::Release = self.next_event(None).await {
                return;
            }
        }
    }

    /// Waits for a stable low (press). A falling edge followed by a rising edge within
    /// `EMI_FILTER_TIME` is treated as bounce or EMI and ignored.
    async fn wait_for_stable_low(&mut self) {
        loop {
            self.pin.wait_for_low().await;
            if self
                .pin
                .wait_for_high()
                .with_timeout(EMI_FILTER_TIME)
                .await
                .is_err()
            {
                // Timed out without pin switching state, this counts as a stable low.
                return;
            }
        }
    }

    /// Waits for a stable high (release). A rising edge followed by a falling edge within
    /// `EMI_FILTER_TIME` is treated as bounce or EMI and ignored. Returns `true` if a
    /// stable release was confirmed, `false` if `deadline` expired first.
    async fn wait_for_stable_high(&mut self, deadline: Option<Instant>) -> bool {
        loop {
            match deadline {
                Some(deadline) => {
                    if self
                        .pin
                        .wait_for_high()
                        .with_deadline(deadline)
                        .await
                        .is_err()
                    {
                        // Timed out waiting for `deadline`
                        return false;
                    }
                }
                None => self.pin.wait_for_high().await,
            }
            if self
                .pin
                .wait_for_low()
                .with_timeout(EMI_FILTER_TIME)
                .await
                .is_err()
            {
                // Timed out without pin switching state, this counts as a stable high.
                return true;
            }
            // If `deadline` ran out during waiting for EMI_FILTER_TIME, we handle it in the next loop iteration.
        }
    }

    async fn wait_for_debounce(&mut self) {
        if let Some(t) = self.debounce_until {
            Timer::at(t).await;
            self.debounce_until = None;
        }
    }
}
