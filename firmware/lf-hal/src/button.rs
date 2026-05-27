use embassy_time::{Duration, Instant, Timer, with_deadline};
use esp_hal::gpio::{Input, InputConfig, InputPin, Pull};

const DEBOUNCE_TIME: Duration = Duration::from_millis(50);

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
            ButtonState::Released => self.pin.wait_for_low().await,
            ButtonState::Pressed { press_start } => {
                if let Some(d) = long_press_duration {
                    if with_deadline(press_start + d, self.pin.wait_for_high())
                        .await
                        .is_err()
                    {
                        self.state = ButtonState::LongPressEmitted;
                        return ButtonEvent::LongPress;
                    }
                } else {
                    self.pin.wait_for_high().await;
                }
            }
            ButtonState::LongPressEmitted => self.pin.wait_for_high().await,
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

    async fn wait_for_debounce(&mut self) {
        if let Some(t) = self.debounce_until {
            Timer::at(t).await;
            self.debounce_until = None;
        }
    }
}
