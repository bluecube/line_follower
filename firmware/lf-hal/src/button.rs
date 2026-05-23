use embassy_time::{Duration, Instant};
use esp_hal::gpio::{Input, InputConfig, InputPin, Pull};

const DEBOUNCE_TIME: Duration = Duration::from_millis(50);

pub enum ButtonEvent {
    Press,
    LongPress,
    /// Not emitted when the press already produced a `LongPress`.
    Release(Duration),
}

enum PressState {
    Released,
    Pressed {
        start: Instant,
        emitted_long_press: bool,
    },
}

pub struct Button<'d> {
    pin: Input<'d>,
    /// Debounced state.
    state: PressState,
    /// Time when debounce interval will end or None if outside debounce interval
    debounce_until: Option<Instant>,
}

impl<'d> Button<'d> {
    pub fn new(pin: impl InputPin + 'd) -> Self {
        Self {
            pin: Input::new(pin, InputConfig::default().with_pull(Pull::Up)),
            state: PressState::Released,
            debounce_until: None,
        }
    }

    /// Advance the debounce state machine. Call regularly (e.g. each control
    /// loop iteration). Returns an event when a transition is confirmed.
    pub fn poll(&mut self) -> Option<ButtonEvent> {
        self.poll_internal(None)
    }

    /// Like `poll`, but also emits `LongPress` once the button has been held
    /// for at least `threshold`. The `Release` for that press is suppressed.
    pub fn poll_with_threshold(&mut self, threshold: Duration) -> Option<ButtonEvent> {
        self.poll_internal(Some(threshold))
    }

    fn poll_internal(&mut self, long_press_threshold: Option<Duration>) -> Option<ButtonEvent> {
        let raw = self.pin.is_low();
        let now = Instant::now();

        let debouncing = self
            .debounce_until
            .map(|until| now < until)
            .unwrap_or_default();

        if debouncing {
            return None;
        }

        if let PressState::Pressed {
            start: press_start,
            emitted_long_press,
        } = self.state
        {
            if raw {
                // Pressed, no change
                if !emitted_long_press
                    && long_press_threshold
                        .map(|threshold| now > press_start + threshold)
                        .unwrap_or_default()
                {
                    self.state = PressState::Pressed {
                        start: press_start,
                        emitted_long_press: true,
                    };
                    Some(ButtonEvent::LongPress)
                } else {
                    None
                }
            } else {
                // Just released
                self.debounce_until = Some(now + DEBOUNCE_TIME);
                self.state = PressState::Released;
                if emitted_long_press {
                    None
                } else {
                    Some(ButtonEvent::Release(now - press_start))
                }
            }
        } else if raw {
            // Just pressed
            self.debounce_until = Some(now + DEBOUNCE_TIME);
            self.state = PressState::Pressed {
                start: now,
                emitted_long_press: false,
            };
            Some(ButtonEvent::Press)
        } else {
            // Not pressed, no change
            None
        }
    }
}
