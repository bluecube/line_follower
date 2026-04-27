use esp_hal::gpio::{AnyPin, Flex};

// Maps LED index to (low_pin, high_pin, hiz_pin) indices into self.pins.
// Pins order: [0=GPIO27, 1=GPIO32, 2=GPIO26]
const LED_PATTERNS: [(usize, usize, usize); 6] = [
    (0, 1, 2),
    (1, 0, 2),
    (0, 2, 1),
    (2, 0, 1),
    (2, 1, 0),
    (1, 2, 0),
];

pub struct LineSensor<'d> {
    pins: [Flex<'d>; 3],
}

impl<'d> LineSensor<'d> {
    pub fn new(led_pins: [AnyPin<'d>; 3]) -> Self {
        let [p0, p1, p2] = led_pins;
        Self {
            pins: [Flex::new(p0), Flex::new(p1), Flex::new(p2)],
        }
    }

    /// Enable the LED at `index` (0–5) by setting the charlieplex pattern.
    pub fn enable_led(&mut self, index: usize) {
        let (low, high, hiz) = LED_PATTERNS[index];
        self.pins[hiz].set_output_enable(false);
        self.pins[low].set_low();
        self.pins[low].set_output_enable(true);
        self.pins[high].set_high();
        self.pins[high].set_output_enable(true);
    }

    /// Disable all LEDs by floating all three pins.
    pub fn disable_leds(&mut self) {
        for pin in &mut self.pins {
            pin.set_output_enable(false);
        }
    }
}
