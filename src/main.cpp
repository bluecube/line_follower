#include <Arduino.h>
#include "LfHal.h"

LfHal hal;

bool led = false;

void setup() {
    hal.setup();
}

void loop() {
    led = !led;
    digitalWrite(LfHal::builtinLed, led ? HIGH : LOW);

    delay(100);
}
