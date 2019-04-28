#include <Arduino.h>
#include "LfHal.h"

LfHal hal;

bool led = false;

void setup() {
    hal.setup();
    Serial.begin(9600);

    while (!Serial) {
      hal.enableBuiltinLed(true);
      delay(50);
      hal.enableBuiltinLed(false);
      delay(50);
    }
}

void loop() {
    led = !led;
    hal.enableBuiltinLed(led);

    auto range = hal.readRange();
    hal.enableSensorLed(map(range, 0, 1024, 0, LfHal::lineSensorLedCount - 1));

    Serial.println(hal.readLineSensor(0));

    delay(500);
}
