#include <Arduino.h>
#include "LfHal.h"

LfHal hal;

bool led = false;
uint8_t sensorLed = 0;

void setup() {
    hal.setup();
}

void loop() {
    led = !led;
    hal.enableBuiltinLed(led);

    sensorLed = (sensorLed + 1) % LfHal::lineSensorLedCount;
    hal.enableSensorLed(sensorLed);

    delay(100);
}
