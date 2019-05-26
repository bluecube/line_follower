#include <Arduino.h>
#include "LfHal.h"

LfHal hal;

bool led = false;

void setup() {
    hal.setup();
    Serial.begin(9600);

    while (!Serial) {
      hal.setBuiltinLed(true);
      delay(50);
      hal.setBuiltinLed(false);
      delay(50);
    }
}

void loop() {
    hal.setBuiltinLed(led);
    led = !led;

    delay(2000);

    LfHal::LineSensorBufferT buffer;
    hal.readLineSensor(buffer);
    for (uint8_t i = 0; i < 8; ++i)
    {
        Serial.print(" ");
        Serial.print(buffer[i]);
    }

    Serial.println();
}
