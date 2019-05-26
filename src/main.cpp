#include <Arduino.h>
#include "LfHal.h"

bool led = false;

void setup() {
    Serial.begin(9600);

    while (!Serial) {
      LfHal::instance().setBuiltinLed(true);
      delay(50);
      LfHal::instance().setBuiltinLed(false);
      delay(50);
    }
}

void loop() {
    LfHal::instance().setBuiltinLed(led);
    led = !led;

    delay(2000);

    LfHal::LineSensorBufferT buffer;
    LfHal::instance().readLineSensor(buffer);
    for (uint8_t i = 0; i < 8; ++i)
    {
        Serial.print(" ");
        Serial.print(buffer[i]);
    }

    Serial.println();
}
