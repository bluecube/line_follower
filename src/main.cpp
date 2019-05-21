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

void readLineSensor(LfHal& hal, int (&buffer)[8])
{
    // TODO: Put this to some more reasonable space.
    // TODO: Restructure the constants in LfHal to contain output buffer size, move the static asserts to this definition.
    // TODO: Calibration
    // TODO: Make the ADC reads asynchronous
    // TODO: Use parallel reading with the two ADCs

    static constexpr int ledDelay = 1;

    // First disable sensor LEDS and do ambient light suppression
    hal.enableSensorLed(LfHal::lineSensorLedDisabled);
    delay(ledDelay); // Wait a bit for the LED to actually turn off. // TODO: Is this necessary?
    static_assert(sizeof(buffer) / sizeof(buffer[0]) == 2 * LfHal::lineSensorCount,
        "Each line sensor element must have two corresponding buffer fields.");
    for (uint8_t i = 0; i < LfHal::lineSensorCount; ++i)
    {
        int measurement = hal.readLineSensor(i);
        Serial.println(measurement);
        buffer[2 * i] = -measurement;
        buffer[2 * i + 1] = -measurement;
    }

    static_assert(LfHal::lineSensorCount == LfHal::lineSensorLedCount - 1,
        "There must be exactly one less line sensor elements than line sensor LEDs");
    for (uint8_t i = 0; i < LfHal::lineSensorLedCount; ++i)
    {
        hal.enableSensorLed(i);
        delay(ledDelay); // Wait a bit for the LED to actually turn off. // TODO: Is this necessary?

        if (i > 0)
        {
            buffer[2 * i - 1] += hal.readLineSensor(i - 1);
        }
        if (i < LfHal::lineSensorCount)
        {
            buffer[2 * i] += hal.readLineSensor(i);
        }
    }
    hal.enableSensorLed(LfHal::lineSensorLedDisabled);
}

void loop() {
    hal.enableBuiltinLed(led);
    led = !led;

    delay(2000);

    int buffer[8];
    readLineSensor(hal, buffer);
    for (uint8_t i = 0; i < 8; ++i)
    {
        Serial.print(" ");
        Serial.print(buffer[i]);
    }

    Serial.println();
}
