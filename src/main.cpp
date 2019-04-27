#include <Arduino.h>

void setup() {
    for (int pin = 7; pin <= 11; ++pin)
    {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, HIGH);
    }
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
    for (int pin = 7; pin <= 11; ++pin)
    {
        digitalWrite(pin, LOW);
        delay(100);
        digitalWrite(pin, HIGH);
    }
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    for (int pin = 10; pin >= 7; --pin)
    {
        digitalWrite(pin, LOW);
        delay(100);
        digitalWrite(pin, HIGH);
    }
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("Beep");
}
