void setup() {
    for (int pin = 7; pin <= 11; ++pin)
    {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, HIGH);
    }
}

void loop() {
    for (int pin = 7; pin <= 11; ++pin)
    {
        digitalWrite(pin, LOW);
        delay(100);
        digitalWrite(pin, HIGH);
    }
    delay(100);
    for (int pin = 10; pin >= 7; --pin)
    {
        digitalWrite(pin, LOW);
        delay(100);
        digitalWrite(pin, HIGH);
    }
    Serial.println("Beep");
    delay(1000);
}
