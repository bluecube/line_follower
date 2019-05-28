#include "LfHal.h"

#include <algorithm>

#include <Arduino.h>

LfHal& LfHal::instance() {
    static LfHal instance;
    return instance;
}

LfHal::LfHal() {
    this->setupMotorPWM();
    this->setupLineSensor();

    pinMode(builtinLed, OUTPUT);
}

void LfHal::setupMotorPWM() {
    analogWriteRes(std::numeric_limits<PwmT>::digits);
    for (auto pin: {motor0a, motor0b, motor1a, motor1b}) {
        pinMode(pin, OUTPUT);
        analogWriteFrequency(pin, motorPWMFrequency);
    }
}

void LfHal::setupLineSensor() {
    for (uint8_t pin = lineSensorLedFirst; pin <= lineSensorLedLast; ++pin) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, HIGH); // line leds are active low.
    }
    enabledLineSensorLed = lineSensorLedFirst;
}

void LfHal::setSingleMotor(PinT pinA, PinT pinB, PwmT value) {
    if (value > 0) {
        analogWrite(pinA, value);
        digitalWrite(pinB, LOW);
    } else {
        digitalWrite(pinA, LOW);
        analogWrite(pinB, -value);
    }
}

void LfHal::setMotors(PwmT left, PwmT right) {
    setSingleMotor(motor0a, motor0b, left);
    setSingleMotor(motor1a, motor1b, right);
}

void LfHal::enableLineSensorLed(uint8_t ledIndex) {
    this->disableLineSensorLed();
    if (ledIndex < lineSensorLedCount) {
        this->enabledLineSensorLed = static_cast<PinT>(lineSensorLedFirst + ledIndex);
        digitalWrite(this->enabledLineSensorLed, LOW);
    }
}

void LfHal::disableLineSensorLed() {
    digitalWrite(this->enabledLineSensorLed, HIGH);
}

void LfHal::setBuiltinLed(bool enable) {
    digitalWrite(builtinLed, enable ? HIGH : LOW);
}

int LfHal::readRange() {
    return analogRead(rangeSensor);
}

std::tuple<LfHal::LineSensorT, LfHal::LineSensorT> LfHal::readLineSensor(LineSensorBufferT& buffer) {
    // TODO: Calibration
    // TODO: Make this asynchronous

    static_assert(std::tuple_size<LineSensorBufferT>::value == 8,
        "Must have exactly 8 pixels on the line sensor.");
    static_assert(lineSensorLedCount == 5,
        "Must have exactly 5 LEDs on the line sensor.");
    static_assert(lineSensorCount == 4,
        "Must have exactly 4 phototransistors on the line sensor.");

    for (uint8_t i = 0; i < LfHal::lineSensorLedCount; ++i)
    {
        this->enableLineSensorLed(i);
        delayMicroseconds(lineSensorLedDelay); // Wait a bit for the LED to actually turn on.

        if (i == 2)
        {
            // Sensors 1 and 2 can be read in parallel
            auto result = adc.analogSyncRead(lineSensorFirst - 2, lineSensorFirst - 1);
            buffer[4] = result.result_adc0;
            buffer[3] = result.result_adc1;
        }
        else
        {
            if (i > 0)
                buffer[2 * i - 1] = adc.analogRead(lineSensorFirst - i + 1);
            if (i < LfHal::lineSensorCount)
                buffer[2 * i] = adc.analogRead(lineSensorFirst - i);
        }
    }

    // Ambient light suppression
    this->disableLineSensorLed();
    delayMicroseconds(lineSensorLedDelay); // Wait a bit for the LED to actually turn off.

    auto minValue = std::numeric_limits<LineSensorT>::max();
    auto maxValue = std::numeric_limits<LineSensorT>::min();

    for (uint8_t i = 0; i < 2; ++i)
    {
        auto result = adc.analogSyncRead(lineSensorFirst - 2 - i, lineSensorFirst - i);
        for (uint8_t j = 0; j < 2; ++j)
        {
            buffer[2 * i + j + 4] -= result.result_adc0;
            minValue = std::min(buffer[2 * i + j + 4], minValue);
            maxValue = std::max(buffer[2 * i + j + 4], maxValue);

            buffer[2 * i + j] -= result.result_adc1;
            minValue = std::min(buffer[2 * i + j], minValue);
            maxValue = std::max(buffer[2 * i + j], maxValue);
        }
    }

    return std::make_tuple(minValue, maxValue);
}
