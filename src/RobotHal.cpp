#include "RobotHal.h"

#include <algorithm>

#include <Arduino.h>

RobotHal& RobotHal::instance() {
    static RobotHal instance;
    return instance;
}

RobotHal::RobotHal() {
    this->setupMotorPWM();
    this->setupLineSensor();
    this->setupButton();

    pinMode(builtinLed, OUTPUT);
}

void RobotHal::setupMotorPWM() {
    analogWriteRes(std::numeric_limits<PwmT>::digits);
    for (auto pin: {motor0a, motor0b, motor1a, motor1b}) {
        pinMode(pin, OUTPUT);
        analogWriteFrequency(pin, motorPWMFrequency);
    }
}

void RobotHal::setupLineSensor() {
    for (uint8_t pin = lineSensorLedFirst; pin <= lineSensorLedLast; ++pin) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, HIGH); // line leds are active low.
    }
}

void RobotHal::setupButton() {
    pinMode(button, INPUT_PULLUP);
    delay(buttonDebounceDelay);
    this->buttonState = !digitalRead(button);
    this->buttonStateChangeTime = millis();
}

void RobotHal::setSingleMotor(PinT pinA, PinT pinB, PwmT value) {
    if (value > 0) {
        analogWrite(pinA, value);
        digitalWrite(pinB, LOW);
    } else {
        digitalWrite(pinA, LOW);
        analogWrite(pinB, -value);
    }
}

void RobotHal::setMotors(PwmT left, PwmT right) {
    setSingleMotor(motor0a, motor0b, left);
    setSingleMotor(motor1a, motor1b, right);
}

void RobotHal::enableLineSensorLed(uint8_t ledIndex) {
    this->disableLineSensorLed();
    if (ledIndex < lineSensorLedCount) {
        this->enabledLineSensorLed = static_cast<PinT>(lineSensorLedFirst + ledIndex);
        digitalWrite(this->enabledLineSensorLed, LOW);
    }
}

void RobotHal::disableLineSensorLed() {
    digitalWrite(this->enabledLineSensorLed, HIGH);
}

void RobotHal::setBuiltinLed(bool enable) {
    digitalWrite(builtinLed, enable ? HIGH : LOW);
}

int RobotHal::readRange() {
    return analogRead(rangeSensor);
}

std::tuple<RobotHal::LineSensorT, RobotHal::LineSensorT> RobotHal::readLineSensor(LineSensorBufferT& buffer) {
    // TODO: Calibration
    // TODO: Make this asynchronous

    static_assert(std::tuple_size<LineSensorBufferT>::value == 8,
        "Must have exactly 8 pixels on the line sensor.");
    static_assert(lineSensorLedCount == 5,
        "Must have exactly 5 LEDs on the line sensor.");
    static_assert(lineSensorCount == 4,
        "Must have exactly 4 phototransistors on the line sensor.");

    for (uint8_t i = 0; i < RobotHal::lineSensorLedCount; ++i)
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
            if (i < RobotHal::lineSensorCount)
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

RobotHal::ButtonEvent RobotHal::pollButton() {
    bool currentState = !digitalRead(button); // Active low
    uint32_t now = millis();

    if (currentState == this->buttonState)
        return ButtonEvent::None; // No change

    uint32_t elapsed = now - this->buttonStateChangeTime; // TODO: This overflows after 49 days, it's not an issue for this application though.

    if (elapsed < buttonDebounceDelay)
        return ButtonEvent::None; // Change within debounce interval

    this->buttonState = currentState;
    this->buttonStateChangeTime = now;

    if (currentState)
        return ButtonEvent::None; // We only report releasing events, not pressing

    if (elapsed > buttonLongPressDelay)
        return ButtonEvent::LongPress;
    else
        return ButtonEvent::ShortPress;
}
