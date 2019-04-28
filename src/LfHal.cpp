#include "LfHal.h"

#include "ADC.h"

void LfHal::setup() {
    this->setupMotorPWM();

    pinMode(builtinLed, OUTPUT);
}

void LfHal::setupMotorPWM() {
    analogWriteRes(std::numeric_limits<PwmT>::digits);
    for (auto pin: {motor0a, motor0b, motor1a, motor1b})
    {
        pinMode(pin, OUTPUT);
        analogWriteFrequency(pin, motorPWMFrequency);
    }
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
