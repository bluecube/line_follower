#pragma once

#include <cstdint>
#include <limits>

class LfHal
{
public:
    // Pin mappings:
    using PinT = uint8_t;
    static constexpr PinT motor0a = 3;
    static constexpr PinT motor0b = 4;
    static constexpr PinT motor1a = 5;
    static constexpr PinT motor1b = 6;
    static constexpr PinT lineLedFirst = 7;
    static constexpr PinT lineLedLast = 11;
    static constexpr PinT button = 12;
    static constexpr PinT builtinLed = 13;
    static constexpr PinT lineSensorLast = 14;
    static constexpr PinT lineSensorFirst = 17;
    static constexpr PinT i2cSda = 18;
    static constexpr PinT i2cScl = 19;
    static constexpr PinT motor1Current = 20;
    static constexpr PinT motor0Current = 21;
    static constexpr PinT batteryVoltage = 22;
    static constexpr PinT range = 23;

    // Motor PWM settings
    using PwmT = int16_t;
    static constexpr PwmT motorMaxValue = std::numeric_limits<PwmT>::max();
    static constexpr PwmT motorMinValue = -motorMaxValue;
    static constexpr float motorPWMFrequency = 20e3; // 20kHz to keep things quiet

    LfHal() = default;

    void setup();

    void setMotors(PwmT left, PwmT right);

protected:
    void setupMotorPWM();

    void setSingleMotor(PinT pinA, PinT pinB, PwmT value);

private:
    LfHal(const LfHal&) = delete;
    void operator=(const LfHal&) = delete;
};
