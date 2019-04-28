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

    static constexpr uint8_t lineSensorLedCount = lineLedLast - lineLedFirst + 1;
    static constexpr uint8_t lineSensorLedDisabled = ~0;

    LfHal() = default;

    void setup();

    /// Set PWM signals for motors. Positive driving forward,
    /// rangege is from motorMinValue to motorMaxValue.
    /// Note that the actual hardware doesn't use the whole resolution of the inputs
    /// (expected actual resolution for 96Mhz CPU and ~20kHz PWM is 11bit).
    void setMotors(PwmT left, PwmT right);

    /// Enable given line sensor LED or disable all.
    /// Makes sure that only one is on at a time.
    /// Range 0 - lineSensorLedCount, or lineSensorLedDisabled to disable.
    void enableSensorLed(uint8_t ledIndex);

    /// Enable or disable the Teensy builtin amber LED.
    void enableBuiltinLed(bool enabled);

protected:
    void setupMotorPWM();
    void setupLineSensor();

    /// Helper function to set pwm output for a single motor, handlnig
    /// reversing correctly.
    void setSingleMotor(PinT pinA, PinT pinB, PwmT value);

    /// Line sensor led pin that is currently enabled, or any of the line sensor
    /// led pins if none is enabled.
    PinT enabledLineLed;

private:
    LfHal(const LfHal&) = delete;
    void operator=(const LfHal&) = delete;
};
