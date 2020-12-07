#pragma once

#include <cstdint>
#include <limits>
#include <array>
#include <tuple>


class RobotHal
{
protected:
    enum PinT: uint8_t
    {
        motor0a = 3,
        motor0b = 4,
        motor1a = 5,
        motor1b = 6,
        lineSensorLedFirst = 7,
        lineSensorLedLast = 11,
        button = 12,
        builtinLed = 13,
        lineSensorLast = 14,
        lineSensorFirst = 17,
        i2cSda = 18,
        i2cScl = 19,
        motor1Current = 20,
        motor0Current = 21,
        batteryVoltage = 22,
        rangeSensor = 23,
    };

public:
    enum class ButtonEvent: uint8_t {
        None,
        ShortPress,
        LongPress,
    };

    using MillisecondsT = uint32_t;
    using MicrosecondsT = uint32_t;

    using PwmT = int16_t;
    static constexpr PwmT motorMaxValue = std::numeric_limits<PwmT>::max();
    static constexpr PwmT motorMinValue = -motorMaxValue;

    using LineSensorT = int;
    using LineSensorBufferT = std::array<LineSensorT, 8>;

    static constexpr uint8_t lineSensorLedCount = std::tuple_size<LineSensorBufferT>::value / 2 + 1;
    static_assert(PinT::lineSensorLedLast - PinT::lineSensorLedFirst == lineSensorLedCount - 1,
        "Line sensor LED pins must be consecutive and the exactly right count.");
    static constexpr uint8_t lineSensorCount = std::tuple_size<LineSensorBufferT>::value / 2;
    static_assert(lineSensorFirst - lineSensorLast == lineSensorCount - 1,
        "Line sensor pins must be reversely consecutive and the exactly right count.");

    /// Return reference to the current HAL instance.
    static RobotHal& instance();

    /// Set PWM signals for motors. Positive driving forward,
    /// rangege is from motorMinValue to motorMaxValue.
    /// Note that the actual hardware doesn't use the whole resolution of the inputs
    /// (expected actual resolution for 96Mhz CPU and ~20kHz PWM is 11bit).
    void setMotors(PwmT left, PwmT right);

    /// Enable given line sensor LED or disable all (for index out of range).
    /// Makes sure that only one is on at a time.
    void enableLineSensorLed(uint8_t ledIndex);

    /// Disable any sensor led currently enabled.
    void disableLineSensorLed();

    /// Enable or disable the Teensy builtin amber LED.
    void setBuiltinLed(bool enabled);

    int readRange();

    float readBatteryVoltage();

    /// Read the line sensor values into the buffer (using ambient light suppression)
    /// and return minimum and maximum value encountered.
    /// This call is relatively slow and uses the sensor LEDs.
    /// The LEDs end up all disabled after the call.
    std::tuple<LineSensorT, LineSensorT> readLineSensor(LineSensorBufferT& buffer);

    ButtonEvent pollButton();


protected:
    void setupMotorPWM();
    void setupLineSensor();
    void setupButton();
private:
    RobotHal(); // Hal is only accessible through its singleton instance.
    RobotHal(const RobotHal&) = delete;
    void operator=(const RobotHal&) = delete;
};
