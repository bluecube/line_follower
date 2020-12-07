#pragma once

#include <cstdint>
#include <limits>
#include <array>
#include <tuple>


class RobotHal
{
protected:
    RobotHal(); // Hal is only accessible through its singleton instance.
public:
    RobotHal(const RobotHal&) = delete;
    RobotHal(RobotHal&&) = delete;
    RobotHal& operator=(const RobotHal&) = delete;
    RobotHal& operator=(RobotHal&&) = delete;

    /// Return reference to the current HAL instance.
    static RobotHal& instance();

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
    using PinT = int; // Int to match the ESP-IDF function signatures.

    // Board pins, manually copied from the schematic
    struct Pins
    {
        static constexpr std::array<std::pair<PinT, PinT>, 2> motor = {
            std::make_pair(13, 4),
            std::make_pair(23, 19)
        };
        static constexpr std::array<std::pair<PinT, PinT>, 2> encoder = {
            std::make_pair(16, 17),
            std::make_pair(36, 39)
        };
        static constexpr std::array<PinT, 5> lineSensor = {33, 14, 35, 25, 34};
        static constexpr std::array<PinT, 3> lineLed = {27, 32, 26};
        static constexpr PinT range = 12;
        static constexpr PinT scl = 22;
        static constexpr PinT sda = 21;
        static constexpr PinT accel_PinT = 18;
        static constexpr PinT mainBbutton = 5;
        static constexpr PinT bootBbutton = 0;
        static constexpr PinT batSense = 15;
        static constexpr PinT indicatorLed = 2;
    };

    void setupMotorPWM();
    void setupLineSensor();
    void setupButton();
};
