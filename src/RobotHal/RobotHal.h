#pragma once

#include "idf_util.h"

#include <driver/adc.h>

#include <cstdint>
#include <limits>
#include <array>
#include <tuple>

class RobotHal {
protected:
    using PinT = IdfUtil::PinT;

    // Board pins, manually copied from the schematic
    struct Pins {
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

        // How many line leds there are after charlieplexing
        static constexpr unsigned lineLedCount = lineLed.size() * (lineLed.size() - 1);
    };

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

    using PwmT = int16_t;
    static constexpr PwmT motorMaxValue = std::numeric_limits<PwmT>::max();
    static constexpr PwmT motorMinValue = -motorMaxValue;

    using LineSensorT = int32_t;
    using LineSensorBufferT = std::array<LineSensorT, Pins::lineSensor.size() * 2>; // Each line sensor element can be read from two sides

    /// Set PWM signals for motors. Positive driving forward,
    /// rangege is from motorMinValue to motorMaxValue.
    /// Note that the actual hardware doesn't use the whole resolution of the inputs
    /// (expected actual resolution for 96Mhz CPU and ~20kHz PWM is 11bit).
    void setMotors(PwmT left, PwmT right);

    std::pair<int16_t, int16_t> readMotorEncoders() const;

    /// Enable given line sensor LED or disable all (for index out of range).
    /// Makes sure that only one is on at a time.
    void enableLineSensorLed(uint32_t ledIndex);

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
    static constexpr auto adcWidth = ADC_WIDTH_BIT_12;

    void setupMotors();
    void setupLineSensor();
    void setupButtons();
    void setupIMU();
    void setupMisc();

    /// Sleep for some time to allow the line sensor to settle after LED change.
    void lineSensorSettle();

    /// Read from two channels of an ADC at the same time
    static std::pair<int32_t, int32_t> readAdcPair(adc1_channel_t ch1, adc2_channel_t ch2);

    /// Wrapper around readAdcPair that takes index of a line sensor element instead of ADC channel.
    /// The sensors must actually belong connected to channel 1 and channel 2 of the ADC!
    static std::pair<LineSensorT, LineSensorT> readAdcLineSensorPair(int ch1Sensor, int ch2Sensor);

    /// Wrapper around adc1_get_raw that takes index of a line sensor element instead of ADC channel.
    /// The sensor must actually belong connected to channel 1 of the ADC!
    static LineSensorT readAdc1LineSensor(int ch1Sensor);

    void setLineSensorAttenuation(adc_atten_t attenuation);

    /// Read line sensor on all positions with LEDs.
    /// Two readings per sensor, reading pairs of values at a time.
    template <typename LedFn, typename OutputFn>
    static inline void readLineSensor(LedFn ledFn, OutputFn outputFn);

    /// Read line sensor on all positions without LEDs.
    /// Compared to readLineSensor(ledFn, outputFn) this joins the two readings
    /// per sensor into one, making this function twice as fast.
    template <typename OutputFn>
    static inline void readLineSensor(OutputFn outputFn);
};

