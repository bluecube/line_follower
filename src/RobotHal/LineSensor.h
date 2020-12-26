#pragma once

#include "Pins.h"

#include <driver/adc.h>

#include <array>
#include <cstdint>

namespace RobotHal {

class Hal;

class LineSensor {
protected:
    LineSensor() {} // line sensor is only accessible through HAL
public:
    using ValueT = int32_t;
    using BufferT = std::array<ValueT, 10>;

    LineSensor(const LineSensor&) = delete;
    LineSensor(LineSensor&&) = delete;
    LineSensor& operator=(const LineSensor&) = delete;
    LineSensor& operator=(LineSensor&&) = delete;

    /// Enable given line sensor LED or disable all (for index out of range).
    /// Makes sure that only one is on at a time.
    void enableLed(uint32_t ledIndex);

    /// Disable any sensor led currently enabled.
    void disableLed();

    /// Read the line sensor values into the buffer (using ambient light suppression)
    /// and return minimum and maximum value encountered.
    /// This call is relatively slow and uses the sensor LEDs.
    /// The LEDs end up all disabled after the call.
    std::pair<ValueT, ValueT> read(BufferT& buffer);

    void calibrate();

protected:
    // How many line leds there are after charlieplexing
    static constexpr unsigned lineLedCount =
        Pins::lineLed.size() * (Pins::lineLed.size() - 1);

    void setup();

    /// Sleep for some time to allow the line sensor to settle after LED change.
    void settle();

    /// Wrapper around readAdcPair that takes index of a line sensor element instead of ADC channel.
    /// The sensors must actually belong connected to channel 1 and channel 2 of the ADC!
    static std::pair<ValueT, ValueT> readAdcPair(int ch1Sensor, int ch2Sensor);

    /// Wrapper around adc1_get_raw that takes index of a line sensor element instead of ADC channel.
    /// The sensor must actually belong connected to channel 1 of the ADC!
    static ValueT readAdc1(int ch1Sensor);

    void setAttenuation(adc_atten_t attenuation);

    /// Read line sensor on all positions with LEDs.
    /// Two readings per sensor, reading pairs of values at a time.
    template <typename LedFn, typename OutputFn>
    static inline void read(LedFn ledFn, OutputFn outputFn);

    /// Read line sensor on all positions without LEDs.
    /// Compared to readLineSensor(ledFn, outputFn) this joins the two readings
    /// per sensor into one, making this function twice as fast.
    template <typename OutputFn>
    static inline void read(OutputFn outputFn);

    friend class Hal;
};

}
