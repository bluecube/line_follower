#pragma once

#include "Pins.h"

#include <array>
#include <cstdint>

namespace RobotHal {

class Hal;

class LineSensor {
protected:
    LineSensor(Hal& hal): hal(hal) {} // line sensor is only accessible through HAL
public:
    using ValueT = int16_t;
    using BufferT = std::array<ValueT, 10>;

    // How many line leds there are after charlieplexing
    static constexpr unsigned lineLedCount =
        Pins::lineLed.size() * (Pins::lineLed.size() - 1);

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
    Hal& hal;

    void setup();

    /// Sleep for some time to allow the line sensor to settle after LED change.
    void settle();

    /// Wrapper around readAdcPair that takes index of a line sensor element instead of ADC channel.
    /// The sensors must actually belong connected to channel 1 and channel 2 of the ADC!
    std::pair<ValueT, ValueT> readAdcPair(int ch1Sensor, int ch2Sensor);

    /// Reads a value from a single ADC.
    ValueT readAdc(int channel);

    /// Read line sensor on all positions, while blinking LED patterns.
    /// @param ledFn ledFn(i) is called to enable LED i (and disable others).
    /// @param outputFn outputFn(i, value) is a callback to set the output.
    ///     Range of i is to BufferT::size().
    template <typename LedFn, typename OutputFn>
    inline void read_leds(LedFn ledFn, OutputFn outputFn);

    /// Read line sensor on all positions without changing LEDs.
    template <typename OutputFn>
    /// @param outputFn outputFn(i, value) is a callback to set the output.
    ///     Range of i is to BufferT::size().
    inline void read_no_leds(OutputFn outputFn);

    friend class Hal;
};

}
