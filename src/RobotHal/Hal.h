#pragma once

#include "LineSensor.h"
#include "Motors.h"
#include "Mpu6050.h"

#include <driver/adc.h>

#include <cstdint>
#include <cstddef>
#include <limits>
#include <array>

namespace RobotHal {

class Hal {
protected:
    Hal(); // Hal is only accessible through its singleton instance.
public:
    Hal(const Hal&) = delete;
    Hal(Hal&&) = delete;
    Hal& operator=(const Hal&) = delete;
    Hal& operator=(Hal&&) = delete;

    /// Return reference to the current HAL instance.
    static Hal& instance();

    enum class ButtonEvent: uint8_t {
        None,
        ShortPress,
        LongPress,
    };

    /// Enable or disable the blue LED on the module.
    void setBuiltinLed(bool enabled);

    int readRange();

    float readBatteryVoltage();

    ButtonEvent pollButton();

    LineSensor lineSensor;
    Motors motors;
    Mpu6050 imu;

protected:
    static constexpr auto adcWidth = ADC_WIDTH_BIT_12;
    static constexpr std::array<adc_atten_t, 3> adcAttenuations{
        ADC_ATTEN_DB_0, ADC_ATTEN_DB_6, ADC_ATTEN_DB_11
    };

    /// Two battery voltage measurements for battery voltage sensing calibration
    /// (raw value, correct indicated voltage)
    /// Uncomment a block inside Hal::readBatteryVoltage() to obtain the raw measurements.
    static constexpr std::array<std::pair<float, float>, 2> batteryVoltageCalibration = {
        std::make_pair(1050.0, 6.0f),
        std::make_pair(3207.3, 17.0f),
    };

    void setup();
    void setupMotors();
    void setupButtons();
    void setupI2C();
    void setupMisc();

    /// Read count consecutive 8bit registers from I2C device at deviceAddress,
    /// starting at registerAddress.
    /// Device address must be only 7 bits long, or the top bit will be ignored.
    static void i2cRead(
        uint8_t deviceAddress, uint8_t registerAddress,
        uint8_t* data, size_t count
    );

    static uint8_t i2cRead(uint8_t deviceAddress, uint8_t registerAddress) {
        uint8_t byte = 0xCF;
        i2cRead(deviceAddress, registerAddress, &byte, 1);
        return byte;
    }

    /// Write count consecutive 8bit registers to I2C device at deviceAddress,
    /// starting at registerAddress.
    /// Device address must be only 7 bits long, or the top bit will be ignored.
    static void i2cWrite(
        uint8_t deviceAddress, uint8_t registerAddress,
        const uint8_t* data, size_t count
    );

    static void i2cWrite(uint8_t deviceAddress, uint8_t registerAddress, uint8_t byte) {
        i2cWrite(deviceAddress, registerAddress, &byte, 1);
    }

    /// Read from two channels of an ADC at the same time
    static std::pair<int32_t, int32_t> readAdcPair(adc1_channel_t ch1, adc2_channel_t ch2);

    friend class LineSensor;
    friend class Mpu6050;
};

}
