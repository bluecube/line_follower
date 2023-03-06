#pragma once

#include "LineSensor.h"
#include "Motors.h"
#include "Mpu6050.h"
#include "Button.h"

#include <esp_adc/adc_oneshot.h>

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

    LineSensor lineSensor{*this};
    Motors motors;
    Mpu6050 imu;
    Button bootButton;
    Button deckButton;

protected:
    static constexpr adc_bitwidth_t adcWidth = ADC_BITWIDTH_12;

    /// Two battery voltage measurements for battery voltage sensing calibration
    /// (raw value, correct indicated voltage)
    /// Uncomment a block inside Hal::readBatteryVoltage() to obtain the raw measurements.
    static constexpr std::array<std::pair<float, float>, 2> batteryVoltageCalibration = {
        std::make_pair(1050.0, 6.0f),
        std::make_pair(3207.3, 17.0f),
    };

    void setup();
    void setupAdc();
    void setupMotors();
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

    std::array<adc_oneshot_unit_handle_t, 2> adcHandles;

    friend class LineSensor;
    friend class Mpu6050;
};

}
