#pragma once

#include "RobotHalPins.h"
#include "RobotHalLineSensor.h"
#include "RobotHalImu.h"

#include <driver/adc.h>

#include <cstdint>
#include <cstddef>
#include <limits>
#include <array>

class RobotHal {
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

    using PwmT = int16_t;
    static constexpr PwmT motorMaxValue = std::numeric_limits<PwmT>::max();
    static constexpr PwmT motorMinValue = -motorMaxValue;

    /// Set PWM signals for motors. Positive driving forward,
    /// rangege is from motorMinValue to motorMaxValue.
    /// Note that the actual hardware doesn't use the whole resolution of the inputs
    /// (expected actual resolution for 96Mhz CPU and ~20kHz PWM is 11bit).
    void setMotors(PwmT left, PwmT right);

    std::pair<int16_t, int16_t> readMotorEncoders() const;

    /// Enable or disable the blue LED on the module.
    void setBuiltinLed(bool enabled);

    int readRange();

    float readBatteryVoltage();

    ButtonEvent pollButton();

    RobotHalLineSensor lineSensor;
    RobotHalImu imu;

protected:
    static constexpr auto adcWidth = ADC_WIDTH_BIT_12;
    static constexpr std::array<adc_atten_t, 3> adcAttenuations{
        ADC_ATTEN_DB_0, ADC_ATTEN_DB_6, ADC_ATTEN_DB_11
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

    friend class RobotHalLineSensor;
    friend class RobotHalImu;
};

