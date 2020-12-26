#include "RobotHalImu.h"

#include "RobotHal.h"

#include <driver/i2c.h>

#include <cstdio>

void RobotHalImu::setup() {
    printf("Setting up IMU\n");

    // Register 25:
    RobotHal::i2cWrite(i2cAddress, 25,
        1000 / 100 - 1 // Set sample rate to 100Hz
    );

    // Register 26:
    RobotHal::i2cWrite(i2cAddress, 26,
        0 << 3 | // Disable external frame synchronization
        2 << 0 // Set digital low pass filter to configuration 2
            // (94Hz bandwith for accelerometer, 98Hz bandwidth of gyro)
    );

    // Register 27: Gyro configuration
    RobotHal::i2cWrite(i2cAddress, 27,
        1 << 3 // full scale range +-500°/s
    );

    // Register 28: Accelerometer configuration
    //    Not enable accelerometer self test
    RobotHal::i2cWrite(i2cAddress, 58,
        1 << 3 // full scale range +-4g
    );

    // Register 56: Interrupt enable
    RobotHal::i2cWrite(i2cAddress, 56,
        1 << 0 // Data ready interrupt
    );
}

Vector3D<int16_t> RobotHalImu::readI16Vector(uint8_t registerAddress) {
    uint8_t bytes[6];
    RobotHal::i2cRead(i2cAddress, registerAddress, bytes, sizeof(bytes));
    return Vector3D<uint32_t>::indices().apply([&](auto index) -> int16_t {
        return static_cast<int16_t>(bytes[2 * index]) << 8 |
            static_cast<int16_t>(bytes[2 * index + 1]);
    });
}

int16_t RobotHalImu::readI16(uint8_t registerAddress) {
    uint8_t bytes[2];
    RobotHal::i2cRead(i2cAddress, registerAddress, bytes, sizeof(bytes));
    return static_cast<int16_t>(bytes[0]) << 8 | static_cast<int16_t>(bytes[1]);
}

Vector3D<int16_t> RobotHalImu::readAccelerometer() {
    return readI16Vector(59);
    // TODO: Convert to m/s**2
}

int32_t RobotHalImu::readTemperature() {
    auto raw = readI16(65);
    return (static_cast<int32_t>(raw) + 12420) / 340;
}

Vector3D<int16_t> RobotHalImu::readGyro() {
    return readI16Vector(67);
    // TODO: Convert to deg/s
}

