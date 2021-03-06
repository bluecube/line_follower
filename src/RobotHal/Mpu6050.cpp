#include "Mpu6050.h"

#include "Hal.h"

#include <driver/i2c.h>

#include <cstdio>

namespace RobotHal {

void Mpu6050::setup() {
    printf("Setting up IMU\n");

    // Register 25:
    Hal::i2cWrite(i2cAddress, 25,
        1000 / 100 - 1 // Set sample rate to 100Hz
    );

    // Register 26:
    Hal::i2cWrite(i2cAddress, 26,
        2 << 0 // Set digital low pass filter to configuration 2
            // (94Hz bandwith for accelerometer, 98Hz bandwidth of gyro)
    );

    // Register 27: Gyro configuration
    Hal::i2cWrite(i2cAddress, 27,
        1 << 3 // full scale range +-500°/s
    );

    // Register 28: Accelerometer configuration
    //    Not enable accelerometer self test
    Hal::i2cWrite(i2cAddress, 58,
        1 << 3 // full scale range +-4g
    );

    // Register 56: Interrupt enable
    Hal::i2cWrite(i2cAddress, 56,
        1 << 0 // Data ready interrupt
    );

    // Register 107: Power management 1
    Hal::i2cWrite(i2cAddress, 107,
        0 << 5 | // Sleep mode disabled
        1 << 0 // Using X gyro axis as clock source
    );

    checkConnection();
}

Vector3D<int16_t> Mpu6050::readI16Vector(uint8_t registerAddress) {
    uint8_t bytes[6];
    Hal::i2cRead(i2cAddress, registerAddress, bytes, sizeof(bytes));
    return Vector3D<uint32_t>::indices().apply([&](auto index) -> int16_t {
        return static_cast<int16_t>(bytes[2 * index]) << 8 |
            static_cast<int16_t>(bytes[2 * index + 1]);
    });
}

int16_t Mpu6050::readI16(uint8_t registerAddress) {
    uint8_t bytes[2];
    Hal::i2cRead(i2cAddress, registerAddress, bytes, sizeof(bytes));
    return static_cast<int16_t>(bytes[0]) << 8 | static_cast<int16_t>(bytes[1]);
}

Vector3D<int16_t> Mpu6050::readAccelerometer() {
    return readI16Vector(59);
    // TODO: Convert to m/s**2
}

int32_t Mpu6050::readTemperature() {
    auto raw = readI16(65);
    return (static_cast<int32_t>(raw) + 12420) / 340;
}

Vector3D<int16_t> Mpu6050::readGyro() {
    return readI16Vector(67);
    // TODO: Convert to deg/s
}

bool Mpu6050::checkConnection() {
    uint8_t answer;
    Hal::i2cRead(i2cAddress, 117, &answer, 1);

    uint8_t expected = i2cAddress & 0xfe;

    bool ok = (answer == expected);

    if (!ok)
        printf("MPU6050 WHO_AM_I register has value 0x%02x, expected 0x%02x\n",
            answer, expected);

    return ok;
}

}
