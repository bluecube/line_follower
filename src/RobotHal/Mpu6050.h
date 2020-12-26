#pragma once

#include "../Vector3D.h"

#include <cstdint>

namespace RobotHal {

class Hal;

/// Wraps functionality of MPU6050 in Hal
class Mpu6050 {
protected:
    Mpu6050() {}
public:
    Mpu6050(const Mpu6050&) = delete;
    Mpu6050(Mpu6050&&) = delete;
    Mpu6050& operator=(const Mpu6050&) = delete;
    Mpu6050& operator=(Mpu6050&&) = delete;

    Vector3D<int16_t> readAccelerometer();
    Vector3D<int16_t> readGyro();
    int32_t readTemperature();

protected:
    static constexpr uint8_t i2cAddress = 0b1101000; // pin AD0 not set

    void setup();

    static Vector3D<int16_t> readI16Vector(uint8_t registerAddress);
    static int16_t readI16(uint8_t registerAddress);

    friend class Hal;
};

}
