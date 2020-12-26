#pragma once

#include "../Vector3D.h"

#include <cstdint>

class RobotHal;

/// Wraps functionality of MPU6050 in RobotHal
class RobotHalImu {
protected:
    RobotHalImu() {}
public:
    RobotHalImu(const RobotHalImu&) = delete;
    RobotHalImu(RobotHalImu&&) = delete;
    RobotHalImu& operator=(const RobotHalImu&) = delete;
    RobotHalImu& operator=(RobotHalImu&&) = delete;

    Vector3D<int16_t> readAccelerometer();
    Vector3D<int16_t> readGyro();
    int32_t readTemperature();

protected:
    static constexpr uint8_t i2cAddress = 0b1101000; // pin AD0 not set

    void setup();

    static Vector3D<int16_t> readI16Vector(uint8_t registerAddress);
    static int16_t readI16(uint8_t registerAddress);

    friend class RobotHal;
};
