#pragma once

#include <cstdint>
#include <utility>

namespace RobotHal {

class Hal;

class Motors {

protected:
    Motors() {}
public:
    Motors(const Motors&) = delete;
    Motors(Motors&&) = delete;
    Motors& operator=(const Motors&) = delete;
    Motors& operator=(Motors&&) = delete;

    /// Set PWM signals for motors. Positive driving forward,
    void set(int16_t left, int16_t right);

    std::pair<int16_t, int16_t> readEncoders() const;

protected:
    void setup();

    friend class Hal;
};

}
