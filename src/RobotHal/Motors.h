#pragma once

#include <driver/mcpwm.h>

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

    // Using float percent is an artefat of ESP-IDF mcpwm module
    using PwmT = float;
    static constexpr PwmT motorMaxValue = 100.0f;

    /// Set PWM signals for motors. Positive values mean driving forward,
    void set(PwmT left, PwmT right) {
        set(MCPWM_UNIT_0, left);
        set(MCPWM_UNIT_1, right);
    }


    std::pair<int16_t, int16_t> readEncoders() const;

protected:
    static void setup();

    static void set(mcpwm_unit_t unit, float duty);

    friend class Hal;
};

}
