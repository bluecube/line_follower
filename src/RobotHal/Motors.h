#pragma once

#include "driver/mcpwm_prelude.h"
#include "idf_util.h"

#include <cstdint>
#include <utility>
#include <array>

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

    using PwmT = int32_t;

    /// Set PWM signals for motors. Positive values mean driving forward,
    void set(PwmT left, PwmT right) {
        motor[0].set(left);
        motor[1].set(right);
    }

    std::pair<int16_t, int16_t> readEncoders() const;

    static inline constexpr PwmT maxPwm() { return Motor::pwmPeriodTicks; }

protected:
    struct Motor {
        void setup(IdfUtil::PinT pinA, IdfUtil::PinT pinB);
        void set(PwmT duty);
        void forward();
        void reverse();
        void stop();

        mcpwm_timer_handle_t timer;
        mcpwm_oper_handle_t oper;
        mcpwm_cmpr_handle_t comparatorA;
        mcpwm_cmpr_handle_t comparatorB;
        mcpwm_gen_handle_t generatorA;
        mcpwm_gen_handle_t generatorB;

        PwmT lastPwm;

        static constexpr int pwmGroupId = 0;
        static constexpr uint32_t pwmResolutionHz = 10000000; // 10MHz PWM base timer
        static constexpr uint32_t pwmPeriodTicks = pwmResolutionHz / 25000; // 25kHz PWM
    };

    Motor motor[2];

    void setup();

    friend class Hal;
};

}
