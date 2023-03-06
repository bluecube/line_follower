#pragma once

#include "idf_util.h"

#include <driver/mcpwm_prelude.h>
#include <driver/pulse_cnt.h>

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

    std::pair<int32_t, int32_t> readEncoders() const {
        return std::make_pair(motor[0].readEncoder(), motor[1].readEncoder());
    }

    static inline constexpr PwmT maxPwm() { return Motor::pwmPeriodTicks; }

protected:
    struct Motor {
        void setup_pwm(IdfUtil::PinT pinA, IdfUtil::PinT pinB);
        void setup_encoder(IdfUtil::PinT pinA, IdfUtil::PinT pinB);
        void set(PwmT duty);
        int32_t readEncoder() const;

        mcpwm_timer_handle_t timer;
        mcpwm_oper_handle_t oper;
        mcpwm_cmpr_handle_t comparatorA, comparatorB;
        mcpwm_gen_handle_t generatorA, generatorB;

        pcnt_unit_handle_t encoder;
        pcnt_channel_handle_t channelA, channelB;

        PwmT lastPwm;

        static constexpr int pwmGroupId = 0;
        static constexpr uint32_t pwmResolutionHz = 10000000; // 10MHz PWM base timer
        static constexpr uint32_t pwmPeriodTicks = pwmResolutionHz / 25000; // 25kHz PWM
        static constexpr int pcntLimit = 30000;
            // An arbitrary value selecting how often the encoder count overflows.
            // It doesn't really show up anywhere, as the count is extended by ESP-IDF
            // to full int32_t in software.
            // To avoid interrupts firing too often, we set this value rather high (~100m of distance)
    };

    Motor motor[2];

    void setup();

    friend class Hal;
};

}
