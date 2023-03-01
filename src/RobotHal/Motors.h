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

    /// Reconfigure the motors so that they can't drive and only make noise.
    /// See setBeepTone() for description of the pitch parameters.
    void startBeep(uint8_t leftPitch, uint8_t rightPitch);

    /// Set pitch of the motor beeping.
    /// Uses MIDI note numbers for pitch values:
    /// chromatic scale starting at C five octaves below middle C (8.1758 Hz),
    /// going up a octave / for every 12 units.
    /// Value 69 corresponds to concert A (440 Hz)
    void setBeepTone(uint8_t leftPitch, uint8_t rightPitch);

    /// Stop the beeping sound, reset motor to default settings and zero speed.
    void stopBeep() {
        // TODO:
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

        /// Convert pitch value to tick count.
        /// See setBeepTone() for description of the tuning.
        static constexpr uint32_t beepPitchToTicks(uint8_t pitch);
    };

    Motor motor[2];

    void setup();

    friend class Hal;
};

}
