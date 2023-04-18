#pragma once

#include <cassert>
#include <cmath>
#include <chrono>
#include <utility>

namespace MockHal {

class Hal {
protected:
    Hal();
public:

    /// Steady clock with identical resolution as EspTimer
    class Clock {
    public:
        using rep = int64_t;
        using period = std::micro;
        using duration = std::chrono::duration<rep, period>;
        using time_point = std::chrono::time_point<Clock>;
        static constexpr bool is_steady = true;

        static inline time_point now() noexcept {
            const auto steady_now = std::chrono::steady_clock::now();
            return time_point(std::chrono::duration_cast<duration>(steady_now.time_since_epoch()));
        }
    };

    struct LineSensorT {

    };

    struct MotorsT {
        using PwmT = int32_t;
        static inline constexpr PwmT maxPwm() { return 400; }
        static inline constexpr double metersPerTick() { return 40e-3 * M_PI / 280; }


        void set(PwmT left, PwmT right) {
            assert(false);
        }

        std::pair<int32_t, int32_t> readEncoders() const {
            assert(false);
            return std::make_pair(0, 0);
        }
    };

    struct ImuT {

    };

    Hal(const Hal&) = delete;
    Hal(Hal&&) = delete;
    Hal& operator=(const Hal&) = delete;
    Hal& operator=(Hal&&) = delete;

    /// Return reference to the current HAL instance.
    static Hal& instance();

    /*
    /// Enable or disable the blue LED on the module.
    void setBuiltinLed(bool enabled);

    int readRange();

    float readBatteryVoltage();
    */

    //LineSensorT lineSensor;
    MotorsT motors;
    //ImuT imu;
    //Button bootButton;
    //Button deckButton;

};

}
