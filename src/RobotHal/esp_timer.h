#pragma once

#include <esp_timer.h>

#include <chrono>
#include <ratio>
#include <cstdint>

namespace RobotHal {

/// C++ wrapper around esp_timer_get_time().
struct EspTimer {
    using rep = uint64_t;
    using period = std::micro;
    using duration = std::chrono::duration<rep, period>;
    using time_point = std::chrono::time_point<EspTimer>;
    static constexpr bool is_steady = true;

    static inline time_point now() {
        return time_point(duration(esp_timer_get_time()));
    }
};

}
