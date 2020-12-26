/** Configuration parameters of the rorobot */
#pragma once
#include "Hal.h"
#include "defines.h"

#include <stdint.h>
#include <limits>

namespace Parameters {

namespace LineDetector {
    static constexpr auto lineType = LineType::BlackOnWhite;
    static constexpr std::array<int32_t, 4> detectionKernel = {-1, 1, 1, -1};
}

namespace FollowingLine {
    // TODO: Tune the PID
    static constexpr int32_t kP = 25 * std::numeric_limits<int16_t>::max() / 100;
    static constexpr int32_t kI = std::numeric_limits<int16_t>::max() / 2000;
    static constexpr int32_t kD = 2 * std::numeric_limits<int16_t>::max() / 10;

    static constexpr int32_t turningSpeedParameter = std::numeric_limits<int16_t>::max() * std::numeric_limits<int16_t>::max();
}

namespace Hardware {
    static constexpr float motorPWMFrequency = 25e3; // 25kHz to keep things quiet
    static constexpr int lineSensorLedDelay = 250; // Microseconds to wait after setting line sensor LED to give the sensor time to react
    static constexpr int buttonDebounceDelay = 40; // Time in miliseconds after the first switch of the button that is taken as bounce period.
    static constexpr int buttonLongPressDelay = 2000; // Time in miliseconds for the press to be registered as long press

    static constexpr int period = 10;

    static constexpr float voltageCutoff = 9.0;
}

}
