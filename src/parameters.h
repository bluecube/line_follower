/** Configuration parameters of the rorobot */
#pragma once
#include <stdint.h>
#include "Hal.h"
#include "defines.h"

namespace Parameters {

namespace LineDetector {
    static constexpr auto lineType = LineType::BlackOnWhite;
    static constexpr std::array<int32_t, 4> detectionKernel = {-1, 1, 1, -1};
}

namespace FollowingLine {
    // TODO: Tune the PID
    static constexpr int32_t kP = 25 * Hal::motorMaxValue / 100;
    static constexpr int32_t kI = Hal::motorMaxValue / 2000;
    static constexpr int32_t kD = 2 * Hal::motorMaxValue / 10;

    static constexpr int32_t turningSpeedParameter = Hal::motorMaxValue * Hal::motorMaxValue;
}

namespace Hardware {
    static constexpr float motorPWMFrequency = 20e3; // 20kHz to keep things quiet
    static constexpr Hal::MicrosecondsT lineSensorLedDelay = 250; // Microseconds to wait after setting line sensor LED to give the sensor time to react
    static constexpr Hal::MillisecondsT buttonDebounceDelay = 40; // Time in miliseconds after the first switch of the button that is taken as bounce period.
    static constexpr Hal::MillisecondsT buttonLongPressDelay = 2000; // Time in miliseconds for the press to be registered as long press

    static constexpr Hal::MillisecondsT period = 10;

    static constexpr float voltageCutoff = 9.0;
}

}
