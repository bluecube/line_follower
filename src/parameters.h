/** Configuration parameters of the robot */
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
    static constexpr float voltageCutoff = 9.0;
}

}
