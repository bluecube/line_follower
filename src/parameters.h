/** Configuration parameters of the rorobot */
#pragma once
#include <stdint.h>
#include "defines.h"

namespace Parameters {

namespace LineDetector {
    static constexpr auto lineType = LineType::BlackOnWhite;
    static constexpr std::array<int32_t, 4> detectionKernel = {-1, 1, 1, -1};
}

}
