/** Configuration parameters of the rorobot */
#pragma once
#include <stdint.h>
#include "defines.h"

namespace Parameters {

namespace LineDetector {
    static constexpr auto lineType = LineType::BlackOnWhite;
    static constexpr int32_t detectionKernel[] = {-1, 1, 1, -1};
}

}
