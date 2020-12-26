/** This header is the replacement point for different HAL implementations.
 * It includes one and typedefs it as "Hal". Selection of active implementation must
 * be done by using preprocessor directives. */
#pragma once

// For now we just use the actual hardware, no other options available

#include "RobotHal/Hal.h"
using Hal = RobotHal::Hal; // This typedef specialises the code to use RobotHal.
