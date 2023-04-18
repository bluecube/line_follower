/** This header is the replacement point for different HAL implementations.
 * It includes one and typedefs it as "Hal". Selection of active implementation must
 * be done by using preprocessor directives. */
#pragma once

#ifdef PIO_UNIT_TESTING
    #include "MockHal/Hal.h"
    using Hal = MockHal::Hal;
#else
    #include "RobotHal/Hal.h"
    using Hal = RobotHal::Hal;
#endif
