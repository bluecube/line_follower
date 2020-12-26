#pragma once

#include "idf_util.h"

#include <array>
#include <utility>

// Board pins, manually copied from the schematic
struct RobotHalPins {
    using PinT = IdfUtil::PinT;

    static constexpr std::array<std::pair<PinT, PinT>, 2> motor = {
        std::make_pair(13, 4),
        std::make_pair(23, 19)
    };
    static constexpr std::array<std::pair<PinT, PinT>, 2> encoder = {
        std::make_pair(16, 17),
        std::make_pair(36, 39)
    };
    static constexpr std::array<PinT, 5> lineSensor = {33, 14, 35, 25, 34};
    static constexpr std::array<PinT, 3> lineLed = {27, 32, 26};
    static constexpr PinT range = 12;
    static constexpr PinT scl = 22;
    static constexpr PinT sda = 21;
    static constexpr PinT accelInterrupt = 18;
    static constexpr PinT mainBbutton = 5;
    static constexpr PinT bootBbutton = 0;
    static constexpr PinT batSense = 15;
    static constexpr PinT indicatorLed = 2;
};
