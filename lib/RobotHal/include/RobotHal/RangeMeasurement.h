#pragma once
#include <optional>

namespace RobotHal {

/// Represents measurement of a range finder, with the ambiguity between short
/// and long measurements
struct RangeMeasurement {
    int rawMeasurement;

    float longOption() { return rawToDistanceLong(rawMeasurement); }
    std::optional<float> shortOption() { return rawToDistanceShort(rawMeasurement); }

    static float rawToDistanceLong(int rawMeasurement) { return 0.0; }
    static std::optional<float> rawToDistanceShort(int rawMeasurement) { return std::nullopt; }
    static int distanceToRaw(float distance) { return 0; }
};

}
