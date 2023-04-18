/// Uilities for dealing with fixed point values
#pragma once

#include <chrono>

#define strcat_(x, y) x ## y
#define strcat(x, y) strcat_(x, y)
#define COMPILE_TIME_PRINT_VALUE(x) template <int> struct strcat(strcat(value_of_, x), _is); static_assert(strcat(strcat(value_of_, x), _is)<x>::x, "");

/// Parametrized by a std::chrono::duration, this returns number of seconds in one unit as floating point.
template <typename D>
static constexpr double periodSeconds = D::period::num / static_cast<double>(D::period::den);

/// Divide a value by a unit conversion factor and round it to type T.
/// value is in some base unit (eg. meters), unitConversion is a conversion
/// factor baseUnit / targetUnit (eg. meters / encoder tick).
template <typename T>
static constexpr int32_t to_unit(double value, double unitConversion) {
    return static_cast<int32_t>(0.5 + value / unitConversion);
}

template <typename T, typename U=T>
constexpr U rounded_div(T a, T b) {
    return static_cast<U>((a + (b / 2)) / b);
}

template <typename T>
constexpr T sq(T x) {
    return x * x;
}

