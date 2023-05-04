#include "EncoderObserver.h"

#include <iostream>
#include <cstdlib>
#include <cassert>
#include <numeric>

#ifndef NDEBUG
#include <cmath>
#endif

void EncoderObserver::reset(int32_t initialVelocity) {
    // We need at least two values to estimate the velocity and accelration.
    // This violates the minSamples condition, but allows full functionality
    // while still having as fast response to changes as possible.
    velocities.clear();
    velocities.push_back(initialVelocity);
    velocities.push_back(initialVelocity);

    sum = 2 * initialVelocity;
    sum2 = sum * initialVelocity;
    indexSum = initialVelocity;

#ifndef NDEBUG
    assert_state();
#endif
}

int32_t EncoderObserver::get_velocity() const {
    uint32_t k = velocities.size(); // Name matches the jupyter notebook
    return fancy_div(
        velocityScale,
        2 * k * sum - sum - 3 * indexSum, // 24bit
        k * (k + 1) / 2
    );
}

int32_t EncoderObserver::get_acceleration() const {
    uint32_t k = velocities.size(); // Name matches the jupyter notebook
    return fancy_div(
        accelerationScale,
        k * sum - sum - 2 * indexSum, // 24bit
        k * (k * k - 1) / 6
    );
}

void EncoderObserver::update(int32_t velocity) {
    auto drop_velocity = [&]() {
        auto popped = velocities.pop_front();
        indexSum -= static_cast<int32_t>(velocities.size()) * popped;
        sum -= popped;
        sum2 -= popped * popped;
    };

    // If the window is full then drop an item to make room for the currently added one
    if (velocities.full())
        drop_velocity();

    // Add the current item
    indexSum += sum;
    sum += velocity;
    sum2 += velocity * velocity;
    velocities.push_back(velocity);

    // Drop values from velocities buffer until the condition is valid, or we're
    // at the minimum supported window size.
    while (velocities.size() > minSamples) {
        uint32_t k = velocities.size();
        int64_t condition = 0;

        // Collect the terms of the condition,
        // attempting to do as little work as possible in 64bit.
        condition += fancy_div(
            1,
            fitRmseThresholdNumerator * fitRmseThresholdNumerator * k * k * (k - 1) * (k + 1),
            fitRmseThresholdDenominator * fitRmseThresholdDenominator
        );
        condition -= extending_mul(sum2, k * (k - 1) * (k + 1));
        condition += extending_mul(12 * indexSum, indexSum);
        condition -= extending_mul(12 * indexSum, sum * (k - 1));
        condition += extending_mul(2 * sum * (k - 1), sum * (2 * k - 1));

        assert((condition < 0) == (get_fit_rmse() > double(fitRmseThresholdNumerator) / fitRmseThresholdDenominator));

        if (condition < 0)
            drop_velocity();
        else
            break;
    }

#ifndef NDEBUG
    assert_state();
#endif
}

#ifndef NDEBUG
void EncoderObserver::assert_state() const {
    //print_state();
    assert(velocities.size() >= minSamples);
    assert(get_fit_rmse() < double(fitRmseThresholdNumerator) / fitRmseThresholdDenominator);

    int32_t baselineSum = 0;
    int32_t baselineIndexSum = 0;
    uint32_t baselineSum2 = 0;

    for (uint32_t i = 0; i < velocities.size(); ++i) {
        baselineSum += velocities[i];
        baselineIndexSum += (velocities.size() - i - 1) * velocities[i];
        baselineSum2 += velocities[i] * velocities[i];
    }

    assert(sum == baselineSum);
    assert(indexSum == baselineIndexSum);
    assert(sum2 == baselineSum2);
}

double EncoderObserver::get_fit_rmse() const {
    double k = velocities.size();

    return sqrt(
        (
            static_cast<double>(sum2) * k * (k - 1) * (k + 1)
            - 12.0 * static_cast<double>(indexSum) * indexSum
            + 12.0 * static_cast<double>(indexSum) * sum * (k - 1.0)
            - 2.0 * static_cast<double>(sum) * sum * (k - 1) * (2 * k - 1)
        )
        / (k * k * (k - 1) * (k + 1))
    );
}

double EncoderObserver::get_fit_rmse2() const {
    const double velocity0 = static_cast<double>(get_velocity()) / velocityScale;
    const double acceleration0 = static_cast<double>(get_acceleration()) / accelerationScale;
    double sum = 0;

    for (uint32_t i = 0; i < velocities.size(); ++i) {
        double expectedVelocity = velocity0 - (velocities.size() - i - 1) * acceleration0;
        double measuredVelocity = velocities[i];
        double error = expectedVelocity - measuredVelocity;
        sum += error * error;
    }

    return sqrt(sum / velocities.size());
}
#endif

int32_t EncoderObserver::fancy_div(int32_t scale, int32_t a, uint32_t b) {
    assert(b != 0);

    int32_t signedB = b;

    assert(!__builtin_mul_overflow_p(a, scale, (int32_t)0));
    a *= scale;

    int32_t rounding = b / 2;
    if (a < 0)
        rounding -= rounding;

    assert(!__builtin_add_overflow_p(a, rounding, (int32_t)0));
    a += rounding;

    return a / signedB;
}

int64_t EncoderObserver::extending_mul(int32_t a, int32_t b) {
    return static_cast<int64_t>(a) * static_cast<int64_t>(b);
}

void EncoderObserver::print_state() const {
    std::cout << "    Velocity: " << (static_cast<double>(get_velocity()) / velocityScale) << ", Acceleration: " << (static_cast<double>(get_acceleration()) / accelerationScale) << "\n";
    std::cout << "    Window size: " << velocities.size() << ", sum: " << sum << ", sum2: " << sum2 << ", indexSum: " << indexSum << "\n";
    std::cout << "    Fit RMSE from sums: " << get_fit_rmse() << "; direct: " << get_fit_rmse2() << "\n";
}
