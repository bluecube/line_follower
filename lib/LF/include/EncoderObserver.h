#pragma once

#include <cstdint>
#include "util/circular_buffer.hpp"

/// Estimates speed and acceleration from low resolution encoders.
/// Works by fitting a linear function to the finite differences velocity.
///
/// Tuned for update rate of 100Hz, encoder tick size  of 0.28mm,
/// max velocity 2m/s and max acceleration 2m/s**2.
class EncoderObserver {
public:
    static constexpr int32_t velocityScale = 1<<7;
    static constexpr int32_t accelerationScale = 1<<14;

    EncoderObserver() { reset(); }

    /// Reset to the default state, zero initial velocity, as short smoothing window
    /// as possible.
    void reset(int32_t initialVelocity = 0);

    /// Update the internal state based on observed finite differences velocity.
    /// Must be called at regular intervals.
    /// Input is in encoder ticks
    void update(int32_t velocity);

    /// Calculate velocity is in encoder ticks per (velocity scale * timer interval)
    /// based on the current state.
    int32_t get_velocity() const;

    /// Calculate acceleration is in encoder ticks per (acceleration scale * timer itnterval**2)
    /// based on the current state.
    int32_t get_acceleration() const;

    void print_state() const;

#ifndef NDEBUG
    /// Check the internal state, assert that everything is ok.
    /// Runs in O(n)
    void assert_state() const;

    double get_fit_rmse() const;
    double get_fit_rmse2() const;
#endif

private:
    /// Calculate scale * a / b with proper rounding, asserts that the scaling does not
    /// overflow and that b is not zero.
    static int32_t fancy_div(int32_t scale, int32_t a, uint32_t b);

    static int64_t extending_mul(int32_t a, int32_t b);


    /// Calculate scaled sum of error squares
    int32_t get_error_square_sum() const;

    /// Minimum number of samples allowed in the velocities buffer
    static constexpr uint32_t minSamples = 2;

    /// Maximum allowed residual root mean square error from the least squares fit.
    /// If the error is larger than this, decrease the window size.
    /// In encoder tick units
    static constexpr int32_t fitRmseThresholdNumerator = 3;
    static constexpr int32_t fitRmseThresholdDenominator = 4;

    /// Previously observed noisy velcocity measurements,
    /// obtained as finite differences of the input positions.
    /// Unit is encoder ticks per update interval.
    /// Absolute value of each element is approximately 14bit
    tablog::util::CircularBuffer<int32_t, 64, uint32_t> velocities;

    /// Sum of speeds in the window.
    /// 19bit
    int32_t sum;

    /// Sum of squared speeds in the window.
    /// 31bits
    uint32_t sum2;

    /// Sum of products of speed times an index in the window (index 0 is the most recent value)
    /// 24bit
    int32_t indexSum;

};
