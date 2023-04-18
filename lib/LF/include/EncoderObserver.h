#pragma once

#include "Hal.h"

#include <cstdint>

#include "util/fp_util.h"


/// Estimates speed and accurate position from low resolution encoders.
///
/// Uses a Kalman filter, where state contains position and speed,
/// observations yield a low resolution (and possibly noisy) position.
///
/// To fit the calculation into int32_t, we're using rather complex system of fixed point
/// arithmetic / non-standard units:
///   E ... native encoder tick
///      ~0.45mm
///   Es ... scaled encoder tick
///      E / positionScale
///      ~0.028mm
///   C ... clock tick
///      1us
///   Cs ... scaled clock tick
///      C * clockScale
///      ~1s
///   V ... native velocity unit
///      E / C
///   Vs ... velocity unit
///      Es / Cs = V / velocityScale
///      ~0.027mm/s
///
/// See file jupyter/Encoder_smoothing_kalman_filter.ipynb
class EncoderObserver {
public:
    static_assert(
        Hal::Clock::duration::period::num == 1 &&
        Hal::Clock::duration::period::den == 1000000,
        "Value ranges don't work out if clocks rate is different than 1Mhz"
    );

    EncoderObserver() {}

    void init(int32_t initialObservedPosition);

    /// Step the state estimation forward and correct it using the observed position.
    /// observeddPosition is in E units.
    void update(int32_t observedPosition, Hal::Clock::duration dt);

    static constexpr uint32_t positionScale = 16;
    static constexpr uint32_t clockScale = 1<<17;

    /// Contains conversion factor from a given specialty unit to SI
    struct Units {
        /// Native encoder tick [m]
        static constexpr double E = Hal::MotorsT::metersPerTick(); // ~0.028mm
        /// Scaled encoder tick [m]
        static constexpr double Es = E / positionScale; // ~0.056mm
        /// Clock tick [s]
        static constexpr double C = periodSeconds<Hal::Clock::duration>; // 1us
        /// Scaled clock tick [s]
        static constexpr double Cs = C * clockScale; // ~0.13s
        /// Scaled velocity unit [m/s]
        static constexpr double Vs = Es / Cs; // ~0.21mm/s
    };

    int32_t getPosition() const { return position; }
    int32_t getVelocity() const { return velocity; }

    void print_state() const;

private:
    /// Standard deviation of the line follower accelerations
    /// This is mostly a guess, real accelerations will likely be way smaller,
    /// but we also have to account for things like hitting a wall.
    static constexpr int32_t accelerationStDev = to_unit<int32_t>(1 /* [m/s**2] */, Units::Vs / Units::Cs); // 10bit

    /// How far from the actual encoder value we are reading.
    /// This mostly covers rounding errors, but also will help covering up irregular
    /// encoder tick distribution.
    static constexpr uint32_t measurementStDev = to_unit<int32_t>(0.75 * Units::E /* [m] */, Units::Es); // 4 bits

    void predict(uint32_t dt);
    void observe(int32_t observedPosition);

    int32_t position; // [Es], full 32bit range
    int32_t velocity; // [Vs], ~2m/s, 15bit

    uint32_t positionVariance; // [Es**2], 8bit
    uint32_t velocityVariance; // [Vs**2] 20bit
        // This should be lower, but since we add the update variance in every step
        // this gets increased by 20 bit
    uint32_t positionVelocityCovariance; // [Es * Vs], ~8bit
};
