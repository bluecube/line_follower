#include "EncoderObserver.h"

#include <iostream>

void EncoderObserver::init(int32_t initialObservedPosition) {
    position = initialObservedPosition * positionScale;
    velocity = 0;

    positionVariance = measurementStDev * measurementStDev;
    velocityVariance = 0;
    positionVelocityCovariance = 0;
}

void EncoderObserver::update(int32_t observedPosition, Hal::Clock::duration dt) {
    predict(dt.count());
    observe(observedPosition * positionScale);
}

void EncoderObserver::predict(uint32_t dt) {
    // dt: <= 15bit

    // integrate turns [X / Cs] unit into [X]
    auto integrate = [&](auto v) { return rounded_div<decltype(v)>(v * static_cast<decltype(v)>(dt), clockScale); };

    auto addedVelocityVariance = sq(integrate(accelerationStDev)); // [Vs**2], <= 14bit

    // Predict position change
    position += integrate(velocity);

    // Predict covariance matrix changes
    positionVariance += integrate(2 * positionVelocityCovariance + integrate(velocityVariance + addedVelocityVariance / 4));
    positionVelocityCovariance += integrate(velocityVariance + addedVelocityVariance / 2);
    velocityVariance += addedVelocityVariance;

    std::cout << "After predict:\n";
    print_state();
}

void EncoderObserver::observe(int32_t observedPosition) {
    std::cout << "Acceleration stdev: " << accelerationStDev << "Vs/Cs\n";
    std::cout << "Measurement stdev: " << measurementStDev << "Es\n";


    auto divider = positionVariance + sq(measurementStDev);
    auto observedResidual = observedPosition - position;

    const uint32_t dividerScale = 1024;
    auto pDiv = rounded_div(dividerScale * positionVariance, divider);
    auto pvDiv = rounded_div(dividerScale * positionVelocityCovariance, divider);

    position += rounded_div<int32_t>(observedResidual * static_cast<int32_t>(pDiv), dividerScale);
    velocity += rounded_div<int32_t>(observedResidual * static_cast<int32_t>(pvDiv), dividerScale);

    positionVariance -= rounded_div(positionVariance * pDiv, dividerScale);
    positionVelocityCovariance -= rounded_div(positionVelocityCovariance * pDiv, dividerScale);
    velocityVariance -= rounded_div(positionVelocityCovariance * pvDiv, dividerScale);

    std::cout << "After observe:\n";
    print_state();
}

void EncoderObserver::print_state() const {
    std::cout << "    Position: " << position << " Es; " << position * Units::Es * 1e3 << "mm\n";
    std::cout << "    Velocity: " << velocity << " Vs; " << position * Units::Vs * 1e3 << "mm/s\n";
    std::cout << "    Position variance: " << positionVariance << " Es**2 (stdev: " << fsqrt(positionVariance) << "Es; " << fsqrt(positionVariance) * Units::Es * 1e3 << "mm)\n";
    std::cout << "    Velocity variance: " << velocityVariance << " Vs**2 (stdev: " << fsqrt(velocityVariance) << "Vs; " << fsqrt(velocityVariance) * Units::Vs * 1e3 << "mm/s)\n";
    std::cout << "    Position-velocity covariance: " << positionVelocityCovariance << " Es * Vs\n";
}
