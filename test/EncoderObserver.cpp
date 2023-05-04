#include <doctest.h>

#include "Hal.h"
#include "EncoderObserver.h"

#include <chrono>
#include <iostream>
#include <cmath>



class Simulator {
public:
    static constexpr double dt = 1.0 / 100;
    static constexpr double encoderTick = Hal::MotorsT::metersPerTick();
    static constexpr double velocityUnit = encoderTick / (EncoderObserver::velocityScale * dt);
    static constexpr double accelerationUnit = encoderTick / (EncoderObserver::accelerationScale* dt * dt);

    Simulator(EncoderObserver& eo, double initialPosition = 0.0)
        : eo(eo), position(initialPosition)
    {
        get_input_velocity(); // Cycle the conversion once, update previous encoder state
    }

    void simulate(
        double startVelocity, double endVelocity,
        double testDurationSeconds,
        uint32_t ignoreForStats = 0
    ) {
        uint32_t stepCount = testDurationSeconds / dt;
        double acceleration = (endVelocity - startVelocity) / testDurationSeconds;
        //std::cout << "stepCount: " << stepCount << "\n";
        for (uint32_t i = 0; i < stepCount; ++i) {
            double velocity = startVelocity + (endVelocity - startVelocity) * i / (stepCount - 1);
            position += velocity * dt;

            //std::cout << "i: " << i << "\n";
            //std::cout << "velocity: " << velocity * 1e3 << "mm/s\n";

            eo.update(get_input_velocity());

            auto outputVelocity = eo.get_velocity();
            //std::cout << "output velocity: " << outputVelocity << " (" << outputVelocity * velocityUnit * 1e3 << "mm/s)\n";

            auto outputAcceleration = eo.get_acceleration();
            //std::cout << "output acceleration: " << outputAcceleration << " (" << outputAcceleration * accelerationUnit * 1e3 << "mm/s**2)\n";

            if (i > ignoreForStats) {
                double velocityError = outputVelocity * velocityUnit - velocity;
                velocityMseAccumulator += velocityError * velocityError;

                double accelerationError = outputAcceleration * accelerationUnit - acceleration;
                accelerationMseAccumulator += accelerationError * accelerationError;

                counter += 1;
            }
        }
    }

    double position;

    void print_mse() {
        std::cout << "Velocity RMSE: " << velocity_rmse() * 1e3 << "mm/s\n";
        std::cout << "Acceleration RMSE: " << acceleration_rmse() * 1e3 << "mm/s\n";
    }

    double velocity_rmse() const {
        return sqrt(velocityMseAccumulator / counter);
    }

    double acceleration_rmse() const {
        return sqrt(accelerationMseAccumulator / counter);
    }

private:
    int32_t get_input_velocity() {
        int32_t encoderState = static_cast<int32_t>(position / encoderTick);
        int32_t ret = encoderState - prevEncoderState;

        prevEncoderState = encoderState;
        return ret;
    }

    int32_t prevEncoderState = 0;
    uint64_t counter = 0;
    double velocityMseAccumulator = 0.0;
    double accelerationMseAccumulator = 0.0;
    EncoderObserver& eo;
};

TEST_CASE("Constant velocity") {
    double velocity;  // [m/s]
    SUBCASE("stopped") { velocity = 0; }
    SUBCASE("medium") { velocity = 0.5; }
    SUBCASE("fast") { velocity = 2.0; }
    SUBCASE("slow") { velocity = 1.0e-3; }

    CAPTURE(velocity);

    std::cout << "Velocity: " << velocity * 1e3 << " mm/s\n";

    EncoderObserver eo;
    Simulator sim(eo);
    double testDurationSeconds = 2;
    sim.simulate(velocity, velocity, testDurationSeconds, 8);

    sim.print_mse();

    REQUIRE(sim.velocity_rmse() <= velocity * 0.025 + 1e-3);
    REQUIRE(sim.acceleration_rmse() <= velocity + 2e-3);
}

/// Tests driving hard at the limit of the speed and acceleration
/// Mostly important for checking numerical overflows (through asserts) in the encoder
/// observer.
TEST_CASE("Accelerate and cruise") {
    double velocity;
    SUBCASE("positive") { velocity = 2.0; }
    SUBCASE("negative") { velocity = 2.0; }
    double acceleration = 2.0;

    EncoderObserver eo;

    Simulator sim(eo);

    sim.simulate(0, velocity, fabs(velocity) / acceleration); // Max acceleration
    sim.simulate(velocity, velocity, 2); // Cruise stage at max speed

    sim.print_mse();

    REQUIRE(sim.velocity_rmse() <= velocity * 0.05 + 1e-3);
    REQUIRE(sim.acceleration_rmse() <= velocity + 2e-3);
}

/// Similar to constant velocity input, but works on lower level, checks that there
/// are no significant problems in the least squares velocity estimation.
/// Also provides some more oportunities for catching integer overflows.
TEST_CASE("Constant input") {
    int32_t velocity;

    SUBCASE("stopped") { velocity = 0; }
    SUBCASE("medium") { velocity = 2500; }
    SUBCASE("reverse") { velocity = -2500; }
    SUBCASE("fast") { velocity = 4457; }
    SUBCASE("slow") { velocity = 1; }

    EncoderObserver eo;

    eo.reset(velocity);

    double result = static_cast<double>(eo.get_velocity()) / static_cast<double>(EncoderObserver::velocityScale);
    REQUIRE(result == velocity);
    REQUIRE(eo.get_acceleration() == 0);

    for (unsigned i = 0; i < 100; ++i) {
        eo.update(velocity);
        result = static_cast<double>(eo.get_velocity()) / static_cast<double>(EncoderObserver::velocityScale);
        REQUIRE(result == velocity);
        REQUIRE(eo.get_acceleration() == 0);
    }
}

TEST_CASE("Hit the wall") {
    EncoderObserver eo;

    Simulator sim(eo);

    sim.simulate(0.5, 0.5, 1); // Cruising at 0.5m/s
    sim.simulate(0, 0, 0.1); // Suddenly stopped

    REQUIRE(fabs(eo.get_velocity() * Simulator::velocityUnit) < 1e-4);

    sim.print_mse();
}
