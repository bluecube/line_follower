#include <doctest.h>

#include "Hal.h"
#include "EncoderObserver.h"

#include <chrono>
#include <iostream>
#include <cmath>



class Simulator {
public:
    Simulator(EncoderObserver& eo, double initialPosition = 0.0)
        : eo(eo), position(initialPosition)
    {}

    void simulate(
        double startVelocity, double endVelocity,
        Hal::Clock::duration dt,
        Hal::Clock::duration testDuration
    ) {
        uint32_t stepCount = testDuration / dt;
        std::cout << "stepCount: " << stepCount << "\n";
        for (uint32_t i = 0; i < stepCount; ++i) {
            double velocity = startVelocity + (endVelocity - startVelocity) * i / (stepCount - 1);
            position += (velocity * dt.count()) / Hal::Clock::duration{std::chrono::seconds{1}}.count();

            std::cout << "i: " << i << "\n";
            std::cout << "position: " << position / EncoderObserver::Units::Es << "Es; " << position * 1e3 << "mm\n";
            std::cout << "velocity: " << velocity / EncoderObserver::Units::Vs << "Vs; " << velocity * 1e3 << "mm/s\n";

            eo.update(static_cast<int32_t>(position / EncoderObserver::Units::E), dt);

            double positionError = fabs(eo.getPosition() * EncoderObserver::Units::Es - position);
            positionMaeAccumulator += positionError;

            double velocityError = fabs(eo.getVelocity() * EncoderObserver::Units::Vs - velocity);
            velocityMaeAccumulator += velocityError;

            counter += 1;

            REQUIRE(positionError < EncoderObserver::Units::E * 1.01);
            REQUIRE(velocityError < EncoderObserver::Units::E * Hal::Clock::duration{std::chrono::seconds{1}}.count() / dt.count());
        }
    }

    double position;

    void printMae() {
        std::cout << "Position MAE: " << positionMaeAccumulator / counter * 1e3 << "mm\n";
        std::cout << "Velocity MAE: " << velocityMaeAccumulator / counter * 1e3 << "mm/s\n";
    }

private:
    uint64_t counter = 0;
    double positionMaeAccumulator = 0.0, velocityMaeAccumulator = 0.0;
    EncoderObserver& eo;
};

TEST_CASE("Stopped") {
    Hal::Clock::duration dt{std::chrono::milliseconds{20}};
    Hal::Clock::duration testDuration{std::chrono::seconds{2/*5*/}};

    EncoderObserver eo;
    Simulator sim(eo);
    eo.init(static_cast<int32_t>(sim.position / EncoderObserver::Units::E));
    sim.simulate(0, 0, dt, testDuration);
}

TEST_CASE("Accelerate and cruise") {
    double velocity;  // [m/s]
    SUBCASE("medium") { velocity = 0.5; }
    //SUBCASE("fast") { velocity = 2.0; }
    //SUBCASE("slow") { velocity = 1.0e-3; }

    Hal::Clock::duration dt{std::chrono::milliseconds{20}};
        // Worst supported case for variable range,
        // still bad enough for slow speed resolution

    Hal::Clock::duration testDuration{std::chrono::seconds{2}};

    EncoderObserver eo;

    Simulator sim(eo);

    eo.init(static_cast<int32_t>(sim.position / EncoderObserver::Units::E));

    sim.simulate(0, velocity, dt, testDuration); // Acceleratin stage
    sim.simulate(velocity, velocity, dt, testDuration); // Cruise stage

    sim.printMae();
}
