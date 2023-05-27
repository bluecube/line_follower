#pragma once

#include "Hal.h"
#include "EncoderObserver.h"

#include <mutex>
#include <condition_variable>

class MotorVelocityControler {
public:
    MotorVelocityControler(Hal& hal, int32_t kP, int32_t kI, int32_t kD);

    void set_requested_velocity(int32_t left, int32_t right);
    void set_coeffs(int32_t kP, int32_t kI, int32_t kD);

    void update(Hal& hal);

    static constexpr int32_t coeffScale = 1 << 12;
private:

    struct Motor {
        int32_t requestedVelocity;
        int32_t lastEncoderState;
        int32_t velocityErrorIntegral;

        EncoderObserver eo;

        void init(int32_t encoderState);
        int32_t update(int32_t encoderState, int32_t kP, int32_t kI, int32_t kD);
    };

    Motor motor[2];
    int32_t kP, kI, kD;
};

class MotorVelocityControlerTask {
public:
    MotorVelocityControlerTask(Hal& hal, int32_t kP, int32_t kI, int32_t kD)
        :
        task(hal.start_periodic_task(
            MotorVelocityControler(hal, kP, kI, kD),
            [&hal](MotorVelocityControler& mvc) { mvc.update(hal); },
            updatePeriod
        ))
    {}

    void set_coeffs(int32_t kP, int32_t kI, int32_t kD) {
        task.access([&](MotorVelocityControler& mvc) { mvc.set_coeffs(kP, kI, kD); });
    }

    void set_requested_velocity(int32_t left, int32_t right) {
        task.access([&](MotorVelocityControler& mvc) { mvc.set_requested_velocity(left, right); });
    }

    static constexpr std::chrono::milliseconds updatePeriod{10};

    /// Speed in m/s corresponding to 1 velocity unit.
    static constexpr double velocityUnit =
        Hal::MotorsT::metersPerTick() /
        std::chrono::duration_cast<std::chrono::duration<double>>(updatePeriod).count() /
        EncoderObserver::velocityScale;
private:
    Hal::PeriodicTaskT<MotorVelocityControler> task;
};
