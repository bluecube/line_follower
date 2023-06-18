#include "MotorVelocityControler.h"

MotorVelocityControler::MotorVelocityControler(Hal& hal, int32_t kP, int32_t kI, int32_t kD)
    :
    motor{Motor(), Motor()},
    kP(kP), kI(kI), kD(kD)
{
    hal.motors.set(0, 0);
    auto currentEncoderState = hal.motors.readEncoders();
    motor[0].init(currentEncoderState.first);
    motor[1].init(currentEncoderState.second);
}

void MotorVelocityControler::set_requested_velocity(int32_t left, int32_t right) {
    motor[0].requestedVelocity = left;
    motor[1].requestedVelocity = right;
}

void MotorVelocityControler::set_coeffs(int32_t kP, int32_t kI, int32_t kD) {
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
}

void MotorVelocityControler::update(Hal& hal) {
    auto currentEncoderState = hal.motors.readEncoders();

    auto leftCommand = motor[0].update(currentEncoderState.first, kP, kI, kD);
    auto rightCommand = motor[1].update(currentEncoderState.second, kP, kI, kD);

    hal.motors.set(leftCommand, rightCommand);
}

void MotorVelocityControler::Motor::init(int32_t encoderState) {
    requestedVelocity = 0;
    lastEncoderState = encoderState;
    velocityErrorIntegral = 0;
}

int32_t MotorVelocityControler::Motor::update(int32_t encoderState, int32_t kP, int32_t kI, int32_t kD) {
    const int32_t rawVelocity = encoderState - lastEncoderState;
    eo.update(rawVelocity);

    lastEncoderState = encoderState;

    const int32_t filteredVelocity = eo.get_velocity();
    const int32_t filteredAcceleration = eo.get_acceleration();

    const int32_t velocityError = requestedVelocity - filteredVelocity;
    int32_t output = kP * velocityError;
    velocityErrorIntegral += kI * velocityError;
    output += velocityErrorIntegral;
    output += kD * filteredAcceleration;

    output /= coeffScale;

    //printf("requestedVelocity: %ld, filteredVelocity: %ld, error: %ld, integral: %ld output: %ld, kP: %ld, kI: %ld, kD: %ld\n", requestedVelocity, filteredVelocity, velocityError, velocityErrorIntegral, output, kP, kI, kD);

    static constexpr auto maxPwm = Hal::MotorsT::maxPwm();
    // Output limiting and anti-windup
    if (output > maxPwm) {
        velocityErrorIntegral -= coeffScale * (output - maxPwm);
        return maxPwm;
    } else if (output < -maxPwm) {
        velocityErrorIntegral += coeffScale * (-maxPwm - output);
        return -maxPwm;
    } else
        return output;
}

std::pair<int32_t, int32_t> MotorVelocityControler::get_velocity() const {
    return std::make_pair(motor[0].eo.get_velocity(), motor[1].eo.get_velocity());
}
