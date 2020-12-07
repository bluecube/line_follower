#include "RobotHal.h"

#include <algorithm>

#include "parameters.h"

RobotHal& RobotHal::instance() {
    static RobotHal instance;
    return instance;
}

RobotHal::RobotHal() {
}

void RobotHal::setupMotorPWM() {
}

void RobotHal::setupLineSensor() {
}

void RobotHal::setupButton() {
}

void RobotHal::setMotors(PwmT left, PwmT right) {
    (void)left;
    (void)right;
}

void RobotHal::enableLineSensorLed(uint8_t ledIndex) {
    (void)ledIndex;
}

void RobotHal::disableLineSensorLed() {
}

void RobotHal::setBuiltinLed(bool enable) {
    (void)enable;
}

int RobotHal::readRange() {
    return 0;
}

std::tuple<RobotHal::LineSensorT, RobotHal::LineSensorT> RobotHal::readLineSensor(LineSensorBufferT& buffer) {
    // TODO: Calibration
    // TODO: Make this asynchronous

    return std::make_tuple(0, 0);
    /*static_assert(std::tuple_size<LineSensorBufferT>::value == 8,
        "Must have exactly 8 pixels on the line sensor.");
    static_assert(lineSensorLedCount == 5,
        "Must have exactly 5 LEDs on the line sensor.");
    static_assert(lineSensorCount == 4,
        "Must have exactly 4 phototransistors on the line sensor.");

    for (uint8_t i = 0; i < RobotHal::lineSensorLedCount; ++i)
    {
        this->enableLineSensorLed(i);
        delayMicroseconds(Parameters::Hardware::lineSensorLedDelay); // Wait a bit for the LED to actually turn on.

        if (i == 2)
        {
            // Sensors 1 and 2 can be read in parallel
            auto result = adc.analogSyncRead(lineSensorFirst - 2, lineSensorFirst - 1);
            buffer[4] = result.result_adc0;
            buffer[3] = result.result_adc1;
        }
        else
        {
            if (i > 0)
                buffer[2 * i - 1] = adc.analogRead(lineSensorFirst - i + 1);
            if (i < RobotHal::lineSensorCount)
                buffer[2 * i] = adc.analogRead(lineSensorFirst - i);
        }
    }

    // Ambient light suppression
    this->disableLineSensorLed();
    delayMicroseconds(Parameters::Hardware::lineSensorLedDelay); // Wait a bit for the LED to actually turn off.

    auto minValue = std::numeric_limits<LineSensorT>::max();
    auto maxValue = std::numeric_limits<LineSensorT>::min();

    for (uint8_t i = 0; i < 2; ++i)
    {
        auto result = adc.analogSyncRead(lineSensorFirst - 2 - i, lineSensorFirst - i);
        for (uint8_t j = 0; j < 2; ++j)
        {
            buffer[2 * i + j + 4] -= result.result_adc0;
            minValue = std::min(buffer[2 * i + j + 4], minValue);
            maxValue = std::max(buffer[2 * i + j + 4], maxValue);

            buffer[2 * i + j] -= result.result_adc1;
            minValue = std::min(buffer[2 * i + j], minValue);
            maxValue = std::max(buffer[2 * i + j], maxValue);
        }
    }

    return std::make_tuple(minValue, maxValue);*/
}

float RobotHal::readBatteryVoltage()
{
    return 0.0f;
    //return analogRead(batteryVoltage) * batteryVoltsPerUnit;
}
