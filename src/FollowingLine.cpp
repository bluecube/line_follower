#include "FollowingLine.h"

#include <algorithm>

#include "parameters.h"
#include "Hal.h"
#include "StateMachine.h"
#include "Waiting.h"

FollowingLine::FollowingLine()
    : pid(Parameters::FollowingLine::kP, Parameters::FollowingLine::kI, Parameters::FollowingLine::kD) {}

void FollowingLine::update(StateMachine& stateMachine, Hal::MillisecondsT elapsed)
{
    if (Hal::instance().pollButton() != Hal::ButtonEvent::None)
    {
        Hal::instance().setMotors(0, 0);
        stateMachine.changeState(Waiting());
        return;
    }

    auto linePosition = findLine();
    //Serial.print("LP: "); Serial.print(linePosition);
    auto turningSpeed = -this->pid.update(linePosition, 0, elapsed);
    //Serial.print(" TS: "); Serial.print(turningSpeed);
    //Serial.println();

    // TODO: Refactor the speed control part somewhere else.

    auto absTurningSpeed = turningSpeed > 0 ? turningSpeed : -turningSpeed;
    int32_t speed;

    if (absTurningSpeed != 0)
        speed = std::min<decltype(turningSpeed)>(
            Parameters::FollowingLine::turningSpeedParameter / absTurningSpeed + absTurningSpeed / 2,
            Hal::motorMaxValue / 2);
    else
        speed = Hal::motorMaxValue / 2;

    if (turningSpeed >= 0)
        Hal::instance().setMotors(speed, speed - absTurningSpeed);
    else
        Hal::instance().setMotors(speed - absTurningSpeed, speed);
}

int32_t FollowingLine::findLine() {
    constexpr auto& kernel = Parameters::LineDetector::detectionKernel;

    Hal::LineSensorT minValue, maxValue;
    Hal::LineSensorBufferT buffer;
    std::tie(minValue, maxValue) = Hal::instance().readLineSensor(buffer);

    constexpr auto valueSign = Parameters::LineDetector::lineType == LineType::WhiteOnBlack ? 1 : -1;
    auto valueOffset = Parameters::LineDetector::lineType == LineType::WhiteOnBlack ? minValue : maxValue;

    int32_t sum = 0;
    int32_t sumWeights = 0;

    int32_t bestPosition = -1;
    int32_t bestSumWeights = 0;

    for (int32_t offsetIndex = 1 - static_cast<int32_t>(kernel.size());
         offsetIndex < static_cast<int32_t>(buffer.size());
         ++offsetIndex)
    {
        const size_t min = std::max<int32_t>(-offsetIndex, 0);
        const size_t max = std::min<int32_t>(buffer.size() - offsetIndex, kernel.size());

        static_assert((kernel.size() - buffer.size()) % 2 == 0, "Kernel size must be even iff buffer size is even");
        const int32_t position = offsetIndex - (buffer.size() - kernel.size()) / 2;

        Hal::LineSensorT weight = 0;

        for (size_t i = min; i < max; ++i)
        {
            auto value = valueSign * (buffer[offsetIndex + i] - valueOffset);
            weight += kernel[i] * value;
        }
        if (weight > 0)
        {
            sum += weight * position;
            sumWeights += weight;
        }
        else if (sumWeights > 0)
        {
            auto averagedPosition = (16 * sum) / sumWeights;
            if (sumWeights >= bestSumWeights) {
                bestSumWeights = sumWeights;
                bestPosition = averagedPosition;
            }
            sum = 0;
            sumWeights = 0;
        }
    }

    auto averagedPosition = (16 * sum) / sumWeights;
    if (sumWeights >= bestSumWeights) {
        bestSumWeights = sumWeights;
        bestPosition = averagedPosition;
    }
    // TODO: Also return bestSumWeights.
    return bestPosition;
}
