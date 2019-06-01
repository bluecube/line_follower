#pragma once

template <typename ValueT, typename TimeT = uint32_t>
class Pid {
public:

        Pid(ValueT kP, ValueT kI, ValueT kD)
            : kP(kP), kI(kI), kD(kD), lastValue(0), integral(0) {}

        ValueT update(ValueT value, ValueT setpoint, TimeT timeSinceLastUpdate = 1U) {
            auto error = setpoint - value;

            auto output = this->kP * error;

            this->integral += error * timeSinceLastUpdate;
            output += this->kI * this->integral;

            output += (this->kD * (value - this->lastValue)) / timeSinceLastUpdate;
            this->lastValue = value;

            return output;
        }

        ValueT kP, kI, kD;
private:
        ValueT lastValue;
        ValueT integral;
};
