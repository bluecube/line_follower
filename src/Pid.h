#pragma once

template <typename InputT_, typename OutputT_,  typename TimeT_>
class Pid {
public:
    // Input / output types:
    using InputT = InputT_;
    using OutputT = OutputT_;
    using TimeT = TimeT_;

    // Internal types for integrals and derivatives
    using IntegralT = decltype(InputT() * TimeT());
    using DerivativeT = decltype(InputT() / TimeT());

    // Parameter types
    using KPT = decltype(OutputT() / InputT());
    using KIT = decltype(OutputT() / IntegralT());
    using KDT = decltype(OutputT() / DerivativeT());

    Pid(KPT kP, KIT kI, KDT kD)
        : kP(kP), kI(kI), kD(kD), lastValue(InputT()), integral(IntegralT()) {}

    OutputT update(InputT value, InputT setpoint, TimeT timeSinceLastUpdate) {
        auto error = setpoint - value;

        auto output = this->kP * error;

        this->integral += error * timeSinceLastUpdate;
        output += this->kI * this->integral;

        output += (this->kD * (value - this->lastValue)) / timeSinceLastUpdate;
        this->lastValue = value;

        return output;
    }

    KPT kP;
    KIT kI;
    KDT kD;
private:
    InputT lastValue;
    IntegralT integral;
};
