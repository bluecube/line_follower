#pragma once
#include <cstdint>
#include "Hal.h"

class StateMachine;

class ErrorState {
public:
    ErrorState(uint8_t code);
    void update(StateMachine& stateMachine, Hal::MillisecondsT elapsed);

private:
    bool ledState = false;
};

