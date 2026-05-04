#pragma once
#include <cstdint>
#include "Hal.h"

class StateMachine;

class ErrorState {
public:
    ErrorState(uint8_t code);
    void update(StateMachine& stateMachine, int elapsed);

private:
    bool ledState = false;
};

