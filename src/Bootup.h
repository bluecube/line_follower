#pragma once
#include "Hal.h"

class StateMachine;

class Bootup {
public:
    void update(StateMachine& stateMachine, Hal::MillisecondsT elapsed);

private:
    bool ledState = false;
};

