#pragma once
#include "Hal.h"

class StateMachine;

struct Waiting {
    void update(StateMachine& stateMachine, Hal::MillisecondsT elapsed);
};

