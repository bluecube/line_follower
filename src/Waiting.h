#pragma once
#include "Hal.h"

class StateMachine;

struct Waiting {
    void update(StateMachine& stateMachine, int elapsed);
};

