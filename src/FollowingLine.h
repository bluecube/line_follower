#pragma once

#include "Hal.h"
#include "Pid.h"

#include <cstdint>
#include <algorithm>

class StateMachine;

class FollowingLine {
public:
    FollowingLine();
    void update(StateMachine& stateMachine, Hal::MillisecondsT elapsed);

    static int_fast8_t findLine();

private:
    Pid<int32_t, uint32_t> pid;
};

