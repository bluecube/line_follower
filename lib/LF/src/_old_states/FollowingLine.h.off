#pragma once

#include "Hal.h"
#include "Pid.h"

#include <cstdint>
#include <algorithm>

class StateMachine;

class FollowingLine {
public:
    FollowingLine();
    void update(StateMachine& stateMachine, int elapsed);

    static int32_t findLine();

private:
    PidControler<int32_t, uint32_t> pid;
};

