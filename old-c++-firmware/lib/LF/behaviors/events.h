#pragma once

#include "Hal.h"

struct ButtonPressed {};
struct ButtonReleased {};
struct TimerEvent { Hal::Clock::time_point time; };
