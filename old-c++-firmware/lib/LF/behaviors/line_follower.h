#pragma once

#include "Behavior.h"

namespace Lf {

struct LfState: StateBase<State> {
    virtual StateTransition react(ButtonPressed e) { return StateTransition::none; }
    virtual StateTransition react(ButtonReleased e) { return StateTransition<State>::none; }
    virtual StateTransition react(TimerEvent e) { return StateTransition<State>::none; }
};

/// Waiting for button press.
struct PreStart: public LfState {
    
};

/// Main state.
struct FollowingLine: public LfState {
    
};

struct AvoidingObstacle: public LfState {

};

/// Trying to drive over a missing piece of track
struct Bridging: public LfState {

};

/// Stopped by the button. Trap state for now.
struct Stopped: public LfState {};

}
