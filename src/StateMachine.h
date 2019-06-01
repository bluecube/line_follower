/** Defines the main state machine for the robot.
Since we don't have std::variant available on Teensy, we use a ugly hackish workaround built using X macros. */

#pragma once

#include "Bootup.h"
#include "Waiting.h"
#include "FollowingLine.h"

class StateMachine {
private:
    enum class Variant {
        NoState, // Marker for use during class construction
        #define X(StateName) StateName,
        #include "StatesXMacro.h"
        #undef X
    };

    union State {
        #define X(StateName) StateName s##StateName;
        #include "StatesXMacro.h"
        #undef X

        State() {}
    };

public:
    StateMachine();
    ~StateMachine();

    void update(Hal::MillisecondsT elapsed);

    /** Change the current state to state.
    Beware that this destroys the currently existing state, so calling it from a state is risky */
    #define X(StateName) \
    void changeState(StateName&& state);
    #include "StatesXMacro.h"
    #undef X

    const char* getStateName() const;

private:
    void destroyState();

    Variant variant;
    State state;
};


