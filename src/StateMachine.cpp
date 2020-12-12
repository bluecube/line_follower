#include "StateMachine.h"

StateMachine::StateMachine() {
    this->changeState(Bootup());
}

StateMachine::~StateMachine() {
    this->destroyState();
}

void StateMachine::update(int elapsed) {
    switch (this->variant) {
    #define X(StateName) \
    case Variant::StateName: \
        this->state.s##StateName.update(*this, elapsed); \
        break;
    #include "StatesXMacro.h"
    #undef X
    default:
        break;
    }
}

/** Change the current state to state.
Beware that this destroys the currently existing state, so calling it from a state is risky */
#define X(StateName) \
void StateMachine::changeState(StateName&& state) { \
    this->destroyState(); \
    this->variant = Variant::StateName; \
    new (&(this->state.s##StateName)) StateName(state); \
}
#include "StatesXMacro.h"
#undef X

void StateMachine::destroyState() {
    switch (this->variant) {
    #define X(StateName) \
    case Variant::StateName: \
        this->state.s##StateName.~StateName(); \
        break;
    #include "StatesXMacro.h"
    #undef X
    default:
        break;
    }
}

const char* StateMachine::getStateName() const
{
    switch (this->variant) {
    #define X(StateName) \
    case Variant::StateName: \
        return #StateName;
    #include "StatesXMacro.h"
    #undef X
    default:
        return "<no state>";
    }
}
