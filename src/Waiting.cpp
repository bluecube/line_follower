#include "Waiting.h"
#include "Hal.h"
#include "StateMachine.h"

void Waiting::update(StateMachine& stateMachine, Hal::MillisecondsT elapsed) {
    if (Hal::instance().pollButton() == Hal::ButtonEvent::LongPress)
        stateMachine.changeState(FollowingLine());
}
