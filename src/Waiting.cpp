#include "Waiting.h"
#include "Hal.h"
#include "StateMachine.h"

void Waiting::update(StateMachine& stateMachine, int elapsed) {
    if (Hal::instance().pollButton() == Hal::ButtonEvent::LongPress)
        stateMachine.changeState(FollowingLine());
}
