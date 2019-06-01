#include "Bootup.h"
#include "Hal.h"
#include "StateMachine.h"
#include "Waiting.h"
#include "FollowingLine.h"

void Bootup::update(StateMachine& stateMachine, Hal::MillisecondsT elapsed) {
    auto buttonState = Hal::instance().pollButton();

    if (buttonState == Hal::ButtonEvent::LongPress) {
        stateMachine.changeState(FollowingLine());
        Hal::instance().setBuiltinLed(false);
        return;
    }
    else if (buttonState == Hal::ButtonEvent::ShortPress || Hal::instance().isSerialReady()) {
        stateMachine.changeState(Waiting());
        Hal::instance().setBuiltinLed(false);
        return;
    }

    this->ledState = !this->ledState;
    Hal::instance().setBuiltinLed(this->ledState);
}
