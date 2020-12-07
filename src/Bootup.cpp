#include "Bootup.h"
#include "Hal.h"
#include "StateMachine.h"
#include "Waiting.h"
#include "FollowingLine.h"
#include "ErrorState.h"
#include "parameters.h"

void Bootup::update(StateMachine& stateMachine, Hal::MillisecondsT elapsed) {
    if (Hal::instance().readBatteryVoltage() < Parameters::Hardware::voltageCutoff) {
        stateMachine.changeState(ErrorState(0));
        Hal::instance().setBuiltinLed(false);
        return;
    }

    auto buttonState = Hal::instance().pollButton();

    if (buttonState == Hal::ButtonEvent::LongPress) {
        stateMachine.changeState(FollowingLine());
        Hal::instance().setBuiltinLed(false);
        return;
    }
    else if (buttonState == Hal::ButtonEvent::ShortPress) {
        stateMachine.changeState(Waiting());
        Hal::instance().setBuiltinLed(false);
        return;
    }

    this->ledState = !this->ledState;
    Hal::instance().setBuiltinLed(this->ledState);
}
