#include "ErrorState.h"
#include "Hal.h"
#include "StateMachine.h"

ErrorState::ErrorState(uint8_t code)
{
    Hal::instance().enableLineSensorLed(code);
}

void ErrorState::update(StateMachine& stateMachine, Hal::MillisecondsT elapsed) {
    this->ledState = !this->ledState;
    Hal::instance().setBuiltinLed(this->ledState);
}
