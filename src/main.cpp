#include <Arduino.h>
#include "StateMachine.h"
#include "Hal.h"
#include "parameters.h"

StateMachine stateMachine;
uint32_t lastUpdate;

void setup() {
    lastUpdate = millis();
}

void loop() {
    uint32_t now = millis();
    auto elapsed = now - lastUpdate;
    lastUpdate = now;

    stateMachine.update(elapsed);

    auto taken = millis() - now;
    if (taken < Parameters::Hardware::period)
        delay(Parameters::Hardware::period - taken);
}
