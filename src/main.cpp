#include "StateMachine.h"

StateMachine stateMachine;

void setup() {
    Serial.begin(9600);

    while (!Serial) {
        Hal::instance().setBuiltinLed(true);
        delay(50);
        Hal::instance().setBuiltinLed(false);
        delay(50);
    }
}

void loop() {
    stateMachine.update(100);
    delay(100);
}
