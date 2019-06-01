#include "StateMachine.h"

StateMachine stateMachine;

void setup() {}

void loop() {
    stateMachine.update(100);
    delay(100);
}
