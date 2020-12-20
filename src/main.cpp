/*#include <Arduino.h>
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
}*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "Hal.h"

extern "C" void app_main(void)
{
    auto& hal = Hal::instance();

    while(1) {
        /*
        for (uint32_t i = 0; i < 6; ++i) {
            printf("Line led %d\n", i);
            hal.enableLineSensorLed(i);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        hal.disableLineSensorLed();

        printf("Builtin\n");
        hal.setBuiltinLed(true);
        vTaskDelay(500 / portTICK_PERIOD_MS);

        printf("OFF\n");
        hal.setBuiltinLed(false);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        */

        Hal::LineSensorBufferT buffer;
        hal.readLineSensor(buffer);
        for (auto value: buffer)
            printf("%d ", value);
        printf("\n");
        auto encoders = hal.readMotorEncoders();
        printf("Encoders: %d, %d\n", encoders.first, encoders.second);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
