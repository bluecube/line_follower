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

        decltype(hal.lineSensor)::BufferT buffer;
        hal.lineSensor.read(buffer);
        for (auto value: buffer)
            printf("%d ", value);
        printf("\n");
        auto encoders = hal.motors.readEncoders();
        printf("Encoders: %d, %d\n", encoders.first, encoders.second);
        printf("Battery voltage: %fV\n", hal.readBatteryVoltage());
        //printf("accelerometer: %s\n", hal.imu.readAccelerometer().str().c_str());
        //printf("temperature: %d\n", hal.imu.readTemperature());
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
