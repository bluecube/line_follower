#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "Hal.h"

void print_stuff(auto &hal) {
    printf("\n");

    typename decltype(hal.lineSensor)::BufferT buffer;
    hal.lineSensor.read(buffer);
    for (auto value: buffer)
        printf("%" PRId16 " ", value);
    printf("\n");
    auto encoders = hal.motors.readEncoders();
    printf("Encoders: %" PRId32 ", %" PRId32 "\n", encoders.first, encoders.second);
    printf("Battery voltage: %fV\n", hal.readBatteryVoltage());
    printf("accelerometer: %s, gyro: %s\n",
        hal.imu.readAccelerometer().str().c_str(),
        hal.imu.readGyro().str().c_str()
        );
    printf("temperature: %" PRId32 "\n", hal.imu.readTemperature());
    printf("deck state: %spressed\n", hal.deckButton.state() ? "" : "not ");
    printf("boot button state: %spressed\n", hal.bootButton.state() ? "" : "not ");
}

extern "C" void app_main(void)
{
    auto& hal = Hal::instance();

    for (uint32_t i = 0; i < hal.lineSensor.lineLedCount; ++i) {
        hal.lineSensor.enableLed(i);
        vTaskDelay(150 / portTICK_PERIOD_MS);
    }
    hal.lineSensor.disableLed();

    bool motorState = false;

    hal.bootButton.setReleaseCallback([&](auto) {
        motorState = !motorState;
        hal.setBuiltinLed(motorState);
        auto motorSpeed = motorState ? hal.motors.maxPwm() : 0;
        hal.motors.set(motorSpeed, motorSpeed);
    });

    int count = 0;
    hal.deckButton.setReleaseCallback([&](auto) {
        printf("\n%d:", count++);
        print_stuff(hal);
    });

    while(1) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
