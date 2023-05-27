#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "MotorVelocityControler.h"

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
    printf("\n");
    printf("Velocity unit: %f\n", MotorVelocityControlerTask::velocityUnit);
}

extern "C" void app_main(void)
{
    auto& hal = Hal::instance();

    MotorVelocityControlerTask mvc(hal, 8192, 1024, 0);

    int32_t velocityReq = 0.02 / MotorVelocityControlerTask::velocityUnit;
    mvc.set_requested_velocity(velocityReq, velocityReq);
    /*auto task = hal.start_periodic_task(
        0.02,
        [&](auto& velocity) {
            int32_t velocityReq = velocity / MotorVelocityControlerTask::velocityUnit;
            printf("Setting velocity to %f m/s (%ld units)\n", velocity, velocityReq);
            mvc.set_requested_velocity(velocityReq, velocityReq);
            velocity *= -1;
        },
        std::chrono::seconds(5)
    );*/

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
