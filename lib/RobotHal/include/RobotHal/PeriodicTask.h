#pragma once

#include <freertos/semphr.h>
#include <freertos/task.h>
#include <chrono>
#include <functional>

namespace RobotHal {

template <typename T>
class PeriodicTask {
public:
    PeriodicTask(const PeriodicTask&) = delete;
    PeriodicTask(PeriodicTask&&) = delete;
    PeriodicTask operator=(const PeriodicTask&) = delete;
    PeriodicTask operator=(PeriodicTask&&) = delete;

    template<typename F>
    void access(F f) {
        Lock lock(mutex);
        f(data);
    }
private:
    class Lock {
    public:
        Lock(SemaphoreHandle_t semaphore)
            :
            semaphore(semaphore)
        {
            xSemaphoreTake(semaphore, portMAX_DELAY);
        }
        ~Lock() {
            xSemaphoreGive(semaphore);
        }
    private:
        SemaphoreHandle_t semaphore;
    };


    template<typename F, typename Duration>
    PeriodicTask(T&& data_, F function, Duration period)
    :
        data(std::move(data_)),
        function(function)
    {
        mutex = xSemaphoreCreateMutex();
        xTaskCreate(
            task_trampoline,
            "Periodic task",
            4096, /* Stack size */
            this, /* parametetrs */
            20, /* Priority */
            &taskHandle /* Task handle */
        );

        // Configure and start the high-resolution timer
        esp_timer_create_args_t timerConfig = {
            .callback = timer_callback,
            .arg = taskHandle,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "Periodic task",
            .skip_unhandled_events = false,
        };
        ESP_ERROR_CHECK(esp_timer_create(&timerConfig, &timer));
        ESP_ERROR_CHECK(esp_timer_start_periodic(timer, duration_cast<std::chrono::microseconds>(period).count()));
    }

    static void task_trampoline(void *periodicTask) {
        static_cast<PeriodicTask<T>*>(periodicTask)->task_main();
    }

    static void timer_callback(void *taskHandle) {
        BaseType_t higherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(static_cast<TaskHandle_t>(taskHandle), &higherPriorityTaskWoken);
        portYIELD_FROM_ISR(higherPriorityTaskWoken);
    }

    void task_main() {
        while (true) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            Lock lock(mutex);
            function(data);
        }
    }

    T data;
    std::function<void(T&)> function;

    TaskHandle_t taskHandle;
    SemaphoreHandle_t mutex;
    esp_timer_handle_t timer;

    friend class Hal;
};

}
