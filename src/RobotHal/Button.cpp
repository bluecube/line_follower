#include "Button.h"

#include "esp_log.h"

namespace RobotHal {

void Button::setup(IdfUtil::PinT pin) {
    gpio = IdfUtil::gpioPin(pin);

    gpio_config_t gpioConfig {
        .pin_bit_mask = IdfUtil::bit64(pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&gpioConfig));
    ESP_ERROR_CHECK(gpio_isr_handler_add(gpio, Button::isr_trampoline, this));

    esp_timer_create_args_t timerConfig = {
        .callback = Button::timer_trampoline,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "Button debounce",
        .skip_unhandled_events = false,
    };
    ESP_ERROR_CHECK(esp_timer_create(&timerConfig, &timer));

    eventQueue = xQueueCreate(8, sizeof(PressDuration));
    xTaskCreate(
        Button::event_task_trampoline,
        "Button event task",
        2048, /* Stack size */
        this, /* parametetrs */
        20, /* Priority */
        NULL /* Task handle */
    );

    lastState = false;
    timer_handler();
        // Timer handler samples the actual state of the button and enables the interrupt
        // It is a little random that timer handler is the right thing to do this sort of thing,
        // but I'll take it :)
}

bool Button::raw_state() const {
    return !gpio_get_level(gpio);
}

void Button::event_detected(bool newState) {
    lastState = newState;
    if (lastState) {
        pressTime = EspTimer::now();
        PressDuration zero{0};
        xQueueSendToBackFromISR(eventQueue, &zero, NULL);
    } else {
        auto elapsed = std::chrono::duration_cast<PressDuration>(EspTimer::now() - pressTime);
        xQueueSendToBackFromISR(eventQueue, &elapsed, NULL);
    }
}

void Button::isr_handler() {
    // ISR handler should always be called with the correct button lastState
    event_detected(!lastState);

    ESP_ERROR_CHECK(gpio_intr_disable(gpio));
    ESP_ERROR_CHECK(esp_timer_start_once(timer, std::chrono::duration_cast<std::chrono::microseconds>(debounce).count()));
}

void Button::timer_handler() {
    auto newState = raw_state();
    if (newState != lastState)
        event_detected(newState);

    ESP_ERROR_CHECK(gpio_set_intr_type(gpio, lastState ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE));
    ESP_ERROR_CHECK(gpio_intr_enable(gpio));
}

void Button::event_task() {
    while (true) {
        PressDuration pressDuration;
        if (xQueueReceive(eventQueue, &pressDuration, portMAX_DELAY) == pdTRUE) {
            if (pressDuration.count()) {
                if (releaseCallback)
                    releaseCallback(pressDuration);
            } else {
                if (pressCallback)
                    pressCallback();
            }
        }
    }
}

}
