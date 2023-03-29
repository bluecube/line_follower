#pragma once
#include "idf_util.h"
#include "EspTimer.h"

#include <driver/pulse_cnt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <functional>

namespace RobotHal {

class Hal;

class Button {
    Button() {}
public:
    Button(const Button&) = delete;
    Button(Button&&) = delete;
    Button& operator=(const Button&) = delete;
    Button& operator=(Button&&) = delete;

    using PressDuration = std::chrono::duration<uint32_t, std::milli>;

    void setPressCallback(std::function<void()> cb) { pressCallback = cb; }
    void setReleaseCallback(std::function<void(PressDuration)> cb) { releaseCallback = cb; }

    /// Returns debounced state of the button.
    bool state() const { return lastState; }

    /// Returns the state of the button, but might include bounce artefactcs.
    bool raw_state() const;
private:
    void setup(IdfUtil::PinT pin);

    void isr_handler();
    void timer_handler();
    [[noreturn]] void event_task();

    /// Handle a change of state.
    /// Measures time since press and dispatches callbacks.
    void event_detected(bool newState);

    /// Static typeless wrapper for use as a ESP-IDF ISR handler
    static void isr_trampoline(void *button) {
        static_cast<Button *>(button)->isr_handler();
    }

    /// Static typeless wrapper for use as a ESP-IDF timer handler
    static void timer_trampoline(void *button) {
        static_cast<Button *>(button)->timer_handler();
    }

    static void event_task_trampoline [[noreturn]] (void *button) {
        static_cast<Button *>(button)->event_task();
    }

    /// How long to ignore everything after registering an event.
    static constexpr std::chrono::milliseconds debounce{50};

    gpio_num_t gpio;
    esp_timer_handle_t timer;
    QueueHandle_t eventQueue;

    /// Last state of the button
    bool lastState;

    /// When was the button last pressed
    EspTimer::time_point pressTime;

    std::function<void()> pressCallback;
    std::function<void(PressDuration)> releaseCallback;

    friend class Hal;
};

}
