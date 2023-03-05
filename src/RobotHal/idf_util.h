#pragma once

#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_adc/adc_oneshot.h>

#include <cstdint>
#include <utility>

namespace IdfUtil {

using PinT = int; // Int to match some of the ESP-IDF function signatures.

constexpr gpio_num_t gpioPin(PinT pin) {
    return static_cast<gpio_num_t>(pin);
}

constexpr uint64_t bit64(int pin) {
    return uint64_t(1) << pin;
}

static inline std::pair<adc_unit_t, adc_channel_t> gpioToADCChannel(PinT pin) {
    adc_unit_t unit;
    adc_channel_t channel;
    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(pin, &unit, &channel));
    return std::make_pair(unit, channel);
}

}
