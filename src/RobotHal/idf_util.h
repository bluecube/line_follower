#pragma once

#include <driver/gpio.h>
#include <driver/adc.h>
#include <esp_err.h>

#include <cstdint>

namespace IdfUtil {

#define HAL_CHECK(value) \
    do { \
        esp_err_t code = (value); \
        if (code != ESP_OK) \
            printf("%s:%d: ESP call failed: %s\n", __FILE__, __LINE__, esp_err_to_name(code)); \
    } while(0)

using PinT = int; // Int to match some of the ESP-IDF function signatures.

constexpr gpio_num_t gpioPin(PinT pin) {
    return static_cast<gpio_num_t>(pin);
}

constexpr uint64_t bit64(int pin) {
    return uint64_t(1) << pin;
}

constexpr adc1_channel_t adc1Pin(PinT pin) {
    switch (pin) {
    case 36: return ADC1_GPIO36_CHANNEL;
    case 37: return ADC1_GPIO37_CHANNEL;
    case 38: return ADC1_GPIO38_CHANNEL;
    case 39: return ADC1_GPIO39_CHANNEL;
    case 32: return ADC1_GPIO32_CHANNEL;
    case 33: return ADC1_GPIO33_CHANNEL;
    case 34: return ADC1_GPIO34_CHANNEL;
    case 35: return ADC1_GPIO35_CHANNEL;
    default: __builtin_unreachable();
    }
}

constexpr adc2_channel_t adc2Pin(PinT pin) {
    switch (pin) {
    case 4: return ADC2_GPIO4_CHANNEL;
    case 0: return ADC2_GPIO0_CHANNEL;
    case 2: return ADC2_GPIO2_CHANNEL;
    case 15: return ADC2_GPIO15_CHANNEL;
    case 13: return ADC2_GPIO13_CHANNEL;
    case 12: return ADC2_GPIO12_CHANNEL;
    case 14: return ADC2_GPIO14_CHANNEL;
    case 27: return ADC2_GPIO27_CHANNEL;
    case 25: return ADC2_GPIO25_CHANNEL;
    case 26: return ADC2_GPIO26_CHANNEL;
    default: __builtin_unreachable();
    }
}

}
