#pragma once

#include "driver/gpio.h"

#include <cstdint>

static constexpr gpio_num_t gpio_pin(int pin) {
    return static_cast<gpio_num_t>(pin);
}

static constexpr uint64_t bit64(int pin) {
    return uint64_t(1) << pin;
}
