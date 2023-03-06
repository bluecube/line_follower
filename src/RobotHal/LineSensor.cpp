#include "LineSensor.h"

#include "Hal.h"
#include "idf_util.h"

#include <driver/gpio.h>

#include <cstdio>
#include <numeric>
#include <utility>

namespace RobotHal {

void LineSensor::setup() {
    static_assert(LineSensor::BufferT().size() ==  2 * Pins::lineSensor.size(),
        "Buffer size must be twice the number of line sensor elements (each element illuminated from both sides)");
    static_assert(Pins::lineSensor.size() == lineLedCount  - 1,
        "There must be exactly one line sensor between every two charlieplexed line leds.");
    static_assert(Pins::lineSensor.size() & 1,
        "Number of line sensors must be odd (so that the last sensor is ADC1 again).");

    printf("Setting up line sensor\n");
    gpio_config_t config = {
        .pin_bit_mask = std::accumulate(
            Pins::lineLed.begin(), Pins::lineLed.end(), uint64_t(0),
            [](auto accumulator, auto pin) { return accumulator | IdfUtil::bit64(pin); }
        ),
        .mode = GPIO_MODE_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&config);
    // All pins are set to drive strength 2 by default and that is what we want
    // for the line sensors

    adc_oneshot_chan_cfg_t chanConfig = {
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = Hal::adcWidth,
    };
    for (uint32_t i = 0u; i < Pins::lineSensor.size(); i += 1) {
        auto [unit, channel] = IdfUtil::gpioToADCChannel(Pins::batSense);
        adc_oneshot_config_channel(hal.adcHandles[unit], channel, &chanConfig);
    }
}

void LineSensor::enableLed(uint32_t ledIndex) {
    static_assert(
        Pins::lineLed.size() == 3,
        "Charlieplexing code here is hardcoded for three LED lines"
    );

    uint32_t pairIndex = ledIndex >> 1;
    uint32_t low = pairIndex & 2;
    uint32_t high = (pairIndex & 1) + 1;
    uint32_t highZ = 2 - pairIndex;

    bool firstInPair = !(ledIndex & 1);

    auto mappedLow = IdfUtil::gpioPin(Pins::lineLed[firstInPair ? low : high]);
    auto mappedHigh = IdfUtil::gpioPin(Pins::lineLed[firstInPair ? high : low]);
    auto mappedHighZ = IdfUtil::gpioPin(Pins::lineLed[highZ]);

    gpio_set_direction(mappedHighZ, GPIO_MODE_DISABLE);
    gpio_set_direction(mappedLow, GPIO_MODE_OUTPUT);
    gpio_set_level(mappedLow, 0);
    gpio_set_direction(mappedHigh, GPIO_MODE_OUTPUT);
    gpio_set_level(mappedHigh, 1);
}

void LineSensor::disableLed() {
    for (auto pin: Pins::lineLed)
        gpio_set_direction(IdfUtil::gpioPin(pin), GPIO_MODE_DISABLE);
}

std::pair<LineSensor::ValueT, LineSensor::ValueT>
LineSensor::read(LineSensor::BufferT& buffer) {
    // TODO: Calibration
    // TODO: Make this asynchronous

    read_leds(
        [&](auto i) {
            this->enableLed(i);
            this->settle();
        },
        [&](auto i, auto v)
        {
            buffer[i] = v;
        }
    );

    // Ambient light suppression and buffer statistics
    this->disableLed();
    this->settle();

    auto minValue = std::numeric_limits<ValueT>::max();
    auto maxValue = std::numeric_limits<ValueT>::min();

    read_no_leds(
        [&](auto i, auto v)
        {
            buffer[i] -= v;
            minValue = std::min(minValue, buffer[i]);
            maxValue = std::max(maxValue, buffer[i]);
        }
    );

    return std::make_pair(minValue, maxValue);
}

void LineSensor::calibrate() {
    printf("Line sensor calibration\n");

    BufferT buffer;
    read_leds(
        [&](auto i) {
            this->enableLed(i);
            this->settle();
        },
        [&](auto i, auto v)
        {
            buffer[i] = v;
        }
    );

}

std::pair<LineSensor::ValueT, LineSensor::ValueT>
LineSensor::readAdcPair(int ch1Sensor, int ch2Sensor) {
    auto v1 = readAdc(ch1Sensor);
    auto v2 = readAdc(ch2Sensor);
    // TODO: Actually do the reads in parallel

    return std::make_pair(v1, v2);
}

LineSensor::ValueT LineSensor::readAdc(int sensor) {
    auto [unit, channel] = IdfUtil::gpioToADCChannel(Pins::lineSensor[sensor]);
    int value;
    ESP_ERROR_CHECK(adc_oneshot_read(hal.adcHandles[unit], channel, &value));
    return value;
}

template <typename LedFn, typename OutputFn>
inline void LineSensor::read_leds(LedFn ledFn, OutputFn outputFn) {
    // First LED only illuminates a single line sensor
    ledFn(0);
    outputFn(0, readAdc(0));

    for (uint32_t i = 1u; i < (lineLedCount - 1u); ++i)
    {
        ledFn(i);

        if (i & 1) {
            auto [v1, v2] = readAdcPair(i - 1, i);
            outputFn(2 * i - 1, v1);
            outputFn(2 * i, v2);
        } else {
            auto [v2, v1] = readAdcPair(i, i - 1);
            outputFn(2 * i - 1, v1);
            outputFn(2 * i, v2);
        }
    }

    // Last LED only illuminates a single line sensor
    ledFn(lineLedCount - 1);
    outputFn(2 * Pins::lineSensor.size() - 1, readAdc(Pins::lineSensor.size() - 1));
}

template <typename OutputFn>
inline void LineSensor::read_no_leds(OutputFn outputFn) {
    for (uint32_t i = 0u; i < (Pins::lineSensor.size() - 1); i += 2)
    {
        auto [v1, v2] = readAdcPair(i, i + 1);
        outputFn(2 * i + 0, v1);
        outputFn(2 * i + 1, v1);
        outputFn(2 * i + 2, v2);
        outputFn(2 * i + 3, v2);
    }
    auto v = readAdc(Pins::lineSensor.size() - 1);
    outputFn(2 * Pins::lineSensor.size() - 2, v);
    outputFn(2 * Pins::lineSensor.size() - 1, v);
}

void LineSensor::settle() {
    vTaskDelay(1 / portTICK_PERIOD_MS); // TODO: Old version used just 250us
}

}
