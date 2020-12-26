#include "RobotHalLineSensor.h"
#include "RobotHal.h"

#include "idf_util.h"

#include <driver/gpio.h>
#include <driver/adc.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cstdio>
#include <numeric>
#include <utility>


void RobotHalLineSensor::setup() {
    static_assert(RobotHalLineSensor::BufferT().size() ==  2 * RobotHalPins::lineSensor.size(),
        "Buffer size must be twice the number of line sensor elements (each element illuminated from both sides)");
    static_assert(RobotHalPins::lineSensor.size() == lineLedCount  - 1,
        "There must be exactly one line sensor between every two charlieplexed line leds.");
    static_assert(RobotHalPins::lineSensor.size() & 1,
        "Number of line sensors must be odd (so that the last sensor is ADC1 again).");

    printf("Setting up line sensor\n");
    gpio_config_t config = {
        .pin_bit_mask = std::accumulate(
            RobotHalPins::lineLed.begin(), RobotHalPins::lineLed.end(), uint64_t(0),
            [](auto accumulator, auto pin) { return accumulator | IdfUtil::bit64(pin); }
        ),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&config);
    // All pins are set to drive strength 2 by default and that is what we want
    // for the line sensors

    adc1_config_width(RobotHal::adcWidth);
    setAttenuation(ADC_ATTEN_DB_11);
}

void RobotHalLineSensor::enableLed(uint32_t ledIndex) {
    static_assert(
        RobotHalPins::lineLed.size() == 3,
        "Charlieplexing code here is hardcoded for three LED lines"
    );

    uint32_t pairIndex = ledIndex >> 1;
    uint32_t low = pairIndex & 2;
    uint32_t high = (pairIndex & 1) + 1;
    uint32_t highZ = 2 - pairIndex;

    bool firstInPair = !(ledIndex & 1);

    auto mappedLow = IdfUtil::gpioPin(RobotHalPins::lineLed[firstInPair ? low : high]);
    auto mappedHigh = IdfUtil::gpioPin(RobotHalPins::lineLed[firstInPair ? high : low]);
    auto mappedHighZ = IdfUtil::gpioPin(RobotHalPins::lineLed[highZ]);

    gpio_set_direction(mappedHighZ, GPIO_MODE_DISABLE);
    gpio_set_direction(mappedLow, GPIO_MODE_OUTPUT);
    gpio_set_level(mappedLow, 0);
    gpio_set_direction(mappedHigh, GPIO_MODE_OUTPUT);
    gpio_set_level(mappedHigh, 1);
}

void RobotHalLineSensor::disableLed() {
    for (auto pin: RobotHalPins::lineLed)
        gpio_set_direction(IdfUtil::gpioPin(pin), GPIO_MODE_DISABLE);
}

std::pair<RobotHalLineSensor::ValueT, RobotHalLineSensor::ValueT>
RobotHalLineSensor::read(RobotHalLineSensor::BufferT& buffer) {
    // TODO: Calibration
    // TODO: Make this asynchronous

    read(
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

    read(
        [&](auto i, auto v)
        {
            buffer[i] -= v;
            minValue = std::min(minValue, buffer[i]);
            maxValue = std::max(maxValue, buffer[i]);
        }
    );

    return std::make_pair(minValue, maxValue);
}

void RobotHalLineSensor::calibrate() {
    printf("Line sensor calibration\n");

    BufferT buffer;
    read(
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

std::pair<RobotHalLineSensor::ValueT, RobotHalLineSensor::ValueT>
RobotHalLineSensor::readAdcPair(int ch1Sensor, int ch2Sensor) {
    return RobotHal::readAdcPair(
        IdfUtil::adc1Pin(RobotHalPins::lineSensor[ch1Sensor]),
        IdfUtil::adc2Pin(RobotHalPins::lineSensor[ch2Sensor])
    );
}

RobotHalLineSensor::ValueT RobotHalLineSensor::readAdc1(int ch1Sensor) {
    return adc1_get_raw(IdfUtil::adc1Pin(RobotHalPins::lineSensor[ch1Sensor]));
}

/// Read ADC on every location of the line sensor.
template <typename LedFn, typename OutputFn>
inline void RobotHalLineSensor::read(LedFn ledFn, OutputFn outputFn) {
    // First LED only illuminates a single line sensor
    ledFn(0);
    outputFn(0, readAdc1(0));

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
    outputFn(2 * RobotHalPins::lineSensor.size() - 1, readAdc1(RobotHalPins::lineSensor.size() - 1));
}

template <typename OutputFn>
inline void RobotHalLineSensor::read(OutputFn outputFn) {
    for (uint32_t i = 0u; i < (RobotHalPins::lineSensor.size() - 1); i += 2)
    {
        auto [v1, v2] = readAdcPair(i, i + 1);
        outputFn(2 * i + 0, v1);
        outputFn(2 * i + 1, v1);
        outputFn(2 * i + 2, v2);
        outputFn(2 * i + 3, v2);
    }
    auto v = readAdc1(RobotHalPins::lineSensor.size() - 1);
    outputFn(2 * RobotHalPins::lineSensor.size() - 2, v);
    outputFn(2 * RobotHalPins::lineSensor.size() - 1, v);
}

void RobotHalLineSensor::settle() {
    vTaskDelay(1 / portTICK_PERIOD_MS); // TODO: Old version used just 250us
}

void RobotHalLineSensor::setAttenuation(adc_atten_t attenuation)
{
    for (uint32_t i = 0u; i < RobotHalPins::lineSensor.size(); i += 2) {
        auto ch = IdfUtil::adc1Pin(RobotHalPins::lineSensor[i]);
        adc1_config_channel_atten(ch, attenuation);
    }
    for (uint32_t i = 1u; i < RobotHalPins::lineSensor.size(); i += 2) {
        auto ch = IdfUtil::adc2Pin(RobotHalPins::lineSensor[i]);
        adc2_config_channel_atten(ch, attenuation);
    }
}

