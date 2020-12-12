#include "RobotHal.h"

#include "parameters.h"
#include "idf_util.h"

#include <driver/gpio.h>
#include <driver/adc.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <limits>
#include <numeric>

RobotHal& RobotHal::instance() {
    static RobotHal instance;
    return instance;
}

RobotHal::RobotHal() {
    this->setupMotors();
    this->setupLineSensor();
    this->setupButtons();
    this->setupIMU();
    this->setupMisc();
}

void RobotHal::setupMotors() {
}

void RobotHal::setupLineSensor() {
    gpio_config_t config = {
        .pin_bit_mask = std::accumulate(
            Pins::lineLed.begin(), Pins::lineLed.end(), uint64_t(0),
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
}

void RobotHal::setupButtons() {
}

void RobotHal::setupIMU() {

}

void RobotHal::setupMisc() {
    gpio_config_t config = {
        .pin_bit_mask = IdfUtil::bit64(Pins::indicatorLed),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&config);
}

void RobotHal::setMotors(PwmT left, PwmT right) {
    (void)left;
    (void)right;
}

void RobotHal::enableLineSensorLed(uint32_t ledIndex) {
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

void RobotHal::disableLineSensorLed() {
    for (auto pin: Pins::lineLed)
        gpio_set_direction(IdfUtil::gpioPin(pin), GPIO_MODE_DISABLE);
}

void RobotHal::setBuiltinLed(bool enable) {
    gpio_set_level(IdfUtil::gpioPin(Pins::indicatorLed), static_cast<uint32_t>(enable));
}

int RobotHal::readRange() {
    return 0;
}

std::tuple<RobotHal::LineSensorT, RobotHal::LineSensorT> RobotHal::readLineSensor(RobotHal::LineSensorBufferT& buffer) {
    // TODO: Calibration
    // TODO: Make this asynchronous

    readLineSensor(
        [&](uint32_t i) {
            this->enableLineSensorLed(i);
            this->lineSensorSettle();
        },
        [&](uint32_t i, LineSensorT v)
        {
            buffer[i] = v;
        }
    );

    // Ambient light suppression and buffer statistics
    this->disableLineSensorLed();
    this->lineSensorSettle();

    LineSensorT minValue = std::numeric_limits<LineSensorT>::max();
    LineSensorT maxValue = 0;

    readLineSensor(
        [&](uint32_t i, LineSensorT v)
        {
            buffer[i] -= v;
            minValue = std::min(minValue, buffer[i]);
            maxValue = std::max(maxValue, buffer[i]);
        }
    );

    return std::make_pair(minValue, maxValue);
}

float RobotHal::readBatteryVoltage() {
    return 0.0f;
    //return analogRead(batteryVoltage) * batteryVoltsPerUnit;
}

std::pair<int32_t, int32_t> RobotHal::readAdcPair(adc1_channel_t ch1, adc2_channel_t ch2) {
    int32_t v1 = adc1_get_raw(ch1);
    int32_t v2;
    adc2_get_raw(ch2, adcWidth, &v2);
    // TODO: Actually do the reads in parallel
    return std::make_pair(v1, v2);
}

std::pair<RobotHal::LineSensorT, RobotHal::LineSensorT> RobotHal::readAdcLineSensorPair(int ch1Sensor, int ch2Sensor) {
    return readAdcPair(
        IdfUtil::adc1Pin(Pins::lineSensor[ch1Sensor]),
        IdfUtil::adc2Pin(Pins::lineSensor[ch2Sensor])
    );
}

RobotHal::LineSensorT RobotHal::readAdc1LineSensor(int ch1Sensor) {
    return adc1_get_raw(IdfUtil::adc1Pin(Pins::lineSensor[ch1Sensor]));
}

/// Read ADC on every location of the line sensor.
template <typename LedFn, typename OutputFn>
inline void RobotHal::readLineSensor(LedFn ledFn, OutputFn outputFn) {
    static_assert(Pins::lineSensor.size() == Pins::lineLedCount  - 1,
        "There must be exactly one line sensor between every two charlieplexed line leds.");
    static_assert(Pins::lineSensor.size() & 1,
        "Number of line sensors must be odd (so that the last sensor is ADC1 again).");

    // First LED only illuminates a single line sensor
    ledFn(0);
    outputFn(0, readAdc1LineSensor(0));

    for (uint32_t i = 1u; i < (Pins::lineLedCount - 1u); ++i)
    {
        ledFn(i);

        if (i & 1) {
            auto [v1, v2] = readAdcLineSensorPair(i - 1, i);
            outputFn(2 * i - 1, v1);
            outputFn(2 * i, v2);
        } else {
            auto [v2, v1] = readAdcLineSensorPair(i, i - 1);
            outputFn(2 * i - 1, v1);
            outputFn(2 * i, v2);
        }
    }

    // Last LED only illuminates a single line sensor
    ledFn(Pins::lineLedCount - 1);
    outputFn(2 * Pins::lineSensor.size() - 1, readAdc1LineSensor(Pins::lineSensor.size() - 1));
}

template <typename OutputFn>
inline void RobotHal::readLineSensor(OutputFn outputFn) {
    static_assert(Pins::lineSensor.size() == Pins::lineLedCount  - 1,
        "There must be exactly one line sensor between every two charlieplexed line leds.");
    static_assert(Pins::lineSensor.size() & 1,
        "Number of line sensors must be odd (so that the last sensor is ADC1 again).");

    for (uint32_t i = 0u; i < (Pins::lineSensor.size() - 1); i += 2)
    {
        auto [v1, v2] = readAdcLineSensorPair(i, i + 1);
        outputFn(2 * i + 0, v1);
        outputFn(2 * i + 1, v1);
        outputFn(2 * i + 2, v2);
        outputFn(2 * i + 3, v2);
    }
    auto v = readAdc1LineSensor(Pins::lineSensor.size() - 1);
    outputFn(2 * Pins::lineSensor.size() - 2, v);
    outputFn(2 * Pins::lineSensor.size() - 1, v);
}

void RobotHal::lineSensorSettle() {
    vTaskDelay(1 / portTICK_PERIOD_MS); // TODO: Old version used just 250us
}
