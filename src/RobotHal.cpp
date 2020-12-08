#include "RobotHal.h"

#include "parameters.h"
#include "idf_util.h"

#include "driver/gpio.h"

#include <numeric>
#include <algorithm>

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
            [](auto accumulator, auto pin) { return accumulator | bit64(pin); }
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
        .pin_bit_mask = bit64(Pins::indicatorLed),
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
    uint32_t high = (pairIndex & 1) + 1;
    uint32_t low = pairIndex & 2;
    uint32_t highZ = 2 - pairIndex;

    if (ledIndex & 1)
        std::swap(low, high);

    auto mappedHigh = gpio_pin(Pins::lineLed[high]);
    auto mappedLow = gpio_pin(Pins::lineLed[low]);
    auto mappedHighZ = gpio_pin(Pins::lineLed[highZ]);

    gpio_set_direction(mappedHighZ, GPIO_MODE_DISABLE);
    gpio_set_direction(mappedLow, GPIO_MODE_OUTPUT);
    gpio_set_level(mappedLow, 0);
    gpio_set_direction(mappedHigh, GPIO_MODE_OUTPUT);
    gpio_set_level(mappedHigh, 1);
}

void RobotHal::disableLineSensorLed() {
    for (auto pin: Pins::lineLed)
        gpio_set_direction(gpio_pin(pin), GPIO_MODE_DISABLE);
}

void RobotHal::setBuiltinLed(bool enable) {
    gpio_set_level(gpio_pin(Pins::indicatorLed), static_cast<uint32_t>(enable));
}

int RobotHal::readRange() {
    return 0;
}

std::tuple<RobotHal::LineSensorT, RobotHal::LineSensorT> RobotHal::readLineSensor(LineSensorBufferT& buffer) {
    // TODO: Calibration
    // TODO: Make this asynchronous

    return std::make_tuple(0, 0);
    /*static_assert(std::tuple_size<LineSensorBufferT>::value == 8,
        "Must have exactly 8 pixels on the line sensor.");
    static_assert(lineSensorLedCount == 5,
        "Must have exactly 5 LEDs on the line sensor.");
    static_assert(lineSensorCount == 4,
        "Must have exactly 4 phototransistors on the line sensor.");

    for (uint8_t i = 0; i < RobotHal::lineSensorLedCount; ++i)
    {
        this->enableLineSensorLed(i);
        delayMicroseconds(Parameters::Hardware::lineSensorLedDelay); // Wait a bit for the LED to actually turn on.

        if (i == 2)
        {
            // Sensors 1 and 2 can be read in parallel
            auto result = adc.analogSyncRead(lineSensorFirst - 2, lineSensorFirst - 1);
            buffer[4] = result.result_adc0;
            buffer[3] = result.result_adc1;
        }
        else
        {
            if (i > 0)
                buffer[2 * i - 1] = adc.analogRead(lineSensorFirst - i + 1);
            if (i < RobotHal::lineSensorCount)
                buffer[2 * i] = adc.analogRead(lineSensorFirst - i);
        }
    }

    // Ambient light suppression
    this->disableLineSensorLed();
    delayMicroseconds(Parameters::Hardware::lineSensorLedDelay); // Wait a bit for the LED to actually turn off.

    auto minValue = std::numeric_limits<LineSensorT>::max();
    auto maxValue = std::numeric_limits<LineSensorT>::min();

    for (uint8_t i = 0; i < 2; ++i)
    {
        auto result = adc.analogSyncRead(lineSensorFirst - 2 - i, lineSensorFirst - i);
        for (uint8_t j = 0; j < 2; ++j)
        {
            buffer[2 * i + j + 4] -= result.result_adc0;
            minValue = std::min(buffer[2 * i + j + 4], minValue);
            maxValue = std::max(buffer[2 * i + j + 4], maxValue);

            buffer[2 * i + j] -= result.result_adc1;
            minValue = std::min(buffer[2 * i + j], minValue);
            maxValue = std::max(buffer[2 * i + j], maxValue);
        }
    }

    return std::make_tuple(minValue, maxValue);*/
}

float RobotHal::readBatteryVoltage()
{
    return 0.0f;
    //return analogRead(batteryVoltage) * batteryVoltsPerUnit;
}
