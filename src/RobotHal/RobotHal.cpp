#include "RobotHal.h"

#include "parameters.h"
#include "idf_util.h"

#include <driver/gpio.h>
#include <driver/adc.h>
#include <driver/pcnt.h>
#include <driver/i2c.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cstdio>
#include <limits>
#include <numeric>

RobotHal& RobotHal::instance() {
    static RobotHal instance;
    return instance;
}

RobotHal::RobotHal()
: imu(*this) {
    setup();
}

void RobotHal::setup() {
    printf("Initializing RobotHal\n");
    setupMotors();
    setupLineSensor();
    setupButtons();
    setupI2C();
    imu.setup();
    setupMisc();
}

void RobotHal::setupMotors() {
    printf("Setting up motors\n");

    uint32_t i = 0;
    for (auto pins: Pins::encoder) {
        printf("Setting up unit %d at pins %d,%d\n", i, pins.first, pins.second);
        // Configure both channels of the counter to the same pair of pins,
        // once with first pin as pulse and second as control, then the other way
        // around.
        // This gives us full 4 ticks per revolution of the encoder.
        // TODO: Isn't this actually too much?
        pcnt_config_t config = {
            .pulse_gpio_num = pins.first,
            .ctrl_gpio_num = pins.second,
            .lctrl_mode = PCNT_MODE_REVERSE,
            .hctrl_mode = PCNT_MODE_KEEP,
            .pos_mode = PCNT_COUNT_INC,
            .neg_mode = PCNT_COUNT_DEC,
            .counter_h_lim = std::numeric_limits<int16_t>::max(),
            .counter_l_lim = std::numeric_limits<int16_t>::min(),
            .unit = static_cast<pcnt_unit_t>(i++),
            .channel = PCNT_CHANNEL_0
        };
        pcnt_unit_config(&config);

        printf("Second channel\n");

        config.pulse_gpio_num = pins.second;
        config.ctrl_gpio_num = pins.first;
        config.lctrl_mode = PCNT_MODE_KEEP;
        config.hctrl_mode = PCNT_MODE_REVERSE;
        config.channel = PCNT_CHANNEL_1;
        pcnt_unit_config(&config);

        // This seems to be necessary for the counter to start working
        pcnt_counter_clear(config.unit);
    }
}

void RobotHal::setupLineSensor() {
    printf("Setting up line sensor\n");
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

    adc1_config_width(adcWidth);
    setLineSensorAttenuation(ADC_ATTEN_DB_11);
}

void RobotHal::setLineSensorAttenuation(adc_atten_t attenuation)
{
    for (uint32_t i = 0u; i < Pins::lineSensor.size(); i += 2) {
        auto ch = IdfUtil::adc1Pin(Pins::lineSensor[i]);
        adc1_config_channel_atten(ch, attenuation);
    }
    for (uint32_t i = 1u; i < Pins::lineSensor.size(); i += 2) {
        auto ch = IdfUtil::adc2Pin(Pins::lineSensor[i]);
        adc2_config_channel_atten(ch, attenuation);
    }
}

void RobotHal::setupButtons() {
    //printf("Setting up buttons\n");
}

void RobotHal::setupI2C() {
    printf("Setting up I2C\n");
    i2c_config_t config = {
        .mode=I2C_MODE_MASTER,
        .sda_io_num = Pins::sda,
        .scl_io_num = Pins::scl,
        .sda_pullup_en = true,
        .scl_pullup_en = true,
        .master = {
            .clk_speed = 400'000, // Maximum supported by MPU6050
        }
    };
    i2c_param_config(I2C_NUM_0, &config);
    i2c_driver_install(
        I2C_NUM_0,
        I2C_MODE_MASTER,
        0 /* slave RX buffer size */,
        0 /* slave TX buffer size */,
        0 /* interrupt alloc flags */
    );
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

void RobotHal::calibrateLineSensor() {
    printf("Sensor calibration\n");

    LineSensorBufferT buffer;
    readLineSensor(
        [&](uint32_t i) {
            printf("LED %d on\n", i);
            this->enableLineSensorLed(i);
            this->lineSensorSettle();
        },
        [&](uint32_t i, LineSensorT v)
        {
            printf("buffer[%d] = %d\n", i, v);
            buffer[i] = v;
        }
    );

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

std::pair<int16_t, int16_t> RobotHal::readMotorEncoders() const
{
    int16_t v1 = 0xffff;
    int16_t v2 = 0xffff;
    pcnt_get_counter_value(PCNT_UNIT_0, &v1);
    pcnt_get_counter_value(PCNT_UNIT_1, &v2);
    return std::make_pair(v1, v2);
}

void RobotHal::i2cRead(
    uint8_t deviceAddress, uint8_t registerAddress,
    uint8_t* data, size_t count
) {
    auto cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceAddress << 1) | I2C_MASTER_READ, true);
    i2c_master_write_byte(cmd, registerAddress, true);
    i2c_master_read(cmd, data, count, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
}

void RobotHal::i2cWrite(
    uint8_t deviceAddress, uint8_t registerAddress,
    const uint8_t* data, size_t count
) {
    auto cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceAddress << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, registerAddress, true);
    i2c_master_write(cmd, const_cast<uint8_t*>(data), count, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
}
