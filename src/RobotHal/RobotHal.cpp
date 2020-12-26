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

RobotHal& RobotHal::instance() {
    static RobotHal instance;
    return instance;
}

RobotHal::RobotHal() {
    setup();
}

void RobotHal::setup() {
    printf("Initializing RobotHal\n");
    setupMotors();
    setupButtons();
    setupI2C();
    setupMisc();

    lineSensor.setup();
    imu.setup();
}

void RobotHal::setupMotors() {
    printf("Setting up motors\n");

    uint32_t i = 0;
    for (auto pins: RobotHalPins::encoder) {
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

void RobotHal::setupButtons() {
    //printf("Setting up buttons\n");
}

void RobotHal::setupI2C() {
    printf("Setting up I2C\n");
    i2c_config_t config = {
        .mode=I2C_MODE_MASTER,
        .sda_io_num = RobotHalPins::sda,
        .scl_io_num = RobotHalPins::scl,
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
        .pin_bit_mask = IdfUtil::bit64(RobotHalPins::indicatorLed),
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

void RobotHal::setBuiltinLed(bool enable) {
    gpio_set_level(IdfUtil::gpioPin(RobotHalPins::indicatorLed), static_cast<uint32_t>(enable));
}

int RobotHal::readRange() {
    return 0;
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
