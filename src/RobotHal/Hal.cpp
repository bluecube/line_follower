#include "Hal.h"

#include "parameters.h"
#include "idf_util.h"

#include <driver/gpio.h>
#include <driver/i2c.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cstdio>

namespace RobotHal {

Hal& Hal::instance() {
    static Hal instance;
    return instance;
}

Hal::Hal() {
    setup();
}

void Hal::setup() {
    printf("Initializing Hal\n");
    setupButtons();
    setupI2C();
    setupMisc();

    lineSensor.setup();
    motors.setup();
    imu.setup();
    printf("Hal is ready\n");
}

void Hal::setupButtons() {
    //printf("Setting up buttons\n");
}

void Hal::setupI2C() {
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

void Hal::setupMisc() {
    printf("Setting up misc board functions\n");
    // Built-in LED
    gpio_config_t config = {
        .pin_bit_mask = IdfUtil::bit64(Pins::indicatorLed),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&config);

    // Battery voltage
    adc2_config_channel_atten(IdfUtil::adc2Pin(Pins::batSense), ADC_ATTEN_DB_11);
}

void Hal::setBuiltinLed(bool enable) {
    gpio_set_level(IdfUtil::gpioPin(Pins::indicatorLed), static_cast<uint32_t>(enable));
}

int Hal::readRange() {
    return 0;
}

float Hal::readBatteryVoltage() {
    int32_t raw;
    adc2_get_raw(IdfUtil::adc2Pin(Pins::batSense), adcWidth, &raw);

    // Uncomment this code and repeatedly call this function to get calibration values
    /*
    static float smoothedRaw = 0.0f;
    static uint32_t valueCount = 0;
    smoothedRaw = smoothedRaw + (raw - smoothedRaw) / (++valueCount);
    printf("Raw voltage measurement: %d, smoothed: %.2f\n", raw, smoothedRaw);
    */

    static constexpr float k =
        (batteryVoltageCalibration[1].second - batteryVoltageCalibration[0].second) /
        (batteryVoltageCalibration[1].first - batteryVoltageCalibration[0].first);
    static constexpr float a =
        batteryVoltageCalibration[0].second - k * batteryVoltageCalibration[0].first;

    return k * raw + a;
}

std::pair<int32_t, int32_t> Hal::readAdcPair(adc1_channel_t ch1, adc2_channel_t ch2) {
    int32_t v1 = adc1_get_raw(ch1);
    int32_t v2;
    adc2_get_raw(ch2, adcWidth, &v2);
    // TODO: Actually do the reads in parallel
    return std::make_pair(v1, v2);
}

void Hal::i2cRead(
    uint8_t deviceAddress, uint8_t registerAddress,
    uint8_t* data, size_t count
) {
    auto cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceAddress << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, registerAddress, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceAddress << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, count, I2C_MASTER_LAST_NACK);

    i2c_master_stop(cmd);

    HAL_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS));

    i2c_cmd_link_delete(cmd);
}

void Hal::i2cWrite(
    uint8_t deviceAddress, uint8_t registerAddress,
    const uint8_t* data, size_t count
) {
    auto cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceAddress << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, registerAddress, true);
    i2c_master_write(cmd, const_cast<uint8_t*>(data), count, true);
    i2c_master_stop(cmd);

    HAL_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS));

    i2c_cmd_link_delete(cmd);
}

}
