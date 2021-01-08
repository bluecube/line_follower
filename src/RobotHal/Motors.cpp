#include "Motors.h"

#include "Pins.h"

#include <driver/pcnt.h>

#include <cstdio>
#include <limits>

namespace RobotHal {

void Motors::setup() {
    printf("Setting up motors\n");

    uint32_t i = 0;
    for (auto pins: Pins::encoder) {
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

        config.pulse_gpio_num = pins.second;
        config.ctrl_gpio_num = pins.first;
        config.lctrl_mode = PCNT_MODE_KEEP;
        config.hctrl_mode = PCNT_MODE_REVERSE;
        config.channel = PCNT_CHANNEL_1;
        pcnt_unit_config(&config);

        // This seems to be necessary for the counter to start working
        pcnt_counter_clear(config.unit);
    }

    setupMotors();
}

void Motors::setupMotors() {
    uint32_t i = 0;
    for (auto pins: Pins::motor) {
        auto mcpwmUnit = static_cast<mcpwm_unit_t>(i++);
        mcpwm_gpio_init(mcpwmUnit, MCPWM0A, pins.first);
        mcpwm_gpio_init(mcpwmUnit, MCPWM0B, pins.second);
        mcpwm_config_t config = {
            .frequency = 25000,
            .cmpr_a = 0,
            .cmpr_b = 0,
            .duty_mode = MCPWM_DUTY_MODE_0,
            .counter_mode = MCPWM_UP_COUNTER
        };
        HAL_CHECK(mcpwm_init(mcpwmUnit, MCPWM_TIMER_0, &config));
        HAL_CHECK(mcpwm_start(mcpwmUnit, MCPWM_TIMER_0));
    }
}

void Motors::startBeep(uint8_t leftPitch, uint8_t rightPitch) {
    uint32_t i = 0;
    std::array<uint8_t, 2> pitches = {leftPitch, rightPitch};
    for (auto pins: Pins::motor) {
        auto mcpwmUnit = static_cast<mcpwm_unit_t>(i);
        mcpwm_gpio_init(mcpwmUnit, MCPWM0A, pins.first);
        mcpwm_gpio_init(mcpwmUnit, MCPWM0B, pins.second);
        mcpwm_config_t config = {
            .frequency = beepPitchToFrequency(pitches[i]),
            .cmpr_a = 50,
            .cmpr_b = 50,
            .duty_mode = MCPWM_DUTY_MODE_0,
            .counter_mode = MCPWM_UP_COUNTER
        };
        HAL_CHECK(mcpwm_init(mcpwmUnit, MCPWM_TIMER_0, &config));
        mcpwm_set_duty_type(mcpwmUnit, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
        mcpwm_set_duty_type(mcpwmUnit, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_1);
        HAL_CHECK(mcpwm_start(mcpwmUnit, MCPWM_TIMER_0));

        ++i;
    }
}

std::pair<int16_t, int16_t> Motors::readEncoders() const
{
    int16_t v1;
    int16_t v2;
    pcnt_get_counter_value(PCNT_UNIT_0, &v1);
    pcnt_get_counter_value(PCNT_UNIT_1, &v2);
    return std::make_pair(v1, v2);
}

void Motors::set(mcpwm_unit_t unit, float duty) {
    if (duty > 0.0f) {
        mcpwm_set_duty(unit, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
        mcpwm_set_duty_type(unit, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    } else
        mcpwm_set_signal_low(unit, MCPWM_TIMER_0, MCPWM_OPR_A);

    if (duty < 0.0f) {
        mcpwm_set_duty(unit, MCPWM_TIMER_0, MCPWM_OPR_B, -duty);
        mcpwm_set_duty_type(unit, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    } else
        mcpwm_set_signal_low(unit, MCPWM_TIMER_0, MCPWM_OPR_B);
}

void Motors::setBeepTone(uint8_t leftPitch, uint8_t rightPitch) {
    mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, beepPitchToFrequency(leftPitch));
    mcpwm_set_frequency(MCPWM_UNIT_1, MCPWM_TIMER_0, beepPitchToFrequency(rightPitch));
}

constexpr uint32_t Motors::beepPitchToFrequency(uint8_t pitch) {
    constexpr int32_t octaveOffset = 11;
    constexpr std::array<uint32_t, 12> freqTable = {
        16744, 17740, 18795, 19912, 21096, 22351,
        23680, 25088, 26580, 28160, 29834, 31609
    };

    int32_t octave = pitch / 12;
    int32_t tablePos = pitch % 12;
    uint32_t baseFreq = freqTable[tablePos];

    if (octave < octaveOffset)
        return baseFreq >> (octaveOffset - octave);
    else
        return baseFreq << (octave - octaveOffset);
}

}
