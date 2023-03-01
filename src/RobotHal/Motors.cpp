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

    for (int i = 0; i < 2; ++i) {
        motor[i].setup(Pins::motor[i].first, Pins::motor[i].second);
    }
}

void Motors::Motor::setup(IdfUtil::PinT pinA, IdfUtil::PinT pinB) {
    // Adapted from the bdc_motor library
    // https://github.com/espressif/idf-extra-components/blob/master/bdc_motor/src/bdc_motor_mcpwm_impl.c
    mcpwm_timer_config_t timerConfig = {
        .group_id = pwmGroupId,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = pwmResolutionHz,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = pwmPeriodTicks,
    };
    HAL_CHECK(mcpwm_new_timer(&timerConfig, &timer));
    HAL_CHECK(mcpwm_timer_enable(timer));

    mcpwm_operator_config_t operatorConfig = {
        .group_id = pwmGroupId,
    };
    HAL_CHECK(mcpwm_new_operator(&operatorConfig, &oper));

    HAL_CHECK(mcpwm_operator_connect_timer(oper, timer));

    mcpwm_comparator_config_t comparatorConfig = {
        .flags = { .update_cmp_on_tez = true },
    };
    HAL_CHECK(mcpwm_new_comparator(oper, &comparatorConfig, &comparatorA));
    HAL_CHECK(mcpwm_new_comparator(oper, &comparatorConfig, &comparatorB));

    mcpwm_comparator_set_compare_value(comparatorA, 0);
    mcpwm_comparator_set_compare_value(comparatorB, 0);

    mcpwm_generator_config_t generatorConfig = {
        .gen_gpio_num = pinA
    };
    HAL_CHECK(mcpwm_new_generator(oper, &generatorConfig, &generatorA));
    generatorConfig.gen_gpio_num = pinB;
    HAL_CHECK(mcpwm_new_generator(oper, &generatorConfig, &generatorB));

    HAL_CHECK(mcpwm_generator_set_actions_on_timer_event(generatorA,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_TIMER_EVENT_ACTION_END()
    ));
    HAL_CHECK(mcpwm_generator_set_actions_on_compare_event(generatorA,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparatorA, MCPWM_GEN_ACTION_LOW),
        MCPWM_GEN_COMPARE_EVENT_ACTION_END()
    ));
    HAL_CHECK(mcpwm_generator_set_actions_on_timer_event(generatorB,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_TIMER_EVENT_ACTION_END()
    ));
    HAL_CHECK(mcpwm_generator_set_actions_on_compare_event(generatorB,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparatorB, MCPWM_GEN_ACTION_LOW),
        MCPWM_GEN_COMPARE_EVENT_ACTION_END()
    ));

    HAL_CHECK(mcpwm_generator_set_force_level(generatorA, 0, true));
    HAL_CHECK(mcpwm_generator_set_force_level(generatorB, 0, true));

    HAL_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    lastPwm = 0;
}

void Motors::Motor::set(PwmT pwm) {
    if (pwm == lastPwm) {
        printf("Pwm did not change\n");
        return;
    }
    else if (pwm == 0) {
        printf("Stopping motor\n");
        HAL_CHECK(mcpwm_generator_set_force_level(generatorA, 0, true));
        HAL_CHECK(mcpwm_generator_set_force_level(generatorB, 0, true));
    } else if (pwm > 0) {
        if (lastPwm <= 0) {
            printf("Spin forward\n");
            HAL_CHECK(mcpwm_generator_set_force_level(generatorA, -1, true));
            HAL_CHECK(mcpwm_generator_set_force_level(generatorB, 0, true));
        }
        HAL_CHECK(mcpwm_comparator_set_compare_value(comparatorA, pwm));
    } else {
        if (lastPwm >= 0) {
            printf("Spin backward\n");
            HAL_CHECK(mcpwm_generator_set_force_level(generatorA, 0, true));
            HAL_CHECK(mcpwm_generator_set_force_level(generatorB, -1, true));
        }
        HAL_CHECK(mcpwm_comparator_set_compare_value(comparatorB, -pwm));
    }
    lastPwm = pwm;
}

std::pair<int16_t, int16_t> Motors::readEncoders() const
{
    int16_t v1;
    int16_t v2;
    pcnt_get_counter_value(PCNT_UNIT_0, &v1);
    pcnt_get_counter_value(PCNT_UNIT_1, &v2);
    return std::make_pair(v1, v2);
}

}
