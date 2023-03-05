#include "Motors.h"

#include "Pins.h"

#include <cstdio>

namespace RobotHal {

void Motors::setup() {
    for (int i = 0; i < 2; ++i) {
        motor[i].setup_pwm(Pins::motor[i].first, Pins::motor[i].second);
        motor[i].setup_encoder(Pins::encoder[i].first, Pins::encoder[i].second);
    }
}

void Motors::Motor::setup_pwm(IdfUtil::PinT pinA, IdfUtil::PinT pinB) {
    mcpwm_timer_config_t timerConfig = {
        .group_id = pwmGroupId,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = pwmResolutionHz,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = pwmPeriodTicks,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timerConfig, &timer));
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));

    mcpwm_operator_config_t operatorConfig = {
        .group_id = pwmGroupId,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operatorConfig, &oper));

    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    mcpwm_comparator_config_t comparatorConfig = {
        .flags = { .update_cmp_on_tez = true },
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparatorConfig, &comparatorA));
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparatorConfig, &comparatorB));

    mcpwm_comparator_set_compare_value(comparatorA, 0);
    mcpwm_comparator_set_compare_value(comparatorB, 0);

    mcpwm_generator_config_t generatorConfig = {
        .gen_gpio_num = pinA
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generatorConfig, &generatorA));
    generatorConfig.gen_gpio_num = pinB;
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generatorConfig, &generatorB));

    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(generatorA,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_TIMER_EVENT_ACTION_END()
    ));
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(generatorA,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparatorA, MCPWM_GEN_ACTION_LOW),
        MCPWM_GEN_COMPARE_EVENT_ACTION_END()
    ));
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(generatorB,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_TIMER_EVENT_ACTION_END()
    ));
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(generatorB,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparatorB, MCPWM_GEN_ACTION_LOW),
        MCPWM_GEN_COMPARE_EVENT_ACTION_END()
    ));

    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generatorA, 0, true));
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generatorB, 0, true));

    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    lastPwm = 0;
}

void Motors::Motor::setup_encoder(IdfUtil::PinT pinA, IdfUtil::PinT pinB) {
    pcnt_unit_config_t pcntUnitConfig = {
        .low_limit = -pcntLimit,
        .high_limit = pcntLimit,
        .flags = { .accum_count = true }, // enable counter accumulation
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&pcntUnitConfig, &encoder));

    // Watch points for counter accumulation
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(encoder, -pcntLimit));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(encoder, pcntLimit));

    pcnt_glitch_filter_config_t glitchFilterConfig = {
        .max_glitch_ns = 10000, // About 20x shorter than the count signal at max RPM
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(encoder, &glitchFilterConfig));


    pcnt_chan_config_t channelConfig = {
        .edge_gpio_num = pinB,
        .level_gpio_num = pinA,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(encoder, &channelConfig, &channelA));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(channelA, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(channelA, PCNT_CHANNEL_LEVEL_ACTION_INVERSE, PCNT_CHANNEL_LEVEL_ACTION_KEEP));

    // Configure second channel of the counter to the same pair of pins,
    // with edge and level pins swapped.
    // This gives us full 4 ticks per revolution of the encoder.
    std::swap(channelConfig.edge_gpio_num, channelConfig.level_gpio_num);
    ESP_ERROR_CHECK(pcnt_new_channel(encoder, &channelConfig, &channelB));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(channelB, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(channelB, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_enable(encoder));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(encoder));
    ESP_ERROR_CHECK(pcnt_unit_start(encoder));
}

void Motors::Motor::set(PwmT pwm) {
    if (pwm == lastPwm)
        return;
    else if (pwm == 0) {
        ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generatorA, 0, true));
        ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generatorB, 0, true));
    } else if (pwm > 0) {
        if (lastPwm <= 0) {
            ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generatorA, -1, true));
            ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generatorB, 0, true));
        }
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparatorA, pwm));
    } else {
        if (lastPwm >= 0) {
            ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generatorA, 0, true));
            ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generatorB, -1, true));
        }
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparatorB, -pwm));
    }
    lastPwm = pwm;
}

int32_t Motors::Motor::readEncoder() const {
    int count;
    ESP_ERROR_CHECK(pcnt_unit_get_count(encoder, &count));
    return count;
}

}
