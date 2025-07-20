/*
 * Copyright (c) 2025, Mezael Docoy
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "pwm_hal.hpp"

Pwm_Error_t PwmHal::setup(uint8_t gpio_num, uint32_t freq_hz, uint32_t resolution_hz) {
    // Configure timer
    mcpwm_timer_config_t timer_config = {};
    timer_config.group_id = PWM_GROUP_ID;
    timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_config.resolution_hz = resolution_hz;
    timer_config.period_ticks = resolution_hz / freq_hz;
    timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    timer_config.intr_priority = 0;
    timer_config.flags = {};

    if (mcpwm_new_timer(&timer_config, &timer) != ESP_OK) 
        return Pwm_Error_t::PWM_ERROR;

    // Configure operator
    mcpwm_operator_config_t operator_config = {};
    operator_config.group_id = PWM_GROUP_ID;
    operator_config.intr_priority = 0;
    operator_config.flags = {};

    if (mcpwm_new_operator(&operator_config, &oper) != ESP_OK) 
        return Pwm_Error_t::PWM_ERROR;

    if (mcpwm_operator_connect_timer(oper, timer) != ESP_OK) 
        return Pwm_Error_t::PWM_ERROR;

    // Configure comparator
    mcpwm_comparator_config_t comparator_config = {};
    comparator_config.flags.update_cmp_on_tez = true;

    if (mcpwm_new_comparator(oper, &comparator_config, &comparator) != ESP_OK) 
        return Pwm_Error_t::PWM_ERROR;

    // Configure generator
    mcpwm_generator_config_t generator_config = {};
    generator_config.gen_gpio_num = gpio_num;
    generator_config.flags = {};

    if (mcpwm_new_generator(oper, &generator_config, &generator) != ESP_OK) 
        return Pwm_Error_t::PWM_ERROR;

    // Set initial compare value (center pulse)
    if (mcpwm_comparator_set_compare_value(comparator, resolution_hz / freq_hz / 2) != ESP_OK)
        return Pwm_Error_t::PWM_ERROR;

    // Set generator actions
    if (mcpwm_generator_set_action_on_timer_event(generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)) != ESP_OK)
        return Pwm_Error_t::PWM_ERROR;

    if (mcpwm_generator_set_action_on_compare_event(generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)) != ESP_OK)
        return Pwm_Error_t::PWM_ERROR;

    // Start timer
    if (mcpwm_timer_enable(timer) != ESP_OK) 
        return Pwm_Error_t::PWM_ERROR;
    if (mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP) != ESP_OK) 
        return Pwm_Error_t::PWM_ERROR;

    return Pwm_Error_t::PWM_OK;
}

Pwm_Error_t PwmHal::setCompareValue(uint32_t pulse_width_us) {
    if (mcpwm_comparator_set_compare_value(comparator, pulse_width_us) != ESP_OK) {
        return Pwm_Error_t::PWM_ERROR;
    }
    return Pwm_Error_t::PWM_OK;
}
