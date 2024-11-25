/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "servo_driver.h"
#include "esp_timer.h"
static const char *TAG = "servo_control";

// Global variables for servo angles
float servo_angle_x = 0;
float servo_angle_y = 0;
float latency_servo = 0;

// Function to convert angle to compare value
uint32_t example_angle_to_compare(int angle)
{
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

// Servo task to control a servo based on angle commands
void servo_task(void *pvParameter)
{
    int gpio_num = *(int *)pvParameter;
    ESP_LOGI(TAG, "Create timer and operator for GPIO %d", gpio_num);

    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    ESP_LOGI(TAG, "Connect timer and operator for GPIO %d", gpio_num);
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    ESP_LOGI(TAG, "Create comparator and generator from the operator for GPIO %d", gpio_num);
    mcpwm_cmpr_handle_t comparator = NULL;
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = gpio_num,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    // Set the initial compare value, so that the servo will spin to the center position
    // ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0)));

    ESP_LOGI(TAG, "Set generator action on timer and compare event for GPIO %d", gpio_num);
    // Go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // Go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Enable and start timer for GPIO %d", gpio_num);
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0)));

    while (1) {
        int64_t start_time = esp_timer_get_time(); // Start time
        float angle = (gpio_num == SERVO1_PULSE_GPIO) ? servo_angle_x : servo_angle_y;
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(angle)));
        // Add delay, since it takes time for servo to rotate, usually 200ms/60degree rotation under 5V power supply
        int64_t end_time = esp_timer_get_time();    // End time
        int64_t latency_us = end_time - start_time; // Latency in microseconds
        latency_servo = latency_us / 1000.0;     // Latency in milliseconds
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// Function to initialize the servos
void register_servo_driver(void)
{
    // Define GPIO numbers for each servo
    static int servo1_gpio = SERVO1_PULSE_GPIO;
    static int servo2_gpio = SERVO2_PULSE_GPIO;

    // Create tasks for each servo
    xTaskCreate(servo_task, "servo1_task", 4096, &servo1_gpio, 5, NULL);
    xTaskCreate(servo_task, "servo2_task", 4096, &servo2_gpio, 5, NULL);
}
