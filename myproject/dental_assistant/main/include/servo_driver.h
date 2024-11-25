/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"

// Servo control parameters
#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE        -90   // Minimum angle
#define SERVO_MAX_DEGREE        90    // Maximum angle

#define SERVO1_PULSE_GPIO            1        // GPIO connects to the PWM signal line for servo 1
#define SERVO2_PULSE_GPIO            2        // GPIO connects to the PWM signal line for servo 2
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms



#ifdef __cplusplus
extern "C" {
#endif


extern float servo_angle_x;
extern float servo_angle_y;
extern float latency_servo;
// Function to convert angle to compare value
uint32_t example_angle_to_compare(int angle);

// Servo task to control a servo based on angle commands
void servo_task(void *pvParameter);

// Function to initialize the servos
// void register_servo_driver  (QueueHandle_t frame_i,
//                             QueueHandle_t event,
//                             QueueHandle_t result,
//                             QueueHandle_t frame_o = NULL,
//                             const bool camera_fb_return = false);

void register_servo_driver();

#ifdef __cplusplus
}
#endif

#endif // SERVO_CONTROL_H
