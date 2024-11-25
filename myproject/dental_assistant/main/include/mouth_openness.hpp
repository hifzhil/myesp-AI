// #pragma once
#ifndef MOUTH_OPENNESS_HPP
#define MOUTH_OPENNESS_HPP

// Header file content
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "mo_utils.hpp"
#include "image_util.h"

// #include <utility>

void register_human_face_detection(QueueHandle_t frame_i,
                                   QueueHandle_t event,
                                   QueueHandle_t result,
                                   QueueHandle_t frame_o = NULL,
                                   const bool camera_fb_return = false);
extern std::pair<int, int> mouth_position;
extern float latency_mo;
#endif // MOUTH_OPENNESS_HPP
