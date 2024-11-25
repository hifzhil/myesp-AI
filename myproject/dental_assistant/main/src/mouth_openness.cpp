#include "mouth_openness.hpp"

#include "esp_log.h"
#include "esp_camera.h"

#include "dl_image.hpp"
#include "image_util.h"
#include "human_face_detect_msr01.hpp"
#include "human_face_detect_mnp01.hpp"

#include "who_ai_utils.hpp"

#define TWO_STAGE_ON 1

static const char *TAG = "human_face_detection";

static QueueHandle_t xQueueFrameI = NULL;
static QueueHandle_t xQueueEvent = NULL;
static QueueHandle_t xQueueFrameO = NULL;
static QueueHandle_t xQueueResult = NULL;

std::pair<int, int> mouth_position;
float latency_mo = 0;

static bool gEvent = true;
static bool gReturnFB = true;
// static bool 

static void task_process_handler(void *arg)
{
    camera_fb_t *frame = NULL;
// camera_fb_t *resized_image = NULL;
    HumanFaceDetectMSR01 detector(0.3F, 0.3F, 10, 0.3F);
#if TWO_STAGE_ON
    HumanFaceDetectMNP01 detector2(0.4F, 0.3F, 10);
#endif


    while (true)
    {
        if (gEvent)
        {
            int64_t start_time = esp_timer_get_time();
            bool is_detected = false;
            if (xQueueReceive(xQueueFrameI, &frame, portMAX_DELAY))
            {
#if TWO_STAGE_ON
                std::list<dl::detect::result_t> &detect_candidates = detector.infer((uint16_t *)frame->buf, {(int)frame->height, (int)frame->width, 3});
                std::list<dl::detect::result_t> &detect_results = detector2.infer((uint16_t *)frame->buf, {(int)frame->height, (int)frame->width, 3}, detect_candidates);
#else
                std::list<dl::detect::result_t> &detect_results = detector.infer((uint16_t *)frame->buf, {(int)frame->height, (int)frame->width, 3});
#endif
                if (detect_results.size() > 0)
                {
                    draw_detection_result((uint16_t *)frame->buf, frame->height, frame->width, detect_results);
                    // print_detection_result(detect_results);
                    mouth_position = get_mouth_center(detect_results);
                    is_detected = true;


                }
            }

            if (xQueueFrameO)
            {
                xQueueSend(xQueueFrameO, &frame, portMAX_DELAY);
            }
            else if (gReturnFB)
            {
                esp_camera_fb_return(frame);
            }
            else
            {
                free(frame);
            }

            if (xQueueResult)
            {
                xQueueSend(xQueueResult, &is_detected, portMAX_DELAY);
            }
            int64_t end_time = esp_timer_get_time();    // End time
            int64_t latency_us = end_time - start_time; // Latency in microseconds
            latency_mo = latency_us / 1000.0;     // Latency in milliseconds

            // Print latency
            // ESP_LOGI(TAG, "Latency: %.2f ms", latency_ms);
        }
    }
}

static void task_event_handler(void *arg)
{
    while (true)
    {
        xQueueReceive(xQueueEvent, &(gEvent), portMAX_DELAY);
    }
}

void register_human_face_detection(const QueueHandle_t frame_i,
                                   const QueueHandle_t event,
                                   const QueueHandle_t result,
                                   const QueueHandle_t frame_o,
                                   const bool camera_fb_return)
{
    xQueueFrameI = frame_i;
    xQueueFrameO = frame_o;
    xQueueEvent = event;
    xQueueResult = result;
    gReturnFB = camera_fb_return;

    xTaskCreatePinnedToCore(task_process_handler, TAG, 4 * 1024, NULL, 5, NULL, 0);
    if (xQueueEvent)
        xTaskCreatePinnedToCore(task_event_handler, TAG, 4 * 1024, NULL, 5, NULL, 1);
}
