#include "camera_node.hpp"
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include "dl_image.hpp"

static const char *TAG = "who_camera";
static QueueHandle_t xQueueFrameO = NULL;
static QueueHandle_t xQueueResizedFrameO = NULL;
static QueueHandle_t xQueueNormalizeFrame = NULL;

void print_resized_image(const uint8_t *image_data, int width, int height, int channels) {
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (channels == 1) {
                // Grayscale image
                printf("%d ", image_data[y * width + x]);
            } else if (channels == 3) {
                // RGB image
                printf("(%d,%d,%d) ", 
                       image_data[(y * width + x) * 3],
                       image_data[(y * width + x) * 3 + 1],
                       image_data[(y * width + x) * 3 + 2]);
            }
        }
        printf("\n");
    }
}


static void task_process_handler(void *arg)
{
    while (true)
    {
        camera_fb_t *frame = esp_camera_fb_get();
        if (frame)
            xQueueSend(xQueueFrameO, &frame, portMAX_DELAY);
    }
}

static void task_resize_handler(void *arg)
{
    while (true)
    {
        camera_fb_t *frame = esp_camera_fb_get();
        if (frame)
        {
            int target_width = 28;
            int target_height = 28;
            int target_channel = 1;

            uint8_t *resized_data = (uint8_t *)malloc(target_width * target_height * target_channel);
            if (!resized_data) {
                ESP_LOGE(TAG, "Failed to allocate memory for resized data");
                esp_camera_fb_return(frame);
                continue;
            }

            dl::image::crop_and_resize<uint8_t>(
                resized_data,
                target_width,
                target_channel,
                0, target_height,
                0, target_width,
                (uint16_t *)frame->buf,
                frame->height,
                frame->width,
                3,  // Assuming RGB input
                0, frame->height,
                0, frame->width,
                dl::image::IMAGE_RESIZE_NEAREST
            );

            // Create a new camera_fb_t structure for the resized image
            camera_fb_t *resized_frame = (camera_fb_t *)malloc(sizeof(camera_fb_t));
            resized_frame->width = target_width;
            resized_frame->height = target_height;
            resized_frame->format = PIXFORMAT_GRAYSCALE;
            resized_frame->len = target_width * target_height * target_channel;
            resized_frame->buf = resized_data;
            // Send the resized frame to the new queue
            print_resized_image(resized_data, target_width, target_height, target_channel);
            usleep(200000);
            xQueueSend(xQueueResizedFrameO, &resized_frame, portMAX_DELAY);

            esp_camera_fb_return(frame);

            // Free the resized frame
            free(resized_frame->buf);
            free(resized_frame);
        }
    }
}


void register_camera(const pixformat_t pixel_fromat,
                     const framesize_t frame_size,
                     const uint8_t fb_count,
                     const QueueHandle_t frame_o,
                     const QueueHandle_t resized_frame_o)
{
    ESP_LOGI(TAG, "Camera module is %s", CAMERA_MODULE_NAME);

#if CONFIG_CAMERA_MODULE_ESP_EYE || CONFIG_CAMERA_MODULE_ESP32_CAM_BOARD
    /* IO13, IO14 is designed for JTAG by default,
     * to use it as generalized input,
     * firstly declair it as pullup input */
    gpio_config_t conf;
    conf.mode = GPIO_MODE_INPUT;
    conf.pull_up_en = GPIO_PULLUP_ENABLE;
    conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    conf.intr_type = GPIO_INTR_DISABLE;
    conf.pin_bit_mask = 1LL << 13;
    gpio_config(&conf);
    conf.pin_bit_mask = 1LL << 14;
    gpio_config(&conf);
#endif

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = CAMERA_PIN_D0;
    config.pin_d1 = CAMERA_PIN_D1;
    config.pin_d2 = CAMERA_PIN_D2;
    config.pin_d3 = CAMERA_PIN_D3;
    config.pin_d4 = CAMERA_PIN_D4;
    config.pin_d5 = CAMERA_PIN_D5;
    config.pin_d6 = CAMERA_PIN_D6;
    config.pin_d7 = CAMERA_PIN_D7;
    config.pin_xclk = CAMERA_PIN_XCLK;
    config.pin_pclk = CAMERA_PIN_PCLK;
    config.pin_vsync = CAMERA_PIN_VSYNC;
    config.pin_href = CAMERA_PIN_HREF;
    config.pin_sscb_sda = CAMERA_PIN_SIOD;
    config.pin_sscb_scl = CAMERA_PIN_SIOC;
    config.pin_pwdn = CAMERA_PIN_PWDN;
    config.pin_reset = CAMERA_PIN_RESET;
    config.xclk_freq_hz = XCLK_FREQ_HZ;
    config.pixel_format = pixel_fromat;
    config.frame_size = frame_size;
    config.jpeg_quality = 12;
    config.fb_count = fb_count;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

    // camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID || s->id.PID == OV2640_PID) {
        // s->set_vflip(s, 1); //flip it back
// I change this becouse my configuration is upside down
    s->set_vflip(s, 0);
    } else if (s->id.PID == GC0308_PID) {
        s->set_hmirror(s, 0);
    } else if (s->id.PID == GC032A_PID) {
        s->set_vflip(s, 1);
    }

    //initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID)
    {
        s->set_brightness(s, 1);  //up the blightness just a bit
        s->set_saturation(s, -2); //lower the saturation
    }

    xQueueFrameO = frame_o;
    xQueueResizedFrameO = resized_frame_o;

    // xTaskCreatePinnedToCore(task_process_handler, TAG, 3 * 1024, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(task_resize_handler, TAG, 4 * 1024, NULL, 5, NULL, 1);
}
