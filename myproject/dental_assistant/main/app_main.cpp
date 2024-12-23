 #include "mouth_openness.hpp"
#include "camera_node.hpp"
#include "servo_driver.h"
#include "mo_utils.hpp"
#include "app_wifi.h"
#include "app_httpd.hpp"
#include "app_mdns.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_defs.h"
#include "esp_vfs_fat.h"

// static QueueHandle_t xQueueFaceFrame = NULL;
static QueueHandle_t xQueueAIFrame = NULL;
static QueueHandle_t xQueueHttpFrame = NULL;
static QueueHandle_t xQueueResizedFrame = NULL;
static QueueHandle_t xQueueResult = NULL;

// visual servoing flag
#define CAMERA_WIDTH 240
#define CAMERA_HEIGHT 240
#define DESIRED_X 120
#define DESIRED_Y 120
#define Kp 0.1           // Proportional gain
const int threshold = 5; // Adjust this threshold as neede

#define GPIO_OUTPUT_PIN_LED GPIO_NUM_3
#define GPIO_OUTPUT_PIN_SEL (1ULL << GPIO_OUTPUT_PIN_LED)

bool detected = false;
int error_x = 0;
int error_y = 0;
float error_mouth = 0;
float latency = 0;
static const char *TAG = "MAIN";

static FILE *log_file = NULL;

void init_sd_card()
{
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024};
    sdmmc_card_t *card;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to mount SD card");
        return;
    }
    log_file = fopen("/sdcard/log.txt", "w");
    if (log_file == NULL)
    {
        ESP_LOGE(TAG, "Failed to open log file");
    }
}

void log_data(int mouth_x, int mouth_y, int error_x, int error_y, int64_t latency)
{
    if (log_file != NULL)
    {
        fprintf(log_file, "Mouth Position: (%d, %d), Error: (%d, %d), Latency: %lld us\n", mouth_x, mouth_y, error_x, error_y, latency);
        fflush(log_file);
    }
}

bool is_on_target(int current_x, int current_y, int target_x, int target_y, int threshold)
{
    // Calculate the Euclidean distance
    int dx = current_x - target_x;
    int dy = current_y - target_y;
    int distance_squared = dx * dx + dy * dy;
    error_mouth = sqrt(distance_squared);
    // Compare the squared distance with the squared threshold
    return distance_squared <= threshold * threshold;
}

void check_mouth_position()
{
    // Example mouth position
    int mouth_x = mouth_position.first;
    int mouth_y = mouth_position.second;

    // Check if the mouth is on target
    detected = is_on_target(mouth_x, mouth_y, DESIRED_X, DESIRED_Y, threshold);

    if (detected)
    {
        // ESP_LOGI("APP_MAIN", "Mouth is on target!");
    }
    else
    {
        // ESP_LOGI("APP_MAIN", "Mouth is not on target.");
    }
}

void init_gpio(void)
{
    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_INTR_DISABLE;

    io_conf.mode = GPIO_MODE_OUTPUT;

    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;

    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;

    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    esp_err_t ret = gpio_config(&io_conf);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "GPIO configured successfully");
    }
    else
    {
        ESP_LOGE(TAG, "GPIO configuration failed");
    }
}

void control_leds(bool detected)
{
    if (detected)
    {
        // Set GPIO3 to high (LED ON)
        gpio_set_level(GPIO_OUTPUT_PIN_LED, 0);
        // ESP_LOGI(TAG, "LED is ON");
    }
    else
    {
        // Set GPIO3 to low (LED OFF)
        gpio_set_level(GPIO_OUTPUT_PIN_LED, 1);
        // ESP_LOGI(TAG, "LED is OFF");
    }
}

void compute_servo_angle(int x, int y, float *servo_x, float *servo_y, float depth)
{
    error_x = x - DESIRED_X;
    error_y = y - DESIRED_Y;

    float L[2][2] = {
        {1.0f / depth, 0},
        {0, -1.0f / depth}};

    *servo_x = 0.05 * (L[0][0] * error_x + L[0][1] * error_y);
    *servo_y = 0.08 * (L[1][0] * error_x + L[1][1] * error_y);

    *servo_x = fmax(fmin(*servo_x, 90), -90);
    *servo_y = fmax(fmin(*servo_y, 90), -90);
}

extern "C" void app_main(void)
{
    float servo_angle_x_local, servo_angle_y_local;
    float depth = 0.5f;
    app_wifi_main();

    xQueueAIFrame = xQueueCreate(2, sizeof(camera_fb_t *));
    xQueueHttpFrame = xQueueCreate(2, sizeof(camera_fb_t *));
    xQueueResizedFrame = xQueueCreate(2, sizeof(camera_fb_t *));
    xQueueResult = xQueueCreate(2, sizeof(bool));

    register_camera(PIXFORMAT_RGB565, FRAMESIZE_240X240, 2, xQueueAIFrame, xQueueResizedFrame);
    app_mdns_main();
    // register_human_face_detection(xQueueAIFrame, NULL, NULL, xQueueHttpFrame, false);
    register_httpd(xQueueResizedFrame, NULL, true);

    // Initialize the servo driver
    register_servo_driver();
    ESP_LOGI("APP_MAIN", "Servo driver initialized successfully");

    // Initialize SD card
    // init_sd_card();

    // Log the initial state
    // ESP_LOGI(TAG, "Initial LED state set");

    // Log the initial state
    ESP_LOGI(TAG, "Initial LED state set");

    init_gpio();
    usleep(100000);
    int counter_log = 0;
    while (1)
    {
        int64_t start_time = esp_timer_get_time();
        compute_servo_angle(mouth_position.first, mouth_position.second, &servo_angle_x_local, &servo_angle_y_local, depth);

        // Update global variables
        servo_angle_x = servo_angle_x_local;
        servo_angle_y = servo_angle_y_local;
        // servo_angle_x = 0;
        // servo_angle_y = 0;

        check_mouth_position();

        control_leds(detected);

        int64_t end_time = esp_timer_get_time();
        float latency = (end_time - start_time) / 1000;

        // Log data
        // log_data(mouth_position.first, mouth_position.second, error_x, error_y, latency);

        if (counter_log % 10 == 0)
        { // Execute every 1 second (assuming vTaskDelay(pdMS_TO_TICKS(100)) is 100ms)
            // printf("Mouth Position: (%d, %d)\n", mouth_position.first, mouth_position.second);
            // printf("Servo Angle: (%d, %d)\n", servo_angle_x_local, servo_angle_y_local);
            // printf("Error: (%d, %d)\n", error_x, error_y);
            // printf("Error mouth: %f\n", error_mouth);
            // printf("Latency: %f\n", latency);
            // printf("Latency mo: %f\n", latency_mo);
            // printf("Latency servo: %f\n", latency_servo);
            // printf("time series: %lld\n", esp_timer_get_time());
            printf("%d,%d,%f,%f,%d,%d,%f,%f,%f,%f,%lld\n", mouth_position.first, mouth_position.second, servo_angle_x_local, servo_angle_y_local, error_x, error_y, error_mouth, latency, latency_mo, latency_servo, esp_timer_get_time());
        }

        vTaskDelay(pdMS_TO_TICKS(100));
        counter_log++;
    }
}
