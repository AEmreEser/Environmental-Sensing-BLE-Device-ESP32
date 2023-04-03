#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h" // for vTaskDelay
#include "esp_log.h"
#include "driver/i2c.h"
#include <driver/gpio.h>

#include "./components/bmp180.h"
// BMP180 LIBRARY TAKEN FROM: https://github.com/ESP32Tutorials/BMP180-ESP32-ESP-IDF/tree/main/components

#define PIN_SCL 22 // GPIO_NUM_22
#define PIN_SDA 21 // GPIO_NUM_21

static const char * TAGBMP = "BMP180"; // defined in bmp180.c

void app_main() {

    float temp_at_instance = 0;
    uint32_t pressure_at_instance = 0;

    esp_err_t error = bmp180_init(21, 22);

    vTaskDelay(500);

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    while (1){
        error = bmp180_read_temperature(&temp_at_instance);
        ESP_LOGI(TAGBMP, "temperature at instance: %.9f", temp_at_instance);
        error = bmp180_read_pressure(&pressure_at_instance);
        ESP_LOGI(TAGBMP, "pressure at instance: %ld", pressure_at_instance);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAGBMP, "%d",error);
    
}