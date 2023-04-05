#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h" // for vTaskDelay
#include "esp_log.h"
#include "driver/i2c.h"
#include <driver/gpio.h>

#include "./components/bmp180.h"
// BMP180 LIBRARY TAKEN FROM: https://github.com/ESP32Tutorials/BMP180-ESP32-ESP-IDF/tree/main/components
#include "./components/bh1750.h"
// BH1750 LIBRARY TAKEN FROM: https://github.com/pcbreflux/espressif/tree/master/esp32/app/ESP32_bh1750_oled/main

#define INCLUDE_vTaskDelay 1 // to enable the vtaskdelay function : https://www.freertos.org/a00127.html
#define PIN_SCL 22 // GPIO_NUM_22
#define PIN_SDA 21 // GPIO_NUM_21 // IMPORTANT --> ALSO DEFINED IN THE BH1750.H FILE --> MODIFY IT AS WELL

#define ERROR_CHECK(error_param, error, TAG) error = ( (error_param) != ESP_OK) ? (1) : (0); ESP_LOGI(TAG, "error status: %d",error);

static const char * TAGBMP = "BMP180"; // defined in bmp180.c
static const char * TAGBH = "BH1750"; // defined in bh1750.c

void app_main() {

    float temp_at_instance = 0; // in Centigrade
    uint32_t pressure_at_instance = 0; // in I think Pa
    float light_strength = 0; // in lux
    uint8_t error = 0;

    // initialize bmp
    ERROR_CHECK(bmp180_init(21, 22), error, TAGBMP);
    vTaskDelay(1000 / portTICK_PERIOD_MS); // 100 ms delay

    // initialize bh
    // cannot error check this here, error checking done internally
    // bh1750_init(); // no need for this - the port is already configured by the bmp 180 library above -- will throw an error if both configuration functions are called on the same port without removing configurations
    vTaskDelay(1000 / portTICK_PERIOD_MS); // 100 ms delay

    while (1){
        ERROR_CHECK(bmp180_read_temperature(&temp_at_instance), error, TAGBMP);
        ESP_LOGI(TAGBMP, "temperature at instance: %f", temp_at_instance);

        ERROR_CHECK(bmp180_read_pressure(&pressure_at_instance), error, TAGBMP);
        ESP_LOGI(TAGBMP, "pressure at instance: %ld", pressure_at_instance);

        vTaskDelay(2400 / portTICK_PERIOD_MS); // 240 ms delay // need twice of high res measurement update period as the measurements are taken twice

        ERROR_CHECK(light_strength = bh1750_read(), error, TAGBH); // ISSUE: FOR SOME REASON WE ALWAYS GET AN ERROR = 1 WITH THE LUX MEASUREMENT
        ESP_LOGI(TAGBH, "light strength at instance: %f", light_strength);

        vTaskDelay(1000 / portTICK_PERIOD_MS); // 100 ms delay

    }
    
}

/**
 * TODO: fixes
 * @lux measurement always returns 1 -> independent of actual measurement failure
 * 
 * TODO: features
 * @ --DONE-- light sensor stuff 
 * @ read from sensor function 
 * @ moving median function
 * @ lifo queue for sensor values -- decide on queue item type carefully
 * @ BLE functionality
 * 
 * Extra Features:
 * @ -- unit conversion
*/