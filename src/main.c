#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h" // for vTaskDelay
#include "esp_log.h"
#include "driver/i2c.h"
#include <driver/gpio.h>

#include "defs.h"
#include "moving_med_filter.h"
#include "basic_stack.h"
#include "ble_control.h"

#include "./components/bmp180.h" // BMP180 LIBRARY TAKEN FROM: https://github.com/ESP32Tutorials/BMP180-ESP32-ESP-IDF/tree/main/components
#include "./components/bh1750.h" // BH1750 LIBRARY TAKEN FROM: https://github.com/pcbreflux/espressif/tree/master/esp32/app/ESP32_bh1750_oled/main

/** @brief reads from sensors, multiplexed by addres -> also converts pressure reading from pa to mbar, involves vtaskdelays
* @remark sensor reading frequency adjusted here: 1Hz < < 2.4 Hz
*/
static float i2c_sensor_read(const uint8_t addr, uint8_t * error){

    float ret = -1.0;
    uint32_t pressure_temp;

    switch (addr)
    {
    case BMP180_ADDR:
        *error = bmp180_read_pressure(&pressure_temp); // in pa
        ret = (float) (pressure_temp);
        ESP_LOGI(TAGBMP, "pressure at instance: %f \n", ret);
        vTaskDelay(1000 / portTICK_PERIOD_MS); 
        break;

    case BH1750_ADDR:
    case BH1750_ADDR_ALT:

        ERROR_CHECK( (ret = bh1750_read() ) != -1 ? ESP_OK : 1, *error, TAGBH); 
        *error = (ret == -1); // bh returns -1 for failed readings
        ESP_LOGI(TAGBH, "light strength at instance: %f \n", ret);
        vTaskDelay(720 /* 180 ms delay for sensor output (high res mode) inside bh1750_read() */ / portTICK_PERIOD_MS); // 100 ms delay for light measurement
        break;
    
    default:
        ESP_LOGI(TAGFILTER, "error while reading sensor values, invalid address %x", addr);

        break;
    }

    return ret;

}

/** @brief adds new value to the filter, gets new value of filtered sensor readings 
 * @return new filtered sensor reading, -1.0 for unitialized filter
 * @param raw_reading newly read raw value from sensor
 * @param filter the filter containing values for the sensor
*/
static float filter_sensor_reading(float raw_reading, moving_med_filter * filter){

    float ret = -1;
    esp_err_t error = ESP_OK;

    if (filter->value_array == NULL){
        ESP_LOGI(TAGFILTER, "filter value NULL inside filter_sensor_reading, cannot filter sensor reading");
    }
    else {

        error = add_to_filter(filter, raw_reading);
        ERROR_CHECK(error, error, TAGFILTER);

        error = get_filtered_value(filter, &ret);
        ERROR_CHECK(error, error, TAGFILTER);
    }

    return ret;

}

// LIFO QUEUES INSIDE ble_control.h


void app_main() { // sensor reading loop, bluedroid stack by default runs on a different thread

    // float temp_at_instance = 0; // in Centigrade
    float pressure_at_instance = 0; // in I think Pa
    float light_strength_at_instance = 0; // in lux
    uint8_t error = 0;

    stack_init_reset(&pressure_stack);
    stack_init_reset(&light_stack);

    moving_med_filter pressure_filter;
    moving_med_filter light_filter;

    mov_med_filter_init(&pressure_filter, PRESSURE_FILTER_MAX_SIZE, PRESSURE_FILTER_WINDOW_SIZE);
    mov_med_filter_init(&light_filter, LIGHT_FILTER_MAX_SIZE, LIGHT_FILTER_WINDOW_SIZE);

    // initialize bmp
    ERROR_CHECK(bmp180_init(PIN_SDA, PIN_SCL), error, TAGBMP);
    vTaskDelay(1000 / portTICK_PERIOD_MS); // 1000 ms delay

    // no need to initalize bh1750 --> initalization procedure concerns i2c port only and is not sensor specific
    // hence only initializign bmp would work for us

    init_ble_gap_routine(); // initialize ble thread

    vTaskDelay(1000 / portTICK_PERIOD_MS); // precaution making sure that the bluetooth adjustments are in line with the main thread

    ESP_LOGI(TAGGEN, "general check: moving median function, pressure filter is full %d, light filter is full %d", filter_is_full(&pressure_filter), filter_is_full(&light_filter));

    while (1){ // bt runs on a separate thread

        ESP_LOGI(TAGGEN, "WHILE ITERATION");

        uint8_t error;

        pressure_at_instance = i2c_sensor_read(BMP180_ADDR, &error);
        
        if (error == ESP_OK){
            ESP_LOGI(TAGGEN, "read pressure data without errors");
            pressure_at_instance /= 100; // mbar conversion (from pa)
            pressure_at_instance = filter_sensor_reading(pressure_at_instance, &pressure_filter);

            if (filter_is_full(&pressure_filter)){ // filter being full means: there are enough values to form an accurate sensor reading --> once full the filter will remain full unless reset
                ESP_LOGI(TAGGEN, "pressure filter is full");
                new_pressure_value_flag = true;
                error = stack_push(&pressure_stack, pressure_at_instance);
                if (error != ESP_OK)
                    ESP_LOGI(TAGGEN,"error while pushing onto stack - pressure, stack top: %d", pressure_stack.top);
                else 
                    ESP_LOGI(TAGGEN,"successful stack push - pressure, stack top: %d", pressure_stack.top);
            }
        }

        light_strength_at_instance = i2c_sensor_read(BH1750_ADDR, &error);

        if (error == ESP_OK){
            ESP_LOGI(TAGGEN, "read light data without errors");
            light_strength_at_instance = filter_sensor_reading(light_strength_at_instance, &light_filter);

            if (filter_is_full(&light_filter)){
                ESP_LOGI(TAGGEN, "light filter is full");
                new_light_value_flag = true;
                error = stack_push(&light_stack, light_strength_at_instance);
                if (error != ESP_OK)
                    ESP_LOGI(TAGGEN,"error while pushing onto stack - light, stack top: %d", light_stack.top);
                else 
                    ESP_LOGI(TAGGEN,"successful stack push - light, stack top: %d", light_stack.top);
            }
        }


        if (new_light_value_flag){
            new_light_value_flag = false;
            error = stack_pop(&light_stack, &(final_sensor_readings[1].value));
            ESP_LOGI(TAGGEN, "pop from stack %s, stack size %d", ((error == ESP_OK) ? ("successful") : ("failed")), light_stack.top );
            ESP_LOGI(TAGGEN, "updating adv data - new light value: %f", final_sensor_readings[1].value);
            update_adv_data(&(final_sensor_readings[1]));
            ESP_LOGI(TAGGEN,"new man_data contents %x, %x, %x, %x", man_data[0],man_data[1],man_data[2],man_data[3]);
            // adv_data.p_manufacturer_data = man_data;

            error = esp_ble_gap_config_adv_data(&adv_data); // triggers the conf set event
                ESP_LOGI(TAGGEN,"Reconfiguring adv data %d", error);

        }
        // vTaskDelay(1000 / portTICK_PERIOD_MS);

        if (new_pressure_value_flag){
            new_pressure_value_flag = false;
            error = stack_pop(&pressure_stack, &(final_sensor_readings[0].value));
            ESP_LOGI(TAGGEN, "pop from stack %s, %d", (error == ESP_OK) ? ("successful") : ("failed"), pressure_stack.top);
            ESP_LOGI(TAGGEN, "updating adv data - new pressure value: %f", final_sensor_readings[0].value);
            update_adv_data(&(final_sensor_readings[0]));
            ESP_LOGI(TAGGEN,"new man_data contents %x, %x, %x, %x", man_data[0],man_data[1],man_data[2],man_data[3]);
            
            // adv_data.p_manufacturer_data = man_data;

            error = esp_ble_gap_config_adv_data(&adv_data); // triggers the conf set event
                ESP_LOGI(TAGGEN,"Reconfiguring adv data %d", error);

        }
        // vTaskDelay(1000 / portTICK_PERIOD_MS);

    }
    
}

/**
 * TODO: fixes
 * -- DONE -- @lux measurement always returns 1 -> independent of actual measurement failure
 * 
 * TODO: features
 * @ --DONE-- light sensor stuff 
 * @ --DONE-- read from sensor function 
 * @ --DONE-- moving median function 
 * @ --DONE-- lifo queue for sensor values -- decide on queue item type carefully
 * @ --TESTING-- BLE functionality
 * 
 * Extra Features:
 * @ --DONE-- pa/mbar unit conversion
*/