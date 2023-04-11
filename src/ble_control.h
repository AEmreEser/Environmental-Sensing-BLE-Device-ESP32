#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <esp_bt.h>

#include <esp_bt_defs.h>
#include <esp_bt_main.h>
#include <esp_gap_ble_api.h>

#include "sdkconfig.h"
#include "defs.h"

#include "basic_stack.h" // float stack for sensor values

// STATIC LIFO QUEUES for sensor data
static stack_t pressure_stack; // stack size defined in defs.h
static stack_t light_stack;



/** @brief updates the manufacturer data part of the adv packet , updates by overwriting the data pointed by the advertisin packet
 *  @param sensor_value the new value of the manufacturer data --> need to pass the array pointer inside union type
*/
static void inline update_adv_data(sensor_reading * sensor_value){ 

    man_data[0] = sensor_value->byte_array[3]; // reversed values since we have a difference in the way data in kept between these two datatypes
    man_data[1] = sensor_value->byte_array[2];
    man_data[2] = sensor_value->byte_array[1];
    man_data[3] = sensor_value->byte_array[0];

    ESP_LOGI(BLE_TAG, "updated value of adv data: %x, %x, %x, %x ", man_data[0], man_data[1], man_data[2],man_data[3]);
}


/** @brief intializes bluetooth low energy gap procedure acc. to some predefined specs
*/
void init_ble_gap_routine(){

    esp_err_t error;

    error = nvs_flash_init();
        ERROR_CHECK(error, error, TAGGEN);

    // error = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
       // ERROR_CHECK(error, error, TAGGEN);

    esp_bt_controller_config_t def_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    error = esp_bt_controller_init(&def_cfg);
    ERROR_CHECK(error, error, TAGGEN);

    error = esp_bt_controller_enable(ESP_BT_MODE_BLE);
        ERROR_CHECK(error, error, TAGGEN);

    error = esp_bluedroid_init();
        ERROR_CHECK(error, error, TAGGEN);

    error = esp_bluedroid_enable();
        ERROR_CHECK(error, error, TAGGEN);

    error = esp_ble_gap_register_callback(gap_event_handler);
        ERROR_CHECK(error, error, TAGGEN);

    error = esp_ble_gap_set_device_name("ESP32 UNIT"); // creatively named :)
        ERROR_CHECK(error, error, TAGGEN);

    error = esp_ble_gap_config_adv_data(&adv_data);
        ERROR_CHECK(error, error, TAGGEN);    
    
}

/** @brief callback function for GAP related events
*/
static void gap_event_handler(esp_gap_ble_cb_event_t event , esp_ble_gap_cb_param_t *param ){

    esp_err_t error = 0;

    switch (event){
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT: // advertising data set complete
     
        ESP_LOGI(TAGGEN, "data config set complete, starting to advertise");
        esp_ble_gap_start_advertising(&adv_params);

        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT: // upon starting advertising

        ESP_LOGI(TAGGEN, "began advertising, man_data contents %x, %x, %x, %x", man_data[0],man_data[1],man_data[2],man_data[3]);
    
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT: // could be useful when switching advertised data

        ESP_LOGI(TAGGEN,"received adv stop event");

        if (new_light_value_flag ){
            new_light_value_flag = false;
            error = stack_pop(&light_stack, &(final_sensor_readings[1].value));
            ESP_LOGI(TAGGEN, "pop from stack %s, stack size %d", ((error == ESP_OK) ? ("successful") : ("failed")), light_stack.top );
            ESP_LOGI(TAGGEN, "updating adv data - new light value: %f", final_sensor_readings[1].value);
            update_adv_data(&(final_sensor_readings[1]));

            adv_data.p_manufacturer_data = man_data;

            error = esp_ble_gap_config_adv_data(&adv_data);
                ESP_LOGI(TAGGEN,"Reconfiguring adv data %d", error);

            vTaskDelay(1000 / portTICK_PERIOD_MS);

        } 
        
        if (new_pressure_value_flag){
            new_pressure_value_flag = false;
            error = stack_pop(&pressure_stack, &(final_sensor_readings[0].value));
            ESP_LOGI(TAGGEN, "pop from stack %s, %d", (error == ESP_OK) ? ("successful") : ("failed"), pressure_stack.top);
            ESP_LOGI(TAGGEN, "updating adv data - new pressure value: %f", final_sensor_readings[0].value);
            update_adv_data(&(final_sensor_readings[0]));

            adv_data.p_manufacturer_data = man_data;

            error = esp_ble_gap_config_adv_data(&adv_data);
                ESP_LOGI(TAGGEN,"Reconfiguring adv data %d", error);

            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

            // esp_ble_gap_start_advertising(&adv_params);

        break;

    default: 
        ESP_LOGI(TAGGEN, "unrecognized gap event");
        break; 
    }
}

