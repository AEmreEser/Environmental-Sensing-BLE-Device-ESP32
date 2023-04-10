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
