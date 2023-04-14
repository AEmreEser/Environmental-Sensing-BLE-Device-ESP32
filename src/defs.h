#ifndef _DEFS_H_
#define _DEFS_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <esp_system.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_bt.h>
#include <esp_bt_defs.h>
#include <esp_bt_main.h>
#include <esp_gap_ble_api.h>

#include "sdkconfig.h"

// for main.c
#define INCLUDE_vTaskDelay 1 // to enable the vtaskdelay function : https://www.freertos.org/a00127.html
#define PIN_SCL 22 // GPIO_NUM_22
#define PIN_SDA 21 // GPIO_NUM_21 // IMPORTANT --> ALSO DEFINED IN THE BH1750.H FILE --> MODIFY IT AS WELL
#define ERROR_CHECK(error_param, error, TAG) error = ( (error_param) != ESP_OK) ? (1) : (0); ESP_LOGI(TAG, "error status: %d \n",error);

/** @brief holds sensor readings as float values, values could also be accessed as individual bytes through byte array field  
*/
typedef union  {
    float value;
    char byte_array[4]; // to access individual bytes of a float
} sensor_reading ;


// Buffer to keep the last two sensor readings -> index 0: pressure, index 1: light strength
static sensor_reading final_sensor_readings[2]; // = {23,34}; // contains the data that will be sent in the advertisement packet

// flags to signal new sensor reading 
static bool new_pressure_value_flag = 0;
static bool new_light_value_flag = 0; // controls when to send the adv data

// Logging tags
static const char * TAGBMP = "BMP180"; // defined in bmp180.c
static const char * TAGBH = "BH1750"; // defined in bh1750.c
static const char * TAGGEN = "GENERAL"; // general purpose tag 
static const char * TAGFILTER = "mov med filter tag: "; // tag for moving med filtering functions

// addresses of i2c sensors
static const uint8_t BMP180_ADDR = 0x77;
static const uint8_t BH1750_ADDR = 0x23; // tied to low
static const uint8_t BH1750_ADDR_ALT = 0x5C; // tied to high

//  moving_med_filter.h definitions
#define PRESSURE_FILTER_MAX_SIZE 5
#define PRESSURE_FILTER_WINDOW_SIZE 2
#define LIGHT_FILTER_MAX_SIZE 5
#define LIGHT_FILTER_WINDOW_SIZE 2

// basic_stack.h
#define STACK_SIZE 50
#define DISCARD_THRESHOLD 45

// ble_control.h

// Advertising data packet values
static uint8_t adv_svc_uuid [] = {0x58,0x70,0x01,0xFB,0x1B,0x2F,0x16,0x1E,0x40,0x0A,0x91,0x5B,0xF2,0x96,0x66,0x5E};
static const uint16_t MAN_LEN = 4; // same size as a float variable
static uint8_t man_data[4] = {0x00, 0x01, 0x02, 0x03}; // buffer to store data to be sent through BLE
static const char * BLE_TAG = "BLE Log"; // unused -> replaced by TAGGEN, left here for future use

// advertising data configuration --> throughout the program's lifecyle only the man_data value will change
static esp_ble_adv_data_t adv_data =  { 
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    /* interval calculated as: value of interval parameter * 1.25 ms */
    .min_interval = 0x5DBF, // 28.75 secs - advertisement interval
    .max_interval = 0x5DC1, // 31.25 secs - advertisement interval
    // .appearance = 0x0300, // generic thermometer
    .manufacturer_len = MAN_LEN,
    .appearance = 384, // generic remote control
    .p_manufacturer_data = man_data,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_svc_uuid),
    .p_service_uuid = adv_svc_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT ), // last flag tells connections that we are using ble 
};

// gap advertising configuration
static esp_ble_adv_params_t adv_params /* configuring the advertising process */ = { // basically the loosest adv configs anywhere : )
    .adv_int_min        = 0x5DBF,
    .adv_int_max        = 0x5DC1,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// declaration of the event handler function --> definition in ble_control.h
static void gap_event_handler(esp_gap_ble_cb_event_t event , esp_ble_gap_cb_param_t *param );

// error check macro with error value specification
#define ERROR_CHECK_err_val(error_param, error, error_value, TAG) error = ( (error_param) != error_value) ? (1) : (0); ESP_LOGI(TAG, "error status: %d \n",error);

#endif // header guard

