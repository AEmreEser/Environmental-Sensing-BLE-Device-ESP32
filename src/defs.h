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

// typedef enum {PRESSURE = 0, LIGHT = 1} sensor_t;

static const char * TAGBMP = "BMP180"; // defined in bmp180.c
static const char * TAGBH = "BH1750"; // defined in bh1750.c
static const char * TAGGEN = "GENERAL"; // general purpose tag 
static const char * TAGFILTER = "mov med filter tag: "; // tag for moving med filtering functions

    // addresses of i2c sensors
static const uint8_t BMP180_ADDR = 0x77;
static const uint8_t BH1750_ADDR = 0x23; // tied to low
static const uint8_t BH1750_ADDR_ALT = 0x5C; // tied to high

// ***** moving med functions are inside moving_med_filter.h *****

// basic_stack.h
#define STACK_SIZE 50
#define DISCARD_THRESHOLD 45

#define TESTING

// ble_control.h
/** @brief holds sensor readings as float values, values could also be accessed as individual bytes through byte array field  
*/
typedef union  {
    float value;
    char byte_array[4];
} sensor_reading ;

static uint8_t adv_svc_uuid [] = {0x58,0x70,0x01,0xFB,0x1B,0x2F,0x16,0x1E,0x40,0x0A,0x91,0x5B,0xF2,0x96,0x66,0x5E};

static const uint16_t MAN_LEN = 4; // same size as a float variable
static uint8_t man_data[4] = {0x00, 0x01, 0x02, 0x03}; //  ( uint8_t const *) (malloc(sizeof(uint8_t) * 4)); // 

static const char * BLE_TAG = "BLE Log"; 

// sample config for easy access -> could use shallow copying as there'll only be one of these throught the lifetime of the program
static esp_ble_adv_data_t adv_data =  { 
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0004,
    .max_interval = 0x00F0,
    // .appearance = 0x0300, // generic thermometer
    .manufacturer_len = MAN_LEN, // won't change from packet to packet
    .appearance = 384, // generic remote control
    .p_manufacturer_data = man_data,
    .service_data_len = 0, //MAN_LEN, // codes for advertised services
    .p_service_data = NULL,// &man_data,
    .service_uuid_len = sizeof(adv_svc_uuid),
    .p_service_uuid = adv_svc_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT ), // last flag tells connections that we are usign ble 
};

// static esp_ble_adv_data_t * adv_data;

static esp_ble_adv_params_t adv_params /* configuring the advertising process */ = { // basically the loosest adv configs anywhere : )
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};


static void gap_event_handler(esp_gap_ble_cb_event_t event , esp_ble_gap_cb_param_t *param );

// error check with error value specification
#define ERROR_CHECK_err_val(error_param, error, error_value, TAG) error = ( (error_param) != error_value) ? (1) : (0); ESP_LOGI(TAG, "error status: %d \n",error);

// TESTING FUNCTIONS:
#ifdef TESTING

    /** @brief sets man data to the contents of new_data --> assumes size 4
    */
    static void set_man_data(uint8_t new_data[] ){
        memcpy((void *) (man_data), (const void *) (new_data), 4);
    }

    /** @brief intializes man_data with dummy values --> made for an array of size 4
    */
    static void initialize_man_data_dummy_values(){

        uint8_t dummy_data[] = {0x04, 0x01, 0x02, 0x03};
        set_man_data(dummy_data);
    }

#endif // TESTING ifdef



#endif // header guard

