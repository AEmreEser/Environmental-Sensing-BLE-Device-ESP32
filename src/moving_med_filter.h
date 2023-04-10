#ifndef _MOVING_MED_FILTER_H_
#define _MOVING_MED_FILTER_H_

#include "defs.h"

#include "./components/bmp180.h" // BMP180 LIBRARY TAKEN FROM: https://github.com/ESP32Tutorials/BMP180-ESP32-ESP-IDF/tree/main/components
#include "./components/bh1750.h" // BH1750 LIBRARY TAKEN FROM: https://github.com/pcbreflux/espressif/tree/master/esp32/app/ESP32_bh1750_oled/main

typedef struct {
    uint8_t max_size;
    uint8_t actual_size; 
    float * value_array;
    uint8_t window_size;
    bool window_size_odd_even; // actual size

} moving_med_filter;

/** @brief initialize/ reset a moving_med_filter object
 * @remark if object is already initialized any values contained inside value_array will be freed
*/
static void mov_med_filter_init(moving_med_filter * filter, uint8_t array_size, uint8_t window_size){

    ESP_LOGI(TAGGEN, "initializing moving median filter");

    filter->max_size = array_size;
    filter->value_array = (float *) (malloc(sizeof(float) * array_size));
    filter->actual_size = 0;
    filter->window_size = window_size;
    filter->window_size_odd_even = !( (window_size & 0x01) == 1); // even -> size 0
}

static void mov_med_filter_reset(moving_med_filter * filter){

    ESP_LOGI(TAGGEN, "mvoing median filter reset");

    filter->actual_size = 0;

}

/** @brief check if the filter array is full
 * @return 1 for full, 0 for not full
*/
static bool filter_is_full(moving_med_filter * filter) {
    return (filter->actual_size == filter->max_size);
}

/** @brief adds to the provided median value filter, triggers value discard if 
 *  @return ESP_OK for successful, ESP_ERR_INVALID_ARG for unitialized filter
*/
static esp_err_t add_to_filter(moving_med_filter * filter, float new_val){

    ESP_LOGI(TAGGEN, "adding value to moving median filter: %f", new_val);

    if (filter->value_array == NULL){
        ESP_LOGI(TAGFILTER, "value_array for provided moving_med_filter argument is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // rest of the function generated by chat gpt

    // If the value array is full, discard the oldest value and shift the array to the left

    if (filter_is_full(filter)){
        for (int i = filter->actual_size; i > 0; i--){
            filter->value_array[i] = filter->value_array[i-1]; // discard oldest value
        }
        filter->value_array[0] = new_val; // Add the new value to the value array
    }
    else {
        filter->value_array[filter->actual_size++] = new_val;   
    }
    return ESP_OK;
}

// function below generated by chat gpt
/** @brief  helper function to compare two float values
 *  @return 0 for equality, 1 for a > b, -1 for b > a
 */
int float_cmp(const void *a, const void *b) {
    float fa = *((float *)a);
    float fb = *((float *)b);
    return (fa > fb) - (fa < fb);
}

/** @brief sorts the array and calculates the median 
 * @param filtered_value result of moving median filtering
 * @return esp_ok if no error, ESP_ERR_INVALID_ARG if filter is uninitialized
*/
static esp_err_t get_filtered_value(moving_med_filter * filter, float * median_val){

    if (filter->value_array == NULL){
        ESP_LOGI(TAGFILTER, "value_array for provided moving_med_filter argument is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // bulk of function contents generated by chat gpt with occasional optimizations by Emre Eser
    // Make a copy of the data and sort it
    uint8_t filter_array_size = filter->actual_size;
    float * sorted_data = (float *) (malloc(sizeof(float) * filter_array_size) );

    ESP_LOGI(TAGGEN, "inside get_filtered_value: actual size %d", filter->actual_size);
    
    // memcpy((void *)(sorted_data), (const void *) (filter->value_array), filter_array_size); // optimized by Emre Eser

    for (int i = 0; i < filter->actual_size; i++){

        (sorted_data[i]) = filter->value_array[i];
        // ESP_LOGI(TAGGEN, "transferring data between arrays in get_filtered_value: index: %f, value in value array: %f, value in sorted array");

    }

    qsort(sorted_data, filter_array_size, sizeof(float), float_cmp);

    // Compute the median
    if ( (filter_array_size & 0x01) == 0) { // even: optimized by Emre Eser
        *median_val = (float) ( (sorted_data[filter_array_size / 2 - 1] + sorted_data[filter_array_size / 2]) / 2.0) ;
    } else {
        *median_val = (float) ( sorted_data[filter_array_size / 2] );
    }

    ESP_LOGI(TAGGEN, "getting filtered value from moving median func: %f", *median_val);

    if (sorted_data != NULL) free(sorted_data);
    return ESP_OK;

}


#endif // header guard