/* Copyright (c) 2017 pcbreflux. All Rights Reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>. *
 */
#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"
#include "esp_log.h"
#include "driver/i2c.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bh1750.h"

static const char * BHTAG = "BH1750";

#define I2C_ADDR BH1750_ADDRESS1
#define BH1750_MODE BH1750_CONTINUOUS_HIGH_RES_MODE

#define PIN_SDA 21
#define PIN_SCL 22
#define I2C_MASTER_NUM I2C_NUM_0   /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_MASTER_FREQ_HZ   10000     /*!< I2C master clock frequency */
#define ACK_CHECK_EN   0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL    0x0         /*!< I2C ack value */
#define NACK_VAL   0x1         /*!< I2C nack value */


void bh1750_reset(void) {
   	ESP_LOGE(BHTAG, "reset");
	bh1750_I2C_write(I2C_ADDR, BH1750_POWER_ON, NULL, 0);
	bh1750_I2C_write(I2C_ADDR, BH1750_RESET, NULL, 0);
	vTaskDelay(10 / portTICK_PERIOD_MS); // sleep 10ms
}

/** 
 * 
 * @brief reads from the sensor and returns the value 
 * @return sensor value float in lux, returns -1 for error during read
 * @remarks @paragraph 
 * 
 * reads 2 bytes into a 2 byte uint8 array using the i2c read function 
 * consoldiates them into a 16 bit integer
 * calculates value based on measurement accuracy setting etc
 * returns the calculated value, not any error code
 * 
 * @endparblock
*/
float bh1750_read(void) {
	uint8_t buf[2 /* 32 */]; // 32 byte array --> reduced into 2 bytes
	uint8_t mode = BH1750_MODE;
	uint8_t sleepms = 1;
	uint8_t resdiv = 1;
	float luxval=0;
	int ret = -1;

	switch (mode) {
	case BH1750_CONTINUOUS_HIGH_RES_MODE:
	case BH1750_ONE_TIME_HIGH_RES_MODE:
		sleepms=180;
		break;
	case BH1750_CONTINUOUS_HIGH_RES_MODE_2:
	case BH1750_ONE_TIME_HIGH_RES_MODE_2:
		sleepms=180;
		resdiv=2;
		break;
	case BH1750_CONTINUOUS_LOW_RES_MODE:
		sleepms=24;
		break;
	case BH1750_ONE_TIME_LOW_RES_MODE:
		sleepms=50;
		break;
	}

	ret=bh1750_I2C_write(I2C_ADDR, mode, NULL, 0); // probably a testing sequence
    if (ret != ESP_OK) {
    	bh1750_reset();
    	return -1;
    }

	vTaskDelay(sleepms / portTICK_PERIOD_MS); // sleep ms
	ret=bh1750_I2C_read(I2C_ADDR, 0xFF, buf, 2);
    if (ret != ESP_OK) {
    	bh1750_reset();
    	return -1;
    }
	uint16_t luxraw = (uint16_t) ( ((uint16_t)(buf[0]<<8)) | ((uint16_t)buf[1]) ); // pack into one 16 bit measurement
	luxval = ( (float) luxraw )/1.2/resdiv;
	ESP_LOGI(BHTAG, "sensraw=%u lux=%f", luxraw, luxval);

	return luxval;
}
/** 
 * @brief configure the i2c port
*/
void bh1750_init(void) {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
	ESP_LOGI(BHTAG, "sda_io_num %d", PIN_SDA);
    conf.sda_io_num = PIN_SDA;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	ESP_LOGI(BHTAG, "scl_io_num %d", PIN_SCL);
    conf.scl_io_num = PIN_SCL;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	ESP_LOGI(BHTAG, "clk_speed %d", I2C_MASTER_FREQ_HZ);
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
	ESP_LOGI(BHTAG, "i2c_param_config %d", conf.mode);
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
	ESP_LOGI(BHTAG, "i2c_driver_install %d", I2C_MASTER_NUM);
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));

}

/*  ERROR LOG FROM FIRST RUN
    ERROR CAUSE: ATTEMPTING TO RE-INSTALL A DIFFERENT CONFIG TO A CONFIGURED PORT

␛[0;32mI (212) cpu_start: Application information:␛[0m
␛[0;32mI (217) cpu_start: Project name:     Borda_3␛[0m
␛[0;32mI (222) cpu_start: App version:      dc28255-dirty␛[0m
␛[0;32mI (227) cpu_start: Compile time:     Apr  5 2023 19:46:18␛[0m
␛[0;32mI (233) cpu_start: ELF file SHA256:  37eae859a15c4082...␛[0m
␛[0;32mI (239) cpu_start: ESP-IDF:          5.0.1␛[0m
␛[0;32mI (244) cpu_start: Min chip rev:     v0.0␛[0m
␛[0;32mI (249) cpu_start: Max chip rev:     v3.99 ␛[0m
␛[0;32mI (254) cpu_start: Chip rev:         v3.0␛[0m
␛[0;32mI (258) heap_init: Initializing. RAM available for dynamic allocation:␛[0m
␛[0;32mI (266) heap_init: At 3FFAE6E0 len 00001920 (6 KiB): DRAM␛[0m
␛[0;32mI (272) heap_init: At 3FFB2830 len 0002D7D0 (181 KiB): DRAM␛[0m
␛[0;32mI (278) heap_init: At 3FFE0440 len 00003AE0 (14 KiB): D/IRAM␛[0m
␛[0;32mI (284) heap_init: At 3FFE4350 len 0001BCB0 (111 KiB): D/IRAM␛[0m
␛[0;32mI (291) heap_init: At 4008CF80 len 00013080 (76 KiB): IRAM␛[0m
␛[0;32mI (298) spi_flash: detected chip: gd␛[0m
␛[0;32mI (301) spi_flash: flash io: dio␛[0m
␛[0;32mI (306) cpu_start: Starting scheduler on PRO CPU.␛[0m
␛[0;32mI (0) cpu_start: Starting scheduler on APP CPU.␛[0m
␛[0;32mI (317) BMP180 I2C Driver: I2C master driver has been installed.␛[0m
␛[0;32mI (327) BMP180 I2C Driver: BMP180 sensor found at 0x77␛[0m
␛[0;32mI (337) BMP180 I2C Driver: AC1: 8428, AC2: -1171, AC3: -14681, AC4: 33922, AC5: 25315, AC6: 15135␛[0m
␛[0;32mI (337) BMP180 I2C Driver: B1: 6515, B2: 46, MB: -32768, MC: -11786, MD: 2628␛[0m
␛[0;32mI (347) BMP180: 0␛[0m
␛[0;32mI (1347) BH1750: sda_io_num 21␛[0m
␛[0;32mI (1347) BH1750: scl_io_num 22␛[0m
␛[0;32mI (1347) BH1750: clk_speed 10000␛[0m
␛[0;32mI (1347) BH1750: i2c_param_config 1␛[0m
␛[0;32mI (1347) BH1750: i2c_driver_install 0␛[0m
␛[0;31mE (1347) i2c: i2c driver install error␛[0m
ESP_ERROR_CHECK failed: esp_err_t 0xffffffff (ESP_FAIL) at 0x400d0f4d
file: "src/components/bh1750.c" line 125
func: bh1750_init
expression: i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0)

abort() was called at PC 0x40086a47 on core 0


Backtrace: 0x40082526:0x3ffb49d0 0x40086a51:0x3ffb49f0 0x4008bb46:0x3ffb4a10 0x40086a47:0x3ffb4a80 0x400d0f4d:0x3ffb4ab0 0x400d1745:0x3ffb4af0 0x400e8782:0x3ffb4b30 0x400895bd:0x3ffb4b60




ELF file SHA256: 37eae859a15c4082

Rebooting...
ets Jul 29 2019 12:21:46

rst:0xc (SW_CPU_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
configsip: 0, SPIWP:0xee
clk_drv:0x00,q_drv:0x00,d_drv:0x00,cs0_drv:0x00,hd_drv:0x00,wp_drv:0x00
mode:DIO, clock div:2
load:0x3fff0030,len:6940
ho 0 tail 12 room 4
load:0x40078000,len:15500
load:0x40080400,len:3844
entry 0x4008064c
␛[0;32mI (29) boot: ESP-IDF 5.0.1 2nd stage bootloader␛[0m
␛[0;32mI (29) boot: compile time 19:48:52␛[0m
␛[0;32mI (29) boot: chip revision: v3.0␛[0m
␛[0;32mI (32) boot.esp32: SPI Speed      : 40MHz␛[0m
␛[0;32mI (37) boot.esp32: SPI Mode       : DIO␛[0m
␛[0;32mI (41) boot.esp32: SPI Flash Size : 4MB␛[0m
␛[0;32mI (46) boot: Enabling RNG early entropy source...␛[0m
␛[0;32mI (51) boot: Partition Table:␛[0m
␛[0;32mI (55) boot: ## Label            Usage          Type ST Offset   Length␛[0m
␛[0;32mI (62) boot:  0 nvs              WiFi data        01 02 00009000 00006000␛[0m
␛[0;32mI (69) boot:  1 phy_init         RF data          01 01 0000f000 00001000␛[0m
␛[0;32mI (77) boot:  2 factory          factory app      00 00 00010000 00100000␛[0m
␛[0;32mI (84) boot: End of partition table␛[0m
␛[0;32mI (89) esp_image: segment 0: paddr=00010020 vaddr=3f400020 size=0a574h ( 42356) map␛[0m
␛[0;32mI (112) esp_image: segment 1: paddr=0001a59c vaddr=3ffb0000 size=01f0ch (  7948) load␛[0m
␛[0;32mI (116) esp_image: segment 2: paddr=0001c4b0 vaddr=40080000 size=03b68h ( 15208) load␛[0m
␛[0;32mI (125) esp_image: segment 3: paddr=00020020 vaddr=400d0020 size=18df4h (101876) map␛[0m
␛[0;32mI (163) esp_image: segment 4: paddr=00038e1c vaddr=40083b68 size=09418h ( 37912) load␛[0m
␛[0;32mI (186) boot: Loaded app from partition at offset 0x10000␛[0m
␛[0;32mI (186) boot: Disabling RNG early entropy source...␛[0m
␛[0;32mI (197) cpu_start: Pro cpu up.␛[0m
␛[0;32mI (198) cpu_start: Starting app cpu, entry point is 0x40081a90␛[0m
␛[0;32mI (184) cpu_start: App cpu up.␛[0m
␛[0;32mI (212) cpu_start: Pro cpu start user code␛[0m
␛[0;32mI (212) cpu_start: cpu freq: 160000000 Hz␛[0m
␛[0;32mI (212) cpu_start: Application information:␛[0m
␛[0;32mI (217) cpu_start: Project name:     Borda_3␛[0m
␛[0;32mI (222) cpu_start: App version:      dc28255-dirty␛[0m
␛[0;32mI (227) cpu_start: Compile time:     Apr  5 2023 19:46:18␛[0m
␛[0;32mI (233) cpu_start: ELF file SHA256:  37eae859a15c4082...␛[0m
␛[0;32mI (239) cpu_start: ESP-IDF:          5.0.1␛[0m
␛[0;32mI (244) cpu_start: Min chip rev:     v0.0␛[0m
␛[0;32mI (249) cpu_start: Max chip rev:     v3.99 ␛[0m
␛[0;32mI (254) cpu_start: Chip rev:         v3.0␛[0m
␛[0;32mI (258) heap_init: Initializing. RAM available for dynamic allocation:␛[0m
␛[0;32mI (266) heap_init: At 3FFAE6E0 len 00001920 (6 KiB): DRAM␛[0m
␛[0;32mI (272) heap_init: At 3FFB2830 len 0002D7D0 (181 KiB): DRAM␛[0m
␛[0;32mI (278) heap_init: At 3FFE0440 len 00003AE0 (14 KiB): D/IRAM␛[0m
␛[0;32mI (284) heap_init: At 3FFE4350 len 0001BCB0 (111 KiB): D/IRAM␛[0m
␛[0;32mI (291) heap_init: At 4008CF80 len 00013080 (76 KiB): IRAM␛[0m
␛[0;32mI (298) spi_flash: detected chip: gd␛[0m
␛[0;32mI (301) spi_flash: flash io: dio␛[0m
␛[0;32mI (306) cpu_start: Starting scheduler on PRO CPU.␛[0m
␛[0;32mI (0) cpu_start: Starting scheduler on APP CPU.␛[0m
␛[0;32mI (317) BMP180 I2C Driver: I2C master driver has been installed.␛[0m
␛[0;32mI (327) BMP180 I2C Driver: BMP180 sensor found at 0x77␛[0m
␛[0;32mI (337) BMP180 I2C Driver: AC1: 8428, AC2: -1171, AC3: -14681, AC4: 33922, AC5: 25315, AC6: 15135␛[0m
␛[0;32mI (337) BMP180 I2C Driver: B1: 6515, B2: 46, MB: -32768, MC: -11786, MD: 2628␛[0m
␛[0;32mI (347) BMP180: 0␛[0m
␛[0;32mI (1347) BH1750: sda_io_num 21␛[0m
␛[0;32mI (1347) BH1750: scl_io_num 22␛[0m
␛[0;32mI (1347) BH1750: clk_speed 10000␛[0m
␛[0;32mI (1347) BH1750: i2c_param_config 1␛[0m
␛[0;32mI (1347) BH1750: i2c_driver_install 0␛[0m
␛[0;31mE (1347) i2c: i2c driver install error␛[0m
ESP_ERROR_CHECK failed: esp_err_t 0xffffffff (ESP_FAIL) at 0x400d0f4d
file: "src/components/bh1750.c" line 125
func: bh1750_init
expression: i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0)

abort() was called at PC 0x40086a47 on core 0


Backtrace: 0x40082526:0x3ffb49d0 0x40086a51:0x3ffb49f0 0x4008bb46:0x3ffb4a10 0x40086a47:0x3ffb4a80 0x400d0f4d:0x3ffb4ab0 0x400d1745:0x3ffb4af0 0x400e8782:0x3ffb4b30 0x400895bd:0x3ffb4b60




ELF file SHA256: 37eae859a15c4082

Rebooting...


*/

void bh1750_deinit(void) {
	ESP_LOGI(BHTAG, "i2c_driver_delete");
	ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));

}

/**	@brief The function is used as I2C bus write
*	\return : Status of the I2C read
*	\param dev_addr : The device address of the sensor
*	\param reg_addr : Address of the first register, data is going to be read from
*	\param reg_data : This data read from the sensor
*	\param cnt : The no of data bytes to be read
*/
int bh1750_I2C_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt) {
	int ret = 0;

	ESP_LOGD(BHTAG, "bh1750_I2C_write I2CAddress 0x%02X len %d reg 0x%02X", dev_addr,cnt,reg_addr);
	if (cnt>0 && reg_data != NULL && LOG_LOCAL_LEVEL >= ESP_LOG_DEBUG) {
		for (int pos = 0; pos < cnt; pos++) {
			printf("0x%02X ",*(reg_data + pos));
		}
		printf("\n");
	}
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, dev_addr<<1| I2C_MASTER_WRITE, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
	if (cnt>0 && reg_data != NULL) {
		i2c_master_write(cmd, reg_data, cnt, ACK_CHECK_EN);
	}
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
    	ESP_LOGE(BHTAG, "bh1750_I2C_write write data fail I2CAddress 0x%02X len %d reg 0x%02X", dev_addr,cnt,reg_addr);
    }

	return ret;
}

 /**
  * @brief: The function is used as I2C bus read
 *	\return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be read
 *	\param reg_data : This data read from the sensor, which is hold in an array
 *	\param cnt : The no of data byte of to be read
 */
int bh1750_I2C_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt) {
	int ret = 0;
	int pos;
	i2c_cmd_handle_t cmd;

	ESP_LOGD(BHTAG, "bh1750_I2C_read I2CAddress 0x%02X len %d reg 0x%02X", dev_addr,cnt,reg_addr);

	if (reg_addr!=0xFF) {
		cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, dev_addr<<1| I2C_MASTER_WRITE, ACK_CHECK_EN);
		i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
		i2c_master_stop(cmd);
		ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
		i2c_cmd_link_delete(cmd);
		if (ret != ESP_OK) {
			ESP_LOGE(BHTAG, "bh1750_I2C_read write reg fail %d",ret);
			return ret;
		}
	}

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, dev_addr<<1| I2C_MASTER_READ, ACK_CHECK_EN);
	for (pos = 0; pos < (cnt-1); pos++) {
		i2c_master_read_byte(cmd, reg_data + pos, ACK_VAL);
	}
	i2c_master_read_byte(cmd, reg_data + cnt -1, NACK_VAL);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
    	ESP_LOGE(BHTAG, "bh1750_I2C_read read data fail %d",ret);
        return ret;
    }
    if (LOG_LOCAL_LEVEL >= ESP_LOG_DEBUG) {
		for (pos = 0; pos < cnt; pos++) {
			printf("0x%02X ",*(reg_data + pos));
		}
		printf("\n");
    }


	return ret;
}