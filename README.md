# Environmental Sensing BLE DEVICE
## Implemented on the ESP32 DOIT DEVKIT V1 Board


### Sensors:
- BMP180 Pressure and Temperature Sensor
- BH1750 Ambient Light Sensor

### Sensor Libraries:
**(slightly modified by me)**
- [BMP180](https://github.com/ESP32Tutorials/BMP180-ESP32-ESP-IDF/tree/main/components)
- [BH1750](https://github.com/pcbreflux/espressif/tree/master/esp32/app/ESP32_bh1750_oled/main)

## Current Status (April 2023): Under Development
### Working Features
- sensor input reading functional with occasional minor errors
    - Pressure (Pa)
    - Temperature (C) (not broadcasted in latest version)
    - Ambient light (lux)
- Moving median function for eliminating sensor noise
- BLE output of sensor reading values
- LIFO queue for BLE output
### Coming Features
- RTOS multithreaded sensor read and broadcast functionality
    - note: will take some time : )
