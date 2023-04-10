cmake_minimum_required(VERSION 3.16.0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
include_directories(C:/.platformio/packages/framework-espidf)
#for some reason xtensa esp compiler cannot find the ble headers --> adjust for your own system directory tree if you encounter the same problem
include_directories(C:/.platformio/packages/framework-espidf/components/bt/host/bluedroid/api/include/api)
project(Borda_3)
