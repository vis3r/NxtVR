cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(NxtVR_pico)
pico_sdk_init()
add_executable(nxtvr    
   nxtvr.cpp
   sensors/mpu6050/mpu6050.cpp
   usb_descriptors.cpp)
target_link_libraries(nxtvr pico_stdlib hardware_i2c tinyusb_device tinyusb_board) 
target_include_directories(nxtvr PRIVATE ${CMAKE_CURRENT_LIST_DIR})
pico_add_extra_outputs(nxtvr)

pico_enable_stdio_usb(nxtvr 1)

