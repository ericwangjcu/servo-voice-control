cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(EE3600_Motor_PID_RP2040 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(EE3600_Motor_PID_RP2040
    main.c
)

target_link_libraries(EE3600_Motor_PID_RP2040
    pico_stdlib
    hardware_pwm
    hardware_gpio
    hardware_clocks
    hardware_sync
)

pico_enable_stdio_usb(EE3600_Motor_PID_RP2040 1)
pico_enable_stdio_uart(EE3600_Motor_PID_RP2040 0)

pico_add_extra_outputs(EE3600_Motor_PID_RP2040)
