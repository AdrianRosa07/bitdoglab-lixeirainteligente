# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(bitdoglab_lixeira C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(bitdoglab_lixeira
    main.c
)

target_link_libraries(bitdoglab_lixeira
    pico_stdlib
    hardware_pwm
    hardware_uart
)

pico_enable_stdio_usb(bitdoglab_lixeira 1)
pico_enable_stdio_uart(bitdoglab_lixeira 0)
pico_add_extra_outputs(bitdoglab_lixeira)

# Teste standalone do servo (build separado)
add_executable(bitdoglab_servo_test
    servo_test.c
)

target_link_libraries(bitdoglab_servo_test
    pico_stdlib
    hardware_pwm
    hardware_clocks
)

pico_enable_stdio_usb(bitdoglab_servo_test 1)
pico_enable_stdio_uart(bitdoglab_servo_test 0)
pico_add_extra_outputs(bitdoglab_servo_test)
