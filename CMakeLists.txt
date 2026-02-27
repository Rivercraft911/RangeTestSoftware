# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(RangeTestSoftware C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(RangeTestSoftware
        RangeTestSoftware.c
        src/tests/range_test_node.c
        src/hal/board_pins.c
        src/radio/radio_sx1280f27.c
        src/radio/radio_rfm98pw.c
        )

pico_set_program_name(RangeTestSoftware "RangeTestSoftware")
pico_set_program_version(RangeTestSoftware "0.1")

target_compile_definitions(RangeTestSoftware PRIVATE RANGE_NODE_NO_MAIN SX1280F27_LOG_LEVEL=2)

# Generate PIO header
pico_generate_pio_header(RangeTestSoftware ${CMAKE_CURRENT_LIST_DIR}/pio/ws2812.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(RangeTestSoftware 0)
pico_enable_stdio_usb(RangeTestSoftware 1)

# Add the standard library to the build
target_link_libraries(RangeTestSoftware
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(RangeTestSoftware PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/include
)

# Add any user requested libraries
target_link_libraries(RangeTestSoftware 
        hardware_spi
        hardware_pio
        hardware_timer
        hardware_watchdog
        )

pico_add_extra_outputs(RangeTestSoftware)

add_executable(ManualRadioBringupTest
        src/tests/manual_radio_bringup_test.c
        src/hal/board_pins.c
        src/radio/radio_sx1280f27.c
        src/radio/radio_rfm98pw.c
        )

pico_set_program_name(ManualRadioBringupTest "ManualRadioBringupTest")
pico_set_program_version(ManualRadioBringupTest "0.1")

pico_generate_pio_header(ManualRadioBringupTest ${CMAKE_CURRENT_LIST_DIR}/pio/ws2812.pio)

pico_enable_stdio_uart(ManualRadioBringupTest 0)
pico_enable_stdio_usb(ManualRadioBringupTest 1)

target_link_libraries(ManualRadioBringupTest
        pico_stdlib
        hardware_spi
        hardware_pio
        hardware_timer
        hardware_watchdog
        )

target_include_directories(ManualRadioBringupTest PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/include
)

pico_add_extra_outputs(ManualRadioBringupTest)

add_executable(UsbConsoleSanityTest
        src/tests/usb_console_sanity_test.c
        )

pico_set_program_name(UsbConsoleSanityTest "UsbConsoleSanityTest")
pico_set_program_version(UsbConsoleSanityTest "0.1")

pico_enable_stdio_uart(UsbConsoleSanityTest 0)
pico_enable_stdio_usb(UsbConsoleSanityTest 1)

target_link_libraries(UsbConsoleSanityTest
        pico_stdlib
        )

pico_add_extra_outputs(UsbConsoleSanityTest)

add_executable(UHFConstantFullPowerTx
        src/tests/uhf_constant_full_power_tx.c
        src/hal/board_pins.c
        src/radio/radio_rfm98pw.c
        )

pico_set_program_name(UHFConstantFullPowerTx "UHFConstantFullPowerTx")
pico_set_program_version(UHFConstantFullPowerTx "0.1")

pico_enable_stdio_uart(UHFConstantFullPowerTx 0)
pico_enable_stdio_usb(UHFConstantFullPowerTx 1)

target_link_libraries(UHFConstantFullPowerTx
        pico_stdlib
        hardware_spi
        hardware_timer
        )

target_include_directories(UHFConstantFullPowerTx PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/include
)

pico_add_extra_outputs(UHFConstantFullPowerTx)

add_executable(RangeTestNode
        src/tests/range_test_node.c
        src/hal/board_pins.c
        src/radio/radio_sx1280f27.c
        src/radio/radio_rfm98pw.c
        )

pico_set_program_name(RangeTestNode "RangeTestNode")
pico_set_program_version(RangeTestNode "0.1")

pico_generate_pio_header(RangeTestNode ${CMAKE_CURRENT_LIST_DIR}/pio/ws2812.pio)

pico_enable_stdio_uart(RangeTestNode 0)
pico_enable_stdio_usb(RangeTestNode 1)

target_link_libraries(RangeTestNode
        pico_stdlib
        hardware_spi
        hardware_pio
        hardware_timer
        hardware_watchdog
        )

target_include_directories(RangeTestNode PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/include
)

target_compile_definitions(RangeTestNode PRIVATE SX1280F27_LOG_LEVEL=2)

pico_add_extra_outputs(RangeTestNode)

add_executable(RangeTestNodeA
        src/tests/range_test_node.c
        src/hal/board_pins.c
        src/radio/radio_sx1280f27.c
        src/radio/radio_rfm98pw.c
        )

target_compile_definitions(RangeTestNodeA PRIVATE RANGE_NODE_FORCE_ROLE=1 SX1280F27_LOG_LEVEL=2)

pico_set_program_name(RangeTestNodeA "RangeTestNodeA")
pico_set_program_version(RangeTestNodeA "0.1")

pico_generate_pio_header(RangeTestNodeA ${CMAKE_CURRENT_LIST_DIR}/pio/ws2812.pio)

pico_enable_stdio_uart(RangeTestNodeA 0)
pico_enable_stdio_usb(RangeTestNodeA 1)

target_link_libraries(RangeTestNodeA
        pico_stdlib
        hardware_spi
        hardware_pio
        hardware_timer
        hardware_watchdog
        )

target_include_directories(RangeTestNodeA PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/include
)

pico_add_extra_outputs(RangeTestNodeA)

add_executable(RangeTestNodeB
        src/tests/range_test_node.c
        src/hal/board_pins.c
        src/radio/radio_sx1280f27.c
        src/radio/radio_rfm98pw.c
        )

target_compile_definitions(RangeTestNodeB PRIVATE RANGE_NODE_FORCE_ROLE=2 SX1280F27_LOG_LEVEL=2)

pico_set_program_name(RangeTestNodeB "RangeTestNodeB")
pico_set_program_version(RangeTestNodeB "0.1")

pico_generate_pio_header(RangeTestNodeB ${CMAKE_CURRENT_LIST_DIR}/pio/ws2812.pio)

pico_enable_stdio_uart(RangeTestNodeB 0)
pico_enable_stdio_usb(RangeTestNodeB 1)

target_link_libraries(RangeTestNodeB
        pico_stdlib
        hardware_spi
        hardware_pio
        hardware_timer
        hardware_watchdog
        )

target_include_directories(RangeTestNodeB PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/include
)

pico_add_extra_outputs(RangeTestNodeB)

add_executable(RangeTestObserver
        src/tests/range_test_node.c
        src/hal/board_pins.c
        src/radio/radio_sx1280f27.c
        src/radio/radio_rfm98pw.c
        )

target_compile_definitions(RangeTestObserver PRIVATE RANGE_NODE_FORCE_ROLE=3 SX1280F27_LOG_LEVEL=2)

pico_set_program_name(RangeTestObserver "RangeTestObserver")
pico_set_program_version(RangeTestObserver "0.1")

pico_generate_pio_header(RangeTestObserver ${CMAKE_CURRENT_LIST_DIR}/pio/ws2812.pio)

pico_enable_stdio_uart(RangeTestObserver 0)
pico_enable_stdio_usb(RangeTestObserver 1)

target_link_libraries(RangeTestObserver
        pico_stdlib
        hardware_spi
        hardware_pio
        hardware_timer
        hardware_watchdog
        )

target_include_directories(RangeTestObserver PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/include
)

pico_add_extra_outputs(RangeTestObserver)

add_executable(ImageTransfer
        src/tests/image_transfer.c
        src/hal/board_pins.c
        src/radio/radio_sx1280f27.c
        )

target_compile_definitions(ImageTransfer PRIVATE SX1280F27_LOG_LEVEL=2)

pico_set_program_name(ImageTransfer "ImageTransfer")
pico_set_program_version(ImageTransfer "0.1")

pico_enable_stdio_uart(ImageTransfer 0)
pico_enable_stdio_usb(ImageTransfer 1)

target_link_libraries(ImageTransfer
        pico_stdlib
        hardware_spi
        hardware_timer
        )

target_include_directories(ImageTransfer PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/include
)

pico_add_extra_outputs(ImageTransfer)
