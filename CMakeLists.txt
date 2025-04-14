# CMake configuration for the nRF52-DK Demo Project using Zephyr

cmake_minimum_required(VERSION 3.20.0)

# Define board
set(BOARD nrf52dk/nrf52832)

# Set devicetree overlays
set(DTC_OVERLAY_FILE boards/nrf52832.overlay)

# Pull Zephyr build system
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# Define project
project(zephyr_nrf52_demo)

# Verify the project is built through sysbuild to ensure MCUboot is
# automatically included
test_sysbuild()

# Add sources
file(
  GLOB
  SOURCES_APP
  src/demo_task1.c
  src/demo_task2.c
  src/sensor_task.c
  src/serialization.c
  src/mpu_driver/*.c)

# BLE task sources
if(CONFIG_BLE_SENSOR_DATA)
  list(APPEND SOURCES_APP ${CMAKE_CURRENT_LIST_DIR}/src/ble_task.c)
endif()

# UART task sources
if(CONFIG_UART_SENSOR_DATA)
  list(APPEND SOURCES_APP ${CMAKE_CURRENT_LIST_DIR}/src/uart_task.c)
endif()

target_sources(app PRIVATE src/main.c ${SOURCES_APP})

# Build the llext
set(ext_name user_app)
file(GLOB ext_src ${CMAKE_CURRENT_LIST_DIR}/src/app/*.c)
set(ext_bin ${PROJECT_BINARY_DIR}/llext/${ext_name}.llext)
set(ext_inc ${ZEPHYR_BINARY_DIR}/include/generated/${ext_name}_ext.inc)
add_llext_target(${ext_name}_ext OUTPUT ${ext_bin} SOURCES ${ext_src})
generate_inc_file_for_target(app ${ext_bin} ${ext_inc})

# Include headers
target_include_directories(app PRIVATE src/mpu_driver)
