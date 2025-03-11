# CMake configuration for the nRF52-DK Demo Project using Zephyr

cmake_minimum_required(VERSION 3.20.0)

# Define board
set(BOARD nrf52dk/nrf52832)

# Set devicetree overlays
set(DTC_OVERLAY_FILE boards/nrf52832.overlay)

# Pull Zephyr build system
find_package(Zephyr REQUIRED)

# Define project
project(zephyr_nrf52_demo)

# Include nanopb module
list(APPEND CMAKE_MODULE_PATH "${ZEPHYR_BASE}/modules/nanopb")
include(nanopb)
zephyr_nanopb_sources(app src/simple.proto)

# Add sources
target_sources(app PRIVATE src/main.c src/mpu_driver/mpu6050.c src/ble/ble.c
                           src/ble/stream_tx.c src/ble/stream_lc3.c)

# Include headers
target_include_directories(app PRIVATE src/mpu_driver)
target_include_directories(app PRIVATE src/ble)
