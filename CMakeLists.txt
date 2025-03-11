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

# Add sources
target_sources(app PRIVATE src/main.c src/demo_task1.c src/demo_task2.c
                           src/mpu_driver/mpu6050.c)

# Include headers
target_include_directories(app PRIVATE src/mpu_driver)
