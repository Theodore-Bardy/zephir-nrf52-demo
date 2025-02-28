# CMake configuration for the nRF52-DK Demo Project using Zephyr

cmake_minimum_required(VERSION 3.20.0)

# Define board
set(BOARD nrf52dk/nrf52832)

# Pull Zephyr build system
find_package(Zephyr REQUIRED)

# Define project
project(zephyr_nrf52_demo)

# Add sources
target_sources(app PRIVATE src/main.c)

