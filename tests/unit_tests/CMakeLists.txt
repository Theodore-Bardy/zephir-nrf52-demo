# CMake configuration for the main module tests

cmake_minimum_required(VERSION 3.20.0)

set(ROOT_DIR_PROJECT ../..)

find_package(
  Zephyr
  COMPONENTS unittest
  REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(unit_test)

file(GLOB TEST_SOURCES ${CMAKE_CURRENT_LIST_DIR}/src/main.c)
target_sources(testbinary PRIVATE ${TEST_SOURCES}
                                  ${ROOT_DIR_PROJECT}/src/serialization.c)

# Include headers
target_include_directories(testbinary PRIVATE ${ROOT_DIR_PROJECT}/src)
