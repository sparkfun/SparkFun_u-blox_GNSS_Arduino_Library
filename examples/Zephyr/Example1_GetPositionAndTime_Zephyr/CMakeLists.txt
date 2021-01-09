# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr HINTS $ENV{ZEPHYR_BASE})
project(sparkfun_ublox_zephyr_library)

zephyr_compile_options(-fdiagnostics-color=always)

zephyr_include_directories(.)
target_sources(app PRIVATE src/SparkFun_Ublox_Zephyr_Library.cpp)
target_sources(app PRIVATE src/SparkFun_Ublox_Zephyr_Interface.cpp)

target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/main.c)
