# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
set(ENV{UBXLIB_DIR} $../../../ubxlib)
include($ENV{UBXLIB_DIR}/zephyr/ubxlib.cmake)


find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hello_world)
add_definitions(-DSIMPLE_TX)



target_sources(app PRIVATE src/main.c)
file(REAL_PATH "$../../../ubxlib/common" APP_COMMON_DIR)
file(GLOB APP_COMMONS "${APP_COMMON_DIR}/*.c")
target_sources(app PRIVATE ${APP_COMMONS})
target_include_directories(app PRIVATE ${APP_COMMON_DIR})

