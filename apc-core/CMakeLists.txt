cmake_minimum_required(VERSION 3.20)
project(APCCore VERSION 0.1.0 LANGUAGES CXX)

include(cmake/APCCompilerFlags.cmake)
include(cmake/APCDeterminism.cmake)

add_subdirectory(src)
add_subdirectory(tests)

configure_file(apc_config.h.in \/apc_config.h)
target_include_directories(apc_core PRIVATE \)
