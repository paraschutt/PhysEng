cmake_minimum_required(VERSION 3.20)
project(APCCore VERSION 0.1.0 LANGUAGES CXX)

include(CTest)

# Include our custom modules
include(cmake/APCCompilerFlags.cmake)
include(cmake/APCDeterminism.cmake)

# Generate apc_config.h from the .in template into the build directory
configure_file(apc_config.h.in ${CMAKE_BINARY_DIR}/apc_config.h)

# ---------------------------------------------------------
# Core Engine Library
# ---------------------------------------------------------
add_library(apc_core STATIC
    # Math
    src/apc_math/apc_vec3.cpp
    src/apc_math/apc_quat.cpp
    src/apc_math/apc_mat3.cpp
    src/apc_math/apc_math_common.cpp
    
    # Platform
    src/apc_platform/apc_fp_mode_x64.cpp
    src/apc_platform/apc_fp_mode_arm64.cpp
    # src/apc_platform/apc_fp_mode_console.cpp # Uncomment when targeting specific consoles
)

# Enforce strict FP rules on the core library
apc_set_deterministic_flags(apc_core)

# Tell the compiler where to find our headers
target_include_directories(apc_core 
    PUBLIC 
        # Find headers like "apc_math/apc_vec3.h"
        ${CMAKE_CURRENT_SOURCE_DIR}/src 
        
        # Find the generated "apc_config.h"
        ${CMAKE_BINARY_DIR}
)

# ---------------------------------------------------------
# Application Executable
# ---------------------------------------------------------
add_executable(apc_application
    src/apc_app/main.cpp
)
target_link_libraries(apc_application PRIVATE apc_core)
apc_set_deterministic_flags(apc_application)

# ---------------------------------------------------------
# Tests
# ---------------------------------------------------------
add_subdirectory(tests)