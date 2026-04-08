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
# SDL2 Windowed Application (Phase 8)
# ---------------------------------------------------------

# Find SDL2 — try system package first, fall back to bundled third_party
find_package(SDL2 QUIET)

if(NOT SDL2_FOUND)
    message(STATUS "SDL2 not found via find_package, using bundled third_party")
    set(SDL2_FOUND 1)

    # Locate the extracted third-party SDL2 dev package
    set(_SDL2_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/sdl2/usr")

    # Include directories (need both base and multiarch for _real_SDL_config.h)
    set(SDL2_INCLUDE_DIRS "${_SDL2_ROOT}/include")
    if(EXISTS "${_SDL2_ROOT}/include/x86_64-linux-gnu")
        list(APPEND SDL2_INCLUDE_DIRS "${_SDL2_ROOT}/include/x86_64-linux-gnu")
    endif()

    # Link directories and libraries
    if(EXISTS "${_SDL2_ROOT}/lib/x86_64-linux-gnu")
        set(SDL2_LIBRARIES
            "${_SDL2_ROOT}/lib/x86_64-linux-gnu/libSDL2main.a"
        )
        set(SDL2_LIBRARY_DIRS "${_SDL2_ROOT}/lib/x86_64-linux-gnu")
    elseif(EXISTS "${_SDL2_ROOT}/lib")
        set(SDL2_LIBRARIES
            "${_SDL2_ROOT}/lib/libSDL2main.a"
            "${_SDL2_ROOT}/lib/libSDL2.so"
        )
        set(SDL2_LIBRARY_DIRS "${_SDL2_ROOT}/lib")
    endif()

    if(NOT SDL2_INCLUDE_DIRS OR NOT SDL2_LIBRARIES)
        message(WARNING "SDL2 third_party package not found. "
                        "Run: apt-get download libsdl2-dev && dpkg-deb -x libsdl2-dev*.deb third_party/sdl2/")
        set(SDL2_FOUND 0)
    endif()
endif()

if(SDL2_FOUND)
    message(STATUS "SDL2 include: ${SDL2_INCLUDE_DIRS}")
    message(STATUS "SDL2 libraries: ${SDL2_LIBRARIES}")

    add_executable(apc_sdl2_application
        src/apc_app/sdl2_main.cpp
    )
    target_link_libraries(apc_sdl2_application PRIVATE apc_core ${SDL2_LIBRARIES})
    target_include_directories(apc_sdl2_application PRIVATE ${SDL2_INCLUDE_DIRS})
    if(SDL2_LIBRARY_DIRS)
        target_link_directories(apc_sdl2_application PRIVATE ${SDL2_LIBRARY_DIRS})
    endif()

    # Also link against system SDL2 shared library for runtime
    # The static libSDL2.a requires too many system deps (wayland, X11, etc.)
    # Find by soname since the .so symlink may not exist
    find_library(SDL2_RUNTIME_LIB SDL2-2.0 PATHS /lib/x86_64-linux-gnu /usr/lib/x86_64-linux-gnu NO_DEFAULT_PATH)
    if(SDL2_RUNTIME_LIB)
        message(STATUS "SDL2 runtime lib: ${SDL2_RUNTIME_LIB}")
        target_link_libraries(apc_sdl2_application PRIVATE ${SDL2_RUNTIME_LIB})
    else()
        # Fallback: try direct path
        if(EXISTS "/lib/x86_64-linux-gnu/libSDL2-2.0.so.0")
            target_link_libraries(apc_sdl2_application PRIVATE "/lib/x86_64-linux-gnu/libSDL2-2.0.so.0")
            message(STATUS "SDL2 runtime lib: /lib/x86_64-linux-gnu/libSDL2-2.0.so.0 (direct)")
        endif()
    endif()

    # Use C++17
    target_compile_features(apc_sdl2_application PUBLIC cxx_std_17)

    # SDL2 needs threads and dl on Linux
    find_package(Threads REQUIRED)
    target_link_libraries(apc_sdl2_application PRIVATE Threads::Threads dl)
else()
    message(WARNING "SDL2 not available — apc_sdl2_application target will not be built")
endif()

# ---------------------------------------------------------
# Tests
# ---------------------------------------------------------
add_subdirectory(tests)