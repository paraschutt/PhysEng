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
# Application Executable (headless)
# ---------------------------------------------------------
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/apc_app/main.cpp")
    add_executable(apc_application
        src/apc_app/main.cpp
    )
    target_link_libraries(apc_application PRIVATE apc_core)
    apc_set_deterministic_flags(apc_application)
endif()

# ---------------------------------------------------------
# SDL2 Visualizer (Optional — only if SDL2 is found)
# ---------------------------------------------------------
# Works on:
#   Windows: vcpkg install sdl2:x64-windows  (or MSYS2 pacman -S mingw-w64-x86_64-SDL2)
#   Linux:   apt-get install libsdl2-dev       (or use third_party/sdl2)
#   macOS:   brew install sdl2
#
# Usage:
#   cmake -B build -DCMAKE_TOOLCHAIN_FILE=[vcpkg]/scripts/buildsystems/vcpkg.cmake
#   cmake --build build --target apc_sdl2_application
# ---------------------------------------------------------

message(STATUS "")
message(STATUS "=== SDL2 Visualizer Setup ===")

# Try find_package first (works with vcpkg, system installs, MSYS2)
# SDL2 provides both a Config and Module find; we prefer Config for vcpkg compat.
find_package(SDL2 QUIET CONFIG)
if(NOT SDL2_FOUND AND NOT TARGET SDL2::SDL2 AND NOT TARGET SDL2::SDL2-static)
    find_package(SDL2 QUIET MODULE)
    if(NOT SDL2_FOUND)
        set(SDL2_FOUND FALSE)
    endif()
endif()

# Manual SDL2_DIR hint support (user sets -DSDL2_DIR=...)
# Only CONFIG mode supports PATHS/NO_DEFAULT_PATH; MODULE mode does not.
if(NOT SDL2_FOUND AND NOT TARGET SDL2::SDL2 AND NOT TARGET SDL2::SDL2-static AND DEFINED SDL2_DIR)
    find_package(SDL2 QUIET CONFIG PATHS ${SDL2_DIR} NO_DEFAULT_PATH)
endif()

# Linux fallback: try pkg-config
if(NOT SDL2_FOUND AND NOT WIN32)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(SDL2_PC QUIET sdl2)
        if(SDL2_PC_FOUND)
            set(SDL2_FOUND TRUE)
            message(STATUS "SDL2 found via pkg-config: ${SDL2_PC_LIBRARIES}")
        endif()
    endif()
endif()

# Linux fallback: try third_party/sdl2 extraction
if(NOT SDL2_FOUND AND NOT WIN32 AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/sdl2")
    set(SDL2_FOUND TRUE)

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
    else()
        message(STATUS "SDL2 found via third_party/sdl2")
    endif()
endif()

# Final check: consider SDL2 found if either the variable or a CMake target exists
if(NOT SDL2_FOUND AND TARGET SDL2::SDL2)
    set(SDL2_FOUND TRUE)
    message(STATUS "SDL2 found via CMake target SDL2::SDL2")
endif()
if(NOT SDL2_FOUND AND TARGET SDL2::SDL2-static)
    set(SDL2_FOUND TRUE)
    set(SDL2_USE_STATIC ON)
    message(STATUS "SDL2 found via CMake target SDL2::SDL2-static")
endif()

if(SDL2_FOUND)
    message(STATUS "SDL2 found — building apc_sdl2_application target")

    # Prefer src/apc_app/sdl2_main.cpp (full soccer sim); fall back to
    # src/apc_sdl2/apc_sdl2_application.cpp (simple bouncing balls demo)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/apc_app/sdl2_main.cpp")
        set(_SDL2_SRC src/apc_app/sdl2_main.cpp)
    elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/apc_sdl2/apc_sdl2_application.cpp")
        set(_SDL2_SRC src/apc_sdl2/apc_sdl2_application.cpp)
    else()
        message(WARNING "SDL2 source file not found — skipping apc_sdl2_application")
        set(SDL2_FOUND 0)
    endif()
endif()

if(SDL2_FOUND)
    add_executable(apc_sdl2_application ${_SDL2_SRC})

    # Core engine dependency
    target_link_libraries(apc_sdl2_application PRIVATE apc_core)

    # C++17 required
    target_compile_features(apc_sdl2_application PRIVATE cxx_std_17)

    # SDL2 linking — handle different find_package results
    if(TARGET SDL2::SDL2)
        # Modern CMake target (vcpkg dynamic, newer system installs)
        target_link_libraries(apc_sdl2_application PRIVATE SDL2::SDL2)
        if(TARGET SDL2::SDL2main)
            target_link_libraries(apc_sdl2_application PRIVATE SDL2::SDL2main)
        endif()
    elseif(TARGET SDL2::SDL2-static)
        # vcpkg static build
        target_link_libraries(apc_sdl2_application PRIVATE SDL2::SDL2-static)
        if(TARGET SDL2::SDL2main)
            target_link_libraries(apc_sdl2_application PRIVATE SDL2::SDL2main)
        endif()
    elseif(SDL2_PC_FOUND)
        # pkg-config path (Linux)
        target_include_directories(apc_sdl2_application PRIVATE ${SDL2_PC_INCLUDE_DIRS})
        target_link_libraries(apc_sdl2_application PRIVATE ${SDL2_PC_LIBRARIES})
        target_link_directories(apc_sdl2_application PRIVATE ${SDL2_PC_LIBRARY_DIRS})
    elseif(DEFINED SDL2_INCLUDE_DIRS AND DEFINED SDL2_LIBRARIES)
        # Module find or third_party path
        target_include_directories(apc_sdl2_application PRIVATE ${SDL2_INCLUDE_DIRS})
        target_link_libraries(apc_sdl2_application PRIVATE ${SDL2_LIBRARIES})
        if(DEFINED SDL2_MAIN_LIBRARIES)
            target_link_libraries(apc_sdl2_application PRIVATE ${SDL2_MAIN_LIBRARIES})
        endif()
        if(SDL2_LIBRARY_DIRS)
            target_link_directories(apc_sdl2_application PRIVATE ${SDL2_LIBRARY_DIRS})
        endif()

        # Also link against system SDL2 shared library for runtime
        # (static libSDL2.a requires too many system deps — wayland, X11, etc.)
        if(NOT WIN32)
            find_library(SDL2_RUNTIME_LIB SDL2-2.0 PATHS /lib/x86_64-linux-gnu /usr/lib/x86_64-linux-gnu NO_DEFAULT_PATH)
            if(SDL2_RUNTIME_LIB)
                message(STATUS "SDL2 runtime lib: ${SDL2_RUNTIME_LIB}")
                target_link_libraries(apc_sdl2_application PRIVATE ${SDL2_RUNTIME_LIB})
            elseif(EXISTS "/lib/x86_64-linux-gnu/libSDL2-2.0.so.0")
                target_link_libraries(apc_sdl2_application PRIVATE "/lib/x86_64-linux-gnu/libSDL2-2.0.so.0")
                message(STATUS "SDL2 runtime lib: /lib/x86_64-linux-gnu/libSDL2-2.0.so.0 (direct)")
            endif()
        endif()
    endif()

    # Platform-specific libraries
    if(WIN32)
        target_link_libraries(apc_sdl2_application PRIVATE opengl32 glu32)
    elseif(APPLE)
        find_library(OPENGL_FRAMEWORK OpenGL REQUIRED)
        target_link_libraries(apc_sdl2_application PRIVATE ${OPENGL_FRAMEWORK})
    else()
        # SDL2 needs threads, dl, and GL on Linux
        find_package(Threads REQUIRED)
        target_link_libraries(apc_sdl2_application PRIVATE Threads::Threads GL dl)
    endif()

    # Warnings but don't error the visualizer (it's not in the deterministic hot path)
    if(MSVC)
        target_compile_options(apc_sdl2_application PRIVATE /W4 /Z7)
    else()
        target_compile_options(apc_sdl2_application PRIVATE -Wall -Wextra -g)
    endif()

    message(STATUS "SDL2 visualizer target: apc_sdl2_application")
else()
    message(WARNING "SDL2 not found — apc_sdl2_application target will NOT be built")
    message(STATUS "")
    message(STATUS "To enable SDL2 on Windows:")
    message(STATUS "  1. Install vcpkg:  git clone https://github.com/microsoft/vcpkg && cd vcpkg && bootstrap-vcpkg.bat")
    message(STATUS "  2. Install SDL2:  vcpkg install sdl2:x64-windows")
    message(STATUS "  3. Configure:    cmake -B build -DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake")
    message(STATUS "  OR with MSYS2:")
    message(STATUS "  1. pacman -S mingw-w64-x86_64-SDL2")
    message(STATUS "  2. cmake -B build -G \"MinGW Makefiles\"")
    message(STATUS "")
    message(STATUS "To enable SDL2 on Linux:")
    message(STATUS "  sudo apt-get install libsdl2-dev")
    message(STATUS "  cmake -B build && cmake --build build --target apc_sdl2_application")
    message(STATUS "")
endif()

message(STATUS "=== End SDL2 Setup ===")
message(STATUS "")

# ---------------------------------------------------------
# Tests
# ---------------------------------------------------------
add_subdirectory(tests)
