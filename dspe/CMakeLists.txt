cmake_minimum_required(VERSION 3.25)
project(DSPE VERSION 1.1 LANGUAGES CXX)

# ============================================================================
# C++20 requirement
# ============================================================================
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ============================================================================
# Determinism-critical compiler flags
# See revised brief Section 11 Determinism Checklist:
#   - ffloat-store: flush extended-precision registers on store
#   - mfpmath=sse -msse2: force 32/64-bit SSE FP (prevent x87 80-bit)
#   - ffp-contract=off: disable FMA (different rounding from separate mul+add)
# ============================================================================
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(
        -ffloat-store
        -mfpmath=sse
        -msse2
        -ffp-contract=off
        -Wall
        -Wextra
        -Wno-unused-parameter
    )
elseif(MSVC)
    add_compile_options(
        /fp:precise   # Strict IEEE-754, no extended precision
                      # Note: MSVC has no equivalent of -ffp-contract=off.
                      # /fp:precise disables most FMA contraction by default.
                      # For guaranteed no-FMA, add /fp:strict (disables some opts).
        /W3
        /wd4554       # Suppress: check operator precedence for >> (false positive on .raw >> N)
    )
endif()

# ============================================================================
# Source files
# ============================================================================
set(DSPE_SOURCES
    src/systems/integrator.cpp
    src/systems/broad_phase.cpp
    src/systems/narrow_phase.cpp
    src/systems/constraint_solver.cpp
    src/systems/ccd.cpp
    src/systems/skeleton.cpp
    src/systems/sleep_system.cpp
    src/systems/trigger_system.cpp
    src/world.cpp
)

# ============================================================================
# Main library
# ============================================================================
add_library(dspe STATIC ${DSPE_SOURCES})
target_include_directories(dspe PUBLIC include)

# ============================================================================
# Tests
# ============================================================================
add_executable(dspe_tests tests/test_phase1.cpp)
target_link_libraries(dspe_tests PRIVATE dspe)

enable_testing()
add_test(NAME Phase1Suite COMMAND dspe_tests)

# ============================================================================
# Performance profiling build (stripped, -O3)
# ============================================================================
add_executable(dspe_bench tests/test_phase1.cpp)
target_link_libraries(dspe_bench PRIVATE dspe)
target_compile_options(dspe_bench PRIVATE
    $<IF:$<CXX_COMPILER_ID:MSVC>,/O2 /DNDEBUG,-O3 -DNDEBUG>
)

# ============================================================================
# Sanity: print active determinism flags in build output
# ============================================================================
message(STATUS "DSPE Determinism flags active:")
message(STATUS "  -ffloat-store   (flush extended precision on store)")
message(STATUS "  -mfpmath=sse    (SSE FP, not x87 80-bit)")
message(STATUS "  -msse2          (SSE2 instruction set)")
message(STATUS "  -ffp-contract=off (no FMA fusion)")