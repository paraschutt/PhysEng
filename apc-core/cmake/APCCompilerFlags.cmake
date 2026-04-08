# APC Compiler Flags - Determinism First

function(apc_set_deterministic_flags target)
    # Force C++17 standard (Modern CMake way - works on MSVC, GCC, Clang)
    target_compile_features(${target} PUBLIC cxx_std_17)
    
    # Warnings as errors
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /WX /Z7)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Werror -g
            -Wno-unused-but-set-variable -Wno-unused-parameter -Wno-unused-variable
        )
    endif()

    # Strict Determinism Flags
    if(MSVC)
        target_compile_options(${target} PRIVATE /fp:strict /Zp8)
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        target_compile_options(${target} PRIVATE 
            -ffp-contract=off -fno-fast-math -fno-unsafe-math-optimizations 
            -fno-finite-math-only -fno-rounding-math -fexcess-precision=standard
            -fno-tree-vectorize -fno-tree-slp-vectorize -fno-interleave-cfg
            -fpack-struct=8
        )
    else() # GCC
        target_compile_options(${target} PRIVATE 
            -ffp-contract=off -fno-fast-math -fno-unsafe-math-optimizations 
            -fno-finite-math-only -fno-rounding-math -fno-signaling-nans 
            -fno-trapping-math -fno-math-errno -fexcess-precision=standard
            -fno-tree-vectorize -fno-tree-slp-vectorize -fpack-struct=8
        )
    endif()

    target_compile_definitions(${target} PRIVATE
        APC_DETERMINISM_ENABLED=1
        APC_STRICT_FP=1
    )
    
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        target_compile_definitions(${target} PRIVATE APC_VERIFY_FP_MODE_EVERY_FRAME=1)
    endif()
endfunction()