# APC Compiler Flags - Determinism First
# These flags are NON-NEGOTIABLE for any target linking apc_core

function(apc_set_deterministic_flags target)
    set(APC_COMMON_FLAGS
        # C++ standard
        /std:c++17    # MSVC
        -std=c++17    # GCC/Clang
        
        # Warnings as errors
        /W4 /WX       # MSVC
        -Wall -Wextra -Werror  # GCC/Clang
        
        # Debug info (always, for crash analysis)
        /Z7           # MSVC
        -g            # GCC/Clang
    )
    
    set(APC_DETERMINISM_FLAGS_MSVC
        # STRICT floating point
        /fp:strict
        
        # Disable FMA (fused multiply-add breaks determinism)
        /d2FFastMath-          # Disable fast math in backend
        
        # Ensure consistent struct packing
        /Zp8
        
        # No intrinsics auto-vectorization that might reorder FP
        /d2fp:strict
    )
    
    set(APC_DETERMINISM_FLAGS_GCC
        # EXPLICITLY disable fast math
        -ffp-contract=off      # NO FMA fusion
        -fno-fast-math         # Disable ALL fast-math flags
        -fno-unsafe-math-optimizations
        -fno-finite-math-only  # Keep NaN/Inf handling consistent
        -fno-rounding-math
        -fno-signaling-nans
        -fno-trapping-math
        -fexcess-precision=standard  # x86: don't use 80-bit intermediates
        
        # Explicitly set rounding mode in code, but don't let compiler assume
        -fno-math-errno        # Don't set errno (consistent behavior)
        
        # Disable auto-vectorization that might reorder FP ops
        -fno-tree-vectorize
        -fno-tree-slp-vectorize
        
        # Struct packing
        -fpack-struct=8
    )
    
    set(APC_DETERMINISM_FLAGS_CLANG
        # Same as GCC plus Clang-specific
        -ffp-contract=off
        -fno-fast-math
        -fno-unsafe-math-optimizations
        -fno-finite-math-only
        -fno-rounding-math
        -fexcess-precision=standard
        
        # Clang-specific
        -fno-slp-vectorize
        -fno-vectorize
        -fno-interleave-cfg
    )
    
    if(MSVC)
        target_compile_options(${target} PRIVATE ${APC_DETERMINISM_FLAGS_MSVC})
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        target_compile_options(${target} PRIVATE ${APC_DETERMINISM_FLAGS_CLANG})
    else()  # GCC
        target_compile_options(${target} PRIVATE ${APC_DETERMINISM_FLAGS_GCC})
    endif()
    
    # DEFINITIONS
    target_compile_definitions(${target} PRIVATE
        APC_DETERMINISM_ENABLED=1
        APC_STRICT_FP=1
    )
    
    # DEBUG-only definitions
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        target_compile_definitions(${target} PRIVATE
            APC_VERIFY_FP_MODE_EVERY_FRAME=1
            APC_DEBUG_PHYSICS=1
        )
    endif()
    
endfunction()

# Macro to mark a function as hot-path (disables operator overloads)
function(apc_mark_hot_path target)
    target_compile_definitions(${target} PRIVATE APC_SIM_HOT_PATH=1)
endfunction()