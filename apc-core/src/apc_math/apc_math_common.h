#pragma once
#include <cstdint>  // REQUIRED FOR MSVC
#include <cmath>

// Cross-platform force inline definition
#ifndef APC_FORCEINLINE
    #if defined(_MSC_VER)
        #define APC_FORCEINLINE __forceinline
    #else
        #define APC_FORCEINLINE __attribute__((always_inline)) inline
    #endif
#endif

namespace apc {
    // Tolerances
    constexpr float APC_EPSILON      = 1.0e-6f;
    constexpr float APC_EPSILON_SQ   = APC_EPSILON * APC_EPSILON;
    constexpr float APC_PI           = 3.14159265358979323846f;
    constexpr float APC_TWO_PI       = APC_PI * 2.0f;
    constexpr float APC_HALF_PI      = APC_PI * 0.5f;

    // Scalar type abstraction
    #ifdef APC_USE_FIXED_POINT
        using apc_scalar = int64_t; // Future implementation
    #else
        using apc_scalar = float;
    #endif

    // Math operations wrapper
    namespace math {
        APC_FORCEINLINE apc_scalar sqrt(apc_scalar v) {
#ifdef APC_USE_FIXED_POINT
            return 0; 
#else
            return std::sqrt(v);
#endif
        }

        APC_FORCEINLINE apc_scalar abs(apc_scalar v) {
#ifdef APC_USE_FIXED_POINT
            return v < 0 ? -v : v;
#else
            return std::fabs(v);
#endif
        }

        APC_FORCEINLINE apc_scalar sin(apc_scalar v) {
#ifdef APC_USE_FIXED_POINT
            return 0;
#else
            return std::sin(v);
#endif
        }

        APC_FORCEINLINE apc_scalar cos(apc_scalar v) {
#ifdef APC_USE_FIXED_POINT
            return 0;
#else
            return std::cos(v);
#endif
        }
    }
}