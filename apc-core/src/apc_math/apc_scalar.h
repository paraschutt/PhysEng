#pragma once
#include "apc_config.h"

namespace apc {

#ifdef APC_USE_FIXED_POINT
    // Future mobile path (e.g., 32.32 fixed point)
    using apc_scalar = int64_t;
    // ... fixed point math specializations ...
#else
    using apc_scalar = float;
#endif

// Strictly bounded math operations that work for both float and future fixed-point
namespace math {
    APC_FORCEINLINE apc_scalar sqrt(apc_scalar v) {
#ifdef APC_USE_FIXED_POINT
        // Future: Integer square root algorithm
#else
        return std::sqrt(v);
#endif
    }
    
    APC_FORCEINLINE apc_scalar sin(apc_scalar v) {
#ifdef APC_USE_FIXED_POINT
        // Future: CORDIC or LUT
#else
        return std::sin(v);
#endif
    }
    
    APC_FORCEINLINE apc_scalar abs(apc_scalar v) {
        return v < 0 ? -v : v;
    }
}

} // namespace apc