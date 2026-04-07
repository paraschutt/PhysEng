#pragma once
#include <cstdint>

// Platform detection (should match apc_config.h.in, duplicated here for header-only safety)
#if defined(_M_X64) || defined(__x86_64__)
    #define APC_PLATFORM_X64 1
#elif defined(_M_ARM64) || defined(__aarch64__)
    #define APC_PLATFORM_ARM64 1
#elif defined(APC_PLATFORM_CONSOLE)
    // Defined by build system
#else
    #error "Unsupported platform for APC"
#endif

namespace apc {

// Raw state required to restore FP mode later (e.g., when calling 3rd party libs)
struct FPUState {
    uint64_t raw_state;
    bool    is_valid;
};

// What the hardware actually supports
struct FPCapabilities {
    bool supports_flush_denormals_to_zero;  // DAZ
    bool supports_denormals_are_zero;       // FTZ
    bool supports_strict_rounding;          // Round-to-nearest-even
    bool has_fma_that_breaks_determinism;   // FMA fused ops
};

// Call ONCE at engine startup. Crashes if mode cannot be achieved.
FPUState enforce_deterministic_fp_mode();

// Call at top of simulation frame in DEBUG builds to catch 3rd party library FP corruption.
bool verify_fp_mode(const FPUState& expected);

// Restore previous FP state before crossing DLL boundaries to non-APC code.
void restore_fp_mode(const FPUState& state);

// Query platform capabilities (informational)
FPCapabilities query_fp_capabilities();

} // namespace apc