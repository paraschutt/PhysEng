#pragma once
#include "apc_config.h"

namespace apc {

// Represents the FP state we REQUIRE for simulation
struct FPUState {
    // x64: SSE control word, x87 control word (legacy)
    // ARM64: FPCR
    // All: whether we successfully set the mode
    uint64_t raw_state;
    bool    is_valid;
};

// Call ONCE at engine startup before any physics runs.
// Returns the state we set (for verification later).
// Crashes with detailed error if mode cannot be achieved.
FPUState APC_CALL enforce_deterministic_fp_mode();

// Call to verify we're still in the right mode.
// Use this at the top of every simulation frame in DEBUG builds.
bool APC_CALL verify_fp_mode(const FPUState& expected);

// Restore previous FP state (for interop with non-APC code).
void APC_CALL restore_fp_mode(const FPUState& state);

// Query platform capabilities
struct FPCapabilities {
    bool supports_flush_denormals_to_zero;  // DAZ
    bool supports_denormals_are_zero;       // FTZ
    bool supports_strict_rounding;          // Round-to-nearest-even
    bool has_fma_that_breaks_determinism;   // FMA fused ops
};
FPCapabilities APC_CALL query_fp_capabilities();

} // namespace apc