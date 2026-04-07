#include "apc_fp_mode.h"

// Consoles require specific platform SDKs. 
// The build system must define one of these (e.g., -DAPC_PLATFORM_PS5)

#if defined(APC_PLATFORM_PS5)
    // #include <sdk_version.h>
    // PS5 uses x86-64 architecture, but requires specific SDK calls for some HW contexts.
    // For now, fallback to standard x64.
    #warning "PS5 target detected. Using generic x64 FP enforcement. Verify with SDK docs."
    #define APC_CONSOLE_IS_X64 1

#elif defined(APC_PLATFORM_XBOX_SERIES)
    // Xbox Series X|S is x86-64. GameCore OS handles FP state per-thread.
    #warning "Xbox target detected. Using generic x64 FP enforcement."
    #define APC_CONSOLE_IS_X64 1

#elif defined(APC_PLATFORM_SWITCH)
    // Nintendo Switch is ARM64 (A57).
    #warning "Switch target detected. Using generic ARM64 FP enforcement."
    #define APC_CONSOLE_IS_ARM64 1

#else
    #error "Console platform macro (APC_PLATFORM_PS5/XBOX_SERIES/SWITCH) not defined."
#endif

// Delegate to the appropriate generic implementation
#if APC_CONSOLE_IS_X64
    extern apc::FPUState apc_enforce_x64_fp_mode();
    extern bool apc_verify_x64_fp_mode(const apc::FPUState&);
    extern void apc_restore_x64_fp_mode(const apc::FPUState&);
    extern apc::FPCapabilities apc_query_x64_capabilities();
#elif APC_CONSOLE_IS_ARM64
    extern apc::FPUState apc_enforce_arm64_fp_mode();
    extern bool apc_verify_arm64_fp_mode(const apc::FPUState&);
    extern void apc_restore_arm64_fp_mode(const apc::FPUState&);
    extern apc::FPCapabilities apc_query_arm64_capabilities();
#endif

namespace apc {

FPUState enforce_deterministic_fp_mode() {
#if APC_CONSOLE_IS_X64
    return apc_enforce_x64_fp_mode();
#else
    return apc_enforce_arm64_fp_mode();
#endif
}

bool verify_fp_mode(const FPUState& expected) {
#if APC_CONSOLE_IS_X64
    return apc_verify_x64_fp_mode(expected);
#else
    return apc_verify_arm64_fp_mode(expected);
#endif
}

void restore_fp_mode(const FPUState& state) {
#if APC_CONSOLE_IS_X64
    apc_restore_x64_fp_mode(state);
#else
    apc_restore_arm64_fp_mode(state);
#endif
}

FPCapabilities query_fp_capabilities() {
#if APC_CONSOLE_IS_X64
    return apc_query_x64_capabilities();
#else
    return apc_query_arm64_capabilities();
#endif
}

} // namespace apc