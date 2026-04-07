#include "apc_fp_mode.h"

#if APC_PLATFORM_ARM64

namespace apc {

FPUState enforce_deterministic_fp_mode() {
    FPUState result = {};
    uint64_t fpcr;

#if defined(_MSC_VER)
    // MSVC ARM64: Use intrinsics for system registers
    // 0x5A28 is the ARM64 FPCR register ID
    fpcr = _ReadStatusReg(0x5A28);
#else
    __asm__ __volatile__("mrs %0, fpcr" : "=r"(fpcr));
#endif

    fpcr |= (1ULL << 24); // FZ
    fpcr |= (1ULL << 9);  // DN
    fpcr &= ~(3ULL << 22);// RMode

#if defined(_MSC_VER)
    _WriteStatusReg(0x5A28, fpcr);
#else
    __asm__ __volatile__("msr fpcr, %0" : : "r"(fpcr));
#endif
    
    result.raw_state = fpcr;
    result.is_valid = true;
    return result;
}

bool verify_fp_mode(const FPUState& expected) {
    uint64_t current;
#if defined(_MSC_VER)
    current = _ReadStatusReg(0x5A28);
#else
    __asm__ __volatile__("mrs %0, fpcr" : "=r"(current));
#endif
    
    const uint64_t mask = (1ULL << 24) | (1ULL << 9) | (3ULL << 22);
    return (current & mask) == (expected.raw_state & mask);
}

void restore_fp_mode(const FPUState& state) {
#if defined(_MSC_VER)
    _WriteStatusReg(0x5A28, state.raw_state);
#else
    __asm__ __volatile__("msr fpcr, %0" : : "r"(state.raw_state));
#endif
}

FPCapabilities query_fp_capabilities() {
    FPCapabilities caps = {};
    caps.supports_flush_denormals_to_zero = true;
    caps.supports_denormals_are_zero = true;
    caps.supports_strict_rounding = true;
    caps.has_fma_that_breaks_determinism = false; 
    return caps;
}

} // namespace apc
#endif // APC_PLATFORM_ARM64