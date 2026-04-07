#include "apc_fp_mode.h"

#if APC_PLATFORM_X64

#if defined(_MSC_VER)
    #include <intrin.h>
    #include <float.h>  // ADDED: Required for _controlfp_s, _MCW_DN, _RC_NEAR
#else
    #include <immintrin.h>
    #include <cpuid.h>  // GCC/Clang: __cpuid intrinsic
#endif

namespace apc {

FPUState enforce_deterministic_fp_mode() {
    FPUState result = {};
    
#if defined(_MSC_VER)
    // MSVC x64: No inline assembly allowed. Use _controlfp_s.
    unsigned int current_cw = 0;
    _controlfp_s(&current_cw, 0, 0); // Get current state

    // _MCW_DN: Denormal control (FTZ + DAZ)
    // _DN_FLUSH: Flush denormals to zero
    // _MCW_RC: Rounding control
    // _RC_NEAR: Round to nearest
    unsigned int new_cw = _DN_FLUSH | _RC_NEAR;
    _controlfp_s(&current_cw, new_cw, _MCW_DN | _MCW_RC);

    // x87 is effectively unused in 64-bit Windows, so we don't touch it.
    result.raw_state = static_cast<uint64_t>(current_cw) << 32;
#else
    // GCC/Clang x64: SSE control (x87 is effectively unused on x86-64;
    // all float/double math uses SSE2. Modifying x87 CW can cause SIGFPE
    // on some platforms, so we leave it alone.)
    unsigned int sse_cw = _mm_getcsr();
    sse_cw |= (1u << 15); // FZ (flush denormals to zero)
    sse_cw |= (1u << 6);  // DAZ (denormals are zero)
    sse_cw &= ~(3u << 13); // RC (round to nearest even)
    // Note: we intentionally keep PM bit set (precision exceptions masked).
    // Clearing PM would cause SIGFPE on denormals/overflow in unoptimized builds.
    // Determinism is ensured by FZ/DAZ/RC, not by exception signaling.
    
    _mm_setcsr(sse_cw);
    result.raw_state = (static_cast<uint64_t>(sse_cw) << 32);

    // x87: read but do not modify on x86-64 (not used for scalar math)
    unsigned short x87_cw = 0;
    __asm__ __volatile__("fnstcw %0" : "=m"(x87_cw));
    result.raw_state |= static_cast<uint64_t>(x87_cw);
#endif

    result.is_valid = true;

    // Verification
#if defined(_MSC_VER)
    unsigned int verify_cw = 0;
    _controlfp_s(&verify_cw, 0, 0);
    if ((verify_cw & _MCW_DN) != _DN_FLUSH || (verify_cw & _MCW_RC) != _RC_NEAR) {
        __debugbreak(); // MSVC trap
    }
#else
    unsigned int verify_sse = _mm_getcsr();
    unsigned int expected_sse = static_cast<unsigned int>(result.raw_state >> 32);
    const unsigned int mask = (1u << 15) | (1u << 6) | (3u << 13);
    if ((verify_sse & mask) != (expected_sse & mask)) {
        __builtin_trap(); // GCC/Clang trap
    }
#endif
    
    return result;
}

bool verify_fp_mode(const FPUState& expected) {
#if defined(_MSC_VER)
    unsigned int current = 0;
    _controlfp_s(&current, 0, 0);
    unsigned int expected_cw = static_cast<unsigned int>(expected.raw_state >> 32);
    
    // Create a mask for the bits we care about
    unsigned int mask = _MCW_DN | _MCW_RC;
    return (current & mask) == (expected_cw & mask);
#else
    unsigned int current = _mm_getcsr();
    unsigned int expected_sse = static_cast<unsigned int>(expected.raw_state >> 32);
    const unsigned int mask = (1u << 15) | (1u << 6) | (3u << 13);
    return (current & mask) == (expected_sse & mask);
#endif
}

void restore_fp_mode(const FPUState& state) {
#if defined(_MSC_VER)
    unsigned int target_cw = static_cast<unsigned int>(state.raw_state >> 32);
    _controlfp_s(nullptr, target_cw, _MCW_DN | _MCW_RC);
#else
    unsigned int sse_cw = static_cast<unsigned int>(state.raw_state >> 32);
    _mm_setcsr(sse_cw);
    // x87: restore only if the saved state differs from current
    unsigned short current_x87 = 0;
    __asm__ __volatile__("fnstcw %0" : "=m"(current_x87));
    unsigned short saved_x87 = static_cast<unsigned short>(state.raw_state & 0xFFFF);
    if (current_x87 != saved_x87) {
        __asm__ __volatile__("fldcw %0" : : "m"(saved_x87));
    }
#endif
}

FPCapabilities query_fp_capabilities() {
    FPCapabilities caps = {};
    caps.supports_flush_denormals_to_zero = true;
    caps.supports_denormals_are_zero = true;
    caps.supports_strict_rounding = true;
    
#if defined(_MSC_VER)
    // MSVC __cpuid takes an array
    int cpu_info[4];
    __cpuid(cpu_info, 1);
    caps.has_fma_that_breaks_determinism = (cpu_info[2] & (1 << 12)) != 0;
#else
    uint32_t eax = 0, ebx = 0, ecx = 0, edx = 0;
    __get_cpuid(1, &eax, &ebx, &ecx, &edx);
    caps.has_fma_that_breaks_determinism = (ecx & (1 << 12)) != 0;
#endif
    
    return caps;
}

} // namespace apc
#endif // APC_PLATFORM_X64