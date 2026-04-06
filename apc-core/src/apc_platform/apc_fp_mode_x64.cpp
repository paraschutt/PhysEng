#include "apc_fp_mode.h"
#include <cstdint>
#include <immintrin.h>

#ifdef APC_PLATFORM_X64

namespace apc {

FPUState enforce_deterministic_fp_mode() {
    FPUState result = {};
    
    // --- SSE Control Word ---
    // Bits we SET:
    //   bit 15 (FZ): Flush-to-zero (denormals → zero)
    //   bit  6 (DAZ): Denormals-are-zero on load
    // Bits we CLEAR:
    //   bits 13-14 (RC): Round-to-nearest-even (00)
    //   bit 12 (PM): Precision mask (we WANT exceptions in debug)
    
    unsigned int sse_cw = _mm_getcsr();
    
    // Enable flush-to-zero and denormals-are-zero
    sse_cw |= (1u << 15);  // FZ
    sse_cw |= (1u << 6);   // DAZ
    
    // Force round-to-nearest-even
    sse_cw &= ~(3u << 13); // Clear RC bits
    
    // In DEBUG, unmask precision exception to catch denormal issues
#ifndef NDEBUG
    sse_cw &= ~(1u << 12); // Clear PM mask
#endif
    
    _mm_setcsr(sse_cw);
    result.raw_state = ((uint64_t)sse_cw << 32);
    
    // --- x87 Control Word (legacy, but still exists) ---
    // Force 64-bit extended precision, round-to-nearest
    unsigned short x87_cw;
    __asm__ __volatile__("fnstcw %0" : "=m"(x87_cw));
    
    x87_cw &= ~(3u << 10);  // RC = round-to-nearest
    x87_cw &= ~(3u << 8);   // PC = 64-bit extended (doesn't matter if we only use SSE)
    x87_cw |= (3u << 8);    // Actually, double precision is fine, we only use SSE
    
    __asm__ __volatile__("fldcw %0" : : "m"(x87_cw));
    result.raw_state |= x87_cw;
    
    result.is_valid = true;
    
    // VERIFY we got what we asked for
    unsigned int verify_sse = _mm_getcsr();
    if ((verify_sse & 0xC060) != (sse_cw & 0xC060)) {
        // Failed to set FZ+DAZ - CRASH
        __builtin_trap();
    }
    
    return result;
}

bool verify_fp_mode(const FPUState& expected) {
    unsigned int current = _mm_getcsr();
    unsigned int expected_sse = static_cast<unsigned int>(expected.raw_state >> 32);
    
    // Only check the bits we care about
    const unsigned int mask = (1u << 15) | (1u << 6) | (3u << 13);
    return (current & mask) == (expected_sse & mask);
}

void restore_fp_mode(const FPUState& state) {
    unsigned int sse_cw = static_cast<unsigned int>(state.raw_state >> 32);
    _mm_setcsr(sse_cw);
    
    unsigned short x87_cw = static_cast<unsigned short>(state.raw_state & 0xFFFF);
    __asm__ __volatile__("fldcw %0" : : "m"(x87_cw));
}

FPCapabilities query_fp_capabilities() {
    FPCapabilities caps = {};
    
    // x64 always supports FTZ/DAZ via SSE
    caps.supports_flush_denormals_to_zero = true;
    caps.supports_denormals_are_zero = true;
    caps.supports_strict_rounding = true;
    
    // FMA: Check CPUID for FMA support
    // If FMA is available, we must NOT use -mfma or it will fuse multiplies
    uint32_t eax, ebx, ecx, edx;
    __cpuid(1, eax, ebx, ecx, edx);
    caps.has_fma_that_breaks_determinism = (ecx & (1 << 12)) != 0;
    
    return caps;
}

} // namespace apc

#endif // APC_PLATFORM_X64