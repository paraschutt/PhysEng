#include "apc_platform/apc_fp_mode.h"
#include <cstdio>
#include <cstdint>

// External test functions defined in the other det_*.cpp files
extern uint64_t run_det_vec3_ops();
extern uint64_t run_det_quat_ops();
extern uint64_t run_det_mat3_ops();

// Combine hashes (similar to boost::hash_combine)
inline uint64_t hash_combine(uint64_t seed, uint64_t value) {
    seed ^= value + 0x9e3779b97f4a7c15ULL + (seed << 12) + (seed >> 4);
    return seed;
}

int main() {
    // 1. Enforce FP mode BEFORE any math happens
    apc::FPUState fp_state = apc::enforce_deterministic_fp_mode();
    if (!fp_state.is_valid) {
        std::fprintf(stderr, "FATAL: Could not enforce deterministic FP mode\n");
        return 1;
    }

    // 2. Run tests and aggregate hashes
    uint64_t final_hash = 0xcbf29ce484222325ULL; // FNV-1a offset basis

    std::fprintf(stdout, "Running Vec3 determinism tests...\n");
    final_hash = hash_combine(final_hash, run_det_vec3_ops());

    std::fprintf(stdout, "Running Quat determinism tests...\n");
    final_hash = hash_combine(final_hash, run_det_quat_ops());

    std::fprintf(stdout, "Running Mat3 determinism tests...\n");
    final_hash = hash_combine(final_hash, run_det_mat3_ops());

    // 3. Output standardized format for hash_compare.py
    std::fprintf(stdout, "\n=== DETERMINISM RESULT ===\n");
    std::fprintf(stdout, "Final hash: %016llx\n", (unsigned long long)final_hash);
    
    // Optional metadata for debugging
#if defined(_M_X64) || defined(__x86_64__)
    std::fprintf(stdout, "Platform: x64\n");
#elif defined(_M_ARM64) || defined(__aarch64__)
    std::fprintf(stdout, "Platform: ARM64\n");
#endif
    
#if defined(_MSC_VER)
    std::fprintf(stdout, "Compiler: MSVC\n");
#elif defined(__clang__)
    std::fprintf(stdout, "Compiler: Clang\n");
#elif defined(__GNUC__)
    std::fprintf(stdout, "Compiler: GCC\n");
#endif
    std::fprintf(stdout, "==========================\n");

    return 0;
}