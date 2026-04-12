// =============================================================================
// test_broadphase.cpp — Sweep-and-Prune Broadphase Correctness Tests
// =============================================================================
// Tests the Broadphase (insertion-sort SAP) from apc_collision/apc_broadphase.h.
// Verifies pair generation for various AABB configurations and determinism.
// All tests run under enforce_deterministic_fp_mode() and produce a state hash.
// =============================================================================

#include "apc_collision/apc_broadphase.h"
#include "apc_platform/apc_fp_mode.h"
#include "apc_math/apc_vec3.h"
#include <cstdio>
#include <cstring>

using namespace apc;

// ---------------------------------------------------------------------------
// Hashing
// ---------------------------------------------------------------------------
static uint32_t fnv1a_bytes(const void* data, size_t len) {
    const uint8_t* p = static_cast<const uint8_t*>(data);
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < len; ++i) {
        h ^= p[i];
        h *= 16777619u;
    }
    return h;
}

static void hash_u32(uint64_t& h, uint32_t v) {
    h ^= (uint64_t)v + 0x9e3779b9 + (h << 6);
}

static void hash_size(uint64_t& h, size_t v) {
    hash_u32(h, static_cast<uint32_t>(v));
}

// ---------------------------------------------------------------------------
// Helper: compare two pair arrays for exact equality
// ---------------------------------------------------------------------------
static bool pairs_match(const BroadphasePair* a, uint32_t a_count,
                        const BroadphasePair* b, uint32_t b_count) {
    if (a_count != b_count) return false;
    for (uint32_t i = 0; i < a_count; ++i) {
        if (a[i].body_a_id != b[i].body_a_id || a[i].body_b_id != b[i].body_b_id)
            return false;
    }
    return true;
}

int main() {
    enforce_deterministic_fp_mode();

    int passed = 0;
    int total  = 6;
    uint64_t state_hash = 0;

    std::printf("=== Broadphase SAP Correctness Tests ===\n\n");

    // -----------------------------------------------------------------------
    // Test 1: Two overlapping AABBs → 1 pair
    // -----------------------------------------------------------------------
    {
        Broadphase bp;
        bp.clear();
        bp.add_aabb(1, { Vec3(-1.0f, -1.0f, -1.0f), Vec3(1.0f, 1.0f, 1.0f) });
        bp.add_aabb(2, { Vec3(0.5f, -0.5f, -0.5f), Vec3(1.5f, 0.5f, 0.5f) });
        bp.compute_pairs();

        const BroadphasePair* pairs = bp.get_pairs();
        uint32_t pc = bp.get_pair_count();
        bool ok = (pc == 1)
               && (pairs[0].body_a_id == 1 && pairs[0].body_b_id == 2);
        std::printf("[%s] Test 1: Two overlapping AABBs → %u pair(s) (expect 1)\n",
                     ok ? "PASS" : "FAIL", pc);
        if (ok) ++passed;
        hash_size(state_hash, pc);
    }

    // -----------------------------------------------------------------------
    // Test 2: Two separated AABBs → 0 pairs
    // -----------------------------------------------------------------------
    {
        Broadphase bp;
        bp.clear();
        bp.add_aabb(1, { Vec3(-2.0f, -1.0f, -1.0f), Vec3(0.0f, 1.0f, 1.0f) });
        bp.add_aabb(2, { Vec3(1.0f, -0.5f, -0.5f),  Vec3(3.0f, 0.5f, 0.5f) });
        bp.compute_pairs();

        uint32_t pc = bp.get_pair_count();
        bool ok = (pc == 0);
        std::printf("[%s] Test 2: Two separated AABBs → %u pair(s) (expect 0)\n",
                     ok ? "PASS" : "FAIL", pc);
        if (ok) ++passed;
        hash_size(state_hash, pc);
    }

    // -----------------------------------------------------------------------
    // Test 3: Three AABBs, two overlapping → correct pairs
    // -----------------------------------------------------------------------
    //   AABB 1: (-2, -1, -1) to (0, 1, 1)    — does NOT overlap with 2 or 3
    //   AABB 2: (0.5, -0.5, -0.5) to (1.5, 0.5, 0.5) — overlaps with 3
    //   AABB 3: (1.0, -1.0, -1.0) to (2.0, 1.0, 1.0) — overlaps with 2
    {
        Broadphase bp;
        bp.clear();
        bp.add_aabb(1, { Vec3(-2.0f, -1.0f, -1.0f), Vec3(0.0f, 1.0f, 1.0f) });
        bp.add_aabb(2, { Vec3(0.5f, -0.5f, -0.5f), Vec3(1.5f, 0.5f, 0.5f) });
        bp.add_aabb(3, { Vec3(1.0f, -1.0f, -1.0f), Vec3(2.0f, 1.0f, 1.0f) });
        bp.compute_pairs();

        const BroadphasePair* pairs = bp.get_pairs();
        uint32_t pc = bp.get_pair_count();
        bool ok = (pc == 1)
               && (pairs[0].body_a_id == 2 && pairs[0].body_b_id == 3);
        std::printf("[%s] Test 3: Three AABBs (1 pair overlapping) → %u pair(s)\n",
                     ok ? "PASS" : "FAIL", pc);
        if (ok) ++passed;
        hash_size(state_hash, pc);
    }

    // -----------------------------------------------------------------------
    // Test 4: Many overlapping AABBs (20) → correct count
    // -----------------------------------------------------------------------
    // All 20 AABBs are at the origin with half-extent 1.0.  This creates
    // C(20,2) = 190 overlapping pairs.
    {
        Broadphase bp;
        bp.clear();

        for (uint32_t i = 0; i < 20; ++i) {
            bp.add_aabb(i + 1,
                { Vec3(-1.0f, -1.0f, -1.0f), Vec3(1.0f, 1.0f, 1.0f) });
        }
        bp.compute_pairs();

        uint32_t pc = bp.get_pair_count();
        // Expected: C(20, 2) = 190
        bool ok = (pc == 190);
        std::printf("[%s] Test 4: 20 identical AABBs → %u pairs (expect 190)\n",
                     ok ? "PASS" : "FAIL", pc);
        if (ok) ++passed;
        hash_size(state_hash, pc);
    }

    // -----------------------------------------------------------------------
    // Test 5: Identical AABBs → correctly reports overlap
    // -----------------------------------------------------------------------
    {
        Broadphase bp;
        bp.clear();
        bp.add_aabb(10, { Vec3(0.0f, 0.0f, 0.0f), Vec3(2.0f, 2.0f, 2.0f) });
        bp.add_aabb(20, { Vec3(0.0f, 0.0f, 0.0f), Vec3(2.0f, 2.0f, 2.0f) });
        bp.compute_pairs();

        const BroadphasePair* pairs = bp.get_pairs();
        uint32_t pc = bp.get_pair_count();
        bool ok = (pc == 1)
               && (pairs[0].body_a_id == 10 && pairs[0].body_b_id == 20);
        std::printf("[%s] Test 5: Identical AABBs → %u pair(s) (expect 1)\n",
                     ok ? "PASS" : "FAIL", pc);
        if (ok) ++passed;
        hash_size(state_hash, pc);
    }

    // -----------------------------------------------------------------------
    // Test 6: Determinism — run compute_pairs twice → same output
    // -----------------------------------------------------------------------
    {
        Broadphase bp;
        bp.clear();
        bp.add_aabb(1, { Vec3(-3.0f, -1.0f, -1.0f), Vec3(1.0f, 1.0f, 1.0f) });
        bp.add_aabb(2, { Vec3(0.5f, -0.5f, -0.5f),  Vec3(2.5f, 0.5f, 0.5f) });
        bp.add_aabb(3, { Vec3(-1.0f, -2.0f, -2.0f), Vec3(1.0f, 2.0f, 2.0f) });
        bp.add_aabb(4, { Vec3(5.0f, -1.0f, -1.0f),  Vec3(7.0f, 1.0f, 1.0f) });
        bp.add_aabb(5, { Vec3(-0.5f, -0.5f, -0.5f), Vec3(0.5f, 0.5f, 0.5f) });
        bp.compute_pairs();

        // Snapshot first run
        uint32_t first_count = bp.get_pair_count();
        BroadphasePair first_pairs[64];
        uint32_t snap = (first_count < 64) ? first_count : 64;
        for (uint32_t i = 0; i < snap; ++i) first_pairs[i] = bp.get_pairs()[i];

        // Reset and re-run on same data
        bp.clear();
        bp.add_aabb(1, { Vec3(-3.0f, -1.0f, -1.0f), Vec3(1.0f, 1.0f, 1.0f) });
        bp.add_aabb(2, { Vec3(0.5f, -0.5f, -0.5f),  Vec3(2.5f, 0.5f, 0.5f) });
        bp.add_aabb(3, { Vec3(-1.0f, -2.0f, -2.0f), Vec3(1.0f, 2.0f, 2.0f) });
        bp.add_aabb(4, { Vec3(5.0f, -1.0f, -1.0f),  Vec3(7.0f, 1.0f, 1.0f) });
        bp.add_aabb(5, { Vec3(-0.5f, -0.5f, -0.5f), Vec3(0.5f, 0.5f, 0.5f) });
        bp.compute_pairs();

        uint32_t second_count = bp.get_pair_count();
        bool ok = pairs_match(first_pairs, first_count,
                              bp.get_pairs(), second_count);
        std::printf("[%s] Test 6: Determinism — two runs produce identical output (%u pairs)\n",
                     ok ? "PASS" : "FAIL", first_count);
        if (ok) ++passed;

        // Hash pair data for cross-platform comparison
        for (uint32_t i = 0; i < first_count; ++i) {
            hash_u32(state_hash, first_pairs[i].body_a_id);
            hash_u32(state_hash, first_pairs[i].body_b_id);
        }
    }

    // -----------------------------------------------------------------------
    // Summary & state hash
    // -----------------------------------------------------------------------
    std::printf("\n=== Broadphase Test Results: %d/%d passed ===\n", passed, total);

    uint32_t final_hash = fnv1a_bytes(&state_hash, sizeof(state_hash));
    std::printf("State Hash: 0x%08x\n", final_hash);

    return (passed == total) ? 0 : 1;
}
