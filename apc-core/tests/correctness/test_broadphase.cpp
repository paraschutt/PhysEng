// =============================================================================
// test_broadphase.cpp — Sweep-and-Prune Broadphase Correctness Tests
// =============================================================================
// Tests the BroadphaseSAP from apc_collision/apc_broadphase.h.
// Verifies pair generation for various AABB configurations and determinism.
// All tests run under enforce_deterministic_fp_mode() and produce a state hash.
// =============================================================================

#include "apc_collision/apc_broadphase.h"
#include "apc_platform/apc_fp_mode.h"
#include "apc_math/apc_vec3.h"
#include <cstdio>
#include <cstring>
#include <vector>

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
// Helper: compare two pair vectors for exact equality
// ---------------------------------------------------------------------------
static bool pairs_match(const std::vector<BroadphasePair>& a,
                        const std::vector<BroadphasePair>& b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i].id_a != b[i].id_a || a[i].id_b != b[i].id_b) return false;
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
        BroadphaseSAP sap;
        std::vector<BroadphaseSAP::Proxy> proxies;
        proxies.push_back({ 1, { Vec3(-1.0f, -1.0f, -1.0f), Vec3(1.0f, 1.0f, 1.0f) } });
        proxies.push_back({ 2, { Vec3(0.5f, -0.5f, -0.5f), Vec3(1.5f, 0.5f, 0.5f) } });

        sap.update(proxies);
        sap.generate_pairs();

        const auto& pairs = sap.get_potential_pairs();
        bool ok = (pairs.size() == 1)
               && (pairs[0].id_a == 1 && pairs[0].id_b == 2);
        std::printf("[%s] Test 1: Two overlapping AABBs → %zu pair(s) (expect 1)\n",
                     ok ? "PASS" : "FAIL", pairs.size());
        if (ok) ++passed;
        hash_size(state_hash, pairs.size());
    }

    // -----------------------------------------------------------------------
    // Test 2: Two separated AABBs → 0 pairs
    // -----------------------------------------------------------------------
    {
        BroadphaseSAP sap;
        std::vector<BroadphaseSAP::Proxy> proxies;
        proxies.push_back({ 1, { Vec3(-2.0f, -1.0f, -1.0f), Vec3(0.0f, 1.0f, 1.0f) } });
        proxies.push_back({ 2, { Vec3(1.0f, -0.5f, -0.5f),  Vec3(3.0f, 0.5f, 0.5f) } });

        sap.update(proxies);
        sap.generate_pairs();

        const auto& pairs = sap.get_potential_pairs();
        bool ok = (pairs.size() == 0);
        std::printf("[%s] Test 2: Two separated AABBs → %zu pair(s) (expect 0)\n",
                     ok ? "PASS" : "FAIL", pairs.size());
        if (ok) ++passed;
        hash_size(state_hash, pairs.size());
    }

    // -----------------------------------------------------------------------
    // Test 3: Three AABBs, two overlapping → correct pairs
    // -----------------------------------------------------------------------
    //   AABB 1: (-2, -1, -1) to (0, 1, 1)    — does NOT overlap with 2 or 3
    //   AABB 2: (0.5, -0.5, -0.5) to (1.5, 0.5, 0.5) — overlaps with 3
    //   AABB 3: (1.0, -1.0, -1.0) to (2.0, 1.0, 1.0) — overlaps with 2
    {
        BroadphaseSAP sap;
        std::vector<BroadphaseSAP::Proxy> proxies;
        proxies.push_back({ 1, { Vec3(-2.0f, -1.0f, -1.0f), Vec3(0.0f, 1.0f, 1.0f) } });
        proxies.push_back({ 2, { Vec3(0.5f, -0.5f, -0.5f), Vec3(1.5f, 0.5f, 0.5f) } });
        proxies.push_back({ 3, { Vec3(1.0f, -1.0f, -1.0f), Vec3(2.0f, 1.0f, 1.0f) } });

        sap.update(proxies);
        sap.generate_pairs();

        const auto& pairs = sap.get_potential_pairs();
        bool ok = (pairs.size() == 1)
               && (pairs[0].id_a == 2 && pairs[0].id_b == 3);
        std::printf("[%s] Test 3: Three AABBs (1 pair overlapping) → %zu pair(s)\n",
                     ok ? "PASS" : "FAIL", pairs.size());
        if (ok) ++passed;
        hash_size(state_hash, pairs.size());
    }

    // -----------------------------------------------------------------------
    // Test 4: Many overlapping AABBs (20) → correct count
    // -----------------------------------------------------------------------
    // All 20 AABBs are at the origin with half-extent 1.0.  This creates
    // C(20,2) = 190 overlapping pairs.
    {
        BroadphaseSAP sap;
        std::vector<BroadphaseSAP::Proxy> proxies;

        for (uint32_t i = 0; i < 20; ++i) {
            proxies.push_back({
                i + 1,
                { Vec3(-1.0f, -1.0f, -1.0f), Vec3(1.0f, 1.0f, 1.0f) }
            });
        }

        sap.update(proxies);
        sap.generate_pairs();

        const auto& pairs = sap.get_potential_pairs();
        // Expected: C(20, 2) = 190
        bool ok = (pairs.size() == 190);
        std::printf("[%s] Test 4: 20 identical AABBs → %zu pairs (expect 190)\n",
                     ok ? "PASS" : "FAIL", pairs.size());
        if (ok) ++passed;
        hash_size(state_hash, pairs.size());
    }

    // -----------------------------------------------------------------------
    // Test 5: Identical AABBs → correctly reports overlap
    // -----------------------------------------------------------------------
    {
        BroadphaseSAP sap;
        std::vector<BroadphaseSAP::Proxy> proxies;
        proxies.push_back({ 10, { Vec3(0.0f, 0.0f, 0.0f), Vec3(2.0f, 2.0f, 2.0f) } });
        proxies.push_back({ 20, { Vec3(0.0f, 0.0f, 0.0f), Vec3(2.0f, 2.0f, 2.0f) } });

        sap.update(proxies);
        sap.generate_pairs();

        const auto& pairs = sap.get_potential_pairs();
        bool ok = (pairs.size() == 1)
               && (pairs[0].id_a == 10 && pairs[0].id_b == 20);
        std::printf("[%s] Test 5: Identical AABBs → %zu pair(s) (expect 1)\n",
                     ok ? "PASS" : "FAIL", pairs.size());
        if (ok) ++passed;
        hash_size(state_hash, pairs.size());
    }

    // -----------------------------------------------------------------------
    // Test 6: Determinism — run generate_pairs twice → same output
    // -----------------------------------------------------------------------
    {
        BroadphaseSAP sap;
        std::vector<BroadphaseSAP::Proxy> proxies;
        proxies.push_back({ 1, { Vec3(-3.0f, -1.0f, -1.0f), Vec3(1.0f, 1.0f, 1.0f) } });
        proxies.push_back({ 2, { Vec3(0.5f, -0.5f, -0.5f),  Vec3(2.5f, 0.5f, 0.5f) } });
        proxies.push_back({ 3, { Vec3(-1.0f, -2.0f, -2.0f), Vec3(1.0f, 2.0f, 2.0f) } });
        proxies.push_back({ 4, { Vec3(5.0f, -1.0f, -1.0f),  Vec3(7.0f, 1.0f, 1.0f) } });
        proxies.push_back({ 5, { Vec3(-0.5f, -0.5f, -0.5f), Vec3(0.5f, 0.5f, 0.5f) } });

        sap.update(proxies);
        sap.generate_pairs();
        std::vector<BroadphasePair> first_run = sap.get_potential_pairs();

        // Reset and re-run on same data
        sap.update(proxies);
        sap.generate_pairs();
        std::vector<BroadphasePair> second_run = sap.get_potential_pairs();

        bool ok = pairs_match(first_run, second_run);
        std::printf("[%s] Test 6: Determinism — two runs produce identical output (%zu pairs)\n",
                     ok ? "PASS" : "FAIL", first_run.size());
        if (ok) ++passed;

        // Hash pair data for cross-platform comparison
        for (const auto& p : first_run) {
            hash_u32(state_hash, p.id_a);
            hash_u32(state_hash, p.id_b);
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
