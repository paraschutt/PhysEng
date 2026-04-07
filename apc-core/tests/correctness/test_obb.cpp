// =============================================================================
// test_obb.cpp — OBB-OBB SAT (15-axis) Intersection Correctness Tests
// =============================================================================
// Tests the OBB overlap test from apc_collision/apc_obb.h.
// All tests run under enforce_deterministic_fp_mode() and produce a state hash.
// =============================================================================

#include "apc_collision/apc_obb.h"
#include "apc_platform/apc_fp_mode.h"
#include <cstdio>
#include <cmath>
#include <cstring>

using namespace apc;

// ---------------------------------------------------------------------------
// FNV-1a hash for cross-platform determinism verification
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

static void hash_bool(uint64_t& h, bool v) {
    uint8_t b = v ? 1 : 0;
    h ^= (uint64_t)b + 0x9e3779b9 + (h << 6);
}

int main() {
    enforce_deterministic_fp_mode();

    int passed = 0;
    int total  = 5;
    uint64_t state_hash = 0;

    std::printf("=== OBB-OBB Intersection Tests (SAT 15-axis) ===\n\n");

    // -----------------------------------------------------------------------
    // Test 1: Two unit cubes at origin — overlapping → true
    // -----------------------------------------------------------------------
    {
        OBB a, b;
        a.center      = Vec3(0.0f, 0.0f, 0.0f);
        a.extents     = Vec3(0.5f, 0.5f, 0.5f);
        a.orientation = Quat::identity();
        a.update_cache();

        b.center      = Vec3(0.0f, 0.0f, 0.0f);
        b.extents     = Vec3(0.5f, 0.5f, 0.5f);
        b.orientation = Quat::identity();
        b.update_cache();

        bool result = obb_intersect(a, b);
        bool ok     = (result == true);
        std::printf("[%s] Test 1: Two unit cubes at origin — overlapping\n", ok ? "PASS" : "FAIL");
        if (ok) ++passed;
        hash_bool(state_hash, result);
    }

    // -----------------------------------------------------------------------
    // Test 2: Two unit cubes, one shifted far along X — separated → false
    // -----------------------------------------------------------------------
    {
        OBB a, b;
        a.center      = Vec3(0.0f, 0.0f, 0.0f);
        a.extents     = Vec3(0.5f, 0.5f, 0.5f);
        a.orientation = Quat::identity();
        a.update_cache();

        b.center      = Vec3(5.0f, 0.0f, 0.0f);
        b.extents     = Vec3(0.5f, 0.5f, 0.5f);
        b.orientation = Quat::identity();
        b.update_cache();

        bool result = obb_intersect(a, b);
        bool ok     = (result == false);
        std::printf("[%s] Test 2: Separated cubes (X offset = 5.0)\n", ok ? "PASS" : "FAIL");
        if (ok) ++passed;
        hash_bool(state_hash, result);
    }

    // -----------------------------------------------------------------------
    // Test 3: Two rotated OBBs that overlap → true
    // -----------------------------------------------------------------------
    // Cube A rotated 45° around Y at origin, cube B rotated -45° around Y
    // offset slightly along X.  They must still overlap.
    {
        OBB a, b;
        a.center      = Vec3(0.0f, 0.0f, 0.0f);
        a.extents     = Vec3(0.5f, 0.5f, 0.5f);
        a.orientation = Quat::from_axis_angle(Vec3(0.0f, 1.0f, 0.0f), APC_PI * 0.25f);
        a.update_cache();

        b.center      = Vec3(0.3f, 0.0f, 0.0f);
        b.extents     = Vec3(0.5f, 0.5f, 0.5f);
        b.orientation = Quat::from_axis_angle(Vec3(0.0f, 1.0f, 0.0f), -APC_PI * 0.25f);
        b.update_cache();

        bool result = obb_intersect(a, b);
        bool ok     = (result == true);
        std::printf("[%s] Test 3: Rotated OBBs (±45° Y) with X offset — overlap\n", ok ? "PASS" : "FAIL");
        if (ok) ++passed;
        hash_bool(state_hash, result);
    }

    // -----------------------------------------------------------------------
    // Test 4: Edge-edge cross product edge case → check returns correctly
    // -----------------------------------------------------------------------
    // Two thin plate-like OBBs rotated around perpendicular axes so that the
    // 9 cross-product edge axes become the deciding factor.
    {
        OBB a, b;
        a.center      = Vec3(0.0f, 0.0f, 0.0f);
        a.extents     = Vec3(1.0f, 0.1f, 0.1f);   // Long thin bar along X
        a.orientation = Quat::from_axis_angle(Vec3(1.0f, 0.0f, 0.0f), APC_PI * 0.25f);
        a.update_cache();

        b.center      = Vec3(0.0f, 0.25f, 0.0f);
        b.extents     = Vec3(0.1f, 1.0f, 0.1f);   // Long thin bar along Y
        b.orientation = Quat::from_axis_angle(Vec3(0.0f, 0.0f, 1.0f), APC_PI * 0.25f);
        b.update_cache();

        bool result = obb_intersect(a, b);
        // At offset 0.25 with half-extent 0.1 and 45° rotations, the bars
        // should still overlap in 3D.
        bool ok = (result == true);
        std::printf("[%s] Test 4: Edge-edge cross product (thin rotated bars)\n", ok ? "PASS" : "FAIL");
        if (ok) ++passed;
        hash_bool(state_hash, result);
    }

    // -----------------------------------------------------------------------
    // Test 5: Degenerate zero-extent OBB — should not crash
    // -----------------------------------------------------------------------
    {
        OBB a, b;
        a.center      = Vec3(0.0f, 0.0f, 0.0f);
        a.extents     = Vec3(0.0f, 0.0f, 0.0f);    // Degenerate: point at origin
        a.orientation = Quat::identity();
        a.update_cache();

        b.center      = Vec3(0.0f, 0.0f, 0.0f);
        b.extents     = Vec3(0.5f, 0.5f, 0.5f);    // Unit cube
        b.orientation = Quat::identity();
        b.update_cache();

        // A point at the origin lies inside the unit cube, so overlap = true.
        // The critical check is that the function does NOT crash (zero extents
        // mean all projection radii are zero; the SAT test should handle it).
        bool result = obb_intersect(a, b);
        bool ok     = (result == true);
        std::printf("[%s] Test 5: Degenerate zero-extent OBB (no crash)\n", ok ? "PASS" : "FAIL");
        if (ok) ++passed;
        hash_bool(state_hash, result);
    }

    // -----------------------------------------------------------------------
    // Summary & state hash
    // -----------------------------------------------------------------------
    std::printf("\n=== OBB Test Results: %d/%d passed ===\n", passed, total);

    // Produce a determinism hash from the accumulated state
    uint32_t final_hash = fnv1a_bytes(&state_hash, sizeof(state_hash));
    std::printf("State Hash: 0x%08x\n", final_hash);

    return (passed == total) ? 0 : 1;
}
