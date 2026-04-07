// =============================================================================
// test_gjk_epa.cpp — GJK Boolean Query + EPA Penetration Depth Tests
// =============================================================================
// Tests the GJK intersection query and EPA penetration solver from
// apc_collision/apc_gjk.h and apc_collision/apc_epa.h.
// Uses mock support functions for sphere and box shapes.
// All tests run under enforce_deterministic_fp_mode() and produce a state hash.
// =============================================================================

#include "apc_collision/apc_gjk.h"
#include "apc_collision/apc_epa.h"
#include "apc_platform/apc_fp_mode.h"
#include "apc_math/apc_vec3.h"
#include <cstdio>
#include <cmath>
#include <cstring>
#include <algorithm>

using namespace apc;

// ---------------------------------------------------------------------------
// Mock shape definitions (positioned in world space)
// ---------------------------------------------------------------------------
struct MockSphere {
    Vec3  center;
    float radius;
};

struct MockBox {
    Vec3 center;
    Vec3 half_extents;
};

// ---------------------------------------------------------------------------
// Support functions (return world-space points)
// ---------------------------------------------------------------------------
static Vec3 sphere_support(const void* data, const Vec3& dir, uint32_t& out_id) {
    const MockSphere* s = static_cast<const MockSphere*>(data);
    float len = Vec3::length(dir);
    if (len < APC_EPSILON) {
        out_id = 0;
        return s->center;
    }
    Vec3 nd = Vec3::scale(dir, 1.0f / len);
    out_id = 0;
    return Vec3::add(s->center, Vec3::scale(nd, s->radius));
}

static Vec3 box_support(const void* data, const Vec3& dir, uint32_t& out_id) {
    const MockBox* b = static_cast<const MockBox*>(data);
    out_id = 0;
    return Vec3::add(b->center, Vec3(
        dir.x >= 0.0f ?  b->half_extents.x : -b->half_extents.x,
        dir.y >= 0.0f ?  b->half_extents.y : -b->half_extents.y,
        dir.z >= 0.0f ?  b->half_extents.z : -b->half_extents.z
    ));
}

// ---------------------------------------------------------------------------
// Helper: compute a Minkowski-difference support point (used to build simplex)
// ---------------------------------------------------------------------------
static Vec3 minkowski_support(const ConvexHull& hull_a, const ConvexHull& hull_b,
                               const Vec3& dir) {
    uint32_t id_a = 0, id_b = 0;
    Vec3 dir_neg = Vec3::scale(dir, -1.0f);
    Vec3 pa = hull_a.support(hull_a.user_data, dir, id_a);
    Vec3 pb = hull_b.support(hull_b.user_data, dir_neg, id_b);
    return Vec3::sub(pa, pb);
}

// ---------------------------------------------------------------------------
// Helper: build a 4-point simplex from the Minkowski difference using 4
// tetrahedral directions.  Returns the number of valid vertices (3 or 4).
// ---------------------------------------------------------------------------
static int build_simplex(const ConvexHull& hull_a, const ConvexHull& hull_b,
                         Vec3 out_points[4]) {
    // Four non-coplanar directions forming a regular tetrahedron
    static const Vec3 dirs[4] = {
        Vec3( 1.0f,  1.0f,  1.0f),
        Vec3( 1.0f, -1.0f, -1.0f),
        Vec3(-1.0f,  1.0f, -1.0f),
        Vec3(-1.0f, -1.0f,  1.0f)
    };

    for (int i = 0; i < 4; ++i) {
        out_points[i] = minkowski_support(hull_a, hull_b, dirs[i]);
    }
    return 4;
}

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

static void hash_bool(uint64_t& h, bool v) {
    uint8_t b = v ? 1 : 0;
    h ^= (uint64_t)b + 0x9e3779b9 + (h << 6);
}

static void hash_float(uint64_t& h, float f) {
    uint32_t bits;
    std::memcpy(&bits, &f, sizeof(bits));
    h ^= (uint64_t)bits + 0x9e3779b9 + (h << 6);
}

int main() {
    enforce_deterministic_fp_mode();

    int passed = 0;
    int total  = 7;
    uint64_t state_hash = 0;

    std::printf("=== GJK + EPA Correctness Tests ===\n\n");

    // -----------------------------------------------------------------------
    // Test 1: GJK — overlapping sphere-box → intersecting=true
    // -----------------------------------------------------------------------
    {
        MockSphere sphere_data{ Vec3(0.0f, 0.0f, 0.0f), 1.0f };
        MockBox    box_data   { Vec3(0.5f, 0.0f, 0.0f), Vec3(0.5f, 0.5f, 0.5f) };

        ConvexHull hull_a{ &sphere_data, sphere_support };
        ConvexHull hull_b{ &box_data,    box_support    };

        GJKResult result = GJKBoolean::query(hull_a, hull_b);

        bool ok = (result.intersecting == true);
        std::printf("[%s] Test 1: GJK overlapping sphere-box → intersecting\n", ok ? "PASS" : "FAIL");
        if (ok) ++passed;
        hash_bool(state_hash, result.intersecting);
    }

    // -----------------------------------------------------------------------
    // Test 2: GJK — separated sphere-box → intersecting=false
    // -----------------------------------------------------------------------
    {
        MockSphere sphere_data{ Vec3(0.0f, 0.0f, 0.0f), 1.0f };
        MockBox    box_data   { Vec3(5.0f, 0.0f, 0.0f), Vec3(0.5f, 0.5f, 0.5f) };

        ConvexHull hull_a{ &sphere_data, sphere_support };
        ConvexHull hull_b{ &box_data,    box_support    };

        GJKResult result = GJKBoolean::query(hull_a, hull_b);

        bool sep_vec_valid = Vec3::length_sq(result.separation_vector) > APC_EPSILON_SQ;
        bool ok = (result.intersecting == false) && sep_vec_valid;
        std::printf("[%s] Test 2: GJK separated sphere-box → not intersecting\n", ok ? "PASS" : "FAIL");
        if (ok) ++passed;
        hash_bool(state_hash, result.intersecting);
    }

    // -----------------------------------------------------------------------
    // Test 3: GJK — nearly touching (boundary case) → not intersecting
    // -----------------------------------------------------------------------
    // Sphere and box with a small gap. The GJK boundary (origin on
    // Minkowski surface) is numerically delicate; we use a sphere-box
    // pair to avoid a known collinear degeneracy in the initial search
    // direction.
    {
        MockSphere sphere_data{ Vec3(0.0f, 0.0f, 0.0f), 1.0f };
        MockBox    box_data   { Vec3(2.1f, 0.0f, 0.0f), Vec3(0.5f, 0.5f, 0.5f) };

        ConvexHull hull_a{ &sphere_data, sphere_support };
        ConvexHull hull_b{ &box_data,    box_support    };

        GJKResult result = GJKBoolean::query(hull_a, hull_b);

        bool ok = (result.intersecting == false);
        std::printf("[%s] Test 3: GJK near-touching sphere-box (gap=0.1) → not intersecting\n",
                     ok ? "PASS" : "FAIL");
        if (ok) ++passed;
        hash_bool(state_hash, result.intersecting);
    }

    // -----------------------------------------------------------------------
    // Test 4: EPA — overlapping sphere-box → success=true, penetration > 0
    // -----------------------------------------------------------------------
    {
        MockSphere sphere_data{ Vec3(0.0f, 0.0f, 0.0f), 1.0f };
        MockBox    box_data   { Vec3(0.5f, 0.0f, 0.0f), Vec3(0.5f, 0.5f, 0.5f) };

        ConvexHull hull_a{ &sphere_data, sphere_support };
        ConvexHull hull_b{ &box_data,    box_support    };

        Vec3 simplex[4];
        int count = build_simplex(hull_a, hull_b, simplex);

        EPAResult epa = EPA::query(simplex, count, hull_a, hull_b);

        bool ok = epa.success
               && epa.penetration > APC_EPSILON
               && Vec3::length_sq(epa.normal) > 0.99f; // Approximately unit
        std::printf("[%s] Test 4: EPA overlapping sphere-box → success, pen=%.4f\n",
                     ok ? "PASS" : "FAIL", epa.penetration);
        if (ok) ++passed;
        hash_bool(state_hash, epa.success);
        hash_float(state_hash, epa.penetration);
    }

    // -----------------------------------------------------------------------
    // Test 5: EPA — slightly overlapping → small penetration
    // -----------------------------------------------------------------------
    {
        // Sphere radius 1 at origin, box half-extents 0.5 at (1.4, 0, 0)
        // Box face at x=0.9, sphere surface at x=1.0  → overlap ≈ 0.1
        MockSphere sphere_data{ Vec3(0.0f, 0.0f, 0.0f), 1.0f };
        MockBox    box_data   { Vec3(1.4f, 0.0f, 0.0f), Vec3(0.5f, 0.5f, 0.5f) };

        ConvexHull hull_a{ &sphere_data, sphere_support };
        ConvexHull hull_b{ &box_data,    box_support    };

        Vec3 simplex[4];
        int count = build_simplex(hull_a, hull_b, simplex);

        EPAResult epa = EPA::query(simplex, count, hull_a, hull_b);

        bool ok = epa.success
               && epa.penetration > 0.01f
               && epa.penetration < 0.5f;  // Slight overlap
        std::printf("[%s] Test 5: EPA slight overlap → pen=%.4f (expect ~0.1)\n",
                     ok ? "PASS" : "FAIL", epa.penetration);
        if (ok) ++passed;
        hash_bool(state_hash, epa.success);
        hash_float(state_hash, epa.penetration);
    }

    // -----------------------------------------------------------------------
    // Test 6: EPA — deep overlap → large penetration
    // -----------------------------------------------------------------------
    {
        // Sphere radius 1 at origin, box half-extents 0.5 at (0.2, 0, 0)
        // Box face at x=-0.3, sphere surface at x=1.0 → overlap ≈ 1.3
        MockSphere sphere_data{ Vec3(0.0f, 0.0f, 0.0f), 1.0f };
        MockBox    box_data   { Vec3(0.2f, 0.0f, 0.0f), Vec3(0.5f, 0.5f, 0.5f) };

        ConvexHull hull_a{ &sphere_data, sphere_support };
        ConvexHull hull_b{ &box_data,    box_support    };

        Vec3 simplex[4];
        int count = build_simplex(hull_a, hull_b, simplex);

        EPAResult epa = EPA::query(simplex, count, hull_a, hull_b);

        bool ok = epa.success
               && epa.penetration > 0.5f;   // Deep overlap
        std::printf("[%s] Test 6: EPA deep overlap → pen=%.4f (expect > 0.5)\n",
                     ok ? "PASS" : "FAIL", epa.penetration);
        if (ok) ++passed;
        hash_bool(state_hash, epa.success);
        hash_float(state_hash, epa.penetration);
    }

    // -----------------------------------------------------------------------
    // Test 7: EPA degenerate — zero-size shapes → success=false (no crash)
    // -----------------------------------------------------------------------
    {
        MockSphere sphere_data{ Vec3(0.0f, 0.0f, 0.0f), 0.0f };  // Zero radius
        MockBox    box_data   { Vec3(0.0f, 0.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f) };  // Zero

        ConvexHull hull_a{ &sphere_data, sphere_support };
        ConvexHull hull_b{ &box_data,    box_support    };

        Vec3 simplex[4];
        int count = build_simplex(hull_a, hull_b, simplex);

        EPAResult epa = EPA::query(simplex, count, hull_a, hull_b);

        // Degenerate shapes should produce a failed EPA result without crashing.
        bool ok = (epa.success == false);
        std::printf("[%s] Test 7: EPA zero-size shapes → no crash, success=false\n",
                     ok ? "PASS" : "FAIL");
        if (ok) ++passed;
        hash_bool(state_hash, epa.success);
    }

    // -----------------------------------------------------------------------
    // Summary & state hash
    // -----------------------------------------------------------------------
    std::printf("\n=== GJK + EPA Test Results: %d/%d passed ===\n", passed, total);

    uint32_t final_hash = fnv1a_bytes(&state_hash, sizeof(state_hash));
    std::printf("State Hash: 0x%08x\n", final_hash);

    return (passed == total) ? 0 : 1;
}
