#include "apc_collision/apc_broadphase.h"
#include "apc_collision/apc_gjk.h"
#include "apc_platform/apc_fp_mode.h"
#include <cassert>
#include <cstdio>

using namespace apc;

// --- Mock Shapes for GJK ---
struct Sphere { Vec3 center; float radius; };
struct Box { Vec3 center; Vec3 half_extents; };

Vec3 sphere_support(const void* data, const Vec3& dir, uint32_t& out_id) {
    const Sphere* s = static_cast<const Sphere*>(data);
    float len = Vec3::length(dir);
    if (len < 0.0001f) { out_id = 0; return s->center; }
    Vec3 norm_dir = Vec3::scale(dir, 1.0f / len);
    out_id = 0;
    return Vec3::add(s->center, Vec3::scale(norm_dir, s->radius));
}

Vec3 box_support(const void* data, const Vec3& dir, uint32_t& out_id) {
    const Box* b = static_cast<const Box*>(data);
    out_id = 0;
    return Vec3::add(b->center, Vec3(
        (dir.x > 0.0f ? b->half_extents.x : -b->half_extents.x),
        (dir.y > 0.0f ? b->half_extents.y : -b->half_extents.y),
        (dir.z > 0.0f ? b->half_extents.z : -b->half_extents.z)
    ));
}

int main() {
    enforce_deterministic_fp_mode(); // From Week 1

    // Setup: Sphere at origin, radius 1. Box centered at (0.5,0,0), half-extents 0.5.
    // They are overlapping by 0.5 units.
    Sphere sphere_data{Vec3(0.0f, 0.0f, 0.0f), 1.0f};
    Box box_data{Vec3(0.5f, 0.0f, 0.0f), apc::Vec3(0.5f, 0.5f, 0.5f)};

    ConvexHull hull_sphere{ &sphere_data, sphere_support };
    ConvexHull hull_box{ &box_data, box_support };

    // Setup Broadphase — TEST 1: Overlapping pair
    Broadphase bp;
    bp.clear();
    // Sphere AABB (center 0,0,0, radius 1)
    bp.add_aabb(1, { Vec3(-1, -1, -1), Vec3(1, 1, 1) });
    // Box AABB (center 0.5, 0, 0, half-extents 0.5) -> min 0,0,0 max 1,1,1
    bp.add_aabb(2, { Vec3(0, -0.5f, -0.5f), Vec3(1.0f, 0.5f, 0.5f) });
    bp.compute_pairs();

    const BroadphasePair* pairs = bp.get_pairs();
    uint32_t pair_count = bp.get_pair_count();

    // VERIFY BROADPHASE
    assert(pair_count == 1);
    assert(pairs[0].body_a_id == 1 && pairs[0].body_b_id == 2);
    std::printf("[PASS] Broadphase detected 1 overlapping pair (1, 2).\n");

    // VERIFY NARROWPHASE (GJK)
    GJKResult result = GJKBoolean::query(hull_sphere, hull_box);

    assert(result.intersecting == true);
    std::printf("[PASS] GJK correctly identified overlap.\n");

    // TEST 2: Move box far away (center at 10,0,0 → no overlap with sphere at origin)
    bp.clear();
    bp.add_aabb(1, { Vec3(-1, -1, -1), Vec3(1, 1, 1) });
    bp.add_aabb(2, { Vec3(9.5f, -0.5f, -0.5f), Vec3(10.5f, 0.5f, 0.5f) });
    box_data.center = Vec3(10.0f, 0.0f, 0.0f);

    bp.compute_pairs();
    pair_count = bp.get_pair_count();

    assert(pair_count == 0);
    std::printf("[PASS] Broadphase correctly filtered separated pair.\n");

    // TEST 3: High-speed tunneling edge case (Sphere leaps past box)
    // Broadphase shows overlap (due to AABB sweep), GJK must handle it gracefully.
    bp.clear();
    bp.add_aabb(1, { Vec3(-1, -1, -1), Vec3(10, 1, 1) }); // Massive AABB representing a fast sweep
    bp.add_aabb(2, { Vec3(4.5f, -0.5f, -0.5f), Vec3(5.5f, 0.5f, 0.5f) });
    box_data.center = Vec3(5.0f, 0.0f, 0.0f); // Box center at (5,0,0), well beyond sphere at origin

    bp.compute_pairs();
    pair_count = bp.get_pair_count();
    assert(pair_count == 1);

    // Sphere center is at 0,0,0. Box center is at 5,0,0.
    // Even though broadphase says overlap, actual shapes do not touch.
    result = GJKBoolean::query(hull_sphere, hull_box);
    assert(result.intersecting == false);
    std::printf("[PASS] GJK correctly rejected broadphase false-positive (CCD catch).\n");

    std::printf("\n=== ALL FRIDAY REVIEW TESTS PASSED ===\n");
    return 0;
}
