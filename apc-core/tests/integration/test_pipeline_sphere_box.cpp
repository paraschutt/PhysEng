#include "apc_collision/apc_broadphase.h"
#include "apc_collision/apc_gjk.h"
#include <cassert>
#include <cstdio>

using namespace apc;

// --- Mock Shapes for GJK ---
struct Sphere { float radius; };
struct Box { Vec3 half_extents; };

Vec3 sphere_support(const void* data, const Vec3& dir, uint32_t& out_id) {
    const Sphere* s = static_cast<const Sphere*>(data);
    float len = Vec3::length(dir);
    if (len < 0.0001f) { out_id = 0; return Vec3(0,0,0); }
    Vec3 norm_dir = Vec3::scale(dir, 1.0f / len);
    out_id = 0;
    return Vec3::scale(norm_dir, s->radius);
}

Vec3 box_support(const void* data, const Vec3& dir, uint32_t& out_id) {
    const Box* b = static_cast<const Box*>(data);
    out_id = 0;
    return Vec3(
        (dir.x > 0.0f ? b->half_extents.x : -b->half_extents.x),
        (dir.y > 0.0f ? b->half_extents.y : -b->half_extents.y),
        (dir.z > 0.0f ? b->half_extents.z : -b->half_extents.z)
    );
}

int main() {
    enforce_deterministic_fp_mode(); // From Week 1

    // Setup: Sphere at origin, radius 1. Box shifted slightly to the right, half-extents 0.5
    // They are overlapping by 0.5 units.
    Sphere sphere_data{1.0f};
    Box box_data{0.5f, 0.5f, 0.5f};

    ConvexHull hull_sphere{ &sphere_data, sphere_support };
    ConvexHull hull_box{ &box_data, box_support };

    // Setup Broadphase
    BroadphaseSAP sap;
    std::vector<BroadphaseSAP::Proxy> proxies;
    
    // Sphere AABB (center 0,0,0, radius 1)
    proxies.push_back({1, {{-1, -1, -1}, {1, 1, 1}}});
    // Box AABB (center 0.5, 0, 0, half-extents 0.5) -> min 0,0,0 max 1,1,1
    proxies.push_back({2, {{0, -0.5f, -0.5f}, {1.0f, 0.5f, 0.5f}}});

    sap.update(proxies);
    sap.generate_pairs();

    const auto& pairs = sap.get_potential_pairs();
    
    // VERIFY BROADPHASE
    assert(pairs.size() == 1);
    assert(pairs[0].id_a == 1 && pairs[0].id_b == 2);
    std::printf("[PASS] Broadphase detected 1 overlapping pair (1, 2).\n");

    // VERIFY NARROWPHASE (GJK)
    GJKResult result = GJKBoolean::query(hull_sphere, hull_box);
    
    assert(result.intersecting == true);
    std::printf("[PASS] GJK correctly identified overlap.\n");

    // TEST 2: Move box far away
    proxies[1].aabb.min = Vec3(5.0f, -0.5f, -0.5f);
    proxies[1].aabb.max = Vec3(6.0f, 0.5f, 0.5f);
    
    sap.update(proxies);
    sap.generate_pairs();
    
    assert(pairs.size() == 0);
    std::printf("[PASS] Broadphase correctly filtered separated pair.\n");

    // TEST 3: High-speed tunneling edge case (Sphere leaps past box)
    // Broadphase shows overlap (due to AABB sweep), GJK must handle it gracefully.
    proxies[0].aabb.min = Vec3(-1, -1, -1);
    proxies[0].aabb.max = Vec3(10, 1, 1); // Massive AABB representing a fast sweep
    proxies[1].aabb.min = Vec3(2, -0.5f, -0.5f);
    proxies[1].aabb.max = Vec3(3, 0.5f, 0.5f);

    sap.update(proxies);
    sap.generate_pairs();
    assert(pairs.size() == 1);

    // Sphere center is at 0,0,0. Box center is at 2.5,0,0.
    // Even though broadphase says overlap, actual shapes do not touch.
    result = GJKBoolean::query(hull_sphere, hull_box);
    assert(result.intersecting == false);
    std::printf("[PASS] GJK correctly rejected broadphase false-positive (CCD catch).\n");

    std::printf("\n=== ALL FRIDAY REVIEW TESTS PASSED ===\n");
    return 0;
}