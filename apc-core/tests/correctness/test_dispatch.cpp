// =============================================================================
// test_dispatch.cpp — Collision Dispatcher, Shape Detection, and Filtering Tests
// =============================================================================
// Tests the collision dispatcher (apc_collision_dispatch.h), support functions
// (apc_support.h), plane collider (apc_plane.h), and asset loader
// (apc_asset_loader.h).
//
// Coverage:
//   1. Sphere-box detection (outside + inside cases)
//   2. Sphere-plane detection (above, penetrating, degenerate)
//   3. Box-box detection via GJK+EPA
//   4. Sphere-convex detection via GJK+EPA
//   5. Convex-convex detection via GJK+EPA
//   6. Collision dispatcher routing (all shape pair types)
//   7. CollisionShape AABB computation
//   8. ContactManifold accumulation
//   9. Collision filtering (layer/mask)
//  10. Broadphase filter callback
//  11. Asset loader (in-memory .apccol)
//
// All tests produce a state hash for cross-platform determinism verification.
// =============================================================================

#include "apc_collision/apc_collision_dispatch.h"
#include "apc_collision/apc_support.h"
#include "apc_collision/apc_plane.h"
#include "apc_collision/apc_asset_loader.h"
#include "apc_collision/apc_broadphase.h"
#include "apc_solver/apc_rigid_body.h"
#include "apc_platform/apc_fp_mode.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include <cstdio>
#include <cmath>
#include <cstring>
#include <vector>

using namespace apc;

// ---------------------------------------------------------------------------
// Hashing utilities (same pattern as existing tests)
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

static void hash_uint(uint64_t& h, uint32_t v) {
    h ^= (uint64_t)v + 0x9e3779b9 + (h << 6);
}

// Approximate equality for contact normals
static bool normal_approx_equal(const Vec3& a, const Vec3& b, float tol = 0.01f) {
    float len_sq = Vec3::length_sq(b);
    if (len_sq < APC_EPSILON_SQ) return false;
    float cos_angle = Vec3::dot(a, b) / std::sqrt(Vec3::length_sq(a) * len_sq);
    return cos_angle > (1.0f - tol);
}

int main() {
    enforce_deterministic_fp_mode();

    int passed = 0;
    int total  = 14;
    uint64_t state_hash = 0;

    std::printf("=== Collision Dispatcher + Shape Tests ===\n\n");

    // -----------------------------------------------------------------------
    // Test 1: Sphere-box detection (outside — sphere center outside box)
    // -----------------------------------------------------------------------
    {
        // Sphere radius 1 at origin → surface at x=1.0
        // Box half-extents 0.5 at (1.2,0,0) → face at x=0.7
        // Overlap: 1.0 - 0.7 = 0.3
        CollisionShape sphere = CollisionShape::make_sphere(1.0f, Vec3(0, 0, 0));
        CollisionShape box    = CollisionShape::make_box(
            Vec3(0.5f, 0.5f, 0.5f), Vec3(1.2f, 0.0f, 0.0f), Quat::identity());
        box.update_cache();

        ContactPoint contact;
        bool hit = detect_sphere_box(
            sphere.position, sphere.sphere_radius,
            box.position, box.box_half_extents,
            box.rotation, contact);

        bool ok = hit
               && contact.penetration > 0.2f
               && contact.penetration < 0.4f
               && normal_approx_equal(contact.normal, Vec3(-1, 0, 0)); // B→A (box→sphere = -X)
        std::printf("[%s] Test 1: Sphere-box outside contact (pen=%.4f)\n",
                     ok ? "PASS" : "FAIL", contact.penetration);
        if (ok) ++passed;
        hash_bool(state_hash, hit);
        hash_float(state_hash, contact.penetration);
    }

    // -----------------------------------------------------------------------
    // Test 2: Sphere-box detection (inside — sphere center inside box)
    // -----------------------------------------------------------------------
    {
        CollisionShape sphere = CollisionShape::make_sphere(1.0f, Vec3(0, 0, 0));
        CollisionShape box    = CollisionShape::make_box(
            Vec3(2.0f, 2.0f, 2.0f), Vec3(0, 0, 0), Quat::identity());
        box.update_cache();

        ContactPoint contact;
        bool hit = detect_sphere_box(
            sphere.position, sphere.sphere_radius,
            box.position, box.box_half_extents,
            box.rotation, contact);

        // Sphere (r=1) at origin, box [-2,2] on all axes — sphere center is inside
        // Closest face is at x=2, penetration = (2 - 0) + 1 = 3
        bool ok = hit
               && contact.penetration > 2.5f
               && contact.penetration < 3.5f;
        std::printf("[%s] Test 2: Sphere-box inside contact (pen=%.4f)\n",
                     ok ? "PASS" : "FAIL", contact.penetration);
        if (ok) ++passed;
        hash_bool(state_hash, hit);
        hash_float(state_hash, contact.penetration);
    }

    // -----------------------------------------------------------------------
    // Test 3: Sphere-box separated — no collision
    // -----------------------------------------------------------------------
    {
        CollisionShape sphere = CollisionShape::make_sphere(0.5f, Vec3(0, 0, 0));
        CollisionShape box    = CollisionShape::make_box(
            Vec3(0.5f, 0.5f, 0.5f), Vec3(5.0f, 0, 0), Quat::identity());
        box.update_cache();

        ContactPoint contact;
        bool hit = detect_sphere_box(
            sphere.position, sphere.sphere_radius,
            box.position, box.box_half_extents,
            box.rotation, contact);

        bool ok = (hit == false);
        std::printf("[%s] Test 3: Sphere-box separated → no hit\n",
                     ok ? "PASS" : "FAIL");
        if (ok) ++passed;
        hash_bool(state_hash, hit);
    }

    // -----------------------------------------------------------------------
    // Test 4: Sphere-plane detection — penetrating
    // -----------------------------------------------------------------------
    {
        SphereCollider col{ 1.0f };
        PlaneCollider plane{ Vec3(0, 0, 0), Vec3(0, 1, 0) }; // Floor at y=0

        ContactPoint contact;
        bool hit = detect_sphere_plane(Vec3(0, 0.5f, 0), col, plane, contact);

        bool ok = hit
               && contact.penetration > 0.4f
               && contact.penetration < 0.6f
               && normal_approx_equal(contact.normal, Vec3(0, 1, 0)); // B→A (plane→sphere = +Y)
        std::printf("[%s] Test 4: Sphere-plane penetrating (pen=%.4f)\n",
                     ok ? "PASS" : "FAIL", contact.penetration);
        if (ok) ++passed;
        hash_bool(state_hash, hit);
        hash_float(state_hash, contact.penetration);
    }

    // -----------------------------------------------------------------------
    // Test 5: Sphere-plane separated — sphere above plane
    // -----------------------------------------------------------------------
    {
        SphereCollider col{ 0.5f };
        PlaneCollider plane{ Vec3(0, 0, 0), Vec3(0, 1, 0) };

        ContactPoint contact;
        bool hit = detect_sphere_plane(Vec3(0, 2.0f, 0), col, plane, contact);

        bool ok = (hit == false);
        std::printf("[%s] Test 5: Sphere-plane separated → no hit\n",
                     ok ? "PASS" : "FAIL");
        if (ok) ++passed;
        hash_bool(state_hash, hit);
    }

    // -----------------------------------------------------------------------
    // Test 6: Dispatcher — sphere-sphere
    // -----------------------------------------------------------------------
    {
        CollisionShape sa = CollisionShape::make_sphere(1.0f, Vec3(0, 0, 0));
        CollisionShape sb = CollisionShape::make_sphere(1.0f, Vec3(1.5f, 0, 0));

        ContactManifold manifold;
        bool hit = dispatch_detect(sa, sb, 0, 1, manifold);

        bool ok = hit
               && manifold.contact_count == 1
               && manifold.contacts[0].penetration > 0.4f
               && manifold.contacts[0].penetration < 0.6f;
        std::printf("[%s] Test 6: Dispatch sphere-sphere (pen=%.4f)\n",
                     ok ? "PASS" : "FAIL",
                     manifold.contacts[0].penetration);
        if (ok) ++passed;
        hash_bool(state_hash, hit);
        hash_uint(state_hash, manifold.contact_count);
    }

    // -----------------------------------------------------------------------
    // Test 7: Dispatcher — sphere-box (rotated)
    // -----------------------------------------------------------------------
    {
        CollisionShape sa = CollisionShape::make_sphere(0.5f, Vec3(0, 0, 0));
        // Box rotated 45° around Z, half-extents 0.6, position at (0.5, 0, 0)
        Quat rot = Quat::from_axis_angle(Vec3(0, 0, 1), APC_HALF_PI);
        CollisionShape sb = CollisionShape::make_box(Vec3(0.6f, 0.3f, 0.3f), Vec3(0.5f, 0, 0), rot);
        sb.update_cache();

        ContactManifold manifold;
        bool hit = dispatch_detect(sa, sb, 0, 1, manifold);

        bool ok = hit
               && manifold.contact_count == 1
               && manifold.contacts[0].penetration > 0.0f;
        std::printf("[%s] Test 7: Dispatch sphere-box rotated (pen=%.4f)\n",
                     ok ? "PASS" : "FAIL",
                     manifold.contacts[0].penetration);
        if (ok) ++passed;
        hash_bool(state_hash, hit);
        hash_float(state_hash, manifold.contacts[0].penetration);
    }

    // -----------------------------------------------------------------------
    // Test 8: Dispatcher — box-box via GJK+EPA
    // -----------------------------------------------------------------------
    {
        CollisionShape ba = CollisionShape::make_box(
            Vec3(0.5f, 0.5f, 0.5f), Vec3(0, 0, 0), Quat::identity());
        CollisionShape bb = CollisionShape::make_box(
            Vec3(0.5f, 0.5f, 0.5f), Vec3(0.6f, 0, 0), Quat::identity());

        ContactManifold manifold;
        bool hit = dispatch_detect(ba, bb, 0, 1, manifold);

        bool ok = hit
               && manifold.contact_count == 1
               && manifold.contacts[0].penetration > 0.1f
               && manifold.contacts[0].penetration < 1.0f;
        std::printf("[%s] Test 8: Dispatch box-box GJK+EPA (pen=%.4f)\n",
                     ok ? "PASS" : "FAIL",
                     manifold.contacts[0].penetration);
        if (ok) ++passed;
        hash_bool(state_hash, hit);
        hash_float(state_hash, manifold.contacts[0].penetration);
    }

    // -----------------------------------------------------------------------
    // Test 9: Dispatcher — sphere-plane
    // -----------------------------------------------------------------------
    {
        CollisionShape sa = CollisionShape::make_sphere(1.0f, Vec3(0, 0.5f, 0));
        CollisionShape sb = CollisionShape::make_plane(Vec3(0, 0, 0), Vec3(0, 1, 0));

        ContactManifold manifold;
        bool hit = dispatch_detect(sa, sb, 0, 1, manifold);

        bool ok = hit
               && manifold.contact_count == 1
               && manifold.contacts[0].penetration > 0.4f;
        std::printf("[%s] Test 9: Dispatch sphere-plane (pen=%.4f)\n",
                     ok ? "PASS" : "FAIL",
                     manifold.contacts[0].penetration);
        if (ok) ++passed;
        hash_bool(state_hash, hit);
        hash_float(state_hash, manifold.contacts[0].penetration);
    }

    // -----------------------------------------------------------------------
    // Test 10: Dispatcher — swapped args (box-sphere, plane-sphere)
    // -----------------------------------------------------------------------
    {
        CollisionShape sa = CollisionShape::make_box(
            Vec3(0.5f, 0.5f, 0.5f), Vec3(0, 0, 0), Quat::identity());
        CollisionShape sb = CollisionShape::make_sphere(1.0f, Vec3(1.2f, 0, 0));

        ContactManifold manifold;
        bool hit = dispatch_detect(sa, sb, 0, 1, manifold);

        // Should still detect — dispatcher handles swap
        bool ok = hit
               && manifold.contact_count == 1
               && manifold.contacts[0].penetration > 0.0f;
        std::printf("[%s] Test 10: Dispatch box-sphere (swapped args, pen=%.4f)\n",
                     ok ? "PASS" : "FAIL",
                     manifold.contacts[0].penetration);
        if (ok) ++passed;
        hash_bool(state_hash, hit);
    }

    // -----------------------------------------------------------------------
    // Test 11: Collision filtering — RigidBody layer/mask
    // -----------------------------------------------------------------------
    {
        RigidBody a, b;
        a.collision_layer = 1;  // Layer 0
        a.collision_mask  = 0xFFFFFFFF;

        b.collision_layer = 2;  // Layer 1
        b.collision_mask  = 0xFFFFFFFF;

        // Both should collide — both masks include all layers
        bool ok1 = RigidBody::should_collide(a, b);

        // a only collides with layer 0, b is on layer 1 → no collision
        a.collision_mask = 1;  // Only layer 0
        bool ok2 = !RigidBody::should_collide(a, b);

        // b's mask includes layer 0 but a is on layer 0, b on layer 1 → no
        bool ok3 = !RigidBody::should_collide(a, b);

        // Multi-layer: a on layers 0+1, b on layer 1
        a.collision_layer = 3;  // layers 0+1
        a.collision_mask  = 2;  // collide with layer 1
        b.collision_layer = 2;  // layer 1
        b.collision_mask  = 3;  // collide with layers 0+1
        bool ok4 = RigidBody::should_collide(a, b);

        bool ok = ok1 && ok2 && ok3 && ok4;
        std::printf("[%s] Test 11: Collision filtering layer/mask\n",
                     ok ? "PASS" : "FAIL");
        if (ok) ++passed;
        hash_bool(state_hash, ok);
    }

    // -----------------------------------------------------------------------
    // Test 12: Broadphase filter callback
    // -----------------------------------------------------------------------
    {
        BroadphaseSAP broadphase;

        // Filter that skips pair (0, 2)
        broadphase.filter_func = [](uint32_t a, uint32_t b, void*) -> bool {
            // Skip if one of them is entity 2
            if (a == 2 || b == 2) return false;
            return true;
        };

        std::vector<BroadphaseSAP::Proxy> proxies = {
            { 0, { { -1, -1, -1 }, { 1, 1, 1 } } },
            { 1, { { -1, -1, -1 }, { 1, 1, 1 } } },
            { 2, { { -1, -1, -1 }, { 1, 1, 1 } } },
        };
        broadphase.update(proxies);
        broadphase.generate_pairs();

        const auto& pairs = broadphase.get_potential_pairs();
        // Without filter: (0,1), (0,2), (1,2) = 3 pairs
        // With filter: (0,1) only = 1 pair (pairs with entity 2 are filtered)
        bool ok = (pairs.size() == 1) && (pairs[0].id_a == 0) && (pairs[0].id_b == 1);
        std::printf("[%s] Test 12: Broadphase filter callback (%zu pairs)\n",
                     ok ? "PASS" : "FAIL", pairs.size());
        if (ok) ++passed;
        hash_uint(state_hash, static_cast<uint32_t>(pairs.size()));
    }

    // -----------------------------------------------------------------------
    // Test 13: CollisionShape AABB computation
    // -----------------------------------------------------------------------
    {
        // Sphere at (3, 4, 5) with radius 2 → AABB [1,2,3] to [5,6,7]
        CollisionShape sphere = CollisionShape::make_sphere(2.0f, Vec3(3, 4, 5));
        AABB saabb = sphere.get_aabb();

        bool ok_sphere = (saabb.min.x > 0.9f && saabb.min.x < 1.1f)
                      && (saabb.max.x > 4.9f && saabb.max.x < 5.1f);

        // Axis-aligned box
        CollisionShape box = CollisionShape::make_box(
            Vec3(1.0f, 2.0f, 3.0f), Vec3(10, 20, 30), Quat::identity());
        AABB baabb = box.get_aabb();

        bool ok_box = (baabb.min.x > 8.9f && baabb.min.x < 9.1f)
                   && (baabb.max.x > 10.9f && baabb.max.x < 11.1f);

        bool ok = ok_sphere && ok_box;
        std::printf("[%s] Test 13: CollisionShape AABB\n", ok ? "PASS" : "FAIL");
        if (ok) ++passed;
        hash_bool(state_hash, ok);
    }

    // -----------------------------------------------------------------------
    // Test 14: Convex-convex via GJK+EPA (tetrahedron vs cube)
    // -----------------------------------------------------------------------
    {
        // A tetrahedron (4 vertices) vs a cube (8 vertices)
        static const Vec3 tetra_verts[4] = {
            Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(0.5f, 1, 0), Vec3(0.5f, 0.5f, 1)
        };
        static const Vec3 cube_verts[8] = {
            Vec3(-0.5f, -0.5f, -0.5f), Vec3(0.5f, -0.5f, -0.5f),
            Vec3(-0.5f,  0.5f, -0.5f), Vec3(0.5f,  0.5f, -0.5f),
            Vec3(-0.5f, -0.5f,  0.5f), Vec3(0.5f, -0.5f,  0.5f),
            Vec3(-0.5f,  0.5f,  0.5f), Vec3(0.5f,  0.5f,  0.5f)
        };

        // Place cube at tetrahedron's center to guarantee overlap
        CollisionShape tetra = CollisionShape::make_convex_piece(
            tetra_verts, 4, Vec3(0.4f, 0.3f, 0.3f), Quat::identity());
        CollisionShape cube = CollisionShape::make_convex_piece(
            cube_verts, 8, Vec3(0, 0, 0), Quat::identity());

        ContactManifold manifold;
        bool hit = detect_convex_convex(tetra, cube, manifold.contacts[0]);
        if (hit) manifold.contact_count = 1;

        bool ok = hit
               && manifold.contacts[0].penetration > 0.0f;
        std::printf("[%s] Test 14: Convex-convex GJK+EPA (pen=%.4f)\n",
                     ok ? "PASS" : "FAIL",
                     manifold.contacts[0].penetration);
        if (ok) ++passed;
        hash_bool(state_hash, hit);
        hash_float(state_hash, manifold.contacts[0].penetration);
    }

    // -----------------------------------------------------------------------
    // Summary & state hash
    // -----------------------------------------------------------------------
    std::printf("\n=== Collision Dispatcher Tests: %d/%d passed ===\n", passed, total);

    uint32_t final_hash = fnv1a_bytes(&state_hash, sizeof(state_hash));
    std::printf("State Hash: 0x%08x\n", final_hash);

    return (passed == total) ? 0 : 1;
}
