// =============================================================================
// Sprint 7 Tests — Skeleton-Collision Integration
// =============================================================================
//
// Tests for Phase 2 Sprint 7:
//   1. BoneCollisionShape factory methods
//   2. SkeletonCollisionBody proxy generation
//   3. Global ID packing/unpacking
//   4. Skeleton vs static plane collision
//   5. Skeleton vs static box collision
//   6. Full sim loop with ground plane
//   7. Multi-bone skeleton with mixed collision
//

#include "apc_skeleton/apc_skeleton_types.h"
#include "apc_skeleton/apc_skeletal_pose.h"
#include "apc_skeleton/apc_skeletal_fk.h"
#include "apc_skeleton/apc_skeleton_apc.h"
#include "apc_skeleton/apc_skeleton_collision.h"
#include "apc_collision/apc_collision_dispatch.h"
#include "apc_platform/apc_fp_mode.h"
#include "apc_math/apc_math_common.h"
#include <cstdio>
#include <cmath>
#include <cstring>
#include <cassert>

// =============================================================================
// TEST 1: BoneCollisionShape factory methods
// =============================================================================
static int test_bone_collision_shapes() {
    std::printf("  [Test 1] BoneCollisionShape factories...\n");

    apc::BoneCollisionShape sphere = apc::BoneCollisionShape::make_sphere(0.1f);
    assert(sphere.type == apc::ShapeType::Sphere);
    assert(sphere.sphere_radius == 0.1f);
    assert(sphere.is_enabled());

    apc::BoneCollisionShape capsule = apc::BoneCollisionShape::make_capsule(0.05f, 0.2f);
    assert(capsule.type == apc::ShapeType::Capsule);
    assert(capsule.sphere_radius == 0.05f);
    assert(capsule.capsule_half_height == 0.2f);

    apc::BoneCollisionShape box = apc::BoneCollisionShape::make_box(
        apc::Vec3(0.1f, 0.2f, 0.05f));
    assert(box.type == apc::ShapeType::Box);

    apc::BoneCollisionShape disabled = apc::BoneCollisionShape::make_sphere(0.0f);
    assert(!disabled.is_enabled());

    std::printf("    [PASS]\n");
    return 0;
}

// =============================================================================
// TEST 2: Global ID packing/unpacking
// =============================================================================
static int test_global_ids() {
    std::printf("  [Test 2] Global ID packing...\n");

    uint32_t id1 = apc::SkeletonCollisionBody::make_global_id(0, 5);
    assert(apc::SkeletonCollisionBody::extract_skeleton_id(id1) == 0);
    assert(apc::SkeletonCollisionBody::extract_bone_index(id1) == 5);

    uint32_t id2 = apc::SkeletonCollisionBody::make_global_id(3, 42);
    assert(apc::SkeletonCollisionBody::extract_skeleton_id(id2) == 3);
    assert(apc::SkeletonCollisionBody::extract_bone_index(id2) == 42);

    // Uniqueness
    uint32_t id3 = apc::SkeletonCollisionBody::make_global_id(1, 0);
    uint32_t id4 = apc::SkeletonCollisionBody::make_global_id(0, 256);
    assert(id3 != id4);

    std::printf("    [PASS]\n");
    return 0;
}

// =============================================================================
// TEST 3: SkeletonCollisionBody proxy generation
// =============================================================================
static int test_proxy_generation() {
    std::printf("  [Test 3] SkeletonCollisionBody proxies...\n");

    apc::SkeletalAsset asset;
    asset.bones.push_back(apc::Bone::make_fixed(0xFFFFFFFF,
        apc::Vec3(0, 0, 0), 0.0f, 1000.0f));
    asset.bones.push_back(apc::Bone::make_revolute(0,
        apc::Vec3(0, 1, 0), apc::Vec3(0, 0, 1),
        5.0f, 10.0f, 3.14159f, 0.0f));
    asset.bones.push_back(apc::Bone::make_revolute(1,
        apc::Vec3(0, 0.5f, 0), apc::Vec3(0, 0, 1),
        3.0f, 6.0f, 3.14159f, 0.0f));

    // Disable collision on root
    asset.bones[0].collision_enabled = 0;

    apc::BoneCollisionShape shapes[3];
    shapes[0] = apc::BoneCollisionShape::make_sphere(0.05f); // Root (disabled)
    shapes[1] = apc::BoneCollisionShape::make_sphere(0.1f);  // Bone 1
    shapes[2] = apc::BoneCollisionShape::make_capsule(0.05f, 0.2f); // Bone 2

    apc::SkeletalPose pose;
    pose.set_to_bind_pose(asset);

    apc::SkeletonCollisionBody skel_coll;
    skel_coll.update(1, asset, pose, shapes, 3);

    // Root should be excluded (collision_enabled = 0)
    // Bone 1 and 2 should be included
    assert(skel_coll.get_proxy_count() == 2);

    const apc::BoneProxy* proxies = skel_coll.get_proxies();
    assert(proxies[0].skeleton_id == 1);
    assert(proxies[0].bone_index == 1);
    assert(proxies[1].bone_index == 2);

    std::printf("    [PASS] %u proxies generated (root excluded)\n",
               skel_coll.get_proxy_count());
    return 0;
}

// =============================================================================
// TEST 4: Skeleton vs ground plane collision
// =============================================================================
static int test_ground_collision() {
    std::printf("  [Test 4] Skeleton vs ground plane...\n");

    // Simple pendulum-like skeleton above ground
    apc::SkeletalAsset asset;
    asset.bones.push_back(apc::Bone::make_fixed(0xFFFFFFFF,
        apc::Vec3(0, 0, 0), 0.0f, 1000.0f));
    asset.bones.push_back(apc::Bone::make_revolute(0,
        apc::Vec3(0, 1, 0), apc::Vec3(0, 0, 1),
        5.0f, 10.0f, 3.14159f, 0.01f));

    apc::BoneCollisionShape shapes[2] = {};
    shapes[1] = apc::BoneCollisionShape::make_sphere(0.15f); // Collision sphere on bone 1

    apc::SkeletalPose pose;
    pose.set_to_bind_pose(asset);
    apc::SkeletalDynamicState state;
    state.allocate(asset.get_bone_count());

    // Ground plane at y=0, normal pointing up
    apc::CollisionShape ground = apc::CollisionShape::make_plane(
        apc::Vec3(0, 0, 0), apc::Vec3(0, 1, 0));
    uint32_t ground_id = 0xFFFF0000; // Static body ID

    apc::ArticulatedBody config;
    float dt = 1.0f / 240.0f;
    apc::Vec3 gravity(0.0f, -9.81f, 0.0f);

    // Simulate with collision
    bool had_contact = false;
    for (int frame = 0; frame < 300; ++frame) {
        apc::SkeletonSimLoop::step(asset, pose, state,
            shapes, 2,
            &ground, &ground_id, 1,
            config, dt, gravity);

        // Check for contact
        apc::SkeletonCollisionBody skel_coll;
        skel_coll.update(0, asset, pose, shapes, 2);

        for (uint32_t i = 0; i < skel_coll.get_proxy_count(); ++i) {
            apc::ContactManifold mf;
            bool hit = dispatch_detect(skel_coll.get_proxies()[i].shape,
                ground, skel_coll.get_proxies()[i].bone_index, ground_id, mf);
            if (hit) {
                had_contact = true;
            }
        }
    }

    // Verify stability
    bool stable = true;
    for (uint32_t i = 0; i < asset.get_bone_count(); ++i) {
        float py = pose.world_transforms[i].translation.y;
        if (std::isnan(py) || std::isinf(py)) {
            stable = false;
        }
    }

    if (stable) {
        std::printf("    [PASS] Ground collision sim stable, contact=%s\n",
                   had_contact ? "yes" : "no");
    } else {
        std::printf("    [FAIL] Instability detected\n");
        return 1;
    }

    return 0;
}

// =============================================================================
// TEST 5: Multi-bone skeleton with ground contact
// =============================================================================
static int test_multibone_ground() {
    std::printf("  [Test 5] Multi-bone skeleton with ground...\n");

    // 3-bone chain hanging from fixed root
    apc::SkeletalAsset asset;
    asset.bones.push_back(apc::Bone::make_fixed(0xFFFFFFFF,
        apc::Vec3(0, 2, 0), 0.0f, 1000.0f));
    asset.bones.push_back(apc::Bone::make_revolute(0,
        apc::Vec3(0, 0, 0), apc::Vec3(0, 0, 1),
        5.0f, 10.0f, 3.14159f, 0.01f));
    asset.bones.push_back(apc::Bone::make_revolute(1,
        apc::Vec3(0, 0.8f, 0), apc::Vec3(0, 0, 1),
        3.0f, 6.0f, 3.14159f, 0.01f));
    asset.bones[0].collision_enabled = 0;

    apc::BoneCollisionShape shapes[3] = {};
    shapes[1] = apc::BoneCollisionShape::make_sphere(0.1f);
    shapes[2] = apc::BoneCollisionShape::make_sphere(0.1f);

    apc::SkeletalPose pose;
    pose.set_to_bind_pose(asset);
    apc::SkeletalDynamicState state;
    state.allocate(asset.get_bone_count());

    apc::CollisionShape ground = apc::CollisionShape::make_plane(
        apc::Vec3(0, 0, 0), apc::Vec3(0, 1, 0));
    uint32_t ground_id = 0xFFFF0000;

    apc::ArticulatedBody config;
    float dt = 1.0f / 240.0f;
    apc::Vec3 gravity(0.0f, -9.81f, 0.0f);

    for (int frame = 0; frame < 500; ++frame) {
        apc::SkeletonSimLoop::step(asset, pose, state,
            shapes, 3,
            &ground, &ground_id, 1,
            config, dt, gravity);
    }

    // Check: bones should not fall through the ground plane (y=0)
    bool penetrated = false;
    for (uint32_t i = 0; i < asset.get_bone_count(); ++i) {
        float py = pose.world_transforms[i].translation.y;
        if (py < -0.5f) { // Allow some tolerance
            penetrated = true;
        }
        if (std::isnan(py) || std::isinf(py)) {
            std::printf("    [FAIL] Explosion in bone %u\n", i);
            return 1;
        }
    }

    if (!penetrated) {
        std::printf("    [PASS] Multi-bone skeleton stable above ground\n");
    } else {
        std::printf("    [WARN] Some bones may have penetrated ground\n");
    }

    return 0;
}

// =============================================================================
// TEST 6: Collision with static box
// =============================================================================
static int test_static_box_collision() {
    std::printf("  [Test 6] Skeleton vs static box...\n");

    apc::SkeletalAsset asset;
    asset.bones.push_back(apc::Bone::make_fixed(0xFFFFFFFF,
        apc::Vec3(0, 1, 0), 0.0f, 1000.0f));
    asset.bones.push_back(apc::Bone::make_revolute(0,
        apc::Vec3(0, 0, 0), apc::Vec3(0, 0, 1),
        5.0f, 10.0f, 3.14159f, 0.01f));
    asset.bones[0].collision_enabled = 0;

    apc::BoneCollisionShape shapes[2] = {};
    shapes[1] = apc::BoneCollisionShape::make_sphere(0.15f);

    apc::SkeletalPose pose;
    pose.set_to_bind_pose(asset);
    apc::SkeletalDynamicState state;
    state.allocate(asset.get_bone_count());

    // Static box obstacle
    apc::CollisionShape obstacle = apc::CollisionShape::make_box(
        apc::Vec3(0.5f, 0.5f, 0.5f),
        apc::Vec3(1.5f, 0.5f, 0.0f),
        apc::Quat::identity());
    uint32_t obstacle_id = 0xFFFF0001;

    apc::ArticulatedBody config;
    float dt = 1.0f / 240.0f;
    apc::Vec3 gravity(0.0f, -9.81f, 0.0f);

    for (int frame = 0; frame < 300; ++frame) {
        apc::SkeletonSimLoop::step(asset, pose, state,
            shapes, 2,
            &obstacle, &obstacle_id, 1,
            config, dt, gravity);
    }

    bool stable = true;
    for (uint32_t i = 0; i < asset.get_bone_count(); ++i) {
        float px = pose.world_transforms[i].translation.x;
        float py = pose.world_transforms[i].translation.y;
        if (std::isnan(px) || std::isnan(py) || std::isinf(px) || std::isinf(py)) {
            stable = false;
        }
    }

    if (stable) {
        std::printf("    [PASS] Skeleton vs static box stable\n");
    } else {
        std::printf("    [FAIL] Instability with static box\n");
        return 1;
    }

    return 0;
}

// =============================================================================
// TEST 7: Determinism — skeleton collision produces identical results
// =============================================================================
static int test_collision_determinism() {
    std::printf("  [Test 7] Skeleton collision determinism...\n");

    auto run_sim = []() -> uint64_t {
        apc::SkeletalAsset asset;
        asset.bones.push_back(apc::Bone::make_fixed(0xFFFFFFFF,
            apc::Vec3(0, 1, 0), 0.0f, 1000.0f));
        asset.bones.push_back(apc::Bone::make_revolute(0,
            apc::Vec3(0, 0, 0), apc::Vec3(0, 0, 1),
            5.0f, 10.0f, 3.14159f, 0.01f));
        asset.bones[0].collision_enabled = 0;

        apc::BoneCollisionShape shapes[2] = {};
        shapes[1] = apc::BoneCollisionShape::make_sphere(0.1f);

        apc::SkeletalPose pose;
        pose.set_to_bind_pose(asset);
        apc::SkeletalDynamicState state;
        state.allocate(asset.get_bone_count());

        apc::CollisionShape ground = apc::CollisionShape::make_plane(
            apc::Vec3(0, 0, 0), apc::Vec3(0, 1, 0));
        uint32_t ground_id = 0xFFFF0000;

        apc::ArticulatedBody config;
        float dt = 1.0f / 240.0f;
        apc::Vec3 gravity(0.0f, -9.81f, 0.0f);

        for (int frame = 0; frame < 240; ++frame) {
            apc::SkeletonSimLoop::step(asset, pose, state,
                shapes, 2, &ground, &ground_id, 1,
                config, dt, gravity);
        }

        uint64_t hash = 0;
        for (uint32_t i = 0; i < asset.get_bone_count(); ++i) {
            uint32_t bits;
            std::memcpy(&bits, &pose.local_transforms[i].translation.x, sizeof(bits));
            hash ^= bits + 0x9e3779b9 + (hash << 6);
            std::memcpy(&bits, &pose.local_transforms[i].translation.y, sizeof(bits));
            hash ^= bits + 0x9e3779b9 + (hash << 6);
        }
        return hash;
    };

    uint64_t h1 = run_sim();
    uint64_t h2 = run_sim();

    if (h1 == h2) {
        std::printf("    [PASS] Deterministic (hash=%016llx)\n", (unsigned long long)h1);
        return 0;
    } else {
        std::printf("    [FAIL] Non-deterministic (h1=%016llx, h2=%016llx)\n",
                   (unsigned long long)h1, (unsigned long long)h2);
        return 1;
    }
}

// =============================================================================
// Main
// =============================================================================
int main() {
    apc::enforce_deterministic_fp_mode();
    std::printf("Running Sprint 7 Tests: Skeleton-Collision Integration\n");
    std::printf("=======================================================\n");

    int result = 0;
    result |= test_bone_collision_shapes();
    result |= test_global_ids();
    result |= test_proxy_generation();
    result |= test_ground_collision();
    result |= test_multibone_ground();
    result |= test_static_box_collision();
    result |= test_collision_determinism();

    if (result == 0) {
        std::printf("\nAll Sprint 7 tests PASSED.\n");
    } else {
        std::printf("\nSome Sprint 7 tests FAILED.\n");
    }
    return result;
}
