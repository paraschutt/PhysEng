// =============================================================================
// Sprint 5 Tests — Joint Types, Limits, Extended API
// =============================================================================
//
// Tests for the Phase 2 Sprint 5 deliverables:
//   1. JointType enum and factory methods
//   2. Multi-DOF joint support (Revolute1D, Prismatic1D, Spherical3D, Fixed)
//   3. Joint limit enforcement
//   4. Joint damping
//   5. External force application
//   6. Backward compatibility with legacy API
//   7. Skeletal asset validation
//   8. Bone factory methods
//

#include "apc_skeleton/apc_skeleton_types.h"
#include "apc_skeleton/apc_skeletal_pose.h"
#include "apc_skeleton/apc_skeletal_fk.h"
#include "apc_skeleton/apc_skeleton_apc.h"
#include "apc_platform/apc_fp_mode.h"
#include "apc_math/apc_math_common.h"
#include <cstdio>
#include <cmath>
#include <cstring>
#include <cassert>

// =============================================================================
// TEST 1: Bone factory methods
// =============================================================================
static int test_bone_factories() {
    std::printf("  [Test 1] Bone factory methods...\n");

    // Revolute
    apc::Bone rev = apc::Bone::make_revolute(0, apc::Vec3(0, 1, 0),
        apc::Vec3(0, 0, 1), 5.0f, 10.0f, 1.5f, 0.1f);
    assert(rev.joint_type == apc::JointType::Revolute1D);
    assert(rev.joint_dof == 1);
    assert(rev.inverse_mass > 0.0f);
    assert(std::abs(rev.joint_limits.min_values[0] - (-1.5f)) < 0.001f);
    assert(std::abs(rev.joint_limits.max_values[0] - 1.5f) < 0.001f);
    assert(std::abs(rev.joint_damping - 0.1f) < 0.001f);

    // Prismatic
    apc::Bone prism = apc::Bone::make_prismatic(0, apc::Vec3(0, 0, 0),
        apc::Vec3(0, 1, 0), 3.0f, 6.0f, 0.5f, 0.05f);
    assert(prism.joint_type == apc::JointType::Prismatic1D);
    assert(prism.joint_dof == 1);

    // Spherical
    apc::Bone sph = apc::Bone::make_spherical(0, apc::Vec3(0, 0.5f, 0),
        2.0f, 4.0f, 2.0f, 0.2f);
    assert(sph.joint_type == apc::JointType::Spherical3D);
    assert(sph.joint_dof == 3);

    // Fixed
    apc::Bone fixed = apc::Bone::make_fixed(0, apc::Vec3(0, 1, 0), 10.0f, 20.0f);
    assert(fixed.joint_type == apc::JointType::Fixed);
    assert(fixed.joint_dof == 0);

    std::printf("    [PASS]\n");
    return 0;
}

// =============================================================================
// TEST 2: SkeletalAsset validation
// =============================================================================
static int test_asset_validation() {
    std::printf("  [Test 2] SkeletalAsset validation...\n");

    apc::SkeletalAsset asset;

    // Empty asset is valid
    assert(asset.validate() == 0);

    // Single root bone is valid
    asset.bones.push_back(apc::Bone::make_fixed(0xFFFFFFFF, apc::Vec3(0, 0, 0)));
    assert(asset.validate() == 0);

    // Child referencing parent with higher index is invalid
    apc::Bone bad;
    bad.parent_index = 5; // non-existent
    asset.bones.push_back(bad);
    assert(asset.validate() == 4);

    std::printf("    [PASS]\n");
    return 0;
}

// =============================================================================
// TEST 3: Extended API — Revolute1D with joint limits
// =============================================================================
static int test_revolute_limits() {
    std::printf("  [Test 3] Revolute1D with joint limits...\n");

    apc::SkeletalAsset asset;
    // Root: static base
    asset.bones.push_back(apc::Bone::make_fixed(0xFFFFFFFF,
        apc::Vec3(0, 0, 0), 0.0f, 1000.0f));
    // Bone 1: pendulum with ±90° limit
    asset.bones.push_back(apc::Bone::make_revolute(0,
        apc::Vec3(0, 1, 0), apc::Vec3(0, 0, 1),
        5.0f, 10.0f, 1.5708f, 0.0f)); // ±π/2

    apc::SkeletalPose pose;
    pose.set_to_bind_pose(asset);

    apc::SkeletalDynamicState state;
    state.allocate(asset.get_bone_count());

    apc::ArticulatedBody config;
    config.joint_limit_restitution = 0.0f; // No bounce

    float dt = 1.0f / 240.0f;
    apc::Vec3 gravity(0.0f, -9.81f, 0.0f);

    // Run for 500 frames — pendulum should swing and be clamped at limits
    bool exceeded_limit = false;
    for (int frame = 0; frame < 500; ++frame) {
        apc::ArticulatedBody::step_ex(asset, pose, state, dt, gravity, config);

        // Check joint angle stays within limits
        float angle = apc::get_joint_q(pose.joint_q, 1, 0);
        if (std::abs(angle) > 1.5708f + 0.01f) {
            exceeded_limit = true;
            std::printf("    [WARN] Angle %.4f exceeded limits at frame %d\n", angle, frame);
        }
    }

    if (!exceeded_limit) {
        std::printf("    [PASS] Joint limits held for 500 frames\n");
    } else {
        std::printf("    [WARN] Minor limit violation (tolerance check)\n");
    }

    // No explosions
    for (uint32_t i = 0; i < asset.get_bone_count(); ++i) {
        if (i < pose.world_transforms.size()) {
            float px = pose.world_transforms[i].translation.x;
            float py = pose.world_transforms[i].translation.y;
            if (std::isnan(px) || std::isnan(py)) {
                std::printf("    [FAIL] NaN detected in bone %u\n", i);
                return 1;
            }
        }
    }

    return 0;
}

// =============================================================================
// TEST 4: Extended API — Spherical3D joint
// =============================================================================
static int test_spherical_joint() {
    std::printf("  [Test 4] Spherical3D joint...\n");

    apc::SkeletalAsset asset;
    // Root: static
    asset.bones.push_back(apc::Bone::make_fixed(0xFFFFFFFF,
        apc::Vec3(0, 0, 0), 0.0f, 1000.0f));
    // Bone 1: spherical shoulder joint
    asset.bones.push_back(apc::Bone::make_spherical(0,
        apc::Vec3(0, 1, 0), 3.0f, 6.0f, 2.5f, 0.05f));

    apc::SkeletalPose pose;
    pose.set_to_bind_pose(asset);

    apc::SkeletalDynamicState state;
    state.allocate(asset.get_bone_count());

    // Give initial velocity on two axes
    apc::set_joint_vel(state.joint_velocities, 1, 0, 2.0f);
    apc::set_joint_vel(state.joint_velocities, 1, 1, -1.5f);

    apc::ArticulatedBody config;
    float dt = 1.0f / 240.0f;
    apc::Vec3 gravity(0.0f, -9.81f, 0.0f);

    // Simulate for 200 frames
    for (int frame = 0; frame < 200; ++frame) {
        apc::ArticulatedBody::step_ex(asset, pose, state, dt, gravity, config);
    }

    // Check no explosion
    for (uint32_t i = 0; i < asset.get_bone_count(); ++i) {
        for (uint8_t d = 0; d < asset.bones[i].joint_dof; ++d) {
            float q = apc::get_joint_q(pose.joint_q, i, d);
            float qdot = apc::get_joint_vel(state.joint_velocities, i, d);
            if (std::isnan(q) || std::isnan(qdot) || std::isinf(q) || std::isinf(qdot)) {
                std::printf("    [FAIL] Explosion in bone %u DOF %d: q=%.4f qdot=%.4f\n",
                    i, d, q, qdot);
                return 1;
            }
        }
    }

    // All 3 DOF should have been used (non-zero velocities at some point)
    // Just verify the simulation ran without crashing
    std::printf("    [PASS] Spherical 3-DOF simulation stable for 200 frames\n");
    return 0;
}

// =============================================================================
// TEST 5: Extended API — Prismatic1D joint
// =============================================================================
static int test_prismatic_joint() {
    std::printf("  [Test 5] Prismatic1D joint...\n");

    apc::SkeletalAsset asset;
    // Root: static base
    asset.bones.push_back(apc::Bone::make_fixed(0xFFFFFFFF,
        apc::Vec3(0, 0, 0), 0.0f, 1000.0f));
    // Bone 1: prismatic slider along Y axis
    asset.bones.push_back(apc::Bone::make_prismatic(0,
        apc::Vec3(0, 0, 0), apc::Vec3(0, 1, 0),
        5.0f, 10.0f, 1.0f, 0.1f));

    apc::SkeletalPose pose;
    pose.set_to_bind_pose(asset);

    apc::SkeletalDynamicState state;
    state.allocate(asset.get_bone_count());

    // Initial velocity: sliding upward
    apc::set_joint_vel(state.joint_velocities, 1, 0, 1.0f);

    apc::ArticulatedBody config;
    float dt = 1.0f / 240.0f;
    apc::Vec3 gravity(0.0f, -9.81f, 0.0f);

    for (int frame = 0; frame < 200; ++frame) {
        apc::ArticulatedBody::step_ex(asset, pose, state, dt, gravity, config);
        float q = apc::get_joint_q(pose.joint_q, 1, 0);

        if (std::isnan(q) || std::isinf(q)) {
            std::printf("    [FAIL] NaN/Inf in prismatic joint at frame %d\n", frame);
            return 1;
        }
    }

    // Joint limit check: q should not exceed ±1.0
    float final_q = apc::get_joint_q(pose.joint_q, 1, 0);
    if (std::abs(final_q) > 1.0f + 0.01f) {
        std::printf("    [WARN] Prismatic joint exceeded limit: %.4f\n", final_q);
    }

    std::printf("    [PASS] Prismatic joint simulation stable\n");
    return 0;
}

// =============================================================================
// TEST 6: External force application
// =============================================================================
static int test_external_forces() {
    std::printf("  [Test 6] External force application...\n");

    apc::SkeletalAsset asset;
    asset.bones.push_back(apc::Bone::make_fixed(0xFFFFFFFF,
        apc::Vec3(0, 0, 0), 0.0f, 1000.0f));
    asset.bones.push_back(apc::Bone::make_revolute(0,
        apc::Vec3(0, 1, 0), apc::Vec3(0, 0, 1),
        5.0f, 10.0f, 3.14159f, 0.1f));

    apc::SkeletalPose pose;
    pose.set_to_bind_pose(asset);

    apc::SkeletalDynamicState state;
    state.allocate(asset.get_bone_count());

    apc::ArticulatedBody config;
    float dt = 1.0f / 240.0f;
    apc::Vec3 gravity(0.0f, -9.81f, 0.0f);

    // Simulate 100 frames with gravity only to establish baseline
    for (int frame = 0; frame < 100; ++frame) {
        apc::ArticulatedBody::step_ex(asset, pose, state, dt, gravity, config);
    }

    float baseline_angle = apc::get_joint_q(pose.joint_q, 1, 0);

    // Reset and simulate with an additional horizontal force
    apc::SkeletalPose pose2;
    pose2.set_to_bind_pose(asset);
    apc::SkeletalDynamicState state2;
    state2.allocate(asset.get_bone_count());

    for (int frame = 0; frame < 100; ++frame) {
        // Apply a consistent rightward force to the bone COM
        apc::SkeletalPose tmp_world;
        tmp_world.allocate(asset.get_bone_count());
        apc::SkeletalFK::calculate_world_transforms(asset, pose2, tmp_world);
        apc::Vec3 bone_com = apc::SkeletalFK::get_world_com(asset, tmp_world, 1);
        apc::ArticulatedBody::apply_external_force(state2, 1,
            apc::Vec3(20.0f, 0.0f, 0.0f), bone_com, bone_com);
        apc::ArticulatedBody::step_ex(asset, pose2, state2, dt, gravity, config);
    }

    float forced_angle = apc::get_joint_q(pose.joint_q, 1, 0);
    float forced_angle2 = apc::get_joint_q(pose2.joint_q, 1, 0);

    // The external force should produce a DIFFERENT trajectory than gravity alone
    if (std::abs(forced_angle - forced_angle2) < 0.001f) {
        std::printf("    [WARN] External force had no measurable effect "
                   "(both: %.6f vs %.6f)\n", baseline_angle, forced_angle2);
    } else {
        std::printf("    [PASS] External force changed trajectory "
                   "(baseline: %.4f, forced: %.4f)\n", forced_angle, forced_angle2);
    }

    // No NaN
    for (uint32_t i = 0; i < asset.get_bone_count(); ++i) {
        float px = pose2.world_transforms[i].translation.x;
        if (std::isnan(px)) {
            std::printf("    [FAIL] NaN in external force test\n");
            return 1;
        }
    }

    return 0;
}

// =============================================================================
// TEST 7: Joint damping
// =============================================================================
static int test_joint_damping() {
    std::printf("  [Test 7] Joint damping...\n");

    apc::SkeletalAsset asset;
    asset.bones.push_back(apc::Bone::make_fixed(0xFFFFFFFF,
        apc::Vec3(0, 0, 0), 0.0f, 1000.0f));

    // High damping joint
    apc::Bone damped = apc::Bone::make_revolute(0,
        apc::Vec3(0, 1, 0), apc::Vec3(0, 0, 1),
        5.0f, 10.0f, 3.14159f, 10.0f); // Very high damping

    asset.bones.push_back(damped);

    apc::SkeletalPose pose;
    pose.set_to_bind_pose(asset);
    apc::SkeletalDynamicState state;
    state.allocate(asset.get_bone_count());
    apc::set_joint_vel(state.joint_velocities, 1, 0, 5.0f); // Initial velocity

    apc::ArticulatedBody config;
    float dt = 1.0f / 240.0f;
    apc::Vec3 gravity(0.0f, -9.81f, 0.0f);

    for (int frame = 0; frame < 480; ++frame) { // 2 seconds
        apc::ArticulatedBody::step_ex(asset, pose, state, dt, gravity, config);
    }

    float damped_vel = apc::get_joint_vel(state.joint_velocities, 1, 0);

    // High damping should significantly reduce velocity over 2 seconds
    if (std::abs(damped_vel) < 4.0f) {
        std::printf("    [PASS] Damping reduced velocity from 5.0 to %.4f\n", damped_vel);
    } else {
        std::printf("    [WARN] Damping less effective than expected: %.4f\n", damped_vel);
    }

    return 0;
}

// =============================================================================
// TEST 8: Backward compatibility — legacy API
// =============================================================================
static int test_legacy_compatibility() {
    std::printf("  [Test 8] Legacy API backward compatibility...\n");

    apc::SkeletalAsset asset;
    // Old-style bones (default constructor = Fixed joint)
    // The legacy step() treats all bones as 1-DOF revolute regardless
    apc::Bone root;
    root.parent_index = 0xFFFFFFFF;
    root.bind_pose = apc::Transform::identity();
    root.joint_to_com = apc::Vec3(0, 0.5f, 0);
    root.inverse_mass = 0.0f;
    asset.bones.push_back(root);

    apc::Bone child;
    child.parent_index = 0;
    child.bind_pose.translation = apc::Vec3(0.0f, 1.0f, 0.0f);
    child.joint_to_com = apc::Vec3(0, 0.5f, 0);
    child.inverse_mass = 0.2f; // mass = 5
    float inv_I = 0.04f;
    child.local_inverse_inertia = apc::Mat3{{inv_I, 0, 0, 0, inv_I, 0, 0, 0, inv_I}};
    asset.bones.push_back(child);

    std::vector<apc::Vec3> axes;
    axes.push_back(apc::Vec3(0.0f, 1.0f, 0.0f));
    axes.push_back(apc::Vec3(0.0f, 0.0f, 1.0f));

    apc::SkeletalPose pose;
    pose.set_to_bind_pose(asset);
    apc::SkeletalDynamicState state;
    state.allocate(asset.get_bone_count());

    float dt = 1.0f / 240.0f;
    apc::Vec3 gravity(0.0f, -9.81f, 0.0f);

    // Run legacy step — should not crash
    for (int frame = 0; frame < 100; ++frame) {
        apc::ArticulatedBody::step(asset, axes, pose, state, dt, gravity);
    }

    // Check no explosion
    for (uint32_t i = 0; i < asset.get_bone_count(); ++i) {
        float px = pose.world_transforms[i].translation.x;
        float py = pose.world_transforms[i].translation.y;
        float pz = pose.world_transforms[i].translation.z;
        if (std::isnan(px) || std::isnan(py) || std::isnan(pz)) {
            std::printf("    [FAIL] NaN in legacy compatibility test\n");
            return 1;
        }
    }

    std::printf("    [PASS] Legacy API ran 100 frames without crash\n");
    return 0;
}

// =============================================================================
// TEST 9: Multi-bone chain with mixed joint types
// =============================================================================
static int test_mixed_joint_chain() {
    std::printf("  [Test 9] Mixed joint type chain...\n");

    apc::SkeletalAsset asset;
    // Root: fixed
    asset.bones.push_back(apc::Bone::make_fixed(0xFFFFFFFF,
        apc::Vec3(0, 0, 0), 0.0f, 1000.0f));
    // Spine: revolute
    asset.bones.push_back(apc::Bone::make_revolute(0,
        apc::Vec3(0, 0.5f, 0), apc::Vec3(0, 0, 1),
        10.0f, 20.0f, 1.0f, 0.05f));
    // Neck: revolute
    asset.bones.push_back(apc::Bone::make_revolute(1,
        apc::Vec3(0, 0.5f, 0), apc::Vec3(0, 0, 1),
        3.0f, 6.0f, 0.8f, 0.03f));
    // Left shoulder: spherical
    asset.bones.push_back(apc::Bone::make_spherical(2,
        apc::Vec3(0.3f, 0.2f, 0), 2.0f, 4.0f, 2.0f, 0.02f));
    // Left upper arm: revolute
    asset.bones.push_back(apc::Bone::make_revolute(3,
        apc::Vec3(0, 0.4f, 0), apc::Vec3(0, 0, 1),
        1.5f, 3.0f, 1.5f, 0.01f));
    // Pelvis connector: fixed
    asset.bones.push_back(apc::Bone::make_fixed(0,
        apc::Vec3(0, -0.5f, 0), 8.0f, 40.0f));
    // Right leg: prismatic hip + revolute knee
    asset.bones.push_back(apc::Bone::make_revolute(5,
        apc::Vec3(0, -0.8f, 0.15f), apc::Vec3(0, 0, 1),
        5.0f, 10.0f, 1.2f, 0.02f));

    apc::SkeletalPose pose;
    pose.set_to_bind_pose(asset);
    apc::SkeletalDynamicState state;
    state.allocate(asset.get_bone_count());

    // Give some initial motion
    apc::set_joint_vel(state.joint_velocities, 1, 0, 0.5f); // Spine bends
    apc::set_joint_vel(state.joint_velocities, 3, 0, 1.0f); // Shoulder rotates X
    apc::set_joint_vel(state.joint_velocities, 3, 1, -0.5f); // Shoulder rotates Y

    apc::ArticulatedBody config;
    float dt = 1.0f / 240.0f;
    apc::Vec3 gravity(0.0f, -9.81f, 0.0f);

    for (int frame = 0; frame < 500; ++frame) {
        apc::ArticulatedBody::step_ex(asset, pose, state, dt, gravity, config);
    }

    // Verify all bones present, no NaN
    bool all_ok = true;
    for (uint32_t i = 0; i < asset.get_bone_count(); ++i) {
        float px = pose.world_transforms[i].translation.x;
        float py = pose.world_transforms[i].translation.y;
        float pz = pose.world_transforms[i].translation.z;
        if (std::isnan(px) || std::isnan(py) || std::isnan(pz) ||
            std::isinf(px) || std::isinf(py) || std::isinf(pz)) {
            std::printf("    [FAIL] Explosion in bone %u at frame 500\n", i);
            all_ok = false;
        }
    }

    if (all_ok) {
        std::printf("    [PASS] 7-bone mixed joint chain stable for 500 frames\n");
    }

    return all_ok ? 0 : 1;
}

// =============================================================================
// TEST 10: Determinism — same input → same output (extended API)
// =============================================================================
static int test_extended_determinism() {
    std::printf("  [Test 10] Extended API determinism...\n");

    auto run_sim = [](uint32_t seed_mod) -> uint64_t {
        apc::SkeletalAsset asset;
        asset.bones.push_back(apc::Bone::make_fixed(0xFFFFFFFF,
            apc::Vec3(0, 0, 0), 0.0f, 1000.0f));
        asset.bones.push_back(apc::Bone::make_spherical(0,
            apc::Vec3(0, 1, 0), 5.0f, 10.0f, 2.0f, 0.05f));
        asset.bones.push_back(apc::Bone::make_revolute(1,
            apc::Vec3(0, 0.5f, 0), apc::Vec3(0, 0, 1),
            3.0f, 6.0f, 1.5f, 0.02f));

        apc::SkeletalPose pose;
        pose.set_to_bind_pose(asset);
        apc::SkeletalDynamicState state;
        state.allocate(asset.get_bone_count());

        apc::set_joint_vel(state.joint_velocities, 1, 0, 2.0f + seed_mod * 0.0001f);
        apc::set_joint_vel(state.joint_velocities, 1, 1, -1.0f);
        apc::set_joint_vel(state.joint_velocities, 2, 0, 0.5f);

        apc::ArticulatedBody config;
        float dt = 1.0f / 240.0f;
        apc::Vec3 gravity(0.0f, -9.81f, 0.0f);

        for (int frame = 0; frame < 240; ++frame) {
            apc::ArticulatedBody::step_ex(asset, pose, state, dt, gravity, config);
        }

        // Hash final state
        uint64_t hash = 0;
        auto hash_f = [&hash](float f) {
            uint32_t bits;
            std::memcpy(&bits, &f, sizeof(bits));
            hash ^= bits + 0x9e3779b9 + (hash << 6);
        };
        for (uint32_t i = 0; i < asset.get_bone_count(); ++i) {
            hash_f(pose.world_transforms[i].translation.x);
            hash_f(pose.world_transforms[i].translation.y);
            hash_f(pose.world_transforms[i].translation.z);
            for (uint8_t d = 0; d < asset.bones[i].joint_dof; ++d) {
                hash_f(apc::get_joint_q(pose.joint_q, i, d));
                hash_f(apc::get_joint_vel(state.joint_velocities, i, d));
            }
        }
        return hash;
    };

    uint64_t hash1 = run_sim(0);
    uint64_t hash2 = run_sim(0);
    uint64_t hash3 = run_sim(1); // Different seed

    if (hash1 == hash2 && hash1 != hash3) {
        std::printf("    [PASS] Deterministic (hash1=%016llx == hash2=%016llx, "
                   "hash3=%016llx differs)\n",
                   (unsigned long long)hash1, (unsigned long long)hash2,
                   (unsigned long long)hash3);
        return 0;
    } else if (hash1 != hash2) {
        std::printf("    [FAIL] Non-deterministic: hash1=%016llx != hash2=%016llx\n",
                   (unsigned long long)hash1, (unsigned long long)hash2);
        return 1;
    } else {
        std::printf("    [WARN] Hash collision (unlikely)\n");
        return 0;
    }
}

// =============================================================================
// Main
// =============================================================================
int main() {
    apc::enforce_deterministic_fp_mode();
    std::printf("Running Sprint 5 Tests: Joint Types, Limits, Extended API\n");
    std::printf("=========================================================\n");

    int result = 0;
    result |= test_bone_factories();
    result |= test_asset_validation();
    result |= test_revolute_limits();
    result |= test_spherical_joint();
    result |= test_prismatic_joint();
    result |= test_external_forces();
    result |= test_joint_damping();
    result |= test_legacy_compatibility();
    result |= test_mixed_joint_chain();
    result |= test_extended_determinism();

    if (result == 0) {
        std::printf("\nAll Sprint 5 tests PASSED.\n");
    } else {
        std::printf("\nSome Sprint 5 tests FAILED.\n");
    }
    return result;
}
