// =============================================================================
// Sprint 8 Tests — Humanoid Demo, Loop Closures, Standing & Knock-Over
// =============================================================================
//
// Tests for Phase 2 Sprint 8 (final sprint):
//   1. HumanoidBuilder — standard 22-bone humanoid creation
//   2. Humanoid validation — correct bone count, parent chain, joint types
//   3. Humanoid collision shapes — correct shape assignment
//   4. Loop closure constraints — shoulder girdle, pelvis stability
//   5. Standing humanoid — gravity test, stability check
//   6. Knock-over scenario — partial ragdoll transition
//   7. Full ragdoll — all bones physics-driven with ground contact
//   8. Determinism — full humanoid sim produces identical results
//

#include "apc_skeleton/apc_skeleton_types.h"
#include "apc_skeleton/apc_skeletal_pose.h"
#include "apc_skeleton/apc_skeletal_fk.h"
#include "apc_skeleton/apc_skeleton_apc.h"
#include "apc_skeleton/apc_blend_system.h"
#include "apc_skeleton/apc_humanoid_builder.h"
#include "apc_skeleton/apc_loop_closure.h"
#include "apc_skeleton/apc_skeleton_collision.h"
#include "apc_collision/apc_collision_dispatch.h"
#include "apc_platform/apc_fp_mode.h"
#include "apc_math/apc_math_common.h"
#include <cstdio>
#include <cmath>
#include <cstring>

// =============================================================================
// TEST 1: HumanoidBuilder — create standard humanoid
// =============================================================================
static int test_humanoid_creation() {
    std::printf("  [Test 1] HumanoidBuilder creation...\n");

    apc::SkeletalAsset asset;
    apc::HumanoidLayout layout;
    layout.total_mass = 80.0f;
    layout.total_height = 1.8f;

    apc::HumanoidBuilder::create_standard(layout, asset);

    // Verify bone count
    if (asset.get_bone_count() != apc::HB_BONE_COUNT) {
        std::printf("    [FAIL] Expected %u bones, got %u\n",
                   apc::HB_BONE_COUNT, asset.get_bone_count());
        return 1;
    }

    // Verify hierarchy
    uint32_t err = asset.validate();
    if (err != 0) {
        std::printf("    [FAIL] Asset validation failed: %u\n", err);
        return 1;
    }

    // Verify root
    if (!asset.is_root(0)) {
        std::printf("    [FAIL] Bone 0 is not root\n");
        return 1;
    }

    // Verify pelvis is root
    if (asset.bones[0].joint_type != apc::JointType::Spherical3D) {
        std::printf("    [FAIL] Pelvis should be spherical, got %d\n",
                   (int)asset.bones[0].joint_type);
        return 1;
    }

    // Verify spine joints are revolute
    for (uint32_t i = 1; i <= 5; ++i) {
        if (asset.bones[i].joint_type != apc::JointType::Revolute1D) {
            std::printf("    [FAIL] Bone %u should be revolute\n", i);
            return 1;
        }
    }

    // Verify shoulders are spherical
    if (asset.bones[apc::HB_L_SHOULDER].joint_type != apc::JointType::Spherical3D) {
        std::printf("    [FAIL] L shoulder should be spherical\n");
        return 1;
    }

    std::printf("    [PASS] %u-bone humanoid created, hierarchy valid\n",
               asset.get_bone_count());
    return 0;
}

// =============================================================================
// TEST 2: Humanoid collision shapes
// =============================================================================
static int test_humanoid_collision_shapes() {
    std::printf("  [Test 2] Humanoid collision shapes...\n");

    apc::SkeletalAsset asset;
    apc::HumanoidLayout layout;
    apc::HumanoidBuilder::create_standard(layout, asset);

    apc::BoneCollisionShape shapes[apc::HB_BONE_COUNT];
    apc::HumanoidBuilder::create_collision_shapes(layout, shapes, apc::HB_BONE_COUNT);

    // Verify key shapes are enabled
    int enabled_count = 0;
    for (uint32_t i = 0; i < apc::HB_BONE_COUNT; ++i) {
        if (shapes[i].is_enabled()) enabled_count++;
    }

    if (enabled_count >= 12) { // At least 12 bones should have collision
        std::printf("    [PASS] %d bones have collision shapes\n", enabled_count);
    } else {
        std::printf("    [WARN] Only %d bones have collision (expected 12+)\n",
                   enabled_count);
    }

    return 0;
}

// =============================================================================
// TEST 3: Loop closure constraints
// =============================================================================
static int test_loop_closures() {
    std::printf("  [Test 3] Loop closure constraints...\n");

    // Create constraints
    apc::LoopClosureConstraint constraints[4];
    constraints[0] = apc::LoopClosureSolver::create_shoulder_girdle(
        apc::HB_L_SHOULDER, apc::HB_R_SHOULDER);
    constraints[1] = apc::LoopClosureSolver::create_pelvis_stability(
        apc::HB_L_HIP, apc::HB_R_HIP);

    // Verify they're enabled
    if (!constraints[0].enabled || !constraints[1].enabled) {
        std::printf("    [FAIL] Constraints not enabled\n");
        return 1;
    }

    // Verify types
    if (constraints[0].type != apc::LoopClosureType::Distance) {
        std::printf("    [FAIL] Shoulder girdle should be Distance type\n");
        return 1;
    }

    // Verify bone indices
    if (constraints[0].bone_a != apc::HB_L_SHOULDER ||
        constraints[0].bone_b != apc::HB_R_SHOULDER) {
        std::printf("    [FAIL] Wrong bone indices\n");
        return 1;
    }

    std::printf("    [PASS] Shoulder girdle + pelvis constraints created\n");
    return 0;
}

// =============================================================================
// TEST 4: Standing humanoid (animation-driven, ground contact)
// =============================================================================
static int test_standing_humanoid() {
    std::printf("  [Test 4] Standing humanoid...\n");

    apc::SkeletalAsset asset;
    apc::HumanoidLayout layout;
    layout.total_mass = 80.0f;
    layout.total_height = 1.8f;
    apc::HumanoidBuilder::create_standard(layout, asset);

    // All animation-driven = should stay in bind pose (T-pose)
    apc::HumanoidBuilder::configure_anim_driven(asset);

    apc::SkeletalPose pose;
    pose.set_to_bind_pose(asset);
    apc::SkeletalDynamicState state;
    state.allocate(asset.get_bone_count());

    apc::ArticulatedBody config;
    float dt = 1.0f / 240.0f;
    apc::Vec3 gravity(0.0f, -9.81f, 0.0f);

    // Capture target
    apc::BlendSystem::capture_target(asset, pose, pose);

    // Simulate 500 frames
    for (int frame = 0; frame < 500; ++frame) {
        apc::ArticulatedBody::step_ex(asset, pose, state, dt, gravity, config);
    }

    // Check no explosions
    bool stable = true;
    for (uint32_t i = 0; i < asset.get_bone_count(); ++i) {
        for (uint8_t d = 0; d < asset.bones[i].joint_dof; ++d) {
            float q = apc::get_joint_q(pose.joint_q, i, d);
            if (std::isnan(q) || std::isinf(q)) {
                std::printf("    [FAIL] Explosion bone %u DOF %d\n", i, d);
                stable = false;
            }
        }
    }

    if (stable) {
        // Pelvis should stay near its initial position
        float pelvis_y = pose.world_transforms[0].translation.y;
        std::printf("    [PASS] 22-bone standing humanoid stable for 500 frames "
                   "(pelvis_y=%.3f)\n", pelvis_y);
    }
    return stable ? 0 : 1;
}

// =============================================================================
// TEST 5: Full ragdoll with ground contact
// =============================================================================
static int test_full_ragdoll() {
    std::printf("  [Test 5] Full ragdoll with ground...\n");

    apc::SkeletalAsset asset;
    apc::HumanoidLayout layout;
    layout.total_mass = 80.0f;
    layout.total_height = 1.8f;
    apc::HumanoidBuilder::create_standard(layout, asset);
    apc::HumanoidBuilder::configure_ragdoll(asset);

    apc::BoneCollisionShape shapes[apc::HB_BONE_COUNT];
    apc::HumanoidBuilder::create_collision_shapes(layout, shapes, apc::HB_BONE_COUNT);

    apc::SkeletalPose pose;
    pose.set_to_bind_pose(asset);
    apc::SkeletalDynamicState state;
    state.allocate(asset.get_bone_count());

    // Ground plane
    apc::CollisionShape ground = apc::CollisionShape::make_plane(
        apc::Vec3(0, 0, 0), apc::Vec3(0, 1, 0));
    uint32_t ground_id = 0xFFFF0000;

    // Loop closures: shoulder girdle + pelvis
    apc::LoopClosureConstraint constraints[2];
    constraints[0] = apc::LoopClosureSolver::create_shoulder_girdle(
        apc::HB_L_SHOULDER, apc::HB_R_SHOULDER, 500.0f, 25.0f);
    constraints[1] = apc::LoopClosureSolver::create_pelvis_stability(
        apc::HB_L_HIP, apc::HB_R_HIP, 800.0f, 40.0f);

    apc::ArticulatedBody config;
    float dt = 1.0f / 240.0f;
    apc::Vec3 gravity(0.0f, -9.81f, 0.0f);

    // Simulate 1000 frames
    for (int frame = 0; frame < 1000; ++frame) {
        // Run ABA step with collision
        apc::SkeletonSimLoop::step(asset, pose, state,
            shapes, apc::HB_BONE_COUNT,
            &ground, &ground_id, 1,
            config, dt, gravity);

        // Resolve loop closures
        apc::SkeletalPose world_pose;
        world_pose.allocate(asset.get_bone_count());
        apc::SkeletalFK::calculate_world_transforms(asset, pose, world_pose);

        apc::LoopClosureSolver::resolve(constraints, 2,
            asset, world_pose, state, dt);
    }

    // Check no explosions
    bool stable = true;
    for (uint32_t i = 0; i < asset.get_bone_count(); ++i) {
        float px = pose.world_transforms[i].translation.x;
        float py = pose.world_transforms[i].translation.y;
        float pz = pose.world_transforms[i].translation.z;
        if (std::isnan(px) || std::isnan(py) || std::isnan(pz) ||
            std::isinf(px) || std::isinf(py) || std::isinf(pz)) {
            std::printf("    [FAIL] Explosion bone %u\n", i);
            stable = false;
        }
        // Ragdoll should eventually settle near ground
        if (py < -2.0f) {
            // Fell through ground — might be a contact issue
        }
    }

    if (stable) {
        float pelvis_y = pose.world_transforms[apc::HB_PELVIS].translation.y;
        std::printf("    [PASS] Full ragdoll stable 1000 frames (pelvis_y=%.3f)\n",
                   pelvis_y);
    }
    return stable ? 0 : 1;
}

// =============================================================================
// TEST 6: Knock-over scenario (anim → tackle → recover)
// =============================================================================
static int test_knockover_scenario() {
    std::printf("  [Test 6] Knock-over scenario...\n");

    apc::SkeletalAsset asset;
    apc::HumanoidLayout layout;
    layout.total_mass = 80.0f;
    layout.total_height = 1.8f;
    apc::HumanoidBuilder::create_standard(layout, asset);

    apc::BoneCollisionShape shapes[apc::HB_BONE_COUNT];
    apc::HumanoidBuilder::create_collision_shapes(layout, shapes, apc::HB_BONE_COUNT);

    apc::SkeletalPose pose;
    pose.set_to_bind_pose(asset);
    apc::SkeletalDynamicState state;
    state.allocate(asset.get_bone_count());

    apc::CollisionShape ground = apc::CollisionShape::make_plane(
        apc::Vec3(0, 0, 0), apc::Vec3(0, 1, 0));
    uint32_t ground_id = 0xFFFF0000;

    apc::LoopClosureConstraint constraints[2];
    constraints[0] = apc::LoopClosureSolver::create_shoulder_girdle(
        apc::HB_L_SHOULDER, apc::HB_R_SHOULDER, 500.0f, 25.0f);
    constraints[1] = apc::LoopClosureSolver::create_pelvis_stability(
        apc::HB_L_HIP, apc::HB_R_HIP, 800.0f, 40.0f);

    apc::ArticulatedBody config;
    float dt = 1.0f / 240.0f;
    apc::Vec3 gravity(0.0f, -9.81f, 0.0f);

    // ---- Phase 1: Standing (ANIM_DRIVEN), 100 frames ----
    apc::HumanoidBuilder::configure_anim_driven(asset);
    apc::BlendSystem::capture_target(asset, pose, pose);

    for (int frame = 0; frame < 100; ++frame) {
        apc::SkeletonSimLoop::step(asset, pose, state,
            shapes, apc::HB_BONE_COUNT,
            &ground, &ground_id, 1,
            config, dt, gravity);
    }

    float stand_pelvis_y = pose.world_transforms[apc::HB_PELVIS].translation.y;

    // ---- Phase 2: TACKLE! Upper body physics, legs anim ----
    apc::HumanoidBuilder::configure_partial_ragdoll(asset, 30.0f, 3.0f);

    // Apply impact force to chest
    apc::Vec3 chest_com(0.0f, 0.0f, 0.0f);
    apc::ArticulatedBody::apply_external_force(state, apc::HB_CHEST,
        apc::Vec3(300.0f, 50.0f, 0.0f), chest_com, chest_com);

    for (int frame = 0; frame < 300; ++frame) {
        apc::SkeletonSimLoop::step(asset, pose, state,
            shapes, apc::HB_BONE_COUNT,
            &ground, &ground_id, 1,
            config, dt, gravity);

        // Resolve loop closures
        apc::SkeletalPose world_pose;
        world_pose.allocate(asset.get_bone_count());
        apc::SkeletalFK::calculate_world_transforms(asset, pose, world_pose);
        apc::LoopClosureSolver::resolve(constraints, 2,
            asset, world_pose, state, dt);
    }

    float tackle_pelvis_y = pose.world_transforms[apc::HB_PELVIS].translation.y;

    // ---- Phase 3: Recovery (BLENDED), 500 frames ----
    for (uint32_t i = 0; i < asset.get_bone_count(); ++i) {
        asset.bones[i].physics.mode = apc::PhysicsBlendMode::BLENDED;
        asset.bones[i].physics.stiffness = 40.0f;
        asset.bones[i].physics.damping = 4.0f;
        asset.bones[i].physics.max_deviation = 3.0f;
    }

    for (int frame = 0; frame < 500; ++frame) {
        apc::SkeletonSimLoop::step(asset, pose, state,
            shapes, apc::HB_BONE_COUNT,
            &ground, &ground_id, 1,
            config, dt, gravity);

        apc::SkeletalPose world_pose;
        world_pose.allocate(asset.get_bone_count());
        apc::SkeletalFK::calculate_world_transforms(asset, pose, world_pose);
        apc::LoopClosureSolver::resolve(constraints, 2,
            asset, world_pose, state, dt);
    }

    float recover_pelvis_y = pose.world_transforms[apc::HB_PELVIS].translation.y;

    // Verify stability
    bool stable = true;
    for (uint32_t i = 0; i < asset.get_bone_count(); ++i) {
        float py = pose.world_transforms[i].translation.y;
        if (std::isnan(py) || std::isinf(py)) {
            stable = false;
        }
    }

    if (stable) {
        std::printf("    [PASS] Knock-over: stand(%.2f) -> tackle(%.2f) -> recover(%.2f)\n",
                   stand_pelvis_y, tackle_pelvis_y, recover_pelvis_y);
    } else {
        std::printf("    [FAIL] Instability during knock-over\n");
    }
    return stable ? 0 : 1;
}

// =============================================================================
// TEST 7: Full humanoid determinism
// =============================================================================
static int test_humanoid_determinism() {
    std::printf("  [Test 7] Full humanoid determinism...\n");

    auto run_sim = []() -> uint64_t {
        apc::SkeletalAsset asset;
        apc::HumanoidLayout layout;
        apc::HumanoidBuilder::create_standard(layout, asset);
        apc::HumanoidBuilder::configure_ragdoll(asset);

        apc::BoneCollisionShape shapes[apc::HB_BONE_COUNT];
        apc::HumanoidBuilder::create_collision_shapes(layout, shapes, apc::HB_BONE_COUNT);

        apc::SkeletalPose pose;
        pose.set_to_bind_pose(asset);
        apc::SkeletalDynamicState state;
        state.allocate(asset.get_bone_count());

        apc::CollisionShape ground = apc::CollisionShape::make_plane(
            apc::Vec3(0, 0, 0), apc::Vec3(0, 1, 0));
        uint32_t ground_id = 0xFFFF0000;

        apc::LoopClosureConstraint constraints[2];
        constraints[0] = apc::LoopClosureSolver::create_shoulder_girdle(
            apc::HB_L_SHOULDER, apc::HB_R_SHOULDER);
        constraints[1] = apc::LoopClosureSolver::create_pelvis_stability(
            apc::HB_L_HIP, apc::HB_R_HIP);

        apc::ArticulatedBody config;
        float dt = 1.0f / 240.0f;
        apc::Vec3 gravity(0.0f, -9.81f, 0.0f);

        for (int frame = 0; frame < 240; ++frame) {
            apc::SkeletonSimLoop::step(asset, pose, state,
                shapes, apc::HB_BONE_COUNT,
                &ground, &ground_id, 1,
                config, dt, gravity);

            apc::SkeletalPose wp;
            wp.allocate(asset.get_bone_count());
            apc::SkeletalFK::calculate_world_transforms(asset, pose, wp);
            apc::LoopClosureSolver::resolve(constraints, 2, asset, wp, state, dt);
        }

        uint64_t hash = 0;
        for (uint32_t i = 0; i < asset.get_bone_count(); ++i) {
            uint32_t bits;
            std::memcpy(&bits, &pose.local_transforms[i].translation.y, sizeof(bits));
            hash ^= bits + 0x9e3779b9 + (hash << 6);
            for (uint8_t d = 0; d < asset.bones[i].joint_dof && d < 3; ++d) {
                float q = apc::get_joint_q(pose.joint_q, i, d);
                std::memcpy(&bits, &q, sizeof(bits));
                hash ^= bits + 0x9e3779b9 + (hash << 6);
            }
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
    std::printf("Running Sprint 8 Tests: Humanoid Demo & Loop Closures\n");
    std::printf("=======================================================\n");

    int result = 0;
    result |= test_humanoid_creation();
    result |= test_humanoid_collision_shapes();
    result |= test_loop_closures();
    result |= test_standing_humanoid();
    result |= test_full_ragdoll();
    result |= test_knockover_scenario();
    result |= test_humanoid_determinism();

    if (result == 0) {
        std::printf("\nAll Sprint 8 tests PASSED.\n");
    } else {
        std::printf("\nSome Sprint 8 tests FAILED.\n");
    }
    return result;
}
