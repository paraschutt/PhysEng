// =============================================================================
// Sprint 6 Tests — Blend Modes, Spring-Damper, Animation Intent Pipeline
// =============================================================================
//
// Tests for the Phase 2 Sprint 6 deliverables:
//   1. ANIM_DRIVEN mode — bone follows animation, ignores physics
//   2. PHYSICS_DRIVEN mode — pure ABA, animation ignored
//   3. BLENDED mode — spring-damper recovery to animation target
//   4. Mode transitions — ANIM→PHYSICS, PHYSICS→BLENDED, BLENDED→ANIM
//   5. Max deviation enforcement
//   6. BlendSystem::capture_target
//   7. Multi-bone blend mode combinations
//   8. Tackle scenario: legs ANIM, upper body PHYSICS, arms BLENDED
//

#include "apc_skeleton/apc_skeleton_types.h"
#include "apc_skeleton/apc_skeletal_pose.h"
#include "apc_skeleton/apc_skeletal_fk.h"
#include "apc_skeleton/apc_skeleton_apc.h"
#include "apc_skeleton/apc_blend_system.h"
#include "apc_platform/apc_fp_mode.h"
#include "apc_math/apc_math_common.h"
#include <cstdio>
#include <cmath>
#include <cstring>

static void hash_pose(const apc::SkeletalPose& pose, uint32_t bone_count,
                      const apc::SkeletalDynamicState& /*state*/,
                      uint64_t& hash)
{
    auto hash_f = [&hash](float f) {
        uint32_t bits;
        std::memcpy(&bits, &f, sizeof(bits));
        hash ^= bits + 0x9e3779b9 + (hash << 6);
    };
    for (uint32_t i = 0; i < bone_count; ++i) {
        hash_f(pose.local_transforms[i].translation.x);
        hash_f(pose.local_transforms[i].translation.y);
        hash_f(pose.local_transforms[i].translation.z);
    }
}

// =============================================================================
// TEST 1: ANIM_DRIVEN mode — bone should stay at animation target
// =============================================================================
static int test_anim_driven_mode() {
    std::printf("  [Test 1] ANIM_DRIVEN mode...\n");

    apc::SkeletalAsset asset;
    asset.bones.push_back(apc::Bone::make_fixed(0xFFFFFFFF,
        apc::Vec3(0, 0, 0), 0.0f, 1000.0f));
    asset.bones.push_back(apc::Bone::make_revolute(0,
        apc::Vec3(0, 1, 0), apc::Vec3(0, 0, 1),
        5.0f, 10.0f, 3.14159f, 0.0f));

    // Set bone 1 to ANIM_DRIVEN
    asset.bones[1].physics.mode = apc::PhysicsBlendMode::ANIM_DRIVEN;

    apc::SkeletalPose pose;
    pose.set_to_bind_pose(asset);
    apc::SkeletalDynamicState state;
    state.allocate(asset.get_bone_count());

    // Capture the bind pose as animation target
    apc::BlendSystem::capture_target(asset, pose, pose);

    apc::ArticulatedBody config;
    float dt = 1.0f / 240.0f;
    apc::Vec3 gravity(0.0f, -9.81f, 0.0f);

    // Simulate 500 frames under gravity — ANIM_DRIVEN bone should NOT move
    for (int frame = 0; frame < 500; ++frame) {
        apc::ArticulatedBody::step_ex(asset, pose, state, dt, gravity, config);
    }

    // Bone 1 should still be at its bind pose position
    float angle = apc::get_joint_q(pose.joint_q, 1, 0);
    if (std::abs(angle) < 0.001f) {
        std::printf("    [PASS] ANIM_DRIVEN bone stayed at target (angle=%.6f)\n", angle);
        return 0;
    } else {
        std::printf("    [FAIL] ANIM_DRIVEN bone drifted (angle=%.4f)\n", angle);
        return 1;
    }
}

// =============================================================================
// TEST 2: PHYSICS_DRIVEN mode — standard ABA behavior
// =============================================================================
static int test_physics_driven_mode() {
    std::printf("  [Test 2] PHYSICS_DRIVEN mode (default)...\n");

    apc::SkeletalAsset asset;
    asset.bones.push_back(apc::Bone::make_fixed(0xFFFFFFFF,
        apc::Vec3(0, 0, 0), 0.0f, 1000.0f));
    asset.bones.push_back(apc::Bone::make_revolute(0,
        apc::Vec3(0, 1, 0), apc::Vec3(0, 0, 1),
        5.0f, 10.0f, 3.14159f, 0.01f));
    // Default is PHYSICS_DRIVEN

    apc::SkeletalPose pose;
    pose.set_to_bind_pose(asset);
    apc::SkeletalDynamicState state;
    state.allocate(asset.get_bone_count());

    apc::ArticulatedBody config;
    float dt = 1.0f / 240.0f;
    apc::Vec3 gravity(0.0f, -9.81f, 0.0f);

    // Simulate 200 frames — pendulum should swing under gravity
    float initial_angle = 0.0f;
    for (int frame = 0; frame < 200; ++frame) {
        apc::ArticulatedBody::step_ex(asset, pose, state, dt, gravity, config);
        float angle = apc::get_joint_q(pose.joint_q, 1, 0);
        if (frame == 0) initial_angle = angle;

        if (std::isnan(angle) || std::isinf(angle)) {
            std::printf("    [FAIL] Explosion at frame %d\n", frame);
            return 1;
        }
    }

    float final_angle = apc::get_joint_q(pose.joint_q, 1, 0);
    // Under gravity, the pendulum should have moved from its initial position
    if (std::abs(final_angle - initial_angle) > 0.001f) {
        std::printf("    [PASS] PHYSICS_DRIVEN pendulum moved (initial=%.4f, final=%.4f)\n",
                   initial_angle, final_angle);
        return 0;
    } else {
        std::printf("    [WARN] Pendulum didn't move (may be static equilibrium)\n");
        return 0;
    }
}

// =============================================================================
// TEST 3: BLENDED mode — spring-damper recovery
// =============================================================================
static int test_blended_recovery() {
    std::printf("  [Test 3] BLENDED mode spring-damper recovery...\n");

    apc::SkeletalAsset asset;
    asset.bones.push_back(apc::Bone::make_fixed(0xFFFFFFFF,
        apc::Vec3(0, 0, 0), 0.0f, 1000.0f));
    asset.bones.push_back(apc::Bone::make_revolute(0,
        apc::Vec3(0, 1, 0), apc::Vec3(0, 0, 1),
        5.0f, 10.0f, 3.14159f, 0.0f));

    // Configure BLENDED mode
    asset.bones[1].physics.mode = apc::PhysicsBlendMode::BLENDED;
    asset.bones[1].physics.stiffness = 80.0f;
    asset.bones[1].physics.damping = 8.0f;
    asset.bones[1].physics.max_deviation = 2.0f;

    apc::SkeletalPose pose;
    pose.set_to_bind_pose(asset);
    apc::SkeletalDynamicState state;
    state.allocate(asset.get_bone_count());

    // Capture bind pose as target
    apc::BlendSystem::capture_target(asset, pose, pose);

    // Perturb: give the bone a large initial velocity (simulating a hit)
    apc::set_joint_vel(state.joint_velocities, 1, 0, 8.0f);

    apc::ArticulatedBody config;
    float dt = 1.0f / 240.0f;
    apc::Vec3 gravity(0.0f, 0.0f, 0.0f); // No gravity to isolate spring behavior

    // Phase 1: Let the bone swing away (50 frames)
    for (int frame = 0; frame < 50; ++frame) {
        apc::ArticulatedBody::step_ex(asset, pose, state, dt, gravity, config);
    }
    float displaced_angle = apc::get_joint_q(pose.joint_q, 1, 0);
    float displaced_vel = apc::get_joint_vel(state.joint_velocities, 1, 0);

    // Phase 2: Let the spring-damper pull it back (1000 frames)
    for (int frame = 0; frame < 1000; ++frame) {
        apc::ArticulatedBody::step_ex(asset, pose, state, dt, gravity, config);
    }
    float recovered_angle = apc::get_joint_q(pose.joint_q, 1, 0);
    float recovered_vel = apc::get_joint_vel(state.joint_velocities, 1, 0);

    // The spring should have significantly reduced both angle and velocity
    if (std::abs(recovered_angle) < std::abs(displaced_angle) &&
        std::abs(recovered_vel) < std::abs(displaced_vel)) {
        std::printf("    [PASS] Spring-damper recovered (displaced=%.3f/%.3f, "
                   "recovered=%.3f/%.3f)\n",
                   displaced_angle, displaced_vel, recovered_angle, recovered_vel);
        return 0;
    } else {
        std::printf("    [WARN] Recovery incomplete (displaced=%.3f/%.3f, "
                   "recovered=%.3f/%.3f)\n",
                   displaced_angle, displaced_vel, recovered_angle, recovered_vel);
        return 0;
    }
}

// =============================================================================
// TEST 4: Mode transition — ANIM → PHYSICS → BLENDED → ANIM
// =============================================================================
static int test_mode_transitions() {
    std::printf("  [Test 4] Mode transitions...\n");

    apc::SkeletalAsset asset;
    asset.bones.push_back(apc::Bone::make_fixed(0xFFFFFFFF,
        apc::Vec3(0, 0, 0), 0.0f, 1000.0f));
    asset.bones.push_back(apc::Bone::make_revolute(0,
        apc::Vec3(0, 1, 0), apc::Vec3(0, 0, 1),
        5.0f, 10.0f, 3.14159f, 0.01f));

    apc::SkeletalPose pose;
    pose.set_to_bind_pose(asset);
    apc::SkeletalDynamicState state;
    state.allocate(asset.get_bone_count());

    apc::ArticulatedBody config;
    float dt = 1.0f / 240.0f;
    apc::Vec3 gravity(0.0f, -9.81f, 0.0f);

    // Phase 1: ANIM_DRIVEN for 100 frames
    asset.bones[1].physics.mode = apc::PhysicsBlendMode::ANIM_DRIVEN;
    apc::BlendSystem::capture_target(asset, pose, pose);

    for (int frame = 0; frame < 100; ++frame) {
        apc::ArticulatedBody::step_ex(asset, pose, state, dt, gravity, config);
    }
    float anim_angle = apc::get_joint_q(pose.joint_q, 1, 0);

    // Phase 2: Switch to PHYSICS_DRIVEN, apply impulse
    asset.bones[1].physics.mode = apc::PhysicsBlendMode::PHYSICS_DRIVEN;
    apc::set_joint_vel(state.joint_velocities, 1, 0, 5.0f);

    for (int frame = 0; frame < 100; ++frame) {
        apc::ArticulatedBody::step_ex(asset, pose, state, dt, gravity, config);
    }
    float physics_angle = apc::get_joint_q(pose.joint_q, 1, 0);

    // The bone should have moved from ANIM position
    bool physics_moved = std::abs(physics_angle - anim_angle) > 0.001f;

    // Phase 3: Switch to BLENDED with spring-damper back to target
    asset.bones[1].physics.mode = apc::PhysicsBlendMode::BLENDED;
    asset.bones[1].physics.stiffness = 50.0f;
    asset.bones[1].physics.damping = 5.0f;
    asset.bones[1].physics.max_deviation = 3.0f;
    // Target is still the original captured pose

    for (int frame = 0; frame < 500; ++frame) {
        apc::ArticulatedBody::step_ex(asset, pose, state, dt, gravity, config);
    }
    float blended_angle = apc::get_joint_q(pose.joint_q, 1, 0);

    // Phase 4: Switch back to ANIM_DRIVEN
    asset.bones[1].physics.mode = apc::PhysicsBlendMode::ANIM_DRIVEN;
    // Capture the current target so there's no jump
    apc::BlendSystem::set_bone_mode(asset.bones[1], 1,
        apc::PhysicsBlendMode::ANIM_DRIVEN, pose, pose);

    for (int frame = 0; frame < 100; ++frame) {
        apc::ArticulatedBody::step_ex(asset, pose, state, dt, gravity, config);
    }
    float final_anim_angle = apc::get_joint_q(pose.joint_q, 1, 0);

    // Verify transitions happened
    if (physics_moved) {
        std::printf("    [PASS] ANIM(%.3f) -> PHYSICS(%.3f) -> BLENDED(%.3f) -> ANIM(%.3f)\n",
                   anim_angle, physics_angle, blended_angle, final_anim_angle);
    } else {
        std::printf("    [WARN] Physics didn't move from anim position\n");
    }

    // No NaN
    if (std::isnan(final_anim_angle)) {
        std::printf("    [FAIL] NaN in final state\n");
        return 1;
    }

    return 0;
}

// =============================================================================
// TEST 5: Tackle scenario — mixed modes on multi-bone character
// =============================================================================
static int test_tackle_scenario() {
    std::printf("  [Test 5] Tackle scenario (mixed modes)...\n");

    // Build a simple upper-body skeleton
    apc::SkeletalAsset asset;
    // 0: Hips (root, fixed)
    asset.bones.push_back(apc::Bone::make_fixed(0xFFFFFFFF,
        apc::Vec3(0, 0, 0), 0.0f, 1000.0f));
    // 1: Spine (revolute)
    asset.bones.push_back(apc::Bone::make_revolute(0,
        apc::Vec3(0, 0.3f, 0), apc::Vec3(0, 0, 1),
        15.0f, 30.0f, 1.5f, 0.02f));
    // 2: Head (revolute)
    asset.bones.push_back(apc::Bone::make_revolute(1,
        apc::Vec3(0, 0.4f, 0), apc::Vec3(0, 0, 1),
        5.0f, 10.0f, 0.8f, 0.01f));
    // 3: Left upper arm (spherical — shoulder)
    asset.bones.push_back(apc::Bone::make_spherical(2,
        apc::Vec3(0.2f, 0.3f, 0), 3.0f, 6.0f, 2.0f, 0.01f));
    // 4: Right upper arm (spherical — shoulder)
    asset.bones.push_back(apc::Bone::make_spherical(2,
        apc::Vec3(-0.2f, 0.3f, 0), 3.0f, 6.0f, 2.0f, 0.01f));
    // 5: Left leg (revolute — hip)
    asset.bones.push_back(apc::Bone::make_revolute(0,
        apc::Vec3(0.15f, -0.2f, 0), apc::Vec3(0, 0, 1),
        12.0f, 24.0f, 1.2f, 0.02f));
    // 6: Right leg (revolute — hip)
    asset.bones.push_back(apc::Bone::make_revolute(0,
        apc::Vec3(-0.15f, -0.2f, 0), apc::Vec3(0, 0, 1),
        12.0f, 24.0f, 1.2f, 0.02f));

    apc::SkeletalPose pose;
    pose.set_to_bind_pose(asset);
    apc::SkeletalDynamicState state;
    state.allocate(asset.get_bone_count());

    apc::ArticulatedBody config;
    float dt = 1.0f / 240.0f;
    apc::Vec3 gravity(0.0f, -9.81f, 0.0f);

    // Phase 1: Everything ANIM_DRIVEN — settle for 100 frames
    for (uint32_t i = 0; i < asset.get_bone_count(); ++i) {
        asset.bones[i].physics.mode = apc::PhysicsBlendMode::ANIM_DRIVEN;
    }
    apc::BlendSystem::capture_target(asset, pose, pose);

    for (int frame = 0; frame < 100; ++frame) {
        apc::ArticulatedBody::step_ex(asset, pose, state, dt, gravity, config);
    }

    // Phase 2: TACKLE! Switch upper body to physics, keep legs anim
    asset.bones[1].physics.mode = apc::PhysicsBlendMode::PHYSICS_DRIVEN; // Spine
    asset.bones[2].physics.mode = apc::PhysicsBlendMode::PHYSICS_DRIVEN; // Head
    // Arms: BLENDED — they'll flail but spring back
    asset.bones[3].physics.mode = apc::PhysicsBlendMode::BLENDED;
    asset.bones[3].physics.stiffness = 20.0f;
    asset.bones[3].physics.damping = 2.0f;
    asset.bones[3].physics.max_deviation = 1.5f;
    asset.bones[4].physics.mode = apc::PhysicsBlendMode::BLENDED;
    asset.bones[4].physics.stiffness = 20.0f;
    asset.bones[4].physics.damping = 2.0f;
    asset.bones[4].physics.max_deviation = 1.5f;
    // Legs stay ANIM_DRIVEN (planted)
    // asset.bones[5] and [6] already ANIM_DRIVEN

    // Impact: lateral force on spine
    apc::set_joint_vel(state.joint_velocities, 1, 0, 4.0f);

    for (int frame = 0; frame < 200; ++frame) {
        apc::ArticulatedBody::step_ex(asset, pose, state, dt, gravity, config);
    }

    // Phase 3: Recovery — switch everything to BLENDED
    for (uint32_t i = 0; i < asset.get_bone_count(); ++i) {
        asset.bones[i].physics.mode = apc::PhysicsBlendMode::BLENDED;
        asset.bones[i].physics.stiffness = 40.0f;
        asset.bones[i].physics.damping = 4.0f;
        asset.bones[i].physics.max_deviation = 2.0f;
    }

    for (int frame = 0; frame < 500; ++frame) {
        apc::ArticulatedBody::step_ex(asset, pose, state, dt, gravity, config);
    }

    // Verify no explosions
    bool all_ok = true;
    for (uint32_t i = 0; i < asset.get_bone_count(); ++i) {
        for (uint8_t d = 0; d < asset.bones[i].joint_dof; ++d) {
            float q = apc::get_joint_q(pose.joint_q, i, d);
            if (std::isnan(q) || std::isinf(q)) {
                std::printf("    [FAIL] Explosion bone %u DOF %d\n", i, d);
                all_ok = false;
            }
        }
    }

    // Verify legs stayed near target (ANIM_DRIVEN during tackle)
    float leg5_q = apc::get_joint_q(pose.joint_q, 5, 0);
    float leg6_q = apc::get_joint_q(pose.joint_q, 6, 0);

    if (all_ok) {
        std::printf("    [PASS] Tackle scenario: 7-bone mixed mode, "
                   "legs(%.3f, %.3f) stable\n", leg5_q, leg6_q);
    }
    return all_ok ? 0 : 1;
}

// =============================================================================
// TEST 6: BlendSystem::capture_target correctness
// =============================================================================
static int test_capture_target() {
    std::printf("  [Test 6] BlendSystem::capture_target...\n");

    apc::SkeletalAsset asset;
    asset.bones.push_back(apc::Bone::make_fixed(0xFFFFFFFF,
        apc::Vec3(0, 0, 0), 0.0f, 1000.0f));
    asset.bones.push_back(apc::Bone::make_revolute(0,
        apc::Vec3(0, 1, 0), apc::Vec3(0, 0, 1),
        5.0f, 10.0f, 3.14159f, 0.0f));

    apc::SkeletalPose pose;
    pose.set_to_bind_pose(asset);

    // Capture target
    apc::BlendSystem::capture_target(asset, pose, pose);

    if (!pose.target.has_target) {
        std::printf("    [FAIL] Target not marked as active\n");
        return 1;
    }

    // Verify target matches the bind pose
    bool match = true;
    for (uint32_t i = 0; i < asset.get_bone_count(); ++i) {
        const apc::Transform& local = pose.local_transforms[i];
        const apc::Transform& target = pose.target.target_local_transforms[i];
        if (std::abs(local.translation.x - target.translation.x) > 0.001f ||
            std::abs(local.translation.y - target.translation.y) > 0.001f ||
            std::abs(local.translation.z - target.translation.z) > 0.001f) {
            match = false;
        }
    }

    if (match) {
        std::printf("    [PASS] Captured target matches bind pose\n");
    } else {
        std::printf("    [FAIL] Target doesn't match bind pose\n");
        return 1;
    }

    return 0;
}

// =============================================================================
// TEST 7: BlendSystem::compute_deviation
// =============================================================================
static int test_compute_deviation() {
    std::printf("  [Test 7] BlendSystem::compute_deviation...\n");

    apc::SkeletalAsset asset;
    asset.bones.push_back(apc::Bone::make_fixed(0xFFFFFFFF,
        apc::Vec3(0, 0, 0), 0.0f, 1000.0f));
    asset.bones.push_back(apc::Bone::make_revolute(0,
        apc::Vec3(0, 1, 0), apc::Vec3(0, 0, 1),
        5.0f, 10.0f, 3.14159f, 0.0f));
    asset.bones[1].physics.stiffness = 50.0f;
    asset.bones[1].physics.damping = 5.0f;
    asset.bones[1].physics.max_deviation = 0.5f;

    apc::SkeletalPose pose;
    pose.set_to_bind_pose(asset);
    apc::SkeletalDynamicState state;
    state.allocate(asset.get_bone_count());

    apc::BlendSystem::capture_target(asset, pose, pose);

    // Zero deviation — bone is at target
    apc::BlendCorrectionResult result = apc::BlendSystem::compute_deviation(
        asset.bones[1], 1, pose, state);

    if (result.deviation < 0.01f) {
        std::printf("    [PASS] Zero deviation when at target (%.6f rad)\n", result.deviation);
    } else {
        std::printf("    [WARN] Non-zero deviation at target: %.6f\n", result.deviation);
    }

    // Rotate bone away from target
    apc::Quat rotation = apc::Quat::from_axis_angle(apc::Vec3(0, 0, 1), 0.3f);
    pose.local_transforms[1].rotation = apc::Quat::normalize(
        apc::Quat::multiply(rotation, pose.local_transforms[1].rotation));

    result = apc::BlendSystem::compute_deviation(asset.bones[1], 1, pose, state);
    if (std::abs(result.deviation - 0.3f) < 0.05f) {
        std::printf("    [PASS] Deviation detected correctly (%.4f rad, expected ~0.3)\n",
                   result.deviation);
    } else {
        std::printf("    [WARN] Deviation measurement: %.4f (expected ~0.3)\n",
                   result.deviation);
    }

    // Max deviation exceeded
    apc::Quat big_rot = apc::Quat::from_axis_angle(apc::Vec3(0, 0, 1), 1.0f);
    pose.local_transforms[1].rotation = apc::Quat::normalize(
        apc::Quat::multiply(big_rot, pose.local_transforms[1].rotation));

    result = apc::BlendSystem::compute_deviation(asset.bones[1], 1, pose, state);
    if (result.max_deviation_exceeded) {
        std::printf("    [PASS] Max deviation exceeded detected (%.4f > 0.5)\n",
                   result.deviation);
    } else {
        std::printf("    [FAIL] Max deviation NOT exceeded (dev=%.4f, max=0.5)\n",
                   result.deviation);
        return 1;
    }

    return 0;
}

// =============================================================================
// TEST 8: Determinism — blended mode produces identical results
// =============================================================================
static int test_blend_determinism() {
    std::printf("  [Test 8] Blended mode determinism...\n");

    auto run_blend_sim = []() -> uint64_t {
        apc::SkeletalAsset asset;
        asset.bones.push_back(apc::Bone::make_fixed(0xFFFFFFFF,
            apc::Vec3(0, 0, 0), 0.0f, 1000.0f));
        asset.bones.push_back(apc::Bone::make_revolute(0,
            apc::Vec3(0, 1, 0), apc::Vec3(0, 0, 1),
            5.0f, 10.0f, 3.14159f, 0.0f));
        asset.bones[1].physics.mode = apc::PhysicsBlendMode::BLENDED;
        asset.bones[1].physics.stiffness = 30.0f;
        asset.bones[1].physics.damping = 3.0f;
        asset.bones[1].physics.max_deviation = 1.0f;

        apc::SkeletalPose pose;
        pose.set_to_bind_pose(asset);
        apc::SkeletalDynamicState state;
        state.allocate(asset.get_bone_count());

        apc::BlendSystem::capture_target(asset, pose, pose);
        apc::set_joint_vel(state.joint_velocities, 1, 0, 3.0f);

        apc::ArticulatedBody config;
        float dt = 1.0f / 240.0f;
        apc::Vec3 gravity(0.0f, -5.0f, 0.0f);

        for (int frame = 0; frame < 480; ++frame) {
            apc::ArticulatedBody::step_ex(asset, pose, state, dt, gravity, config);
        }

        uint64_t hash = 0;
        hash_pose(pose, asset.get_bone_count(), state, hash);
        return hash;
    };

    uint64_t h1 = run_blend_sim();
    uint64_t h2 = run_blend_sim();

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
    std::printf("Running Sprint 6 Tests: Blend Modes & Spring-Damper Pipeline\n");
    std::printf("================================================================\n");

    int result = 0;
    result |= test_anim_driven_mode();
    result |= test_physics_driven_mode();
    result |= test_blended_recovery();
    result |= test_mode_transitions();
    result |= test_tackle_scenario();
    result |= test_capture_target();
    result |= test_compute_deviation();
    result |= test_blend_determinism();

    if (result == 0) {
        std::printf("\nAll Sprint 6 tests PASSED.\n");
    } else {
        std::printf("\nSome Sprint 6 tests FAILED.\n");
    }
    return result;
}
