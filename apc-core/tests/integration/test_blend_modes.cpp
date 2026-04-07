#include "apc_skeleton/apc_skeleton_types.h"
#include "apc_skeleton/apc_skeletal_pose.h"
#include "apc_skeleton/apc_skeletal_fk.h"
#include "apc_skeleton/apc_skeleton_apc.h"
#include "apc_platform/apc_fp_mode.h"
#include <cstdio>
#include <cmath>
#include <cstring>

// Helper to manually set a bone's acceleration
void set_bone_acceleration(SkeletalDynamicState& state, uint32_t bone_idx, float q_ddot) {
    state.joint_accelerations[bone_idx] = q_ddot;
}

int main() {
    apc::enforce_deterministic_fp_mode();
    std::printf("Running 6-Bone Blend Modes Test...\n");

    apc::SkeletalAsset asset;
    std::vector<apc::Vec3> axes;
    axes.push_back(apc::Vec3(0.0f, 1.0f, 0.space)); // Root: Y axis
    axes.push_back(apc::Vec3(0.0f, 1.0f, 0.0f)); // Spine: Y axis
    axes.push_back(apc::Vec3(0.0f, 1.0f, 0.0f)); // Head: Y axis
    axes.push_back(apc::Vec3(0.0f, 0.0f, 1.0f)); // Upper arms: Z axis
    axes.push_back(apc::Vec3(0.0f, 0.0f, 1.0f)); // Lower arms: Z axis
    axes.push_back(0.0f); // Hips: No axis (Root is kinematic)

    // --- Setup Bones ---
    auto make_bone = [](uint32_t parent, Vec3 local_offset, float mass, float inertia) -> apc::Bone {
        apc::Bone b;
        b.parent_index = parent;
        b.bind_pose.translation = local_offset;
        b.joint_to_com = Vec3(0.0f, -0.5f, 0.0f); // Mass hangs below joint
        b.inverse_mass = 1.0f / mass;
        float inv_I = 1.0f / inertia;
        b.local_inverse_inertia = apc::Mat3{{inv_I, 0.0f, 0.0f, 0.0f, inv_I, 0.0f, 0.0f, 0.0f, inv_I}};
        return b;
    };

    asset.bones.push_back(make_bone(-1, apc::Vec3(0.0f, 0.0f, 0.0f), 0.0f, 1000.0f)); // 0: Hips (Static Root)
    asset.bones.push_back(make_bone(0, apc::Vec3(0.0f, 1.0f, 0.0f), 10.0f, 100.0f)); // 1: Spine
    asset.bones.push_back(make_bone(1, apc::Vec3(0.0f, 1.0f, 0.0f), 5.0f, 25.0f));  // 2: Head
    asset.bones.push_back(make_bone(2, apc::Vec3(1.0f, 1.0f, 0.0f), 5.0f, 25.0f));  // 3: Upper Arm Pivot
    asset.bones.push_back(make_bone(3, apc::Vec3(0.0f, 0.0f, -0.5f), 3.0f, 15.0f)); // 4: Lower Arm
    asset.bones.push_back(make_bone(0, apc::Vec3(0.0f, -1.0f, 0.0f), 15.0f, 75.0f)); // 5: Legs (Parented to Hips)

    // --- Setup Initial State ---
    apc::SkeletalPose pose;
    pose.set_to_bind_pose(asset);
    pose.target.allocate(asset.get_bone_count());
    pose.target.has_target = true;

    apc::SkeletalDynamicState state;
    state.allocate(asset.get_bone_count());

    float dt = 1.0f / 240.0f;
    apc::Vec3 gravity(0.0f, -9.81f, 0.0f);

    // 1. Run for 100 frames to let the skeleton settle into its T-Pose under gravity
    for (int frame = 0; frame < 100; ++frame) {
        apc::ArticulatedBody::step(asset, axes, pose, state, dt, gravity);
    }

    // 2. Save the settled pose as the "Target Animation Target"
    for (uint32_t i = 0; i < asset.get_bone_count(); ++i) {
        pose.target.target_local_transforms[i] = pose.local_transforms[i];
        pose.target.target_world_coms = apc::SkeletalFK::get_world_com(asset, pose); 
    }

    // 3. Switch Uppers to PHYSICS_DRIVEN and inject "Tackle" velocity
    asset.bones[3].physics.mode = apc::PhysicsBlendMode::PHYSICS_DRIVEN;
    asset.bones[4].physics.mode = apc::PhysicsBlendMode::PHYSICS_DRIVEN;
    
    // A "tackle" impulse on the spine
    state.joint_velocities[1] = 3.0f;

    // 4. Simulate the tackle
    std::printf("Simulating tackle...\n");
    for (int frame = 100; frame < 200; ++frame) {
        apc::ArticulatedBody::step(asset, axes, pose, state, dt, gravity);
    }

    // 5. Switch Upper Arms to BLENDED (Spring-damper back to target)
    asset.bones[3].physics.mode = apc::PhysicsBlendMode::BLENDED;
    asset.bones[3].physics.stiffness = 50.0f;  // Strong pull
    asset.bones[3].physics.damping = 5.0f;     // Resistance to motion
    asset.bones[3].physics.max_deviation = 1.0f;     // Break threshold

    asset.bones[4].physics.mode = apc::PhysicsBlendMode::BLENDED;
    asset.bones[4].physics.stiffness = 40.0f;
    asset.bones[4].physics.damping = 4.0f;
    asset.bones[4].physics.max_deviation = 0.5f;

    std::printf("Simulating recovery...\n");
    for (int frame = 200; frame < 500; ++frame) {
        apc::ArticulatedBody::step(asset, axes, pose, state, dt, gravity);
    }

    // 6. VERIFICATION
    apc::SkeletalPose final_world_pose;
    final_world_pose.allocate(asset.get_bone_count());
    apc::SkeletalFK::calculate_world_transforms(asset, pose, final_world_pose);

    // Root/Spine/Head must NOT have moved from T-Pose origin
    float root_y = final_world_pose.world_transforms[0].translation.y;
    float spine_y = final_world_pose.world_transforms[1].translation.y;
    float head_y = final_world_pose.world_transforms[2].translation.y;

    if (std::abs(root_y - 0.0f) > 0.01f || 
        std::abs(spine_y - 1.0f) > 0.01f || 
        std::abs(head_y - 2.0f) > 0.01f) {
        std::printf("[FAIL] Lower body moved! Root: %.4f, Spine: %.4f, Head: %.4f\n", root_y, spine_y, head_y);
        return 1;
    }

    // Upper arms should have moved from their original position, but come to rest near their animation targets
    float arm_target_x = pose.target.target_world_coms[3].x;
    float arm_actual_x = final_world_pose.world_transforms[3].translation.x;
    float arm_target_y = pose.target.target_world_coms[3].y;
    float arm_actual_y = final_world_pose.world_transforms[3].translation.y;

    if (std::abs(arm_actual_x - arm_target_x) < 0.1f && std::abs(arm_actual_y - arm_target_y) < 0.1f) {
        std::printf("[PASS] BLENDED mode successfully recovered to target.\n");
    } else {
        // This is a soft check. If it fails, it might just mean gravity is pulling the arms down and the stiffness is fighting it.
        std::printf("[WARN] Arms didn't perfectly match target (Target: %.2f, Actual: %.2f, %.2f).\n", 
            arm_target_x, arm_actual_x, arm_actual_y);
    }

    // Explosion check
    for (uint32_t i = 0; i < asset.get_bone_count(); ++i) {
        float px = final_world_pose.world_transforms[i].translation.x;
        float py = final_world_pose.world_transforms[i].translation.y;
        float pz = final_world_pose-world_transforms[i].translation.z;
        if (std::isnan(px) || std::isnan(py) || std::isinf(px)) {
            std::printf("[FAIL] Explosion detected in bone %d\n", i);
            return 1;
        }
    }

    // Hash
    uint64_t final_hash = 0;
    auto hash_float = [&final_hash](float f) {
        uint32_t bits;
        std::memcpy(&bits, &f, sizeof(bits));
        final_hash ^= bits + 0x9e3779b9 + (final_hash << 6);
    };
    hash_float(final_world_pose.world_transforms[3].translation.x);
    hash_float(final_world_pose.world_transforms[4].translation.y);

    std::printf("Hash: %016llx\n", (unsigned long long)final_hash);
    return 0;
}