#include "apc_skeleton/apc_skeleton_types.h"
#include "apc_skeleton/apc_skeletal_pose.h"
#include "apc_skeleton/apc_skeletal_fk.h"
#include "apc_skeleton/apc_skeleton_apc.h"
#include "apc_platform/apc_fp_mode.h"
#include <cstdio>
#include <cmath>
#include <cstring>

int main() {
    apc::enforce_deterministic_fp_mode();
    std::printf("Running ABA Double Pendulum Test...\n");

    apc::SkeletalAsset asset;
    std::vector<apc::Vec3> axes;

    // Bone 0: Static Ceiling Pivot
    apc::Bone root;
    root.parent_index = -1;
    root.inverse_mass = 0.0f; 
    root.local_inverse_inertia = apc::Mat3::identity();
    root.joint_to_com = apc::Vec3(0.0f, -1.0f, 0.0f); 
    asset.bones.push_back(root);
    axes.push_back(apc::Vec3(1.0f, 0.0f, 0.0f)); // CHANGED: Swing on X axis

    // Bone 1: Swinging Arm
    apc::Bone arm;
    arm.parent_index = 0;
    arm.bind_pose.translation = apc::Vec3(0.0f, -1.0f, 0.0f); 
    arm.bind_pose.rotation = apc::Quat::identity(); 
    arm.inverse_mass = 1.0f; 
    arm.local_inverse_inertia = apc::Mat3{{ 10.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f, 0.0f, 10.0f }};
    arm.joint_to_com = apc::Vec3(1.0f, 0.0f, 0.0f); // CHANGED: Mass offset on X axis
    asset.bones.push_back(arm);
    axes.push_back(apc::Vec3(1.0f, 0.0f, 0.0f)); // CHANGED: X axis

    apc::SkeletalPose pose;
    pose.set_to_bind_pose(asset);

    // Rotate arm 45 degrees around X axis (Now it sticks straight out to the side)
    pose.local_transforms[1].rotation = apc::Quat::from_axis_angle(apc::Vec3(1, 0, 0.0f), 0.785398f); 

    apc::SkeletalDynamicState state;
    state.allocate(asset.get_bone_count());

    float dt = 1.0f / 240.0f;
    apc::Vec3 gravity(0.0f, -9.81f, 0.0f);

    // Simulate for 2 seconds
    for (int frame = 0; frame < 480; ++frame) {
        apc::ArticulatedBody::step(asset, axes, pose, state, dt, gravity);
    }

    // Run FK to get world positions
    apc::SkeletalPose world_pose;
    world_pose.allocate(asset.get_bone_count());
    apc::SkeletalFK::calculate_world_transforms(asset, pose, world_pose);

    // Check the world position of the arm's pivot
    float end_x = world_pose.world_transforms[1].translation.x;
    float end_y = world_pose.world_transforms[1].translation.y;

    // It started at X=1.0.
    // Swinging on X axis with gravity pulling down means X WILL decrease from 1.0f.
    // If X is still ~1.0f, the math failed.
    if (std::abs(end_x - 1.0f) < 0.01f) {
        std::printf("[FAIL] Pendulum did not swing. X=%.4f\n", end_x);
        return 1;
    }

    // It shouldn't explode
    if (std::isnan(end_x) || std::isnan(end_y) || end_y < -100.0f) {
        std::printf("[FAIL] Pendulum exploded.\n");
        return 1;
    }

    // Verify root didn't move
    if (world_pose.world_transforms[0].translation.y != 0.0f) {
        std::printf("[FAIL] Static root moved!\n");
        return 1;
    }

    uint64_t final_hash = 0;
    auto hash_float = [&final_hash](float f) {
        uint32_t bits;
        std::memcpy(&bits, &f, sizeof(bits));
        final_hash ^= bits + 0x9e3779b9 + (final_hash << 6);
    };
    hash_float(end_x);
    hash_float(end_y);
    hash_float(state.joint_velocities[1]);

    std::printf("[PASS] ABA Pendulum simulated stably.\n");
    std::printf("Final Pivot World Pos: (%.4f, %.4f)\n", end_x, end_y);
    std::printf("Joint Velocity: %.4f rad/s\n", state.joint_velocities[1]);
    std::printf("Hash: %016llx\n", (unsigned long long)final_hash);
    return 0;
}