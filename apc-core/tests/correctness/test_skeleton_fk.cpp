#include "apc_skeleton/apc_skeleton_types.h"
#include "apc_skeleton/apc_skeletal_pose.h"
#include "apc_skeleton/apc_skeletal_fk.h"
#include "apc_math/apc_math_common.h"
#include <cassert>
#include <cmath>

int main() {
    apc::SkeletalAsset asset;
    
    // Bone 0: Root (at origin)
    apc::Bone root;
    root.parent_index = -1;
    root.bind_pose = apc::Transform::identity();
    root.joint_to_com = apc::Vec3(0, 0.5f, 0); 
    asset.bones.push_back(root);

    // Bone 1: Child (offset 1.0 up in local space)
    apc::Bone child;
    child.parent_index = 0;
    child.bind_pose.translation = apc::Vec3(0.0f, 1.0f, 0.0f);
    child.bind_pose.rotation = apc::Quat::identity();
    child.joint_to_com = apc::Vec3(0, 0.5f, 0);
    asset.bones.push_back(child);

    // Bone 2: Grandchild (offset 1.0 up in local space)
    apc::Bone grandchild;
    grandchild.parent_index = 1;
    grandchild.bind_pose.translation = apc::Vec3(0.0f, 1.0f, 0.0f);
    grandchild.bind_pose.rotation = apc::Quat::identity();
    grandchild.joint_to_com = apc::Vec3(0, 0.5f, 0);
    asset.bones.push_back(grandchild);

    // Setup Pose (Start in T-Pose)
    apc::SkeletalPose local_pose;
    local_pose.set_to_bind_pose(asset);

    apc::SkeletalPose world_pose;
    world_pose.allocate(asset.get_bone_count());
    apc::SkeletalFK::calculate_world_transforms(asset, local_pose, world_pose);

    // TEST 1: Verify T-Pose World Positions
    assert(std::abs(world_pose.world_transforms[0].translation.y) < 0.001f);
    assert(std::abs(world_pose.world_transforms[1].translation.y - 1.0f) < 0.001f);
    assert(std::abs(world_pose.world_transforms[2].translation.y - 2.0f) < 0.001f);

    // TEST 2: Verify Joint-to-COM offsets in T-Pose
    apc::Vec3 root_com = apc::SkeletalFK::get_world_com(asset, world_pose, 0);
    assert(std::abs(root_com.y - 0.5f) < 0.001f); 

    // TEST 3: Rotate Grandchild 90 degrees around Z axis
    // In skeletal rigs, rotating a bone does NOT change its pivot's world position relative to its parent.
    // It ONLY changes where its Center of Mass is, and where its children will be placed.
    apc::Quat z_rotation = apc::Quat::from_axis_angle(apc::Vec3(0, 0, 1.0f), apc::APC_HALF_PI);
    local_pose.local_transforms[2].rotation = z_rotation;

    apc::SkeletalFK::calculate_world_transforms(asset, local_pose, world_pose);

    // Pivot should STILL be at (0, 2, 0)
    float gc_x = world_pose.world_transforms[2].translation.x;
    float gc_y = world_pose.world_transforms[2].translation.y;
    if (std::abs(gc_x) > 0.001f || std::abs(gc_y - 2.0f) > 0.001f) {
        return 1; // Fail: Pivot moved when it shouldn't have
    }

    // The COM offset (0, 0.5, 0) rotated 90 deg around Z becomes (-0.5, 0, 0).
    // So World COM = Pivot (0, 2, 0) + Rotated Offset (-0.5, 0, 0) = (-0.5, 2.0, 0)
    apc::Vec3 gc_com = apc::SkeletalFK::get_world_com(asset, world_pose, 2);
    if (std::abs(gc_com.x - (-0.5f)) > 0.001f || std::abs(gc_com.y - 2.0f) > 0.001f) {
        return 1; // Fail: COM didn't rotate with the bone
    }

    return 0; // Pass
}