#pragma once
#include "apc_skeleton_types.h"
#include "apc_skeletal_pose.h"

namespace apc {

class SkeletalFK {
public:
    // Calculates world transforms from local transforms.
    // Iterates strictly in array order (0 to N). Parent MUST have a lower index than child.
    static void calculate_world_transforms(
        const SkeletalAsset& asset, 
        const SkeletalPose& local_pose, 
        SkeletalPose& out_world_pose) 
    {
        uint32_t bone_count = asset.get_bone_count();
        for (uint32_t i = 0; i < bone_count; ++i) {
            const Bone& bone = asset.bones[i];
            const Transform& local = local_pose.local_transforms[i];

            if (asset.is_root(i)) {
                // Root bone: world space equals local space
                out_world_pose.world_transforms[i] = local;
            } else {
                // Child bone: world = parent_world * local
                const Transform& parent_world = out_world_pose.world_transforms[bone.parent_index];
                out_world_pose.world_transforms[i] = Transform::multiply(parent_world, local);
            }
        }
    }

    // Helper to get the World Space Center of Mass for a specific bone
    static Vec3 get_world_com(
        const SkeletalAsset& asset,
        const SkeletalPose& world_pose,
        uint32_t bone_index)
    {
        const Bone& bone = asset.bones[bone_index];
        const Transform& world_tf = world_pose.world_transforms[bone_index];
        
        // Rotate the local joint_to_com offset into world space and add to world translation
        return Vec3::add(world_tf.translation, world_tf.rotation.rotate(bone.joint_to_com));
    }
};

} // namespace apc