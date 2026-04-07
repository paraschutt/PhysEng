#pragma once
#include "apc_skeleton_types.h"
#include <vector>

namespace apc {

struct SkeletalAnimTarget {
    std::vector<Transform> target_local_transforms;
    std::vector<Vec3> target_world_coms; 
    std::vector<float> target_joint_accelerations; 
    bool has_target = false;

    void allocate(uint32_t bone_count) {
        target_local_transforms.resize(bone_count, Transform::identity());
        target_world_coms.resize(bone_count, Vec3(0.0f, 0.0f, 0.0f));
        target_joint_accelerations.resize(bone_count, 0.0f);
        has_target = false;
    }
};

struct SkeletalPose {
    std::vector<Transform> local_transforms;
    std::vector<Transform> world_transforms;
    
    SkeletalAnimTarget target; // Added: Tracks what animation wants

    void allocate(uint32_t bone_count) {
        local_transforms.resize(bone_count, Transform::identity());
        world_transforms.resize(bone_count, Transform::identity());
        target.allocate(bone_count);
    }
    
    void set_to_bind_pose(const SkeletalAsset& asset) {
        allocate(asset.get_bone_count());
        target.has_target = false;
        for (uint32_t i = 0; i < asset.get_bone_count(); ++i) {
            local_transforms[i] = asset.bones[i].bind_pose;
        }
    }
};

} // namespace apc