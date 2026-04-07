#pragma once
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include "apc_math/apc_mat3.h"
#include <vector>
#include <cstdint>

namespace apc {

struct Transform {
    Vec3 translation;
    Quat rotation;

    APC_FORCEINLINE static Transform identity() {
        return {Vec3(0.0f, 0.0f, 0.0f), Quat::identity()};
    }

    APC_FORCEINLINE static Transform multiply(const Transform& parent, const Transform& local) {
        Transform out;
        out.rotation = Quat::multiply(parent.rotation, local.rotation);
        out.translation = Vec3::add(parent.translation, parent.rotation.rotate(local.translation));
        return out;
    }
};

enum class PhysicsBlendMode {
    ANIM_DRIVEN,     // 0% physics
    PHYSICS_DRIVEN,   // 100% pure ABA ragdoll
    BLENDED          // Spring-damper back to animation target
};

struct BonePhysicsState {
    PhysicsBlendMode mode = PhysicsBlendMode::PHYSICS_DRIVEN;
    
    float stiffness = 0.0f;      
    float damping = 0.0f;        
    float max_deviation = 0.0f;   // If deviation > this, force PHYSICS_DRIVEN for 1 frame
};

struct Bone {
    uint32_t parent_index;
    Transform bind_pose;
    Vec3 joint_to_com;
    Mat3 local_inverse_inertia;
    float inverse_mass;
    BonePhysicsState physics; // Defaults to pure physics
};

struct SkeletalAsset {
    std::vector<Bone> bones;
    uint32_t get_bone_count() const { return static_cast<uint32_t>(bones.size()); }
    
    bool is_root(uint32_t bone_index) const {
        return bone_index == 0xFFFFFFFF || bones[bone_index].parent_index == 0xFFFFFFFF;
    }
};

} // namespace apc