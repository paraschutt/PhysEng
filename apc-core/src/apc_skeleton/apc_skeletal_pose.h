#pragma once
// =============================================================================
// Skeletal Pose — Local/world transforms, animation targets, dynamic state
// =============================================================================
//
// Defines the runtime state for an articulated body simulation:
//   - SkeletalPose: joint-space local transforms + world-space computed transforms
//   - SkeletalAnimTarget: what the animation system wants each bone to do
//   - SkeletalDynamicState: velocities, accelerations, external forces
//   - ExternalBoneForce: per-bone force/torque application
//
// =============================================================================

#include "apc_skeleton_types.h"
#include <vector>
#include <cstdint>

namespace apc {

// =============================================================================
// ExternalBoneForce — Force and torque applied to a bone from external sources
// =============================================================================
struct ExternalBoneForce {
    Vec3 force;      // World-space force (N)
    Vec3 torque;     // World-space torque (N·m) about bone COM

    APC_FORCEINLINE void reset() {
        force = Vec3(0.0f, 0.0f, 0.0f);
        torque = Vec3(0.0f, 0.0f, 0.0f);
    }

    APC_FORCEINLINE void add_force(const Vec3& f) {
        force = Vec3::add(force, f);
    }

    APC_FORCEINLINE void add_torque(const Vec3& t) {
        torque = Vec3::add(torque, t);
    }

    /// Apply a force at a specific world-space point, computing torque about COM.
    APC_FORCEINLINE void apply_at(const Vec3& f, const Vec3& world_point, const Vec3& world_com) {
        force = Vec3::add(force, f);
        Vec3 r = Vec3::sub(world_point, world_com);
        torque = Vec3::add(torque, Vec3::cross(r, f));
    }
};

// =============================================================================
// SkeletalAnimTarget — What the animation system wants each bone to do
// =============================================================================
struct SkeletalAnimTarget {
    std::vector<Transform> target_local_transforms;   // Desired local transforms from anim
    std::vector<Vec3> target_world_coms;              // Desired world COM positions
    std::vector<float> target_joint_velocities;        // Desired joint velocities
    bool has_target;

    void allocate(uint32_t bone_count) {
        target_local_transforms.resize(bone_count, Transform::identity());
        target_world_coms.resize(bone_count, Vec3(0.0f, 0.0f, 0.0f));
        target_joint_velocities.resize(bone_count, 0.0f);
        has_target = false;
    }
};

// =============================================================================
// SkeletalPose — Local + world transforms for all bones
// =============================================================================
struct SkeletalPose {
    std::vector<Transform> local_transforms;     // Joint-space local transforms
    std::vector<Transform> world_transforms;     // Computed world-space transforms

    // Multi-DOF joint state: generalized coordinates and velocities.
    // For Revolute1D: q[0] = angle in radians around joint_axes[0]
    // For Prismatic1D: q[0] = displacement along joint_axes[0]
    // For Spherical3D: q[0..2] = Euler angles (XYZ convention, radians)
    // For Fixed: no q entries needed (dof = 0)
    std::vector<float> joint_q;                  // Generalized positions

    SkeletalAnimTarget target;                   // Animation target data

    void allocate(uint32_t bone_count) {
        local_transforms.resize(bone_count, Transform::identity());
        world_transforms.resize(bone_count, Transform::identity());
        joint_q.resize(bone_count * APC_MAX_JOINT_DOF, 0.0f);
        target.allocate(bone_count);
    }

    void set_to_bind_pose(const SkeletalAsset& asset) {
        allocate(asset.get_bone_count());
        target.has_target = false;
        for (uint32_t i = 0; i < asset.get_bone_count(); ++i) {
            local_transforms[i] = asset.bones[i].bind_pose;
        }
        // Zero out all joint coordinates
        for (uint32_t i = 0; i < static_cast<uint32_t>(joint_q.size()); ++i) {
            joint_q[i] = 0.0f;
        }
    }
};

// =============================================================================
// SkeletalDynamicState — Velocities, accelerations, external forces
// =============================================================================
struct SkeletalDynamicState {
    // Generalized velocities: one per DOF per bone.
    // Indexed as: bone_index * APC_MAX_JOINT_DOF + dof_index
    std::vector<float> joint_velocities;

    // World-space quantities per bone
    std::vector<Vec3> world_omegas;         // Angular velocities (rad/s)
    std::vector<Vec3> world_v_coms;         // Linear velocities of COM (m/s)
    std::vector<Vec3> a_coms;              // Linear accelerations of COM (m/s²)
    std::vector<Vec3> alphas;              // Angular accelerations (rad/s²)

    // External forces per bone (from collision, user input, etc.)
    std::vector<ExternalBoneForce> external_forces;

    void allocate(uint32_t bone_count) {
        uint32_t q_size = bone_count * APC_MAX_JOINT_DOF;
        joint_velocities.resize(q_size, 0.0f);
        world_omegas.resize(bone_count, Vec3(0.0f, 0.0f, 0.0f));
        world_v_coms.resize(bone_count, Vec3(0.0f, 0.0f, 0.0f));
        a_coms.resize(bone_count, Vec3(0.0f, 0.0f, 0.0f));
        alphas.resize(bone_count, Vec3(0.0f, 0.0f, 0.0f));
        external_forces.resize(bone_count);
        for (uint32_t i = 0; i < bone_count; ++i) {
            external_forces[i].reset();
        }
    }

    /// Clear all external forces (call at the start of each sim step).
    void clear_external_forces() {
        for (uint32_t i = 0; i < static_cast<uint32_t>(external_forces.size()); ++i) {
            external_forces[i].reset();
        }
    }
};

} // namespace apc
