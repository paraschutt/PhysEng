#pragma once
// =============================================================================
// Skeletal Types — Joint system, bone definitions, skeletal asset format
// =============================================================================
//
// Defines the core data structures for APC's articulated body simulation:
//   - JointType: supported joint degrees of freedom
//   - JointLimits: per-axis min/max constraints
//   - Bone: complete bone definition with joint, inertia, and physics blend state
//   - BonePhysicsState: per-bone animation/physics blend configuration
//   - SkeletalAsset: collection of bones forming an articulated body
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation (fixed-size arrays where possible)
//   - Deterministic: index-order iteration, no FMA
//   - C++17, GCC + MSVC cross-platform
//
// =============================================================================

#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include "apc_math/apc_mat3.h"
#include "apc_math/apc_math_common.h"
#include <vector>
#include <cstdint>

namespace apc {

// =============================================================================
// Transform — SE(3) rigid body transform (translation + rotation)
// =============================================================================
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

// =============================================================================
// PhysicsBlendMode — How animation and physics combine per-bone
// =============================================================================
enum class PhysicsBlendMode {
    ANIM_DRIVEN,     // 0% physics — joint angle driven entirely by animation
    PHYSICS_DRIVEN,  // 100% pure ABA ragdoll
    BLENDED          // Spring-damper pulling toward animation target
};

// =============================================================================
// JointType — Supported joint degrees of freedom
// =============================================================================
enum class JointType : uint8_t {
    Fixed,        // 0 DOF — rigid connection (weld)
    Revolute1D,   // 1 DOF — hinge joint rotating around a single axis
    Prismatic1D,  // 1 DOF — sliding joint along a single axis
    Spherical3D   // 3 DOF — ball-and-socket joint (3-axis rotation)
};

// Maximum DOF per joint type. Used for sizing fixed arrays.
static constexpr uint32_t APC_MAX_JOINT_DOF = 3u;

// =============================================================================
// JointLimits — Per-axis angle/position limits for joints
// =============================================================================
struct JointLimits {
    float min_values[APC_MAX_JOINT_DOF];  // Lower bounds (radians or meters)
    float max_values[APC_MAX_JOINT_DOF];  // Upper bounds (radians or meters)
    uint8_t dof_count;                     // Number of active axes (1 or 3)

    /// Default: no limits (wide range).
    APC_FORCEINLINE void set_no_limits(uint8_t dof) {
        dof_count = dof;
        for (uint8_t i = 0; i < APC_MAX_JOINT_DOF; ++i) {
            min_values[i] = -1e10f;
            max_values[i] = 1e10f;
        }
    }

    /// Set symmetric limits for all axes. Typical for ragdoll joints.
    APC_FORCEINLINE void set_symmetric(float half_range, uint8_t dof) {
        dof_count = dof;
        for (uint8_t i = 0; i < APC_MAX_JOINT_DOF; ++i) {
            min_values[i] = -half_range;
            max_values[i] = half_range;
        }
    }
};

// =============================================================================
// BonePhysicsState — Per-bone animation/physics blend configuration
// =============================================================================
struct BonePhysicsState {
    PhysicsBlendMode mode = PhysicsBlendMode::PHYSICS_DRIVEN;

    float stiffness = 0.0f;      // Spring constant for BLENDED mode (N·m/rad or N/m)
    float damping = 0.0f;        // Velocity damping for BLENDED mode (N·m·s/rad)
    float max_deviation = 0.0f;  // If deviation > this, snap to PHYSICS_DRIVEN (rad or m)

    /// Reset to default pure physics mode.
    APC_FORCEINLINE void reset() {
        mode = PhysicsBlendMode::PHYSICS_DRIVEN;
        stiffness = 0.0f;
        damping = 0.0f;
        max_deviation = 0.0f;
    }
};

// =============================================================================
// Bone — Complete bone definition for articulated body simulation
// =============================================================================
struct Bone {
    uint32_t parent_index;           // Index of parent bone (0xFFFFFFFF for root)
    Transform bind_pose;             // Bind-pose local transform relative to parent
    Vec3 joint_to_com;              // Offset from joint pivot to center of mass (local)
    Mat3 local_inverse_inertia;     // Inverse inertia tensor in local bone space
    float inverse_mass;             // 1/mass (0.0f for kinematic bones)
    float mass;                     // Mass in kg (cached for external force calc)

    // --- Joint Configuration ---
    JointType joint_type;            // Type of joint connecting to parent
    Vec3 joint_axes[APC_MAX_JOINT_DOF]; // Local joint axes (unit vectors, bone-local space)
    uint8_t joint_dof;              // Number of active degrees of freedom (0, 1, or 3)
    JointLimits joint_limits;       // Per-axis limits
    float joint_damping;            // Passive joint damping (reduces joint velocity)

    // --- Physics Blend State ---
    BonePhysicsState physics;       // Animation/physics blend configuration

    // --- Collision Participation ---
    uint8_t collision_enabled;      // Does this bone generate/resolve collisions?

    /// Default constructor — creates a fixed joint at origin.
    APC_FORCEINLINE Bone()
        : parent_index(0xFFFFFFFF)
        , bind_pose(Transform::identity())
        , joint_to_com(Vec3(0.0f, 0.0f, 0.0f))
        , local_inverse_inertia(Mat3::identity())
        , inverse_mass(0.0f)
        , mass(0.0f)
        , joint_type(JointType::Fixed)
        , joint_dof(0)
        , joint_damping(0.0f)
        , physics()
        , collision_enabled(1)
    {
        joint_axes[0] = Vec3(0.0f, 1.0f, 0.0f);
        joint_axes[1] = Vec3(1.0f, 0.0f, 0.0f);
        joint_axes[2] = Vec3(0.0f, 0.0f, 1.0f);
        joint_limits.set_no_limits(0);
    }

    /// Convenience: configure as a revolute (hinge) joint.
    static Bone make_revolute(
        uint32_t parent,
        const Vec3& local_offset,
        const Vec3& axis,
        float bone_mass,
        float inertia,
        float limit_half_range = 0.0f,
        float damping = 0.0f)
    {
        Bone b;
        b.parent_index = parent;
        b.bind_pose = Transform{local_offset, Quat::identity()};
        b.joint_to_com = Vec3(0.0f, 0.0f, 0.0f);
        b.inverse_mass = (bone_mass > 0.0f) ? (1.0f / bone_mass) : 0.0f;
        b.mass = bone_mass;
        float inv_I = (inertia > 0.0f) ? (1.0f / inertia) : 0.0f;
        b.local_inverse_inertia = Mat3{{inv_I, 0.0f, 0.0f, 0.0f, inv_I, 0.0f, 0.0f, 0.0f, inv_I}};
        b.joint_type = JointType::Revolute1D;
        b.joint_axes[0] = Vec3::normalize(axis);
        b.joint_dof = 1;
        b.joint_damping = damping;
        b.collision_enabled = 1;
        if (limit_half_range > 0.0f) {
            b.joint_limits.set_symmetric(limit_half_range, 1);
        } else {
            b.joint_limits.set_no_limits(1);
        }
        return b;
    }

    /// Convenience: configure as a prismatic (sliding) joint.
    static Bone make_prismatic(
        uint32_t parent,
        const Vec3& local_offset,
        const Vec3& slide_axis,
        float bone_mass,
        float inertia,
        float limit_half_range = 0.0f,
        float damping = 0.0f)
    {
        Bone b;
        b.parent_index = parent;
        b.bind_pose = Transform{local_offset, Quat::identity()};
        b.joint_to_com = Vec3(0.0f, 0.0f, 0.0f);
        b.inverse_mass = (bone_mass > 0.0f) ? (1.0f / bone_mass) : 0.0f;
        b.mass = bone_mass;
        float inv_I = (inertia > 0.0f) ? (1.0f / inertia) : 0.0f;
        b.local_inverse_inertia = Mat3{{inv_I, 0.0f, 0.0f, 0.0f, inv_I, 0.0f, 0.0f, 0.0f, inv_I}};
        b.joint_type = JointType::Prismatic1D;
        b.joint_axes[0] = Vec3::normalize(slide_axis);
        b.joint_dof = 1;
        b.joint_damping = damping;
        b.collision_enabled = 1;
        if (limit_half_range > 0.0f) {
            b.joint_limits.set_symmetric(limit_half_range, 1);
        } else {
            b.joint_limits.set_no_limits(1);
        }
        return b;
    }

    /// Convenience: configure as a spherical (ball-and-socket) joint.
    /// Uses canonical axes: X, Y, Z for the 3 DOF.
    static Bone make_spherical(
        uint32_t parent,
        const Vec3& local_offset,
        float bone_mass,
        float inertia,
        float limit_half_range = 0.0f,
        float damping = 0.0f)
    {
        Bone b;
        b.parent_index = parent;
        b.bind_pose = Transform{local_offset, Quat::identity()};
        b.joint_to_com = Vec3(0.0f, 0.0f, 0.0f);
        b.inverse_mass = (bone_mass > 0.0f) ? (1.0f / bone_mass) : 0.0f;
        b.mass = bone_mass;
        float inv_I = (inertia > 0.0f) ? (1.0f / inertia) : 0.0f;
        b.local_inverse_inertia = Mat3{{inv_I, 0.0f, 0.0f, 0.0f, inv_I, 0.0f, 0.0f, 0.0f, inv_I}};
        b.joint_type = JointType::Spherical3D;
        b.joint_axes[0] = Vec3(1.0f, 0.0f, 0.0f); // X rotation
        b.joint_axes[1] = Vec3(0.0f, 1.0f, 0.0f); // Y rotation
        b.joint_axes[2] = Vec3(0.0f, 0.0f, 1.0f); // Z rotation
        b.joint_dof = 3;
        b.joint_damping = damping;
        b.collision_enabled = 1;
        if (limit_half_range > 0.0f) {
            b.joint_limits.set_symmetric(limit_half_range, 3);
        } else {
            b.joint_limits.set_no_limits(3);
        }
        return b;
    }

    /// Convenience: configure as a fixed (welded) joint.
    static Bone make_fixed(
        uint32_t parent,
        const Vec3& local_offset,
        float bone_mass = 0.0f,
        float inertia = 0.0f)
    {
        Bone b;
        b.parent_index = parent;
        b.bind_pose = Transform{local_offset, Quat::identity()};
        b.joint_to_com = Vec3(0.0f, 0.0f, 0.0f);
        b.inverse_mass = (bone_mass > 0.0f) ? (1.0f / bone_mass) : 0.0f;
        b.mass = bone_mass;
        float inv_I = (inertia > 0.0f) ? (1.0f / inertia) : 0.0f;
        b.local_inverse_inertia = Mat3{{inv_I, 0.0f, 0.0f, 0.0f, inv_I, 0.0f, 0.0f, 0.0f, inv_I}};
        b.joint_type = JointType::Fixed;
        b.joint_dof = 0;
        b.joint_damping = 0.0f;
        b.collision_enabled = 1;
        b.joint_limits.set_no_limits(0);
        return b;
    }
};

// =============================================================================
// SkeletalAsset — Collection of bones forming an articulated body
// =============================================================================
struct SkeletalAsset {
    std::vector<Bone> bones;

    uint32_t get_bone_count() const { return static_cast<uint32_t>(bones.size()); }

    bool is_root(uint32_t bone_index) const {
        return bone_index == 0xFFFFFFFF || bones[bone_index].parent_index == 0xFFFFFFFF;
    }

    /// Get total DOF count across all joints.
    uint32_t get_total_dof() const {
        uint32_t total = 0;
        for (uint32_t i = 0; i < get_bone_count(); ++i) {
            total += bones[i].joint_dof;
        }
        return total;
    }

    /// Validate that the bone hierarchy is well-formed:
    ///   - Root bone exists (index 0 or parent == 0xFFFFFFFF)
    ///   - Parent indices refer to valid bones (or 0xFFFFFFFF)
    ///   - Bones are ordered so parent always has lower index than child
    uint32_t validate() const {
        if (get_bone_count() == 0) return 0; // Empty asset is valid

        if (!is_root(0)) return 2; // First bone must be root

        for (uint32_t i = 1; i < get_bone_count(); ++i) {
            const Bone& b = bones[i];
            if (b.parent_index == 0xFFFFFFFF) return 3; // Only bone 0 can be root
            if (b.parent_index >= i) return 4; // Parent must have lower index
        }

        return 0; // Valid
    }
};

} // namespace apc
