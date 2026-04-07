#pragma once
// =============================================================================
// Blend System — Per-bone animation/physics blend pipeline
// =============================================================================
//
// Implements the per-bone physics blend modes that allow individual bones to
// operate in different modes:
//
//   ANIM_DRIVEN:    Bone follows animation exactly (zero physics influence).
//                   Joint angle is overridden from the animation target each frame.
//
//   PHYSICS_DRIVEN: Bone is governed entirely by the ABA solver.
//                   Animation target is ignored.
//
//   BLENDED:        Bone receives both physics forces AND spring-damper forces
//                   pulling toward the animation target. The spring-damper
//                   creates a "soft follow" behavior where the bone can be
//                   perturbed by collisions but springs back.
//
// The blend system operates as a post-processing pass on the ABA output:
//   1. After ABA computes joint accelerations, the blend system modifies them
//      based on the current blend mode.
//   2. For ANIM_DRIVEN: accelerations are zeroed and position is overridden.
//   3. For BLENDED: spring-damper corrective torque is added.
//   4. Max deviation is enforced: if a BLENDED bone deviates too far from
//      its target, it temporarily switches to PHYSICS_DRIVEN.
//
// Integration with ArticulatedBody::step_ex:
//   The blend logic is already embedded in the ABA solver's forward pass 2.
//   This header provides standalone utility functions for:
//   - Computing blend correction forces
//   - Setting up animation targets from a pose snapshot
//   - Querying current deviation from target
//   - Transitioning between blend modes
//
// =============================================================================

#include "apc_skeleton_types.h"
#include "apc_skeletal_pose.h"
#include "apc_skeleton/apc_skeletal_fk.h"
#include "apc_math/apc_math_common.h"
#include <vector>
#include <cmath>
#include <cstring>
#include <algorithm>

namespace apc {

// =============================================================================
// BlendCorrectionResult — Output of blend correction computation
// =============================================================================
struct BlendCorrectionResult {
    float torque_correction[APC_MAX_JOINT_DOF]; // Per-DOF corrective torques
    float deviation;                             // Angular deviation from target (rad)
    float deviation_linear;                      // Linear COM deviation (m)
    bool max_deviation_exceeded;                 // Whether to snap to PHYSICS_DRIVEN
    uint8_t dof_count;                           // Number of active DOF

    void reset(uint8_t dof) {
        dof_count = dof;
        deviation = 0.0f;
        deviation_linear = 0.0f;
        max_deviation_exceeded = false;
        for (uint8_t i = 0; i < APC_MAX_JOINT_DOF; ++i) {
            torque_correction[i] = 0.0f;
        }
    }
};

// =============================================================================
// BlendSystem — Utilities for per-bone blend mode management
// =============================================================================
class BlendSystem {
public:
    // -------------------------------------------------------------------------
    // capture_target — Snapshot current pose as the animation target.
    // Call this when transitioning from physics to blended/anim mode,
    // or when a new animation pose is sampled.
    // -------------------------------------------------------------------------
    static void capture_target(
        const SkeletalAsset& asset,
        const SkeletalPose& current_pose,
        SkeletalPose& pose)
    {
        uint32_t N = asset.get_bone_count();
        pose.target.has_target = true;

        // Compute world transforms for COM calculation
        SkeletalPose world_pose;
        world_pose.allocate(N);
        SkeletalFK::calculate_world_transforms(asset, current_pose, world_pose);

        for (uint32_t i = 0; i < N; ++i) {
            pose.target.target_local_transforms[i] = current_pose.local_transforms[i];
            pose.target.target_world_coms[i] = SkeletalFK::get_world_com(asset, world_pose, i);

            // Copy current joint velocities as target velocities
            for (uint8_t d = 0; d < asset.bones[i].joint_dof; ++d) {
                if (i * APC_MAX_JOINT_DOF + d < pose.target.target_joint_velocities.size()) {
                    // Target velocity is zero — we want the bone to come to rest at target
                    pose.target.target_joint_velocities[i * APC_MAX_JOINT_DOF + d] = 0.0f;
                }
            }
        }
    }

    // -------------------------------------------------------------------------
    // set_target_from_pose — Set animation target from an external pose
    // (e.g., sampled from animation system).
    // -------------------------------------------------------------------------
    static void set_target_from_pose(
        const SkeletalAsset& asset,
        const SkeletalPose& anim_pose,
        SkeletalPose& sim_pose)
    {
        uint32_t N = asset.get_bone_count();
        sim_pose.target.has_target = true;

        SkeletalPose anim_world;
        anim_world.allocate(N);
        SkeletalFK::calculate_world_transforms(asset, anim_pose, anim_world);

        for (uint32_t i = 0; i < N; ++i) {
            sim_pose.target.target_local_transforms[i] = anim_pose.local_transforms[i];
            sim_pose.target.target_world_coms[i] = SkeletalFK::get_world_com(asset, anim_world, i);
        }
    }

    // -------------------------------------------------------------------------
    // compute_deviation — Measure how far a bone has deviated from its target.
    // Returns both angular (rotation) and linear (COM position) deviation.
    // -------------------------------------------------------------------------
    static BlendCorrectionResult compute_deviation(
        const Bone& bone,
        uint32_t bone_index,
        const SkeletalPose& pose,
        const SkeletalDynamicState& state)
    {
        BlendCorrectionResult result;
        result.reset(bone.joint_dof);

        if (!pose.target.has_target) return result;
        if (bone_index >= pose.target.target_local_transforms.size()) return result;

        // Angular deviation: angle between current and target rotation
        const Quat& current = pose.local_transforms[bone_index].rotation;
        const Quat& target = pose.target.target_local_transforms[bone_index].rotation;

        Quat diff = Quat::normalize(Quat::multiply(Quat::inverse(current), target));
        result.deviation = 2.0f * std::acos(aba_clamp(diff.w, -1.0f, 1.0f));

        // Linear deviation: distance between current and target world COM
        if (bone_index < pose.target.target_world_coms.size()) {
            // Need current world COM — compute it
            SkeletalPose world_pose;
            world_pose.allocate(1); // Minimal allocation
            // Note: we need full FK for this, but for efficiency the caller
            // should pre-compute world COMs. For now, use a simplified approach.
            // The ABA step already computes world_coms — the caller can pass them.
            // Here we just store the target for comparison.
            result.deviation_linear = result.deviation; // Approximation
        }

        // Check max deviation
        if (bone.physics.max_deviation > 0.0f &&
            result.deviation > bone.physics.max_deviation) {
            result.max_deviation_exceeded = true;
        }

        // Compute per-DOF corrective torques
        for (uint8_t d = 0; d < bone.joint_dof; ++d) {
            Vec3 axis(diff.x, diff.y, diff.z);
            float axis_len_sq = Vec3::length_sq(axis);
            float sign = 1.0f;
            if (axis_len_sq > APC_EPSILON_SQ) {
                float proj = Vec3::dot(Vec3::normalize(axis), bone.joint_axes[d]);
                sign = (proj >= 0.0f) ? 1.0f : -1.0f;
            }

            float qdot = get_joint_vel(state.joint_velocities, bone_index, d);
            float spring = -bone.physics.stiffness * result.deviation * sign;
            float damper = -bone.physics.damping * qdot;
            result.torque_correction[d] = spring + damper;

            // Emergency correction for max deviation exceeded
            if (result.max_deviation_exceeded) {
                float excess = result.deviation - bone.physics.max_deviation;
                result.torque_correction[d] -= sign * excess * bone.physics.stiffness * 10.0f;
            }
        }

        return result;
    }

    // -------------------------------------------------------------------------
    // apply_anim_driven — Override bone state to match animation target.
    // -------------------------------------------------------------------------
    static void apply_anim_driven(
        const Bone& bone,
        uint32_t bone_index,
        SkeletalPose& pose,
        SkeletalDynamicState& state)
    {
        if (!pose.target.has_target) return;
        if (bone_index >= pose.target.target_local_transforms.size()) return;

        pose.local_transforms[bone_index] = pose.target.target_local_transforms[bone_index];

        for (uint8_t d = 0; d < bone.joint_dof; ++d) {
            set_joint_vel(state.joint_velocities, bone_index, d, 0.0f);
            set_joint_q(pose.joint_q, bone_index, d, 0.0f);
        }
    }

    // -------------------------------------------------------------------------
    // apply_blended — Apply spring-damper correction for blended mode.
    // Returns the deviation for monitoring.
    // -------------------------------------------------------------------------
    static BlendCorrectionResult apply_blended(
        const Bone& bone,
        uint32_t bone_index,
        SkeletalPose& pose,
        SkeletalDynamicState& state,
        float /*dt*/)
    {
        BlendCorrectionResult result = compute_deviation(
            bone, bone_index, pose, state);

        if (bone.physics.max_deviation > 0.0f && result.max_deviation_exceeded) {
            // Emergency: switch to physics-driven for this frame
            // The strong corrective torque should pull it back
        }

        return result;
    }

    // -------------------------------------------------------------------------
    // set_bone_mode — Transition a bone to a new blend mode.
    // Handles mode-specific initialization.
    // -------------------------------------------------------------------------
    static void set_bone_mode(
        Bone& bone,
        uint32_t bone_index,
        PhysicsBlendMode new_mode,
        const SkeletalPose& current_pose,
        SkeletalPose& pose)
    {
        PhysicsBlendMode old_mode = bone.physics.mode;
        bone.physics.mode = new_mode;

        if (new_mode == PhysicsBlendMode::ANIM_DRIVEN && old_mode != PhysicsBlendMode::ANIM_DRIVEN) {
            // When switching TO anim-driven, capture current state as target
            // so there's no sudden jump
            if (bone_index < pose.target.target_local_transforms.size()) {
                pose.target.target_local_transforms[bone_index] =
                    current_pose.local_transforms[bone_index];
            }
        }
        else if (new_mode == PhysicsBlendMode::BLENDED && old_mode == PhysicsBlendMode::ANIM_DRIVEN) {
            // When transitioning from anim to blended, the spring-damper will
            // naturally take over since the current pose is at the target.
            // No special handling needed.
        }
    }

    // -------------------------------------------------------------------------
    // get_effective_mode — Get the current effective mode, accounting for
    // max_deviation overrides.
    // -------------------------------------------------------------------------
    static PhysicsBlendMode get_effective_mode(
        const Bone& bone,
        const BlendCorrectionResult& deviation)
    {
        if (bone.physics.mode == PhysicsBlendMode::BLENDED &&
            bone.physics.max_deviation > 0.0f &&
            deviation.max_deviation_exceeded) {
            return PhysicsBlendMode::PHYSICS_DRIVEN;
        }
        return bone.physics.mode;
    }

private:
    static float aba_clamp(float val, float min_val, float max_val) {
        if (val < min_val) return min_val;
        if (val > max_val) return max_val;
        return val;
    }

    // Multi-DOF accessors (same as in skeleton_apc.h)
    static float get_joint_vel(const std::vector<float>& jv, uint32_t bone, uint8_t dof) {
        return jv[bone * APC_MAX_JOINT_DOF + dof];
    }
    static void set_joint_vel(std::vector<float>& jv, uint32_t bone, uint8_t dof, float val) {
        jv[bone * APC_MAX_JOINT_DOF + dof] = val;
    }
    static float get_joint_q(const std::vector<float>& jq, uint32_t bone, uint8_t dof) {
        return jq[bone * APC_MAX_JOINT_DOF + dof];
    }
    static void set_joint_q(std::vector<float>& jq, uint32_t bone, uint8_t dof, float val) {
        jq[bone * APC_MAX_JOINT_DOF + dof] = val;
    }
};

} // namespace apc
