#pragma once
// =============================================================================
// Loop Closure Constraints — Position constraints for closed kinematic chains
// =============================================================================
//
// Some skeletal structures have closed kinematic chains (loops) that cannot
// be represented by a tree structure. Examples:
//   - Shoulder girdle: clavicles connect both shoulders to the spine
//   - Pelvis stability: left and right hips should maintain relative position
//
// Since Featherstone ABA only handles tree topologies, loops are resolved
// iteratively after the main ABA step. This is the "constraint stabilization"
// approach:
//   1. Run standard tree-structured ABA
//   2. For each loop closure constraint, compute the constraint error
//   3. Apply corrective forces to reduce the error
//   4. Repeat for a few iterations
//
// The corrective forces are applied as external forces on the bones.
//
// =============================================================================

#include "apc_skeleton/apc_skeleton_types.h"
#include "apc_skeleton/apc_skeletal_pose.h"
#include "apc_skeleton/apc_skeletal_fk.h"
#include "apc_skeleton/apc_skeleton_apc.h"
#include "apc_math/apc_math_common.h"
#include <vector>
#include <cmath>

namespace apc {

// =============================================================================
// LoopClosureType — Types of loop closure constraints
// =============================================================================
enum class LoopClosureType : uint8_t {
    Distance,     // Maintain fixed distance between two bone points
    Position,     // Maintain a relative position offset between two bones
    Orientation   // Maintain a relative orientation between two bones
};

// =============================================================================
// LoopClosureConstraint — A single loop closure constraint
// =============================================================================
struct LoopClosureConstraint {
    LoopClosureType type;
    uint32_t bone_a;          // First bone index
    uint32_t bone_b;          // Second bone index
    Vec3 local_point_a;      // Point on bone A in bone-local space
    Vec3 local_point_b;      // Point on bone B in bone-local space
    Vec3 target_offset;      // Desired relative offset (B - A) in world space
    float stiffness;          // Constraint stiffness (N/m or N·m/rad)
    float damping;            // Velocity damping on constraint
    uint8_t enabled;          // Is this constraint active?

    /// Default: disabled constraint
    LoopClosureConstraint()
        : type(LoopClosureType::Distance)
        , bone_a(0), bone_b(0)
        , local_point_a(0.0f, 0.0f, 0.0f)
        , local_point_b(0.0f, 0.0f, 0.0f)
        , target_offset(0.0f, 0.0f, 0.0f)
        , stiffness(1000.0f)
        , damping(50.0f)
        , enabled(0)
    {}

    /// Create a distance constraint between two bone points.
    static LoopClosureConstraint make_distance(
        uint32_t bone_a, uint32_t bone_b,
        const Vec3& point_a, const Vec3& point_b,
        float stiff = 1000.0f, float damp = 50.0f)
    {
        LoopClosureConstraint c;
        c.type = LoopClosureType::Distance;
        c.bone_a = bone_a;
        c.bone_b = bone_b;
        c.local_point_a = point_a;
        c.local_point_b = point_b;
        c.stiffness = stiff;
        c.damping = damp;
        c.enabled = 1;
        return c;
    }

    /// Create a position constraint (fixed relative offset).
    static LoopClosureConstraint make_position(
        uint32_t bone_a, uint32_t bone_b,
        const Vec3& point_a, const Vec3& point_b,
        const Vec3& offset,
        float stiff = 1000.0f, float damp = 50.0f)
    {
        LoopClosureConstraint c;
        c.type = LoopClosureType::Position;
        c.bone_a = bone_a;
        c.bone_b = bone_b;
        c.local_point_a = point_a;
        c.local_point_b = point_b;
        c.target_offset = offset;
        c.stiffness = stiff;
        c.damping = damp;
        c.enabled = 1;
        return c;
    }
};

// =============================================================================
// LoopClosureSolver — Resolves loop closure constraints iteratively
// =============================================================================
class LoopClosureSolver {
public:
    static constexpr uint32_t MAX_CONSTRAINTS = 16u;
    static constexpr uint32_t MAX_ITERATIONS = 4u;

    // -------------------------------------------------------------------------
    // resolve — Apply loop closure constraints as corrective forces.
    //
    // Call AFTER ArticulatedBody::step_ex() but BEFORE integration if
    // using a split integrate/constraint approach. Alternatively, call
    // AFTER step_ex() and the corrections will be applied next frame.
    //
    // For each enabled constraint:
    //   1. Compute world-space positions of the constrained points
    //   2. Compute the error (deviation from target)
    //   3. Apply spring-damper corrective forces to both bones
    // -------------------------------------------------------------------------
    static void resolve(
        const LoopClosureConstraint* constraints,
        uint32_t constraint_count,
        const SkeletalAsset& asset,
        const SkeletalPose& world_pose,
        SkeletalDynamicState& state,
        float /*dt*/)
    {
        for (uint32_t iter = 0; iter < MAX_ITERATIONS; ++iter) {
            for (uint32_t c = 0; c < constraint_count; ++c) {
                const LoopClosureConstraint& lc = constraints[c];
                if (!lc.enabled) continue;
                if (lc.bone_a >= asset.get_bone_count()) continue;
                if (lc.bone_b >= asset.get_bone_count()) continue;

                switch (lc.type) {
                case LoopClosureType::Distance:
                    resolve_distance(lc, asset, world_pose, state);
                    break;
                case LoopClosureType::Position:
                    resolve_position(lc, asset, world_pose, state);
                    break;
                case LoopClosureType::Orientation:
                    // Orientation constraints not yet implemented
                    break;
                }
            }
        }
    }

    // -------------------------------------------------------------------------
    // create_shoulder_girdle — Create bilateral shoulder constraint.
    // Maintains a fixed distance between left and right shoulder joints.
    // -------------------------------------------------------------------------
    static LoopClosureConstraint create_shoulder_girdle(
        uint32_t l_shoulder_idx,
        uint32_t r_shoulder_idx,
        float stiff = 800.0f,
        float damp = 40.0f)
    {
        return LoopClosureConstraint::make_distance(
            l_shoulder_idx, r_shoulder_idx,
            Vec3(0, 0, 0), Vec3(0, 0, 0),
            stiff, damp);
    }

    // -------------------------------------------------------------------------
    // create_pelvis_stability — Create bilateral hip constraint.
    // Maintains a fixed distance between left and right hip joints.
    // -------------------------------------------------------------------------
    static LoopClosureConstraint create_pelvis_stability(
        uint32_t l_hip_idx,
        uint32_t r_hip_idx,
        float stiff = 1000.0f,
        float damp = 50.0f)
    {
        return LoopClosureConstraint::make_distance(
            l_hip_idx, r_hip_idx,
            Vec3(0, 0, 0), Vec3(0, 0, 0),
            stiff, damp);
    }

private:
    // -------------------------------------------------------------------------
    // resolve_distance — Maintain fixed distance between two bone points
    // -------------------------------------------------------------------------
    static void resolve_distance(
        const LoopClosureConstraint& lc,
        const SkeletalAsset& /*asset*/,
        const SkeletalPose& world_pose,
        SkeletalDynamicState& state)
    {
        const Transform& tf_a = world_pose.world_transforms[lc.bone_a];
        const Transform& tf_b = world_pose.world_transforms[lc.bone_b];

        // World positions of constrained points
        Vec3 world_a = Vec3::add(tf_a.translation,
            tf_a.rotation.rotate(lc.local_point_a));
        Vec3 world_b = Vec3::add(tf_b.translation,
            tf_b.rotation.rotate(lc.local_point_b));

        Vec3 delta = Vec3::sub(world_b, world_a);
        float current_dist = Vec3::length(delta);
        float target_dist = Vec3::length(lc.target_offset);

        // If no target set, use initial distance
        if (target_dist < APC_EPSILON) {
            target_dist = current_dist;
        }

        float error = current_dist - target_dist;

        // Skip if error is tiny
        if (std::abs(error) < APC_EPSILON) return;

        // Compute corrective force along the constraint direction
        Vec3 dir = (current_dist > APC_EPSILON)
            ? Vec3::scale(delta, 1.0f / current_dist)
            : Vec3(0.0f, 1.0f, 0.0f);

        // Spring force: F = -k * error * direction
        Vec3 spring_force = Vec3::scale(dir, -lc.stiffness * error);

        // Apply equal and opposite forces to both bones
        if (lc.bone_a < state.external_forces.size()) {
            state.external_forces[lc.bone_a].add_force(Vec3::scale(spring_force, -1.0f));
        }
        if (lc.bone_b < state.external_forces.size()) {
            state.external_forces[lc.bone_b].add_force(spring_force);
        }
    }

    // -------------------------------------------------------------------------
    // resolve_position — Maintain fixed relative position offset
    // -------------------------------------------------------------------------
    static void resolve_position(
        const LoopClosureConstraint& lc,
        const SkeletalAsset& /*asset*/,
        const SkeletalPose& world_pose,
        SkeletalDynamicState& state)
    {
        const Transform& tf_a = world_pose.world_transforms[lc.bone_a];
        const Transform& tf_b = world_pose.world_transforms[lc.bone_b];

        Vec3 world_a = Vec3::add(tf_a.translation,
            tf_a.rotation.rotate(lc.local_point_a));
        Vec3 world_b = Vec3::add(tf_b.translation,
            tf_b.rotation.rotate(lc.local_point_b));

        Vec3 current_offset = Vec3::sub(world_b, world_a);
        Vec3 error = Vec3::sub(current_offset, lc.target_offset);

        float error_mag = Vec3::length(error);
        if (error_mag < APC_EPSILON) return;

        // Spring force toward target offset
        Vec3 correction = Vec3::scale(error, lc.stiffness);

        if (lc.bone_a < state.external_forces.size()) {
            state.external_forces[lc.bone_a].add_force(Vec3::scale(correction, 0.5f));
        }
        if (lc.bone_b < state.external_forces.size()) {
            state.external_forces[lc.bone_b].add_force(Vec3::scale(correction, -0.5f));
        }
    }
};

} // namespace apc
