#pragma once
// ============================================================================
// DSPE Skeleton Controller
// - 15-bone hierarchy driven by animation target angles
// - Motor torques applied to reach target within strength limits
// - Foot capsules are ACTIVE in constraint solve (not passive sensors)
// - Foot velocity captured at contact for kick impulse calculation
// ============================================================================
#include "../components.h"
#include "event_system.h"
#include <array>

namespace dspe {

// ---------------------------------------------------------------------------
// Bone capsule dimensions (from brief spec)
// ---------------------------------------------------------------------------
struct BoneDimensions {
    FpVel length{};   // Bone length (m)
    FpPos radius{};   // Capsule radius (m)
};

static constexpr std::array<BoneDimensions, BONE_COUNT> BONE_DIMS = {{
    { FpVel::from_float(0.20f), FpPos::from_float(0.18f) }, // PELVIS (torso radius)
    { FpVel::from_float(0.30f), FpPos::from_float(0.15f) }, // SPINE
    { FpVel::from_float(0.22f), FpPos::from_float(0.12f) }, // HEAD
    { FpVel::from_float(0.28f), FpPos::from_float(0.07f) }, // L_ARM
    { FpVel::from_float(0.25f), FpPos::from_float(0.06f) }, // L_FOREARM
    { FpVel::from_float(0.28f), FpPos::from_float(0.07f) }, // R_ARM
    { FpVel::from_float(0.25f), FpPos::from_float(0.06f) }, // R_FOREARM
    { FpVel::from_float(0.40f), FpPos::from_float(0.12f) }, // L_THIGH
    { FpVel::from_float(0.38f), FpPos::from_float(0.10f) }, // L_CALF
    { FpVel::from_float(0.18f), FpPos::from_float(0.08f) }, // L_FOOT
    { FpVel::from_float(0.40f), FpPos::from_float(0.12f) }, // R_THIGH
    { FpVel::from_float(0.38f), FpPos::from_float(0.10f) }, // R_CALF
    { FpVel::from_float(0.18f), FpPos::from_float(0.08f) }, // R_FOOT
}};

// Motor strength limits per bone (N·m)
static constexpr std::array<FpVel, BONE_COUNT> BONE_MAX_TORQUE = {{
    FpVel::from_float(500.0f), // PELVIS
    FpVel::from_float(200.0f), // SPINE
    FpVel::from_float( 50.0f), // HEAD
    FpVel::from_float(150.0f), // L_ARM
    FpVel::from_float(100.0f), // L_FOREARM
    FpVel::from_float(150.0f), // R_ARM
    FpVel::from_float(100.0f), // R_FOREARM
    FpVel::from_float(400.0f), // L_THIGH
    FpVel::from_float(350.0f), // L_CALF
    FpVel::from_float( 80.0f), // L_FOOT
    FpVel::from_float(400.0f), // R_THIGH
    FpVel::from_float(350.0f), // R_CALF
    FpVel::from_float( 80.0f), // R_FOOT
}};

// ---------------------------------------------------------------------------
// Kick parameters
// ---------------------------------------------------------------------------
struct KickParams {
    FpVel  power{};            // [0, 1] normalised
    Vec3Vel direction{};       // Target ball direction (normalised)
    Vec3Vel contact_offset{};  // Offset from ball centre → generates spin
    EntityId kicker_id{INVALID_ENTITY};
    EntityId ball_id{BALL_ENTITY};
};

// ---------------------------------------------------------------------------
// Skeleton controller
// ---------------------------------------------------------------------------
class SkeletonController {
public:
    // Update all skeletons: FK pass, motor torques, foot sensor update
    void update(std::array<Entity, MAX_ENTITIES>& entities,
                FpVel substep_dt,
                EventQueue& events,
                uint32_t frame);

    // Compute kick impulse given foot velocity + ball contact
    static Vec3Vel compute_kick_impulse(const KickParams& params,
                                        const Entity& kicker,
                                        const Entity& ball);

    // Apply kick: impulse + off-centre spin generation
    static void apply_kick(Entity& ball_entity,
                           const KickParams& params,
                           const Entity& kicker,
                           EventQueue& events,
                           uint32_t frame);

    // Apply jump impulse to pelvis
    static void apply_jump(Entity& player_entity, FpVel impulse_strength);

    // Set animation target angles for all bones
    static void set_animation_targets(Skeleton& skel,
                                      const std::array<FpVel, BONE_COUNT>& targets);

private:
    // Forward kinematics: world_pos/rot from parent chain
    static void forward_kinematics(Skeleton& skel,
                                   const Vec3Pos& root_pos,
                                   const QuatVel& root_rot);

    // Spring-damper motor: compute torque to drive joint toward target
    static FpVel motor_torque(FpVel current_angle,
                               FpVel target_angle,
                               FpVel current_vel,
                               FpVel stiffness,
                               FpVel damping,
                               FpVel max_torque);

    // Enforce joint limits (hard clamp on angle)
    static FpVel clamp_joint_angle(FpVel angle, const JointLimit& limit,
                                    JointType type);

    // Ground raycast for foot sensor
    static bool raycast_ground(Vec3Pos foot_world_pos,
                                const std::array<Entity, MAX_ENTITIES>& entities,
                                FpPos ground_y,
                                Vec3Vel& out_normal);

    // Update bone world velocity (for contact response at foot)
    static void update_bone_velocity(BoneState& bone,
                                     const BoneState& prev_bone,
                                     FpVel inv_dt);
};

} // namespace dspe