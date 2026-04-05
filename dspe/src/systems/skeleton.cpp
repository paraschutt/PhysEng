// ============================================================================
// DSPE Skeleton Controller Implementation
// ============================================================================
#include "dspe/systems/skeleton.h"
#include "dspe/systems/sleep_system.h"
#include <algorithm>

namespace dspe {

// ============================================================================
// Forward kinematics — world poses from parent chain
// ============================================================================
void SkeletonController::forward_kinematics(Skeleton& skel,
                                             const Vec3Pos& root_pos,
                                             const QuatVel& root_rot) {
    // PELVIS (root)
    skel.bones[BONE_PELVIS].world_pos = root_pos;
    skel.bones[BONE_PELVIS].world_rot = root_rot;

    // Walk tree in child order (parents always before children in BONE enum)
    for (uint8_t b = 1; b < BONE_COUNT; ++b) {
        int8_t  parent_idx = BONE_PARENT[b];
        if (parent_idx < 0) continue;

        BoneState& parent = skel.bones[parent_idx];
        BoneState& bone   = skel.bones[b];

        // Child position = parent position + parent_rotation * bone_offset
        // Bone offset = along parent bone axis by parent bone length
        FpVel bone_len = BONE_DIMS[parent_idx].length;
        // Bone axis in local space: Y axis (along bone)
        Vec3Vel local_offset = { FpVel::zero(), bone_len, FpVel::zero() };
        Vec3Vel world_offset = parent.world_rot.rotate(local_offset);

        // bone world pos (Q24.8): parent_pos + world_offset (Q15.16 → Q24.8)
        bone.world_pos = {
            FpPos{parent.world_pos.x.raw + (world_offset.x.raw >> 8)},
            FpPos{parent.world_pos.y.raw + (world_offset.y.raw >> 8)},
            FpPos{parent.world_pos.z.raw + (world_offset.z.raw >> 8)},
        };

        // Apply joint rotation around joint axis
        // Joint angle → local quaternion around X axis (simplified revolute)
        FpVel half_angle = bone.joint_angle >> 1;
        // Approximate: for small angles, cos ≈ 1 - θ²/2, sin ≈ θ
        // (avoiding transcendentals — good to ~0.1 rad error for full FK)
        FpVel cos_h = FpVel::one() - (half_angle * half_angle >> 1);
        FpVel sin_h = half_angle;
        QuatVel joint_rot{ cos_h, sin_h, FpVel::zero(), FpVel::zero() };
        bone.world_rot = parent.world_rot * joint_rot;

        // Update world-space capsule collider
        FpVel len  = BONE_DIMS[b].length;
        FpPos rad  = BONE_DIMS[b].radius;
        Vec3Vel local_tip = { FpVel::zero(), len, FpVel::zero() };
        Vec3Vel world_tip = bone.world_rot.rotate(local_tip);
        bone.collider_local.local_base = Vec3Pos::zero();
        bone.collider_local.local_tip  = {
            FpPos{world_tip.x.raw >> 8},
            FpPos{world_tip.y.raw >> 8},
            FpPos{world_tip.z.raw >> 8},
        };
        bone.collider_local.radius = rad;
    }
}

// ============================================================================
// Joint limit clamp
// ============================================================================
FpVel SkeletonController::clamp_joint_angle(FpVel angle, const JointLimit& limit,
                                              JointType type) {
    if (type == JointType::REVOLUTE) {
        return fp_clamp(angle, limit.min_angle, limit.max_angle);
    }
    // Spherical: clamp magnitude (cone)
    if (angle.abs() > limit.cone_angle) {
        return angle.raw >= 0 ? limit.cone_angle : -limit.cone_angle;
    }
    return angle;
}

// ============================================================================
// Spring-damper motor torque
// ============================================================================
FpVel SkeletonController::motor_torque(FpVel current, FpVel target,
                                        FpVel av, FpVel stiffness,
                                        FpVel damping, FpVel max_torque) {
    FpVel err    = target - current;
    FpVel torque = (stiffness * err) - (damping * av);
    return fp_clamp(torque, -max_torque, max_torque);
}

// ============================================================================
// Ground raycast (simplified: check y < ground_y + foot_radius)
// ============================================================================
bool SkeletonController::raycast_ground(Vec3Pos foot_world_pos,
                                         const std::array<Entity, MAX_ENTITIES>&,
                                         FpPos ground_y,
                                         Vec3Vel& out_normal) {
    // Simple flat-pitch ground check (pitched mesh not yet in Phase 1)
    bool grounded = foot_world_pos.y <= (ground_y + FpPos::from_float(0.05f));
    out_normal = { FpVel::zero(), FpVel::one(), FpVel::zero() };
    return grounded;
}

// ============================================================================
// Bone velocity update (finite difference)
// ============================================================================
void SkeletonController::update_bone_velocity(BoneState& bone,
                                               const BoneState& prev,
                                               FpVel inv_dt) {
    // v = (pos_new - pos_old) / dt  (Q24.8 diff → Q15.16 vel)
    auto delta_to_vel = [&](FpPos a, FpPos b) -> FpVel {
        int64_t d = static_cast<int64_t>(a.raw - b.raw) << 8; // Q24.8 → Q15.16
        int64_t v = d * inv_dt.raw >> 16;
        return FpVel{static_cast<int32_t>(v)};
    };
    bone.world_vel = {
        delta_to_vel(bone.world_pos.x, prev.world_pos.x),
        delta_to_vel(bone.world_pos.y, prev.world_pos.y),
        delta_to_vel(bone.world_pos.z, prev.world_pos.z),
    };
}

// ============================================================================
// Main skeleton update
// ============================================================================
void SkeletonController::update(std::array<Entity, MAX_ENTITIES>& entities,
                                  FpVel substep_dt,
                                  EventQueue& events,
                                  uint32_t frame) {
    FpVel inv_dt = FpVel::one() / substep_dt;
    FpVel stiffness = FpVel::from_float(400.0f);
    FpVel damping   = FpVel::from_float(30.0f);

    for (EntityId id = PLAYER_BEGIN; id < PLAYER_END; ++id) {
        Entity& e = entities[id];
        if (!e.has(COMP_SKELETON | COMP_RIGIDBODY)) continue;
        if (e.has(COMP_SLEEP)) continue;

        Skeleton& sk  = e.skeleton;
        sk.entity_id  = id;

        // Save previous bone positions for velocity computation
        std::array<BoneState, BONE_COUNT> prev_bones = sk.bones;

        // Motor drives: apply torque toward animation targets
        for (uint8_t b = 1; b < BONE_COUNT; ++b) {
            BoneState& bone = sk.bones[b];
            FpVel av = e.rigidbody.angular_velocity.length(); // simplified scalar
            FpVel torque = motor_torque(bone.joint_angle, bone.target_angle,
                                        av, stiffness, damping,
                                        BONE_MAX_TORQUE[b]);
            bone.motor_torque = torque;

            // Apply torque to joint angle (integrate directly for FK)
            bone.joint_angle += torque * substep_dt * substep_dt
                                * FpVel::from_float(1.0f / 75.0f); // 1/mass

            // Clamp to joint limits
            JointType jtype = (b == BONE_HEAD) ? JointType::SPHERICAL
                                                : JointType::REVOLUTE;
            bone.joint_angle = clamp_joint_angle(bone.joint_angle,
                                                  BONE_LIMITS[b], jtype);
        }

        // Forward kinematics pass
        forward_kinematics(sk, e.rigidbody.position, e.rigidbody.orientation);

        // Update bone velocities
        for (uint8_t b = 0; b < BONE_COUNT; ++b) {
            update_bone_velocity(sk.bones[b], prev_bones[b], inv_dt);
        }

        // Foot sensors (ground contact)
        Vec3Vel ground_normal;
        sk.left_foot_grounded  = raycast_ground(sk.bones[BONE_L_FOOT].world_pos,
                                                  entities,
                                                  FpPos::from_float(0.08f),
                                                  ground_normal);
        sk.right_foot_grounded = raycast_ground(sk.bones[BONE_R_FOOT].world_pos,
                                                  entities,
                                                  FpPos::from_float(0.08f),
                                                  ground_normal);

        // Capture foot contact velocities (used for kick impulse)
        sk.left_foot_contact_vel  = sk.bones[BONE_L_FOOT].world_vel;
        sk.right_foot_contact_vel = sk.bones[BONE_R_FOOT].world_vel;
    }
}

// ============================================================================
// Kick impulse computation
// ============================================================================
Vec3Vel SkeletonController::compute_kick_impulse(const KickParams& params,
                                                   const Entity& kicker,
                                                   const Entity& ball) {
    // Base impulse: foot_velocity * power + directional intent
    const Skeleton& sk     = kicker.skeleton;
    bool use_right_foot    = true; // determine from animation state / geometry
    Vec3Vel foot_vel       = use_right_foot ? sk.right_foot_contact_vel
                                            : sk.left_foot_contact_vel;

    // Blend foot velocity with intended direction
    FpVel blend = params.power;  // 0 = full foot vel, 1 = full directional
    Vec3Vel base_impulse = foot_vel * (FpVel::one() - blend)
                         + params.direction * (blend * FpVel::from_float(20.0f));

    // Scale by ball mass: impulse = Δv * m
    return base_impulse * ball.rigidbody.mass;
}

// ============================================================================
// Apply kick
// ============================================================================
void SkeletonController::apply_kick(Entity& ball_entity,
                                     const KickParams& params,
                                     const Entity& kicker,
                                     EventQueue& events,
                                     uint32_t frame) {
    Vec3Vel impulse = compute_kick_impulse(params, kicker, ball_entity);

    // Apply linear impulse to ball
    ball_entity.rigidbody.apply_impulse(impulse, Vec3Vel::zero());

    // Off-centre contact generates spin:
    // Δω = (r × J) * I_inv  where r = contact_offset (from ball centre)
    // Convert contact offset to angular impulse on spin vector
    Vec3Vel spin_impulse = params.contact_offset.cross(impulse);
    // Convert to Q8.24 spin increment
    BallProperties& ball = ball_entity.ball;
    FpVel i_inv = FpVel::one() / (BALL_MASS * BALL_RADIUS * BALL_RADIUS
                                 * FpVel::from_float(0.4f)); // 2/5 mr² for sphere
    ball.spin.x.raw += (spin_impulse.x * i_inv).raw << 8;
    ball.spin.y.raw += (spin_impulse.y * i_inv).raw << 8;
    ball.spin.z.raw += (spin_impulse.z * i_inv).raw << 8;

    // Wake ball (force remove sleep flag, even though ball never sleeps — safety)
    ball_entity.remove(COMP_SLEEP);

    // Emit event
    PhysicsEvent ev;
    ev.type              = EventType::KICK;
    ev.frame             = frame;
    ev.entity_a          = params.kicker_id;
    ev.entity_b          = params.ball_id;
    ev.impulse_magnitude = impulse.length();
    events.push(ev);
}

// ============================================================================
// Jump
// ============================================================================
void SkeletonController::apply_jump(Entity& player_entity,
                                     FpVel impulse_strength) {
    if (!player_entity.has(COMP_RIGIDBODY)) return;
    Vec3Vel jump_impulse = { FpVel::zero(), impulse_strength, FpVel::zero() };
    player_entity.rigidbody.apply_impulse(jump_impulse, Vec3Vel::zero());
}

// ============================================================================
// Set animation target angles (called by animation system each tick)
// ============================================================================
void SkeletonController::set_animation_targets(
    Skeleton& skel, const std::array<FpVel, BONE_COUNT>& targets) {
    for (uint8_t b = 0; b < BONE_COUNT; ++b) {
        skel.bones[b].target_angle = targets[b];
    }
}

} // namespace dspe
