#pragma once
// ============================================================================
// DSPE ECS Components
// ============================================================================
#include "math_types.h"
#include "materials.h"
#include "entity.h"
#include <array>
#include <cstdint>

namespace dspe {

// ---------------------------------------------------------------------------
// Timestep constants
// ---------------------------------------------------------------------------
static constexpr int    SUBSTEPS        = 4;
static constexpr float  OUTER_DT_S      = 1.0f / 60.0f;   // 16.666ms
static constexpr float  SUBSTEP_DT_S    = OUTER_DT_S / SUBSTEPS; // 4.166ms

// Q15.16 representation of substep dt
static const FpVel SUBSTEP_DT = FpVel::from_float(SUBSTEP_DT_S);

// Physics constants
static const FpVel GRAVITY      = FpVel::from_float(9.81f);
static const FpVel AIR_DENSITY  = FpVel::from_float(1.225f);   // kg/m^3 sea level

// Ball constants (FIFA standard)
static const FpVel BALL_MASS    = FpVel::from_float(0.43f);    // kg
static const FpVel BALL_RADIUS  = FpVel::from_float(0.11f);    // m

// Player constants
static const FpVel PLAYER_MASS  = FpVel::from_float(75.0f);    // kg

// Velocity safety clamps.
// These are solver-explosion backstops, not gameplay limits —
// values are set well above any physically reachable quantity.
static const FpVel MAX_SPEED    = FpVel::from_float(200.0f);   // m/s linear

// MAX_ANGULAR_SPEED: safety clamp for RigidBody::angular_velocity (Q15.16, rad/s).
//
// Rationale for 50 rad/s (~8 rev/s):
//   - Maximum realistic player tumble: ~5 rad/s
//   - Brief specifies BallProperties::spin (Q8.24) up to ~100 rad/s, but that
//     is a separate field.  RigidBody::angular_velocity drives orientation
//     integration and the constraint solver; runaway values here cause the
//     same int32 overflow path as MAX_SPEED, so the same int64 guard applies.
//   - 50 rad/s is high enough to never interfere with gameplay while catching
//     any solver explosion before it propagates to the position integrator.
//
// NOTE: (50 * 65536)^2 = ~1.07e13 — overflows int32 exactly like MAX_SPEED^2,
// so clamp_angular_velocity must use the same int64 raw arithmetic as
// clamp_velocity.  Do not use length_sq() / operator* for the threshold test.
static const FpVel MAX_ANGULAR_SPEED = FpVel::from_float(50.0f); // rad/s

// ---------------------------------------------------------------------------
// RigidBody component
// ---------------------------------------------------------------------------
struct RigidBody {
    Vec3Pos position{};
    QuatVel orientation{QuatVel::identity()};

    Vec3Vel velocity{};          // Linear velocity (m/s)
    Vec3Vel angular_velocity{};  // Angular velocity (rad/s), Q15.16
    Vec3Vel force_accum{};       // Accumulated forces for this step
    Vec3Vel torque_accum{};      // Accumulated torques

    Vec3Vel acceleration{};      // a(t) from previous step (for Verlet)

    FpVel   mass{};              // kg
    FpVel   inv_mass{};          // 1/mass; 0 for static bodies

    // Inertia tensor (diagonal approximation — sufficient for capsules/spheres)
    FpVel   inv_inertia_x{};
    FpVel   inv_inertia_y{};
    FpVel   inv_inertia_z{};

    // Drag properties
    FpVel   drag_cd{};           // Drag coefficient
    FpVel   cross_section_area{}; // m^2

    // Apply impulse at world point (for contact resolution)
    void apply_impulse(Vec3Vel impulse, Vec3Vel contact_offset) {
        velocity    += impulse * inv_mass;
        // torque_impulse = contact_offset × impulse
        Vec3Vel ang_impulse = contact_offset.cross(impulse);
        angular_velocity.x += ang_impulse.x * inv_inertia_x;
        angular_velocity.y += ang_impulse.y * inv_inertia_y;
        angular_velocity.z += ang_impulse.z * inv_inertia_z;
    }

    void apply_force(Vec3Vel f) { force_accum += f; }
    void apply_torque(Vec3Vel t) { torque_accum += t; }
    void clear_forces() { force_accum = {}; torque_accum = {}; }

    // Velocity at a specific contact point
    Vec3Vel velocity_at(Vec3Vel offset) const {
        return velocity + angular_velocity.cross(offset);
    }

    bool is_static() const { return inv_mass.raw == 0; }
};

// ---------------------------------------------------------------------------
// Collider component — collision shape descriptor
// ---------------------------------------------------------------------------
struct Capsule {
    Vec3Pos local_base{};    // Base of capsule in local space
    Vec3Pos local_tip{};     // Tip of capsule in local space
    FpPos   radius{};
};

struct Sphere {
    FpPos radius{};
};

struct Box {
    Vec3Pos half_extents{};  // Local half-sizes
};

struct Collider {
    ShapeType shape_type{ShapeType::CAPSULE};
    MaterialId material_id{MAT_DRY_GRASS};

    union {
        Capsule capsule;
        Sphere  sphere;
        Box     box;
    };

    // Computed AABB in world space (updated each broad phase)
    AABB world_aabb{};

    // Proxy AABB for sleep/fast-update (inflated by velocity margin)
    AABB proxy_aabb{};

    // Self-skeleton collision: entity ID of parent skeleton (no self-collide within)
    EntityId skeleton_owner{INVALID_ENTITY};

    Collider() : capsule{} {}
};

// ---------------------------------------------------------------------------
// Bone index constants for 15-bone skeleton
// ---------------------------------------------------------------------------
enum Bone : uint8_t {
    BONE_PELVIS = 0,
    BONE_SPINE,
    BONE_HEAD,
    BONE_L_ARM,       BONE_L_FOREARM,
    BONE_R_ARM,       BONE_R_FOREARM,
    BONE_L_THIGH,     BONE_L_CALF,     BONE_L_FOOT,
    BONE_R_THIGH,     BONE_R_CALF,     BONE_R_FOOT,
    BONE_COUNT = 13,
};

// Parent mapping (-1 = root)
static constexpr int8_t BONE_PARENT[BONE_COUNT] = {
    -1,          // PELVIS
    BONE_PELVIS, // SPINE
    BONE_SPINE,  // HEAD
    BONE_SPINE,  BONE_L_ARM,    // L_ARM, L_FOREARM
    BONE_SPINE,  BONE_R_ARM,    // R_ARM, R_FOREARM
    BONE_PELVIS, BONE_L_THIGH,  BONE_L_CALF,  // L_THIGH, L_CALF, L_FOOT
    BONE_PELVIS, BONE_R_THIGH,  BONE_R_CALF,  // R_THIGH, R_CALF, R_FOOT
};

// Joint types
enum class JointType : uint8_t {
    FREE,        // 6DOF pelvis (root)
    REVOLUTE,    // 1-axis hinge (knee, elbow)
    SPHERICAL,   // 3-axis with cone limit (shoulder, hip, head)
};

// Joint limit (revolute: min/max angle in Q15.16; spherical: cone half-angle)
struct JointLimit {
    FpVel min_angle{};  // radians Q15.16
    FpVel max_angle{};  // radians Q15.16
    FpVel cone_angle{}; // for spherical: half cone in radians Q15.16
};

// Per-bone joint config (static, matches the anatomy spec)
static const JointLimit BONE_LIMITS[BONE_COUNT] = {
    // PELVIS — free
    {},
    // SPINE — revolute, small range
    { FpVel::from_float(-0.35f), FpVel::from_float(0.35f) },
    // HEAD — spherical, 60° cone
    { {}, {}, FpVel::from_float(1.047f) },
    // L_ARM — revolute (shoulder simplified to 1-axis)
    { FpVel::from_float(-2.09f), FpVel::from_float(2.09f) },
    // L_FOREARM — revolute (elbow 0–145°)
    { FpVel::from_float(0.0f),   FpVel::from_float(2.53f) },
    // R_ARM
    { FpVel::from_float(-2.09f), FpVel::from_float(2.09f) },
    // R_FOREARM
    { FpVel::from_float(0.0f),   FpVel::from_float(2.53f) },
    // L_THIGH — revolute (hip flexion)
    { FpVel::from_float(-0.52f), FpVel::from_float(2.09f) },
    // L_CALF — revolute (knee 0–140°)
    { FpVel::from_float(0.0f),   FpVel::from_float(2.44f) },
    // L_FOOT — hinge (dorsiflexion -20° to +50°)
    { FpVel::from_float(-0.35f), FpVel::from_float(0.87f) },
    // R_THIGH
    { FpVel::from_float(-0.52f), FpVel::from_float(2.09f) },
    // R_CALF
    { FpVel::from_float(0.0f),   FpVel::from_float(2.44f) },
    // R_FOOT
    { FpVel::from_float(-0.35f), FpVel::from_float(0.87f) },
};

// ---------------------------------------------------------------------------
// Skeleton component — one per player entity
// ---------------------------------------------------------------------------
struct BoneState {
    Vec3Pos world_pos{};
    QuatVel world_rot{QuatVel::identity()};
    Vec3Vel world_vel{};      // bone tip velocity (for contact response)
    FpVel   joint_angle{};    // current joint angle (revolute joints)
    FpVel   target_angle{};   // animation target angle
    FpVel   motor_torque{};   // torque applied by motor this frame
    Capsule collider_local{};
};

struct Skeleton {
    std::array<BoneState, BONE_COUNT> bones{};
    EntityId entity_id{INVALID_ENTITY};

    // Foot contact sensors
    bool left_foot_grounded{false};
    bool right_foot_grounded{false};

    // Foot velocity at last contact (used for kick impulse)
    Vec3Vel left_foot_contact_vel{};
    Vec3Vel right_foot_contact_vel{};

    // Animation state (simple blend tree)
    enum class AnimState : uint8_t {
        IDLE, RUN, JUMP, FALL, TACKLE, KICK
    } anim_state{AnimState::IDLE};

    float anim_blend{0.0f}; // blend factor (rendering only, not physics)
};

// ---------------------------------------------------------------------------
// Ball properties component
// ---------------------------------------------------------------------------
struct BallProperties {
    Vec3Ang spin{};           // Angular velocity ω (Q8.24 rad/s)
    FpVel   magnus_cl{};      // Lift coefficient C_L (tuned vs Goff & Carré)
    FpVel   drag_cd{};        // Ball drag coefficient
    FpVel   spin_decay_k{};   // Rotational drag constant k_rot

    // Surface material this frame (updated on first contact)
    MaterialId surface_material{MAT_DRY_GRASS};

    // Last contact normal (for spin generation on bounce)
    Vec3Vel last_contact_normal{};

    static BallProperties make_default() {
        BallProperties b;
        b.magnus_cl     = FpVel::from_float(1.0f);
        b.drag_cd       = FpVel::from_float(0.25f);
        b.spin_decay_k  = FpVel::from_float(0.0002f);
        return b;
    }
};

// ---------------------------------------------------------------------------
// Trigger volume component (no collision response, events only)
// ---------------------------------------------------------------------------
enum class TriggerEventType : uint8_t {
    GOAL_LEFT,
    GOAL_RIGHT,
    OUT_OF_BOUNDS,
    BALL_IN_BOX_LEFT,
    BALL_IN_BOX_RIGHT,
    KICKOFF_INFRINGEMENT,
};

struct TriggerVolume {
    AABB            bounds{};
    TriggerEventType event_type{};
    bool            inverted{false};  // Fire when entity is OUTSIDE bounds
    bool            ball_only{true};  // Only trigger on ball entity

    // Fire if entity is inside (or outside if inverted)
    bool test(Vec3Pos entity_pos) const {
        bool inside = bounds.contains(entity_pos);
        return inverted ? !inside : inside;
    }
};

// ---------------------------------------------------------------------------
// Sleep state component
// ---------------------------------------------------------------------------
struct SleepState {
    uint32_t ticks_below_threshold{0};
    static constexpr uint32_t SLEEP_TICKS        = 30;
    static constexpr FpVel    LINEAR_THRESHOLD   = FpVel::from_float(0.01f);
    static constexpr FpVel    ANGULAR_THRESHOLD  = FpVel::from_float(0.01f);
    static constexpr FpPos    WAKE_RADIUS        = FpPos::from_float(1.0f);
};

// ---------------------------------------------------------------------------
// Full entity record (SoA-friendly flat array)
// ---------------------------------------------------------------------------
struct Entity {
    uint32_t    component_mask{COMP_NONE};
    RigidBody   rigidbody{};
    Collider    collider{};
    Skeleton    skeleton{};
    BallProperties ball{};
    TriggerVolume  trigger{};
    SleepState  sleep{};

    bool has(uint32_t f) const { return (component_mask & f) != 0; }
    void add(uint32_t f) { component_mask |= f; }
    void remove(uint32_t f) { component_mask &= ~f; }
};

} // namespace dspe