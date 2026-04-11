#pragma once
// =============================================================================
// apc_rigid_body.h — Rigid body with state flags, sleep system, and CCD support
// =============================================================================
//
// Core rigid body representation for the physics solver pipeline:
//
//   BodyStateFlags:  Bitfield enum for efficient filtering in hot loops
//   RigidBody:       Position, velocity, inertia, collision layer, state
//
// Sleep System:
//   Bodies track frames at rest. When both linear and angular velocity stay
//   below a threshold for N consecutive frames, the body enters STATE_SLEEPING.
//   Sleeping bodies are skipped in the solver and integration, saving CPU.
//   Any impulse (apply_impulse) or explicit wake_up() reactivates the body.
//
// CCD (Continuous Collision Detection):
//   Bodies flagged STATE_CCD_ENABLED are candidates for swept-sphere checks.
//   The bounding_radius field is used to determine if the body moves more than
//   its own radius per frame (tunneling risk).
//
// Design:
//   - Header-only, apc:: namespace
//   - Deterministic: same inputs -> same outputs
//   - Cache-friendly: state_flags uses bitwise operations, no branching
//   - C++17
//
// =============================================================================

#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include "apc_math/apc_mat3.h"
#include "apc_math/apc_math_common.h"
#include <cstdint>

namespace apc {

// =============================================================================
// BodyStateFlags — Bitfield for efficient body state filtering
// =============================================================================
// Uses a uint32_t bitmask so that multiple state checks can be combined
// into a single bitwise test in hot loops (e.g. solver iteration).
//
// Usage:
//   if (body.state_flags & STATE_SLEEPING) continue;  // skip sleeping
//   if (body.state_flags & STATE_IS_BALL) { ... }     // ball-specific logic
//   body.state_flags |= STATE_CCD_ENABLED;            // enable CCD
//
enum BodyStateFlags : uint32_t {
    STATE_ACTIVE         = 1u << 0u,  ///< Body participates in simulation
    STATE_SLEEPING       = 1u << 1u,  ///< Body is asleep (zero velocity, skip solve/integrate)
    STATE_KINEMATIC      = 1u << 2u,  ///< Body has infinite mass, driven externally
    STATE_IS_BALL        = 1u << 3u,  ///< Body is a sport ball (triggers extra solver iterations)
    STATE_CCD_ENABLED    = 1u << 4u,  ///< Body should be checked for tunneling (CCD)
    STATE_IGNORE_GRAVITY = 1u << 5u   ///< Body is not affected by gravity
};

// =============================================================================
// RigidBody — Core physics body for the solver pipeline
// =============================================================================
struct RigidBody {
    // --- Identity ---
    uint32_t body_id = 0u;  ///< Index into the solver's body array

    // --- Transform ---
    Vec3 position;
    Vec3 linear_velocity;
    float inverse_mass = 1.0f;  ///< 0.0 = infinite mass (static/kinematic)

    Quat orientation;
    Vec3 angular_velocity;

    Mat3 local_inverse_inertia;
    Mat3 world_inverse_inertia;

    // --- Collision filtering (bitfield) ---
    uint32_t collision_layer = 1u;         ///< Which layer(s) this body is ON
    uint32_t collision_mask  = 0xFFFFFFFFu; ///< Which layer(s) it collides WITH

    // --- State flags (bitfield for cache-friendly filtering) ---
    uint32_t state_flags = STATE_ACTIVE;

    // --- Sleep system ---
    uint32_t frames_at_rest = 0u;  ///< Consecutive frames below velocity threshold

    // --- CCD ---
    float bounding_radius = 0.5f;  ///< Bounding sphere radius for CCD threshold

    // =========================================================================
    // State queries (inline, branchless where possible)
    // =========================================================================

    APC_FORCEINLINE bool is_active() const {
        return (state_flags & STATE_ACTIVE) != 0u;
    }

    APC_FORCEINLINE bool is_sleeping() const {
        return (state_flags & STATE_SLEEPING) != 0u;
    }

    APC_FORCEINLINE bool is_kinematic() const {
        return (state_flags & STATE_KINEMATIC) != 0u;
    }

    APC_FORCEINLINE bool is_ball() const {
        return (state_flags & STATE_IS_BALL) != 0u;
    }

    APC_FORCEINLINE bool is_ccd_enabled() const {
        return (state_flags & STATE_CCD_ENABLED) != 0u;
    }

    // =========================================================================
    // Sleep / Wake
    // =========================================================================

    /// Force the body awake. Resets rest counter and clears sleep flag.
    APC_FORCEINLINE void wake_up() {
        state_flags = (state_flags & ~STATE_SLEEPING) | STATE_ACTIVE;
        frames_at_rest = 0u;
    }

    /// Check if the body should enter sleep state based on velocity thresholds.
    /// Kinematic bodies never sleep. If both linear and angular velocity
    /// stay below their thresholds for `frame_threshold` consecutive frames,
    /// the body enters STATE_SLEEPING and velocities are zeroed.
    ///
    /// Returns true if the body just entered sleep state this call.
    APC_FORCEINLINE bool try_sleep(float linear_vel_sq_threshold,
                                    float angular_vel_sq_threshold,
                                    uint32_t frame_threshold)
    {
        if (state_flags & STATE_KINEMATIC) {
            return false;  // Kinematic bodies never sleep
        }

        // Compute squared velocities (no sqrt needed)
        const float lin_sq = Vec3::length_sq(linear_velocity);
        const float ang_sq = Vec3::length_sq(angular_velocity);

        if (lin_sq < linear_vel_sq_threshold &&
            ang_sq < angular_vel_sq_threshold)
        {
            ++frames_at_rest;
            if (frames_at_rest >= frame_threshold) {
                // Enter sleep: zero velocities, set sleeping flag
                linear_velocity = Vec3(0.0f, 0.0f, 0.0f);
                angular_velocity = Vec3(0.0f, 0.0f, 0.0f);
                state_flags = (state_flags & ~STATE_ACTIVE) | STATE_SLEEPING;
                return true;
            }
        } else {
            // Still moving — reset rest counter
            frames_at_rest = 0u;
            if (state_flags & STATE_SLEEPING) {
                // Was sleeping but now moving: wake up
                state_flags = (state_flags & ~STATE_SLEEPING) | STATE_ACTIVE;
            }
        }
        return false;
    }

    // =========================================================================
    // Collision filtering
    // =========================================================================

    /// Check if two bodies should collide based on layer/mask bitfields.
    /// Also skips collision if both bodies are sleeping (they're at rest).
    static bool should_collide(const RigidBody& a, const RigidBody& b) {
        // Two sleeping bodies don't collide (both at zero velocity)
        if ((a.state_flags & STATE_SLEEPING) &&
            (b.state_flags & STATE_SLEEPING)) {
            return false;
        }
        return (a.collision_layer & b.collision_mask) != 0u &&
               (b.collision_layer & a.collision_mask) != 0u;
    }

    // =========================================================================
    // Inertia
    // =========================================================================

    void update_world_inertia() {
        Mat3 rot = Mat3::from_quat(orientation);
        Mat3 inv_rot = rot.transpose();
        world_inverse_inertia = Mat3::multiply(rot, Mat3::multiply(local_inverse_inertia, inv_rot));
    }

    // =========================================================================
    // Impulse application (auto-wakes sleeping bodies)
    // =========================================================================

    void apply_impulse(const Vec3& impulse, const Vec3& contact_point) {
        // Any impulse wakes the body
        if (state_flags & STATE_SLEEPING) {
            wake_up();
        }

        linear_velocity = Vec3::add(linear_velocity, Vec3::scale(impulse, inverse_mass));

        Vec3 r = Vec3::sub(contact_point, position);
        Vec3 torque_impulse = Vec3::cross(r, impulse);

        // Manual Mat3 * Vec3 multiply (avoids needing operator overloads)
        const Mat3& m = world_inverse_inertia;
        Vec3 angular_impulse(
            torque_impulse.x * m.m[0] + torque_impulse.y * m.m[3] + torque_impulse.z * m.m[6],
            torque_impulse.x * m.m[1] + torque_impulse.y * m.m[4] + torque_impulse.z * m.m[7],
            torque_impulse.x * m.m[2] + torque_impulse.y * m.m[5] + torque_impulse.z * m.m[8]
        );

        angular_velocity = Vec3::add(angular_velocity, angular_impulse);
    }
};

} // namespace apc
