#pragma once
// =============================================================================
// apc_ai_steering.h — Steering behaviors for AI-driven athletes
// =============================================================================
//
// Provides classic steering behaviors for AI-driven athletes:
//
//   SteeringBehavior: enumeration of all supported steering types
//   SteeringOutput:   desired acceleration + angular torque + urgency
//   WeightedSteering: behavior + weight pair for composite blending
//   SteeringRequest:  tunable parameters for all steering behaviors
//   SteeringSystem:   stateless per-behavior calculations + blend
//
// All steering operations work in the XZ plane (Y = 0 for ground movement).
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation (fixed-size arrays)
//   - Deterministic: same inputs -> same outputs
//   - C++17
//
// =============================================================================

#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include "apc_math/apc_math_common.h"
#include <cstdint>
#include <cmath>

namespace apc {

// =============================================================================
// Capacity constants
// =============================================================================
static constexpr uint32_t MAX_STEERING_BEHAVIORS = 8;
static constexpr uint32_t MAX_NEIGHBOR_COUNT     = 22;

// =============================================================================
// SteeringBehavior — Classification of AI steering types
// =============================================================================
enum class SteeringBehavior : uint8_t {
    SEEK           = 0,
    FLEE           = 1,
    ARRIVE         = 2,
    PURSUE         = 3,
    EVADE          = 4,
    WANDER         = 5,
    ALIGN          = 6,
    FACE           = 7,
    SEPARATION     = 8,
    COHESION       = 9,
    LEADER_FOLLOW  = 10,
    PATH_FOLLOWING = 11,
    PREDICTION     = 12
};

// =============================================================================
// SteeringOutput — Result of a single steering calculation
// =============================================================================
struct SteeringOutput {
    Vec3  linear  = { 0.0f, 0.0f, 0.0f }; // Desired acceleration (XZ plane)
    Vec3  angular = { 0.0f, 0.0f, 0.0f }; // Desired torque
    float urgency = 0.0f;                    // 0.0-1.0 importance weight
};

// =============================================================================
// WeightedSteering — Behavior + weight pair for composite blending
// =============================================================================
struct WeightedSteering {
    SteeringBehavior behavior = SteeringBehavior::SEEK;
    float              weight  = 1.0f;
    SteeringOutput     output;
};

// =============================================================================
// SteeringRequest — Tunable parameters for steering behaviors
// =============================================================================
struct SteeringRequest {
    Vec3  target_position  = { 0.0f, 0.0f, 0.0f };
    Vec3  target_velocity  = { 0.0f, 0.0f, 0.0f }; // For pursue/evade
    float arrive_radius     = 1.0f;
    float arrive_slow_radius = 3.0f;
    float wander_radius     = 2.0f;
    float wander_offset     = 2.0f;
    float wander_rate       = 1.5f;
    float separation_radius = 1.5f;
    float separation_weight = 2.0f;
    float max_speed         = 8.0f;
    float max_force         = 20.0f;
};

// =============================================================================
// SteeringSystem — Stateless steering behavior calculations
// =============================================================================
struct SteeringSystem {

    // =========================================================================
    // Individual Steering Behaviors (all static, no state)
    // =========================================================================

    // --- Seek: desired velocity toward target, clamped to max_speed ---
    static SteeringOutput seek(const Vec3& position, const Vec3& target,
                                float max_speed)
    {
        SteeringOutput out;
        Vec3 desired = Vec3::sub(target, position);
        desired.y = 0.0f;

        float len = Vec3::length(desired);
        if (len > APC_EPSILON) {
            desired = Vec3::scale(Vec3::scale(desired, 1.0f / len), max_speed);
        }

        out.linear = truncate(desired, max_speed);
        out.linear.y = 0.0f;
        return out;
    }

    // --- Flee: inverse of seek, only active within panic_radius ---
    static SteeringOutput flee(const Vec3& position, const Vec3& threat,
                                float max_speed, float panic_radius)
    {
        SteeringOutput out;

        Vec3 diff = Vec3::sub(position, threat);
        diff.y = 0.0f;
        float dist = Vec3::length(diff);

        if (dist > panic_radius || dist < APC_EPSILON) {
            return out; // Outside panic range or degenerate
        }

        Vec3 desired = Vec3::scale(diff, max_speed / dist);
        desired.y = 0.0f;

        out.linear = truncate(desired, max_speed);
        out.linear.y = 0.0f;
        return out;
    }

    // --- Arrive: like seek but slows within slow_radius, stops at arrive ---
    static SteeringOutput arrive(const Vec3& position, const Vec3& target,
                                  float max_speed, float arrive_radius,
                                  float slow_radius)
    {
        SteeringOutput out;
        Vec3 diff = Vec3::sub(target, position);
        diff.y = 0.0f;
        float dist = Vec3::length(diff);

        if (dist < arrive_radius) {
            return out; // Already arrived
        }

        // Determine desired speed based on distance
        float desired_speed = max_speed;
        if (dist < slow_radius) {
            desired_speed = max_speed * (dist / slow_radius);
        }

        Vec3 desired = (dist > APC_EPSILON)
            ? Vec3::scale(Vec3::scale(diff, 1.0f / dist), desired_speed)
            : Vec3(0.0f, 0.0f, 0.0f);
        desired.y = 0.0f;

        out.linear = truncate(desired, max_speed);
        out.linear.y = 0.0f;
        return out;
    }

    // --- Pursue: predict future target position and seek it ---
    static SteeringOutput pursue(const Vec3& position, const Vec3& velocity,
                                  const Vec3& target_pos,
                                  const Vec3& target_vel,
                                  float max_speed,
                                  float max_prediction_time)
    {
        (void)velocity; // Reserved for velocity-matching variants

        Vec3 to_target = Vec3::sub(target_pos, position);
        to_target.y = 0.0f;
        float dist = Vec3::length(to_target);

        // Look-ahead time based on distance / max_speed, capped
        float look_ahead = dist / (max_speed + APC_EPSILON);
        if (look_ahead > max_prediction_time) {
            look_ahead = max_prediction_time;
        }

        Vec3 future_target = Vec3::add(target_pos,
            Vec3::scale(target_vel, look_ahead));
        future_target.y = target_pos.y;

        return seek(position, future_target, max_speed);
    }

    // --- Evade: inverse of pursue (flee predicted future threat position) ---
    static SteeringOutput evade(const Vec3& position, const Vec3& velocity,
                                 const Vec3& threat_pos,
                                 const Vec3& threat_vel,
                                 float max_speed,
                                 float max_prediction_time)
    {
        (void)velocity;

        Vec3 to_threat = Vec3::sub(threat_pos, position);
        to_threat.y = 0.0f;
        float dist = Vec3::length(to_threat);

        float look_ahead = dist / (max_speed + APC_EPSILON);
        if (look_ahead > max_prediction_time) {
            look_ahead = max_prediction_time;
        }

        Vec3 future_threat = Vec3::add(threat_pos,
            Vec3::scale(threat_vel, look_ahead));
        future_threat.y = threat_pos.y;

        return flee(position, future_threat, max_speed,
                    max_prediction_time * max_speed * 2.0f);
    }

    // --- Wander: circle-ahead wander with deterministic deviation ---
    static SteeringOutput wander(const Vec3& position,
                                  const Vec3& heading,
                                  float wander_radius,
                                  float wander_offset,
                                  float wander_rate,
                                  float max_speed)
    {
        SteeringOutput out;

        float speed = Vec3::length(heading);
        Vec3 dir = (speed > APC_EPSILON)
            ? Vec3::scale(heading, 1.0f / speed)
            : Vec3(0.0f, 0.0f, 1.0f);

        // Wander center is ahead of the athlete
        Vec3 center = Vec3::add(position,
            Vec3::scale(dir, wander_offset));
        center.y = position.y;

        // Wander deviation on the circle (deterministic angle from speed)
        float angle_offset = speed * wander_rate;
        Vec3 offset(
            std::cos(angle_offset) * wander_radius,
            0.0f,
            std::sin(angle_offset) * wander_radius
        );

        Vec3 target = Vec3::add(center, offset);
        target.y = position.y;

        Vec3 desired = Vec3::sub(target, position);
        desired.y = 0.0f;

        float len = Vec3::length(desired);
        if (len > APC_EPSILON) {
            desired = Vec3::scale(Vec3::scale(desired, 1.0f / len),
                                  max_speed);
        }

        out.linear = truncate(desired, max_speed);
        out.linear.y = 0.0f;
        out.angular = Vec3(0.0f, wander_rate, 0.0f);
        return out;
    }

    // --- Separation: repulsion from neighbors within separation_radius ---
    static SteeringOutput separation(const Vec3* neighbors, uint32_t count,
                                      const Vec3& position,
                                      float separation_radius,
                                      float max_force)
    {
        SteeringOutput out;
        if (count == 0u || neighbors == nullptr) {
            return out;
        }

        Vec3 repulsion(0.0f, 0.0f, 0.0f);

        for (uint32_t i = 0u; i < count; ++i) {
            Vec3 diff = Vec3::sub(position, neighbors[i]);
            diff.y = 0.0f;
            float dist = Vec3::length(diff);

            if (dist < separation_radius && dist > APC_EPSILON) {
                // Stronger repulsion when closer (inverse linear)
                float strength = (1.0f - dist / separation_radius);
                Vec3 dir = Vec3::scale(diff, 1.0f / dist);
                repulsion = Vec3::add(repulsion, Vec3::scale(dir, strength));
            }
        }

        out.linear = truncate(repulsion, max_force);
        out.linear.y = 0.0f;
        return out;
    }

    // --- Cohesion: steer toward center of mass of neighbors ---
    static SteeringOutput cohesion(const Vec3* neighbors, uint32_t count,
                                    const Vec3& position,
                                    float max_speed)
    {
        SteeringOutput out;
        if (count == 0u || neighbors == nullptr) {
            return out;
        }

        Vec3 center(0.0f, 0.0f, 0.0f);
        for (uint32_t i = 0u; i < count; ++i) {
            center = Vec3::add(center, neighbors[i]);
        }
        float inv = 1.0f / static_cast<float>(count);
        center = Vec3::scale(center, inv);
        center.y = position.y;

        return seek(position, center, max_speed);
    }

    // --- Alignment: steer to match average heading of neighbors ---
    // NOTE: neighbors[] contains positions, not velocities. Without access
    // to neighbor velocities, we approximate alignment by steering toward
    // the average position of nearby agents (cohesion-like behavior).
    // True velocity-based alignment requires a neighbor velocity query.
    static SteeringOutput alignment(const Vec3* neighbors, uint32_t count,
                                     const Vec3& /*heading*/,
                                     float /*max_force*/)
    {
        SteeringOutput out;
        // Without neighbor velocity data, alignment is a no-op.
        // The calling code should use cohesion + separation for flocking.
        // TODO: Pass neighbor velocities when EntityManager supports it.
        (void)count;
        (void)neighbors;
        return out;
    }

    // --- Face: compute angular steering to face a target ---
    static SteeringOutput face(const Quat& orientation, const Vec3& target,
                                const Vec3& position, float max_rotation)
    {
        SteeringOutput out;

        Vec3 to_target = Vec3::sub(target, position);
        to_target.y = 0.0f;
        float dist = Vec3::length(to_target);

        if (dist < APC_EPSILON) {
            return out;
        }

        // Desired facing direction
        Vec3 desired = Vec3::scale(to_target, 1.0f / dist);

        // Current facing direction from quaternion (forward = -Z convention)
        Vec3 forward(0.0f, 0.0f, -1.0f);
        Vec3 current = orientation.rotate(forward);
        current.y = 0.0f;
        float cur_len = Vec3::length(current);
        if (cur_len > APC_EPSILON) {
            current = Vec3::scale(current, 1.0f / cur_len);
        }

        // Cross product gives rotation axis (Y component only for XZ plane)
        float cross_y = current.x * desired.z - current.z * desired.x;

        // Dot product for angle
        float dot = Vec3::dot(current, desired);
        if (dot > 1.0f) dot = 1.0f;
        if (dot < -1.0f) dot = -1.0f;
        float angle = std::acos(dot);

        // Clamp rotation speed
        if (angle > max_rotation) {
            angle = max_rotation;
        }

        // Determine rotation direction from cross product
        float rotation = (cross_y > 0.0f) ? angle : -angle;

        out.angular = Vec3(0.0f, rotation, 0.0f);
        return out;
    }

    // =========================================================================
    // Composite Blending
    // =========================================================================

    // --- Blend: weighted average of multiple steering outputs ---
    static SteeringOutput blend(const WeightedSteering* behaviors, uint32_t count)
    {
        SteeringOutput result;

        if (count == 0u || behaviors == nullptr) {
            return result;
        }

        float total_weight = 0.0f;

        for (uint32_t i = 0u; i < count; ++i) {
            float w = behaviors[i].weight;
            result.linear  = Vec3::add(result.linear,
                Vec3::scale(behaviors[i].output.linear, w));
            result.angular = Vec3::add(result.angular,
                Vec3::scale(behaviors[i].output.angular, w));
            result.urgency += behaviors[i].output.urgency * w;
            total_weight += w;
        }

        if (total_weight > APC_EPSILON) {
            float inv = 1.0f / total_weight;
            result.linear  = Vec3::scale(result.linear, inv);
            result.angular = Vec3::scale(result.angular, inv);
            result.urgency *= inv;
        } else {
            result.linear  = Vec3(0.0f, 0.0f, 0.0f);
            result.angular = Vec3(0.0f, 0.0f, 0.0f);
            result.urgency = 0.0f;
        }

        // Ensure output is on XZ plane
        result.linear.y = 0.0f;

        return result;
    }

private:
    // --- Truncate: clamp vector magnitude ---
    APC_FORCEINLINE static Vec3 truncate(Vec3 v, float max_mag) {
        float len_sq = Vec3::length_sq(v);
        if (len_sq > max_mag * max_mag) {
            float len = std::sqrt(len_sq);
            if (len > APC_EPSILON) {
                float inv_len = max_mag / len;
                return Vec3::scale(v, inv_len);
            }
        }
        return v;
    }
};

} // namespace apc
