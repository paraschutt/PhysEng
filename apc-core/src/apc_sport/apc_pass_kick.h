#pragma once
// =============================================================================
// Pass & Kick — Kicking, throwing, passing mechanics for all sports
// =============================================================================
//
// Provides sport-specific mechanics for ball propulsion:
//
//   FOOT:
//     - Soccer kick (instep, side-foot, chip, curl)
//     - Rugby punt, place kick, drop kick
//     - Aussie rules drop punt, torpedo
//
//   HAND:
//     - Basketball chest pass, bounce pass, overhead pass
//     - American football throw (spiral), lateral
//     - Handball throw
//     - Rugby pass (spin, pop)
//
//   GENERAL:
//     - Power model: charge time → kick/throw power
//     - Accuracy model: skill → direction scatter
//     - Spin application: type + rate → angular velocity
//     - Trajectory prediction: future ball position estimation
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation
//   - Deterministic: same inputs → same outputs
//   - C++17
//
// =============================================================================

#include "apc_sport/apc_ball_physics.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include "apc_math/apc_math_common.h"
#include <cstdint>
#include <cmath>

namespace apc {

// =============================================================================
// KickType — Types of foot strikes
// =============================================================================
enum class KickType : uint8_t {
    INSTEP       = 0,   // Soccer instep kick (powerful, straight)
    SIDE_FOOT    = 1,   // Soccer side-foot (accurate, less power)
    CHIP         = 2,   // Soccer chip (lofted, short)
    CURVE        = 3,   // Soccer curl (sidespin, bending)
    LOB          = 4,   // Soccer lob (high arc, drops steeply)
    PUNT         = 5,   // Rugby/American football punt (dropped from hands)
    DROP_KICK    = 6,   // Rugby drop kick (bounces off ground)
    PLACE_KICK   = 7,   // Rugby/American football place kick (tee)
    DROP_PUNT    = 8,   // Aussie rules drop punt (end-over-end)
    TORPEDO      = 9,   // Aussie rules torpedo punt (spinning spiral)
    VOLLEY       = 10,  // Soccer volley (ball in air, full swing)
    HALF_VOLLEY  = 11,  // Soccer half-volley (ball just bouncing)
    BICYCLE      = 12,  // Soccer bicycle/overhead kick (dramatic)
    PANNA        = 13,  // Soccer nutmeg (through legs)
    RABONA       = 14   // Soccer rabona (crossed-leg kick)
};

// =============================================================================
// ThrowType — Types of hand throws
// =============================================================================
enum class ThrowType : uint8_t {
    CHEST_PASS   = 0,   // Basketball chest pass (two hands)
    BOUNCE_PASS  = 1,   // Basketball bounce pass
    OVERHEAD     = 2,   // Basketball overhead pass
    ONE_HAND     = 3,   // Basketball one-hand push
    SPIRAL       = 4,   // American football spiral pass
    LOB_THROW    = 5,   // American football lob/floater
    LATERAL      = 6,   // American football lateral (side pitch)
    SHOT_PUT     = 7,   // Rugby spin pass (tight spiral)
    POP_PASS     = 8,   // Rugby pop pass (lofted, short)
    HAND_THROW   = 9,   // Handball/armball throw (overhead)
    BASEBALL     = 10,  // Overhand baseball throw
    UNDERHAND    = 11,  // Cricket underhand/lob
    SHOVEL       = 12,  // American football shovel pass
    HOOK_PASS    = 13   // Rugby hook pass (around the back)
};

// =============================================================================
// PassResult — Result of a kick or throw
// =============================================================================
struct PassResult {
    Vec3 launch_velocity;          // Initial ball velocity after kick/throw
    Vec3 launch_position;          // Ball position at launch
    Vec3 target_direction;         // Intended direction (pre-scatter)
    float power = 0.0f;            // Applied power (0-1)
    float speed = 0.0f;            // Resulting ball speed (m/s)
    float spin_rate = 0.0f;        // Applied spin rate (rad/s)
    Vec3 spin_axis;                // Spin axis in world space
    SpinAxis spin_type;            // Semantic spin type
    float accuracy_error = 0.0f;   // Direction error in radians
    float launch_angle = 0.0f;     // Vertical launch angle (radians)
    bool executed = false;         // Was the kick/throw actually performed?
};

// =============================================================================
// KickProfile — Per-kick-type parameters
// =============================================================================
struct KickProfile {
    KickType type = KickType::INSTEP;
    const char* name = "instep";

    float max_power = 30.0f;           // Max ball speed (m/s)
    float min_power = 2.0f;            // Minimum useful speed
    float power_charge_rate = 20.0f;   // Power units per second of charge
    float charge_time_to_max = 1.5f;   // Seconds to reach max power

    float base_accuracy = 0.9f;        // Direction accuracy (0-1)
    float power_accuracy_penalty = 0.2f; // Accuracy loss at max power

    float default_launch_angle = 0.15f; // Default vertical angle (rad, ~8.5 deg)
    float max_launch_angle = 1.2f;     // Maximum loft angle (rad, ~69 deg)
    float angle_adjustability = 0.5f;  // How much angle can be adjusted

    float spin_rate_factor = 1.0f;     // Spin rate multiplier
    SpinAxis default_spin = SpinAxis::SIDESPIN;
    float spin_accuracy = 0.8f;        // How accurately spin is applied

    float foot_velocity_factor = 1.5f; // Ball speed / foot speed ratio
    float sweetspot_radius = 0.04f;    // Ideal contact zone (m)

    // --- Factory methods for standard kicks ---

    static KickProfile make_instep() {
        KickProfile p;
        p.type = KickType::INSTEP;
        p.name = "instep";
        p.max_power = 35.0f;       // Professional instep: ~35 m/s (126 km/h)
        p.base_accuracy = 0.85f;
        p.power_accuracy_penalty = 0.25f;
        p.default_launch_angle = 0.1f;  // Low drive
        p.spin_rate_factor = 0.5f;
        p.default_spin = SpinAxis::SIDESPIN;
        p.foot_velocity_factor = 1.4f;
        return p;
    }

    static KickProfile make_side_foot() {
        KickProfile p;
        p.type = KickType::SIDE_FOOT;
        p.name = "side_foot";
        p.max_power = 22.0f;       // Accurate but less powerful
        p.base_accuracy = 0.95f;   // Very accurate
        p.power_accuracy_penalty = 0.1f;
        p.default_launch_angle = 0.05f; // Very low, driven
        p.spin_rate_factor = 0.2f;
        p.default_spin = SpinAxis::SIDESPIN;
        p.foot_velocity_factor = 1.2f;
        return p;
    }

    static KickProfile make_chip() {
        KickProfile p;
        p.type = KickType::CHIP;
        p.name = "chip";
        p.max_power = 18.0f;
        p.base_accuracy = 0.8f;
        p.power_accuracy_penalty = 0.15f;
        p.default_launch_angle = 0.8f;   // ~46 degrees
        p.max_launch_angle = 1.2f;
        p.spin_rate_factor = 1.2f;
        p.default_spin = SpinAxis::BACKSPIN;
        p.foot_velocity_factor = 1.3f;
        return p;
    }

    static KickProfile make_curve() {
        KickProfile p;
        p.type = KickType::CURVE;
        p.name = "curve";
        p.max_power = 28.0f;
        p.base_accuracy = 0.75f;
        p.power_accuracy_penalty = 0.3f;
        p.default_launch_angle = 0.2f;
        p.spin_rate_factor = 2.5f;      // Heavy sidespin
        p.default_spin = SpinAxis::SIDESPIN;
        p.spin_accuracy = 0.7f;
        p.foot_velocity_factor = 1.3f;
        return p;
    }

    static KickProfile make_punt() {
        KickProfile p;
        p.type = KickType::PUNT;
        p.name = "punt";
        p.max_power = 40.0f;       // Long punt
        p.base_accuracy = 0.7f;
        p.power_accuracy_penalty = 0.2f;
        p.default_launch_angle = 0.6f;  // ~34 degrees (good hang time)
        p.spin_rate_factor = 0.8f;
        p.default_spin = SpinAxis::BACKSPIN;
        p.foot_velocity_factor = 1.3f;
        return p;
    }

    static KickProfile make_torpedo() {
        KickProfile p;
        p.type = KickType::TORPEDO;
        p.name = "torpedo";
        p.max_power = 45.0f;       // Very long kick
        p.base_accuracy = 0.65f;
        p.power_accuracy_penalty = 0.15f;
        p.default_launch_angle = 0.2f;
        p.spin_rate_factor = 3.0f;
        p.default_spin = SpinAxis::HELICAL;
        p.foot_velocity_factor = 1.5f;
        return p;
    }

    static KickProfile make_volley() {
        KickProfile p;
        p.type = KickType::VOLLEY;
        p.name = "volley";
        p.max_power = 38.0f;       // Powerful airborne strike
        p.base_accuracy = 0.7f;
        p.power_accuracy_penalty = 0.3f;
        p.default_launch_angle = 0.15f;
        p.spin_rate_factor = 1.0f;
        p.default_spin = SpinAxis::SIDESPIN;
        p.foot_velocity_factor = 1.6f;
        return p;
    }
};

// =============================================================================
// ThrowProfile — Per-throw-type parameters
// =============================================================================
struct ThrowProfile {
    ThrowType type = ThrowType::CHEST_PASS;
    const char* name = "chest_pass";

    float max_power = 15.0f;
    float min_power = 1.0f;
    float power_charge_rate = 15.0f;
    float charge_time_to_max = 1.0f;

    float base_accuracy = 0.9f;
    float power_accuracy_penalty = 0.15f;

    float default_launch_angle = 0.0f;   // Flat for most passes
    float max_launch_angle = 1.0f;
    float min_launch_angle = -0.3f;      // Can throw slightly downward

    float spin_rate_factor = 1.0f;
    SpinAxis default_spin = SpinAxis::BACKSPIN;

    float release_speed_factor = 1.0f;   // Ball speed / hand speed ratio

    // --- Factory methods for standard throws ---

    static ThrowProfile make_chest_pass() {
        ThrowProfile p;
        p.type = ThrowType::CHEST_PASS;
        p.name = "chest_pass";
        p.max_power = 12.0f;
        p.base_accuracy = 0.92f;
        p.power_accuracy_penalty = 0.1f;
        p.default_launch_angle = -0.05f; // Slightly downward
        p.spin_rate_factor = 0.5f;
        p.default_spin = SpinAxis::BACKSPIN;
        p.release_speed_factor = 1.2f;
        return p;
    }

    static ThrowProfile make_bounce_pass() {
        ThrowProfile p;
        p.type = ThrowType::BOUNCE_PASS;
        p.name = "bounce_pass";
        p.max_power = 10.0f;
        p.base_accuracy = 0.88f;
        p.power_accuracy_penalty = 0.1f;
        p.default_launch_angle = -0.5f;  // Downward at ~28 degrees
        p.min_launch_angle = -0.7f;
        p.spin_rate_factor = 0.8f;
        p.default_spin = SpinAxis::TOPSPIN;
        p.release_speed_factor = 1.1f;
        return p;
    }

    static ThrowProfile make_spiral() {
        ThrowProfile p;
        p.type = ThrowType::SPIRAL;
        p.name = "spiral";
        p.max_power = 30.0f;        // NFL quarterback: ~30 m/s
        p.base_accuracy = 0.8f;
        p.power_accuracy_penalty = 0.2f;
        p.default_launch_angle = 0.15f; // Slightly upward
        p.spin_rate_factor = 3.5f;  // Tight spiral
        p.default_spin = SpinAxis::HELICAL;
        p.release_speed_factor = 1.4f;
        return p;
    }

    static ThrowProfile make_spin_pass() {
        ThrowProfile p;
        p.type = ThrowType::SHOT_PUT;
        p.name = "spin_pass";
        p.max_power = 25.0f;        // Rugby spin pass
        p.base_accuracy = 0.85f;
        p.power_accuracy_penalty = 0.15f;
        p.default_launch_angle = 0.0f; // Flat
        p.spin_rate_factor = 2.5f;
        p.default_spin = SpinAxis::HELICAL;
        p.release_speed_factor = 1.3f;
        return p;
    }

    static ThrowProfile make_pop_pass() {
        ThrowProfile p;
        p.type = ThrowType::POP_PASS;
        p.name = "pop_pass";
        p.max_power = 12.0f;
        p.base_accuracy = 0.9f;
        p.power_accuracy_penalty = 0.05f;
        p.default_launch_angle = 0.6f; // Lofted
        p.spin_rate_factor = 0.3f;
        p.default_spin = SpinAxis::BACKSPIN;
        p.release_speed_factor = 1.0f;
        return p;
    }

    static ThrowProfile make_overhead_throw() {
        ThrowProfile p;
        p.type = ThrowType::HAND_THROW;
        p.name = "overhead_throw";
        p.max_power = 25.0f;
        p.base_accuracy = 0.8f;
        p.power_accuracy_penalty = 0.2f;
        p.default_launch_angle = 0.3f;
        p.spin_rate_factor = 1.5f;
        p.default_spin = SpinAxis::BACKSPIN;
        p.release_speed_factor = 1.3f;
        return p;
    }
};

// =============================================================================
// KickExecutor — Executes foot strikes on a ball
// =============================================================================
struct KickExecutor {
    // --- Execute a kick ---
    PassResult execute(BallState& ball, const KickProfile& profile,
                        const Vec3& athlete_pos, const Vec3& athlete_forward,
                        const Vec3& kick_direction, float power,
                        float skill_rating = 0.8f,
                        float charge_time = 0.0f)
    {
        PassResult result;
        result.executed = false;

        // Check if ball is within kick range
        Vec3 to_ball = Vec3::sub(ball.body.position, athlete_pos);
        float distance = Vec3::length(to_ball);
        float kick_reach = 1.2f; // Max kick reach (meters)
        if (distance > kick_reach) return result;

        // Power from charge time or direct input
        float effective_power = power;
        if (charge_time > 0.0f) {
            effective_power = std::min(1.0f, charge_time / profile.charge_time_to_max);
        }
        effective_power = std::max(0.0f, std::min(1.0f, effective_power));

        // Calculate ball speed
        float ball_speed = profile.min_power + effective_power *
            (profile.max_power - profile.min_power);

        // Apply skill to power
        ball_speed *= (0.8f + 0.2f * skill_rating);

        // Direction with accuracy scatter
        float accuracy = profile.base_accuracy -
            effective_power * profile.power_accuracy_penalty;
        accuracy *= skill_rating;
        accuracy = std::max(0.3f, std::min(1.0f, accuracy));

        float error_angle = (1.0f - accuracy) * 0.5f; // Max ±0.25 rad scatter

        // Compute scatter direction
        // Use deterministic pseudo-random based on power (no actual random)
        float scatter_x = std::sin(power * 123.456f + skill_rating * 789.012f) * error_angle;
        float scatter_z = std::cos(power * 345.678f + skill_rating * 234.567f) * error_angle;

        Vec3 dir = kick_direction;
        float dir_len = Vec3::length(dir);
        if (dir_len < APC_EPSILON) {
            dir = athlete_forward;
            dir_len = Vec3::length(dir);
        }
        if (dir_len > APC_EPSILON) {
            dir = Vec3::scale(dir, 1.0f / dir_len);
        }

        // Apply scatter via small rotation
        float cos_s = std::cos(scatter_x);
        float sin_s = std::sin(scatter_x);
        Vec3 scattered(
            dir.x * cos_s - dir.z * sin_s,
            dir.y + scatter_z,
            dir.x * sin_s + dir.z * cos_s
        );

        // Set vertical component (launch angle)
        float launch_angle = profile.default_launch_angle;
        scattered.y = std::sin(launch_angle);
        float horiz = std::cos(launch_angle);
        scattered.x *= horiz;
        scattered.z *= horiz;

        // Normalize and set speed
        float s_len = Vec3::length(scattered);
        if (s_len > APC_EPSILON) {
            scattered = Vec3::scale(scattered, 1.0f / s_len);
        }

        // Apply velocity
        result.launch_velocity = Vec3::scale(scattered, ball_speed);
        result.launch_position = ball.body.position;
        result.target_direction = dir;
        result.power = effective_power;
        result.speed = ball_speed;
        result.accuracy_error = error_angle;
        result.launch_angle = launch_angle;

        // Apply spin
        float spin_amount = ball_speed * profile.spin_rate_factor * 0.5f;
        result.spin_rate = spin_amount;
        result.spin_type = profile.default_spin;

        // Set angular velocity based on spin type
        switch (profile.default_spin) {
        case SpinAxis::TOPSPIN:
            result.spin_axis = Vec3::cross(scattered, Vec3(0.0f, 1.0f, 0.0f));
            break;
        case SpinAxis::BACKSPIN:
            result.spin_axis = Vec3::cross(Vec3(0.0f, 1.0f, 0.0f), scattered);
            break;
        case SpinAxis::SIDESPIN: {
            Vec3 up(0.0f, 1.0f, 0.0f);
            result.spin_axis = Vec3::cross(up, scattered);
            break;
        }
        case SpinAxis::HELICAL:
            result.spin_axis = scattered; // Spin around flight axis
            break;
        }

        float spin_len = Vec3::length(result.spin_axis);
        if (spin_len > APC_EPSILON) {
            result.spin_axis = Vec3::scale(result.spin_axis, 1.0f / spin_len);
        }

        // Apply to ball
        ball.body.linear_velocity = result.launch_velocity;
        ball.body.angular_velocity = Vec3::scale(result.spin_axis, spin_amount);
        ball.on_ground = false;
        ball.in_air = true;
        ball.deformation = effective_power * 0.3f; // Visual squash
        ball.update_spin_info();

        result.executed = true;
        return result;
    }
};

// =============================================================================
// ThrowExecutor — Executes hand throws
// =============================================================================
struct ThrowExecutor {
    PassResult execute(BallState& ball, const ThrowProfile& profile,
                        const Vec3& hand_pos, const Vec3& hand_vel,
                        const Vec3& throw_direction, float power,
                        float skill_rating = 0.8f)
    {
        PassResult result;
        result.executed = false;

        // Power clamping
        float effective_power = std::max(0.0f, std::min(1.0f, power));

        // Calculate release speed
        float release_speed = profile.min_power + effective_power *
            (profile.max_power - profile.min_power);
        release_speed *= (0.8f + 0.2f * skill_rating);

        // Add hand velocity contribution
        float hand_speed = Vec3::length(hand_vel);
        release_speed += hand_speed * profile.release_speed_factor * 0.3f;

        // Direction with accuracy scatter
        float accuracy = profile.base_accuracy -
            effective_power * profile.power_accuracy_penalty;
        accuracy *= skill_rating;
        accuracy = std::max(0.3f, std::min(1.0f, accuracy));

        float error_angle = (1.0f - accuracy) * 0.4f;

        // Deterministic scatter
        float scatter_x = std::sin(power * 456.789f + skill_rating * 123.456f) * error_angle;
        float scatter_z = std::cos(power * 789.012f + skill_rating * 567.890f) * error_angle;

        Vec3 dir = throw_direction;
        float dir_len = Vec3::length(dir);
        if (dir_len < APC_EPSILON) dir = Vec3(0.0f, 0.0f, 1.0f);
        if (dir_len > APC_EPSILON) dir = Vec3::scale(dir, 1.0f / dir_len);

        // Apply scatter
        float cos_s = std::cos(scatter_x);
        float sin_s = std::sin(scatter_x);
        Vec3 scattered(
            dir.x * cos_s - dir.z * sin_s,
            dir.y + scatter_z,
            dir.x * sin_s + dir.z * cos_s
        );

        // Set launch angle
        float launch_angle = profile.default_launch_angle;
        launch_angle = std::max(profile.min_launch_angle,
            std::min(profile.max_launch_angle, launch_angle));
        scattered.y = std::sin(launch_angle);
        float horiz = std::cos(launch_angle);
        scattered.x *= horiz;
        scattered.z *= horiz;

        float s_len = Vec3::length(scattered);
        if (s_len > APC_EPSILON) {
            scattered = Vec3::scale(scattered, 1.0f / s_len);
        }

        result.launch_velocity = Vec3::scale(scattered, release_speed);
        result.launch_position = hand_pos;
        result.target_direction = dir;
        result.power = effective_power;
        result.speed = release_speed;
        result.accuracy_error = error_angle;
        result.launch_angle = launch_angle;

        // Spin
        float spin_amount = release_speed * profile.spin_rate_factor * 0.4f;
        result.spin_rate = spin_amount;
        result.spin_type = profile.default_spin;

        switch (profile.default_spin) {
        case SpinAxis::BACKSPIN:
            result.spin_axis = Vec3::cross(Vec3(0.0f, 1.0f, 0.0f), scattered);
            break;
        case SpinAxis::TOPSPIN:
            result.spin_axis = Vec3::cross(scattered, Vec3(0.0f, 1.0f, 0.0f));
            break;
        case SpinAxis::HELICAL:
            result.spin_axis = scattered;
            break;
        case SpinAxis::SIDESPIN:
            result.spin_axis = Vec3::cross(Vec3(0.0f, 1.0f, 0.0f), scattered);
            break;
        }

        float spin_len = Vec3::length(result.spin_axis);
        if (spin_len > APC_EPSILON) {
            result.spin_axis = Vec3::scale(result.spin_axis, 1.0f / spin_len);
        }

        // Apply to ball
        ball.body.position = hand_pos;
        ball.body.linear_velocity = result.launch_velocity;
        ball.body.angular_velocity = Vec3::scale(result.spin_axis, spin_amount);
        ball.on_ground = false;
        ball.in_air = true;
        ball.update_spin_info();

        result.executed = true;
        return result;
    }
};

// =============================================================================
// TrajectoryPredictor — Predict future ball position (for AI and gameplay)
// =============================================================================
struct TrajectoryPrediction {
    static constexpr uint32_t MAX_POINTS = 64u;

    Vec3 points[MAX_POINTS];
    float times[MAX_POINTS];      // Time at each point (seconds from now)
    uint32_t point_count = 0;

    // --- Predict trajectory without air resistance (parabolic) ---
    uint32_t predict_parabolic(const Vec3& pos, const Vec3& vel,
                                float gravity, float max_time,
                                float time_step = 0.05f)
    {
        point_count = 0;
        float t = 0.0f;

        while (t <= max_time && point_count < MAX_POINTS) {
            points[point_count] = Vec3(
                pos.x + vel.x * t,
                pos.y + vel.y * t + 0.5f * gravity * t * t,
                pos.z + vel.z * t
            );
            times[point_count] = t;

            // Stop if ball hits ground
            if (points[point_count].y < 0.0f) {
                // Interpolate exact ground hit
                float t_hit = (-vel.y - std::sqrt(
                    vel.y * vel.y - 2.0f * gravity * pos.y)) / gravity;
                if (t_hit > 0.0f && t_hit < t) {
                    points[point_count] = Vec3(
                        pos.x + vel.x * t_hit,
                        0.0f,
                        pos.z + vel.z * t_hit
                    );
                    times[point_count] = t_hit;
                    ++point_count;
                }
                break;
            }

            ++point_count;
            t += time_step;
        }

        return point_count;
    }

    // --- Get predicted position at a specific time ---
    Vec3 position_at(float target_time) const {
        // Find bracketing times
        if (point_count == 0) return Vec3(0.0f, 0.0f, 0.0f);
        if (target_time <= times[0]) return points[0];
        if (target_time >= times[point_count - 1]) return points[point_count - 1];

        for (uint32_t i = 0; i < point_count - 1; ++i) {
            if (times[i] <= target_time && times[i + 1] >= target_time) {
                float t_range = times[i + 1] - times[i];
                if (t_range < APC_EPSILON) return points[i];
                float t_frac = (target_time - times[i]) / t_range;
                return Vec3::add(
                    Vec3::scale(points[i], 1.0f - t_frac),
                    Vec3::scale(points[i + 1], t_frac)
                );
            }
        }
        return points[point_count - 1];
    }

    // --- Get time to reach a Y height ---
    float time_to_height(float target_y) const {
        // Check if last point exactly matches (e.g. ground hit at y=0)
        if (point_count > 0u && std::abs(points[point_count - 1].y - target_y) < APC_EPSILON) {
            return times[point_count - 1];
        }
        for (uint32_t i = 0; i < point_count - 1; ++i) {
            if ((points[i].y >= target_y && points[i + 1].y < target_y) ||
                (points[i].y <= target_y && points[i + 1].y > target_y))
            {
                float y_range = points[i + 1].y - points[i].y;
                if (std::abs(y_range) < APC_EPSILON) return times[i];
                float frac = (target_y - points[i].y) / y_range;
                return times[i] + frac * (times[i + 1] - times[i]);
            }
        }
        return -1.0f; // Not reached
    }

    // --- Get landing position ---
    Vec3 landing_position() const {
        if (point_count == 0) return Vec3(0.0f, 0.0f, 0.0f);
        // Landing is the last point (or first point at y=0)
        for (uint32_t i = point_count - 1; i > 0; --i) {
            if (points[i].y <= 0.01f) return points[i];
        }
        return points[point_count - 1];
    }
};

} // namespace apc
