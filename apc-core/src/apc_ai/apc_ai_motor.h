#pragma once
// =============================================================================
// apc_ai_motor.h — Converts steering output into MotorIntent
// =============================================================================
//
// Bridges the AI steering layer and the physics input layer:
//
//   AIMotorController: converts SteeringOutput -> MotorIntent
//
// Key design: AI and human players produce IDENTICAL MotorIntent output.
// The physics engine cannot distinguish between them.
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation (fixed-size arrays)
//   - Deterministic: same inputs -> same outputs
//   - C++17
//
// =============================================================================

#include "apc_ai/apc_ai_steering.h"
#include "apc_input/apc_input_types.h"
#include "apc_entity/apc_entity_types.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include "apc_math/apc_math_common.h"
#include <cstdint>
#include <cstring>
#include <cmath>

namespace apc {

// =============================================================================
// Capacity constants
// =============================================================================
static constexpr uint32_t MAX_AI_CONTROLLERS = 44;

// =============================================================================
// AIMotorController — Converts steering output into MotorIntent
// =============================================================================
struct AIMotorController {
    // --- Tuning parameters ---
    float reaction_delay = 0.1f;   // Simulated reaction time (seconds)
    float accuracy       = 0.9f;   // 0.0-1.0 execution accuracy
    float aggression     = 0.5f;   // 0.0-1.0 risk-taking tendency
    float skill_level    = 0.7f;   // 0.0-1.0 overall ability

    // --- Steering configuration ---
    SteeringRequest steering_config;

    // --- Formation flocking weights ---
    // Used to tune separation/cohesion/alignment balance per role.
    // These are applied when blending flocking behaviors in the AI loop.
    float separation_weight = 1.5f; // Repulsion from nearby agents
    float cohesion_weight   = 0.5f; // Pull toward teammate centroid
    float alignment_weight  = 0.8f; // Match teammate velocity direction

    // --- Per-frame state ---
    float       reaction_timer      = 0.0f;  // Counts down before acting
    MotorIntent accumulated_intent;
    Vec3        current_steering_force = { 0.0f, 0.0f, 0.0f };

    // =========================================================================
    // update — Convert steering output to MotorIntent
    // =========================================================================
    // Takes steering output, current position/orientation, and dt.
    // Returns a MotorIntent identical in format to human input.
    // =========================================================================
    MotorIntent update(const SteeringOutput& steering,
                        const Vec3& position,
                        const Quat& orientation,
                        float dt)
    {
        (void)orientation;

        MotorIntent intent;
        intent.reset();

        // --- 1. Reaction delay: count down before processing ---
        if (reaction_timer > 0.0f) {
            reaction_timer -= dt;
            // During reaction delay, keep previous intent (idle)
            return accumulated_intent;
        }

        // --- 2. Store steering force for debugging ---
        current_steering_force = steering.linear;

        // --- 3. Convert linear steering -> move_direction + move_speed ---
        Vec3 move_dir = steering.linear;
        move_dir.y = 0.0f;
        float mag = Vec3::length(move_dir);

        if (mag > APC_EPSILON) {
            // Apply accuracy noise: scale direction by accuracy
            float effective_accuracy = accuracy * (0.8f + 0.2f * skill_level);
            if (effective_accuracy < 0.1f) effective_accuracy = 0.1f;
            if (effective_accuracy > 1.0f) effective_accuracy = 1.0f;

            // Noise is deterministic: based on position + magnitude
            float noise_scale = (1.0f - effective_accuracy) * 0.3f;
            float noise_x = std::sin(position.x * 12.9898f + position.z * 78.233f +
                                     mag * 43.758f) * noise_scale;
            float noise_z = std::cos(position.z * 12.9898f + position.x * 78.233f +
                                     mag * 43.758f) * noise_scale;

            Vec3 noisy_dir = Vec3::add(move_dir, Vec3(noise_x, 0.0f, noise_z));
            float noisy_mag = Vec3::length(noisy_dir);
            if (noisy_mag > APC_EPSILON) {
                intent.move_direction = Vec3::scale(noisy_dir, 1.0f / noisy_mag);
            } else {
                intent.move_direction = Vec3::scale(move_dir, 1.0f / mag);
            }
        } else {
            intent.move_direction = Vec3(0.0f, 0.0f, 0.0f);
        }

        // --- 4. Move speed from steering magnitude ---
        float max_spd = steering_config.max_speed;
        if (max_spd < APC_EPSILON) max_spd = 8.0f;
        intent.move_speed = (mag > max_spd) ? 1.0f : mag / max_spd;
        if (intent.move_speed < 0.0f) intent.move_speed = 0.0f;
        if (intent.move_speed > 1.0f) intent.move_speed = 1.0f;

        // --- 5. Sprint from aggression and speed ---
        if (intent.move_speed > 0.85f && aggression > 0.5f) {
            intent.sprint_intensity = (intent.move_speed - 0.85f) / 0.15f * aggression;
            if (intent.sprint_intensity > 1.0f) intent.sprint_intensity = 1.0f;
            if (intent.sprint_intensity < 0.0f) intent.sprint_intensity = 0.0f;
        }

        // --- 6. Look direction: face behavior -> look_direction ---
        // If steering has angular component, use it for look direction
        float ang_mag = Vec3::length(steering.angular);
        if (ang_mag > APC_EPSILON) {
            // Rotate current move_direction by angular amount
            float angle = steering.angular.y * dt;
            float cos_a = std::cos(angle);
            float sin_a = std::sin(angle);
            intent.look_direction = Vec3(
                intent.move_direction.x * cos_a - intent.move_direction.z * sin_a,
                0.0f,
                intent.move_direction.x * sin_a + intent.move_direction.z * cos_a
            );
        } else if (mag > APC_EPSILON) {
            // Default: look where moving
            intent.look_direction = intent.move_direction;
            intent.look_direction.y = 0.0f;
        }

        // --- 7. Determine action_type from steering context ---
        // Urgency-based action selection
        if (steering.urgency > 0.8f) {
            intent.action_type = ACTION_SPRINT;
        } else if (steering.urgency > 0.5f) {
            intent.action_type = ACTION_MOVE;
        } else if (steering.urgency > 0.1f) {
            intent.action_type = ACTION_MOVE;
        } else {
            intent.action_type = ACTION_IDLE;
        }

        // --- 8. Apply aggression modifier to speed ---
        intent.move_speed *= (0.8f + 0.4f * aggression);
        if (intent.move_speed > 1.0f) intent.move_speed = 1.0f;

        // --- 9. Store accumulated intent ---
        accumulated_intent = intent;

        // --- 10. Set reaction timer for next update ---
        // Higher skill = lower reaction time
        float effective_reaction = reaction_delay * (1.2f - 0.4f * skill_level);
        if (effective_reaction < 0.01f) effective_reaction = 0.01f;
        reaction_timer = effective_reaction;

        return intent;
    }

    // =========================================================================
    // set_preset — Configure controller for a specific role
    // =========================================================================
    // Uses strcmp for exact string matching instead of first-character
    // comparison, preventing collisions between presets that share
    // the same starting letter (e.g. "forward" vs a hypothetical
    // "fullback").
    // =========================================================================
    void set_preset(const char* name)
    {
        if (name == nullptr) return;

        if (std::strcmp(name, "defender") == 0) {
            // Defender: slow reaction, accurate, low aggression, tight block
            reaction_delay = 0.12f;
            accuracy       = 0.85f;
            aggression     = 0.4f;
            skill_level    = 0.75f;
            steering_config.max_speed = 7.0f;
            steering_config.max_force = 15.0f;
            separation_weight = 1.2f;
            cohesion_weight   = 1.0f; // Stay in defensive line
            alignment_weight  = 1.0f;
        }
        else if (std::strcmp(name, "midfielder") == 0) {
            // Midfielder: balanced, good alignment to move as a unit
            reaction_delay = 0.10f;
            accuracy       = 0.80f;
            aggression     = 0.5f;
            skill_level    = 0.70f;
            steering_config.max_speed = 7.5f;
            steering_config.max_force = 18.0f;
            separation_weight = 1.5f;
            cohesion_weight   = 0.8f;
            alignment_weight  = 1.2f;
        }
        else if (std::strcmp(name, "forward") == 0) {
            // Forward: fast reaction, high aggression, spread for attack
            reaction_delay = 0.08f;
            accuracy       = 0.75f;
            aggression     = 0.7f;
            skill_level    = 0.80f;
            steering_config.max_speed = 8.5f;
            steering_config.max_force = 22.0f;
            separation_weight = 1.8f; // More spacing for attacking width
            cohesion_weight   = 0.5f;
            alignment_weight  = 0.8f;
        }
        else if (std::strcmp(name, "goalkeeper") == 0) {
            // Goalkeeper: fast reaction, very accurate, low aggression
            reaction_delay = 0.06f;
            accuracy       = 0.90f;
            aggression     = 0.3f;
            skill_level    = 0.85f;
            steering_config.max_speed = 9.0f;
            steering_config.max_force = 25.0f;
            separation_weight = 1.0f;
            cohesion_weight   = 1.2f; // Stay near goal line
            alignment_weight  = 0.6f;
        }
        else if (std::strcmp(name, "point_guard") == 0) {
            // Point guard: quick, accurate, moderate spacing
            reaction_delay = 0.07f;
            accuracy       = 0.88f;
            aggression     = 0.6f;
            skill_level    = 0.82f;
            steering_config.max_speed = 8.0f;
            steering_config.max_force = 20.0f;
            separation_weight = 2.0f; // Court spacing
            cohesion_weight   = 0.6f;
            alignment_weight  = 0.9f;
        }
        // Unknown preset name: keep current values (no-op)
    }

    // --- Reset controller state ---
    void reset()
    {
        reaction_delay = 0.1f;
        accuracy       = 0.9f;
        aggression     = 0.5f;
        skill_level    = 0.7f;
        reaction_timer = 0.0f;
        accumulated_intent.reset();
        current_steering_force = { 0.0f, 0.0f, 0.0f };
        steering_config = SteeringRequest();
    }
};

} // namespace apc
