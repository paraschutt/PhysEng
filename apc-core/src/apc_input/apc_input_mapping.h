#pragma once
// =============================================================================
// apc_input_mapping.h — IntentConverter: InputState -> MotorIntent
// =============================================================================
//
// Transforms raw hardware input (InputState) into a MotorIntent that the
// physics system can consume. This is the bridge between human input and
// the engine's action system. The same MotorIntent output is also produced
// by the AI decision layer, making IntentConverter the human side of the
// input/AI -> physics pipeline.
//
//   - IntentConverter: sport-specific input mapping with tunable parameters
//   - Preset configurations: soccer, basketball, football, rugby
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation
//   - Deterministic: same inputs -> same outputs (no FMA)
//   - C++17
//
// =============================================================================

#include "apc_input/apc_input_types.h"
#include "apc_input/apc_input_state.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_math_common.h"
#include <cstdint>
#include <cmath>

namespace apc {

// =============================================================================
// IntentConverter — Transforms InputState + camera into MotorIntent
// =============================================================================
struct IntentConverter {
    // --- Sport-specific tuning parameters ---
    float walk_speed         = 4.0f;    // Walking speed (m/s)
    float run_speed          = 7.0f;    // Running speed (m/s)
    float sprint_multiplier  = 1.5f;    // Sprint speed multiplier
    float jump_force         = 8.0f;    // Jump impulse force (m/s)
    float crouch_transition_speed = 5.0f; // Crouch animation speed
    float slide_threshold    = 0.7f;    // Stick deflection to trigger slide
    float look_sensitivity   = 1.0f;    // Look stick multiplier
    float dead_zone          = 0.15f;   // Analog stick dead zone

    // --- World up vector ---
    Vec3 world_up = { 0.0f, 1.0f, 0.0f };

    // =========================================================================
    // convert — Primary method: raw input -> motor intent
    // =========================================================================
    // Takes the current InputState, the camera's forward vector (for
    // camera-relative movement), and the entity's world position.
    //
    // The camera_forward is projected onto the XZ plane so that movement
    // is relative to the camera's horizontal facing direction only.
    // =========================================================================
    MotorIntent convert(const InputState& input,
                        const Vec3& camera_forward,
                        const Vec3& entity_position) const
    {
        (void)entity_position;
        MotorIntent intent;
        intent.reset();

        // --- 1. Locomotion from left stick + D-pad ---
        Vec3 move = input.get_move_vector();
        float move_mag = std::sqrt(move.x * move.x + move.z * move.z);

        if (move_mag > APC_EPSILON) {
            // Transform stick input relative to camera forward (XZ plane)
            intent.move_direction = camera_relative_move(move.x, move.z, camera_forward);

            // Compute move speed from analog magnitude
            move_mag = std::min(move_mag, 1.0f);
            intent.move_speed = move_mag;

            // Apply sprint multiplier
            if (input.is_button_held(INPUT_SPRINT)) {
                intent.sprint_intensity = 1.0f;
                intent.move_speed *= sprint_multiplier;
            }
        }

        // --- 2. Crouch ---
        if (input.is_button_held(INPUT_CROUCH)) {
            intent.crouch_amount = 1.0f;
            intent.move_speed *= 0.4f; // Crouch speed reduction
            intent.action_type = ACTION_CROUCH;
        }

        // --- 3. Slide ---
        if (input.is_button_pressed(INPUT_SLIDE) && move_mag >= slide_threshold) {
            intent.slide_intensity = move_mag;
            intent.action_type = ACTION_SLIDE;
        }

        // --- 4. Jump ---
        if (input.is_button_pressed(INPUT_JUMP)) {
            // Jump impulse = world_up * jump_force
            intent.jump_impulse = Vec3::scale(world_up, jump_force);
            intent.action_type = ACTION_JUMP;
        }

        // --- 5. Look direction from right stick ---
        Vec3 look = input.get_look_vector();
        float look_mag = std::sqrt(look.x * look.x + look.z * look.z);
        if (look_mag > dead_zone) {
            Vec3 look_dir = camera_relative_move(
                look.x * look_sensitivity,
                look.z * look_sensitivity,
                camera_forward);

            float look_len = Vec3::length(look_dir);
            if (look_len > APC_EPSILON) {
                look_dir = Vec3::scale(look_dir, 1.0f / look_len);
            }
            intent.look_direction = look_dir;
        }

        // --- 6. Raw buttons for game logic ---
        intent.buttons = input.buttons_down;

        // --- 7. Determine action type from button + movement state ---
        if (input.is_button_pressed(INPUT_ACTION_A)) {
            if (input.is_button_held(INPUT_SHOULDER_R)) {
                intent.action_type = ACTION_SHOOT;
            } else {
                intent.action_type = ACTION_PASS;
            }
        } else if (input.is_button_pressed(INPUT_ACTION_B)) {
            intent.action_type = ACTION_TACKLE;
        } else if (input.is_button_pressed(INPUT_ACTION_X)) {
            intent.action_type = ACTION_CATCH;
        } else if (input.is_button_pressed(INPUT_ACTION_Y)) {
            intent.action_type = ACTION_THROW;
        } else if (move_mag > APC_EPSILON) {
            if (intent.sprint_intensity > APC_EPSILON) {
                intent.action_type = ACTION_SPRINT;
            } else {
                intent.action_type = ACTION_MOVE;
            }
        } else {
            intent.action_type = ACTION_IDLE;
        }

        // --- 8. Store buttons ---
        intent.buttons = input.buttons_down;

        return intent;
    }

    // =========================================================================
    // get_action_type — Determine action from input state and flags
    // =========================================================================
    uint8_t get_action_type(const InputState& input,
                             uint16_t flags) const
    {
        (void)flags; // Reserved for flag-based action override

        if (input.is_button_pressed(INPUT_ACTION_A)) {
            if (input.is_button_held(INPUT_SHOULDER_R)) {
                return ACTION_SHOOT;
            }
            return ACTION_PASS;
        }
        if (input.is_button_pressed(INPUT_ACTION_B)) {
            return ACTION_TACKLE;
        }
        if (input.is_button_pressed(INPUT_ACTION_X)) {
            return ACTION_CATCH;
        }
        if (input.is_button_pressed(INPUT_ACTION_Y)) {
            return ACTION_THROW;
        }

        Vec3 move = input.get_move_vector();
        float move_mag = std::sqrt(move.x * move.x + move.z * move.z);
        if (move_mag > APC_EPSILON) {
            if (input.is_button_held(INPUT_SPRINT)) {
                return ACTION_SPRINT;
            }
            return ACTION_MOVE;
        }
        if (input.is_button_pressed(INPUT_JUMP)) {
            return ACTION_JUMP;
        }
        if (input.is_button_held(INPUT_CROUCH)) {
            return ACTION_CROUCH;
        }
        if (input.is_button_pressed(INPUT_SLIDE)) {
            return ACTION_SLIDE;
        }

        return ACTION_IDLE;
    }

    // =========================================================================
    // Sport-specific preset factory methods
    // =========================================================================

    // --- Soccer defaults: foot-dominant, kick-focused ---
    static IntentConverter soccer_defaults() {
        IntentConverter cv;
        cv.walk_speed         = 3.5f;
        cv.run_speed          = 7.0f;
        cv.sprint_multiplier  = 1.5f;
        cv.jump_force         = 5.0f;   // Low jump (soccer)
        cv.crouch_transition_speed = 4.0f;
        cv.slide_threshold    = 0.7f;
        cv.look_sensitivity   = 1.0f;
        cv.dead_zone          = 0.15f;
        return cv;
    }

    // --- Basketball defaults: hand-dominant, throw/pass-focused ---
    static IntentConverter basketball_defaults() {
        IntentConverter cv;
        cv.walk_speed         = 3.0f;
        cv.run_speed          = 6.0f;
        cv.sprint_multiplier  = 1.4f;
        cv.jump_force         = 10.0f;  // Medium jump (basketball)
        cv.crouch_transition_speed = 5.0f;
        cv.slide_threshold    = 0.0f;   // No slide in basketball
        cv.look_sensitivity   = 1.2f;
        cv.dead_zone          = 0.12f;
        return cv;
    }

    // --- American Football defaults: tackle/block focused ---
    static IntentConverter football_defaults() {
        IntentConverter cv;
        cv.walk_speed         = 3.5f;
        cv.run_speed          = 8.0f;
        cv.sprint_multiplier  = 1.6f;
        cv.jump_force         = 4.0f;   // Low jump (football)
        cv.crouch_transition_speed = 5.0f;
        cv.slide_threshold    = 0.0f;   // No slide in football
        cv.look_sensitivity   = 0.9f;
        cv.dead_zone          = 0.15f;
        return cv;
    }

    // --- Rugby defaults: grapple/tackle focused ---
    static IntentConverter rugby_defaults() {
        IntentConverter cv;
        cv.walk_speed         = 3.5f;
        cv.run_speed          = 7.5f;
        cv.sprint_multiplier  = 1.5f;
        cv.jump_force         = 5.0f;   // Low jump (rugby)
        cv.crouch_transition_speed = 4.5f;
        cv.slide_threshold    = 0.0f;   // No slide in rugby
        cv.look_sensitivity   = 1.0f;
        cv.dead_zone          = 0.15f;
        return cv;
    }

private:
    // =========================================================================
    // camera_relative_move — Transform 2D stick input to world-space 3D vector
    // =========================================================================
    // Projects camera_forward onto the XZ plane, then builds a right vector
    // from the cross product with world-up. The stick X maps to camera-right,
    // stick Y maps to camera-forward.
    // =========================================================================
    APC_FORCEINLINE Vec3 camera_relative_move(float move_x, float move_y,
                                               const Vec3& camera_forward) const
    {
        // Project camera forward onto XZ plane
        Vec3 cam_fwd = camera_forward;
        cam_fwd.y = 0.0f;
        float fwd_len = Vec3::length(cam_fwd);
        if (fwd_len < APC_EPSILON) {
            // Camera is looking straight up/down; fall back to +Z
            cam_fwd = Vec3(0.0f, 0.0f, -1.0f);
        } else {
            cam_fwd = Vec3::scale(cam_fwd, 1.0f / fwd_len);
        }

        // Camera right = cross(forward, up) on XZ plane
        Vec3 cam_right = Vec3::cross(cam_fwd, world_up);
        float right_len = Vec3::length(cam_right);
        if (right_len < APC_EPSILON) {
            cam_right = Vec3(1.0f, 0.0f, 0.0f);
        } else {
            cam_right = Vec3::scale(cam_right, 1.0f / right_len);
        }

        // Combine: world_move = right * move_x + forward * move_y
        return Vec3::add(
            Vec3::scale(cam_right, move_x),
            Vec3::scale(cam_fwd, move_y)
        );
    }
};

} // namespace apc
