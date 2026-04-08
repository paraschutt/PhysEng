#pragma once
// =============================================================================
// apc_input_state.h — Raw input capture per frame
// =============================================================================
//
// Captures the per-frame hardware input state before any mapping or
// intent conversion. This is the boundary between platform-specific input
// collection and the engine's input system.
//
//   - InputState: raw digital buttons + analog sticks + triggers
//   - apply_dead_zone(), frame_delta() lifecycle methods
//   - Button/axis query helpers
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation (fixed-size fields)
//   - Deterministic: same inputs -> same outputs
//   - C++17
//
// =============================================================================

#include "apc_input/apc_input_types.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_math_common.h"
#include <cstdint>
#include <cmath>

namespace apc {

// =============================================================================
// MAX_SIMULTANEOUS_INPUTS — Local multiplayer support (4 players)
// =============================================================================
static constexpr uint32_t MAX_SIMULTANEOUS_INPUTS = 4u;

// =============================================================================
// InputState — Per-frame raw input snapshot
// =============================================================================
struct InputState {
    // --- Digital buttons (bitmask) ---
    uint32_t buttons_down    = 0u; // Currently held / pressed
    uint32_t buttons_pressed = 0u; // Just pressed this frame (one-shot)
    uint32_t buttons_released = 0u; // Just released this frame (one-shot)

    // --- Analog sticks ---
    InputAxis left_stick;
    InputAxis right_stick;

    // --- Analog triggers (0.0-1.0) ---
    float trigger_left  = 0.0f;
    float trigger_right = 0.0f;

    // --- Configurable dead zone ---
    float dead_zone = 0.15f;

    // --- Timing ---
    uint32_t frame_number = 0u;

    // --- Previous frame buttons (for delta computation) ---
    uint32_t prev_buttons_down = 0u;

    // =========================================================================
    // Lifecycle
    // =========================================================================

    // --- begin_frame: call at start of each frame ---
    // Stores previous button state for delta computation, resets one-shot bits.
    APC_FORCEINLINE void begin_frame() {
        prev_buttons_down = buttons_down;
        buttons_pressed  = 0u;
        buttons_released = 0u;
    }

    // --- frame_delta: compute pressed/released from current vs previous ---
    // Call after all input events for this frame have been processed.
    // pressed = bits set this frame but not last frame
    // released = bits set last frame but not this frame
    APC_FORCEINLINE void frame_delta() {
        buttons_pressed  = buttons_down & ~prev_buttons_down;
        buttons_released = prev_buttons_down & ~buttons_down;
    }

    // =========================================================================
    // Button helpers
    // =========================================================================

    // --- Query: is button currently held? (persistent) ---
    APC_FORCEINLINE uint8_t is_button_held(uint32_t button_mask) const {
        return (buttons_down & button_mask) != 0u ? 1u : 0u;
    }

    // --- Query: was button just pressed this frame? (one-shot) ---
    APC_FORCEINLINE uint8_t is_button_pressed(uint32_t button_mask) const {
        return (buttons_pressed & button_mask) != 0u ? 1u : 0u;
    }

    // --- Query: was button just released this frame? (one-shot) ---
    APC_FORCEINLINE uint8_t is_button_released(uint32_t button_mask) const {
        return (buttons_released & button_mask) != 0u ? 1u : 0u;
    }

    // --- Set button pressed: sets bit in held bitmask ---
    APC_FORCEINLINE void press_button(uint32_t button_mask) {
        buttons_down |= button_mask;
    }

    // --- Set button released: clears bit in held bitmask ---
    APC_FORCEINLINE void release_button(uint32_t button_mask) {
        buttons_down &= ~button_mask;
    }

    // =========================================================================
    // Dead zone
    // =========================================================================

    // --- apply_dead_zone: normalize sticks below dead_zone to zero ---
    // Applies circular dead zone to left_stick and right_stick XZ components.
    APC_FORCEINLINE void apply_dead_zone() {
        apply_stick_dead_zone(left_stick);
        apply_stick_dead_zone(right_stick);
    }

    // --- get_move_vector: combines left stick + D-pad into Vec3 ---
    // Returns world-space movement vector (Y=0 for ground plane).
    APC_FORCEINLINE Vec3 get_move_vector() const {
        Vec3 result(0.0f, 0.0f, 0.0f);
        // Analog stick contribution
        result.x += left_stick.x;
        result.z += left_stick.y; // Stick Y maps to world Z
        // D-pad contribution
        if (buttons_down & INPUT_MOVE_FORWARD)  result.z -= 1.0f;
        if (buttons_down & INPUT_MOVE_BACK)     result.z += 1.0f;
        if (buttons_down & INPUT_MOVE_LEFT)     result.x -= 1.0f;
        if (buttons_down & INPUT_MOVE_RIGHT)    result.x += 1.0f;
        // Clamp to [-1, 1]
        if (result.x < -1.0f) result.x = -1.0f;
        if (result.x >  1.0f) result.x =  1.0f;
        if (result.z < -1.0f) result.z = -1.0f;
        if (result.z >  1.0f) result.z =  1.0f;
        return result;
    }

    // --- get_look_vector: returns right stick as Vec3 ---
    // Returns look direction (Y=0 for ground plane, stick Y maps to world Z).
    APC_FORCEINLINE Vec3 get_look_vector() const {
        Vec3 result(0.0f, 0.0f, 0.0f);
        result.x = right_stick.x;
        result.z = right_stick.y; // Stick Y maps to world Z
        return result;
    }

    // --- set_left_stick: set stick values, clamped to [-1, 1] ---
    APC_FORCEINLINE void set_left_stick(float x, float y, float z = 0.0f) {
        left_stick.x = clamp_axis(x);
        left_stick.y = clamp_axis(y);
        left_stick.z = clamp_axis(z);
    }

    // --- set_right_stick: set stick values, clamped to [-1, 1] ---
    APC_FORCEINLINE void set_right_stick(float x, float y, float z = 0.0f) {
        right_stick.x = clamp_axis(x);
        right_stick.y = clamp_axis(y);
        right_stick.z = clamp_axis(z);
    }

    // --- set_trigger: set trigger values, clamped to [0, 1] ---
    APC_FORCEINLINE void set_trigger_left(float value) {
        trigger_left = clamp_trigger(value);
    }

    APC_FORCEINLINE void set_trigger_right(float value) {
        trigger_right = clamp_trigger(value);
    }

private:
    // --- Apply circular dead zone to a stick's XZ components ---
    APC_FORCEINLINE void apply_stick_dead_zone(InputAxis& stick) {
        float mag = std::sqrt(stick.x * stick.x + stick.y * stick.y);
        if (mag < dead_zone) {
            stick.x = 0.0f;
            stick.y = 0.0f;
        } else if (mag > APC_EPSILON) {
            // Rescale so edge of dead zone maps to 0, max maps to 1
            float scale = (mag - dead_zone) / (1.0f - dead_zone);
            float inv_mag = 1.0f / mag;
            stick.x = stick.x * inv_mag * scale;
            stick.y = stick.y * inv_mag * scale;
        }
        // Clamp to [-1, 1]
        if (stick.x < -1.0f) stick.x = -1.0f;
        if (stick.x >  1.0f) stick.x =  1.0f;
        if (stick.y < -1.0f) stick.y = -1.0f;
        if (stick.y >  1.0f) stick.y =  1.0f;
    }

    // --- Clamp axis to [-1, 1] ---
    APC_FORCEINLINE static float clamp_axis(float value) {
        if (value < -1.0f) return -1.0f;
        if (value >  1.0f) return  1.0f;
        return value;
    }

    // --- Clamp trigger to [0, 1] ---
    APC_FORCEINLINE static float clamp_trigger(float value) {
        if (value < 0.0f) return 0.0f;
        if (value > 1.0f) return 1.0f;
        return value;
    }
};

} // namespace apc
