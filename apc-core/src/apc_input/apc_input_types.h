#pragma once
// =============================================================================
// apc_input_types.h — Input enumerations and MotorIntent bridge structure
// =============================================================================
//
// Defines the fundamental types shared between human input and AI systems:
//
//   - InputButton: uint32_t bitmask for game buttons (bits 0-20)
//   - InputAxis: normalized analog stick / three-axis struct
//   - MotorIntent: THE critical bridge between input/AI and physics
//   - MotorIntentFlags: uint16_t bitmask for entity state flags
//   - ActionType: uint8_t enum for motor action classification
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation
//   - Deterministic: same inputs -> same outputs
//   - C++17
//
// =============================================================================

#include "apc_math/apc_vec3.h"
#include "apc_math/apc_math_common.h"
#include <cstdint>

namespace apc {

// =============================================================================
// ActionType — Enum for motor action classification
// =============================================================================
enum ActionType : uint8_t {
    ACTION_IDLE    = 0,
    ACTION_MOVE    = 1,
    ACTION_SPRINT  = 2,
    ACTION_JUMP    = 3,
    ACTION_CROUCH  = 4,
    ACTION_SLIDE   = 5,
    ACTION_TACKLE  = 6,
    ACTION_THROW   = 7,
    ACTION_CATCH   = 8,
    ACTION_SHOOT   = 9,
    ACTION_PASS    = 10,
    ACTION_COUNT   = 11
};

// =============================================================================
// InputButton — uint32_t bitmask for game buttons
// =============================================================================
// Bit positions for each button:
//   0-3:   Movement (forward/back/left/right)
//   4-7:   Actions (sprint/jump/crouch/slide)
//   8-11:  Face buttons (A/B/X/Y)
//   12-15: Shoulder/trigger
//   16-18: Menu buttons
//   19-20: Modifiers
static constexpr uint32_t INPUT_MOVE_FORWARD   = 1u << 0u;
static constexpr uint32_t INPUT_MOVE_BACK      = 1u << 1u;
static constexpr uint32_t INPUT_MOVE_LEFT      = 1u << 2u;
static constexpr uint32_t INPUT_MOVE_RIGHT     = 1u << 3u;

static constexpr uint32_t INPUT_SPRINT         = 1u << 4u;
static constexpr uint32_t INPUT_JUMP           = 1u << 5u;
static constexpr uint32_t INPUT_CROUCH         = 1u << 6u;
static constexpr uint32_t INPUT_SLIDE          = 1u << 7u;

static constexpr uint32_t INPUT_ACTION_A       = 1u << 8u;
static constexpr uint32_t INPUT_ACTION_B       = 1u << 9u;
static constexpr uint32_t INPUT_ACTION_X       = 1u << 10u;
static constexpr uint32_t INPUT_ACTION_Y       = 1u << 11u;

static constexpr uint32_t INPUT_SHOULDER_L     = 1u << 12u;
static constexpr uint32_t INPUT_SHOULDER_R     = 1u << 13u;
static constexpr uint32_t INPUT_TRIGGER_L      = 1u << 14u;
static constexpr uint32_t INPUT_TRIGGER_R      = 1u << 15u;

static constexpr uint32_t INPUT_MENU_START     = 1u << 16u;
static constexpr uint32_t INPUT_MENU_BACK      = 1u << 17u;
static constexpr uint32_t INPUT_MENU_SELECT    = 1u << 18u;

static constexpr uint32_t INPUT_MODIFIER_SHIFT = 1u << 19u;
static constexpr uint32_t INPUT_MODIFIER_ALT   = 1u << 20u;

// =============================================================================
// InputAxis — Normalized analog stick / three-axis struct
// =============================================================================
struct InputAxis {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    InputAxis() = default;

    APC_FORCEINLINE float magnitude() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    APC_FORCEINLINE float magnitude_xz() const {
        return std::sqrt(x * x + z * z);
    }

    APC_FORCEINLINE void normalize_xz() {
        float mag = magnitude_xz();
        if (mag > APC_EPSILON) {
            float inv = 1.0f / mag;
            x *= inv;
            z *= inv;
        } else {
            x = 0.0f;
            z = 0.0f;
        }
    }
};

// =============================================================================
// MotorIntent — THE critical bridge: input/AI -> physics
// =============================================================================
// Both the human input mapping layer and the AI decision system produce
// this struct. The physics system consumes it to drive entity locomotion,
// sport actions, and contact.
//
// All fields have deterministic defaults (zero/false) so that an
// unmodified intent produces no action.
// =============================================================================
struct MotorIntent {
    // --- Locomotion ---
    Vec3  move_direction   = { 0.0f, 0.0f, 0.0f }; // World-space desired direction
    float move_speed       = 0.0f;   // 0.0-1.0 normalized
    float sprint_intensity = 0.0f;   // 0.0-1.0

    // --- Look ---
    Vec3  look_direction   = { 0.0f, 0.0f, -1.0f }; // Where entity wants to face

    // --- Jump ---
    Vec3  jump_impulse     = { 0.0f, 0.0f, 0.0f }; // Computed from input

    // --- Crouch / Slide ---
    float crouch_amount    = 0.0f;   // 0.0 = standing, 1.0 = full crouch
    float slide_intensity  = 0.0f;   // 0.0-1.0

    // --- Raw buttons ---
    uint32_t buttons       = 0u;     // Raw button state for game logic

    // --- Action type ---
    uint8_t action_type    = ACTION_IDLE;

    // --- Reset all fields to default (no action) ---
    APC_FORCEINLINE void reset() {
        move_direction   = { 0.0f, 0.0f, 0.0f };
        move_speed       = 0.0f;
        sprint_intensity = 0.0f;
        look_direction   = { 0.0f, 0.0f, -1.0f };
        jump_impulse     = { 0.0f, 0.0f, 0.0f };
        crouch_amount    = 0.0f;
        slide_intensity  = 0.0f;
        buttons          = 0u;
        action_type      = ACTION_IDLE;
    }

    // --- Query: is any form of locomotion active? ---
    APC_FORCEINLINE uint8_t has_locomotion() const {
        if (move_direction.x != 0.0f ||
            move_direction.y != 0.0f ||
            move_direction.z != 0.0f) {
            return 1;
        }
        if (sprint_intensity > APC_EPSILON ||
            crouch_amount > APC_EPSILON ||
            slide_intensity > APC_EPSILON) {
            return 1;
        }
        return 0;
    }
};

// =============================================================================
// MotorIntentFlags — uint16_t bitmask for entity state flags
// =============================================================================
static constexpr uint16_t FLAG_AIMING        = 1u << 0u;
static constexpr uint16_t FLAG_CHARGING      = 1u << 1u;
static constexpr uint16_t FLAG_KNOCKBACK     = 1u << 2u;
static constexpr uint16_t FLAG_STUNNED       = 1u << 3u;
static constexpr uint16_t FLAG_GROUNDED      = 1u << 4u;
static constexpr uint16_t FLAG_IN_AIR        = 1u << 5u;
static constexpr uint16_t FLAG_BALL_CARRIER  = 1u << 6u;
static constexpr uint16_t FLAG_DEFENDING     = 1u << 7u;
static constexpr uint16_t FLAG_OFFSIDING     = 1u << 8u;
static constexpr uint16_t FLAG_INJURED       = 1u << 9u;

} // namespace apc
