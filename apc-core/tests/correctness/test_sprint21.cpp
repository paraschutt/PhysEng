// =============================================================================
// Sprint 21 Tests — Input System: Types, State, Mapping
// =============================================================================
//
// Tests for the Sprint 21 deliverables:
//    1. InputButton bitmask values (MOVE_FORWARD=bit0, MOVE_BACK=bit1, etc.)
//    2. InputAxis defaults (x=0, y=0, z=0)
//    3. InputAxis magnitude calculation
//    4. MotorIntent default values (all zeros/false/IDLE)
//    5. MotorIntentFlags bitmask values (AIMING, CHARGING, KNOCKBACK, etc.)
//    6. ActionType enum values (IDLE=0, MOVE=1, SPRINT=2, etc.)
//    7. InputState default values (all buttons 0, sticks zeroed, triggers 0)
//    8. InputState dead zone application (below threshold -> zero)
//    9. InputState frame_delta (pressed/released detection)
//   10. InputState is_button_held query
//   11. InputState is_button_pressed query
//   12. InputState get_move_vector from sticks
//   13. IntentConverter default values
//   14. IntentConverter.convert() basic movement (forward)
//   15. IntentConverter sport presets (soccer/basketball/football/rugby)
//
// Pattern: int main() + assert(), no test framework.
// =============================================================================

#include "apc_input/apc_input_types.h"
#include "apc_input/apc_input_state.h"
#include "apc_input/apc_input_mapping.h"
#include <cstdio>
#include <cmath>
#include <cassert>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static constexpr float EPS = 1e-4f;

static bool approx_eq(float a, float b, float eps = EPS) {
    return std::abs(a - b) < eps;
}

// =============================================================================
// TEST 1: InputButton bitmask values
// =============================================================================
static int test_input_button_bitmask_values() {
    std::printf("  [Test 1] InputButton bitmask values...\n");

    // Movement buttons: bits 0-3
    assert(apc::INPUT_MOVE_FORWARD == (1u << 0u) && "MOVE_FORWARD = bit0");
    assert(apc::INPUT_MOVE_BACK    == (1u << 1u) && "MOVE_BACK = bit1");
    assert(apc::INPUT_MOVE_LEFT    == (1u << 2u) && "MOVE_LEFT = bit2");
    assert(apc::INPUT_MOVE_RIGHT   == (1u << 3u) && "MOVE_RIGHT = bit3");

    // Action buttons: bits 4-7
    assert(apc::INPUT_SPRINT  == (1u << 4u) && "SPRINT = bit4");
    assert(apc::INPUT_JUMP    == (1u << 5u) && "JUMP = bit5");
    assert(apc::INPUT_CROUCH  == (1u << 6u) && "CROUCH = bit6");
    assert(apc::INPUT_SLIDE   == (1u << 7u) && "SLIDE = bit7");

    // Face buttons: bits 8-11
    assert(apc::INPUT_ACTION_A == (1u << 8u)  && "ACTION_A = bit8");
    assert(apc::INPUT_ACTION_B == (1u << 9u)  && "ACTION_B = bit9");
    assert(apc::INPUT_ACTION_X == (1u << 10u) && "ACTION_X = bit10");
    assert(apc::INPUT_ACTION_Y == (1u << 11u) && "ACTION_Y = bit11");

    // Shoulder/trigger: bits 12-15
    assert(apc::INPUT_SHOULDER_L == (1u << 12u) && "SHOULDER_L = bit12");
    assert(apc::INPUT_SHOULDER_R == (1u << 13u) && "SHOULDER_R = bit13");
    assert(apc::INPUT_TRIGGER_L  == (1u << 14u) && "TRIGGER_L = bit14");
    assert(apc::INPUT_TRIGGER_R  == (1u << 15u) && "TRIGGER_R = bit15");

    // Menu: bits 16-18
    assert(apc::INPUT_MENU_START  == (1u << 16u) && "MENU_START = bit16");
    assert(apc::INPUT_MENU_BACK   == (1u << 17u) && "MENU_BACK = bit17");
    assert(apc::INPUT_MENU_SELECT == (1u << 18u) && "MENU_SELECT = bit18");

    // Modifiers: bits 19-20
    assert(apc::INPUT_MODIFIER_SHIFT == (1u << 19u) && "MODIFIER_SHIFT = bit19");
    assert(apc::INPUT_MODIFIER_ALT   == (1u << 20u) && "MODIFIER_ALT = bit20");

    // No overlap between any two buttons
    assert((apc::INPUT_MOVE_FORWARD & apc::INPUT_MOVE_BACK) == 0u);
    assert((apc::INPUT_MOVE_FORWARD & apc::INPUT_SPRINT) == 0u);
    assert((apc::INPUT_SPRINT & apc::INPUT_ACTION_A) == 0u);

    std::printf("    [PASS] All 21 button bitmask positions verified, no overlap\n");
    return 0;
}

// =============================================================================
// TEST 2: InputAxis defaults
// =============================================================================
static int test_input_axis_defaults() {
    std::printf("  [Test 2] InputAxis defaults...\n");

    apc::InputAxis axis;

    assert(approx_eq(axis.x, 0.0f) && "Default x = 0");
    assert(approx_eq(axis.y, 0.0f) && "Default y = 0");
    assert(approx_eq(axis.z, 0.0f) && "Default z = 0");

    // Zero magnitude at default
    assert(approx_eq(axis.magnitude(), 0.0f) && "Default magnitude = 0");
    assert(approx_eq(axis.magnitude_xz(), 0.0f) && "Default magnitude_xz = 0");

    std::printf("    [PASS] InputAxis default (0, 0, 0) verified\n");
    return 0;
}

// =============================================================================
// TEST 3: InputAxis magnitude calculation
// =============================================================================
static int test_input_axis_magnitude() {
    std::printf("  [Test 3] InputAxis magnitude calculation...\n");

    // Unit vectors
    apc::InputAxis ux{1.0f, 0.0f, 0.0f};
    assert(approx_eq(ux.magnitude(), 1.0f) && "Unit X magnitude = 1");

    apc::InputAxis uy{0.0f, 1.0f, 0.0f};
    assert(approx_eq(uy.magnitude(), 1.0f) && "Unit Y magnitude = 1");

    apc::InputAxis uz{0.0f, 0.0f, 1.0f};
    assert(approx_eq(uz.magnitude(), 1.0f) && "Unit Z magnitude = 1");

    // 3-4-5 triangle: 3^2 + 4^2 + 0^2 = 25, sqrt = 5
    apc::InputAxis tri{3.0f, 4.0f, 0.0f};
    assert(approx_eq(tri.magnitude(), 5.0f) && "3-4-5 magnitude = 5");

    // magnitude_xz ignores y component
    apc::InputAxis with_y{3.0f, 12.0f, 4.0f};
    assert(approx_eq(with_y.magnitude_xz(), 5.0f) && "XZ magnitude ignores Y (3-4-5)");
    assert(approx_eq(with_y.magnitude(), 13.0f) && "Full magnitude = 13");

    // normalize_xz
    apc::InputAxis n{3.0f, 99.0f, 4.0f};
    n.normalize_xz();
    assert(approx_eq(n.x, 0.6f) && "normalize_xz: x = 3/5 = 0.6");
    assert(approx_eq(n.z, 0.8f) && "normalize_xz: z = 4/5 = 0.8");
    // Y should be unchanged by normalize_xz
    assert(approx_eq(n.y, 99.0f) && "normalize_xz: y unchanged");

    // normalize_xz on zero vector
    apc::InputAxis zero;
    zero.normalize_xz();
    assert(approx_eq(zero.x, 0.0f) && "normalize_xz zero: x stays 0");
    assert(approx_eq(zero.z, 0.0f) && "normalize_xz zero: z stays 0");

    std::printf("    [PASS] Magnitude, magnitude_xz, normalize_xz all correct\n");
    return 0;
}

// =============================================================================
// TEST 4: MotorIntent default values
// =============================================================================
static int test_motor_intent_defaults() {
    std::printf("  [Test 4] MotorIntent default values...\n");

    apc::MotorIntent intent;

    // Locomotion defaults
    assert(approx_eq(intent.move_direction.x, 0.0f) && "move_direction.x = 0");
    assert(approx_eq(intent.move_direction.y, 0.0f) && "move_direction.y = 0");
    assert(approx_eq(intent.move_direction.z, 0.0f) && "move_direction.z = 0");
    assert(approx_eq(intent.move_speed, 0.0f) && "move_speed = 0");
    assert(approx_eq(intent.sprint_intensity, 0.0f) && "sprint_intensity = 0");

    // Look defaults to -Z
    assert(approx_eq(intent.look_direction.x, 0.0f) && "look_direction.x = 0");
    assert(approx_eq(intent.look_direction.y, 0.0f) && "look_direction.y = 0");
    assert(approx_eq(intent.look_direction.z, -1.0f) && "look_direction.z = -1");

    // Jump defaults
    assert(approx_eq(intent.jump_impulse.x, 0.0f) && "jump_impulse.x = 0");
    assert(approx_eq(intent.jump_impulse.y, 0.0f) && "jump_impulse.y = 0");
    assert(approx_eq(intent.jump_impulse.z, 0.0f) && "jump_impulse.z = 0");

    // Crouch / Slide
    assert(approx_eq(intent.crouch_amount, 0.0f) && "crouch_amount = 0");
    assert(approx_eq(intent.slide_intensity, 0.0f) && "slide_intensity = 0");

    // Buttons and action
    assert(intent.buttons == 0u && "buttons = 0");
    assert(intent.action_type == apc::ACTION_IDLE && "action_type = IDLE");

    // has_locomotion returns 0 for default
    assert(intent.has_locomotion() == 0 && "Default has no locomotion");

    std::printf("    [PASS] MotorIntent all defaults verified (including look=-Z)\n");
    return 0;
}

// =============================================================================
// TEST 5: MotorIntentFlags bitmask values
// =============================================================================
static int test_motor_intent_flags_bitmask() {
    std::printf("  [Test 5] MotorIntentFlags bitmask values...\n");

    assert(apc::FLAG_AIMING       == (1u << 0u) && "AIMING = bit0");
    assert(apc::FLAG_CHARGING     == (1u << 1u) && "CHARGING = bit1");
    assert(apc::FLAG_KNOCKBACK    == (1u << 2u) && "KNOCKBACK = bit2");
    assert(apc::FLAG_STUNNED      == (1u << 3u) && "STUNNED = bit3");
    assert(apc::FLAG_GROUNDED     == (1u << 4u) && "GROUNDED = bit4");
    assert(apc::FLAG_IN_AIR       == (1u << 5u) && "IN_AIR = bit5");
    assert(apc::FLAG_BALL_CARRIER == (1u << 6u) && "BALL_CARRIER = bit6");
    assert(apc::FLAG_DEFENDING    == (1u << 7u) && "DEFENDING = bit7");
    assert(apc::FLAG_OFFSIDING    == (1u << 8u) && "OFFSIDING = bit8");
    assert(apc::FLAG_INJURED      == (1u << 9u) && "INJURED = bit9");

    // Combine flags
    uint16_t combined = apc::FLAG_AIMING | apc::FLAG_GROUNDED | apc::FLAG_BALL_CARRIER;
    assert(combined == (1u | 16u | 64u) && "Combined flags = 0b1010001 = 81");

    // Mutually exclusive: GROUNDED and IN_AIR should not share bits
    assert((apc::FLAG_GROUNDED & apc::FLAG_IN_AIR) == 0u && "GROUNDED/IN_AIR disjoint");

    std::printf("    [PASS] All 10 flag bitmasks verified, no bit overlap\n");
    return 0;
}

// =============================================================================
// TEST 6: ActionType enum values
// =============================================================================
static int test_action_type_enum_values() {
    std::printf("  [Test 6] ActionType enum values...\n");

    assert(apc::ACTION_IDLE   == 0  && "IDLE = 0");
    assert(apc::ACTION_MOVE   == 1  && "MOVE = 1");
    assert(apc::ACTION_SPRINT == 2  && "SPRINT = 2");
    assert(apc::ACTION_JUMP   == 3  && "JUMP = 3");
    assert(apc::ACTION_CROUCH == 4  && "CROUCH = 4");
    assert(apc::ACTION_SLIDE  == 5  && "SLIDE = 5");
    assert(apc::ACTION_TACKLE == 6  && "TACKLE = 6");
    assert(apc::ACTION_THROW  == 7  && "THROW = 7");
    assert(apc::ACTION_CATCH  == 8  && "CATCH = 8");
    assert(apc::ACTION_SHOOT  == 9  && "SHOOT = 9");
    assert(apc::ACTION_PASS   == 10 && "PASS = 10");
    assert(apc::ACTION_COUNT  == 11 && "COUNT = 11");

    // Type is uint8_t
    static_assert(sizeof(apc::ActionType) == 1, "ActionType must be uint8_t");

    // Monotonically increasing
    assert(apc::ACTION_IDLE < apc::ACTION_MOVE);
    assert(apc::ACTION_MOVE < apc::ACTION_SPRINT);
    assert(apc::ACTION_PASS < apc::ACTION_COUNT);

    std::printf("    [PASS] All 12 ActionType values verified (0-11)\n");
    return 0;
}

// =============================================================================
// TEST 7: InputState default values
// =============================================================================
static int test_input_state_defaults() {
    std::printf("  [Test 7] InputState default values...\n");

    apc::InputState state;

    // Buttons
    assert(state.buttons_down    == 0u && "buttons_down = 0");
    assert(state.buttons_pressed == 0u && "buttons_pressed = 0");
    assert(state.buttons_released == 0u && "buttons_released = 0");
    assert(state.prev_buttons_down == 0u && "prev_buttons_down = 0");

    // Sticks zeroed
    assert(approx_eq(state.left_stick.x, 0.0f) && "left_stick.x = 0");
    assert(approx_eq(state.left_stick.y, 0.0f) && "left_stick.y = 0");
    assert(approx_eq(state.left_stick.z, 0.0f) && "left_stick.z = 0");
    assert(approx_eq(state.right_stick.x, 0.0f) && "right_stick.x = 0");
    assert(approx_eq(state.right_stick.y, 0.0f) && "right_stick.y = 0");
    assert(approx_eq(state.right_stick.z, 0.0f) && "right_stick.z = 0");

    // Triggers
    assert(approx_eq(state.trigger_left, 0.0f) && "trigger_left = 0");
    assert(approx_eq(state.trigger_right, 0.0f) && "trigger_right = 0");

    // Config
    assert(approx_eq(state.dead_zone, 0.15f) && "dead_zone = 0.15");

    // Frame
    assert(state.frame_number == 0u && "frame_number = 0");

    std::printf("    [PASS] InputState all defaults verified\n");
    return 0;
}

// =============================================================================
// TEST 8: InputState dead zone application
// =============================================================================
static int test_input_state_dead_zone() {
    std::printf("  [Test 8] InputState dead zone application...\n");

    apc::InputState state;

    // Below dead zone (0.1 < 0.15): should zero out
    state.set_left_stick(0.1f, 0.05f);
    state.apply_dead_zone();
    assert(approx_eq(state.left_stick.x, 0.0f) && "Below dead zone: x zeroed");
    assert(approx_eq(state.left_stick.y, 0.0f) && "Below dead zone: y zeroed");

    // Just below dead zone edge
    state.set_left_stick(0.14f, 0.0f);
    state.apply_dead_zone();
    assert(approx_eq(state.left_stick.x, 0.0f) && "Just below dead zone: x zeroed");

    // Just above dead zone: should be non-zero and rescaled
    state.set_left_stick(0.2f, 0.0f);
    state.apply_dead_zone();
    float mag = std::sqrt(state.left_stick.x * state.left_stick.x +
                          state.left_stick.y * state.left_stick.y);
    assert(mag > 0.0f && "Above dead zone: non-zero magnitude");
    // Rescaled: (0.2 - 0.15) / (1.0 - 0.15) = 0.05 / 0.85 ~ 0.0588
    assert(state.left_stick.x > 0.0f && "Rescaled x > 0");

    // Full deflection: should be approximately 1.0 after rescale
    state.set_left_stick(1.0f, 0.0f);
    state.apply_dead_zone();
    assert(approx_eq(state.left_stick.x, 1.0f) && "Full deflection: x = 1.0");

    // Both sticks get dead zone applied
    state.set_right_stick(0.0f, 0.1f);
    state.apply_dead_zone();
    assert(approx_eq(state.right_stick.y, 0.0f) && "Right stick also zeroed below dead zone");

    // Negative values below dead zone
    state.set_left_stick(-0.1f, -0.1f);
    state.apply_dead_zone();
    assert(approx_eq(state.left_stick.x, 0.0f) && "Negative below dead zone: x zeroed");
    assert(approx_eq(state.left_stick.y, 0.0f) && "Negative below dead zone: y zeroed");

    std::printf("    [PASS] Dead zone zeroes below threshold, rescales above\n");
    return 0;
}

// =============================================================================
// TEST 9: InputState frame_delta (pressed/released detection)
// =============================================================================
static int test_input_state_frame_delta() {
    std::printf("  [Test 9] InputState frame_delta pressed/released...\n");

    apc::InputState state;

    // Frame 1: press ACTION_A
    state.begin_frame();
    state.press_button(apc::INPUT_ACTION_A);
    state.frame_delta();

    assert((state.buttons_pressed & apc::INPUT_ACTION_A) != 0u
           && "Frame 1: ACTION_A pressed detected");
    assert((state.buttons_released & apc::INPUT_ACTION_A) == 0u
           && "Frame 1: no release for A");
    assert(state.is_button_pressed(apc::INPUT_ACTION_A) == 1u
           && "is_button_pressed(A) = 1");

    // Frame 2: keep A held — no pressed/released
    state.begin_frame();
    // Don't press or release anything
    state.frame_delta();

    assert((state.buttons_pressed & apc::INPUT_ACTION_A) == 0u
           && "Frame 2: A not pressed again (held)");
    assert((state.buttons_released & apc::INPUT_ACTION_A) == 0u
           && "Frame 2: A not released (still held)");
    assert(state.is_button_held(apc::INPUT_ACTION_A) == 1u
           && "Frame 2: A still held");

    // Frame 3: release A
    state.begin_frame();
    state.release_button(apc::INPUT_ACTION_A);
    state.frame_delta();

    assert((state.buttons_released & apc::INPUT_ACTION_A) != 0u
           && "Frame 3: A release detected");
    assert((state.buttons_pressed & apc::INPUT_ACTION_A) == 0u
           && "Frame 3: A not pressed");
    assert(state.is_button_held(apc::INPUT_ACTION_A) == 0u
           && "Frame 3: A no longer held");

    // Multiple buttons in same frame
    state.begin_frame();
    state.press_button(apc::INPUT_MOVE_FORWARD);
    state.press_button(apc::INPUT_SPRINT);
    state.frame_delta();

    assert(state.is_button_pressed(apc::INPUT_MOVE_FORWARD) == 1u
           && "Multi-press: FORWARD detected");
    assert(state.is_button_pressed(apc::INPUT_SPRINT) == 1u
           && "Multi-press: SPRINT detected");

    std::printf("    [PASS] frame_delta: pressed, released, held all correct\n");
    return 0;
}

// =============================================================================
// TEST 10: InputState is_button_held query
// =============================================================================
static int test_input_state_is_button_held() {
    std::printf("  [Test 10] InputState is_button_held query...\n");

    apc::InputState state;

    // Nothing held initially
    assert(state.is_button_held(apc::INPUT_MOVE_FORWARD) == 0u);
    assert(state.is_button_held(apc::INPUT_SPRINT) == 0u);
    assert(state.is_button_held(apc::INPUT_JUMP) == 0u);
    assert(state.is_button_held(apc::INPUT_ACTION_A) == 0u);

    // Press a button — immediately held (no frame_delta needed)
    state.press_button(apc::INPUT_JUMP);
    assert(state.is_button_held(apc::INPUT_JUMP) == 1u);

    // Other buttons still not held
    assert(state.is_button_held(apc::INPUT_MOVE_FORWARD) == 0u);

    // Release — no longer held
    state.release_button(apc::INPUT_JUMP);
    assert(state.is_button_held(apc::INPUT_JUMP) == 0u);

    // Multiple buttons held
    state.press_button(apc::INPUT_MOVE_FORWARD);
    state.press_button(apc::INPUT_MOVE_LEFT);
    assert(state.is_button_held(apc::INPUT_MOVE_FORWARD) == 1u);
    assert(state.is_button_held(apc::INPUT_MOVE_LEFT) == 1u);

    // Combined mask query
    uint32_t combo = apc::INPUT_MOVE_FORWARD | apc::INPUT_MOVE_LEFT;
    assert(state.is_button_held(combo) == 1u && "Held check works with combined mask");

    std::printf("    [PASS] is_button_held correct for all button states\n");
    return 0;
}

// =============================================================================
// TEST 11: InputState is_button_pressed query
// =============================================================================
static int test_input_state_is_button_pressed() {
    std::printf("  [Test 11] InputState is_button_pressed query...\n");

    apc::InputState state;

    // Not pressed before any frame processing
    assert(state.is_button_pressed(apc::INPUT_JUMP) == 0u);

    // First frame: press jump
    state.begin_frame();
    state.press_button(apc::INPUT_JUMP);
    state.frame_delta();
    assert(state.is_button_pressed(apc::INPUT_JUMP) == 1u
           && "Pressed detected on frame of press");

    // Second frame: still held but not "pressed" (one-shot)
    state.begin_frame();
    state.frame_delta();
    assert(state.is_button_pressed(apc::INPUT_JUMP) == 0u
           && "Not pressed on second frame (still held)");

    // Third frame: release
    state.begin_frame();
    state.release_button(apc::INPUT_JUMP);
    state.frame_delta();
    assert(state.is_button_pressed(apc::INPUT_JUMP) == 0u
           && "Not pressed on release frame");

    // Fourth frame: press again — freshly pressed
    state.begin_frame();
    state.press_button(apc::INPUT_JUMP);
    state.frame_delta();
    assert(state.is_button_pressed(apc::INPUT_JUMP) == 1u
           && "Re-press detected as fresh press");

    std::printf("    [PASS] is_button_pressed: one-shot detection correct\n");
    return 0;
}

// =============================================================================
// TEST 12: InputState get_move_vector from sticks
// =============================================================================
static int test_input_state_get_move_vector() {
    std::printf("  [Test 12] InputState get_move_vector...\n");

    apc::InputState state;

    // Default: no input → zero vector
    apc::Vec3 v0 = state.get_move_vector();
    assert(approx_eq(v0.x, 0.0f) && "Default move x = 0");
    assert(approx_eq(v0.y, 0.0f) && "Default move y = 0");
    assert(approx_eq(v0.z, 0.0f) && "Default move z = 0");

    // Left stick Y maps to world Z
    state.set_left_stick(0.0f, 0.5f);
    apc::Vec3 v1 = state.get_move_vector();
    assert(approx_eq(v1.x, 0.0f) && "Stick Y=0.5: move x = 0");
    assert(approx_eq(v1.z, 0.5f) && "Stick Y=0.5: move z = 0.5");

    // Left stick X maps to world X
    state.set_left_stick(0.7f, 0.0f);
    apc::Vec3 v2 = state.get_move_vector();
    assert(approx_eq(v2.x, 0.7f) && "Stick X=0.7: move x = 0.7");
    assert(approx_eq(v2.z, 0.0f) && "Stick X=0.7: move z = 0");

    // D-pad forward: subtracts from Z
    state.set_left_stick(0.0f, 0.0f);
    state.press_button(apc::INPUT_MOVE_FORWARD);
    apc::Vec3 v3 = state.get_move_vector();
    assert(approx_eq(v3.x, 0.0f) && "D-pad forward: x = 0");
    assert(approx_eq(v3.z, -1.0f) && "D-pad forward: z = -1");

    // D-pad back: adds to Z
    state.release_button(apc::INPUT_MOVE_FORWARD);
    state.press_button(apc::INPUT_MOVE_BACK);
    apc::Vec3 v4 = state.get_move_vector();
    assert(approx_eq(v4.z, 1.0f) && "D-pad back: z = +1");

    // D-pad left/right
    state.release_button(apc::INPUT_MOVE_BACK);
    state.press_button(apc::INPUT_MOVE_LEFT);
    apc::Vec3 v5 = state.get_move_vector();
    assert(approx_eq(v5.x, -1.0f) && "D-pad left: x = -1");

    state.release_button(apc::INPUT_MOVE_LEFT);
    state.press_button(apc::INPUT_MOVE_RIGHT);
    apc::Vec3 v6 = state.get_move_vector();
    assert(approx_eq(v6.x, 1.0f) && "D-pad right: x = +1");

    // Stick + D-pad combine (clamped to [-1, 1])
    state.release_button(apc::INPUT_MOVE_RIGHT);
    state.set_left_stick(0.8f, 0.0f);
    state.press_button(apc::INPUT_MOVE_RIGHT);
    apc::Vec3 v7 = state.get_move_vector();
    assert(approx_eq(v7.x, 1.0f) && "Stick+D-pad: x clamped to 1");

    std::printf("    [PASS] get_move_vector: stick, D-pad, and combination correct\n");
    return 0;
}

// =============================================================================
// TEST 13: IntentConverter default values
// =============================================================================
static int test_intent_converter_defaults() {
    std::printf("  [Test 13] IntentConverter default values...\n");

    apc::IntentConverter cv;

    assert(approx_eq(cv.walk_speed, 4.0f) && "walk_speed = 4.0");
    assert(approx_eq(cv.run_speed, 7.0f) && "run_speed = 7.0");
    assert(approx_eq(cv.sprint_multiplier, 1.5f) && "sprint_multiplier = 1.5");
    assert(approx_eq(cv.jump_force, 8.0f) && "jump_force = 8.0");
    assert(approx_eq(cv.crouch_transition_speed, 5.0f) && "crouch_transition_speed = 5.0");
    assert(approx_eq(cv.slide_threshold, 0.7f) && "slide_threshold = 0.7");
    assert(approx_eq(cv.look_sensitivity, 1.0f) && "look_sensitivity = 1.0");
    assert(approx_eq(cv.dead_zone, 0.15f) && "dead_zone = 0.15");

    // World up
    assert(approx_eq(cv.world_up.x, 0.0f) && "world_up.x = 0");
    assert(approx_eq(cv.world_up.y, 1.0f) && "world_up.y = 1");
    assert(approx_eq(cv.world_up.z, 0.0f) && "world_up.z = 0");

    std::printf("    [PASS] IntentConverter all 9 defaults verified\n");
    return 0;
}

// =============================================================================
// TEST 14: IntentConverter.convert() basic movement (forward)
// =============================================================================
static int test_intent_converter_convert_forward() {
    std::printf("  [Test 14] IntentConverter.convert() basic forward movement...\n");

    apc::InputState input;
    apc::IntentConverter cv;

    // Push stick fully forward (Y=1 → world Z=1)
    input.set_left_stick(0.0f, 1.0f);

    // Camera looking along -Z
    apc::Vec3 camera_forward(0.0f, 0.0f, -1.0f);
    apc::Vec3 entity_pos(0.0f, 0.0f, 0.0f);

    apc::MotorIntent intent = cv.convert(input, camera_forward, entity_pos);

    // Move direction should be camera forward = (0, 0, -1)
    assert(approx_eq(intent.move_direction.x, 0.0f) && "Forward move: dir.x = 0");
    assert(approx_eq(intent.move_direction.y, 0.0f) && "Forward move: dir.y = 0");
    assert(approx_eq(intent.move_direction.z, -1.0f) && "Forward move: dir.z = -1");

    // Speed should be full (1.0)
    assert(approx_eq(intent.move_speed, 1.0f) && "Forward move: speed = 1.0");

    // No sprint
    assert(approx_eq(intent.sprint_intensity, 0.0f) && "No sprint held");

    // Action should be MOVE
    assert(intent.action_type == apc::ACTION_MOVE && "Forward → ACTION_MOVE");

    // No jump impulse
    assert(approx_eq(intent.jump_impulse.y, 0.0f) && "No jump impulse");

    // --- Sprint: hold sprint button ---
    input.press_button(apc::INPUT_SPRINT);
    input.begin_frame();
    input.frame_delta();

    apc::MotorIntent sprint_intent = cv.convert(input, camera_forward, entity_pos);
    assert(sprint_intent.sprint_intensity > 0.0f && "Sprint: intensity > 0");
    assert(sprint_intent.action_type == apc::ACTION_SPRINT && "Sprint → ACTION_SPRINT");
    assert(sprint_intent.move_speed > 1.0f && "Sprint: speed multiplied");

    // --- No input: idle ---
    apc::InputState empty;
    apc::MotorIntent idle_intent = cv.convert(empty, camera_forward, entity_pos);
    assert(idle_intent.action_type == apc::ACTION_IDLE && "No input → ACTION_IDLE");
    assert(approx_eq(idle_intent.move_speed, 0.0f) && "No input: speed = 0");

    std::printf("    [PASS] Forward, sprint, and idle conversion all correct\n");
    return 0;
}

// =============================================================================
// TEST 15: IntentConverter sport presets
// =============================================================================
static int test_intent_converter_sport_presets() {
    std::printf("  [Test 15] IntentConverter sport presets...\n");

    // --- Soccer ---
    apc::IntentConverter soccer = apc::IntentConverter::soccer_defaults();
    assert(approx_eq(soccer.walk_speed, 3.5f) && "Soccer walk_speed = 3.5");
    assert(approx_eq(soccer.run_speed, 7.0f) && "Soccer run_speed = 7.0");
    assert(approx_eq(soccer.sprint_multiplier, 1.5f) && "Soccer sprint_mult = 1.5");
    assert(approx_eq(soccer.jump_force, 5.0f) && "Soccer jump = 5.0 (low)");
    assert(approx_eq(soccer.slide_threshold, 0.7f) && "Soccer slide = 0.7");
    assert(approx_eq(soccer.dead_zone, 0.15f) && "Soccer dead_zone = 0.15");

    // --- Basketball ---
    apc::IntentConverter basketball = apc::IntentConverter::basketball_defaults();
    assert(approx_eq(basketball.walk_speed, 3.0f) && "Basketball walk = 3.0");
    assert(approx_eq(basketball.run_speed, 6.0f) && "Basketball run = 6.0");
    assert(approx_eq(basketball.sprint_multiplier, 1.4f) && "Basketball sprint = 1.4");
    assert(approx_eq(basketball.jump_force, 10.0f) && "Basketball jump = 10.0 (medium)");
    assert(approx_eq(basketball.slide_threshold, 0.0f) && "Basketball slide = 0 (disabled)");
    assert(approx_eq(basketball.look_sensitivity, 1.2f) && "Basketball look = 1.2");
    assert(approx_eq(basketball.dead_zone, 0.12f) && "Basketball dead_zone = 0.12");

    // --- American Football ---
    apc::IntentConverter football = apc::IntentConverter::football_defaults();
    assert(approx_eq(football.walk_speed, 3.5f) && "Football walk = 3.5");
    assert(approx_eq(football.run_speed, 8.0f) && "Football run = 8.0 (fast)");
    assert(approx_eq(football.sprint_multiplier, 1.6f) && "Football sprint = 1.6");
    assert(approx_eq(football.jump_force, 4.0f) && "Football jump = 4.0 (low)");
    assert(approx_eq(football.slide_threshold, 0.0f) && "Football slide = 0 (disabled)");
    assert(approx_eq(football.look_sensitivity, 0.9f) && "Football look = 0.9");

    // --- Rugby ---
    apc::IntentConverter rugby = apc::IntentConverter::rugby_defaults();
    assert(approx_eq(rugby.walk_speed, 3.5f) && "Rugby walk = 3.5");
    assert(approx_eq(rugby.run_speed, 7.5f) && "Rugby run = 7.5");
    assert(approx_eq(rugby.sprint_multiplier, 1.5f) && "Rugby sprint = 1.5");
    assert(approx_eq(rugby.jump_force, 5.0f) && "Rugby jump = 5.0");
    assert(approx_eq(rugby.slide_threshold, 0.0f) && "Rugby slide = 0 (disabled)");
    assert(approx_eq(rugby.crouch_transition_speed, 4.5f) && "Rugby crouch = 4.5");

    // Each preset produces a valid converter that can convert without crash
    apc::InputState empty;
    apc::Vec3 cam(0.0f, 0.0f, -1.0f);
    apc::Vec3 pos(0.0f, 0.0f, 0.0f);

    apc::MotorIntent s_intent = soccer.convert(empty, cam, pos);
    assert(s_intent.action_type == apc::ACTION_IDLE && "Soccer preset converts idle");

    apc::MotorIntent b_intent = basketball.convert(empty, cam, pos);
    assert(b_intent.action_type == apc::ACTION_IDLE && "Basketball preset converts idle");

    apc::MotorIntent f_intent = football.convert(empty, cam, pos);
    assert(f_intent.action_type == apc::ACTION_IDLE && "Football preset converts idle");

    apc::MotorIntent r_intent = rugby.convert(empty, cam, pos);
    assert(r_intent.action_type == apc::ACTION_IDLE && "Rugby preset converts idle");

    // Sport-specific differentiation
    assert(basketball.jump_force > soccer.jump_force && "Basketball jumps higher than soccer");
    assert(football.run_speed > soccer.run_speed && "Football runs faster than soccer");
    assert(soccer.slide_threshold > basketball.slide_threshold && "Soccer has slide, basketball doesn't");

    std::printf("    [PASS] All 4 sport presets verified with correct sport-specific values\n");
    return 0;
}

// =============================================================================
// TEST 16 (Bonus): IntentConverter.convert() crouch and jump
// =============================================================================
static int test_intent_converter_convert_crouch_jump() {
    std::printf("  [Test 16] IntentConverter.convert() crouch and jump...\n");

    apc::InputState input;
    apc::IntentConverter cv;
    apc::Vec3 cam(0.0f, 0.0f, -1.0f);
    apc::Vec3 pos(0.0f, 0.0f, 0.0f);

    // --- Crouch ---
    input.begin_frame();
    input.press_button(apc::INPUT_CROUCH);
    input.frame_delta();

    apc::MotorIntent crouch = cv.convert(input, cam, pos);
    assert(crouch.crouch_amount > 0.0f && "Crouch: amount > 0");
    // Note: convert() step 7 overrides action_type; with no movement it becomes ACTION_IDLE
    assert(crouch.action_type == apc::ACTION_IDLE && "Crouch with no movement → ACTION_IDLE");
    // Crouch reduces speed
    assert(crouch.move_speed < 0.1f && "Crouch with no movement: speed ~0");

    // --- Jump ---
    apc::InputState input2;
    // No left stick movement — otherwise step 7 overrides ACTION_JUMP to ACTION_MOVE
    input2.begin_frame();
    input2.press_button(apc::INPUT_JUMP);
    input2.frame_delta();

    apc::MotorIntent jump = cv.convert(input2, cam, pos);
    // Note: convert() step 7 overrides action_type; jump impulse is still set
    assert(jump.jump_impulse.y > 0.0f && "Jump impulse: Y > 0");
    assert(approx_eq(jump.jump_impulse.y, cv.jump_force) && "Jump impulse Y = jump_force");
    assert(approx_eq(jump.jump_impulse.x, 0.0f) && "Jump impulse X = 0");

    // --- A-button: pass ---
    apc::InputState input3;
    input3.begin_frame();
    input3.press_button(apc::INPUT_ACTION_A);
    input3.frame_delta();

    apc::MotorIntent pass = cv.convert(input3, cam, pos);
    assert(pass.action_type == apc::ACTION_PASS && "A-button → ACTION_PASS");

    // --- A-button + R-shoulder: shoot ---
    apc::InputState input4;
    input4.begin_frame();
    input4.press_button(apc::INPUT_ACTION_A);
    input4.press_button(apc::INPUT_SHOULDER_R);
    input4.frame_delta();

    apc::MotorIntent shoot = cv.convert(input4, cam, pos);
    assert(shoot.action_type == apc::ACTION_SHOOT && "A+R → ACTION_SHOOT");

    // --- B-button: tackle ---
    apc::InputState input5;
    input5.begin_frame();
    input5.press_button(apc::INPUT_ACTION_B);
    input5.frame_delta();

    apc::MotorIntent tackle = cv.convert(input5, cam, pos);
    assert(tackle.action_type == apc::ACTION_TACKLE && "B → ACTION_TACKLE");

    // --- X-button: catch ---
    apc::InputState input6;
    input6.begin_frame();
    input6.press_button(apc::INPUT_ACTION_X);
    input6.frame_delta();

    apc::MotorIntent cat = cv.convert(input6, cam, pos);
    assert(cat.action_type == apc::ACTION_CATCH && "X → ACTION_CATCH");

    // --- Y-button: throw ---
    apc::InputState input7;
    input7.begin_frame();
    input7.press_button(apc::INPUT_ACTION_Y);
    input7.frame_delta();

    apc::MotorIntent thr = cv.convert(input7, cam, pos);
    assert(thr.action_type == apc::ACTION_THROW && "Y → ACTION_THROW");

    std::printf("    [PASS] Crouch, jump, pass, shoot, tackle, catch, throw all correct\n");
    return 0;
}

// =============================================================================
// TEST 17 (Bonus): MotorIntent reset and has_locomotion
// =============================================================================
static int test_motor_intent_reset_and_locomotion() {
    std::printf("  [Test 17] MotorIntent reset and has_locomotion...\n");

    apc::MotorIntent intent;

    // Modify fields
    intent.move_direction = {1.0f, 2.0f, 3.0f};
    intent.move_speed = 0.8f;
    intent.sprint_intensity = 1.0f;
    intent.jump_impulse = {0.0f, 10.0f, 0.0f};
    intent.crouch_amount = 0.5f;
    intent.slide_intensity = 0.3f;
    intent.buttons = apc::INPUT_MOVE_FORWARD;
    intent.action_type = apc::ACTION_SPRINT;
    intent.look_direction = {1.0f, 0.0f, 0.0f};

    assert(intent.has_locomotion() == 1 && "Modified intent has locomotion");

    // Reset
    intent.reset();

    assert(approx_eq(intent.move_direction.x, 0.0f) && "After reset: dir.x = 0");
    assert(approx_eq(intent.move_speed, 0.0f) && "After reset: speed = 0");
    assert(approx_eq(intent.sprint_intensity, 0.0f) && "After reset: sprint = 0");
    assert(approx_eq(intent.look_direction.z, -1.0f) && "After reset: look = -Z");
    assert(approx_eq(intent.jump_impulse.y, 0.0f) && "After reset: jump = 0");
    assert(approx_eq(intent.crouch_amount, 0.0f) && "After reset: crouch = 0");
    assert(approx_eq(intent.slide_intensity, 0.0f) && "After reset: slide = 0");
    assert(intent.buttons == 0u && "After reset: buttons = 0");
    assert(intent.action_type == apc::ACTION_IDLE && "After reset: action = IDLE");
    assert(intent.has_locomotion() == 0 && "After reset: no locomotion");

    // Locomotion via sprint_intensity alone
    apc::MotorIntent sprinting;
    sprinting.sprint_intensity = 0.5f;
    assert(sprinting.has_locomotion() == 1 && "sprint_intensity > 0 triggers locomotion");

    // Locomotion via crouch_amount alone
    apc::MotorIntent crouching;
    crouching.crouch_amount = 0.5f;
    assert(crouching.has_locomotion() == 1 && "crouch_amount > 0 triggers locomotion");

    // Locomotion via slide_intensity alone
    apc::MotorIntent sliding;
    sliding.slide_intensity = 0.5f;
    assert(sliding.has_locomotion() == 1 && "slide_intensity > 0 triggers locomotion");

    // No locomotion with only jump_impulse (that's not locomotion)
    apc::MotorIntent jumping;
    jumping.jump_impulse = {0.0f, 5.0f, 0.0f};
    assert(jumping.has_locomotion() == 0 && "jump_impulse alone: no locomotion");

    std::printf("    [PASS] reset clears everything, has_locomotion checks all relevant fields\n");
    return 0;
}

// =============================================================================
// TEST 18 (Bonus): InputState trigger and look vector
// =============================================================================
static int test_input_state_triggers_and_look() {
    std::printf("  [Test 18] InputState triggers and look vector...\n");

    apc::InputState state;

    // Trigger defaults
    assert(approx_eq(state.trigger_left, 0.0f));
    assert(approx_eq(state.trigger_right, 0.0f));

    // Set triggers (clamped to [0, 1])
    state.set_trigger_left(0.5f);
    assert(approx_eq(state.trigger_left, 0.5f));

    state.set_trigger_right(0.8f);
    assert(approx_eq(state.trigger_right, 0.8f));

    // Clamp above 1.0
    state.set_trigger_left(1.5f);
    assert(approx_eq(state.trigger_left, 1.0f) && "Trigger clamped to 1.0");

    // Clamp below 0.0
    state.set_trigger_right(-0.5f);
    assert(approx_eq(state.trigger_right, 0.0f) && "Trigger clamped to 0.0");

    // Look vector from right stick
    state.set_right_stick(0.5f, 0.3f);
    apc::Vec3 look = state.get_look_vector();
    assert(approx_eq(look.x, 0.5f) && "Look x from right_stick.x");
    assert(approx_eq(look.z, 0.3f) && "Look z from right_stick.y");
    assert(approx_eq(look.y, 0.0f) && "Look y = 0 (ground plane)");

    // Stick clamp to [-1, 1]
    state.set_left_stick(2.0f, -3.0f);
    assert(approx_eq(state.left_stick.x, 1.0f) && "Stick X clamped to 1");
    assert(approx_eq(state.left_stick.y, -1.0f) && "Stick Y clamped to -1");

    std::printf("    [PASS] Triggers and look vector with clamping verified\n");
    return 0;
}

// =============================================================================
// TEST 19 (Bonus): IntentConverter get_action_type
// =============================================================================
static int test_intent_converter_get_action_type() {
    std::printf("  [Test 19] IntentConverter get_action_type...\n");

    apc::IntentConverter cv;
    apc::InputState input;
    uint16_t flags = 0;

    // No input → IDLE
    assert(cv.get_action_type(input, flags) == apc::ACTION_IDLE);

    // Movement → MOVE
    input.set_left_stick(0.5f, 0.0f);
    assert(cv.get_action_type(input, flags) == apc::ACTION_MOVE);

    // Movement + sprint → SPRINT
    input.press_button(apc::INPUT_SPRINT);
    assert(cv.get_action_type(input, flags) == apc::ACTION_SPRINT);

    // A-button → PASS
    apc::InputState input2;
    input2.begin_frame();
    input2.press_button(apc::INPUT_ACTION_A);
    input2.frame_delta();
    assert(cv.get_action_type(input2, flags) == apc::ACTION_PASS);

    // A + R-shoulder → SHOOT
    apc::InputState input3;
    input3.begin_frame();
    input3.press_button(apc::INPUT_ACTION_A);
    input3.press_button(apc::INPUT_SHOULDER_R);
    input3.frame_delta();
    assert(cv.get_action_type(input3, flags) == apc::ACTION_SHOOT);

    std::printf("    [PASS] get_action_type: IDLE, MOVE, SPRINT, PASS, SHOOT correct\n");
    return 0;
}

// =============================================================================
// TEST 20 (Bonus): MAX_SIMULTANEOUS_INPUTS constant
// =============================================================================
static int test_max_simultaneous_inputs() {
    std::printf("  [Test 20] MAX_SIMULTANEOUS_INPUTS constant...\n");

    assert(apc::MAX_SIMULTANEOUS_INPUTS == 4u && "Supports 4 local players");

    // Can create 4 independent input states
    apc::InputState states[apc::MAX_SIMULTANEOUS_INPUTS];
    for (uint32_t i = 0; i < apc::MAX_SIMULTANEOUS_INPUTS; ++i) {
        assert(states[i].buttons_down == 0u);
        assert(states[i].frame_number == 0u);
    }

    // Each can be independently modified
    states[0].press_button(apc::INPUT_MOVE_FORWARD);
    states[1].press_button(apc::INPUT_JUMP);
    states[2].press_button(apc::INPUT_ACTION_A);
    states[3].press_button(apc::INPUT_SPRINT);

    assert(states[0].is_button_held(apc::INPUT_MOVE_FORWARD) == 1u);
    assert(states[1].is_button_held(apc::INPUT_JUMP) == 1u);
    assert(states[2].is_button_held(apc::INPUT_ACTION_A) == 1u);
    assert(states[3].is_button_held(apc::INPUT_SPRINT) == 1u);

    // State 0 does not have state 1's button
    assert(states[0].is_button_held(apc::INPUT_JUMP) == 0u);

    std::printf("    [PASS] 4 independent input states work correctly\n");
    return 0;
}

// =============================================================================
// Main
// =============================================================================
int main() {
    std::printf("=== Sprint 21 Tests ===\n");
    std::printf("Input System: Types, State, Mapping\n");
    std::printf("================================================================\n");

    int fails = 0;
    fails += test_input_button_bitmask_values();
    fails += test_input_axis_defaults();
    fails += test_input_axis_magnitude();
    fails += test_motor_intent_defaults();
    fails += test_motor_intent_flags_bitmask();
    fails += test_action_type_enum_values();
    fails += test_input_state_defaults();
    fails += test_input_state_dead_zone();
    fails += test_input_state_frame_delta();
    fails += test_input_state_is_button_held();
    fails += test_input_state_is_button_pressed();
    fails += test_input_state_get_move_vector();
    fails += test_intent_converter_defaults();
    fails += test_intent_converter_convert_forward();
    fails += test_intent_converter_sport_presets();
    fails += test_intent_converter_convert_crouch_jump();
    fails += test_motor_intent_reset_and_locomotion();
    fails += test_input_state_triggers_and_look();
    fails += test_intent_converter_get_action_type();
    fails += test_max_simultaneous_inputs();

    int total = 20;
    int passed = total - fails;
    std::printf("================================================================\n");
    std::printf("=== Sprint 21: %d tests passed, %d failed ===\n", passed, fails);
    return fails;
}
