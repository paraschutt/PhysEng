// =============================================================================
// Sprint 24 Tests — AI Decision System & Formation System
// =============================================================================
//
// Tests for the APC Sprint 24 headers:
//   1.  AIActionType enum values from decision header
//   2.  ResponseCurve enum values
//   3.  UtilityScore struct defaults
//   4.  Consideration struct defaults
//   5.  UtilityAI default values (counts=0)
//   6.  UtilityAI.add_consideration increments count
//   7.  UtilityAI.evaluate returns valid UtilityScore
//   8.  UtilityAI.evaluate picks highest-scoring action
//   9.  UtilityAI.set_action_weight stores weight
//  10. UtilityAI.configure_role sets role_weights
//  11. ContextFactor enum values
//  12. FormationType enum values
//  13. FormationPosition defaults
//  14. FormationSet defaults (position_count=0)
//  15. FormationSystem defaults
//  16. FormationSystem preset_4_4_2 produces valid FormationSet (11 positions)
//  17. FormationSystem preset_4_3_3 produces valid FormationSet
//  18. FormationSystem preset_3_5_2 produces valid FormationSet
//  19. FormationSystem preset_4_2_3_1 produces valid FormationSet
//  20. FormationSystem.get_formation_position returns valid Vec3
//  21. FormationSystem ball influence pulls nearby positions
//  22. FormationSystem possession factor interpolates defense/attack
//
// Pattern: int main() + assert(), no test framework.
// =============================================================================

#include "apc_ai/apc_ai_decision.h"
#include "apc_ai/apc_ai_formation.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include "apc_math/apc_math_common.h"
#include <cassert>
#include <cstdio>
#include <cmath>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static constexpr float EPS = 1e-4f;

static bool approx_eq(float a, float b, float eps = EPS) {
    return std::abs(a - b) < eps;
}

// =============================================================================
// TEST 1: AIActionType enum values from decision header
// =============================================================================
static int test_action_type_enum() {
    std::printf("  [Test 1] AIActionType enum values from decision header...\n");

    assert(static_cast<uint8_t>(apc::AIActionType::IDLE) == 0);
    assert(static_cast<uint8_t>(apc::AIActionType::MOVE_TO_POSITION) == 1);
    assert(static_cast<uint8_t>(apc::AIActionType::CHASE_BALL) == 2);
    assert(static_cast<uint8_t>(apc::AIActionType::PASS_BALL) == 3);
    assert(static_cast<uint8_t>(apc::AIActionType::SHOOT_BALL) == 4);
    assert(static_cast<uint8_t>(apc::AIActionType::TACKLE) == 5);
    assert(static_cast<uint8_t>(apc::AIActionType::BLOCK) == 6);
    assert(static_cast<uint8_t>(apc::AIActionType::INTERCEPT) == 7);
    assert(static_cast<uint8_t>(apc::AIActionType::MARK_OPPONENT) == 8);
    assert(static_cast<uint8_t>(apc::AIActionType::SUPPORT_RUN) == 9);
    assert(static_cast<uint8_t>(apc::AIActionType::CROSS) == 10);
    assert(static_cast<uint8_t>(apc::AIActionType::HEADER) == 11);
    assert(static_cast<uint8_t>(apc::AIActionType::DIVE_SAVE) == 12);
    assert(static_cast<uint8_t>(apc::AIActionType::PUNT) == 13);
    assert(static_cast<uint8_t>(apc::AIActionType::FORMATION_HOLD) == 14);
    assert(static_cast<uint8_t>(apc::AIActionType::PRESS) == 15);
    assert(static_cast<uint8_t>(apc::AIActionType::ACTION_COUNT) == 16);

    std::printf("    [PASS] AIActionType enum values verified (0-16)\n");
    return 0;
}

// =============================================================================
// TEST 2: ResponseCurve enum values
// =============================================================================
static int test_response_curve_enum() {
    std::printf("  [Test 2] ResponseCurve enum values...\n");

    assert(static_cast<uint8_t>(apc::ResponseCurve::LINEAR) == 0);
    assert(static_cast<uint8_t>(apc::ResponseCurve::QUADRATIC) == 1);
    assert(static_cast<uint8_t>(apc::ResponseCurve::EXPONENTIAL) == 2);

    std::printf("    [PASS] ResponseCurve enum values verified (0-2)\n");
    return 0;
}

// =============================================================================
// TEST 3: UtilityScore struct defaults
// =============================================================================
static int test_utility_score_defaults() {
    std::printf("  [Test 3] UtilityScore struct defaults...\n");

    apc::UtilityScore score;

    assert(approx_eq(score.score, 0.0f) && "score = 0.0");
    assert(score.action == apc::AIActionType::IDLE && "action = IDLE");
    assert(approx_eq(score.confidence, 0.0f) && "confidence = 0.0");

    std::printf("    [PASS] UtilityScore defaults: score=0, IDLE, confidence=0\n");
    return 0;
}

// =============================================================================
// TEST 4: Consideration struct defaults
// =============================================================================
static int test_consideration_defaults() {
    std::printf("  [Test 4] Consideration struct defaults...\n");

    apc::Consideration c;

    // name is "" (nullptr check not needed for default-constructed)
    assert(c.name[0] == '\0' && "name = \"\"");
    assert(approx_eq(c.weight, 1.0f) && "weight = 1.0");
    assert(c.curve == apc::ResponseCurve::LINEAR && "curve = LINEAR");
    assert(approx_eq(c.min_input, 0.0f) && "min_input = 0.0");
    assert(approx_eq(c.max_input, 1.0f) && "max_input = 1.0");

    std::printf("    [PASS] Consideration defaults verified\n");
    return 0;
}

// =============================================================================
// TEST 5: UtilityAI default values (counts=0)
// =============================================================================
static int test_utility_ai_defaults() {
    std::printf("  [Test 5] UtilityAI default values (counts=0)...\n");

    apc::UtilityAI ai;

    assert(ai.consideration_count == 0u && "consideration_count = 0");
    assert(ai.action_count == 0u && "action_count = 0");
    assert(ai.role_weight_count == 0u && "role_weight_count = 0");
    assert(ai.context_factor_count == 0u && "context_factor_count = 0");

    std::printf("    [PASS] UtilityAI defaults: all counts = 0\n");
    return 0;
}

// =============================================================================
// TEST 6: UtilityAI.add_consideration increments count
// =============================================================================
static int test_add_consideration() {
    std::printf("  [Test 6] UtilityAI.add_consideration increments count...\n");

    apc::UtilityAI ai;

    assert(ai.consideration_count == 0u);

    uint8_t r1 = ai.add_consideration("dist_to_ball", 1.0f,
        apc::ResponseCurve::LINEAR, 0.0f, 30.0f);
    assert(r1 == 1 && "add_consideration returns 1 on success");
    assert(ai.consideration_count == 1u && "count = 1 after first add");

    uint8_t r2 = ai.add_consideration("stamina", 0.8f,
        apc::ResponseCurve::QUADRATIC, 0.0f, 100.0f);
    assert(r2 == 1 && "second add_consideration returns 1");
    assert(ai.consideration_count == 2u && "count = 2 after second add");

    // Verify the consideration was stored correctly
    assert(ai.considerations[0].weight == 1.0f && "first weight = 1.0");
    assert(ai.considerations[1].weight == 0.8f && "second weight = 0.8");
    assert(ai.considerations[1].curve == apc::ResponseCurve::QUADRATIC && "second curve = QUADRATIC");

    std::printf("    [PASS] add_consideration increments count and stores values\n");
    return 0;
}

// =============================================================================
// TEST 7: UtilityAI.evaluate returns valid UtilityScore
// =============================================================================
static int test_evaluate_returns_score() {
    std::printf("  [Test 7] UtilityAI.evaluate returns valid UtilityScore...\n");

    apc::UtilityAI ai;

    // Add a consideration: LINEAR, [0,1] range
    ai.add_consideration("proximity", 1.0f, apc::ResponseCurve::LINEAR, 0.0f, 1.0f);

    // Add an action to evaluate
    ai.actions[0] = apc::AIActionType::CHASE_BALL;
    ai.action_count = 1u;

    // Input: proximity = 0.5 → LINEAR(0.5) = 0.5
    float inputs[] = { 0.5f };
    apc::UtilityScore result = ai.evaluate(inputs, 1);

    // Score should be valid: product of all consideration evaluations
    assert(approx_eq(result.score, 0.5f, 0.01f) && "score ≈ 0.5 (LINEAR(0.5))");
    assert(result.action == apc::AIActionType::CHASE_BALL && "selected action = CHASE_BALL");
    assert(result.confidence >= 0.0f && result.confidence <= 1.0f && "confidence in [0,1]");

    std::printf("    [PASS] evaluate returns valid UtilityScore\n");
    return 0;
}

// =============================================================================
// TEST 8: UtilityAI.evaluate picks highest-scoring action
// =============================================================================
static int test_evaluate_picks_best() {
    std::printf("  [Test 8] UtilityAI.evaluate picks highest-scoring action...\n");

    apc::UtilityAI ai;

    // Single consideration: LINEAR [0,1]
    ai.add_consideration("proximity", 1.0f, apc::ResponseCurve::LINEAR, 0.0f, 1.0f);

    // Two actions
    ai.actions[0] = apc::AIActionType::IDLE;
    ai.actions[1] = apc::AIActionType::CHASE_BALL;
    ai.action_count = 2u;

    // Input: proximity = 0.9 → LINEAR(0.9) = 0.9
    float inputs[] = { 0.9f };
    apc::UtilityScore result = ai.evaluate(inputs, 1);

    // Both actions score the same (same consideration), so first one wins
    // (score comparison uses >, so first one encountered with highest score wins)
    assert(result.action == apc::AIActionType::IDLE && "same scores: first action wins");

    // Now set a weight on CHASE_BALL to make it score higher
    ai.set_action_weight(apc::AIActionType::CHASE_BALL, 2.0f);

    // IDLE score = 0.9 * 1.0 (no role weight for index 0 since role_weight_count=1)
    // Actually, role_weight_count becomes 2 (index 1 + 1), so:
    //   IDLE (idx=0): 0.9 * role_weights[0]=1.0 = 0.9
    //   CHASE_BALL (idx=2): 0.9 * role_weights[2]=2.0 = 1.8
    // But wait, role_weight_count after set_action_weight(CHASE_BALL, 2.0):
    //   idx = 2, idx >= role_weight_count (0) → role_weight_count = 3
    //   So for IDLE (idx=0): 0 < 3, so multiply by role_weights[0]=1.0
    //   For CHASE_BALL (idx=2): 2 < 3, so multiply by role_weights[2]=2.0
    apc::UtilityScore result2 = ai.evaluate(inputs, 1);
    assert(result2.action == apc::AIActionType::CHASE_BALL && "weighted: CHASE_BALL wins");
    assert(result2.score > result.score && "weighted score > unweighted score");

    std::printf("    [PASS] evaluate picks highest-scoring action with role weights\n");
    return 0;
}

// =============================================================================
// TEST 9: UtilityAI.set_action_weight stores weight
// =============================================================================
static int test_set_action_weight() {
    std::printf("  [Test 9] UtilityAI.set_action_weight stores weight...\n");

    apc::UtilityAI ai;

    assert(ai.role_weight_count == 0u);

    ai.set_action_weight(apc::AIActionType::SHOOT_BALL, 3.5f);

    uint32_t idx = static_cast<uint32_t>(apc::AIActionType::SHOOT_BALL); // = 4
    assert(approx_eq(ai.role_weights[idx], 3.5f) && "SHOOT_BALL weight = 3.5");
    assert(ai.role_weight_count == idx + 1u && "role_weight_count = idx+1");

    // Setting a lower index should not change count
    ai.set_action_weight(apc::AIActionType::IDLE, 0.5f);
    assert(ai.role_weight_count == idx + 1u && "lower index doesn't increase count");

    // Setting a higher index should increase count
    ai.set_action_weight(apc::AIActionType::PRESS, 2.0f);
    uint32_t press_idx = static_cast<uint32_t>(apc::AIActionType::PRESS); // = 15
    assert(approx_eq(ai.role_weights[press_idx], 2.0f) && "PRESS weight = 2.0");
    assert(ai.role_weight_count == press_idx + 1u && "higher index increases count");

    std::printf("    [PASS] set_action_weight stores weight and updates count\n");
    return 0;
}

// =============================================================================
// TEST 10: UtilityAI.configure_role sets role_weights
// =============================================================================
static int test_configure_role() {
    std::printf("  [Test 10] UtilityAI.configure_role sets role_weights...\n");

    apc::UtilityAI ai;

    ai.configure_role(apc::SportRole::SOCCER_GK);

    // GK should have high DIVE_SAVE weight
    uint32_t dive_idx = static_cast<uint32_t>(apc::AIActionType::DIVE_SAVE); // 12
    assert(approx_eq(ai.role_weights[dive_idx], 5.0f) && "GK: DIVE_SAVE = 5.0");

    uint32_t hold_idx = static_cast<uint32_t>(apc::AIActionType::FORMATION_HOLD); // 14
    assert(approx_eq(ai.role_weights[hold_idx], 4.0f) && "GK: FORMATION_HOLD = 4.0");

    uint32_t punt_idx = static_cast<uint32_t>(apc::AIActionType::PUNT); // 13
    assert(approx_eq(ai.role_weights[punt_idx], 3.0f) && "GK: PUNT = 3.0");

    assert(ai.role_weight_count == apc::MAX_ACTIONS && "configure_role sets count = MAX_ACTIONS");

    // ST (striker) should have high SHOOT weight
    ai.configure_role(apc::SportRole::SOCCER_ST);
    uint32_t shoot_idx = static_cast<uint32_t>(apc::AIActionType::SHOOT_BALL); // 4
    assert(approx_eq(ai.role_weights[shoot_idx], 4.0f) && "ST: SHOOT_BALL = 4.0");

    // CB should have high TACKLE weight
    ai.configure_role(apc::SportRole::SOCCER_CB);
    uint32_t tackle_idx = static_cast<uint32_t>(apc::AIActionType::TACKLE); // 5
    assert(approx_eq(ai.role_weights[tackle_idx], 3.0f) && "CB: TACKLE = 3.0");

    std::printf("    [PASS] configure_role sets correct role-specific weights\n");
    return 0;
}

// =============================================================================
// TEST 11: ContextFactor enum values
// =============================================================================
static int test_context_factor_enum() {
    std::printf("  [Test 11] ContextFactor enum values...\n");

    assert(static_cast<uint8_t>(apc::ContextFactor::DISTANCE_TO_BALL) == 0);
    assert(static_cast<uint8_t>(apc::ContextFactor::DISTANCE_TO_GOAL) == 1);
    assert(static_cast<uint8_t>(apc::ContextFactor::DISTANCE_TO_OPPONENT) == 2);
    assert(static_cast<uint8_t>(apc::ContextFactor::STAMINA_PERCENT) == 3);
    assert(static_cast<uint8_t>(apc::ContextFactor::TEAM_POSSESSION) == 4);
    assert(static_cast<uint8_t>(apc::ContextFactor::TIME_REMAINING) == 5);
    assert(static_cast<uint8_t>(apc::ContextFactor::SCORE_DIFFERENTIAL) == 6);
    assert(static_cast<uint8_t>(apc::ContextFactor::POSITION_QUALITY) == 7);

    std::printf("    [PASS] ContextFactor enum values verified (0-7)\n");
    return 0;
}

// =============================================================================
// TEST 12: FormationType enum values
// =============================================================================
static int test_formation_type_enum() {
    std::printf("  [Test 12] FormationType enum values...\n");

    assert(static_cast<uint8_t>(apc::FormationType::FORMATION_4_4_2) == 0);
    assert(static_cast<uint8_t>(apc::FormationType::FORMATION_4_3_3) == 1);
    assert(static_cast<uint8_t>(apc::FormationType::FORMATION_3_5_2) == 2);
    assert(static_cast<uint8_t>(apc::FormationType::FORMATION_4_2_3_1) == 3);
    assert(static_cast<uint8_t>(apc::FormationType::FORMATION_3_4_3) == 4);
    assert(static_cast<uint8_t>(apc::FormationType::FORMATION_4_5_1) == 5);
    assert(static_cast<uint8_t>(apc::FormationType::FORMATION_5_3_2) == 6);
    assert(static_cast<uint8_t>(apc::FormationType::FORMATION_4_1_4_1) == 7);
    assert(static_cast<uint8_t>(apc::FormationType::CUSTOM) == 255);

    std::printf("    [PASS] FormationType enum values verified (0-7, 255)\n");
    return 0;
}

// =============================================================================
// TEST 13: FormationPosition defaults
// =============================================================================
static int test_formation_position_defaults() {
    std::printf("  [Test 13] FormationPosition defaults...\n");

    apc::FormationPosition pos;

    assert(approx_eq(pos.base_position.x, 0.0f) && "base_position.x = 0");
    assert(approx_eq(pos.base_position.y, 0.0f) && "base_position.y = 0");
    assert(approx_eq(pos.base_position.z, 0.0f) && "base_position.z = 0");
    assert(approx_eq(pos.defensive_shift, 0.0f) && "defensive_shift = 0");
    assert(approx_eq(pos.attacking_shift, 0.0f) && "attacking_shift = 0");
    assert(approx_eq(pos.width_role, 0.0f) && "width_role = 0");
    assert(pos.compatible_role == apc::SportRole::SOCCER_CM && "compatible_role = SOCCER_CM");

    std::printf("    [PASS] FormationPosition defaults verified\n");
    return 0;
}

// =============================================================================
// TEST 14: FormationSet defaults (position_count=0)
// =============================================================================
static int test_formation_set_defaults() {
    std::printf("  [Test 14] FormationSet defaults (position_count=0)...\n");

    apc::FormationSet fs;

    assert(fs.type == apc::FormationType::FORMATION_4_4_2 && "type = FORMATION_4_4_2");
    assert(fs.position_count == 0 && "position_count = 0");
    assert(approx_eq(fs.defensive_depth, 0.5f) && "defensive_depth = 0.5");
    assert(approx_eq(fs.pressing_intensity, 0.5f) && "pressing_intensity = 0.5");
    assert(approx_eq(fs.width, 0.5f) && "width = 0.5");

    // Name should be empty
    assert(fs.name[0] == '\0' && "name = \"\"");

    std::printf("    [PASS] FormationSet defaults: count=0, depth=0.5, width=0.5\n");
    return 0;
}

// =============================================================================
// TEST 15: FormationSystem defaults
// =============================================================================
static int test_formation_system_defaults() {
    std::printf("  [Test 15] FormationSystem defaults...\n");

    apc::FormationSystem sys;

    assert(approx_eq(sys.formation_transition_speed, 2.0f) && "transition_speed = 2.0");
    assert(approx_eq(sys.ball_influence_radius, 15.0f) && "ball_influence_radius = 15.0");
    assert(approx_eq(sys.ball_influence_strength, 0.3f) && "ball_influence_strength = 0.3");
    assert(sys.cached_initialized == 0 && "cached_initialized = 0");

    // Sub-formation defaults
    assert(sys.attack_formation.position_count == 0 && "attack position_count = 0");
    assert(sys.defense_formation.position_count == 0 && "defense position_count = 0");

    std::printf("    [PASS] FormationSystem defaults verified\n");
    return 0;
}

// =============================================================================
// TEST 16: FormationSystem preset_4_4_2 produces valid FormationSet (11 positions)
// =============================================================================
static int test_preset_4_4_2() {
    std::printf("  [Test 16] FormationSystem preset_4_4_2 (11 positions)...\n");

    apc::FormationSet fs = apc::FormationSystem::preset_4_4_2();

    assert(fs.type == apc::FormationType::FORMATION_4_4_2 && "type = FORMATION_4_4_2");
    assert(fs.position_count == 11 && "position_count = 11");

    // Verify GK position (first)
    assert(approx_eq(fs.positions[0].base_position.x, -0.9f) && "GK x = -0.9");
    assert(approx_eq(fs.positions[0].base_position.z, 0.0f) && "GK z = 0");
    assert(fs.positions[0].compatible_role == apc::SportRole::SOCCER_GK && "GK role");

    // Verify a striker position (last two)
    assert(approx_eq(fs.positions[9].base_position.x, 0.5f) && "ST1 x = 0.5");
    assert(fs.positions[9].compatible_role == apc::SportRole::SOCCER_ST && "ST1 role");
    assert(approx_eq(fs.positions[10].base_position.x, 0.5f) && "ST2 x = 0.5");

    // Name
    assert(fs.name[0] == '4' && "name starts with '4'");

    // All positions should have Y = 0 (ground level)
    for (uint8_t i = 0; i < fs.position_count; ++i) {
        assert(approx_eq(fs.positions[i].base_position.y, 0.0f) && "all Y = 0");
    }

    std::printf("    [PASS] preset_4_4_2: 11 positions, GK at -0.9, ST at 0.5\n");
    return 0;
}

// =============================================================================
// TEST 17: FormationSystem preset_4_3_3 produces valid FormationSet
// =============================================================================
static int test_preset_4_3_3() {
    std::printf("  [Test 17] FormationSystem preset_4_3_3...\n");

    apc::FormationSet fs = apc::FormationSystem::preset_4_3_3();

    assert(fs.type == apc::FormationType::FORMATION_4_3_3 && "type = FORMATION_4_3_3");
    assert(fs.position_count == 11 && "position_count = 11");

    // GK at -0.9
    assert(approx_eq(fs.positions[0].base_position.x, -0.9f) && "GK x = -0.9");
    assert(fs.positions[0].compatible_role == apc::SportRole::SOCCER_GK && "GK role");

    // 3 midfielders (indices 5,6,7)
    assert(fs.positions[5].compatible_role == apc::SportRole::SOCCER_CDM && "CM1 = CDM");
    assert(fs.positions[6].compatible_role == apc::SportRole::SOCCER_CM && "CM2 = CM");

    // 3 forwards (indices 8,9,10) — LW, ST, RW
    assert(fs.positions[8].compatible_role == apc::SportRole::SOCCER_LW && "LW role");
    assert(fs.positions[9].compatible_role == apc::SportRole::SOCCER_ST && "ST role");
    assert(fs.positions[10].compatible_role == apc::SportRole::SOCCER_RW && "RW role");

    std::printf("    [PASS] preset_4_3_3: 11 positions, correct role layout\n");
    return 0;
}

// =============================================================================
// TEST 18: FormationSystem preset_3_5_2 produces valid FormationSet
// =============================================================================
static int test_preset_3_5_2() {
    std::printf("  [Test 18] FormationSystem preset_3_5_2...\n");

    apc::FormationSet fs = apc::FormationSystem::preset_3_5_2();

    assert(fs.type == apc::FormationType::FORMATION_3_5_2 && "type = FORMATION_3_5_2");
    assert(fs.position_count == 11 && "position_count = 11");

    // 3 CBs (indices 1,2,3)
    assert(fs.positions[1].compatible_role == apc::SportRole::SOCCER_CB && "CB1 role");
    assert(fs.positions[2].compatible_role == apc::SportRole::SOCCER_CB && "CB2 role");
    assert(fs.positions[3].compatible_role == apc::SportRole::SOCCER_CB && "CB3 role");

    // 5 midfielders (indices 4,5,6,7,8)
    assert(fs.positions[4].compatible_role == apc::SportRole::SOCCER_LB && "LWB role");
    assert(fs.positions[6].compatible_role == apc::SportRole::SOCCER_CDM && "CDM role");
    assert(fs.positions[8].compatible_role == apc::SportRole::SOCCER_RB && "RWB role");

    // 2 strikers (indices 9,10)
    assert(fs.positions[9].compatible_role == apc::SportRole::SOCCER_ST && "ST1 role");
    assert(fs.positions[10].compatible_role == apc::SportRole::SOCCER_ST && "ST2 role");

    std::printf("    [PASS] preset_3_5_2: 11 positions, 3CB + 5MF + 2ST\n");
    return 0;
}

// =============================================================================
// TEST 19: FormationSystem preset_4_2_3_1 produces valid FormationSet
// =============================================================================
static int test_preset_4_2_3_1() {
    std::printf("  [Test 19] FormationSystem preset_4_2_3_1...\n");

    apc::FormationSet fs = apc::FormationSystem::preset_4_2_3_1();

    assert(fs.type == apc::FormationType::FORMATION_4_2_3_1 && "type = FORMATION_4_2_3_1");
    assert(fs.position_count == 11 && "position_count = 11");

    // 2 CDMs (indices 5,6)
    assert(fs.positions[5].compatible_role == apc::SportRole::SOCCER_CDM && "CDM1 role");
    assert(fs.positions[6].compatible_role == apc::SportRole::SOCCER_CDM && "CDM2 role");

    // 3 attacking mids (indices 7,8,9)
    assert(fs.positions[7].compatible_role == apc::SportRole::SOCCER_LW && "LW role");
    assert(fs.positions[8].compatible_role == apc::SportRole::SOCCER_CAM && "CAM role");
    assert(fs.positions[9].compatible_role == apc::SportRole::SOCCER_RW && "RW role");

    // 1 ST (index 10)
    assert(fs.positions[10].compatible_role == apc::SportRole::SOCCER_ST && "ST role");
    assert(approx_eq(fs.positions[10].base_position.x, 0.45f) && "ST x = 0.45");

    std::printf("    [PASS] preset_4_2_3_1: 11 positions, 4+2+3+1 layout\n");
    return 0;
}

// =============================================================================
// TEST 20: FormationSystem.get_formation_position returns valid Vec3
// =============================================================================
static int test_get_formation_position() {
    std::printf("  [Test 20] FormationSystem.get_formation_position returns valid Vec3...\n");

    apc::FormationSystem sys;
    sys.set_formation(apc::FormationType::FORMATION_4_4_2,
                      apc::FormationType::FORMATION_4_4_2);

    apc::Vec3 ball(0.0f, 0.0f, 0.0f);
    apc::Vec3 team_goal(-1.0f, 0.0f, 0.0f);
    apc::Vec3 opp_goal(1.0f, 0.0f, 0.0f);

    // Get GK position (index 0) — should be near -0.9 on X
    apc::Vec3 gk_pos = sys.get_formation_position(0, ball, team_goal, opp_goal, 0.5f);
    assert(gk_pos.x < -0.5f && "GK position: x < -0.5");
    assert(approx_eq(gk_pos.y, 0.0f) && "GK position: Y = 0");

    // Get ST position (index 9 or 10) — should be near +0.5 on X
    apc::Vec3 st_pos = sys.get_formation_position(9, ball, team_goal, opp_goal, 0.5f);
    assert(st_pos.x > 0.2f && "ST position: x > 0.2");
    assert(approx_eq(st_pos.y, 0.0f) && "ST position: Y = 0");

    // Invalid index should return zero
    apc::Vec3 invalid_pos = sys.get_formation_position(20, ball, team_goal, opp_goal, 0.5f);
    assert(approx_eq(invalid_pos.x, 0.0f) && "invalid index: x = 0");
    assert(approx_eq(invalid_pos.z, 0.0f) && "invalid index: z = 0");

    std::printf("    [PASS] get_formation_position returns valid Vec3 positions\n");
    return 0;
}

// =============================================================================
// TEST 21: FormationSystem ball influence pulls nearby positions
// =============================================================================
static int test_ball_influence() {
    std::printf("  [Test 21] FormationSystem ball influence pulls nearby positions...\n");

    apc::FormationSystem sys;
    sys.ball_influence_radius = 15.0f;
    sys.ball_influence_strength = 0.3f;

    // Base position at (0, 0, 0)
    apc::Vec3 base(0.0f, 0.0f, 0.0f);

    // Ball very close at (2, 0, 0) — within radius
    apc::Vec3 ball_near(2.0f, 0.0f, 0.0f);
    apc::Vec3 influenced = sys.get_ball_influenced_position(base, ball_near, 0.5f);

    // Position should be pulled toward ball (X should increase)
    assert(influenced.x > base.x && "ball influence: pulled toward ball (+X)");
    assert(approx_eq(influenced.y, 0.0f) && "ball influence: Y = 0");

    // Ball far away at (100, 0, 0) — outside radius
    apc::Vec3 ball_far(100.0f, 0.0f, 0.0f);
    apc::Vec3 not_influenced = sys.get_ball_influenced_position(base, ball_far, 0.5f);

    // Should return base position unchanged
    assert(approx_eq(not_influenced.x, base.x) && "ball far: no influence on X");
    assert(approx_eq(not_influenced.z, base.z) && "ball far: no influence on Z");

    std::printf("    [PASS] ball influence pulls nearby, ignores far positions\n");
    return 0;
}

// =============================================================================
// TEST 22: FormationSystem possession factor interpolates defense/attack
// =============================================================================
static int test_possession_interpolation() {
    std::printf("  [Test 22] FormationSystem possession factor interpolates defense/attack...\n");

    apc::FormationSystem sys;

    // Use different attack/defense formations
    apc::FormationSet def_form = apc::FormationSystem::preset_4_4_2();
    apc::FormationSet atk_form = apc::FormationSystem::preset_4_3_3();

    sys.attack_formation = atk_form;
    sys.defense_formation = def_form;

    apc::Vec3 ball(0.0f, 0.0f, 0.0f);
    apc::Vec3 team_goal(-1.0f, 0.0f, 0.0f);
    apc::Vec3 opp_goal(1.0f, 0.0f, 0.0f);

    // Get striker (index 9) position at different possession factors
    // 4-4-2 ST at x=0.5, 4-3-3 ST at x=0.5 — same in this case
    // Let's use index 5 which differs: 4-4-2 CM at (-0.1, 0, -0.2), 4-3-3 CM at (-0.1, 0, -0.25)
    apc::Vec3 pos_def = sys.get_formation_position(5, ball, team_goal, opp_goal, 0.0f);
    apc::Vec3 pos_atk = sys.get_formation_position(5, ball, team_goal, opp_goal, 1.0f);

    // Verify the positions are valid Vec3s with Y=0
    assert(approx_eq(pos_def.y, 0.0f) && "defense pos: Y = 0");
    assert(approx_eq(pos_atk.y, 0.0f) && "attack pos: Y = 0");

    // Verify clamping: possession_factor = -0.5 should be clamped to 0.0
    apc::Vec3 pos_clamped_low = sys.get_formation_position(5, ball, team_goal, opp_goal, -0.5f);
    assert(approx_eq(pos_clamped_low.x, pos_def.x, 0.001f) && "clamped low = defense");

    // possession_factor = 2.0 should be clamped to 1.0
    apc::Vec3 pos_clamped_high = sys.get_formation_position(5, ball, team_goal, opp_goal, 2.0f);
    assert(approx_eq(pos_clamped_high.x, pos_atk.x, 0.001f) && "clamped high = attack");

    // Mid-possession should interpolate
    apc::Vec3 pos_mid = sys.get_formation_position(5, ball, team_goal, opp_goal, 0.5f);
    float mid_x = 0.5f * (pos_def.x + pos_atk.x);
    assert(approx_eq(pos_mid.x, mid_x, 0.01f) && "mid possession: interpolated X");

    std::printf("    [PASS] possession factor interpolates defense/attack positions\n");
    return 0;
}

// =============================================================================
// TEST 23: Consideration.evaluate response curves
// =============================================================================
static int test_consideration_response_curves() {
    std::printf("  [Test 23] Consideration.evaluate response curves...\n");

    // LINEAR: f(x) = x
    apc::Consideration linear;
    linear.curve = apc::ResponseCurve::LINEAR;
    linear.min_input = 0.0f;
    linear.max_input = 1.0f;

    assert(approx_eq(linear.evaluate(0.0f), 0.0f) && "LINEAR(0) = 0");
    assert(approx_eq(linear.evaluate(0.5f), 0.5f) && "LINEAR(0.5) = 0.5");
    assert(approx_eq(linear.evaluate(1.0f), 1.0f) && "LINEAR(1.0) = 1.0");

    // QUADRATIC: exponent = 0.5 (square root — favors extremes)
    apc::Consideration quad;
    quad.curve = apc::ResponseCurve::QUADRATIC;
    quad.min_input = 0.0f;
    quad.max_input = 1.0f;

    assert(approx_eq(quad.evaluate(0.0f), 0.0f) && "QUADRATIC(0) = 0");
    assert(approx_eq(quad.evaluate(0.25f), 0.5f, 0.01f) && "QUADRATIC(0.25) ≈ 0.5");
    assert(approx_eq(quad.evaluate(1.0f), 1.0f) && "QUADRATIC(1.0) = 1.0");

    // EXPONENTIAL: exponent = 2.0 (squares — favors mid-range)
    apc::Consideration expo;
    expo.curve = apc::ResponseCurve::EXPONENTIAL;
    expo.min_input = 0.0f;
    expo.max_input = 1.0f;

    assert(approx_eq(expo.evaluate(0.0f), 0.0f) && "EXPONENTIAL(0) = 0");
    assert(approx_eq(expo.evaluate(0.5f), 0.25f) && "EXPONENTIAL(0.5) = 0.25");
    assert(approx_eq(expo.evaluate(1.0f), 1.0f) && "EXPONENTIAL(1.0) = 1.0");

    // Clamping: input outside range
    assert(approx_eq(linear.evaluate(-0.5f), 0.0f) && "clamp: below min → 0");
    assert(approx_eq(linear.evaluate(1.5f), 1.0f) && "clamp: above max → 1");

    std::printf("    [PASS] response curves: LINEAR, QUADRATIC, EXPONENTIAL\n");
    return 0;
}

// =============================================================================
// TEST 24: FormationSet.add_position and reset
// =============================================================================
static int test_formation_set_add_reset() {
    std::printf("  [Test 24] FormationSet.add_position and reset...\n");

    apc::FormationSet fs;

    assert(fs.position_count == 0);

    apc::FormationPosition pos;
    pos.base_position = apc::Vec3(0.5f, 0.0f, 0.0f);
    pos.compatible_role = apc::SportRole::SOCCER_ST;

    uint8_t r1 = fs.add_position(pos);
    assert(r1 == 1 && "add_position returns 1 on success");
    assert(fs.position_count == 1 && "count = 1");

    uint8_t r2 = fs.add_position(pos);
    assert(r2 == 1 && "second add returns 1");
    assert(fs.position_count == 2 && "count = 2");

    assert(approx_eq(fs.positions[0].base_position.x, 0.5f) && "stored correctly");

    // Reset
    fs.reset();
    assert(fs.position_count == 0 && "reset: count = 0");
    assert(approx_eq(fs.defensive_depth, 0.5f) && "reset: defensive_depth = 0.5");
    assert(fs.name[0] == '\0' && "reset: name cleared");

    std::printf("    [PASS] add_position and reset work correctly\n");
    return 0;
}

// =============================================================================
// TEST 25: UtilityAI.reset clears all state
// =============================================================================
static int test_utility_ai_reset() {
    std::printf("  [Test 25] UtilityAI.reset clears all state...\n");

    apc::UtilityAI ai;

    // Add some state
    ai.add_consideration("test", 1.0f, apc::ResponseCurve::LINEAR, 0.0f, 1.0f);
    ai.actions[0] = apc::AIActionType::SHOOT_BALL;
    ai.action_count = 1u;
    ai.set_action_weight(apc::AIActionType::SHOOT_BALL, 5.0f);

    assert(ai.consideration_count > 0);
    assert(ai.action_count > 0);
    assert(ai.role_weight_count > 0);

    // Reset
    ai.reset();

    assert(ai.consideration_count == 0u && "reset: consideration_count = 0");
    assert(ai.action_count == 0u && "reset: action_count = 0");
    assert(ai.role_weight_count == 0u && "reset: role_weight_count = 0");
    assert(ai.context_factor_count == 0u && "reset: context_factor_count = 0");

    std::printf("    [PASS] UtilityAI.reset clears all state\n");
    return 0;
}

// =============================================================================
// Main
// =============================================================================
int main() {
    std::printf("=== Sprint 24 Tests ===\n");
    int fails = 0;

    fails += test_action_type_enum();
    fails += test_response_curve_enum();
    fails += test_utility_score_defaults();
    fails += test_consideration_defaults();
    fails += test_utility_ai_defaults();
    fails += test_add_consideration();
    fails += test_evaluate_returns_score();
    fails += test_evaluate_picks_best();
    fails += test_set_action_weight();
    fails += test_configure_role();
    fails += test_context_factor_enum();
    fails += test_formation_type_enum();
    fails += test_formation_position_defaults();
    fails += test_formation_set_defaults();
    fails += test_formation_system_defaults();
    fails += test_preset_4_4_2();
    fails += test_preset_4_3_3();
    fails += test_preset_3_5_2();
    fails += test_preset_4_2_3_1();
    fails += test_get_formation_position();
    fails += test_ball_influence();
    fails += test_possession_interpolation();
    fails += test_consideration_response_curves();
    fails += test_formation_set_add_reset();
    fails += test_utility_ai_reset();

    int passed = 25 - fails;
    std::printf("=== Sprint 24: %d tests passed, %d failed ===\n", passed, fails);
    return fails;
}
