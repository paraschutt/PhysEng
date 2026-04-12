// =============================================================================
// Sprint 28 Tests — AI Hysteresis & Commitment Locking (Phase 14 Action 3)
// =============================================================================
//
// Validates the UtilityAI hysteresis system that prevents decision oscillation
// when two actions have nearly identical scores ("stutter-step" vibration).
//
// Tests:
//   1.  HYSTERESIS_BONUS constant = 0.15f
//   2.  AthleteEntity::active_action_id defaults to 0xFFFFFFFF
//   3.  AthleteEntity::reaction_frames defaults to 12
//   4.  evaluate_with_hysteresis() returns IDLE with score=-1 on empty actions
//   5.  evaluate_with_hysteresis() locks first chosen action into entity
//   6.  evaluate_with_hysteresis() applies 15% bonus to incumbent action
//   7.  Hysteresis prevents flip-flop when two actions have close scores
//   8.  evaluate_with_hysteresis() clamps hysteresis score at 1.15f
//   9.  evaluate_with_hysteresis() resets active_action_id when no actions
//  10.  Confidence is clamped to [0, 1]
//  11.  PerceptionRingBuffer push + get_delayed_state round-trip
//  12.  PerceptionRingBuffer get_delayed_state clamps to oldest available
//  13.  PerceptionRingBuffer empty buffer returns default snapshot
//  14.  evaluate() (without hysteresis) does NOT modify entity state
//
// Pattern: int main() + assert(), no test framework.
//
// IMPORTANT: get_action_score() returns 0 when consideration_count == 0,
// so all scoring tests must add at least one consideration.
// =============================================================================

#include "apc_ai/apc_ai_decision.h"
#include "apc_ai/apc_ai_perception.h"
#include "apc_entity/apc_entity_types.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_math_common.h"
#include <cassert>
#include <cstdlib>
#include <cstdio>
#include <cmath>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static constexpr float EPS = 1e-4f;
static int g_tests = 0;
static int g_passed = 0;

static bool approx_eq(float a, float b, float eps = EPS) {
    return std::abs(a - b) < eps;
}

static void check(bool cond, const char* msg, int line) {
    ++g_tests;
    if (cond) {
        ++g_passed;
    } else {
        std::printf("    [FAIL] %s (line %d, test #%d)\n", msg, line, g_tests);
        std::fflush(stdout);
        std::abort();
    }
}

#define CHECK(cond, msg) check((cond), (msg), __LINE__)

// ---------------------------------------------------------------------------
// Helper: create a UtilityAI with one consideration so scoring works.
// Without at least one consideration, get_action_score() returns 0.
// ---------------------------------------------------------------------------
static apc::UtilityAI make_ai_with_consideration() {
    apc::UtilityAI ai;
    // Add a neutral consideration: weight=0, so it doesn't alter scores,
    // but its presence prevents the early return in get_action_score().
    ai.add_consideration("neutral", 0.0f,
                         apc::ResponseCurve::LINEAR, 0.0f, 100.0f);
    return ai;
}

// =============================================================================
// TEST 1: HYSTERESIS_BONUS constant = 0.15f
// =============================================================================
static void test_hysteresis_constant() {
    std::printf("  [Test 1] HYSTERESIS_BONUS = 0.15f...\n");

    CHECK(apc::UtilityAI::HYSTERESIS_BONUS > 0.14f &&
          apc::UtilityAI::HYSTERESIS_BONUS < 0.16f,
          "HYSTERESIS_BONUS is approximately 0.15f");

    std::printf("    [PASS] HYSTERESIS_BONUS = 0.15f verified\n");
}

// =============================================================================
// TEST 2: AthleteEntity::active_action_id defaults to 0xFFFFFFFF
// =============================================================================
static void test_entity_active_action_default() {
    std::printf("  [Test 2] active_action_id defaults to 0xFFFFFFFF...\n");

    apc::AthleteEntity e;
    CHECK(e.active_action_id == 0xFFFFFFFFu,
          "active_action_id = 0xFFFFFFFF on default construction");

    // Verify reset() also restores it
    e.active_action_id = 5u;
    e.reset();
    CHECK(e.active_action_id == 0xFFFFFFFFu,
          "active_action_id = 0xFFFFFFFF after reset()");

    std::printf("    [PASS] active_action_id defaults verified\n");
}

// =============================================================================
// TEST 3: AthleteEntity::reaction_frames defaults to 12
// =============================================================================
static void test_entity_reaction_frames_default() {
    std::printf("  [Test 3] reaction_frames defaults to 12...\n");

    apc::AthleteEntity e;
    CHECK(e.reaction_frames == 12u,
          "reaction_frames = 12 on default construction");

    // Verify reset() also restores it
    e.reaction_frames = 6u;
    e.reset();
    CHECK(e.reaction_frames == 12u,
          "reaction_frames = 12 after reset()");

    std::printf("    [PASS] reaction_frames defaults verified\n");
}

// =============================================================================
// TEST 4: evaluate_with_hysteresis() with no actions returns IDLE
// =============================================================================
static void test_hysteresis_empty_actions() {
    std::printf("  [Test 4] evaluate_with_hysteresis() with no actions...\n");

    apc::UtilityAI ai;
    apc::AthleteEntity entity;

    float inputs[8] = { 50.0f, 60.0f, 30.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.5f };

    apc::UtilityScore result = ai.evaluate_with_hysteresis(inputs, 8u, entity);

    // With no actions, score should remain -1 (no best found), action stays IDLE
    CHECK(result.action == apc::AIActionType::IDLE,
          "no actions: result.action = IDLE");
    CHECK(result.score < 0.0f,
          "no actions: score remains -1 (no valid action)");

    std::printf("    [PASS] Empty actions handled correctly\n");
}

// =============================================================================
// TEST 5: evaluate_with_hysteresis() locks first chosen action into entity
// =============================================================================
static void test_hysteresis_locks_choice() {
    std::printf("  [Test 5] evaluate_with_hysteresis() locks first choice...\n");

    apc::UtilityAI ai = make_ai_with_consideration();
    // Only one action available: CHASE_BALL
    ai.add_action(apc::AIActionType::CHASE_BALL);

    apc::AthleteEntity entity;
    CHECK(entity.active_action_id == 0xFFFFFFFFu, "pre: no prior action");

    float inputs[8] = { 5.0f, 60.0f, 30.0f, 1.0f, 0.5f, 1.0f, 0.0f, 0.5f };

    apc::UtilityScore result = ai.evaluate_with_hysteresis(inputs, 8u, entity);

    // Entity's active_action_id must now be set to CHASE_BALL (value 2)
    CHECK(entity.active_action_id ==
          static_cast<uint32_t>(apc::AIActionType::CHASE_BALL),
          "entity.active_action_id locked to CHASE_BALL");
    CHECK(result.action == apc::AIActionType::CHASE_BALL,
          "result.action = CHASE_BALL (only option)");
    CHECK(result.score >= 0.0f,
          "score >= 0 for CHASE_BALL at close range");

    std::printf("    [PASS] First choice locked into entity\n");
}

// =============================================================================
// TEST 6: evaluate_with_hysteresis() applies 15% bonus to incumbent action
// =============================================================================
static void test_hysteresis_applies_bonus() {
    std::printf("  [Test 6] Hysteresis applies 15%% bonus to incumbent...\n");

    // Strategy: create a fresh entity, evaluate to establish CHASE_BALL as
    // incumbent. Then create a SECOND entity whose incumbent is SUPPORT_RUN.
    // When both face the same inputs (which favor CHASE_BALL), the first
    // entity should pick CHASE_BALL normally, but the second entity's
    // SUPPORT_RUN should get the +0.15 bonus — enough to flip the winner.
    //
    // Scoring (with 1 neutral weight-0 consideration so blend = raw * 0.7 + 0 * 0.3 = raw * 0.7):
    //   CHASE_BALL:    (1 - dist/50) * (0.5+0.5*stam) * (0.4+0.6*poss)
    //                  → result * 0.7
    //   SUPPORT_RUN:   (0.3+0.7*poss) * (dist<25?0.8:0.4) * stam
    //                  → result * 0.7
    //   FORMATION_HOLD: 0.5 * min(dist/25, 1) → result * 0.7

    apc::UtilityAI ai = make_ai_with_consideration();
    ai.add_action(apc::AIActionType::CHASE_BALL);
    ai.add_action(apc::AIActionType::SUPPORT_RUN);
    ai.add_action(apc::AIActionType::FORMATION_HOLD);

    // Inputs: dist=5, stamina=0.9, poss=0.8
    // CHASE:    (1 - 5/50) * (0.5+0.45) * (0.4+0.48) = 0.9 * 0.95 * 0.88 = 0.7524 → *0.7 = 0.527
    // SUPPORT:  (0.3+0.56) * 0.8 * 0.9 = 0.86 * 0.72 = 0.6192 → *0.7 = 0.433
    // HOLD:     0.5 * (5/25) = 0.1 → *0.7 = 0.07
    // CHASE wins without hysteresis. Not useful for testing the bonus.

    // Inputs: dist=20, stamina=0.5, poss=0.3
    // CHASE:    (1 - 20/50) * (0.5+0.25) * (0.4+0.18) = 0.6 * 0.75 * 0.58 = 0.261 → *0.7 = 0.183
    // SUPPORT:  (0.3+0.21) * 0.8 * 0.5 = 0.51 * 0.4 = 0.204 → *0.7 = 0.143
    // HOLD:     0.5 * (20/25) = 0.4 → *0.7 = 0.28
    // HOLD wins! Not useful either.

    // We need CHASE and another action to be within 0.15 of each other.
    // Let's try: dist=12, stamina=0.7, poss=0.5
    // CHASE:    (1 - 12/50) * (0.5+0.35) * (0.4+0.3) = 0.76 * 0.85 * 0.7 = 0.452 → *0.7 = 0.317
    // SUPPORT:  (0.3+0.35) * 0.8 * 0.7 = 0.65 * 0.56 = 0.364 → *0.7 = 0.255
    // HOLD:     0.5 * (12/25) = 0.24 → *0.7 = 0.168
    // CHASE wins by 0.062. If SUPPORT had +0.15, it would be 0.405 > 0.317.
    // This is a valid test case!

    float inputs[8] = { 12.0f, 45.0f, 25.0f, 0.7f, 0.5f, 1.0f, 0.0f, 0.5f };

    // Test A: entity with no prior action → CHASE_BALL should win
    apc::AthleteEntity e_clean;
    e_clean.active_action_id = 0xFFFFFFFFu;
    apc::UtilityScore r_clean = ai.evaluate_with_hysteresis(inputs, 8u, e_clean);
    CHECK(r_clean.action == apc::AIActionType::CHASE_BALL,
          "no incumbent: CHASE_BALL wins on raw score");

    // Test B: entity whose incumbent is SUPPORT_RUN → bonus should flip winner
    apc::AthleteEntity e_support;
    e_support.active_action_id = static_cast<uint32_t>(apc::AIActionType::SUPPORT_RUN);
    apc::UtilityScore r_support = ai.evaluate_with_hysteresis(inputs, 8u, e_support);
    CHECK(r_support.action == apc::AIActionType::SUPPORT_RUN,
          "SUPPORT_RUN with hysteresis bonus beats higher-raw CHASE_BALL");
    CHECK(e_support.active_action_id ==
          static_cast<uint32_t>(apc::AIActionType::SUPPORT_RUN),
          "entity re-locked to SUPPORT_RUN");

    // Test C: verify the lock persists on second call
    apc::UtilityScore r_support2 = ai.evaluate_with_hysteresis(inputs, 8u, e_support);
    CHECK(r_support2.action == apc::AIActionType::SUPPORT_RUN,
          "SUPPORT_RUN still wins on second call (persistent lock)");

    std::printf("    [PASS] Hysteresis bonus correctly applied to incumbent action\n");
}

// =============================================================================
// TEST 7: Hysteresis prevents flip-flop on close scores
// =============================================================================
static void test_hysteresis_prevents_flip_flop() {
    std::printf("  [Test 7] Hysteresis prevents oscillation on close scores...\n");

    apc::UtilityAI ai = make_ai_with_consideration();
    ai.add_action(apc::AIActionType::CHASE_BALL);
    ai.add_action(apc::AIActionType::SUPPORT_RUN);
    ai.add_action(apc::AIActionType::MOVE_TO_POSITION);

    apc::AthleteEntity entity;

    // Use a base set of inputs. Evaluate once to lock in a choice.
    float inputs[8] = { 8.0f, 45.0f, 25.0f, 0.85f, 0.6f, 1.0f, 0.0f, 0.5f };

    apc::UtilityScore r1 = ai.evaluate_with_hysteresis(inputs, 8u, entity);
    uint32_t first_choice = static_cast<uint32_t>(r1.action);

    // Now run 10 more evaluations with slightly perturbed inputs.
    // Without hysteresis, the choice might flip. With hysteresis, it should
    // remain stable because the incumbent gets +0.15 each frame.
    uint32_t flip_count = 0u;
    float perturbation[8];
    for (int frame = 0; frame < 10; ++frame) {
        for (uint32_t k = 0u; k < 8u; ++k) {
            perturbation[k] = inputs[k];
        }
        // Small noise in dist_to_ball and dist_opp
        perturbation[0] += (frame % 2 == 0) ? 0.5f : -0.5f;
        perturbation[2] += (frame % 3 == 0) ? 0.3f : -0.3f;

        apc::UtilityScore r = ai.evaluate_with_hysteresis(
            perturbation, 8u, entity);
        if (static_cast<uint32_t>(r.action) != first_choice) {
            ++flip_count;
        }
    }

    // With hysteresis, the action should NOT flip under small perturbations.
    CHECK(flip_count <= 1u,
          "hysteresis prevents oscillation (flips <= 1 over 10 frames)");

    std::printf("    [PASS] Hysteresis stabilizes decisions under noise (%u flips)\n",
                flip_count);
}

// =============================================================================
// TEST 8: evaluate_with_hysteresis() clamps hysteresis score at 1.15f
// =============================================================================
static void test_hysteresis_score_clamp() {
    std::printf("  [Test 8] Hysteresis score clamped at 1.15f...\n");

    apc::UtilityAI ai = make_ai_with_consideration();
    ai.add_action(apc::AIActionType::CHASE_BALL);

    apc::AthleteEntity entity;
    entity.active_action_id = static_cast<uint32_t>(apc::AIActionType::CHASE_BALL);

    // dist=0.1, stamina=1.0, possession=1.0 → CHASE_BALL raw ≈ 0.998
    // * 0.7 (consideration blend) = 0.699. With +0.15: 0.849.
    // Hmm, the consideration blend reduces the score significantly.
    // The clamp to 1.15 is still valid code even if we can't reach it easily.
    // Let's just verify the code path works — the clamp exists at line 538:
    //   if (action_score > 1.15f) action_score = 1.15f;
    // We need a very high raw score. The only way to exceed 1.0 is with
    // role weights. Let's configure a GK role where DIVE_SAVE has weight 5.
    ai.configure_role(apc::SportRole::SOCCER_GK);
    ai.clear_actions();
    ai.add_action(apc::AIActionType::DIVE_SAVE);

    entity.active_action_id = static_cast<uint32_t>(apc::AIActionType::DIVE_SAVE);

    // DIVE_SAVE: (dist<10 ? (1-dist/10) : 0) * (dist_goal>35 ? 1 : 0.1) * (0.8+0.2*stam)
    // dist=0.1, stamina=1.0, dist_goal=50:
    //   (1-0.01) * 1.0 * 1.0 = 0.99 → * 5.0 (role weight) = 4.95 → * 0.7 = 3.465
    // With hysteresis: 3.465 + 0.15 = 3.615, clamped to 1.15.
    float inputs[8] = { 0.1f, 50.0f, 30.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.5f };

    apc::UtilityScore result = ai.evaluate_with_hysteresis(inputs, 8u, entity);

    CHECK(result.score <= 1.15f + EPS,
          "hysteresis score clamped at or below 1.15");
    CHECK(result.score > 1.0f,
          "hysteresis score is above 1.0 (bonus applied)");

    std::printf("    [PASS] Hysteresis score correctly clamped at 1.15\n");
}

// =============================================================================
// TEST 9: evaluate_with_hysteresis() resets active_action_id when no actions
// =============================================================================
static void test_hysteresis_reset_on_no_actions() {
    std::printf("  [Test 9] active_action_id reset when no valid actions...\n");

    apc::UtilityAI ai;
    // No actions added — action_count = 0

    apc::AthleteEntity entity;
    entity.active_action_id = static_cast<uint32_t>(apc::AIActionType::CHASE_BALL);

    float inputs[8] = { 5.0f, 40.0f, 30.0f, 1.0f, 0.5f, 1.0f, 0.0f, 0.5f };

    apc::UtilityScore result = ai.evaluate_with_hysteresis(inputs, 8u, entity);

    // With no actions, best_action stays nullptr.
    // The code does: entity.active_action_id = static_cast<uint32_t>(best.action)
    // best.action defaults to AIActionType::IDLE (value 0).
    CHECK(entity.active_action_id == static_cast<uint32_t>(apc::AIActionType::IDLE),
          "active_action_id reset to IDLE (0) when no valid actions");

    std::printf("    [PASS] active_action_id reset on empty action set\n");
}

// =============================================================================
// TEST 10: Confidence is clamped to [0, 1]
// =============================================================================
static void test_confidence_clamped() {
    std::printf("  [Test 10] Confidence clamped to [0, 1]...\n");

    apc::UtilityAI ai = make_ai_with_consideration();
    ai.add_action(apc::AIActionType::CHASE_BALL);
    ai.add_action(apc::AIActionType::FORMATION_HOLD);

    apc::AthleteEntity entity;

    // Normal inputs — confidence should be in [0, 1]
    float inputs[8] = { 5.0f, 40.0f, 30.0f, 1.0f, 0.5f, 1.0f, 0.0f, 0.5f };
    apc::UtilityScore result = ai.evaluate_with_hysteresis(inputs, 8u, entity);

    CHECK(result.confidence >= 0.0f - EPS,
          "confidence >= 0");
    CHECK(result.confidence <= 1.0f + EPS,
          "confidence <= 1");

    // With hysteresis + role weights, score can exceed 1.0 but confidence
    // must still be clamped to 1.0.
    ai.configure_role(apc::SportRole::SOCCER_ST);
    entity.active_action_id = static_cast<uint32_t>(apc::AIActionType::CHASE_BALL);

    float inputs_high[8] = { 0.1f, 40.0f, 30.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.5f };
    apc::UtilityScore result2 = ai.evaluate_with_hysteresis(inputs_high, 8u, entity);

    CHECK(result2.confidence <= 1.0f + EPS,
          "hysteresis confidence clamped to 1.0 even when score > 1.0");

    std::printf("    [PASS] Confidence correctly clamped to [0, 1]\n");
}

// =============================================================================
// TEST 11: PerceptionRingBuffer push + get_delayed_state round-trip
// =============================================================================
static void test_perception_roundtrip() {
    std::printf("  [Test 11] PerceptionRingBuffer push + get_delayed_state...\n");

    apc::PerceptionRingBuffer buffer;

    apc::AthleteEntity athletes[3];
    athletes[0].id.index = 0;
    athletes[0].team = apc::TEAM_HOME;
    athletes[0].is_active = 1;
    athletes[0].position = { 10.0f, 0.0f, 5.0f };

    athletes[1].id.index = 1;
    athletes[1].team = apc::TEAM_AWAY;
    athletes[1].is_active = 1;
    athletes[1].position = { -10.0f, 0.0f, -5.0f };

    athletes[2].id.index = 2;
    athletes[2].team = apc::TEAM_HOME;
    athletes[2].is_active = 1;
    athletes[2].position = { 5.0f, 0.0f, 0.0f };

    for (uint32_t frame = 0u; frame < 20u; ++frame) {
        float x = static_cast<float>(frame) * 2.0f;
        buffer.push_state(
            apc::Vec3(x, 0.11f, 0.0f),
            apc::Vec3(5.0f, 0.0f, 0.0f),
            apc::TEAM_HOME,
            1u,
            athletes, 3u
        );
    }

    CHECK(buffer.get_count() == 20u, "buffer has 20 frames");

    // Latest frame: ball at x = 19 * 2 = 38
    const apc::PerceptionSnapshot& latest = buffer.get_latest_state();
    CHECK(approx_eq(latest.ball_position.x, 38.0f, 0.01f),
          "latest ball_position.x = 38.0");

    // 5-frame delay: ball at x = (20 - 1 - 5) * 2 = 28
    const apc::PerceptionSnapshot& delayed5 = buffer.get_delayed_state(5u);
    CHECK(approx_eq(delayed5.ball_position.x, 28.0f, 0.01f),
          "5-frame delayed ball_position.x = 28.0");

    // Athlete data preserved in delayed snapshot
    CHECK(delayed5.athlete_count == 3u, "delayed snapshot has 3 athletes");
    CHECK(delayed5.athletes[0].team == apc::TEAM_HOME,
          "delayed athlete[0] is TEAM_HOME");
    CHECK(delayed5.athletes[1].team == apc::TEAM_AWAY,
          "delayed athlete[1] is TEAM_AWAY");

    std::printf("    [PASS] PerceptionRingBuffer round-trip verified\n");
}

// =============================================================================
// TEST 12: PerceptionRingBuffer get_delayed_state clamps to oldest
// =============================================================================
static void test_perception_clamps_delay() {
    std::printf("  [Test 12] get_delayed_state clamps to oldest available...\n");

    apc::PerceptionRingBuffer buffer;

    apc::AthleteEntity athletes[1];
    athletes[0].id.index = 0;
    athletes[0].team = apc::TEAM_HOME;
    athletes[0].is_active = 1;
    athletes[0].position = { 0.0f, 0.0f, 0.0f };

    // Push only 5 frames
    for (uint32_t frame = 0u; frame < 5u; ++frame) {
        float x = static_cast<float>(frame) * 3.0f;
        buffer.push_state(
            apc::Vec3(x, 0.0f, 0.0f),
            apc::Vec3(0.0f, 0.0f, 0.0f),
            apc::TEAM_NONE,
            1u,
            athletes, 1u
        );
    }

    // Request delay of 100 frames — should clamp to oldest (frame 0, x=0)
    const apc::PerceptionSnapshot& clamped = buffer.get_delayed_state(100u);
    CHECK(approx_eq(clamped.ball_position.x, 0.0f, 0.01f),
          "100-frame delay clamped to oldest: ball at x=0");

    std::printf("    [PASS] Delay correctly clamped to oldest available frame\n");
}

// =============================================================================
// TEST 13: PerceptionRingBuffer empty buffer returns default
// =============================================================================
static void test_perception_empty_buffer() {
    std::printf("  [Test 13] Empty buffer returns default snapshot...\n");

    apc::PerceptionRingBuffer buffer;

    CHECK(buffer.get_count() == 0u, "empty buffer: count = 0");

    const apc::PerceptionSnapshot& snap = buffer.get_delayed_state(0u);
    CHECK(approx_eq(snap.ball_position.x, 0.0f),
          "empty buffer: ball_position.x = 0");
    CHECK(snap.athlete_count == 0u,
          "empty buffer: athlete_count = 0");
    CHECK(snap.ball_in_play == 1u,
          "empty buffer: ball_in_play defaults to 1");

    std::printf("    [PASS] Empty buffer returns valid default snapshot\n");
}

// =============================================================================
// TEST 14: evaluate() (without hysteresis) does NOT modify entity state
// =============================================================================
static void test_evaluate_no_entity_mutation() {
    std::printf("  [Test 14] evaluate() does NOT modify entity state...\n");

    apc::UtilityAI ai = make_ai_with_consideration();
    ai.add_action(apc::AIActionType::CHASE_BALL);

    apc::AthleteEntity entity;
    entity.active_action_id = static_cast<uint32_t>(apc::AIActionType::SUPPORT_RUN);

    float inputs[8] = { 5.0f, 40.0f, 30.0f, 1.0f, 0.5f, 1.0f, 0.0f, 0.5f };

    // evaluate() does not take entity parameter — it cannot modify entity.
    apc::UtilityScore result = ai.evaluate(inputs, 8u);

    CHECK(result.action == apc::AIActionType::CHASE_BALL,
          "evaluate() returns CHASE_BALL (only action)");
    CHECK(entity.active_action_id ==
          static_cast<uint32_t>(apc::AIActionType::SUPPORT_RUN),
          "evaluate() did NOT modify entity.active_action_id");

    std::printf("    [PASS] evaluate() is side-effect-free on entity\n");
}

// =============================================================================
// main
// =============================================================================
int main() {
    std::printf("=== Sprint 28: AI Hysteresis & Commitment Locking ===\n\n");

    test_hysteresis_constant();
    test_entity_active_action_default();
    test_entity_reaction_frames_default();
    test_hysteresis_empty_actions();
    test_hysteresis_locks_choice();
    test_hysteresis_applies_bonus();
    test_hysteresis_prevents_flip_flop();
    test_hysteresis_score_clamp();
    test_hysteresis_reset_on_no_actions();
    test_confidence_clamped();
    test_perception_roundtrip();
    test_perception_clamps_delay();
    test_perception_empty_buffer();
    test_evaluate_no_entity_mutation();

    int failed = g_tests - g_passed;
    std::printf("\n=== Sprint 28: %d tests passed, %d failed (%d total) ===\n",
                g_passed, failed, g_tests);
    return failed;
}
