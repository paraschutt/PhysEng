// =============================================================================
// test_sprint33.cpp — Phase 16 Action 1: Spatial Event Triggers (Scoring & OOB)
// =============================================================================
//
// Validates the evaluate_spatial_rules() and post_event_timer logic:
//
//   1. scoring_detected_home_goal       — Ball in +X SCORING_TARGET → home_score++
//   2. scoring_detected_away_goal       — Ball in -X SCORING_TARGET → away_score++
//   3. scoring_transitions_state        — SCORING_EVENT state set after goal
//   4. scoring_sets_post_event_timer    — post_event_timer = 3.0f after goal
//   5. oob_detected                     — Ball in OUT_OF_BOUNDS → DEAD_BALL
//   6. oob_sets_post_event_timer        — post_event_timer = 2.0f after OOB
//   7. timer_expiry_resets_to_pregame   — Timer runs out → PRE_GAME
//   8. timer_expiry_calls_reset         — reset_match_positions called on timeout
//   9. no_spatial_check_when_not_live   — evaluate_spatial_rules only called in LIVE
//  10. unload_resets_post_event_timer   — unload() zeroes post_event_timer
//  11. multiple_goals_accumulate        — Scores persist across multiple goals
//
// =============================================================================

#include "apc_app/apc_application.h"
#include "apc_app/apc_scene_manager.h"
#include "apc_app/apc_sport_config.h"
#include "apc_sport/apc_sport_rules.h"

#include <cstdio>
#include <cmath>

using namespace apc;

static int g_tests_passed = 0;
static int g_tests_failed = 0;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::printf("  FAIL: %s (line %d)\n", msg, __LINE__); \
            ++g_tests_failed; \
        } else { \
            ++g_tests_passed; \
        } \
    } while(0)

#define TEST_ASSERT_FLOAT_EQ(a, b, msg, eps) \
    do { \
        float _a = (a), _b = (b); \
        if (std::fabs(_a - _b) > (eps)) { \
            std::printf("  FAIL: %s — expected %.4f, got %.4f (line %d)\n", \
                        msg, _b, _a, __LINE__); \
            ++g_tests_failed; \
        } else { \
            ++g_tests_passed; \
        } \
    } while(0)

// =========================================================================
// Helper: Load a soccer match so the field + zones + entities are set up
// =========================================================================
static SceneState make_loaded_soccer_scene()
{
    SceneState s;
    MatchConfig mc = MatchConfig::soccer_sandbox();
    uint8_t result = s.load_match(mc);
    (void)result;
    s.max_period_duration = 300.0f;
    s.config.halves = 2;
    return s;
}

// =========================================================================
// Test 1: Ball in +X scoring zone → Home Team scores
// =========================================================================
static void test_scoring_detected_home_goal()
{
    std::printf("  [1] scoring_detected_home_goal\n");
    SceneState s = make_loaded_soccer_scene();
    s.rules.current_state = SemanticPlayState::LIVE_PLAY;

    uint32_t initial_home = s.home_score;
    TEST_ASSERT(initial_home == 0u, "Home score should start at 0");

    // Place ball in the +X goal zone (away goal)
    // Soccer SCORING_TARGET at +X: Vec3(half_l, 0, -gw) to Vec3(half_l + gd, 0, gw)
    // half_l = 52.5, gd = 2.0 → ball at x=53.0 is inside
    BallEntity* ball = s.entity_manager.find_ball();
    TEST_ASSERT(ball != nullptr, "Ball should exist after load_match");
    if (ball) {
        ball->position = Vec3(53.0f, 0.5f, 0.0f);

        s.evaluate_spatial_rules();

        TEST_ASSERT(s.home_score == initial_home + 1u,
                    "Home score should increment after +X goal");
        TEST_ASSERT(s.away_score == 0u,
                    "Away score should remain 0");
    }

    s.unload();
}

// =========================================================================
// Test 2: Ball in -X scoring zone → Away Team scores
// =========================================================================
static void test_scoring_detected_away_goal()
{
    std::printf("  [2] scoring_detected_away_goal\n");
    SceneState s = make_loaded_soccer_scene();
    s.rules.current_state = SemanticPlayState::LIVE_PLAY;

    // Place ball in the -X goal zone (home goal)
    // half_l = 52.5, gd = 2.0 → ball at x=-53.0 is inside
    BallEntity* ball = s.entity_manager.find_ball();
    if (ball) {
        ball->position = Vec3(-53.0f, 0.5f, 0.0f);

        s.evaluate_spatial_rules();

        TEST_ASSERT(s.home_score == 0u,
                    "Home score should remain 0");
        TEST_ASSERT(s.away_score == 1u,
                    "Away score should increment after -X goal");
    }

    s.unload();
}

// =========================================================================
// Test 3: Scoring sets SCORING_EVENT state
// =========================================================================
static void test_scoring_transitions_state()
{
    std::printf("  [3] scoring_transitions_state\n");
    SceneState s = make_loaded_soccer_scene();
    s.rules.current_state = SemanticPlayState::LIVE_PLAY;

    BallEntity* ball = s.entity_manager.find_ball();
    if (ball) {
        ball->position = Vec3(53.0f, 0.5f, 0.0f);

        s.evaluate_spatial_rules();

        TEST_ASSERT(s.rules.current_state == SemanticPlayState::SCORING_EVENT,
                    "State should transition to SCORING_EVENT after goal");
    }

    s.unload();
}

// =========================================================================
// Test 4: Scoring sets post_event_timer to 3.0s
// =========================================================================
static void test_scoring_sets_post_event_timer()
{
    std::printf("  [4] scoring_sets_post_event_timer\n");
    SceneState s = make_loaded_soccer_scene();
    s.rules.current_state = SemanticPlayState::LIVE_PLAY;
    s.post_event_timer = 0.0f; // Explicit zero

    BallEntity* ball = s.entity_manager.find_ball();
    if (ball) {
        ball->position = Vec3(53.0f, 0.5f, 0.0f);

        s.evaluate_spatial_rules();

        TEST_ASSERT_FLOAT_EQ(s.post_event_timer, 3.0f,
                             "post_event_timer should be 3.0s after goal", 0.01f);
    }

    s.unload();
}

// =========================================================================
// Test 5: Out of bounds detected → DEAD_BALL
// =========================================================================
static void test_oob_detected()
{
    std::printf("  [5] oob_detected\n");
    SceneState s = make_loaded_soccer_scene();
    s.rules.current_state = SemanticPlayState::LIVE_PLAY;

    // Place ball well outside the field (past the OOB boundary strip)
    BallEntity* ball = s.entity_manager.find_ball();
    if (ball) {
        ball->position = Vec3(0.0f, 0.5f, 100.0f); // Way past sideline

        s.evaluate_spatial_rules();

        TEST_ASSERT(s.rules.current_state == SemanticPlayState::DEAD_BALL,
                    "State should transition to DEAD_BALL after OOB");
    }

    s.unload();
}

// =========================================================================
// Test 6: OOB sets post_event_timer to 2.0s
// =========================================================================
static void test_oob_sets_post_event_timer()
{
    std::printf("  [6] oob_sets_post_event_timer\n");
    SceneState s = make_loaded_soccer_scene();
    s.rules.current_state = SemanticPlayState::LIVE_PLAY;
    s.post_event_timer = 0.0f;

    BallEntity* ball = s.entity_manager.find_ball();
    if (ball) {
        ball->position = Vec3(0.0f, 0.5f, 100.0f);

        s.evaluate_spatial_rules();

        TEST_ASSERT_FLOAT_EQ(s.post_event_timer, 2.0f,
                             "post_event_timer should be 2.0s after OOB", 0.01f);
    }

    s.unload();
}

// =========================================================================
// Test 7: Timer expiry transitions to PRE_GAME
// =========================================================================
static void test_timer_expiry_resets_to_pregame()
{
    std::printf("  [7] timer_expiry_resets_to_pregame\n");
    SceneState s = make_loaded_soccer_scene();
    s.rules.current_state = SemanticPlayState::SCORING_EVENT;
    s.post_event_timer = 3.0f;

    // Tick past the timer
    s.update_match_flow(3.5f);

    TEST_ASSERT(s.rules.current_state == SemanticPlayState::PRE_GAME,
                "State should transition to PRE_GAME after timer expires");
    TEST_ASSERT(s.post_event_timer <= 0.0f,
                "post_event_timer should be <= 0 after expiry");

    s.unload();
}

// =========================================================================
// Test 8: Timer expiry calls reset_match_positions
// =========================================================================
static void test_timer_expiry_calls_reset()
{
    std::printf("  [8] timer_expiry_calls_reset\n");
    SceneState s = make_loaded_soccer_scene();
    s.rules.current_state = SemanticPlayState::DEAD_BALL;
    s.post_event_timer = 2.0f;

    // Move an athlete away from home position to verify reset
    if (s.entity_manager.athlete_count > 0) {
        AthleteEntity& a = s.entity_manager.athletes[0];
        Vec3 original_home = a.home_position;
        a.position = Vec3(999.0f, 0.0f, 999.0f); // Far away
        a.velocity = Vec3(10.0f, 0.0f, 10.0f);   // Moving

        // Tick past the timer
        s.update_match_flow(2.5f);

        // Athlete should be back at home position
        TEST_ASSERT_FLOAT_EQ(a.position.x, original_home.x,
                             "Athlete X should be reset to home", 0.01f);
        TEST_ASSERT_FLOAT_EQ(a.position.z, original_home.z,
                             "Athlete Z should be reset to home", 0.01f);
        TEST_ASSERT_FLOAT_EQ(a.velocity.x, 0.0f,
                             "Athlete velocity X should be zeroed", 0.01f);
        TEST_ASSERT_FLOAT_EQ(a.velocity.z, 0.0f,
                             "Athlete velocity Z should be zeroed", 0.01f);
    }

    s.unload();
}

// =========================================================================
// Test 9: No spatial check when not in LIVE_PLAY
// =========================================================================
static void test_no_spatial_check_when_not_live()
{
    std::printf("  [9] no_spatial_check_when_not_live\n");
    SceneState s = make_loaded_soccer_scene();

    // Set to PRE_GAME (not LIVE)
    s.rules.current_state = SemanticPlayState::PRE_GAME;
    s.post_event_timer = 0.0f;

    BallEntity* ball = s.entity_manager.find_ball();
    if (ball) {
        // Place ball in scoring zone
        ball->position = Vec3(53.0f, 0.5f, 0.0f);

        // Call update_match_flow — should NOT evaluate spatial rules
        // because state is PRE_GAME, not LIVE_PLAY
        s.update_match_flow(1.0f);

        TEST_ASSERT(s.home_score == 0u,
                    "No goal should be detected in PRE_GAME");
        TEST_ASSERT(s.rules.current_state == SemanticPlayState::PRE_GAME,
                    "State should remain PRE_GAME");
    }

    s.unload();
}

// =========================================================================
// Test 10: unload() resets post_event_timer
// =========================================================================
static void test_unload_resets_post_event_timer()
{
    std::printf("  [10] unload_resets_post_event_timer\n");
    SceneState s = make_loaded_soccer_scene();
    s.post_event_timer = 5.0f;

    s.unload();

    TEST_ASSERT_FLOAT_EQ(s.post_event_timer, 0.0f,
                         "unload should zero post_event_timer", 0.0001f);
}

// =========================================================================
// Test 11: Multiple goals accumulate correctly
// =========================================================================
static void test_multiple_goals_accumulate()
{
    std::printf("  [11] multiple_goals_accumulate\n");
    SceneState s = make_loaded_soccer_scene();

    BallEntity* ball = s.entity_manager.find_ball();
    if (ball) {
        // Goal 1: Home scores (+X)
        s.rules.current_state = SemanticPlayState::LIVE_PLAY;
        ball->position = Vec3(53.0f, 0.5f, 0.0f);
        s.evaluate_spatial_rules();
        TEST_ASSERT(s.home_score == 1u, "Home should have 1 goal");

        // Reset to PRE_GAME for next play
        s.rules.current_state = SemanticPlayState::PRE_GAME;
        s.post_event_timer = 0.0f;

        // Goal 2: Away scores (-X)
        s.rules.current_state = SemanticPlayState::LIVE_PLAY;
        ball->position = Vec3(-53.0f, 0.5f, 0.0f);
        s.evaluate_spatial_rules();
        TEST_ASSERT(s.away_score == 1u, "Away should have 1 goal");
        TEST_ASSERT(s.home_score == 1u, "Home should still have 1 goal");

        // Reset for next play
        s.rules.current_state = SemanticPlayState::PRE_GAME;
        s.post_event_timer = 0.0f;

        // Goal 3: Home scores again
        s.rules.current_state = SemanticPlayState::LIVE_PLAY;
        ball->position = Vec3(53.0f, 0.5f, 1.0f);
        s.evaluate_spatial_rules();
        TEST_ASSERT(s.home_score == 2u, "Home should have 2 goals");
        TEST_ASSERT(s.away_score == 1u, "Away should still have 1 goal");
    }

    s.unload();
}

// =========================================================================
// Main
// =========================================================================
int main()
{
    std::printf("=== Sprint 33: Phase 16 Action 1 — Spatial Event Triggers ===\n\n");

    test_scoring_detected_home_goal();
    test_scoring_detected_away_goal();
    test_scoring_transitions_state();
    test_scoring_sets_post_event_timer();
    test_oob_detected();
    test_oob_sets_post_event_timer();
    test_timer_expiry_resets_to_pregame();
    test_timer_expiry_calls_reset();
    test_no_spatial_check_when_not_live();
    test_unload_resets_post_event_timer();
    test_multiple_goals_accumulate();

    std::printf("\n  Results: %d passed, %d failed\n\n", g_tests_passed, g_tests_failed);
    return (g_tests_failed == 0) ? 0 : 1;
}
