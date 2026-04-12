// =============================================================================
// test_sprint34.cpp — Phase 16 Action 2: Possession Tracking & Time Violations
// =============================================================================
//
// Validates the macro-possession and shot clock logic:
//
//   1. macro_possession_initial_state      — Starts as TEAM_NONE after load
//   2. shot_clock_initial_state            — Starts at 0.0f after load
//   3. turnover_resets_shot_clock         — possession_team change → clock reset
//   4. turnover_sets_macro_possession      — macro_possession_team tracks ball
//   5. shot_clock_value_from_config        — 24000ms → 24.0s shot clock
//   6. shot_clock_no_reset_same_team      — Same team touching again → no reset
//   7. shot_clock_ticks_down              — Clock decreases each update_match_flow
//   8. shot_clock_violation_triggers_db   — Clock ≤ 0 → DEAD_BALL
//   9. shot_clock_violation_clears_macro  — Violation clears macro_possession_team
//  10. shot_clock_violation_sets_timer    — Violation sets post_event_timer = 2.0s
//  11. no_violation_when_clock_disabled   — Soccer (0ms) → no shot clock check
//  12. loose_ball_macro_persists          — Ball loose → macro stays, clock ticks
//  13. reset_clears_macro_possession      — reset_match_positions() clears both
//  14. unload_clears_macro_possession     — unload() zeroes both fields
//  15. basketball_config_has_clock        — Basketball rules = 24000ms
//  16. soccer_config_no_clock             — Soccer rules = 0ms (no clock)
//  17. spatial_rule_short_circuits_clock  — Goal/OOB stops shot clock ticking
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
// Helper: Load a basketball match (shot clock sport)
// =========================================================================
static SceneState make_basketball_scene()
{
    SceneState s;
    MatchConfig mc = MatchConfig::basketball_sandbox();
    s.load_match(mc);
    s.max_period_duration = 300.0f;
    return s;
}

// =========================================================================
// Helper: Load a soccer match (no shot clock)
// =========================================================================
static SceneState make_soccer_scene()
{
    SceneState s;
    MatchConfig mc = MatchConfig::soccer_sandbox();
    s.load_match(mc);
    s.max_period_duration = 300.0f;
    return s;
}

// =========================================================================
// Test 1: macro_possession_team starts as TEAM_NONE after load
// =========================================================================
static void test_macro_possession_initial_state()
{
    std::printf("  [1] macro_possession_initial_state\n");
    SceneState s = make_basketball_scene();

    TEST_ASSERT(s.macro_possession_team == TEAM_NONE,
                "macro_possession_team should start as TEAM_NONE");
    TEST_ASSERT(s.shot_clock_seconds == 0.0f,
                "shot_clock_seconds should start at 0.0f");

    s.unload();
}

// =========================================================================
// Test 2: shot_clock_seconds starts at 0.0f after load
// =========================================================================
static void test_shot_clock_initial_state()
{
    std::printf("  [2] shot_clock_initial_state\n");
    SceneState s = make_soccer_scene();

    TEST_ASSERT(s.shot_clock_seconds == 0.0f,
                "shot_clock_seconds should start at 0.0f in soccer");

    s.unload();
}

// =========================================================================
// Test 3: Turnover (possession_team change) resets the shot clock
// =========================================================================
static void test_turnover_resets_shot_clock()
{
    std::printf("  [3] turnover_resets_shot_clock\n");
    SceneState s = make_basketball_scene();
    s.rules.current_state = SemanticPlayState::LIVE_PLAY;

    BallEntity* ball = s.entity_manager.find_ball();
    TEST_ASSERT(ball != nullptr, "Ball should exist");
    if (!ball) { s.unload(); return; }

    // First possession: TEAM_HOME grabs the ball
    ball->possession_team = TEAM_HOME;
    s.update(1.0f / 240.0f);
    TEST_ASSERT(s.macro_possession_team == TEAM_HOME,
                "macro should be TEAM_HOME after first possession");
    TEST_ASSERT(s.shot_clock_seconds > 0.0f,
                "Shot clock should be set after first possession");
    float first_clock = s.shot_clock_seconds;

    // Simulate clock ticking down
    for (int i = 0; i < 240; ++i) {
        s.update(1.0f / 240.0f);
    }
    TEST_ASSERT(s.shot_clock_seconds < first_clock,
                "Shot clock should have decreased after 1 second");

    // Turnover: TEAM_AWAY steals the ball
    ball->possession_team = TEAM_AWAY;
    s.update(1.0f / 240.0f);
    TEST_ASSERT(s.macro_possession_team == TEAM_AWAY,
                "macro should switch to TEAM_AWAY after turnover");
    TEST_ASSERT_FLOAT_EQ(s.shot_clock_seconds, 24.0f,
                         "Shot clock should reset to 24.0s on turnover", 0.1f);

    s.unload();
}

// =========================================================================
// Test 4: Turnover sets macro_possession_team
// =========================================================================
static void test_turnover_sets_macro_possession()
{
    std::printf("  [4] turnover_sets_macro_possession\n");
    SceneState s = make_basketball_scene();
    s.rules.current_state = SemanticPlayState::LIVE_PLAY;

    BallEntity* ball = s.entity_manager.find_ball();
    if (!ball) { s.unload(); return; }

    // TEAM_HOME gains possession
    ball->possession_team = TEAM_HOME;
    s.update(1.0f / 240.0f);
    TEST_ASSERT(s.macro_possession_team == TEAM_HOME,
                "macro should be TEAM_HOME");

    // TEAM_AWAY gains possession (steal)
    ball->possession_team = TEAM_AWAY;
    s.update(1.0f / 240.0f);
    TEST_ASSERT(s.macro_possession_team == TEAM_AWAY,
                "macro should be TEAM_AWAY after steal");

    // Back to TEAM_HOME
    ball->possession_team = TEAM_HOME;
    s.update(1.0f / 240.0f);
    TEST_ASSERT(s.macro_possession_team == TEAM_HOME,
                "macro should be TEAM_HOME after re-steal");

    s.unload();
}

// =========================================================================
// Test 5: Shot clock value derived from config (24000ms → 24.0s)
// =========================================================================
static void test_shot_clock_value_from_config()
{
    std::printf("  [5] shot_clock_value_from_config\n");
    SceneState s = make_basketball_scene();
    s.rules.current_state = SemanticPlayState::LIVE_PLAY;

    // Verify config was set correctly
    TEST_ASSERT(s.rules.possession_clock_max_ms == 24000u,
                "Basketball config should have 24000ms shot clock");

    BallEntity* ball = s.entity_manager.find_ball();
    if (!ball) { s.unload(); return; }

    ball->possession_team = TEAM_HOME;
    s.update(1.0f / 240.0f);

    TEST_ASSERT_FLOAT_EQ(s.shot_clock_seconds, 24.0f,
                         "Shot clock should be 24.0s from 24000ms config", 0.1f);

    s.unload();
}

// =========================================================================
// Test 6: Same team touching again does NOT reset shot clock
// =========================================================================
static void test_shot_clock_no_reset_same_team()
{
    std::printf("  [6] shot_clock_no_reset_same_team\n");
    SceneState s = make_basketball_scene();
    s.rules.current_state = SemanticPlayState::LIVE_PLAY;

    BallEntity* ball = s.entity_manager.find_ball();
    if (!ball) { s.unload(); return; }

    ball->possession_team = TEAM_HOME;
    s.update(1.0f / 240.0f);
    float clock_after_first = s.shot_clock_seconds;

    // Tick some time
    for (int i = 0; i < 120; ++i) {
        s.update(1.0f / 240.0f);
    }
    float clock_after_tick = s.shot_clock_seconds;
    TEST_ASSERT(clock_after_tick < clock_after_first,
                "Clock should have decreased after 0.5s");

    // Same team touches again (e.g., pass between teammates)
    // This should NOT reset the clock because macro_possession_team is already TEAM_HOME
    float before_retouch = s.shot_clock_seconds;
    ball->possession_team = TEAM_HOME; // Same team
    s.update(1.0f / 240.0f);
    TEST_ASSERT(std::fabs(s.shot_clock_seconds - (before_retouch - 1.0f/240.0f)) < 0.01f,
                "Clock should continue ticking, not reset, for same-team touch");

    s.unload();
}

// =========================================================================
// Test 7: Shot clock ticks down during LIVE_PLAY
// =========================================================================
static void test_shot_clock_ticks_down()
{
    std::printf("  [7] shot_clock_ticks_down\n");
    SceneState s = make_basketball_scene();
    s.rules.current_state = SemanticPlayState::LIVE_PLAY;

    BallEntity* ball = s.entity_manager.find_ball();
    if (!ball) { s.unload(); return; }

    ball->possession_team = TEAM_HOME;
    s.update(1.0f / 240.0f);
    float initial = s.shot_clock_seconds;

    // Advance 5 seconds (5 * 240 = 1200 ticks)
    for (int i = 0; i < 1200; ++i) {
        s.update(1.0f / 240.0f);
    }

    TEST_ASSERT(s.shot_clock_seconds < initial - 4.0f,
                "Shot clock should have decreased by ~5s after 5s of play");
    TEST_ASSERT(s.shot_clock_seconds > 0.0f,
                "Shot clock should still be positive after 5s (24s total)");

    s.unload();
}

// =========================================================================
// Test 8: Shot clock violation triggers DEAD_BALL
// =========================================================================
static void test_shot_clock_violation_triggers_dead_ball()
{
    std::printf("  [8] shot_clock_violation_triggers_dead_ball\n");
    SceneState s = make_basketball_scene();
    s.rules.current_state = SemanticPlayState::LIVE_PLAY;

    BallEntity* ball = s.entity_manager.find_ball();
    if (!ball) { s.unload(); return; }

    // Set up possession with a small remaining clock
    ball->possession_team = TEAM_HOME;
    s.macro_possession_team = TEAM_HOME;
    s.shot_clock_seconds = 0.5f; // Only 0.5s left

    // Tick past the violation
    s.update_match_flow(1.0f);

    TEST_ASSERT(s.rules.current_state == SemanticPlayState::DEAD_BALL,
                "State should be DEAD_BALL after shot clock violation");

    s.unload();
}

// =========================================================================
// Test 9: Shot clock violation clears macro_possession_team
// =========================================================================
static void test_shot_clock_violation_clears_macro()
{
    std::printf("  [9] shot_clock_violation_clears_macro\n");
    SceneState s = make_basketball_scene();
    s.rules.current_state = SemanticPlayState::LIVE_PLAY;

    BallEntity* ball = s.entity_manager.find_ball();
    if (!ball) { s.unload(); return; }

    ball->possession_team = TEAM_HOME;
    s.macro_possession_team = TEAM_HOME;
    s.shot_clock_seconds = 0.1f;

    s.update_match_flow(0.5f);

    TEST_ASSERT(s.macro_possession_team == TEAM_NONE,
                "macro_possession_team should be cleared after violation");

    s.unload();
}

// =========================================================================
// Test 10: Shot clock violation sets post_event_timer = 2.0s
// =========================================================================
static void test_shot_clock_violation_sets_timer()
{
    std::printf("  [10] shot_clock_violation_sets_timer\n");
    SceneState s = make_basketball_scene();
    s.rules.current_state = SemanticPlayState::LIVE_PLAY;

    BallEntity* ball = s.entity_manager.find_ball();
    if (!ball) { s.unload(); return; }

    ball->possession_team = TEAM_HOME;
    s.macro_possession_team = TEAM_HOME;
    s.shot_clock_seconds = 0.01f;

    s.update_match_flow(0.1f);

    TEST_ASSERT(s.rules.current_state == SemanticPlayState::DEAD_BALL,
                "Should be DEAD_BALL");
    TEST_ASSERT_FLOAT_EQ(s.post_event_timer, 2.0f,
                         "post_event_timer should be 2.0s after shot clock violation", 0.01f);

    s.unload();
}

// =========================================================================
// Test 11: No violation when shot clock is disabled (soccer)
// =========================================================================
static void test_no_violation_when_clock_disabled()
{
    std::printf("  [11] no_violation_when_clock_disabled\n");
    SceneState s = make_soccer_scene();
    s.rules.current_state = SemanticPlayState::LIVE_PLAY;

    TEST_ASSERT(s.rules.possession_clock_max_ms == 0u,
                "Soccer should have 0ms possession clock");

    BallEntity* ball = s.entity_manager.find_ball();
    if (!ball) { s.unload(); return; }

    // Manually set macro possession and a zero/expired shot clock.
    // In soccer, the violation block is gated by possession_clock_max_ms > 0,
    // so even with macro_possession_team set and shot_clock at 0, no violation.
    ball->possession_team = TEAM_HOME;
    s.macro_possession_team = TEAM_HOME;
    s.shot_clock_seconds = -5.0f; // Would trigger violation if clock were enabled

    // Tick several frames — if the clock check fires, we get DEAD_BALL
    for (int i = 0; i < 240; ++i) {
        s.update_match_flow(1.0f / 240.0f);
    }

    // State should still be LIVE_PLAY — no shot clock violation in soccer
    TEST_ASSERT(s.rules.current_state == SemanticPlayState::LIVE_PLAY,
                "Soccer should remain LIVE_PLAY (no shot clock enforcement)");

    s.unload();
}

// =========================================================================
// Test 12: Loose ball — macro_possession_team persists, clock keeps ticking
// =========================================================================
static void test_loose_ball_macro_persists()
{
    std::printf("  [12] loose_ball_macro_persists\n");
    SceneState s = make_basketball_scene();
    s.rules.current_state = SemanticPlayState::LIVE_PLAY;

    BallEntity* ball = s.entity_manager.find_ball();
    if (!ball) { s.unload(); return; }

    // TEAM_HOME has possession
    ball->possession_team = TEAM_HOME;
    s.update(1.0f / 240.0f);
    float clock_after_possess = s.shot_clock_seconds;

    // Ball goes loose (no one touching it)
    ball->possession_team = TEAM_NONE;
    s.possession_timer = 0.0f; // Force immediate loose detection

    // Tick some time
    for (int i = 0; i < 120; ++i) {
        s.update(1.0f / 240.0f);
    }

    TEST_ASSERT(s.macro_possession_team == TEAM_HOME,
                "macro_possession_team should persist while ball is loose");
    TEST_ASSERT(s.shot_clock_seconds < clock_after_possess,
                "Shot clock should still tick while ball is loose");

    s.unload();
}

// =========================================================================
// Test 13: reset_match_positions() clears macro possession
// =========================================================================
static void test_reset_clears_macro_possession()
{
    std::printf("  [13] reset_clears_macro_possession\n");
    SceneState s = make_basketball_scene();
    s.rules.current_state = SemanticPlayState::LIVE_PLAY;

    BallEntity* ball = s.entity_manager.find_ball();
    if (!ball) { s.unload(); return; }

    ball->possession_team = TEAM_HOME;
    s.update(1.0f / 240.0f);
    TEST_ASSERT(s.macro_possession_team == TEAM_HOME,
                "Precondition: macro should be set");

    s.reset_match_positions();

    TEST_ASSERT(s.macro_possession_team == TEAM_NONE,
                "macro_possession_team should be TEAM_NONE after reset");
    TEST_ASSERT_FLOAT_EQ(s.shot_clock_seconds, 0.0f,
                         "shot_clock_seconds should be 0.0f after reset", 0.0001f);

    s.unload();
}

// =========================================================================
// Test 14: unload() clears macro possession
// =========================================================================
static void test_unload_clears_macro_possession()
{
    std::printf("  [14] unload_clears_macro_possession\n");
    SceneState s = make_basketball_scene();

    s.macro_possession_team = TEAM_HOME;
    s.shot_clock_seconds = 15.0f;

    s.unload();

    TEST_ASSERT(s.macro_possession_team == TEAM_NONE,
                "macro_possession_team should be TEAM_NONE after unload");
    TEST_ASSERT_FLOAT_EQ(s.shot_clock_seconds, 0.0f,
                         "shot_clock_seconds should be 0.0f after unload", 0.0001f);
}

// =========================================================================
// Test 15: Basketball config has 24000ms shot clock
// =========================================================================
static void test_basketball_config_has_clock()
{
    std::printf("  [15] basketball_config_has_clock\n");
    SceneState s = make_basketball_scene();

    TEST_ASSERT(s.rules.possession_clock_max_ms == 24000u,
                "Basketball should have 24-second shot clock");
    TEST_ASSERT(s.rules.hands_allowed_field == true,
                "Basketball should allow hands");
    TEST_ASSERT(s.rules.feet_allowed_field == false,
                "Basketball should not allow feet");
    TEST_ASSERT(s.rules.offside_rule_active == false,
                "Basketball should not have offside");

    s.unload();
}

// =========================================================================
// Test 16: Soccer config has 0ms shot clock (disabled)
// =========================================================================
static void test_soccer_config_no_clock()
{
    std::printf("  [16] soccer_config_no_clock\n");
    SceneState s = make_soccer_scene();

    TEST_ASSERT(s.rules.possession_clock_max_ms == 0u,
                "Soccer should have no possession clock");
    TEST_ASSERT(s.rules.hands_allowed_field == false,
                "Soccer should not allow hands");
    TEST_ASSERT(s.rules.feet_allowed_field == true,
                "Soccer should allow feet");
    TEST_ASSERT(s.rules.offside_rule_active == true,
                "Soccer should have offside active");

    s.unload();
}

// =========================================================================
// Test 17: Spatial rule (goal/OOB) short-circuits shot clock ticking
// =========================================================================
static void test_spatial_rule_short_circuits_clock()
{
    std::printf("  [17] spatial_rule_short_circuits_clock\n");
    SceneState s = make_basketball_scene();
    s.rules.current_state = SemanticPlayState::LIVE_PLAY;

    BallEntity* ball = s.entity_manager.find_ball();
    TEST_ASSERT(ball != nullptr, "Ball should exist");
    if (!ball) { s.unload(); return; }

    // Set up possession with shot clock running
    ball->possession_team = TEAM_HOME;
    s.update(1.0f / 240.0f);
    float clock_before = s.shot_clock_seconds;
    TEST_ASSERT(clock_before > 0.0f, "Shot clock should be set");

    // Move ball to SCORING_TARGET zone (triggers goal → SCORING_EVENT)
    // Basketball SCORING_TARGET at -X: Vec3(-half_l - 0.5, goal_h-1, -gw/2)
    // half_l = 14.0, goal_h = 3.05, gw = 0.457
    ball->position = Vec3(-14.3f, 3.0f, 0.0f);

    // update_match_flow should detect the goal and NOT tick the shot clock
    s.update_match_flow(1.0f / 240.0f);

    TEST_ASSERT(s.rules.current_state == SemanticPlayState::SCORING_EVENT,
                "State should be SCORING_EVENT after goal");
    // The shot clock should NOT have ticked down further because the spatial
    // rule triggered a state change, which returns early before the clock check
    TEST_ASSERT(s.shot_clock_seconds >= clock_before - 0.01f,
                "Shot clock should NOT tick after spatial event triggers");

    s.unload();
}

// =========================================================================
// Main
// =========================================================================
int main()
{
    std::printf("=== Sprint 34: Phase 16 Action 2 — Possession Tracking & Time Violations ===\n\n");

    test_macro_possession_initial_state();
    test_shot_clock_initial_state();
    test_turnover_resets_shot_clock();
    test_turnover_sets_macro_possession();
    test_shot_clock_value_from_config();
    test_shot_clock_no_reset_same_team();
    test_shot_clock_ticks_down();
    test_shot_clock_violation_triggers_dead_ball();
    test_shot_clock_violation_clears_macro();
    test_shot_clock_violation_sets_timer();
    test_no_violation_when_clock_disabled();
    test_loose_ball_macro_persists();
    test_reset_clears_macro_possession();
    test_unload_clears_macro_possession();
    test_basketball_config_has_clock();
    test_soccer_config_no_clock();
    test_spatial_rule_short_circuits_clock();

    std::printf("\n  Results: %d passed, %d failed\n\n", g_tests_passed, g_tests_failed);
    return (g_tests_failed == 0) ? 0 : 1;
}
