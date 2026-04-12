// =============================================================================
// test_sprint31.cpp — Phase 15 Action 1: Match State Machine & Clock Management
// =============================================================================
//
// Validates the referee / clock system:
//
//   1. default_state_is_pre_game      — SceneState starts in PRE_GAME
//   2. clock_does_not_tick_pre_game   — Period/match time frozen before kickoff
//   3. clock_ticks_during_live_play   — Period + match time advance at 1:1 dt
//   4. clock_freezes_on_dead_ball     — Switching to DEAD_BALL halts clock
//   5. clock_freezes_on_intermission  — Switching to INTERMISSION halts clock
//   6. period_expires_triggers_intermission — Auto-transition when period ends
//   7. full_time_two_period_game      — 2-half game: period 2 expiry = INTERMISSION
//   8. unload_resets_clock_state      — unload() wipes all clock members
//   9. reset_match_resets_clock_state — reset_match() returns to PRE_GAME
//  10. load_match_configures_rules    — load_match() stores SportRulesConfig
//  11. max_period_duration_custom     — Custom period duration respected
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

// Helper: create a minimal SceneState for clock-only tests (no entities needed)
static SceneState make_test_scene()
{
    SceneState s;
    s.max_period_duration = 10.0f; // 10-second periods for fast tests
    s.config.halves = 2;
    return s;
}

// =========================================================================
// Test 1: Default state is PRE_GAME
// =========================================================================
static void test_default_state_is_pre_game()
{
    std::printf("  [1] default_state_is_pre_game\n");
    SceneState s = make_test_scene();

    TEST_ASSERT(s.rules.current_state == SemanticPlayState::PRE_GAME,
                "Initial state should be PRE_GAME");
    TEST_ASSERT(s.current_period == 1u,
                "Initial period should be 1");
    TEST_ASSERT_FLOAT_EQ(s.period_time_seconds, 0.0f,
                         "Period time should start at 0", 0.0001f);
    TEST_ASSERT_FLOAT_EQ(s.match_time_seconds, 0.0f,
                         "Match time should start at 0", 0.0001f);
}

// =========================================================================
// Test 2: Clock does not tick during PRE_GAME
// =========================================================================
static void test_clock_does_not_tick_pre_game()
{
    std::printf("  [2] clock_does_not_tick_pre_game\n");
    SceneState s = make_test_scene();
    // Deliberately leave state as PRE_GAME (default)

    s.update_match_flow(1.0f);
    s.update_match_flow(1.0f);
    s.update_match_flow(0.5f);

    TEST_ASSERT_FLOAT_EQ(s.period_time_seconds, 0.0f,
                         "Period time should not advance in PRE_GAME", 0.0001f);
    TEST_ASSERT_FLOAT_EQ(s.match_time_seconds, 0.0f,
                         "Match time should not advance in PRE_GAME", 0.0001f);
}

// =========================================================================
// Test 3: Clock ticks during LIVE_PLAY at 1:1 ratio
// =========================================================================
static void test_clock_ticks_during_live_play()
{
    std::printf("  [3] clock_ticks_during_live_play\n");
    SceneState s = make_test_scene();
    s.rules.current_state = SemanticPlayState::LIVE_PLAY;

    // Simulate 240 ticks at 1/240s each = 1.0s total
    float dt = 1.0f / 240.0f;
    for (int i = 0; i < 240; ++i) {
        s.update_match_flow(dt);
    }

    TEST_ASSERT_FLOAT_EQ(s.period_time_seconds, 1.0f,
                         "Period time should be ~1.0s after 240 ticks", 0.01f);
    TEST_ASSERT_FLOAT_EQ(s.match_time_seconds, 1.0f,
                         "Match time should be ~1.0s after 240 ticks", 0.01f);
}

// =========================================================================
// Test 4: Clock freezes on DEAD_BALL
// =========================================================================
static void test_clock_freezes_on_dead_ball()
{
    std::printf("  [4] clock_freezes_on_dead_ball\n");
    SceneState s = make_test_scene();
    s.rules.current_state = SemanticPlayState::LIVE_PLAY;

    // Tick 1 second of live play
    s.update_match_flow(1.0f);
    TEST_ASSERT_FLOAT_EQ(s.period_time_seconds, 1.0f,
                         "Should have 1.0s after live play", 0.0001f);

    // Dead ball — clock stops
    s.rules.current_state = SemanticPlayState::DEAD_BALL;
    s.update_match_flow(1.0f);
    s.update_match_flow(2.0f);

    TEST_ASSERT_FLOAT_EQ(s.period_time_seconds, 1.0f,
                         "Period time should freeze on dead ball", 0.0001f);
    TEST_ASSERT_FLOAT_EQ(s.match_time_seconds, 1.0f,
                         "Match time should freeze on dead ball", 0.0001f);
}

// =========================================================================
// Test 5: Clock freezes on INTERMISSION
// =========================================================================
static void test_clock_freezes_on_intermission()
{
    std::printf("  [5] clock_freezes_on_intermission\n");
    SceneState s = make_test_scene();
    s.rules.current_state = SemanticPlayState::LIVE_PLAY;

    s.update_match_flow(0.5f);
    s.rules.current_state = SemanticPlayState::INTERMISSION;
    s.update_match_flow(5.0f);

    TEST_ASSERT_FLOAT_EQ(s.period_time_seconds, 0.5f,
                         "Period time should freeze on intermission", 0.0001f);
    TEST_ASSERT_FLOAT_EQ(s.match_time_seconds, 0.5f,
                         "Match time should freeze on intermission", 0.0001f);
}

// =========================================================================
// Test 6: Period expiry triggers INTERMISSION automatically
// =========================================================================
static void test_period_expires_triggers_intermission()
{
    std::printf("  [6] period_expires_triggers_intermission\n");
    SceneState s = make_test_scene();
    s.max_period_duration = 5.0f;
    s.rules.current_state = SemanticPlayState::LIVE_PLAY;

    // Tick just under the limit
    s.update_match_flow(4.9f);
    TEST_ASSERT(s.rules.current_state == SemanticPlayState::LIVE_PLAY,
                "Should still be LIVE at 4.9s");

    // Push past the limit
    s.update_match_flow(0.2f);
    TEST_ASSERT(s.rules.current_state == SemanticPlayState::INTERMISSION,
                "Should transition to INTERMISSION at 5.1s");
}

// =========================================================================
// Test 7: Full time in a 2-period game
// =========================================================================
static void test_full_time_two_period_game()
{
    std::printf("  [7] full_time_two_period_game\n");
    SceneState s = make_test_scene();
    s.max_period_duration = 3.0f;
    s.config.halves = 2;
    s.current_period = 1u;
    s.rules.current_state = SemanticPlayState::LIVE_PLAY;

    // Period 1 expires
    s.update_match_flow(3.0f);
    TEST_ASSERT(s.rules.current_state == SemanticPlayState::INTERMISSION,
                "Period 1 expiry should trigger INTERMISSION");

    // Manually advance to period 2 (simulating half-time whistle)
    s.current_period = 2u;
    s.period_time_seconds = 0.0f;
    s.rules.current_state = SemanticPlayState::LIVE_PLAY;

    // Period 2 expires — this is the final period
    s.update_match_flow(3.0f);
    TEST_ASSERT(s.rules.current_state == SemanticPlayState::INTERMISSION,
                "Period 2 expiry (full time) should trigger INTERMISSION");
}

// =========================================================================
// Test 8: unload() resets all clock state
// =========================================================================
static void test_unload_resets_clock_state()
{
    std::printf("  [8] unload_resets_clock_state\n");
    SceneState s = make_test_scene();
    s.rules.current_state = SemanticPlayState::LIVE_PLAY;
    s.update_match_flow(5.0f);
    s.current_period = 2u;

    // Verify we have non-default state
    TEST_ASSERT(s.period_time_seconds > 0.0f, "Precondition: period_time > 0");

    s.unload();

    TEST_ASSERT(s.rules.current_state == SemanticPlayState::PRE_GAME,
                "unload should reset state to PRE_GAME");
    TEST_ASSERT(s.current_period == 1u,
                "unload should reset period to 1");
    TEST_ASSERT_FLOAT_EQ(s.period_time_seconds, 0.0f,
                         "unload should zero period_time", 0.0001f);
    TEST_ASSERT_FLOAT_EQ(s.match_time_seconds, 0.0f,
                         "unload should zero match_time", 0.0001f);
    TEST_ASSERT_FLOAT_EQ(s.max_period_duration, 300.0f,
                         "unload should reset max_period_duration to 300", 0.0001f);
}

// =========================================================================
// Test 9: reset_match() resets clock to PRE_GAME
// =========================================================================
static void test_reset_match_resets_clock_state()
{
    std::printf("  [9] reset_match_resets_clock_state\n");
    SceneState s = make_test_scene();
    s.rules.current_state = SemanticPlayState::LIVE_PLAY;
    s.update_match_flow(7.0f);
    s.current_period = 2u;
    s.period_time_seconds = 4.0f;

    s.reset_match();

    TEST_ASSERT(s.rules.current_state == SemanticPlayState::PRE_GAME,
                "reset_match should return to PRE_GAME");
    TEST_ASSERT(s.current_period == 1u,
                "reset_match should reset period to 1");
    TEST_ASSERT_FLOAT_EQ(s.period_time_seconds, 0.0f,
                         "reset_match should zero period_time", 0.0001f);
    TEST_ASSERT_FLOAT_EQ(s.match_time_seconds, 0.0f,
                         "reset_match should zero match_time", 0.0001f);
}

// =========================================================================
// Test 10: load_match() configures rules via assign_all_ai()
// =========================================================================
static void test_load_match_configures_rules()
{
    std::printf("  [10] load_match_configures_rules\n");
    SceneState s;
    s.max_period_duration = 5.0f;
    s.config.halves = 2;

    // Load a match — this calls assign_all_ai() which calls
    // load_sport_configuration() which stores rules
    MatchConfig mc = MatchConfig::soccer_sandbox();
    uint8_t result = s.load_match(mc);
    TEST_ASSERT(result != 0u, "load_match should succeed");

    // rules should now be the default SportRulesConfig (PRE_GAME)
    TEST_ASSERT(s.rules.current_state == SemanticPlayState::PRE_GAME,
                "After load_match, rules should be in PRE_GAME");

    // Verify the rules member was actually written (not left at default
    // without the load_sport_configuration fix from Phase 15)
    // We can check by toggling state and verifying it persists
    s.rules.current_state = SemanticPlayState::LIVE_PLAY;
    s.update_match_flow(1.0f);
    TEST_ASSERT_FLOAT_EQ(s.period_time_seconds, 1.0f,
                         "Rules should be live after explicit state change", 0.01f);

    s.unload();
}

// =========================================================================
// Test 11: Custom max_period_duration is respected
// =========================================================================
static void test_max_period_duration_custom()
{
    std::printf("  [11] max_period_duration_custom\n");
    SceneState s = make_test_scene();
    s.max_period_duration = 7.5f;
    s.config.halves = 4; // basketball-style quarters
    s.rules.current_state = SemanticPlayState::LIVE_PLAY;

    // Tick 7.4s — should still be live
    s.update_match_flow(7.4f);
    TEST_ASSERT(s.rules.current_state == SemanticPlayState::LIVE_PLAY,
                "Should be LIVE at 7.4s with 7.5s period");

    // Tick 0.2s more — should transition
    s.update_match_flow(0.2f);
    TEST_ASSERT(s.rules.current_state == SemanticPlayState::INTERMISSION,
                "Should transition at 7.6s with 7.5s period");
}

// =========================================================================
// Main
// =========================================================================
int main()
{
    std::printf("=== Sprint 31: Phase 15 Action 1 — Match State Machine & Clock ===\n\n");

    test_default_state_is_pre_game();
    test_clock_does_not_tick_pre_game();
    test_clock_ticks_during_live_play();
    test_clock_freezes_on_dead_ball();
    test_clock_freezes_on_intermission();
    test_period_expires_triggers_intermission();
    test_full_time_two_period_game();
    test_unload_resets_clock_state();
    test_reset_match_resets_clock_state();
    test_load_match_configures_rules();
    test_max_period_duration_custom();

    std::printf("\n  Results: %d passed, %d failed\n\n", g_tests_passed, g_tests_failed);
    return (g_tests_failed == 0) ? 0 : 1;
}
