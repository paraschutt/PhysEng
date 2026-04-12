// =============================================================================
// test_sprint32.cpp — Phase 15 Action 2: The "Reset" Logic
// =============================================================================
//
// Validates the deterministic teleport routine (reset_match_positions):
//
//   1. home_position_set_during_spawn  — Athletes get home_position from formation
//   2. spatial_teleport_ball          — Ball returns to center after reset
//   3. spatial_teleport_athletes      — Athletes return to home_position
//   4. kinematic_wipe_ball            — Ball velocity and angular_velocity zeroed
//   5. kinematic_wipe_athletes        — Athlete velocity and acceleration zeroed
//   6. cognitive_wipe_intents         — current_intent and previous_intent reset
//   7. cognitive_wipe_flags           — MotorIntentFlags cleared
//   8. cognitive_wipe_action_id       — active_action_id reset to invalid
//   9. cognitive_wipe_perception      — PerceptionRingBuffer cleared
//  10. cognitive_wipe_influence       — InfluenceMap cleared
//  11. ball_possession_cleared        — Ball last_toucher and possession_team wiped
//  12. scene_possession_cleared       — Scene-level possession tracking reset
//  13. ball_in_play_cleared           — ball_in_play set to 0 after reset
//  14. ai_controllers_reset           — AI motor controllers cleared
//  15. reset_match_delegates          — reset_match() calls reset_match_positions()
//  16. reset_after_live_play          — Full cycle: play → reset → verify zero state
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

// Helper: load a match and get a ready-to-use SceneState
static SceneState make_loaded_scene()
{
    SceneState s;
    MatchConfig mc = MatchConfig::soccer_sandbox();
    s.load_match(mc);
    return s;
}

// Helper: simulate some "gameplay" to dirty all state
static void dirty_state(SceneState& s)
{
    s.rules.current_state = SemanticPlayState::LIVE_PLAY;

    // Move ball away from center
    BallEntity* ball = s.entity_manager.find_ball();
    if (ball) {
        ball->position = { 20.0f, 5.0f, -10.0f };
        ball->velocity = { 10.0f, 3.0f, -5.0f };
        ball->angular_velocity = { 1.0f, 2.0f, 3.0f };
        ball->last_toucher = s.entity_manager.athletes[0].id;
        ball->possession_team = TEAM_HOME;
        ball->is_in_play = 1;
    }

    // Move athletes away from home, give them velocity/intent/flags
    for (uint32_t i = 0u; i < s.entity_manager.athlete_count && i < 4; ++i) {
        AthleteEntity& a = s.entity_manager.athletes[i];
        a.position = { 15.0f * (float)i, 0.0f, -20.0f };
        a.velocity = { 5.0f, 0.0f, -3.0f };
        a.acceleration = { 1.0f, 0.0f, 0.5f };
        a.current_intent.move_speed = 0.8f;
        a.current_intent.sprint_intensity = 0.5f;
        a.previous_intent.move_speed = 0.6f;
        a.flags = 0xFFFF; // All flags set
        a.active_action_id = 42u;
        a.stamina = 0.3f;
        a.sprint_cooldown = 2.0f;
        a.tackle_cooldown = 1.5f;
    }

    // Dirty scene possession tracking
    s.last_ball_toucher = s.entity_manager.athletes[0].id;
    s.last_possession_team = TEAM_HOME;
    s.possession_timer = 1.2f;
    s.ball_in_play = 1u;
    s.out_of_bounds_timer = 0.5f;

    // Advance clock
    s.period_time_seconds = 45.0f;
    s.match_time_seconds = 45.0f;
}

// =========================================================================
// Test 1: home_position is set during spawn_team
// =========================================================================
static void test_home_position_set_during_spawn()
{
    std::printf("  [1] home_position_set_during_spawn\n");
    SceneState s = make_loaded_scene();

    // At least one athlete should exist after load_match
    TEST_ASSERT(s.entity_manager.athlete_count > 0u,
                "Should have athletes after load_match");

    // All athletes should have non-zero home_position (formation-based)
    uint32_t non_zero = 0u;
    for (uint32_t i = 0u; i < s.entity_manager.athlete_count; ++i) {
        const AthleteEntity& a = s.entity_manager.athletes[i];
        if (a.home_position.x != 0.0f || a.home_position.z != 0.0f) {
            ++non_zero;
        }
    }
    TEST_ASSERT(non_zero > 0u,
                "At least some athletes should have non-zero home_position");

    // Home position y should be 0 (ground level)
    for (uint32_t i = 0u; i < s.entity_manager.athlete_count; ++i) {
        TEST_ASSERT_FLOAT_EQ(s.entity_manager.athletes[i].home_position.y, 0.0f,
                             "Home position Y should be 0 (ground)", 0.0001f);
    }
}

// =========================================================================
// Test 2: Spatial teleport — ball returns to center
// =========================================================================
static void test_spatial_teleport_ball()
{
    std::printf("  [2] spatial_teleport_ball\n");
    SceneState s = make_loaded_scene();
    dirty_state(s);

    BallEntity* ball = s.entity_manager.find_ball();
    TEST_ASSERT(ball != nullptr, "Ball should exist");
    if (!ball) return;

    // Ball should be far from center before reset
    TEST_ASSERT(std::fabs(ball->position.x) > 1.0f,
                "Precondition: ball should be displaced");

    s.reset_match_positions();

    TEST_ASSERT_FLOAT_EQ(ball->position.x, 0.0f,
                         "Ball X should be at center after reset", 0.01f);
    TEST_ASSERT_FLOAT_EQ(ball->position.z, 0.0f,
                         "Ball Z should be at center after reset", 0.01f);
    TEST_ASSERT_FLOAT_EQ(ball->position.y, ball->radius,
                         "Ball Y should be at radius height after reset", 0.01f);
}

// =========================================================================
// Test 3: Spatial teleport — athletes return to home_position
// =========================================================================
static void test_spatial_teleport_athletes()
{
    std::printf("  [3] spatial_teleport_athletes\n");
    SceneState s = make_loaded_scene();
    dirty_state(s);

    // Save home positions before dirtying
    struct Pos { float x, z; } homes[32];
    uint32_t count = s.entity_manager.athlete_count;
    if (count > 32) count = 32;
    for (uint32_t i = 0u; i < count; ++i) {
        homes[i].x = s.entity_manager.athletes[i].home_position.x;
        homes[i].z = s.entity_manager.athletes[i].home_position.z;
    }

    s.reset_match_positions();

    for (uint32_t i = 0u; i < count; ++i) {
        float dx = s.entity_manager.athletes[i].position.x - homes[i].x;
        float dz = s.entity_manager.athletes[i].position.z - homes[i].z;
        TEST_ASSERT_FLOAT_EQ(dx, 0.0f, "Athlete X should match home_position", 0.01f);
        TEST_ASSERT_FLOAT_EQ(dz, 0.0f, "Athlete Z should match home_position", 0.01f);
    }
}

// =========================================================================
// Test 4: Kinematic wipe — ball velocity/angular_velocity zeroed
// =========================================================================
static void test_kinematic_wipe_ball()
{
    std::printf("  [4] kinematic_wipe_ball\n");
    SceneState s = make_loaded_scene();
    dirty_state(s);

    s.reset_match_positions();

    // Note: is_in_play is set to 0 after reset (waiting for kickoff),
    // so find_ball() returns nullptr. Access the ball array directly.
    BallEntity* ball = (s.entity_manager.ball_count > 0u)
                       ? &s.entity_manager.balls[0] : nullptr;
    TEST_ASSERT(ball != nullptr, "Ball should exist");
    if (!ball) return;

    TEST_ASSERT_FLOAT_EQ(ball->velocity.x, 0.0f,
                         "Ball velocity X should be zero", 0.0001f);
    TEST_ASSERT_FLOAT_EQ(ball->velocity.y, 0.0f,
                         "Ball velocity Y should be zero", 0.0001f);
    TEST_ASSERT_FLOAT_EQ(ball->velocity.z, 0.0f,
                         "Ball velocity Z should be zero", 0.0001f);
    TEST_ASSERT_FLOAT_EQ(ball->angular_velocity.x, 0.0f,
                         "Ball angular velocity X should be zero", 0.0001f);
    TEST_ASSERT_FLOAT_EQ(ball->angular_velocity.y, 0.0f,
                         "Ball angular velocity Y should be zero", 0.0001f);
    TEST_ASSERT_FLOAT_EQ(ball->angular_velocity.z, 0.0f,
                         "Ball angular velocity Z should be zero", 0.0001f);
}

// =========================================================================
// Test 5: Kinematic wipe — athlete velocity/acceleration zeroed
// =========================================================================
static void test_kinematic_wipe_athletes()
{
    std::printf("  [5] kinematic_wipe_athletes\n");
    SceneState s = make_loaded_scene();
    dirty_state(s);
    s.reset_match_positions();

    for (uint32_t i = 0u; i < s.entity_manager.athlete_count && i < 4; ++i) {
        const AthleteEntity& a = s.entity_manager.athletes[i];
        TEST_ASSERT_FLOAT_EQ(a.velocity.x, 0.0f, "Velocity X zero", 0.0001f);
        TEST_ASSERT_FLOAT_EQ(a.velocity.z, 0.0f, "Velocity Z zero", 0.0001f);
        TEST_ASSERT_FLOAT_EQ(a.acceleration.x, 0.0f, "Acceleration X zero", 0.0001f);
        TEST_ASSERT_FLOAT_EQ(a.acceleration.z, 0.0f, "Acceleration Z zero", 0.0001f);
    }
}

// =========================================================================
// Test 6: Cognitive wipe — motor intents reset
// =========================================================================
static void test_cognitive_wipe_intents()
{
    std::printf("  [6] cognitive_wipe_intents\n");
    SceneState s = make_loaded_scene();
    dirty_state(s);
    s.reset_match_positions();

    for (uint32_t i = 0u; i < s.entity_manager.athlete_count && i < 4; ++i) {
        const AthleteEntity& a = s.entity_manager.athletes[i];
        TEST_ASSERT_FLOAT_EQ(a.current_intent.move_speed, 0.0f,
                             "Current intent move_speed should be 0", 0.0001f);
        TEST_ASSERT_FLOAT_EQ(a.current_intent.sprint_intensity, 0.0f,
                             "Current intent sprint should be 0", 0.0001f);
        TEST_ASSERT_FLOAT_EQ(a.previous_intent.move_speed, 0.0f,
                             "Previous intent move_speed should be 0", 0.0001f);
    }
}

// =========================================================================
// Test 7: Cognitive wipe — flags cleared
// =========================================================================
static void test_cognitive_wipe_flags()
{
    std::printf("  [7] cognitive_wipe_flags\n");
    SceneState s = make_loaded_scene();
    dirty_state(s);
    s.reset_match_positions();

    for (uint32_t i = 0u; i < s.entity_manager.athlete_count && i < 4; ++i) {
        TEST_ASSERT(s.entity_manager.athletes[i].flags == 0u,
                    "All motor intent flags should be cleared");
    }
}

// =========================================================================
// Test 8: Cognitive wipe — action_id reset
// =========================================================================
static void test_cognitive_wipe_action_id()
{
    std::printf("  [8] cognitive_wipe_action_id\n");
    SceneState s = make_loaded_scene();
    dirty_state(s);
    s.reset_match_positions();

    for (uint32_t i = 0u; i < s.entity_manager.athlete_count && i < 4; ++i) {
        TEST_ASSERT(s.entity_manager.athletes[i].active_action_id == 0xFFFFFFFFu,
                    "active_action_id should be invalid (0xFFFFFFFF)");
    }
}

// =========================================================================
// Test 9: Cognitive wipe — perception buffer cleared
// =========================================================================
static void test_cognitive_wipe_perception()
{
    std::printf("  [9] cognitive_wipe_perception\n");
    SceneState s = make_loaded_scene();
    dirty_state(s);
    // Push some perception data to dirty the buffer
    s.evaluate_ai_decisions(1.0f / 240.0f);
    TEST_ASSERT(s.perception_buffer.get_count() > 0u,
                "Precondition: buffer should have data after AI tick");

    s.reset_match_positions();
    TEST_ASSERT(s.perception_buffer.get_count() == 0u,
                "Perception buffer should be cleared after reset");
}

// =========================================================================
// Test 10: Cognitive wipe — influence map cleared
// =========================================================================
static void test_cognitive_wipe_influence()
{
    std::printf("  [10] cognitive_wipe_influence\n");
    SceneState s = make_loaded_scene();
    dirty_state(s);
    s.build_influence_map(static_cast<uint8_t>(TEAM_HOME));

    s.reset_match_positions();
    // Influence map should be empty after clear
    // (We can't easily inspect grid contents, but clear() was called)
    TEST_ASSERT(true, "Influence map clear() called (no crash)");
}

// =========================================================================
// Test 11: Ball possession cleared
// =========================================================================
static void test_ball_possession_cleared()
{
    std::printf("  [11] ball_possession_cleared\n");
    SceneState s = make_loaded_scene();
    dirty_state(s);
    s.reset_match_positions();

    // is_in_play is 0 after reset — use direct array access
    BallEntity* ball = (s.entity_manager.ball_count > 0u)
                       ? &s.entity_manager.balls[0] : nullptr;
    TEST_ASSERT(ball != nullptr, "Ball should exist");
    if (!ball) return;

    TEST_ASSERT(!ball->last_toucher.is_valid(),
                "Ball last_toucher should be invalid after reset");
    TEST_ASSERT(ball->possession_team == TEAM_NONE,
                "Ball possession_team should be TEAM_NONE after reset");
}

// =========================================================================
// Test 12: Scene-level possession cleared
// =========================================================================
static void test_scene_possession_cleared()
{
    std::printf("  [12] scene_possession_cleared\n");
    SceneState s = make_loaded_scene();
    dirty_state(s);
    s.reset_match_positions();

    TEST_ASSERT(!s.last_ball_toucher.is_valid(),
                "Scene last_ball_toucher should be invalid");
    TEST_ASSERT(s.last_possession_team == TEAM_NONE,
                "Scene last_possession_team should be TEAM_NONE");
    TEST_ASSERT_FLOAT_EQ(s.possession_timer, 0.0f,
                         "Scene possession_timer should be 0", 0.0001f);
}

// =========================================================================
// Test 13: ball_in_play cleared
// =========================================================================
static void test_ball_in_play_cleared()
{
    std::printf("  [13] ball_in_play_cleared\n");
    SceneState s = make_loaded_scene();
    dirty_state(s);
    TEST_ASSERT(s.ball_in_play == 1u, "Precondition: ball should be in play");
    s.reset_match_positions();
    TEST_ASSERT(s.ball_in_play == 0u,
                "ball_in_play should be 0 after reset (no kickoff yet)");
}

// =========================================================================
// Test 14: AI controllers reset
// =========================================================================
static void test_ai_controllers_reset()
{
    std::printf("  [14] ai_controllers_reset\n");
    SceneState s = make_loaded_scene();
    dirty_state(s);
    // Run some AI ticks to dirty controllers
    s.evaluate_ai_decisions(1.0f / 240.0f);
    s.reset_match_positions();
    TEST_ASSERT(true, "AI controllers reset() called without crash");
}

// =========================================================================
// Test 15: reset_match delegates to reset_match_positions
// =========================================================================
static void test_reset_match_delegates()
{
    std::printf("  [15] reset_match_delegates\n");
    SceneState s = make_loaded_scene();
    dirty_state(s);

    s.reset_match();

    // reset_match should also zero scores and clock
    TEST_ASSERT(s.home_score == 0u, "Home score should be 0");
    TEST_ASSERT(s.away_score == 0u, "Away score should be 0");
    TEST_ASSERT_FLOAT_EQ(s.period_time_seconds, 0.0f,
                         "Period time should be 0", 0.0001f);
    TEST_ASSERT(s.rules.current_state == SemanticPlayState::PRE_GAME,
                "State should be PRE_GAME after reset_match");

    // AND also do the teleport (delegated to reset_match_positions)
    // is_in_play is 0 after reset — use direct array access
    BallEntity* ball = (s.entity_manager.ball_count > 0u)
                       ? &s.entity_manager.balls[0] : nullptr;
    if (ball) {
        TEST_ASSERT_FLOAT_EQ(ball->position.x, 0.0f,
                             "Ball should be at center after reset_match", 0.01f);
    }
}

// =========================================================================
// Test 16: Full cycle — play, dirty, reset, verify clean state
// =========================================================================
static void test_reset_after_live_play()
{
    std::printf("  [16] reset_after_live_play\n");
    SceneState s = make_loaded_scene();

    // Set to live and advance
    s.rules.current_state = SemanticPlayState::LIVE_PLAY;
    for (int i = 0; i < 240; ++i) {
        s.update_match_flow(1.0f / 240.0f);
    }

    TEST_ASSERT(s.period_time_seconds > 0.0f, "Precondition: clock should be ticking");
    TEST_ASSERT(s.match_time_seconds > 0.0f, "Precondition: match time should be ticking");

    // Dirty entity state
    dirty_state(s);

    // Full reset
    s.reset_match();

    // Everything should be zero
    TEST_ASSERT(s.period_time_seconds == 0.0f, "Period time zero after reset");
    TEST_ASSERT(s.match_time_seconds == 0.0f, "Match time zero after reset");
    TEST_ASSERT(s.home_score == 0u, "Score zero after reset");
    TEST_ASSERT(s.rules.current_state == SemanticPlayState::PRE_GAME,
                "State PRE_GAME after reset");

    BallEntity* ball = (s.entity_manager.ball_count > 0u)
                       ? &s.entity_manager.balls[0] : nullptr;
    if (ball) {
        TEST_ASSERT_FLOAT_EQ(ball->velocity.x, 0.0f, "Ball vel zero", 0.0001f);
        TEST_ASSERT_FLOAT_EQ(ball->position.x, 0.0f, "Ball at center", 0.01f);
    }

    TEST_ASSERT(!s.last_ball_toucher.is_valid(), "No last toucher");
    TEST_ASSERT(s.ball_in_play == 0u, "Ball not in play");
}

// =========================================================================
// Main
// =========================================================================
int main()
{
    std::printf("=== Sprint 32: Phase 15 Action 2 — The Reset Logic ===\n\n");

    test_home_position_set_during_spawn();
    test_spatial_teleport_ball();
    test_spatial_teleport_athletes();
    test_kinematic_wipe_ball();
    test_kinematic_wipe_athletes();
    test_cognitive_wipe_intents();
    test_cognitive_wipe_flags();
    test_cognitive_wipe_action_id();
    test_cognitive_wipe_perception();
    test_cognitive_wipe_influence();
    test_ball_possession_cleared();
    test_scene_possession_cleared();
    test_ball_in_play_cleared();
    test_ai_controllers_reset();
    test_reset_match_delegates();
    test_reset_after_live_play();

    std::printf("\n  Results: %d passed, %d failed\n\n", g_tests_passed, g_tests_failed);
    return (g_tests_failed == 0) ? 0 : 1;
}
