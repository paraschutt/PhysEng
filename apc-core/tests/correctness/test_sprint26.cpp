// =============================================================================
// Sprint 26 Tests — Game Loop, Match Configuration & Scene Management
// =============================================================================
//
// Tests for the APC Sprint 26 headers (apc_game_loop.h, apc_scene_manager.h):
//   1.  TimeState defaults (current_time=0, delta_time=0, frame_count=0)
//   2.  TimeState fixed_delta = 1/240
//   3.  FixedTimestepConfig defaults (physics_hz=240, max_frame_time=0.1)
//   4.  GameState enum values (UNINITIALIZED=0, LOADING=1, PLAYING=3, etc.)
//   5.  GameLoop defaults (state=UNINITIALIZED)
//   6.  GameLoop.init() sets up timing
//   7.  GameLoop.begin_frame() computes delta_time
//   8.  GameLoop.should_step_physics() with enough accumulator
//   9.  GameLoop.should_step_physics() with insufficient accumulator
//   10. GameLoop.step_physics() consumes fixed_delta
//   11. GameLoop.get_interpolation_alpha() returns 0-1
//   12. GameLoop.set_time_scale() changes scale
//   13. GameLoop.pause() sets state to PAUSED
//   14. GameLoop.resume() sets state back to PLAYING
//   15. SportType enum values
//   16. MatchConfig defaults
//   17. MatchConfig sport presets (soccer_match, basketball_match)
//
// Pattern: int main() + assert(), no test framework.
// =============================================================================

#include "apc_app/apc_game_loop.h"
#include "apc_app/apc_scene_manager.h"
#include "apc_math/apc_vec3.h"
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
// TEST 1: TimeState defaults
// =============================================================================
static int test_time_state_defaults() {
    std::printf("  [Test 1] TimeState defaults (current_time=0, delta_time=0, frame_count=0)...\n");

    apc::TimeState ts;

    assert(ts.current_time == 0.0 && "current_time = 0");
    assert(approx_eq(ts.delta_time, 0.0f) && "delta_time = 0");
    assert(approx_eq(ts.accumulator, 0.0f) && "accumulator = 0");
    assert(approx_eq(ts.time_scale, 1.0f) && "time_scale = 1.0");
    assert(ts.frame_count == 0u && "frame_count = 0");
    assert(ts.physics_step_count == 0u && "physics_step_count = 0");
    assert(ts.max_steps_per_frame == apc::MAX_STEPS_PER_FRAME && "max_steps = MAX_STEPS_PER_FRAME");

    std::printf("    [PASS] TimeState defaults verified\n");
    return 0;
}

// =============================================================================
// TEST 2: TimeState fixed_delta = 1/240
// =============================================================================
static int test_time_state_fixed_delta() {
    std::printf("  [Test 2] TimeState fixed_delta = 1/240...\n");

    apc::TimeState ts;
    float expected = 1.0f / 240.0f;

    assert(approx_eq(ts.fixed_delta, expected) && "fixed_delta = 1/240");

    // Verify PHYSICS_DT constant
    assert(approx_eq(apc::PHYSICS_DT, expected) && "PHYSICS_DT = 1/240");
    assert(approx_eq(apc::PHYSICS_HZ, 240.0f) && "PHYSICS_HZ = 240");

    std::printf("    [PASS] TimeState fixed_delta = 1/240 verified\n");
    return 0;
}

// =============================================================================
// TEST 3: FixedTimestepConfig defaults
// =============================================================================
static int test_fixed_timestep_config_defaults() {
    std::printf("  [Test 3] FixedTimestepConfig defaults (physics_hz=240, max_frame_time=0.1)...\n");

    apc::FixedTimestepConfig cfg;

    assert(approx_eq(cfg.physics_hz, 240.0f) && "physics_hz = 240");
    assert(approx_eq(cfg.max_frame_time, 0.1f) && "max_frame_time = 0.1");
    assert(approx_eq(cfg.slow_motion_min, 0.1f) && "slow_motion_min = 0.1");
    assert(approx_eq(cfg.slow_motion_max, 2.0f) && "slow_motion_max = 2.0");
    assert(cfg.interpolate_render == 1 && "interpolate_render = 1");

    std::printf("    [PASS] FixedTimestepConfig defaults verified\n");
    return 0;
}

// =============================================================================
// TEST 4: GameState enum values
// =============================================================================
static int test_game_state_enum() {
    std::printf("  [Test 4] GameState enum values...\n");

    using GS = apc::GameState;

    assert(static_cast<uint8_t>(GS::UNINITIALIZED) == 0);
    assert(static_cast<uint8_t>(GS::LOADING)       == 1);
    assert(static_cast<uint8_t>(GS::WARMUP)        == 2);
    assert(static_cast<uint8_t>(GS::PLAYING)       == 3);
    assert(static_cast<uint8_t>(GS::PAUSED)        == 4);
    assert(static_cast<uint8_t>(GS::GOAL_SCORED)   == 5);
    assert(static_cast<uint8_t>(GS::HALF_TIME)     == 6);
    assert(static_cast<uint8_t>(GS::FULL_TIME)     == 7);
    assert(static_cast<uint8_t>(GS::REPLAY)        == 8);
    assert(static_cast<uint8_t>(GS::MENU)          == 9);

    std::printf("    [PASS] GameState enum values verified\n");
    return 0;
}

// =============================================================================
// TEST 5: GameLoop defaults
// =============================================================================
static int test_game_loop_defaults() {
    std::printf("  [Test 5] GameLoop defaults (state=UNINITIALIZED)...\n");

    apc::GameLoop loop;

    assert(loop.state == apc::GameState::UNINITIALIZED && "state = UNINITIALIZED");

    // TimeState should also be at defaults
    assert(loop.time.current_time == 0.0 && "time.current_time = 0");
    assert(approx_eq(loop.time.delta_time, 0.0f) && "time.delta_time = 0");
    assert(loop.time.frame_count == 0u && "time.frame_count = 0");

    // Config defaults
    assert(approx_eq(loop.config.physics_hz, 240.0f) && "config.physics_hz = 240");

    std::printf("    [PASS] GameLoop defaults verified\n");
    return 0;
}

// =============================================================================
// TEST 6: GameLoop.init() sets up timing
// =============================================================================
static int test_game_loop_init() {
    std::printf("  [Test 6] GameLoop.init() sets up timing...\n");

    apc::GameLoop loop;
    assert(loop.state == apc::GameState::UNINITIALIZED);

    loop.init();
    assert(loop.state == apc::GameState::WARMUP && "state = WARMUP after init");

    // Timing should be reset
    assert(loop.time.current_time == 0.0 && "current_time = 0 after init");
    assert(approx_eq(loop.time.delta_time, 0.0f) && "delta_time = 0 after init");
    assert(approx_eq(loop.time.accumulator, 0.0f) && "accumulator = 0 after init");
    assert(approx_eq(loop.time.time_scale, 1.0f) && "time_scale = 1.0 after init");
    assert(loop.time.frame_count == 0u && "frame_count = 0 after init");
    assert(loop.time.physics_step_count == 0u && "physics_step_count = 0 after init");

    // Config should be set
    assert(approx_eq(loop.config.physics_hz, 240.0f) && "physics_hz = 240 after init");
    assert(approx_eq(loop.config.max_frame_time, 0.1f) && "max_frame_time = 0.1 after init");
    assert(approx_eq(loop.config.slow_motion_min, 0.1f) && "slow_motion_min = 0.1 after init");
    assert(approx_eq(loop.config.slow_motion_max, 2.0f) && "slow_motion_max = 2.0 after init");
    assert(loop.config.interpolate_render == 1 && "interpolate_render = 1 after init");

    // Init with custom physics_hz
    loop.init(60.0f);
    assert(approx_eq(loop.time.fixed_delta, 1.0f / 60.0f) && "fixed_delta = 1/60 for 60Hz");
    assert(approx_eq(loop.config.physics_hz, 60.0f) && "config.physics_hz = 60");

    std::printf("    [PASS] GameLoop.init() verified\n");
    return 0;
}

// =============================================================================
// TEST 7: GameLoop.begin_frame() computes delta_time
// =============================================================================
static int test_game_loop_begin_frame() {
    std::printf("  [Test 7] GameLoop.begin_frame() computes delta_time...\n");

    apc::GameLoop loop;
    loop.init();
    loop.state = apc::GameState::PLAYING;

    // First frame: current_time=0, so delta should remain 0
    loop.begin_frame(0.016);
    assert(loop.time.current_time == 0.016 && "current_time = 0.016");
    // delta_time stays 0 because current_time was 0 (no previous frame)
    assert(approx_eq(loop.time.delta_time, 0.0f) && "delta_time = 0 on first frame");

    // Second frame: should compute delta
    loop.begin_frame(0.033);
    assert(loop.time.current_time == 0.033 && "current_time = 0.033");
    assert(approx_eq(loop.time.delta_time, 0.017f, 0.001f) && "delta_time ~ 0.017");

    // Delta should be clamped to max_frame_time
    loop.begin_frame(0.5); // Big jump
    assert(approx_eq(loop.time.delta_time, 0.1f) && "delta_time clamped to 0.1");

    // Negative delta should be clamped to 0
    loop.begin_frame(0.04); // Going backward
    assert(loop.time.delta_time >= 0.0f && "delta_time >= 0 after backward frame");

    std::printf("    [PASS] GameLoop.begin_frame() verified\n");
    return 0;
}

// =============================================================================
// TEST 8: GameLoop.should_step_physics() with enough accumulator
// =============================================================================
static int test_should_step_physics_enough() {
    std::printf("  [Test 8] GameLoop.should_step_physics() with enough accumulator...\n");

    apc::GameLoop loop;
    loop.init();
    loop.state = apc::GameState::PLAYING;

    // Manually set accumulator to enough for one step
    loop.time.accumulator = loop.time.fixed_delta + 0.001f;

    assert(loop.should_step_physics() == 1 && "should_step = 1 with enough accumulator");

    std::printf("    [PASS] should_step_physics() returns true with enough accumulator\n");
    return 0;
}

// =============================================================================
// TEST 9: GameLoop.should_step_physics() with insufficient accumulator
// =============================================================================
static int test_should_step_physics_insufficient() {
    std::printf("  [Test 9] GameLoop.should_step_physics() with insufficient accumulator...\n");

    apc::GameLoop loop;
    loop.init();
    loop.state = apc::GameState::PLAYING;

    // Set accumulator to half of fixed_delta
    loop.time.accumulator = loop.time.fixed_delta * 0.5f;

    assert(loop.should_step_physics() == 0 && "should_step = 0 with insufficient accumulator");

    // Should also return false when not in PLAYING state
    loop.time.accumulator = loop.time.fixed_delta + 0.01f;
    loop.state = apc::GameState::PAUSED;
    assert(loop.should_step_physics() == 0 && "should_step = 0 when PAUSED");

    loop.state = apc::GameState::WARMUP;
    assert(loop.should_step_physics() == 0 && "should_step = 0 when WARMUP");

    std::printf("    [PASS] should_step_physics() returns false when insufficient\n");
    return 0;
}

// =============================================================================
// TEST 10: GameLoop.step_physics() consumes fixed_delta
// =============================================================================
static int test_step_physics_consumes() {
    std::printf("  [Test 10] GameLoop.step_physics() consumes fixed_delta...\n");

    apc::GameLoop loop;
    loop.init();
    loop.state = apc::GameState::PLAYING;

    float fd = loop.time.fixed_delta;

    // Accumulate enough for 3 steps
    loop.time.accumulator = fd * 3.0f;
    assert(loop.time.physics_step_count == 0u);

    loop.step_physics();
    assert(loop.time.physics_step_count == 1u && "step_count = 1 after first step");
    assert(approx_eq(loop.time.accumulator, fd * 2.0f) && "accumulator = 2*fd after first step");

    loop.step_physics();
    assert(loop.time.physics_step_count == 2u && "step_count = 2");
    assert(approx_eq(loop.time.accumulator, fd * 1.0f) && "accumulator = fd after second step");

    loop.step_physics();
    assert(loop.time.physics_step_count == 3u && "step_count = 3");
    assert(approx_eq(loop.time.accumulator, 0.0f) && "accumulator = 0 after third step");

    std::printf("    [PASS] step_physics() consumes fixed_delta\n");
    return 0;
}

// =============================================================================
// TEST 11: GameLoop.get_interpolation_alpha() returns 0-1
// =============================================================================
static int test_interpolation_alpha() {
    std::printf("  [Test 11] GameLoop.get_interpolation_alpha() returns 0-1...\n");

    apc::GameLoop loop;
    loop.init();

    float fd = loop.time.fixed_delta;

    // No accumulator: alpha = 0
    loop.time.accumulator = 0.0f;
    float alpha0 = loop.get_interpolation_alpha();
    assert(approx_eq(alpha0, 0.0f) && "alpha = 0 when accumulator = 0");

    // Half accumulator: alpha ~= 0.5
    loop.time.accumulator = fd * 0.5f;
    float alpha_half = loop.get_interpolation_alpha();
    assert(approx_eq(alpha_half, 0.5f) && "alpha ~= 0.5 at half accumulator");

    // Full accumulator: alpha = 1.0
    loop.time.accumulator = fd;
    float alpha_full = loop.get_interpolation_alpha();
    assert(approx_eq(alpha_full, 1.0f) && "alpha = 1.0 at full accumulator");

    // Alpha should be clamped to [0, 1]
    loop.time.accumulator = fd * 1.5f;
    float alpha_clamped = loop.get_interpolation_alpha();
    assert(alpha_clamped <= 1.0f + EPS && "alpha <= 1.0 when accumulator > fd");
    assert(alpha_clamped >= 0.0f - EPS && "alpha >= 0.0");

    // Negative accumulator: alpha = 0
    loop.time.accumulator = -fd;
    float alpha_neg = loop.get_interpolation_alpha();
    assert(approx_eq(alpha_neg, 0.0f) && "alpha = 0 with negative accumulator");

    std::printf("    [PASS] get_interpolation_alpha() returns 0-1\n");
    return 0;
}

// =============================================================================
// TEST 12: GameLoop.set_time_scale() changes scale
// =============================================================================
static int test_set_time_scale() {
    std::printf("  [Test 12] GameLoop.set_time_scale() changes scale...\n");

    apc::GameLoop loop;
    loop.init();

    assert(approx_eq(loop.time.time_scale, 1.0f) && "default time_scale = 1.0");

    // Set to slow motion
    loop.set_time_scale(0.5f);
    assert(approx_eq(loop.time.time_scale, 0.5f) && "time_scale = 0.5 after set");

    // Set to fast forward
    loop.set_time_scale(1.5f);
    assert(approx_eq(loop.time.time_scale, 1.5f) && "time_scale = 1.5 after set");

    // Should clamp to slow_motion_min
    loop.set_time_scale(0.01f);
    assert(approx_eq(loop.time.time_scale, loop.config.slow_motion_min) && "clamped to min");

    // Should clamp to slow_motion_max
    loop.set_time_scale(10.0f);
    assert(approx_eq(loop.time.time_scale, loop.config.slow_motion_max) && "clamped to max");

    // Verify accumulator is scaled by time_scale during begin_frame
    loop.state = apc::GameState::PLAYING;
    loop.time.current_time = 0.0;
    loop.time.accumulator = 0.0f;
    loop.time.time_scale = 0.5f;
    loop.begin_frame(0.02); // 20ms wall clock
    float expected_acc = 0.01f; // 20ms * 0.5 = 10ms
    assert(approx_eq(loop.time.accumulator, expected_acc, 0.001f) && "accumulator scaled by 0.5");

    std::printf("    [PASS] set_time_scale() verified\n");
    return 0;
}

// =============================================================================
// TEST 13: GameLoop.pause() sets state to PAUSED
// =============================================================================
static int test_game_loop_pause() {
    std::printf("  [Test 13] GameLoop.pause() sets state to PAUSED...\n");

    apc::GameLoop loop;
    loop.init();
    assert(loop.state == apc::GameState::WARMUP);

    loop.state = apc::GameState::PLAYING;
    loop.pause();
    assert(loop.state == apc::GameState::PAUSED && "state = PAUSED after pause()");

    // Pause from any state should set to PAUSED
    loop.state = apc::GameState::GOAL_SCORED;
    loop.pause();
    assert(loop.state == apc::GameState::PAUSED && "state = PAUSED from GOAL_SCORED");

    std::printf("    [PASS] pause() sets state to PAUSED\n");
    return 0;
}

// =============================================================================
// TEST 14: GameLoop.resume() sets state back to PLAYING
// =============================================================================
static int test_game_loop_resume() {
    std::printf("  [Test 14] GameLoop.resume() sets state back to PLAYING...\n");

    apc::GameLoop loop;
    loop.init();

    // Set to PAUSED first
    loop.state = apc::GameState::PAUSED;
    loop.resume();
    assert(loop.state == apc::GameState::PLAYING && "state = PLAYING after resume()");

    // Resume when not PAUSED should not change state
    loop.state = apc::GameState::WARMUP;
    loop.resume();
    assert(loop.state == apc::GameState::WARMUP && "state unchanged when not PAUSED");

    // Full pause/resume cycle
    loop.state = apc::GameState::PLAYING;
    loop.pause();
    assert(loop.state == apc::GameState::PAUSED);
    loop.resume();
    assert(loop.state == apc::GameState::PLAYING);

    // toggle_pause should also work
    loop.state = apc::GameState::PLAYING;
    loop.toggle_pause();
    assert(loop.state == apc::GameState::PAUSED && "toggle to PAUSED");
    loop.toggle_pause();
    assert(loop.state == apc::GameState::PLAYING && "toggle back to PLAYING");

    std::printf("    [PASS] resume() and toggle_pause() verified\n");
    return 0;
}

// =============================================================================
// TEST 15: SportType enum values
// =============================================================================
static int test_sport_type_enum() {
    std::printf("  [Test 15] SportType enum values...\n");

    assert(static_cast<uint8_t>(apc::SportType::SOCCER)            == 0);
    assert(static_cast<uint8_t>(apc::SportType::BASKETBALL)        == 1);
    assert(static_cast<uint8_t>(apc::SportType::AMERICAN_FOOTBALL)  == 2);
    assert(static_cast<uint8_t>(apc::SportType::RUGBY)             == 3);
    assert(static_cast<uint8_t>(apc::SportType::HOCKEY)            == 4);

    std::printf("    [PASS] SportType enum values verified\n");
    return 0;
}

// =============================================================================
// TEST 16: MatchConfig defaults
// =============================================================================
static int test_match_config_defaults() {
    std::printf("  [Test 16] MatchConfig defaults...\n");

    apc::MatchConfig cfg;

    assert(cfg.sport == apc::SportType::SOCCER && "sport = SOCCER");
    assert(approx_eq(cfg.match_duration_seconds, 5400.0f) && "duration = 5400 (90 min)");
    assert(cfg.halves == 2 && "halves = 2");
    assert(approx_eq(cfg.half_duration, 2700.0f) && "half_duration = 2700");
    assert(cfg.players_per_team == 11 && "players_per_team = 11");
    assert(cfg.subs_per_team == 3 && "subs_per_team = 3");
    assert(approx_eq(cfg.field_length, 105.0f) && "field_length = 105");
    assert(approx_eq(cfg.field_width, 68.0f) && "field_width = 68");
    assert(cfg.offside_enabled == 1 && "offside_enabled = 1");
    assert(cfg.var_enabled == 0 && "var_enabled = 0");
    assert(cfg.injuries_enabled == 0 && "injuries_enabled = 0");

    std::printf("    [PASS] MatchConfig defaults verified\n");
    return 0;
}

// =============================================================================
// TEST 17: MatchConfig sport presets
// =============================================================================
static int test_match_config_presets() {
    std::printf("  [Test 17] MatchConfig sport presets (soccer_match, basketball_match)...\n");

    // Soccer preset
    apc::MatchConfig soccer = apc::SceneState::soccer_match();

    assert(soccer.sport == apc::SportType::SOCCER && "soccer: sport = SOCCER");
    assert(approx_eq(soccer.match_duration_seconds, 5400.0f) && "soccer: duration = 5400");
    assert(soccer.halves == 2 && "soccer: halves = 2");
    assert(soccer.players_per_team == 11 && "soccer: players = 11");
    assert(approx_eq(soccer.field_length, 105.0f) && "soccer: field_length = 105");
    assert(approx_eq(soccer.field_width, 68.0f) && "soccer: field_width = 68");
    assert(soccer.offside_enabled == 1 && "soccer: offside = 1");
    assert(soccer.home_formation == apc::FormationType::FORMATION_4_4_2 && "soccer: home = 4-4-2");
    assert(soccer.away_formation == apc::FormationType::FORMATION_4_3_3 && "soccer: away = 4-3-3");

    // Basketball preset
    apc::MatchConfig basketball = apc::SceneState::basketball_match();

    assert(basketball.sport == apc::SportType::BASKETBALL && "basketball: sport = BASKETBALL");
    assert(approx_eq(basketball.match_duration_seconds, 2880.0f) && "basketball: duration = 2880");
    assert(basketball.halves == 4 && "basketball: halves = 4");
    assert(basketball.players_per_team == 5 && "basketball: players = 5");
    assert(approx_eq(basketball.field_length, 28.0f) && "basketball: field_length = 28");
    assert(approx_eq(basketball.field_width, 15.0f) && "basketball: field_width = 15");
    assert(basketball.offside_enabled == 0 && "basketball: offside = 0");

    std::printf("    [PASS] MatchConfig sport presets verified\n");
    return 0;
}

// =============================================================================
// main
// =============================================================================
int main() {
    std::printf("=== Sprint 26: Game Loop, Match Config & Scene Management ===\n\n");

    int result = 0;
    result |= test_time_state_defaults();
    result |= test_time_state_fixed_delta();
    result |= test_fixed_timestep_config_defaults();
    result |= test_game_state_enum();
    result |= test_game_loop_defaults();
    result |= test_game_loop_init();
    result |= test_game_loop_begin_frame();
    result |= test_should_step_physics_enough();
    result |= test_should_step_physics_insufficient();
    result |= test_step_physics_consumes();
    result |= test_interpolation_alpha();
    result |= test_set_time_scale();
    result |= test_game_loop_pause();
    result |= test_game_loop_resume();
    result |= test_sport_type_enum();
    result |= test_match_config_defaults();
    result |= test_match_config_presets();

    int total = 17;
    int passed = total - result;
    std::printf("\n=== Sprint 26: %d tests passed, %d failed ===\n", passed, result);
    return result;
}
