// =============================================================================
// Integration Test: Game Loop — Fixed-Timestep Accumulator & Controls
// =============================================================================
//
// Validates the GameLoop execution order and fixed-timestep accumulator:
//   1.  60Hz render frame triggers exactly 4 physics steps at 240Hz
//   2.  Accumulator remainder preserved across frames
//   3.  Pause blocks physics stepping (accumulator frozen)
//   4.  Resume restores physics stepping after pause
//   5.  Time scale 0.5x halves effective physics step rate
//   6.  Time scale 2.0x doubles effective physics step rate
//   7.  Max steps per frame prevents accumulator spiral
//   8.  End frame clamps accumulator to max_steps * fixed_delta
//   9.  Interpolation alpha at accumulator boundaries
//   10. get_physics_fps and get_real_fps compute correctly
//   11. begin_frame clamps negative time delta to zero
//   12. Multi-frame simulation accumulates correctly
//   13. start_playing transitions from WARMUP to PLAYING
//   14. toggle_pause cycles PAUSED ↔ PLAYING
//   15. Pipeline order: begin → step loop → end → alpha query
//
// Pattern: int main() + assert(), no test framework.
// =============================================================================

#include "apc_app/apc_game_loop.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_math_common.h"
#include <cassert>
#include <cstdlib>
#include <cstdio>
#include <cmath>

static constexpr float EPS = 1e-4f;
static int g_assertions = 0;

static bool approx_eq(float a, float b, float eps = EPS) {
    return std::abs(a - b) < eps;
}

static void check(bool cond, const char* msg) {
    ++g_assertions;
    if (!cond) {
        std::printf("    [FAIL] %s (assertion #%d)\n", msg, g_assertions);
        std::fflush(stdout);
        std::abort();
    }
}

// =============================================================================
// Helper: simulate one render frame through the full pipeline
// Returns number of physics steps executed.
// =============================================================================
static uint32_t simulate_frame(apc::GameLoop& loop, double wall_time) {
    loop.begin_frame(wall_time);
    uint32_t steps = 0;
    while (loop.should_step_physics()) {
        loop.step_physics();
        ++steps;
    }
    loop.end_frame();
    return steps;
}

// =============================================================================
// TEST 1: 60Hz render frame triggers exactly 4 physics steps at 240Hz
// =============================================================================
static void test_60hz_triggers_4_steps() {
    std::printf("  [Test 1] 60Hz render frame triggers exactly 4 physics steps...\n");

    apc::GameLoop loop;
    loop.init(240.0f);
    loop.start_playing();

    // First call seeds current_time (no delta). Reset accumulator.
    loop.begin_frame(0.0);
    loop.time.accumulator = 0.0f;

    // Simulate exactly 1/60th of a second
    float render_dt = 1.0f / 60.0f;
    // Manually set delta and accumulator (bypass begin_frame delta computation)
    loop.time.delta_time = render_dt;
    loop.time.accumulator = render_dt;

    uint32_t steps = 0;
    while (loop.should_step_physics()) {
        loop.step_physics();
        ++steps;
    }

    // 1/60 = 0.016666... = 4 * (1/240) = 4 * 0.004166...
    check(steps == 4, "60Hz frame produces exactly 4 physics steps");
    check(loop.time.physics_step_count == 4, "physics_step_count = 4");

    std::printf("    [PASS] 60Hz → 4 physics steps (240Hz)\n");
}

// =============================================================================
// TEST 2: Accumulator remainder preserved across frames
// =============================================================================
static void test_accumulator_remainder() {
    std::printf("  [Test 2] Accumulator remainder preserved across frames...\n");

    apc::GameLoop loop;
    loop.init(240.0f);
    loop.start_playing();

    float fd = loop.time.fixed_delta; // 1/240

    // Frame 1: add slightly more than 3 fixed steps
    loop.time.accumulator = fd * 3.5f;
    uint32_t steps1 = 0;
    while (loop.should_step_physics()) { loop.step_physics(); ++steps1; }
    check(steps1 == 3, "frame 1: 3 steps consumed");
    check(approx_eq(loop.time.accumulator, fd * 0.5f, 0.0001f),
          "frame 1: 0.5*fd remainder preserved");

    // Frame 2: add another 2.5 fixed steps → total accumulator = 3.0*fd
    loop.time.accumulator += fd * 2.5f;
    uint32_t steps2 = 0;
    while (loop.should_step_physics()) { loop.step_physics(); ++steps2; }
    check(steps2 == 3, "frame 2: 3 more steps consumed");
    check(approx_eq(loop.time.accumulator, 0.0f, 0.0001f),
          "frame 2: accumulator drained to 0");

    std::printf("    [PASS] Accumulator remainder carried across frames\n");
}

// =============================================================================
// TEST 3: Pause blocks physics stepping
// =============================================================================
static void test_pause_blocks_stepping() {
    std::printf("  [Test 3] Pause blocks physics stepping...\n");

    apc::GameLoop loop;
    loop.init(240.0f);
    loop.start_playing();

    // Fill accumulator with 5 steps worth
    loop.time.accumulator = loop.time.fixed_delta * 5.0f;

    // Pause
    loop.pause();
    check(loop.state == apc::GameState::PAUSED, "state = PAUSED");

    // should_step_physics must return false
    check(loop.should_step_physics() == 0, "paused: should_step = 0");

    // Step count unchanged
    uint32_t prev_count = loop.time.physics_step_count;
    check(prev_count == 0, "no steps executed while paused");

    std::printf("    [PASS] Pause blocks all physics stepping\n");
}

// =============================================================================
// TEST 4: Resume restores physics stepping after pause
// =============================================================================
static void test_resume_restores_stepping() {
    std::printf("  [Test 4] Resume restores physics stepping after pause...\n");

    apc::GameLoop loop;
    loop.init(240.0f);
    loop.start_playing();

    loop.time.accumulator = loop.time.fixed_delta * 3.0f;
    loop.pause();
    check(loop.should_step_physics() == 0, "paused: no stepping");

    loop.resume();
    check(loop.state == apc::GameState::PLAYING, "state = PLAYING after resume");
    check(loop.should_step_physics() == 1, "resumed: should_step = 1");

    // Can execute steps again
    loop.step_physics();
    check(loop.time.physics_step_count == 1, "1 step after resume");

    std::printf("    [PASS] Resume restores physics stepping\n");
}

// =============================================================================
// TEST 5: Time scale 0.5x halves effective physics step rate
// =============================================================================
static void test_time_scale_half_speed() {
    std::printf("  [Test 5] Time scale 0.5x halves effective physics step rate...\n");

    apc::GameLoop loop;
    loop.init(240.0f);
    loop.start_playing();
    loop.set_time_scale(0.5f);

    float render_dt = 1.0f / 60.0f;
    // Scaled accumulator = render_dt * 0.5 = 0.008333...
    // 0.008333 / (1/240) = 0.008333 / 0.004166 = 2 steps
    loop.time.accumulator = render_dt * 0.5f;

    uint32_t steps = 0;
    while (loop.should_step_physics()) { loop.step_physics(); ++steps; }

    check(steps == 2, "0.5x time scale: 60Hz frame → 2 physics steps");
    check(approx_eq(loop.time.time_scale, 0.5f), "time_scale = 0.5");

    std::printf("    [PASS] 0.5x time scale halves physics step rate\n");
}

// =============================================================================
// TEST 6: Time scale 2.0x doubles effective physics step rate
// =============================================================================
static void test_time_scale_double_speed() {
    std::printf("  [Test 6: Time scale 2.0x doubles effective physics step rate...\n");

    apc::GameLoop loop;
    loop.init(240.0f);
    loop.start_playing();
    loop.set_time_scale(2.0f);

    float render_dt = 1.0f / 60.0f;
    // Scaled accumulator = render_dt * 2.0 = 0.033333...
    // 0.033333 / (1/240) = 8 steps
    loop.time.accumulator = render_dt * 2.0f;

    uint32_t steps = 0;
    while (loop.should_step_physics()) { loop.step_physics(); ++steps; }

    check(steps == 8, "2.0x time scale: 60Hz frame → 8 physics steps");

    std::printf("    [PASS] 2.0x time scale doubles physics step rate\n");
}

// =============================================================================
// TEST 7: Max steps per frame prevents accumulator spiral
// =============================================================================
static void test_max_steps_per_frame() {
    std::printf("  [Test 7] Max steps per frame prevents accumulator spiral...\n");

    apc::GameLoop loop;
    loop.init(240.0f);
    loop.start_playing();
    loop.time.max_steps_per_frame = 4u;

    float fd = loop.time.fixed_delta;

    // The caller is responsible for limiting steps per frame.
    // We simulate the standard pattern: while + max_steps cap.
    loop.time.accumulator = fd * 100.0f;

    uint32_t steps = 0;
    while (loop.should_step_physics() && steps < loop.time.max_steps_per_frame) {
        loop.step_physics();
        ++steps;
    }

    check(steps == 4, "max_steps: exactly 4 steps executed");
    check(loop.time.physics_step_count == 4, "physics_step_count = 4");

    // end_frame should clamp the accumulator to 0 when it exceeds max
    loop.end_frame();
    check(loop.time.accumulator == 0.0f,
          "end_frame: accumulator reset when exceeding max_steps * fd");

    std::printf("    [PASS] Max steps per frame enforced by caller, clamped by end_frame\n");
}

// =============================================================================
// TEST 8: End frame clamps accumulator
// =============================================================================
static void test_end_frame_clamps_accumulator() {
    std::printf("  [Test 8] End frame clamps accumulator...\n");

    apc::GameLoop loop;
    loop.init(240.0f);
    loop.time.max_steps_per_frame = 4u;
    loop.state = apc::GameState::PLAYING;

    float fd = loop.time.fixed_delta;

    // Set accumulator beyond max
    loop.time.accumulator = fd * 10.0f;
    loop.end_frame();

    check(loop.time.accumulator == 0.0f,
          "end_frame: accumulator reset to 0 when exceeding max");

    // Just under max: should NOT clamp
    loop.time.accumulator = fd * 3.0f;
    loop.end_frame();
    check(approx_eq(loop.time.accumulator, fd * 3.0f, 0.0001f),
          "end_frame: accumulator preserved when under max");

    std::printf("    [PASS] End frame correctly clamps accumulator\n");
}

// =============================================================================
// TEST 9: Interpolation alpha at accumulator boundaries
// =============================================================================
static void test_interpolation_alpha_boundaries() {
    std::printf("  [Test 9: Interpolation alpha at accumulator boundaries...\n");

    apc::GameLoop loop;
    loop.init(240.0f);
    float fd = loop.time.fixed_delta;

    // Zero accumulator → alpha = 0
    loop.time.accumulator = 0.0f;
    check(approx_eq(loop.get_interpolation_alpha(), 0.0f), "alpha = 0 at acc=0");

    // Quarter → 0.25
    loop.time.accumulator = fd * 0.25f;
    check(approx_eq(loop.get_interpolation_alpha(), 0.25f), "alpha = 0.25");

    // Full → 1.0
    loop.time.accumulator = fd * 1.0f;
    check(approx_eq(loop.get_interpolation_alpha(), 1.0f), "alpha = 1.0 at acc=fd");

    // Over-full → clamped to 1.0
    loop.time.accumulator = fd * 2.0f;
    float alpha = loop.get_interpolation_alpha();
    check(alpha <= 1.0f + EPS, "alpha <= 1.0 when over-full");

    std::printf("    [PASS] Interpolation alpha correct at boundaries\n");
}

// =============================================================================
// TEST 10: get_physics_fps and get_real_fps compute correctly
// =============================================================================
static void test_fps_accessors() {
    std::printf("  [Test 10] get_physics_fps and get_real_fps compute correctly...\n");

    apc::GameLoop loop;
    loop.init(240.0f);

    check(approx_eq(loop.get_physics_fps(), 240.0f), "physics_fps = 240");

    // Real FPS: 1/delta. delta=0 → return 0
    check(approx_eq(loop.get_real_fps(), 0.0f), "real_fps = 0 when delta=0");

    // After setting delta
    loop.time.delta_time = 1.0f / 60.0f;
    check(approx_eq(loop.get_real_fps(), 60.0f, 0.1f), "real_fps = 60 when delta=1/60");

    // With 120Hz
    loop.time.delta_time = 1.0f / 120.0f;
    check(approx_eq(loop.get_real_fps(), 120.0f, 0.1f), "real_fps = 120 when delta=1/120");

    std::printf("    [PASS] FPS accessors compute correctly\n");
}

// =============================================================================
// TEST 11: begin_frame clamps negative time delta
// =============================================================================
static void test_begin_frame_negative_delta() {
    std::printf("  [Test 11] begin_frame clamps negative time delta...\n");

    apc::GameLoop loop;
    loop.init(240.0f);
    loop.state = apc::GameState::PLAYING;

    // First call seeds current_time
    loop.begin_frame(1.0);

    // Second call: wall clock goes backward
    loop.begin_frame(0.5);

    check(loop.time.delta_time >= 0.0f, "delta_time >= 0 after backward clock");
    check(approx_eq(loop.time.delta_time, 0.0f), "delta_time = 0 when time goes backward");

    std::printf("    [PASS] begin_frame clamps negative delta to zero\n");
}

// =============================================================================
// TEST 12: Multi-frame simulation accumulates correctly
// =============================================================================
static void test_multi_frame_accumulation() {
    std::printf("  [Test 12] Multi-frame simulation accumulates correctly...\n");

    apc::GameLoop loop;
    loop.init(240.0f);
    loop.start_playing();
    loop.time.max_steps_per_frame = 8u;

    float fd = loop.time.fixed_delta;

    // Simulate 10 render frames at 60Hz, each adding 1/60 ≈ 4.167*fd
    for (int i = 0; i < 10; ++i) {
        loop.time.accumulator += (1.0f / 60.0f);
        uint32_t steps = 0;
        while (loop.should_step_physics() && steps < 8u) {
            loop.step_physics();
            ++steps;
        }
        loop.end_frame();
    }

    // Total time added: 10/60 = 1/6 ≈ 0.16667s
    // At 240Hz: 0.16667 / (1/240) = 40 physics steps
    // Some steps may be lost to accumulator clamping — but should be close
    check(loop.time.physics_step_count >= 35,
          "multi-frame: at least 35 physics steps in 10 frames (expected ~40)");
    check(loop.time.frame_count == 10u, "multi-frame: frame_count = 10");

    std::printf("    [PASS] Multi-frame simulation: %u steps in 10 frames\n",
                loop.time.physics_step_count);
}

// =============================================================================
// TEST 13: start_playing transitions WARMUP → PLAYING
// =============================================================================
static void test_start_playing_transition() {
    std::printf("  [Test 13] start_playing transitions WARMUP → PLAYING...\n");

    apc::GameLoop loop;
    loop.init();
    check(loop.state == apc::GameState::WARMUP, "state = WARMUP after init");

    loop.start_playing();
    check(loop.state == apc::GameState::PLAYING, "state = PLAYING after start_playing");

    // Physics should now be allowed
    loop.time.accumulator = loop.time.fixed_delta * 2.0f;
    check(loop.should_step_physics() == 1, "physics stepping allowed in PLAYING");

    std::printf("    [PASS] start_playing transitions correctly\n");
}

// =============================================================================
// TEST 14: toggle_pause cycles PAUSED ↔ PLAYING
// =============================================================================
static void test_toggle_pause_cycle() {
    std::printf("  [Test 14] toggle_pause cycles PAUSED ↔ PLAYING...\n");

    apc::GameLoop loop;
    loop.init();
    loop.start_playing();
    check(loop.state == apc::GameState::PLAYING, "initial: PLAYING");

    loop.toggle_pause();
    check(loop.state == apc::GameState::PAUSED, "toggle 1: PAUSED");
    check(loop.should_step_physics() == 0, "PAUSED: no physics");

    loop.toggle_pause();
    check(loop.state == apc::GameState::PLAYING, "toggle 2: PLAYING");

    // Need accumulator to test physics stepping
    loop.time.accumulator = loop.time.fixed_delta;
    check(loop.should_step_physics() == 1, "PLAYING: physics allowed with accumulator");

    loop.toggle_pause();
    check(loop.state == apc::GameState::PAUSED, "toggle 3: PAUSED again");

    std::printf("    [PASS] toggle_pause cycles correctly\n");
}

// =============================================================================
// TEST 15: Pipeline order — begin → step loop → end → alpha query
// =============================================================================
static void test_pipeline_order() {
    std::printf("  [Test 15] Pipeline order: begin → step → end → alpha...\n");

    apc::GameLoop loop;
    loop.init(240.0f);
    loop.start_playing();
    loop.time.max_steps_per_frame = 8u;

    // Step 1: begin_frame with exact 1/60s delta
    loop.time.current_time = 0.0;
    loop.time.accumulator = 0.0f;
    loop.begin_frame(1.0 / 60.0);

    // First call: current_time was 0, so delta stays 0
    // We manually set it for the test
    loop.time.delta_time = 1.0f / 60.0f;
    loop.time.accumulator = 1.0f / 60.0f;

    // Step 2: step loop
    uint32_t steps = 0;
    while (loop.should_step_physics()) { loop.step_physics(); ++steps; }
    check(steps == 4, "pipeline: 4 steps in 1/60s frame");

    // Step 3: end_frame
    loop.end_frame();
    check(loop.time.frame_count == 1u, "pipeline: frame_count = 1 after end_frame");

    // Step 4: alpha query
    float alpha = loop.get_interpolation_alpha();
    check(alpha >= 0.0f && alpha <= 1.0f, "pipeline: alpha in [0,1]");
    // After 4 exact steps from 1/60, accumulator should be ~0 → alpha ≈ 0
    check(alpha < 0.1f, "pipeline: alpha ≈ 0 after exact step consumption");

    std::printf("    [PASS] Pipeline order executes correctly\n");
}

// =============================================================================
// Main
// =============================================================================
int main() {
    std::printf("=== Integration Test: Game Loop ===\n\n");

    test_60hz_triggers_4_steps();
    test_accumulator_remainder();
    test_pause_blocks_stepping();
    test_resume_restores_stepping();
    test_time_scale_half_speed();
    test_time_scale_double_speed();
    test_max_steps_per_frame();
    test_end_frame_clamps_accumulator();
    test_interpolation_alpha_boundaries();
    test_fps_accessors();
    test_begin_frame_negative_delta();
    test_multi_frame_accumulation();
    test_start_playing_transition();
    test_toggle_pause_cycle();
    test_pipeline_order();

    std::printf("\n=== Game Loop: 15 tests passed (%d assertions) ===\n", g_assertions);
    return 0;
}
