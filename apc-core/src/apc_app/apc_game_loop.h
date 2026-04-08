#pragma once
// =============================================================================
// apc_game_loop.h — Fixed-timestep game loop with accumulator
// =============================================================================
//
// Implements the main simulation loop:
//
//   - TimeState: timing state (current, delta, accumulator, scale, counters)
//   - FixedTimestepConfig: tunable parameters (physics Hz, max frame time)
//   - GameState: application lifecycle enumeration
//   - GameLoop: accumulator-based fixed-timestep update with pause/scale
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation
//   - Deterministic: fixed-order updates, fixed timestep physics
//   - C++17
//
// =============================================================================

#include "apc_math/apc_vec3.h"
#include "apc_math/apc_math_common.h"
#include <cstdint>
#include <cmath>

namespace apc {

// =============================================================================
// Constants
// =============================================================================
static constexpr float   PHYSICS_HZ           = 240.0f;
static constexpr float   PHYSICS_DT           = 1.0f / PHYSICS_HZ;
static constexpr uint32_t MAX_STEPS_PER_FRAME = 8u;

// =============================================================================
// TimeState — Timing state for the game loop
// =============================================================================
struct TimeState {
    double   current_time      = 0.0;    // Seconds since start
    float    delta_time        = 0.0f;   // Last frame time
    float    fixed_delta       = PHYSICS_DT;
    float    accumulator       = 0.0f;   // Time to simulate
    float    time_scale        = 1.0f;   // 1.0 = normal, 0.5 = slow-mo
    uint32_t frame_count        = 0u;
    uint32_t physics_step_count = 0u;
    uint32_t max_steps_per_frame = MAX_STEPS_PER_FRAME;
};

// =============================================================================
// FixedTimestepConfig — Tunable parameters for the game loop
// =============================================================================
struct FixedTimestepConfig {
    float   physics_hz         = PHYSICS_HZ;
    float   max_frame_time     = 0.1f;    // Clamp max delta
    float   slow_motion_min    = 0.1f;    // Minimum time scale
    float   slow_motion_max    = 2.0f;    // Maximum time scale
    uint8_t interpolate_render = 1;       // Interpolate between physics states
};

// =============================================================================
// GameState — Application/game lifecycle state
// =============================================================================
enum class GameState : uint8_t {
    UNINITIALIZED = 0,
    LOADING       = 1,
    WARMUP        = 2,
    PLAYING       = 3,
    PAUSED        = 4,
    GOAL_SCORED   = 5,
    HALF_TIME     = 6,
    FULL_TIME     = 7,
    REPLAY        = 8,
    MENU          = 9
};

// =============================================================================
// GameLoop — Accumulator-based fixed-timestep game loop
// =============================================================================
struct GameLoop {
    TimeState          time;
    FixedTimestepConfig config;
    GameState          state = GameState::UNINITIALIZED;

    // =========================================================================
    // init — Initialize timing system
    // =========================================================================
    void init(float physics_hz = PHYSICS_HZ)
    {
        time.current_time      = 0.0;
        time.delta_time        = 0.0f;
        time.fixed_delta       = 1.0f / physics_hz;
        time.accumulator       = 0.0f;
        time.time_scale        = 1.0f;
        time.frame_count        = 0u;
        time.physics_step_count = 0u;
        time.max_steps_per_frame = MAX_STEPS_PER_FRAME;

        config.physics_hz         = physics_hz;
        config.max_frame_time     = 0.1f;
        config.slow_motion_min    = 0.1f;
        config.slow_motion_max    = 2.0f;
        config.interpolate_render = 1;

        state = GameState::WARMUP;
    }

    // =========================================================================
    // begin_frame — Call at start of each frame
    // =========================================================================
    void begin_frame(double wall_clock_time)
    {
        // Compute delta from last frame
        if (time.current_time > 0.0) {
            time.delta_time = static_cast<float>(
                wall_clock_time - time.current_time);
        }

        time.current_time = wall_clock_time;

        // Clamp delta to max_frame_time (prevent spiral of death)
        if (time.delta_time > config.max_frame_time) {
            time.delta_time = config.max_frame_time;
        }
        if (time.delta_time < 0.0f) {
            time.delta_time = 0.0f;
        }

        // Add to accumulator with time_scale
        float scaled_dt = time.delta_time * time.time_scale;
        time.accumulator += scaled_dt;
    }

    // =========================================================================
    // should_step_physics — Check if enough time has accumulated
    // =========================================================================
    uint8_t should_step_physics() const
    {
        if (state != GameState::PLAYING) return 0;
        return (time.accumulator >= time.fixed_delta) ? 1u : 0u;
    }

    // =========================================================================
    // step_physics — Consume one fixed_delta from accumulator
    // =========================================================================
    void step_physics()
    {
        time.accumulator -= time.fixed_delta;
        ++time.physics_step_count;
    }

    // =========================================================================
    // get_interpolation_alpha — For render interpolation between states
    // =========================================================================
    float get_interpolation_alpha() const
    {
        if (time.fixed_delta < APC_EPSILON) return 0.0f;
        float alpha = time.accumulator / time.fixed_delta;
        if (alpha < 0.0f) alpha = 0.0f;
        if (alpha > 1.0f) alpha = 1.0f;
        return alpha;
    }

    // =========================================================================
    // end_frame — Finalize frame, increment counters
    // =========================================================================
    void end_frame()
    {
        ++time.frame_count;

        // Clamp accumulator to prevent unbounded growth
        if (time.accumulator > time.fixed_delta *
            static_cast<float>(time.max_steps_per_frame)) {
            time.accumulator = 0.0f;
        }
    }

    // =========================================================================
    // Controls
    // =========================================================================

    // --- set_time_scale: adjust simulation speed ---
    void set_time_scale(float scale)
    {
        if (scale < config.slow_motion_min) scale = config.slow_motion_min;
        if (scale > config.slow_motion_max) scale = config.slow_motion_max;
        time.time_scale = scale;
    }

    // --- pause: stop simulation ---
    void pause()
    {
        state = GameState::PAUSED;
    }

    // --- resume: continue simulation ---
    void resume()
    {
        if (state == GameState::PAUSED) {
            state = GameState::PLAYING;
        }
    }

    // --- toggle_pause: flip pause state ---
    void toggle_pause()
    {
        if (state == GameState::PAUSED) {
            state = GameState::PLAYING;
        } else if (state == GameState::PLAYING) {
            state = GameState::PAUSED;
        }
    }

    // --- get_physics_fps: get the current physics tick rate ---
    float get_physics_fps() const
    {
        if (time.fixed_delta < APC_EPSILON) return 0.0f;
        return 1.0f / time.fixed_delta;
    }

    // --- get_real_fps: get the actual frame rate ---
    float get_real_fps() const
    {
        if (time.delta_time < APC_EPSILON) return 0.0f;
        return 1.0f / time.delta_time;
    }
};

} // namespace apc
