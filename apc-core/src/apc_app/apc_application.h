#pragma once
// =============================================================================
// apc_application.h — Top-level application lifecycle and configuration
// =============================================================================
//
// Provides the highest-level entry point for the APC Physics Engine:
//
//   - ApplicationConfig: window, rendering, debug settings
//   - ApplicationState: application lifecycle enumeration
//   - Application: owns GameLoop, SceneState, AIDebugVisualizer, InputState
//   - Main loop: begin_frame -> update_input -> tick -> end_frame
//
// This is the final integration layer that ties all subsystems together.
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation (embedded members)
//   - Deterministic
//   - C++17
//
// =============================================================================

#include "apc_app/apc_game_loop.h"
#include "apc_app/apc_perf_timer.h"
#include "apc_app/apc_scene_manager.h"
#include "apc_ai/apc_ai_debug_viz.h"
#include "apc_input/apc_input_types.h"
#include "apc_input/apc_input_state.h"
#include "apc_input/apc_input_mapping.h"
#include "apc_render/apc_debug_draw.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_math_common.h"
#include <cstdint>
#include <cmath>

namespace apc {

// =============================================================================
// Constants
// =============================================================================
static constexpr uint32_t MAX_NAME_LENGTH = 64;
static constexpr uint32_t MAX_PATH_LENGTH = 256;

// =============================================================================
// AI Debug Helpers — action/role name strings for logging
// =============================================================================
static APC_FORCEINLINE const char* ai_action_name(AIActionType a) {
    switch (a) {
    case AIActionType::IDLE:            return "IDLE";
    case AIActionType::MOVE_TO_POSITION: return "MOVE";
    case AIActionType::CHASE_BALL:      return "CHASE";
    case AIActionType::PASS_BALL:       return "PASS";
    case AIActionType::SHOOT_BALL:      return "SHOOT";
    case AIActionType::TACKLE:          return "TACKLE";
    case AIActionType::BLOCK:           return "BLOCK";
    case AIActionType::INTERCEPT:       return "INTERCEPT";
    case AIActionType::MARK_OPPONENT:   return "MARK";
    case AIActionType::SUPPORT_RUN:     return "SUPPORT";
    case AIActionType::CROSS:           return "CROSS";
    case AIActionType::HEADER:          return "HEADER";
    case AIActionType::DIVE_SAVE:       return "DIVESAVE";
    case AIActionType::PUNT:            return "PUNT";
    case AIActionType::FORMATION_HOLD:  return "HOLD";
    case AIActionType::PRESS:           return "PRESS";
    default:                            return "???";
    }
}

static APC_FORCEINLINE const char* sport_role_name(SportRole r) {
    switch (r) {
    case SportRole::SOCCER_GK:  return "GK";
    case SportRole::SOCCER_CB:  return "CB";
    case SportRole::SOCCER_LB:  return "LB";
    case SportRole::SOCCER_RB:  return "RB";
    case SportRole::SOCCER_CDM: return "CDM";
    case SportRole::SOCCER_CM:  return "CM";
    case SportRole::SOCCER_CAM: return "CAM";
    case SportRole::SOCCER_LW:  return "LW";
    case SportRole::SOCCER_RW:  return "RW";
    case SportRole::SOCCER_ST:  return "ST";
    default:                   return "SUB";
    }
}

// =============================================================================
// ApplicationConfig — Top-level application settings
// =============================================================================
struct ApplicationConfig {
    char     window_title[MAX_NAME_LENGTH]  = {};
    uint32_t window_width                     = 1920u;
    uint32_t window_height                    = 1080u;
    float    render_scale                     = 1.0f;
    uint8_t  fullscreen                       = 0;
    uint8_t  vsync                            = 1;
    uint8_t  anti_aliasing_samples            = 4;
    float    target_fps                       = 60.0f;
    char     asset_path[MAX_PATH_LENGTH]      = {};
    uint8_t  enable_debug_draw                 = 1;
    uint8_t  enable_ai_debug                   = 1;
    uint8_t  enable_physics_debug             = 1;
    uint8_t  log_level                        = 3; // 0=none, 1=error, 2=warn, 3=info, 4=debug
};

// =============================================================================
// ApplicationState — Application lifecycle state
// =============================================================================
enum class ApplicationState : uint8_t {
    SHUTDOWN       = 0,
    INITIALIZING   = 1,
    RUNNING       = 2,
    PAUSED         = 3,
    MATCH_LOADING  = 4,
    MATCH_PLAYING  = 5,
    MATCH_ENDED    = 6
};

// =============================================================================
// Application — Top-level application lifecycle manager
// =============================================================================
struct Application {
    ApplicationConfig  config;
    ApplicationState   state = ApplicationState::SHUTDOWN;

    // --- Embedded subsystems ---
    GameLoop           game_loop;
    SceneState         scene;
    AIDebugVisualizer  ai_debug;

    // --- Input state for local multiplayer ---
    InputState         input_states[4]; // MAX_SIMULTANEOUS_INPUTS

    // --- Intent converter for human players ---
    IntentConverter    input_converter;

    // --- Debug draw list (for render backend) ---
    DebugDrawList      debug_draw_list;

    // --- Performance profiling ---
    PerfReport         perf;
    PerfSection*       perf_ai = nullptr;
    PerfSection*       perf_physics = nullptr;
    PerfSection*       perf_ball = nullptr;

    // =========================================================================
    // init — Initialize all systems
    // =========================================================================
    uint8_t init(const ApplicationConfig& cfg)
    {
        config = cfg;
        state = ApplicationState::INITIALIZING;

        // Initialize game loop
        game_loop.init();

        // Configure input converter defaults
        input_converter = IntentConverter::soccer_defaults();

        // Initialize all input states
        for (uint32_t i = 0u; i < 4u; ++i) {
            input_states[i] = InputState();
        }

        // Reset AI debug
        ai_debug = AIDebugVisualizer();

        // Clear debug draw list
        debug_draw_list.clear();

        // Initialize perf profiling
        perf.reset();
        perf.enabled = 1;
        perf_ai = perf.get_section("ai_decision");
        perf_physics = perf.get_section("physics_step");
        perf_ball = perf.get_section("ball_interaction");

        state = ApplicationState::RUNNING;
        return 1;
    }

    // =========================================================================
    // shutdown — Clean shutdown
    // =========================================================================
    void shutdown()
    {
        state = ApplicationState::SHUTDOWN;
        scene.unload();
        game_loop = GameLoop();
        ai_debug = AIDebugVisualizer();
        debug_draw_list.clear();
        for (uint32_t i = 0u; i < 4u; ++i) {
            input_states[i] = InputState();
        }
    }

    // =========================================================================
    // load_match — Load a match
    // =========================================================================
    uint8_t load_match(const MatchConfig& match_config)
    {
        state = ApplicationState::MATCH_LOADING;

        uint8_t result = scene.load_match(match_config);
        if (result) {
            state = ApplicationState::MATCH_PLAYING;
            game_loop.start_playing(); // Transition from WARMUP → PLAYING
        } else {
            state = ApplicationState::RUNNING;
        }
        return result;
    }

    // =========================================================================
    // begin_frame — Start frame processing
    // =========================================================================
    void begin_frame(double wall_time)
    {
        if (state != ApplicationState::MATCH_PLAYING) {
            // Still run game loop timing even when not in match
            game_loop.begin_frame(wall_time);
            return;
        }

        game_loop.begin_frame(wall_time);

        // Begin input frame for all players
        for (uint32_t i = 0u; i < 4u; ++i) {
            input_states[i].begin_frame();
        }

        // Begin debug frame
        ai_debug.begin_frame();
    }

    // =========================================================================
    // update_input — Feed input for a player
    // =========================================================================
    void update_input(uint8_t player_index, const InputState& new_input)
    {
        if (player_index >= 4u) return;
        input_states[player_index] = new_input;
        input_states[player_index].frame_delta();
    }

    // =========================================================================
    // tick — Main update: AI decision -> steering -> motor intent -> physics -> ball
    // =========================================================================
    void tick()
    {
        if (state != ApplicationState::MATCH_PLAYING) return;

        // --- Physics step loop ---
        while (game_loop.should_step_physics()) {
            perf.begin_frame();

            // Clear debug draw list each physics step to prevent trail artifacts
            debug_draw_list.clear();

            float phys_dt = game_loop.time.fixed_delta;

            // --- 1. Convert human input to motor intents ---
            for (uint32_t p = 0u; p < 4u; ++p) {
                for (uint32_t i = 0u; i < scene.entity_manager.athlete_count; ++i) {
                    AthleteEntity& a = scene.entity_manager.athletes[i];
                    if (!a.id.is_valid()) continue;
                    if (!a.is_human_controlled) continue;
                    a.current_intent = input_converter.convert(
                        input_states[p],
                        Vec3(0.0f, 0.0f, -1.0f),
                        a.position);
                    break;
                }
            }

            // --- 2. Step physics counter ---
            game_loop.step_physics();

            // --- 3. AI Decision + Steering Pipeline ---
            {
                ScopedTimer ai_timer(perf_ai);

                // PHASE 12 REFACTOR:
                // The 300-line monolithic Phase 9 AI has been completely purged.
                // We now delegate to the decoupled Phase 11 architecture.
                // SceneState internally handles PerceptionRingBuffer,
                // InfluenceMaps, and Topology-Aware utility evaluations.
                scene.evaluate_ai_decisions(phys_dt);
            }

            // --- 4. Ball interaction ---
            {
                ScopedTimer ball_timer(perf_ball);
                scene.process_ball_interaction();
            }

            // --- 5. Step scene (entities, kinematics, timers) ---
            {
                ScopedTimer phys_timer(perf_physics);
                scene.update(phys_dt);
            }

            // --- 6. Collect debug data ---
            if (config.enable_ai_debug) {
                ai_debug.begin_frame();
                for (uint32_t i = 0u; i < scene.entity_manager.athlete_count; ++i) {
                    const AthleteEntity& a = scene.entity_manager.athletes[i];
                    if (!a.id.is_valid()) continue;

                    SteeringDebugEntry se;
                    se.entity_id = a.id;
                    se.position = a.position;
                    se.steering_force = a.current_intent.move_direction;
                    se.behavior_count = 1;
                    ai_debug.add_steering_entry(se);

                    UtilityDebugEntry ue;
                    ue.entity_id = a.id;
                    ue.chosen_action = static_cast<AIActionType>(a.active_action_id & 0xFFu);
                    ue.confidence = a.stamina;
                    ue.position = a.position;
                    ai_debug.add_utility_entry(ue);
                }
            }
        }

        // --- Render debug visualization ---
        if (config.enable_ai_debug) {
            ai_debug.render(debug_draw_list);
        }

        // --- Print performance report ---
        perf.end_frame();
        perf.print_report(240);

        // --- Simulation state log (every 480 steps = 2 seconds) ---
        if (game_loop.time.physics_step_count % 480u == 0u &&
            game_loop.time.physics_step_count > 0u) {

            uint32_t step = game_loop.time.physics_step_count;
            float sim_t = scene.match_time_seconds;

            std::fprintf(stdout, "\n=== Sim State [step %u] time=%.1fs ===\n", step, sim_t);
            std::fprintf(stdout, "  Score: Home %u - %u Away\n", scene.home_score, scene.away_score);
            std::fprintf(stdout, "================================\n");
        }
    }

    // =========================================================================
    // end_frame — Finalize frame
    // =========================================================================
    void end_frame()
    {
        game_loop.end_frame();
    }

    // =========================================================================
    // run — Main loop (calls begin_frame/tick/end_frame until shutdown)
    // =========================================================================
    // NOTE: This is a convenience method for headless/embedded use.
    // For SDL2/windowed use, the external main loop calls begin_frame/tick/end_frame
    // directly (see sdl2_main.cpp). This method uses a simple frame counter
    // to prevent infinite loops and transitions to MATCH_ENDED after max steps.
    // =========================================================================
    void run()
    {
        state = ApplicationState::MATCH_PLAYING;
        uint32_t safety_counter = 0;
        static constexpr uint32_t MAX_HEADLESS_STEPS = 2400; // 10 seconds at 240Hz

        while (state != ApplicationState::SHUTDOWN) {
            double wall_time = static_cast<double>(safety_counter) * game_loop.time.fixed_delta;

            begin_frame(wall_time);
            tick();
            end_frame();

            ++safety_counter;

            // Safety: prevent infinite loop in headless mode
            if (safety_counter >= MAX_HEADLESS_STEPS) {
                state = ApplicationState::MATCH_ENDED;
            }

            // Check for match end conditions (goal limit or time)
            if (game_loop.state != apc::GameState::PLAYING) {
                state = ApplicationState::MATCH_ENDED;
            }
        }
    }

    // =========================================================================
    // Pause / Resume
    // =========================================================================
    void request_pause()
    {
        game_loop.pause();
    }

    void request_resume()
    {
        game_loop.resume();
    }

    // =========================================================================
    // get_debug_draw_list — For render backend
    // =========================================================================
    DebugDrawList& get_debug_draw_list()
    {
        return debug_draw_list;
    }

    // =========================================================================
    // Static factory methods — Preset configurations
    // =========================================================================

    static ApplicationConfig soccer_defaults()
    {
        ApplicationConfig cfg;
        // Set window title
        const char* title = "APC Physics Engine — Soccer";
        cfg.window_title[0] = '\0';
        for (uint32_t i = 0u; title[i] && i < MAX_NAME_LENGTH - 1u; ++i) {
            cfg.window_title[i] = title[i];
            cfg.window_title[i + 1] = '\0';
        }

        cfg.window_width          = 1920u;
        cfg.window_height         = 1080u;
        cfg.render_scale          = 1.0f;
        cfg.fullscreen           = 0;
        cfg.vsync                = 1;
        cfg.anti_aliasing_samples = 4;
        cfg.target_fps           = 60.0f;
        cfg.enable_debug_draw     = 1;
        cfg.enable_ai_debug       = 1;
        cfg.enable_physics_debug  = 1;
        cfg.log_level            = 3;
        return cfg;
    }

    static ApplicationConfig basketball_defaults()
    {
        ApplicationConfig cfg;
        const char* title = "APC Physics Engine — Basketball";
        cfg.window_title[0] = '\0';
        for (uint32_t i = 0u; title[i] && i < MAX_NAME_LENGTH - 1u; ++i) {
            cfg.window_title[i] = title[i];
            cfg.window_title[i + 1] = '\0';
        }

        cfg.window_width          = 1920u;
        cfg.window_height         = 1080u;
        cfg.render_scale          = 1.0f;
        cfg.fullscreen           = 0;
        cfg.vsync                = 1;
        cfg.anti_aliasing_samples = 4;
        cfg.target_fps           = 60.0f;
        cfg.enable_debug_draw     = 1;
        cfg.enable_ai_debug       = 1;
        cfg.enable_physics_debug  = 1;
        cfg.log_level            = 3;
        return cfg;
    }
};

} // namespace apc
