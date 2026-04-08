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
    // tick — Main update: physics step -> AI -> entities -> debug viz
    // =========================================================================
    void tick()
    {
        if (state != ApplicationState::MATCH_PLAYING) return;

        // --- Physics step loop ---
        while (game_loop.should_step_physics()) {
            // Convert human input to motor intents
            for (uint32_t p = 0u; p < 4u; ++p) {
                // Find human-controlled athletes for this player
                for (uint32_t i = 0u; i < scene.entity_manager.athlete_count; ++i) {
                    AthleteEntity& a = scene.entity_manager.athletes[i];
                    if (!a.id.is_valid()) continue;
                    if (!a.is_human_controlled) continue;
                    // Assign first human-controlled entity to first player slot
                    a.current_intent = input_converter.convert(
                        input_states[p],
                        Vec3(0.0f, 0.0f, -1.0f), // Default camera forward
                        a.position);
                    break;
                }
            }

            // Step physics
            game_loop.step_physics();

            // Step scene (entities, timers)
            scene.update(game_loop.time.fixed_delta);

            // Step AI controllers
            for (uint32_t i = 0u; i < scene.entity_manager.athlete_count; ++i) {
                AthleteEntity& a = scene.entity_manager.athletes[i];
                if (!a.id.is_valid()) continue;
                if (a.is_human_controlled) continue;
                if (i >= MAX_AI_CONTROLLERS) continue;

                // Get steering from a simple seek behavior toward formation target
                SteeringOutput steering = SteeringSystem::seek(
                    a.position,
                    a.current_intent.move_direction,
                    scene.config.field_length * 0.01f * a.current_intent.move_speed);

                // Convert steering to motor intent via AI controller
                a.current_intent = scene.ai_controllers[i].update(
                    steering, a.position, a.orientation,
                    game_loop.time.fixed_delta);
            }

            // --- Collect debug data ---
            if (config.enable_ai_debug) {
                // Collect steering debug entries
                for (uint32_t i = 0u; i < scene.entity_manager.athlete_count; ++i) {
                    const AthleteEntity& a = scene.entity_manager.athletes[i];
                    if (!a.id.is_valid()) continue;

                    SteeringDebugEntry se;
                    se.entity_id = a.id;
                    se.position = a.position;
                    se.steering_force = a.current_intent.move_direction;
                    se.behavior_count = 0;
                    ai_debug.add_steering_entry(se);

                    FormationDebugEntry fe;
                    fe.entity_id = a.id;
                    fe.actual_position = a.position;
                    fe.formation_position = a.position; // Placeholder
                    fe.ball_influenced_position = a.position;
                    ai_debug.add_formation_entry(fe);
                }

                // Collect utility debug entries
                for (uint32_t i = 0u; i < scene.entity_manager.athlete_count; ++i) {
                    const AthleteEntity& a = scene.entity_manager.athletes[i];
                    if (!a.id.is_valid()) continue;

                    UtilityDebugEntry ue;
                    ue.entity_id = a.id;
                    ue.chosen_action = ActionType::IDLE;
                    ue.confidence = a.stamina;
                    ue.position = a.position;
                    ai_debug.add_utility_entry(ue);
                }
            }

            // End physics step
        }

        // --- Render debug visualization ---
        if (config.enable_ai_debug) {
            ai_debug.render(debug_draw_list);
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
    void run()
    {
        state = ApplicationState::MATCH_PLAYING;

        while (state != ApplicationState::SHUTDOWN) {
            double wall_time = 0.0; // Placeholder: real platform provides this

            begin_frame(wall_time);
            tick();
            end_frame();
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
        for (uint32_t i = 0u; title[i] && i < MAX_NAME_LENGTH - 1u; ++i) {
            cfg.window_title[i] = title[i];
        }
        cfg.window_title[27] = '\0';

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
        for (uint32_t i = 0u; title[i] && i < MAX_NAME_LENGTH - 1u; ++i) {
            cfg.window_title[i] = title[i];
        }
        cfg.window_title[29] = '\0';

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
