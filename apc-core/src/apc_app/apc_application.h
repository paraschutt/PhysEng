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

            // --- 3. AI Decision + Steering Pipeline for all AI athletes ---
            const BallEntity* ball = scene.entity_manager.find_ball();
            Vec3 ball_pos = ball ? ball->position : Vec3(0.0f, 0.0f, 0.0f);
            float half_field = scene.config.field_length * 0.5f;

            // Pre-compute team athlete indices for team counts
            uint32_t home_player_idx = 0u;
            uint32_t away_player_idx = 0u;

            for (uint32_t i = 0u; i < scene.entity_manager.athlete_count; ++i) {
                AthleteEntity& a = scene.entity_manager.athletes[i];
                if (!a.id.is_valid() || a.is_human_controlled) continue;
                if (i >= MAX_AI_CONTROLLERS) continue;

                // --- 3a. Compute context factors for Utility AI ---
                float dist_to_ball = Vec3::length(Vec3::sub(a.position, ball_pos));

                // Distance to own goal (defend) and opponent goal (attack)
                float own_goal_x = (a.team == TEAM_HOME) ? -half_field : half_field;
                float opp_goal_x = (a.team == TEAM_HOME) ?  half_field : -half_field;
                float dist_opp_goal = std::abs(a.position.x - opp_goal_x);

                // Possession heuristic: is this team's player closest to ball?
                float closest_own_dist = dist_to_ball;
                for (uint32_t j = 0u; j < scene.entity_manager.athlete_count; ++j) {
                    if (j == i) continue;
                    const AthleteEntity& other = scene.entity_manager.athletes[j];
                    if (!other.id.is_valid()) continue;
                    if (other.team != a.team) continue;
                    float other_dist = Vec3::length(Vec3::sub(other.position, ball_pos));
                    if (other_dist < closest_own_dist) {
                        closest_own_dist = other_dist;
                    }
                }
                float has_possession = (dist_to_ball <= closest_own_dist) ? 1.0f : 0.0f;

                // Feed context factors to UtilityAI (team 0 = home, team 1 = away)
                // Pre-compute formation position for position_quality below
                uint32_t form_idx_pq = (a.team == TEAM_HOME)
                    ? home_player_idx : away_player_idx;
                float possession_factor_pq = 0.3f + 0.7f * has_possession;
                Vec3 form_pos_pq = scene.formation_system.get_formation_position(
                    static_cast<uint8_t>(form_idx_pq),
                    ball_pos,
                    Vec3(own_goal_x, 0.0f, 0.0f),
                    Vec3(opp_goal_x, 0.0f, 0.0f),
                    possession_factor_pq);
                form_pos_pq.x *= half_field;
                form_pos_pq.z *= scene.config.field_width * 0.5f;
                form_pos_pq.y = 0.0f;
                float dist_to_formation = Vec3::length(Vec3::sub(a.position, form_pos_pq));
                float position_quality = 1.0f - std::min(dist_to_formation / 20.0f, 1.0f);

                float context_inputs[8] = {
                    dist_to_ball,           // CONTEXT 0: DISTANCE_TO_BALL
                    dist_opp_goal,          // CONTEXT 1: DISTANCE_TO_GOAL
                    0.0f,                   // CONTEXT 2: DISTANCE_TO_OPPONENT (simplified)
                    a.stamina,              // CONTEXT 3: STAMINA_PERCENT
                    has_possession,         // CONTEXT 4: TEAM_POSSESSION
                    1.0f,                   // CONTEXT 5: TIME_REMAINING (always full for now)
                    0.0f,                   // CONTEXT 6: SCORE_DIFFERENTIAL
                    position_quality         // CONTEXT 7: POSITION_QUALITY
                };

                uint32_t team_idx = (a.team == TEAM_HOME) ? 0u : 1u;
                UtilityScore decision = scene.utility_ai[team_idx].evaluate(
                    context_inputs, 8u);

                // --- 3b. Map AIActionType -> steering target ---
                // Get formation position for this player
                uint32_t form_idx = (a.team == TEAM_HOME)
                    ? home_player_idx : away_player_idx;

                // Possession factor: 0.5 when ball is central, 1.0 when own team has it
                float possession_factor = 0.3f + 0.7f * has_possession;

                Vec3 formation_pos = scene.formation_system.get_formation_position(
                    static_cast<uint8_t>(form_idx),
                    ball_pos,
                    Vec3(own_goal_x, 0.0f, 0.0f),
                    Vec3(opp_goal_x, 0.0f, 0.0f),
                    possession_factor);

                // Scale formation position to world space
                formation_pos.x *= half_field;
                formation_pos.z *= scene.config.field_width * 0.5f;
                formation_pos.y = 0.0f;

                // Select steering target based on chosen action
                Vec3 steer_target = formation_pos; // Default: hold formation
                float max_speed = scene.ai_controllers[i].steering_config.max_speed;
                if (max_speed < APC_EPSILON) max_speed = 7.0f;
                float urgency = 0.2f; // Default low urgency

                switch (decision.action) {
                case AIActionType::CHASE_BALL:
                    steer_target = ball_pos;
                    urgency = 0.9f;
                    // If very close to ball, steer slightly ahead of it
                    if (dist_to_ball < 3.0f && ball) {
                        // Predict where ball is heading
                        Vec3 predicted = Vec3::add(ball_pos,
                            Vec3::scale(ball->velocity, 0.3f));
                        predicted.y = 0.0f;
                        steer_target = predicted;
                    }
                    break;

                case AIActionType::SHOOT_BALL:
                    // Steer toward ball then kick toward opponent goal
                    if (dist_to_ball < 2.0f) {
                        steer_target = Vec3(opp_goal_x, 0.0f, 0.0f);
                        urgency = 1.0f;
                    } else {
                        steer_target = ball_pos;
                        urgency = 0.85f;
                    }
                    break;

                case AIActionType::MOVE_TO_POSITION:
                    steer_target = formation_pos;
                    urgency = 0.5f;
                    break;

                case AIActionType::SUPPORT_RUN:
                    // Run toward a position between ball and opponent goal
                    steer_target = Vec3(
                        (ball_pos.x + opp_goal_x) * 0.5f,
                        0.0f,
                        ball_pos.z + ((a.position.z > ball_pos.z) ? 5.0f : -5.0f)
                    );
                    urgency = 0.6f;
                    break;

                case AIActionType::PRESS:
                    // High urgency: press toward ball aggressively
                    steer_target = ball_pos;
                    urgency = 0.95f;
                    break;

                case AIActionType::INTERCEPT:
                    // Steer toward ball's predicted position
                    if (ball) {
                        float predict_time = dist_to_ball / (max_speed + APC_EPSILON);
                        if (predict_time > 1.0f) predict_time = 1.0f;
                        steer_target = Vec3::add(ball_pos,
                            Vec3::scale(ball->velocity, predict_time));
                        steer_target.y = 0.0f;
                    } else {
                        steer_target = ball_pos;
                    }
                    urgency = 0.8f;
                    break;

                case AIActionType::FORMATION_HOLD:
                case AIActionType::TACKLE:
                    // TACKLE: steer toward nearest opponent within range
                    {
                        float nearest_opp_dist = 999.0f;
                        Vec3 nearest_opp_pos = a.position;
                        for (uint32_t j = 0u; j < scene.entity_manager.athlete_count; ++j) {
                            const AthleteEntity& opp = scene.entity_manager.athletes[j];
                            if (!opp.id.is_valid() || opp.team == a.team) continue;
                            float d = Vec3::length(Vec3::sub(opp.position, a.position));
                            if (d < nearest_opp_dist) {
                                nearest_opp_dist = d;
                                nearest_opp_pos = opp.position;
                            }
                        }
                        static constexpr float TACKLE_RANGE = 3.0f;
                        if (nearest_opp_dist < TACKLE_RANGE) {
                            steer_target = nearest_opp_pos;
                            urgency = 0.95f;
                        } else {
                            steer_target = formation_pos;
                            urgency = 0.3f;
                        }
                    }
                    break;

                case AIActionType::PASS_BALL:
                    // PASS_BALL: steer toward ball, then pass to nearest forward teammate
                    if (dist_to_ball < 2.0f && ball) {
                        // Find best teammate to pass to (furthest forward same-team)
                        float best_forward = -999.0f;
                        Vec3 best_teammate_pos = a.position;
                        for (uint32_t j = 0u; j < scene.entity_manager.athlete_count; ++j) {
                            if (j == i) continue;
                            const AthleteEntity& tm = scene.entity_manager.athletes[j];
                            if (!tm.id.is_valid() || tm.team != a.team) continue;
                            float fwd = (a.team == TEAM_HOME) ? tm.position.x : -tm.position.x;
                            float dist = Vec3::length(Vec3::sub(tm.position, a.position));
                            if (fwd > best_forward && dist > 3.0f && dist < 30.0f) {
                                best_forward = fwd;
                                best_teammate_pos = tm.position;
                            }
                        }
                        steer_target = best_teammate_pos;
                        urgency = 0.9f;
                    } else {
                        steer_target = ball_pos;
                        urgency = 0.7f;
                    }
                    break;

                case AIActionType::BLOCK:
                    // Steer between ball and own goal to block shots
                    steer_target = Vec3(
                        (ball_pos.x + own_goal_x) * 0.5f,
                        0.0f,
                        ball_pos.z
                    );
                    urgency = 0.85f;
                    break;

                case AIActionType::MARK_OPPONENT:
                    // Find nearest opponent and shadow them
                    {
                        float nearest_dist = 999.0f;
                        Vec3 nearest_pos = a.position;
                        for (uint32_t j = 0u; j < scene.entity_manager.athlete_count; ++j) {
                            const AthleteEntity& opp = scene.entity_manager.athletes[j];
                            if (!opp.id.is_valid() || opp.team == a.team) continue;
                            float d = Vec3::length(Vec3::sub(opp.position, a.position));
                            if (d < nearest_dist) {
                                nearest_dist = d;
                                nearest_pos = opp.position;
                            }
                        }
                        // Position between opponent and own goal (goal-side marking)
                        steer_target = Vec3(
                            (nearest_pos.x + own_goal_x) * 0.45f,
                            0.0f,
                            nearest_pos.z
                        );
                        urgency = 0.6f;
                    }
                    break;

                case AIActionType::CROSS:
                    // Cross: steer to ball if far, or toward touchline if close
                    if (dist_to_ball < 2.0f && ball) {
                        // Cross toward opponent goal area
                        steer_target = Vec3(
                            opp_goal_x * 0.6f,
                            0.0f,
                            0.0f
                        );
                        urgency = 0.9f;
                    } else {
                        steer_target = ball_pos;
                        urgency = 0.7f;
                    }
                    break;

                case AIActionType::HEADER:
                    // Header: jump toward ball (simplified as steering to ball)
                    steer_target = ball_pos;
                    urgency = 0.85f;
                    break;

                case AIActionType::DIVE_SAVE:
                    // Goalkeeper dive: steer toward ball urgently
                    if (ball) {
                        // Predict ball landing near goal line
                        Vec3 dive_target = Vec3::add(ball_pos,
                            Vec3::scale(ball->velocity, 0.15f));
                        dive_target.x = own_goal_x + (a.team == TEAM_HOME ? 2.0f : -2.0f);
                        dive_target.y = 0.0f;
                        steer_target = dive_target;
                    } else {
                        steer_target = Vec3(own_goal_x, 0.0f, 0.0f);
                    }
                    urgency = 1.0f;
                    break;

                case AIActionType::PUNT:
                    // Goalkeeper punt: steer to ball then kick long
                    steer_target = ball_pos;
                    urgency = 0.8f;
                    break;

                default:
                    steer_target = formation_pos;
                    urgency = 0.2f;
                    break;
                }

                // --- 3c. Compute composite steering ---
                // Primary: arrive at target (slows down when close)
                SteeringOutput primary = SteeringSystem::arrive(
                    a.position, steer_target, max_speed, 0.5f, 4.0f);
                primary.urgency = urgency;

                // Secondary: separation from nearby teammates
                Vec3 neighbors[MAX_NEIGHBOR_COUNT];
                uint32_t neighbor_count = 0u;
                for (uint32_t j = 0u; j < scene.entity_manager.athlete_count && neighbor_count < MAX_NEIGHBOR_COUNT; ++j) {
                    if (j == i) continue;
                    const AthleteEntity& other = scene.entity_manager.athletes[j];
                    if (!other.id.is_valid()) continue;
                    if (other.team != a.team) continue;
                    float ndist = Vec3::length(Vec3::sub(a.position, other.position));
                    if (ndist < 3.0f) { // Only care about close teammates
                        neighbors[neighbor_count++] = other.position;
                    }
                }
                SteeringOutput sep = SteeringSystem::separation(
                    neighbors, neighbor_count, a.position,
                    2.0f, 15.0f);

                // Blend primary + separation
                WeightedSteering blended[2];
                blended[0].behavior = SteeringBehavior::ARRIVE;
                blended[0].weight = 1.0f;
                blended[0].output = primary;
                blended[1].behavior = SteeringBehavior::SEPARATION;
                blended[1].weight = 2.5f;
                blended[1].output = sep;

                SteeringOutput final_steering = SteeringSystem::blend(blended, 2u);

                // --- 3d. Convert steering -> MotorIntent via AI motor controller ---
                a.current_intent = scene.ai_controllers[i].update(
                    final_steering, a.position, a.orientation, phys_dt);

                // Store chosen action for debug
                a.flags = static_cast<uint16_t>(
                    a.flags & 0xFF00u) | static_cast<uint16_t>(decision.action);

                // Increment team player index for formation lookup
                if (a.team == TEAM_HOME) ++home_player_idx;
                else ++away_player_idx;
            }

            // --- 4. Ball interaction: kick the ball when close ---
            scene.process_ball_interaction();

            // --- 5. Step scene (entities, kinematics, timers) ---
            scene.update(phys_dt);

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
                    se.behavior_count = a.current_intent.action_type;
                    ai_debug.add_steering_entry(se);

                    FormationDebugEntry fe;
                    fe.entity_id = a.id;
                    fe.actual_position = a.position;
                    fe.formation_position = a.position;
                    fe.ball_influenced_position = a.position;
                    ai_debug.add_formation_entry(fe);

                    UtilityDebugEntry ue;
                    ue.entity_id = a.id;
                    ue.chosen_action = static_cast<AIActionType>(a.flags & 0xFFu);
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
