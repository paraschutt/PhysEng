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

            // --- FIX: Clear debug draw list each physics step to prevent trail artifacts ---
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

            // --- 3. AI Decision + Steering Pipeline for all AI athletes ---
            ScopedTimer ai_timer(perf_ai);
            const BallEntity* ball = scene.entity_manager.find_ball();
            Vec3 ball_pos = ball ? ball->position : Vec3(0.0f, 0.0f, 0.0f);
            float half_field = scene.config.field_length * 0.5f;

            // Pre-compute team athlete indices for team counts
            uint32_t home_player_idx = 0u;
            uint32_t away_player_idx = 0u;

            // Per-player formation targets (for debug visualization)
            Vec3 player_formation_targets[MAX_ATHLETES];
            for (uint32_t ii = 0u; ii < scene.entity_manager.athlete_count; ++ii) {
                player_formation_targets[ii] = scene.entity_manager.athletes[ii].position;
            }

            // --- Chase budget: only nearest N players per team may chase ---
            // Pre-compute per-team distance-to-ball rankings
            struct ChaseRank { uint32_t idx; float dist; };
            ChaseRank home_ranks[MAX_ATHLETES];
            ChaseRank away_ranks[MAX_ATHLETES];
            uint32_t home_rank_count = 0u, away_rank_count = 0u;
            for (uint32_t ii = 0u; ii < scene.entity_manager.athlete_count; ++ii) {
                const AthleteEntity& aa = scene.entity_manager.athletes[ii];
                if (!aa.id.is_valid() || aa.is_human_controlled) continue;
                float d = Vec3::length(Vec3::sub(aa.position, ball_pos));
                if (aa.team == TEAM_HOME && home_rank_count < MAX_ATHLETES) {
                    home_ranks[home_rank_count++] = { ii, d };
                } else if (aa.team == TEAM_AWAY && away_rank_count < MAX_ATHLETES) {
                    away_ranks[away_rank_count++] = { ii, d };
                }
            }
            // Simple selection sort by distance (ascending) — small arrays
            for (uint32_t ii = 0u; ii + 1u < home_rank_count; ++ii) {
                for (uint32_t jj = ii + 1u; jj < home_rank_count; ++jj) {
                    if (home_ranks[jj].dist < home_ranks[ii].dist) {
                        ChaseRank tmp = home_ranks[ii]; home_ranks[ii] = home_ranks[jj]; home_ranks[jj] = tmp;
                    }
                }
            }
            for (uint32_t ii = 0u; ii + 1u < away_rank_count; ++ii) {
                for (uint32_t jj = ii + 1u; jj < away_rank_count; ++jj) {
                    if (away_ranks[jj].dist < away_ranks[ii].dist) {
                        ChaseRank tmp = away_ranks[ii]; away_ranks[ii] = away_ranks[jj]; away_ranks[jj] = tmp;
                    }
                }
            }
            // Max 2 chasers per team (nearest + 2nd nearest)
            static constexpr uint32_t MAX_CHASERS = 2u;
            uint8_t home_can_chase[MAX_ATHLETES] = {};
            uint8_t away_can_chase[MAX_ATHLETES] = {};
            for (uint32_t ii = 0u; ii < home_rank_count && ii < MAX_CHASERS; ++ii) {
                home_can_chase[home_ranks[ii].idx] = 1u;
            }
            for (uint32_t ii = 0u; ii < away_rank_count && ii < MAX_CHASERS; ++ii) {
                away_can_chase[away_ranks[ii].idx] = 1u;
            }

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

                // Possession heuristic: check scene-level possession tracker
                // This uses the 1.5s possession window from ball interaction
                float has_possession = 0.0f;
                if (scene.last_possession_team == a.team && scene.possession_timer > 0.0f) {
                    // Team has possession — boost this player's context
                    // Only the nearest 2 players to ball get high possession factor
                    float own_rank = 0u; // How far from ball (1st, 2nd, etc.)
                    for (uint32_t j = 0u; j < scene.entity_manager.athlete_count; ++j) {
                        if (j == i) continue;
                        const AthleteEntity& other = scene.entity_manager.athletes[j];
                        if (!other.id.is_valid() || other.team != a.team) continue;
                        float other_dist = Vec3::length(Vec3::sub(other.position, ball_pos));
                        if (other_dist < dist_to_ball) ++own_rank;
                    }
                    has_possession = (own_rank == 0u) ? 1.0f : (own_rank == 1u) ? 0.5f : 0.2f;
                }

                // Chase budget check: suppress CHASE_BALL if not in top-2 nearest
                uint8_t can_chase = (a.team == TEAM_HOME)
                    ? home_can_chase[i] : away_can_chase[i];

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
                // FIX: Mirror formation for away team (they defend opposite goal)
                if (a.team == TEAM_AWAY) { form_pos_pq.x = -form_pos_pq.x; }
                float dist_to_formation = Vec3::length(Vec3::sub(a.position, form_pos_pq));
                float position_quality = 1.0f - std::min(dist_to_formation / 20.0f, 1.0f);

                // Compute actual nearest opponent distance for MARK_OPPONENT scoring
                float nearest_opp_dist = 999.0f;
                for (uint32_t j = 0u; j < scene.entity_manager.athlete_count; ++j) {
                    if (j == i) continue;
                    const AthleteEntity& opp = scene.entity_manager.athletes[j];
                    if (!opp.id.is_valid() || opp.team == a.team) continue;
                    float od = Vec3::length(Vec3::sub(opp.position, a.position));
                    if (od < nearest_opp_dist) nearest_opp_dist = od;
                }

                float context_inputs[8] = {
                    dist_to_ball,           // CONTEXT 0: DISTANCE_TO_BALL
                    dist_opp_goal,          // CONTEXT 1: DISTANCE_TO_GOAL
                    nearest_opp_dist,       // CONTEXT 2: DISTANCE_TO_OPPONENT (now real)
                    a.stamina,              // CONTEXT 3: STAMINA_PERCENT
                    has_possession,         // CONTEXT 4: TEAM_POSSESSION
                    1.0f,                   // CONTEXT 5: TIME_REMAINING (always full for now)
                    0.0f,                   // CONTEXT 6: SCORE_DIFFERENTIAL
                    position_quality         // CONTEXT 7: POSITION_QUALITY
                };

                uint32_t team_idx = (a.team == TEAM_HOME) ? 0u : 1u;

                // FIX: Per-player role weights — each position gets its own role profile
                // Previously both teams used CM weights for ALL players, causing bunching.
                scene.utility_ai[team_idx].configure_role(a.role);

                UtilityScore decision = scene.utility_ai[team_idx].evaluate(
                    context_inputs, 8u);

                // Enforce chase budget: if player can't chase and chose CHASE,
                // force them to HOLD formation instead
                if (!can_chase &&
                    (decision.action == AIActionType::CHASE_BALL ||
                     decision.action == AIActionType::PRESS)) {
                    decision.action = AIActionType::FORMATION_HOLD;
                    decision.score *= 0.5f;
                }

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

                // FIX: Mirror formation for away team (they defend opposite goal)
                if (a.team == TEAM_AWAY) { formation_pos.x = -formation_pos.x; }

                // Store formation target for debug visualization
                if (i < MAX_ATHLETES) {
                    player_formation_targets[i] = formation_pos;
                }

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
                    se.behavior_count = a.current_intent.action_type;
                    ai_debug.add_steering_entry(se);

                    FormationDebugEntry fe;
                    fe.entity_id = a.id;
                    fe.actual_position = a.position;
                    // FIX: Use computed formation target instead of current position
                    fe.formation_position = (i < MAX_ATHLETES)
                        ? player_formation_targets[i] : a.position;
                    fe.ball_influenced_position = fe.formation_position;
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

        // --- Print performance report ---
        perf.end_frame();
        perf.print_report(240); // Print every 240 physics steps (1 second)

        // --- Simulation state log (every 480 steps = 2 seconds) ---
        if (game_loop.time.physics_step_count % 480u == 0u &&
            game_loop.time.physics_step_count > 0u) {
            const BallEntity* log_ball = scene.entity_manager.find_ball();
            uint32_t step = game_loop.time.physics_step_count;
            float sim_t = scene.match_time_seconds;

            std::fprintf(stdout, "\n=== Sim State [step %u] time=%.1fs ===\n", step, sim_t);

            // Ball state
            if (log_ball && log_ball->id.is_valid()) {
                float bspeed = Vec3::length(log_ball->velocity);
                const char* poss = (log_ball->possession_team == TEAM_HOME) ? "HOME"
                    : (log_ball->possession_team == TEAM_AWAY) ? "AWAY" : "NONE";
                std::fprintf(stdout,
                    "  Ball: pos=(%.1f, %.2f, %.1f) vel=(%.1f, %.1f, %.1f) |v|=%.1f m/s poss=%s\n",
                    log_ball->position.x, log_ball->position.y, log_ball->position.z,
                    log_ball->velocity.x, log_ball->velocity.y, log_ball->velocity.z,
                    bspeed, poss);
            }

            // Per-player state
            uint32_t action_counts[16] = {};
            for (uint32_t i = 0u; i < scene.entity_manager.athlete_count; ++i) {
                const AthleteEntity& a = scene.entity_manager.athletes[i];
                if (!a.id.is_valid() || !a.is_active) continue;

                float dist_b = log_ball
                    ? Vec3::length(Vec3::sub(a.position, log_ball->position)) : 999.0f;
                float speed = Vec3::length(a.velocity);
                AIActionType act = static_cast<AIActionType>(a.flags & 0xFFu);
                uint32_t act_idx = static_cast<uint32_t>(act);
                if (act_idx < 16u) ++action_counts[act_idx];

                const char* team_str = (a.team == TEAM_HOME) ? "H" : "A";
                std::fprintf(stdout,
                    "  %s %2u:%-3s pos=(%6.1f,%5.1f) vel=%4.1f m/s act=%-8s dist_ball=%5.1f stam=%.2f\n",
                    team_str, a.jersey_number,
                    sport_role_name(a.role),
                    a.position.x, a.position.z, speed,
                    ai_action_name(act), dist_b, a.stamina);
            }

            // Action distribution summary
            std::fprintf(stdout, "  Actions:");
            for (uint32_t c = 0u; c < 16u; ++c) {
                if (action_counts[c] > 0u) {
                    std::fprintf(stdout, " %s=%u",
                        ai_action_name(static_cast<AIActionType>(c)), action_counts[c]);
                }
            }
            std::fprintf(stdout, "\n  Score: Home %u - %u Away\n",
                scene.home_score, scene.away_score);
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
