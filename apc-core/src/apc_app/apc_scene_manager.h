#pragma once
// =============================================================================
// apc_scene_manager.h — Match loading, entity spawning, and per-frame orchestration
// =============================================================================
//
// Manages the lifetime of a match scene:
//
//   - SportType: sport enumeration
//   - MatchConfig: match parameters (sport, teams, formations, field)
//   - SceneState: owns EntityManager, FormationSystem, AI controllers, UtilityAI
//   - Static factory methods for common sport configurations
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation (embedded members, fixed-size arrays)
//   - Deterministic
//   - C++17
//
// =============================================================================

#include "apc_app/apc_game_loop.h"
#include "apc_entity/apc_entity_types.h"
#include "apc_entity/apc_entity_manager.h"
#include "apc_ai/apc_ai_formation.h"
#include "apc_ai/apc_ai_motor.h"
#include "apc_ai/apc_ai_decision.h"
#include "apc_input/apc_input_types.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_math_common.h"
#include <cstdint>
#include <cmath>

namespace apc {

// =============================================================================
// SportType — Sport enumeration
// =============================================================================
enum class SportType : uint8_t {
    SOCCER           = 0,
    BASKETBALL       = 1,
    AMERICAN_FOOTBALL = 2,
    RUGBY            = 3,
    HOCKEY           = 4
};

// =============================================================================
// MatchConfig — Parameters for loading a match
// =============================================================================
struct MatchConfig {
    SportType     sport               = SportType::SOCCER;
    float         match_duration_seconds = 5400.0f; // 90 min default
    uint8_t       halves              = 2;        // Soccer = 2, Basketball = 4
    float         half_duration       = 2700.0f;
    uint8_t       players_per_team    = 11;
    uint8_t       subs_per_team       = 3;
    FormationType home_formation      = FormationType::FORMATION_4_4_2;
    FormationType away_formation      = FormationType::FORMATION_4_4_2;
    char          home_team_name[32]  = {};
    char          away_team_name[32]  = {};
    float         field_length        = 105.0f;  // Soccer default
    float         field_width         = 68.0f;   // Soccer default
    uint8_t       offside_enabled     = 1;
    uint8_t       var_enabled         = 0;
    uint8_t       injuries_enabled    = 0;
};

// =============================================================================
// SceneState — Full match scene state
// =============================================================================
struct SceneState {
    // --- Core subsystems ---
    EntityManager    entity_manager;
    FormationSystem  formation_system;
    // NOTE: GameLoop lives in Application, not here. SceneState provides
    // match-level state (scores, time) but does NOT own the physics clock.
    // The old duplicate GameLoop here was dead code — removed in Phase 9a.

    // --- AI controllers (one per athlete) ---
    AIMotorController ai_controllers[MAX_AI_CONTROLLERS];

    // --- Utility AI (one per team) ---
    UtilityAI        utility_ai[2];

    // --- Match configuration ---
    MatchConfig      config;

    // --- State ---
    uint8_t  is_loaded         = 0;
    uint32_t home_score        = 0u;
    uint32_t away_score        = 0u;
    float    match_time_seconds = 0.0f;

    // =========================================================================
    // load_match — Create teams, spawn athletes, assign formations
    // =========================================================================
    uint8_t load_match(const MatchConfig& match_config)
    {
        // Unload any existing match
        unload();

        config = match_config;

        // --- Configure game loop (done by Application, not here) ---
        // The Application owns the single GameLoop that drives physics.

        // --- Set up formations ---
        formation_system.set_formation(config.home_formation, config.away_formation);

        // --- Spawn home team ---
        spawn_team(TEAM_HOME, config.home_formation,
                   config.home_team_name,
                   Vec3(-config.field_length * 0.5f, 0.0f, 0.0f));

        // --- Spawn away team ---
        spawn_team(TEAM_AWAY, config.away_formation,
                   config.away_team_name,
                   Vec3(config.field_length * 0.5f, 0.0f, 0.0f));

        // --- Spawn ball ---
        spawn_ball();

        // --- Assign AI controllers ---
        assign_all_ai();

        is_loaded = 1;
        return 1;
    }

    // =========================================================================
    // unload — Clear everything
    // =========================================================================
    void unload()
    {
        entity_manager.reset();
        formation_system.reset();
        config = MatchConfig();
        is_loaded         = 0;
        home_score        = 0u;
        away_score        = 0u;
        match_time_seconds = 0.0f;

        for (uint32_t i = 0u; i < MAX_AI_CONTROLLERS; ++i) {
            ai_controllers[i].reset();
        }
        utility_ai[0].reset();
        utility_ai[1].reset();
    }

    // =========================================================================
    // spawn_team — Populate athletes for one team
    // =========================================================================
    void spawn_team(TeamId team, FormationType formation, const char* name,
                     const Vec3& goal_position)
    {
        (void)name;
        (void)goal_position;

        FormationSet fs;
        switch (formation) {
        case FormationType::FORMATION_4_3_3:  fs = FormationSystem::preset_4_3_3(); break;
        case FormationType::FORMATION_3_5_2:  fs = FormationSystem::preset_3_5_2(); break;
        case FormationType::FORMATION_4_2_3_1: fs = FormationSystem::preset_4_2_3_1(); break;
        default: fs = FormationSystem::preset_4_4_2(); break;
        }

        // Determine field half direction
        float x_dir = (team == TEAM_HOME) ? 1.0f : -1.0f;

        for (uint8_t i = 0u; i < fs.position_count; ++i) {
            const FormationPosition& pos = fs.positions[i];

            // Scale normalized position to field dimensions
            Vec3 world_pos(
                pos.base_position.x * config.field_length * 0.5f * x_dir,
                0.0f,
                pos.base_position.z * config.field_width * 0.5f
            );

            entity_manager.spawn_athlete(team, pos.compatible_role,
                                          world_pos, i + 1);
        }
    }

    // =========================================================================
    // spawn_ball — Create ball appropriate for sport
    // =========================================================================
    void spawn_ball()
    {
        uint8_t ball_type = 0;
        switch (config.sport) {
        case SportType::BASKETBALL:       ball_type = 1; break;
        case SportType::AMERICAN_FOOTBALL: ball_type = 2; break;
        case SportType::RUGBY:            ball_type = 3; break;
        default: ball_type = 0; break;
        }
        entity_manager.spawn_ball(ball_type, Vec3(0.0f, 0.11f, 0.0f));
    }

    // =========================================================================
    // assign_ai — Configure AI controller for an athlete
    // =========================================================================
    void assign_ai(EntityId athlete_id, SportRole role)
    {
        if (!athlete_id.is_valid()) return;
        if (athlete_id.index >= MAX_AI_CONTROLLERS) return;

        AIMotorController& ctrl = ai_controllers[athlete_id.index];
        ctrl.reset();

        // Set preset based on role
        switch (role) {
        case SportRole::SOCCER_GK:
            ctrl.set_preset("goalkeeper"); break;
        case SportRole::SOCCER_CB:
        case SportRole::SOCCER_LB:
        case SportRole::SOCCER_RB:
            ctrl.set_preset("defender"); break;
        case SportRole::SOCCER_CDM:
        case SportRole::SOCCER_CM:
            ctrl.set_preset("midfielder"); break;
        case SportRole::SOCCER_CAM:
        case SportRole::SOCCER_LW:
        case SportRole::SOCCER_RW:
        case SportRole::SOCCER_ST:
            ctrl.set_preset("forward"); break;
        default:
            ctrl.set_preset("midfielder"); break;
        }
    }

    // =========================================================================
    // assign_human — Mark athlete as human-controlled
    // =========================================================================
    void assign_human(EntityId athlete_id)
    {
        if (!athlete_id.is_valid()) return;
        AthleteEntity* a = entity_manager.get_athlete(athlete_id);
        if (a) {
            a->is_human_controlled = 1;
        }
    }

    // =========================================================================
    // assign_all_ai — Assign AI controllers to all spawned athletes
    // =========================================================================
    void assign_all_ai()
    {
        for (uint32_t i = 0u; i < entity_manager.athlete_count; ++i) {
            AthleteEntity& a = entity_manager.athletes[i];
            if (!a.id.is_valid()) continue;
            if (a.is_human_controlled) continue;

            assign_ai(a.id, a.role);
        }

        // Configure utility AI per team
        utility_ai[0].configure_role(SportRole::SOCCER_CM);
        utility_ai[1].configure_role(SportRole::SOCCER_CM);

        // Register available actions for utility evaluation
        utility_ai[0].actions[utility_ai[0].action_count++] = AIActionType::FORMATION_HOLD;
        utility_ai[0].actions[utility_ai[0].action_count++] = AIActionType::CHASE_BALL;
        utility_ai[0].actions[utility_ai[0].action_count++] = AIActionType::SHOOT_BALL;
        utility_ai[0].actions[utility_ai[0].action_count++] = AIActionType::MOVE_TO_POSITION;
        utility_ai[0].actions[utility_ai[0].action_count++] = AIActionType::SUPPORT_RUN;
        utility_ai[0].actions[utility_ai[0].action_count++] = AIActionType::PRESS;
        utility_ai[0].actions[utility_ai[0].action_count++] = AIActionType::INTERCEPT;
        utility_ai[0].actions[utility_ai[0].action_count++] = AIActionType::TACKLE;
        utility_ai[0].actions[utility_ai[0].action_count++] = AIActionType::PASS_BALL;

        utility_ai[1].actions[utility_ai[1].action_count++] = AIActionType::FORMATION_HOLD;
        utility_ai[1].actions[utility_ai[1].action_count++] = AIActionType::CHASE_BALL;
        utility_ai[1].actions[utility_ai[1].action_count++] = AIActionType::SHOOT_BALL;
        utility_ai[1].actions[utility_ai[1].action_count++] = AIActionType::MOVE_TO_POSITION;
        utility_ai[1].actions[utility_ai[1].action_count++] = AIActionType::SUPPORT_RUN;
        utility_ai[1].actions[utility_ai[1].action_count++] = AIActionType::PRESS;
        utility_ai[1].actions[utility_ai[1].action_count++] = AIActionType::INTERCEPT;
        utility_ai[1].actions[utility_ai[1].action_count++] = AIActionType::TACKLE;
        utility_ai[1].actions[utility_ai[1].action_count++] = AIActionType::PASS_BALL;

        // Add default considerations
        utility_ai[0].add_consideration("dist_ball", 1.5f, ResponseCurve::QUADRATIC, 0.0f, 50.0f);
        utility_ai[0].add_consideration("dist_goal", 1.2f, ResponseCurve::EXPONENTIAL, 0.0f, 60.0f);
        utility_ai[0].add_consideration("stamina", 0.8f, ResponseCurve::LINEAR, 0.0f, 1.0f);
        utility_ai[1].add_consideration("dist_ball", 1.5f, ResponseCurve::QUADRATIC, 0.0f, 50.0f);
        utility_ai[1].add_consideration("dist_goal", 1.2f, ResponseCurve::EXPONENTIAL, 0.0f, 60.0f);
        utility_ai[1].add_consideration("stamina", 0.8f, ResponseCurve::LINEAR, 0.0f, 1.0f);
    }

    // =========================================================================
    // process_ball_interaction — Check proximity, apply kicks/touches
    // =========================================================================
    void process_ball_interaction()
    {
        if (!is_loaded) return;

        BallEntity* ball = entity_manager.find_ball();
        if (!ball) return;

        float half_field = config.field_length * 0.5f;

        for (uint32_t i = 0u; i < entity_manager.athlete_count; ++i) {
            AthleteEntity& a = entity_manager.athletes[i];
            if (!a.id.is_valid() || !a.is_active) continue;

            // Distance from athlete to ball (XZ plane + Y)
            Vec3 diff = Vec3::sub(ball->position, a.position);
            float dist_xz = std::sqrt(diff.x * diff.x + diff.z * diff.z);
            float dist_y = std::abs(diff.y);

            // Interaction range: athlete radius + ball radius + kick reach
            float kick_range = a.radius + ball->radius + 0.8f;
            float ground_reach = a.height * 0.6f; // Can reach up to ~60% of height

            if (dist_xz < kick_range && dist_y < ground_reach) {
                // --- Athlete is close enough to interact with ball ---

                // Get AI action from stored flags
                AIActionType action = static_cast<AIActionType>(a.flags & 0xFFu);

                // Direction from athlete to ball
                Vec3 to_ball = diff;
                to_ball.y = 0.0f;
                float to_ball_len = Vec3::length(to_ball);
                if (to_ball_len < APC_EPSILON) continue;

                Vec3 to_ball_dir = Vec3::scale(to_ball, 1.0f / to_ball_len);

                // Kick force varies by action
                float kick_force = 0.0f;

                if (action == AIActionType::SHOOT_BALL) {
                    // Shoot: hard kick toward opponent goal
                    float goal_x = (a.team == TEAM_HOME) ? half_field : -half_field;
                    Vec3 to_goal(goal_x - ball->position.x, 0.0f,
                                 0.0f - ball->position.z);
                    float goal_dist = Vec3::length(to_goal);
                    if (goal_dist > APC_EPSILON) {
                        kick_force = 20.0f; // Strong shot
                        Vec3 kick_dir = Vec3::scale(to_goal, 1.0f / goal_dist);
                        ball->velocity.x += kick_dir.x * kick_force;
                        ball->velocity.y += 2.0f; // Slight lift
                        ball->velocity.z += kick_dir.z * kick_force;
                    }
                    ball->last_toucher = a.id;
                    ball->possession_team = a.team;

                } else if (action == AIActionType::CHASE_BALL ||
                           action == AIActionType::PRESS ||
                           action == AIActionType::INTERCEPT) {
                    // Dribble: gentle touch in movement direction
                    Vec3 move_dir = a.current_intent.move_direction;
                    float move_mag = Vec3::length(move_dir);
                    if (move_mag > APC_EPSILON) {
                        kick_force = 5.0f; // Gentle touch
                        Vec3 kick_dir = Vec3::scale(move_dir, 1.0f / move_mag);
                        ball->velocity.x += kick_dir.x * kick_force;
                        ball->velocity.z += kick_dir.z * kick_force;
                    } else {
                        // Just nudge ball forward
                        kick_force = 3.0f;
                        ball->velocity.x += to_ball_dir.x * kick_force;
                        ball->velocity.z += to_ball_dir.z * kick_force;
                    }
                    ball->last_toucher = a.id;
                    ball->possession_team = a.team;

                } else if (action == AIActionType::SUPPORT_RUN ||
                           action == AIActionType::MOVE_TO_POSITION) {
                    // Light touch when passing through
                    if (to_ball_len < a.radius + ball->radius + 0.3f) {
                        kick_force = 2.0f;
                        ball->velocity.x += to_ball_dir.x * kick_force;
                        ball->velocity.z += to_ball_dir.z * kick_force;
                        ball->last_toucher = a.id;
                        ball->possession_team = a.team;
                    }
                }

                // Check for goal: ball past goal line
                if (ball->position.x > half_field + 1.0f) {
                    // Away team scores (or home concedes)
                    if (a.team == TEAM_AWAY) {
                        ++away_score;
                    }
                    // Reset ball to center
                    ball->position = Vec3(0.0f, 0.11f, 0.0f);
                    ball->velocity = Vec3(0.0f, 0.0f, 0.0f);
                    ball->angular_velocity = Vec3(0.0f, 0.0f, 0.0f);
                } else if (ball->position.x < -(half_field + 1.0f)) {
                    // Home team scores (or away concedes)
                    if (a.team == TEAM_HOME) {
                        ++home_score;
                    }
                    // Reset ball to center
                    ball->position = Vec3(0.0f, 0.11f, 0.0f);
                    ball->velocity = Vec3(0.0f, 0.0f, 0.0f);
                    ball->angular_velocity = Vec3(0.0f, 0.0f, 0.0f);
                }
            }
        }
    }

    // =========================================================================
    // update — Main per-frame update (driven by Application::tick)
    // =========================================================================
    void update(float dt)
    {
        if (!is_loaded) return;
        // NOTE: The Application's GameLoop gates physics stepping via
        // should_step_physics(). SceneState does not maintain its own
        // play/pause state — it is driven externally.

        // Step kinematics: motor intent -> velocity -> position
        entity_manager.update_all(dt);

        // Update match time
        match_time_seconds += dt;

        // Check half time (uses match config directly)
        if (config.halves > 0 && config.half_duration > 0.0f) {
            float current_half = match_time_seconds / config.half_duration;
            uint32_t half_index = static_cast<uint32_t>(current_half);
            if (half_index > 0 && half_index >= config.halves) {
                // Full time reached
            }
        }
    }

    // =========================================================================
    // reset_match — Reset positions, scores, time
    // =========================================================================
    void reset_match()
    {
        home_score        = 0u;
        away_score        = 0u;
        match_time_seconds = 0.0f;
        // Note: GameLoop reset is done by Application, not here.

        // Reset all athletes to initial positions
        for (uint32_t i = 0u; i < entity_manager.athlete_count; ++i) {
            AthleteEntity& a = entity_manager.athletes[i];
            if (!a.id.is_valid()) continue;
            a.velocity = { 0.0f, 0.0f, 0.0f };
            a.current_intent.reset();
            a.stamina = 1.0f;
            a.health  = 1.0f;
            a.sprint_cooldown = 0.0f;
            a.tackle_cooldown = 0.0f;
        }

        // Reset ball
        for (uint32_t i = 0u; i < entity_manager.ball_count; ++i) {
            BallEntity& b = entity_manager.balls[i];
            if (!b.id.is_valid()) continue;
            b.velocity = { 0.0f, 0.0f, 0.0f };
            b.angular_velocity = { 0.0f, 0.0f, 0.0f };
            b.position = { 0.0f, 0.11f, 0.0f };
        }
    }

    // =========================================================================
    // Static factory methods — Preset match configurations
    // =========================================================================

    static MatchConfig soccer_match()
    {
        MatchConfig cfg;
        cfg.sport               = SportType::SOCCER;
        cfg.match_duration_seconds = 5400.0f;
        cfg.halves              = 2;
        cfg.half_duration       = 2700.0f;
        cfg.players_per_team    = 11;
        cfg.subs_per_team       = 3;
        cfg.home_formation      = FormationType::FORMATION_4_4_2;
        cfg.away_formation      = FormationType::FORMATION_4_3_3;
        cfg.field_length        = 105.0f;
        cfg.field_width         = 68.0f;
        cfg.offside_enabled     = 1;
        // Set team names
        const char* home = "Home";
        for (uint32_t i = 0u; home[i] && i < 31u; ++i) {
            cfg.home_team_name[i] = home[i];
        }
        cfg.home_team_name[4] = '\0';
        const char* away = "Away";
        for (uint32_t i = 0u; away[i] && i < 31u; ++i) {
            cfg.away_team_name[i] = away[i];
        }
        cfg.away_team_name[4] = '\0';
        return cfg;
    }

    static MatchConfig basketball_match()
    {
        MatchConfig cfg;
        cfg.sport               = SportType::BASKETBALL;
        cfg.match_duration_seconds = 2880.0f;
        cfg.halves              = 4;
        cfg.half_duration       = 720.0f;
        cfg.players_per_team    = 5;
        cfg.subs_per_team       = 3;
        cfg.home_formation      = FormationType::FORMATION_4_4_2;
        cfg.away_formation      = FormationType::FORMATION_4_4_2;
        cfg.field_length        = 28.0f;
        cfg.field_width         = 15.0f;
        cfg.offside_enabled     = 0;
        const char* home = "Home";
        for (uint32_t i = 0u; home[i] && i < 31u; ++i) {
            cfg.home_team_name[i] = home[i];
        }
        cfg.home_team_name[4] = '\0';
        const char* away = "Away";
        for (uint32_t i = 0u; away[i] && i < 31u; ++i) {
            cfg.away_team_name[i] = away[i];
        }
        cfg.away_team_name[4] = '\0';
        return cfg;
    }

    static MatchConfig football_match()
    {
        MatchConfig cfg;
        cfg.sport               = SportType::AMERICAN_FOOTBALL;
        cfg.match_duration_seconds = 3600.0f;
        cfg.halves              = 4;
        cfg.half_duration       = 900.0f;
        cfg.players_per_team    = 11;
        cfg.subs_per_team       = 3;
        cfg.home_formation      = FormationType::FORMATION_4_4_2;
        cfg.away_formation      = FormationType::FORMATION_4_4_2;
        cfg.field_length        = 109.7f;
        cfg.field_width         = 48.8f;
        cfg.offside_enabled     = 0;
        const char* home = "Home";
        for (uint32_t i = 0u; home[i] && i < 31u; ++i) {
            cfg.home_team_name[i] = home[i];
        }
        cfg.home_team_name[4] = '\0';
        const char* away = "Away";
        for (uint32_t i = 0u; away[i] && i < 31u; ++i) {
            cfg.away_team_name[i] = away[i];
        }
        cfg.away_team_name[4] = '\0';
        return cfg;
    }

    static MatchConfig rugby_match()
    {
        MatchConfig cfg;
        cfg.sport               = SportType::RUGBY;
        cfg.match_duration_seconds = 4800.0f;
        cfg.halves              = 2;
        cfg.half_duration       = 2400.0f;
        cfg.players_per_team    = 15;
        cfg.subs_per_team       = 4;
        cfg.home_formation      = FormationType::FORMATION_4_4_2;
        cfg.away_formation      = FormationType::FORMATION_4_4_2;
        cfg.field_length        = 100.0f;
        cfg.field_width         = 70.0f;
        cfg.offside_enabled     = 0;
        const char* home = "Home";
        for (uint32_t i = 0u; home[i] && i < 31u; ++i) {
            cfg.home_team_name[i] = home[i];
        }
        cfg.home_team_name[4] = '\0';
        const char* away = "Away";
        for (uint32_t i = 0u; away[i] && i < 31u; ++i) {
            cfg.away_team_name[i] = away[i];
        }
        cfg.away_team_name[4] = '\0';
        return cfg;
    }
};

} // namespace apc
