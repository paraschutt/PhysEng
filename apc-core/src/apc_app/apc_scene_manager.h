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

    // --- Ball possession tracking ---
    EntityId  last_ball_toucher   = EntityId::make_invalid();
    TeamId    last_possession_team = TEAM_NONE;
    float     possession_timer     = 0.0f; // Seconds since last touch
    static constexpr float POSSESSION_WINDOW = 1.5f; // Ball stays "controlled" for 1.5s
    uint8_t   ball_in_play         = 1u;
    float     out_of_bounds_timer  = 0.0f; // Countdown before reset

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
        last_ball_toucher   = EntityId::make_invalid();
        last_possession_team = TEAM_NONE;
        possession_timer     = 0.0f;
        ball_in_play         = 1u;
        out_of_bounds_timer  = 0.0f;

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
        utility_ai[0].actions[utility_ai[0].action_count++] = AIActionType::BLOCK;
        utility_ai[0].actions[utility_ai[0].action_count++] = AIActionType::MARK_OPPONENT;
        utility_ai[0].actions[utility_ai[0].action_count++] = AIActionType::CROSS;
        utility_ai[0].actions[utility_ai[0].action_count++] = AIActionType::HEADER;
        utility_ai[0].actions[utility_ai[0].action_count++] = AIActionType::DIVE_SAVE;
        utility_ai[0].actions[utility_ai[0].action_count++] = AIActionType::PUNT;

        utility_ai[1].actions[utility_ai[1].action_count++] = AIActionType::FORMATION_HOLD;
        utility_ai[1].actions[utility_ai[1].action_count++] = AIActionType::CHASE_BALL;
        utility_ai[1].actions[utility_ai[1].action_count++] = AIActionType::SHOOT_BALL;
        utility_ai[1].actions[utility_ai[1].action_count++] = AIActionType::MOVE_TO_POSITION;
        utility_ai[1].actions[utility_ai[1].action_count++] = AIActionType::SUPPORT_RUN;
        utility_ai[1].actions[utility_ai[1].action_count++] = AIActionType::PRESS;
        utility_ai[1].actions[utility_ai[1].action_count++] = AIActionType::INTERCEPT;
        utility_ai[1].actions[utility_ai[1].action_count++] = AIActionType::TACKLE;
        utility_ai[1].actions[utility_ai[1].action_count++] = AIActionType::PASS_BALL;
        utility_ai[1].actions[utility_ai[1].action_count++] = AIActionType::BLOCK;
        utility_ai[1].actions[utility_ai[1].action_count++] = AIActionType::MARK_OPPONENT;
        utility_ai[1].actions[utility_ai[1].action_count++] = AIActionType::CROSS;
        utility_ai[1].actions[utility_ai[1].action_count++] = AIActionType::HEADER;
        utility_ai[1].actions[utility_ai[1].action_count++] = AIActionType::DIVE_SAVE;
        utility_ai[1].actions[utility_ai[1].action_count++] = AIActionType::PUNT;

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
        float half_width = config.field_width * 0.5f;
        float goal_half_w = 3.66f; // Half of 7.32m goal width

        // --- Goal check (takes priority over OOB) ---
        // A ball past the goal line AND within the goal posts = goal.
        // A ball past any boundary but NOT in goal = out of bounds.
        // Both use the actual field boundary — no extra margin needed
        // since heavy friction already prevents the ball from traveling far.
        if (ball_in_play) {
            bool in_goal = false;
            if (std::abs(ball->position.z) < goal_half_w) {
                if (ball->position.x > half_field) {
                    in_goal = true;
                    ++away_score;
                } else if (ball->position.x < -half_field) {
                    in_goal = true;
                    ++home_score;
                }
            }

            // Out of bounds — ball center past any field edge (not in goal)
            bool oob = false;
            if (!in_goal) {
                if (ball->position.x < -half_field || ball->position.x > half_field) {
                    oob = true; // Beyond goal line (outside goal posts)
                } else if (ball->position.z < -half_width || ball->position.z > half_width) {
                    oob = true; // Beyond touchline
                }
            }

            if (in_goal) {
                // Goal scored — reset to center
                ball->position = Vec3(0.0f, 0.11f, 0.0f);
                ball->velocity = Vec3(0.0f, 0.0f, 0.0f);
                ball->angular_velocity = Vec3(0.0f, 0.0f, 0.0f);
                ball->last_toucher = EntityId::make_invalid();
                ball->possession_team = TEAM_NONE;
                last_ball_toucher = EntityId::make_invalid();
                last_possession_team = TEAM_NONE;
                possession_timer = 0.0f;
            } else if (oob) {
                // Out of bounds — stop ball and reset after brief pause
                ball->velocity = Vec3(0.0f, 0.0f, 0.0f);
                ball_in_play = 0;
                out_of_bounds_timer = 0.5f; // 0.5s pause then reset
            }
        } else {
            // Ball is out of play — count down then reset
            out_of_bounds_timer -= game_loop_dt;
            if (out_of_bounds_timer <= 0.0f) {
                // Reset ball to nearest touchline/goal-line point
                float bx = ball->position.x;
                float bz = ball->position.z;
                // Clamp to field boundary
                if (bx < -half_field) bx = -half_field;
                else if (bx > half_field) bx = half_field;
                if (bz < -half_width) bz = -half_width;
                else if (bz > half_width) bz = half_width;
                // If beyond goal line, reset to center instead
                if (std::abs(bx) >= half_field) {
                    bx = 0.0f; bz = 0.0f;
                }
                ball->position = Vec3(bx, 0.11f, bz);
                ball->velocity = Vec3(0.0f, 0.0f, 0.0f);
                ball->possession_team = TEAM_NONE;
                last_possession_team = TEAM_NONE;
                possession_timer = 0.0f;
                ball_in_play = 1u;
            }
            return; // Skip athlete interactions while out of play
        }

        // --- Ball interaction exclusivity ---
        // Per physics step, only ONE player per team can apply force to the ball
        // (the closest on their team). Opponents can only TACKLE/INTERCEPT to
        // contest possession.

        // First pass: find closest athlete on each team within kick range
        int32_t closest_home_idx = -1;
        int32_t closest_away_idx = -1;
        float closest_home_dist = 999.0f;
        float closest_away_dist = 999.0f;
        float closest_home_dist_xz = 999.0f;
        float closest_away_dist_xz = 999.0f;

        for (uint32_t i = 0u; i < entity_manager.athlete_count; ++i) {
            const AthleteEntity& a = entity_manager.athletes[i];
            if (!a.id.is_valid() || !a.is_active) continue;

            Vec3 diff = Vec3::sub(ball->position, a.position);
            float dist_xz = std::sqrt(diff.x * diff.x + diff.z * diff.z);
            float dist_y = std::abs(diff.y);
            float kick_range = a.radius + ball->radius + 0.8f;
            float ground_reach = a.height * 0.6f;

            if (dist_xz < kick_range && dist_y < ground_reach) {
                float dist_3d = Vec3::length(diff);
                if (a.team == TEAM_HOME) {
                    if (dist_xz < closest_home_dist_xz) {
                        closest_home_dist_xz = dist_xz;
                        closest_home_dist = dist_3d;
                        closest_home_idx = static_cast<int32_t>(i);
                    }
                } else if (a.team == TEAM_AWAY) {
                    if (dist_xz < closest_away_dist_xz) {
                        closest_away_dist_xz = dist_xz;
                        closest_away_dist = dist_3d;
                        closest_away_idx = static_cast<int32_t>(i);
                    }
                }
            }
        }

        // Helper lambda: apply kick from an athlete to ball
        // Returns true if a kick was applied
        const auto apply_kick = [&](uint32_t idx, TeamId controlling_team) -> bool {
            AthleteEntity& a = entity_manager.athletes[idx];
            if (!a.id.is_valid() || !a.is_active) return false;

            Vec3 diff = Vec3::sub(ball->position, a.position);
            float dist_xz = std::sqrt(diff.x * diff.x + diff.z * diff.z);
            float dist_y = std::abs(diff.y);
            float kick_range = a.radius + ball->radius + 0.8f;
            float ground_reach = a.height * 0.6f;

            if (dist_xz >= kick_range || dist_y >= ground_reach) return false;

            Vec3 to_ball = diff;
            to_ball.y = 0.0f;
            float to_ball_len = Vec3::length(to_ball);
            if (to_ball_len < APC_EPSILON) return false;

            Vec3 to_ball_dir = Vec3::scale(to_ball, 1.0f / to_ball_len);
            AIActionType action = static_cast<AIActionType>(a.flags & 0xFFu);
            float kick_force = 0.0f;

            // Possession team can do offensive actions
            // Opponents can only TACKLE/INTERCEPT (which dispossesses)
            bool is_opponent = (a.team != controlling_team);
            bool ball_is_free = (controlling_team == TEAM_NONE);

            if (is_opponent) {
                // Opponents can only TACKLE or INTERCEPT
                if (action == AIActionType::TACKLE || action == AIActionType::INTERCEPT) {
                    // Tackle: dispossess the ball
                    ball->possession_team = a.team;
                    last_ball_toucher = a.id;
                    last_possession_team = a.team;
                    possession_timer = POSSESSION_WINDOW;
                    // Small deflection on tackle
                    kick_force = 2.0f;
                    ball->velocity.x += to_ball_dir.x * kick_force;
                    ball->velocity.z += to_ball_dir.z * kick_force;
                    return true;
                }
                return false; // Other actions blocked for opponents
            }

            // This athlete is on the controlling team (or ball is free)
            // Update possession tracking
            last_ball_toucher = a.id;
            last_possession_team = a.team;
            possession_timer = POSSESSION_WINDOW;
            if (ball_is_free) {
                ball->possession_team = a.team; // Claim possession
            }

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

            } else if (action == AIActionType::CHASE_BALL ||
                       action == AIActionType::PRESS ||
                       action == AIActionType::INTERCEPT) {
                // Dribble: gentle touch in movement direction
                Vec3 move_dir = a.current_intent.move_direction;
                float move_mag = Vec3::length(move_dir);
                if (move_mag > APC_EPSILON) {
                    kick_force = 4.0f; // Gentle touch
                    Vec3 kick_dir = Vec3::scale(move_dir, 1.0f / move_mag);
                    ball->velocity.x += kick_dir.x * kick_force;
                    ball->velocity.z += kick_dir.z * kick_force;
                } else {
                    // Just nudge ball forward
                    kick_force = 2.5f;
                    ball->velocity.x += to_ball_dir.x * kick_force;
                    ball->velocity.z += to_ball_dir.z * kick_force;
                }

            } else if (action == AIActionType::SUPPORT_RUN ||
                       action == AIActionType::MOVE_TO_POSITION ||
                       action == AIActionType::FORMATION_HOLD) {
                // Light touch when passing through
                if (to_ball_len < a.radius + ball->radius + 0.3f) {
                    kick_force = 2.0f;
                    ball->velocity.x += to_ball_dir.x * kick_force;
                    ball->velocity.z += to_ball_dir.z * kick_force;
                }
            }

            // Limit ball max speed to prevent runaway
            float ball_speed = Vec3::length(ball->velocity);
            static constexpr float MAX_BALL_SPEED = 35.0f; // ~126 km/h
            if (ball_speed > MAX_BALL_SPEED) {
                float scale = MAX_BALL_SPEED / ball_speed;
                ball->velocity.x *= scale;
                ball->velocity.y *= scale;
                ball->velocity.z *= scale;
            }

            return kick_force > 0.0f;
        };

        // Apply kicks: the closest on each team gets to interact
        // Ball is controlled by the team with possession_team, or TEAM_NONE if free
        TeamId controlling_team = ball->possession_team;

        // Home team's closest player interacts first
        if (closest_home_idx >= 0) {
            apply_kick(static_cast<uint32_t>(closest_home_idx), controlling_team);
        }
        // Away team's closest player interacts (may have updated controlling_team above)
        if (closest_away_idx >= 0) {
            apply_kick(static_cast<uint32_t>(closest_away_idx), ball->possession_team);
        }

        // --- Extra deceleration for slow-moving ball (simulates grass/turf grip) ---
        {
            float ball_speed_xz = std::sqrt(ball->velocity.x * ball->velocity.x + ball->velocity.z * ball->velocity.z);
            if (ball_speed_xz > 0.01f && ball_speed_xz < 3.0f && ball->position.y <= ball->radius + 0.05f) {
                float slow_friction = 1.5f * game_loop_dt; // Extra grip at low speeds
                if (slow_friction > 0.95f) slow_friction = 0.95f;
                float new_speed = ball_speed_xz * (1.0f - slow_friction);
                if (new_speed < 0.01f) new_speed = 0.0f;
                float scale = new_speed / ball_speed_xz;
                ball->velocity.x *= scale;
                ball->velocity.z *= scale;
            }
        }

        // --- Update possession timer ---
        // (Possession is also set by apply_kick above on each touch)
        if (possession_timer > 0.0f) {
            possession_timer -= game_loop_dt;
            if (possession_timer <= 0.0f) {
                possession_timer = 0.0f;
                ball->possession_team = TEAM_NONE;
            }
        }
    }

    // =========================================================================
    // update — Main per-frame update (driven by Application::tick)
    // =========================================================================
    float game_loop_dt = 0.0f; // Set by Application::tick() before calling update()

    void update(float dt)
    {
        if (!is_loaded) return;
        game_loop_dt = dt;

        // Step kinematics: motor intent -> velocity -> position
        entity_manager.update_all(dt);

        // --- Clamp athletes to field boundary (with small margin) ---
        float half_field = config.field_length * 0.5f;
        float half_width = config.field_width * 0.5f;
        float margin = 2.0f;
        for (uint32_t i = 0u; i < entity_manager.athlete_count; ++i) {
            AthleteEntity& a = entity_manager.athletes[i];
            if (!a.id.is_valid() || !a.is_active) continue;
            if (a.position.x < -(half_field + margin)) {
                a.position.x = -(half_field + margin);
                a.velocity.x = 0.0f;
            }
            if (a.position.x > (half_field + margin)) {
                a.position.x = (half_field + margin);
                a.velocity.x = 0.0f;
            }
            if (a.position.z < -(half_width + margin)) {
                a.position.z = -(half_width + margin);
                a.velocity.z = 0.0f;
            }
            if (a.position.z > (half_width + margin)) {
                a.position.z = (half_width + margin);
                a.velocity.z = 0.0f;
            }
        }

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
