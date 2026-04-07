#pragma once
// =============================================================================
// Sport Physics — Top-level sport physics integration manager
// =============================================================================
//
// Integrates all Phase 4 subsystems into a single simulation manager:
//   - BallPhysicsWorld (ball aerodynamics, bounce)
//   - PossessionSystem (who controls the ball)
//   - DribbleController, CatchController (ball control)
//   - KickExecutor, ThrowExecutor (passing/kicking)
//   - ImplementHitResolver (bat/racket hits)
//   - ContactResolver (tackles, blocks)
//   - SportField (playing field geometry)
//   - ScoringSystem, DisciplineSystem, ClockSystem (rules)
//
// This is the main entry point for games to simulate sport physics.
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation
//   - Deterministic: fixed-order updates across all subsystems
//   - C++17
//
// =============================================================================

#include "apc_sport/apc_ball_physics.h"
#include "apc_sport/apc_ball_control.h"
#include "apc_sport/apc_pass_kick.h"
#include "apc_sport/apc_implement.h"
#include "apc_sport/apc_contact_sport.h"
#include "apc_sport/apc_sport_field.h"
#include "apc_sport/apc_sport_rules.h"
#include <cstdint>
#include <cmath>

namespace apc {

// =============================================================================
// SportConfig — Complete sport configuration
// =============================================================================
struct SportConfig {
    SportType sport = SportType::SOCCER;
    const char* name = "soccer";

    // --- Physics ---
    float physics_dt = 1.0f / 240.0f;   // Internal timestep
    float gravity = -9.81f;

    // --- Team sizes ---
    uint32_t players_per_team = 11;

    // --- Ball ---
    BallConfig ball_config;
    float max_ball_speed = 40.0f;

    // --- Field ---
    FieldGeometry field_geometry;
    SurfaceBounceTable surface_table;

    // --- Rules ---
    float match_duration = 5400.0f;       // 90 minutes
    uint32_t periods = 2;
    float period_duration = 2700.0f;

    // --- Contact ---
    float tackle_allowed = true;
    float shoulder_charge_allowed = true;
    float slide_tackle_allowed = true;
    float offside_enabled = true;
    float offside_line = 0.0f;            // 0 = second-last defender

    // --- Factory methods ---
    static SportConfig make_soccer() {
        SportConfig c;
        c.sport = SportType::SOCCER;
        c.name = "soccer";
        c.players_per_team = 11;
        c.ball_config = BallFactory::make_soccer();
        c.max_ball_speed = 40.0f;
        c.field_geometry = FieldGeometry::make_soccer();
        c.surface_table = SurfaceBounceTable::make_default();
        c.match_duration = 5400.0f;
        c.periods = 2;
        c.period_duration = 2700.0f;
        c.tackle_allowed = true;
        c.shoulder_charge_allowed = true;
        c.slide_tackle_allowed = true;
        c.offside_enabled = true;
        return c;
    }

    static SportConfig make_basketball() {
        SportConfig c;
        c.sport = SportType::BASKETBALL;
        c.name = "basketball";
        c.players_per_team = 5;
        c.ball_config = BallFactory::make_basketball();
        c.max_ball_speed = 20.0f;
        c.field_geometry = FieldGeometry::make_basketball();
        c.surface_table = SurfaceBounceTable::make_default();
        c.match_duration = 2880.0f;
        c.periods = 4;
        c.period_duration = 720.0f;
        c.tackle_allowed = false;
        c.shoulder_charge_allowed = false;
        c.slide_tackle_allowed = false;
        c.offside_enabled = false;
        c.offside_line = 0.0f;
        return c;
    }

    static SportConfig make_american_football() {
        SportConfig c;
        c.sport = SportType::AMERICAN_FOOTBALL;
        c.name = "american_football";
        c.players_per_team = 11;
        c.ball_config = BallFactory::make_american_football();
        c.max_ball_speed = 35.0f;
        c.field_geometry = FieldGeometry::make_american_football();
        c.surface_table = SurfaceBounceTable::make_default();
        c.match_duration = 3600.0f;
        c.periods = 4;
        c.period_duration = 900.0f;
        c.tackle_allowed = true;
        c.shoulder_charge_allowed = true;
        c.slide_tackle_allowed = false;
        c.offside_enabled = true;
        c.offside_line = 0.0f;
        return c;
    }

    static SportConfig make_rugby() {
        SportConfig c;
        c.sport = SportType::RUGBY_UNION;
        c.name = "rugby";
        c.players_per_team = 15;
        c.ball_config = BallFactory::make_rugby();
        c.max_ball_speed = 40.0f;
        c.field_geometry = FieldGeometry::make_rugby();
        c.surface_table = SurfaceBounceTable::make_default();
        c.match_duration = 4800.0f;
        c.periods = 2;
        c.period_duration = 2400.0f;
        c.tackle_allowed = true;
        c.shoulder_charge_allowed = true;
        c.slide_tackle_allowed = false;
        c.offside_enabled = true;
        return c;
    }

    static SportConfig make_tennis() {
        SportConfig c;
        c.sport = SportType::TENNIS;
        c.name = "tennis";
        c.players_per_team = 1;
        c.ball_config = BallFactory::make_tennis();
        c.max_ball_speed = 70.0f;
        c.field_geometry = FieldGeometry::make_tennis();
        c.surface_table = SurfaceBounceTable::make_default();
        c.match_duration = 0.0f; // Tennis is score-based, not time-based
        c.periods = 3; // Best of 3 sets
        c.tackle_allowed = false;
        c.shoulder_charge_allowed = false;
        c.slide_tackle_allowed = false;
        c.offside_enabled = false;
        return c;
    }

    static SportConfig make_ice_hockey() {
        SportConfig c;
        c.sport = SportType::ICE_HOCKEY;
        c.name = "ice_hockey";
        c.players_per_team = 6;
        c.ball_config = BallFactory::make_soccer(); // Puck approximated as sphere
        c.ball_config.radius = 0.038f;             // Puck dimensions
        c.ball_config.mass = 0.17f;
        c.ball_config.drag_coefficient = 0.1f;     // Puck has low drag
        c.max_ball_speed = 45.0f;
        c.field_geometry = FieldGeometry::make_ice_hockey();
        c.surface_table = SurfaceBounceTable::make_default();
        c.match_duration = 3600.0f;
        c.periods = 3;
        c.period_duration = 1200.0f;
        c.tackle_allowed = true;
        c.shoulder_charge_allowed = true;
        c.slide_tackle_allowed = false;
        c.offside_enabled = true;
        return c;
    }
};

// =============================================================================
// SportPhysicsWorld — Complete sport physics simulation
// =============================================================================
struct SportPhysicsWorld {
    // --- Configuration ---
    SportConfig config;

    // --- Subsystems ---
    BallPhysicsWorld ball_world;
    PossessionSystem possession;
    DribbleController dribble;
    CatchController catcher;
    KickExecutor kicker;
    ThrowExecutor thrower;
    ImplementHitResolver implement_resolver;
    ImplementSwingModel swing_model;
    ContactResolver contact_resolver;
    GrappleResolver grapple_resolver;

    // --- Field & Rules ---
    SportField field;
    ScoringSystem scoring;
    DisciplineSystem discipline;
    ClockSystem clock;

    // --- State ---
    PlayState play_state = PlayState::NOT_STARTED;
    uint32_t primary_ball_id = 0;
    float current_time = 0.0f;
    uint32_t frame_count = 0;

    // --- Initialize for a given sport ---
    void initialize(const SportConfig& sport_config) {
        config = sport_config;

        // Setup ball world
        ball_world.gravity_y = config.gravity;
        ball_world.surface_table = config.surface_table;

        // Setup field
        field.setup(config.field_geometry);
        field.surface_table = config.surface_table;

        // Add default goals for rectangular fields
        if (config.field_geometry.type == FieldType::RECTANGLE) {
            float half_l = config.field_geometry.length * 0.5f;
            float goal_h = config.field_geometry.goal_height;
            float goal_w = config.field_geometry.goal_width;
            float goal_d = config.field_geometry.goal_depth;

            // Home goal (negative x end)
            GoalPost home_goal;
            home_goal.position = Vec3(-half_l, 0.0f, 0.0f);
            home_goal.opening_center = Vec3(-half_l, goal_h * 0.5f, 0.0f);
            home_goal.width = goal_w;
            home_goal.height = goal_h;
            home_goal.depth = goal_d;
            home_goal.facing_direction = Vec3(1.0f, 0.0f, 0.0f);
            home_goal.team_id = 0;
            field.add_goal(home_goal);

            // Away goal (positive x end)
            GoalPost away_goal;
            away_goal.position = Vec3(half_l, 0.0f, 0.0f);
            away_goal.opening_center = Vec3(half_l, goal_h * 0.5f, 0.0f);
            away_goal.width = goal_w;
            away_goal.height = goal_h;
            away_goal.depth = goal_d;
            away_goal.facing_direction = Vec3(-1.0f, 0.0f, 0.0f);
            away_goal.team_id = 1;
            field.add_goal(away_goal);
        }

        // Setup clock
        clock.period_duration = config.period_duration;
        clock.total_duration = config.match_duration;
        clock.total_periods = config.periods;

        // Reset state
        possession.reset();
        scoring.reset();
        discipline.reset();
        grapple_resolver.reset();
        current_time = 0.0f;
        frame_count = 0;
        play_state = PlayState::NOT_STARTED;
    }

    // --- Create the primary ball ---
    uint32_t create_ball(const BallConfig& ball_config = BallConfig()) {
        BallConfig bc = (ball_config.mass > 0.0f) ? ball_config : config.ball_config;
        uint32_t id = ball_world.add_ball(bc);
        if (id != 0xFFFFFFFF) {
            possession.register_ball(id);
            primary_ball_id = id;
        }
        return id;
    }

    // --- Main simulation step ---
    void step(float dt) {
        if (play_state != PlayState::LIVE) {
            // Still update clock if timeout/injury
            if (play_state == PlayState::TIMEOUT ||
                play_state == PlayState::INJURY_STOP)
            {
                clock.update(dt);
            }
            return;
        }

        float phys_dt = config.physics_dt;
        float steps = dt / phys_dt;
        uint32_t num_steps = static_cast<uint32_t>(steps);

        for (uint32_t s = 0; s < num_steps; ++s) {
            // 1. Physics: step ball world (gravity + aerodynamics + ground bounce)
            ball_world.gravity_y = config.gravity;
            ball_world.step(phys_dt);

            // 2. Possession update
            possession.update(phys_dt);

            // 3. Grapple update
            grapple_resolver.update(phys_dt, 0.5f, 0.5f);

            // 4. Clock
            clock.update(phys_dt);

            current_time += phys_dt;
            ++frame_count;
        }

        // 5. Field boundary checks (once per frame, not per physics step)
        BallState* ball = ball_world.get_ball(primary_ball_id);
        if (ball) {
            BoundaryEvent evt = field.check_boundary(
                ball->body.position,
                ball->body.linear_velocity,
                ball->config.get_effective_radius(),
                primary_ball_id,
                current_time
            );

            if (evt.type == BoundaryEventType::GOAL_SCORED) {
                scoring.add_score(evt.team_id, 1.0f, "goal",
                    current_time, evt.position);
                play_state = PlayState::GOAL_SCORED;
            }
            else if (evt.type == BoundaryEventType::OUT_OF_BOUNDS) {
                play_state = PlayState::DEAD_BALL;
            }
            else if (evt.type == BoundaryEventType::HIT_POST) {
                // Ball bounces off post — physics handles the bounce
                // Could trigger sound/visual effects via game hooks
            }
        }

        // 6. Period/game over checks
        if (clock.is_game_over() && play_state == PlayState::LIVE) {
            play_state = PlayState::GAME_OVER;
        }
    }

    // --- Convenience: kick the primary ball ---
    PassResult kick(const Vec3& direction, float power, float skill = 0.8f) {
        BallState* ball = ball_world.get_ball(primary_ball_id);
        if (!ball) {
            PassResult r;
            r.executed = false;
            return r;
        }

        KickProfile profile = KickProfile::make_instep();
        Vec3 athlete_pos = ball->body.position; // Simplified: kick from ball pos
        Vec3 forward = direction;

        possession.release(primary_ball_id, ControlState::IN_FLIGHT);
        return kicker.execute(*ball, profile, athlete_pos, forward,
                               direction, power, skill);
    }

    // --- Convenience: throw the primary ball ---
    PassResult throw_ball(const Vec3& hand_pos, const Vec3& hand_vel,
                           const Vec3& direction, float power,
                           float skill = 0.8f)
    {
        BallState* ball = ball_world.get_ball(primary_ball_id);
        if (!ball) {
            PassResult r;
            r.executed = false;
            return r;
        }

        ThrowProfile profile = ThrowProfile::make_chest_pass();
        possession.release(primary_ball_id, ControlState::IN_FLIGHT);
        return thrower.execute(*ball, profile, hand_pos, hand_vel,
                               direction, power, skill);
    }

    // --- Convenience: resolve a tackle ---
    ContactResult tackle(uint32_t attacker_id, uint32_t defender_id,
                          const Vec3& attacker_pos, const Vec3& attacker_vel,
                          const Vec3& defender_pos, const Vec3& defender_vel,
                          TackleType type = TackleType::WRAP)
    {
        TackleProfile profile;
        switch (type) {
        case TackleType::DIVE_SLIDE:
            profile = TackleProfile::make_slide_tackle(); break;
        case TackleType::MID:
            profile = TackleProfile::make_shoulder_tackle(); break;
        default:
            profile = TackleProfile::make_form_tackle(); break;
        }

        ContactResult result = contact_resolver.resolve_tackle(
            attacker_pos, attacker_vel,
            defender_pos, defender_vel,
            profile, 0.8f, 0.8f
        );

        // Check fumble
        if (result.ball_dislodged) {
            possession.release(primary_ball_id, ControlState::FUMBLING);
        }

        // Check foul
        if (result.penalty_flagged) {
            discipline.record_foul(
                FoulType::TACKLE_FROM_BEHIND,
                0, attacker_id, 1, defender_id,
                current_time, defender_pos
            );
        }

        return result;
    }

    // --- Convenience: hit ball with implement ---
    HitResult hit_with_implement(const ImplementProfile& impl,
                                   const Vec3& implement_pos,
                                   const Vec3& implement_vel,
                                   const Vec3& face_normal,
                                   float contact_offset)
    {
        BallState* ball = ball_world.get_ball(primary_ball_id);
        if (!ball) {
            HitResult r;
            r.hit = false;
            return r;
        }

        possession.release(primary_ball_id, ControlState::IN_FLIGHT);
        return implement_resolver.resolve(*ball, impl, implement_vel,
            face_normal, implement_pos, contact_offset, 0.8f);
    }

    // --- Get current score ---
    float get_home_score() const { return scoring.scores[0]; }
    float get_away_score() const { return scoring.scores[1]; }

    // --- Start/restart play ---
    void start_play() {
        play_state = PlayState::LIVE;
        clock.start();
    }

    void stop_play() {
        play_state = PlayState::DEAD_BALL;
        clock.stop();
    }

    // --- Reset everything ---
    void reset() {
        ball_world.ball_count = 0;
        possession.reset();
        scoring.reset();
        discipline.reset();
        grapple_resolver.reset();
        clock = ClockSystem();
        current_time = 0.0f;
        frame_count = 0;
        play_state = PlayState::NOT_STARTED;
        primary_ball_id = 0;
    }
};

// =============================================================================
// SportPhysicsFactory — Quick-create sport physics worlds
// =============================================================================
struct SportPhysicsFactory {
    static SportPhysicsWorld create_soccer() {
        SportPhysicsWorld world;
        world.initialize(SportConfig::make_soccer());
        world.create_ball(BallFactory::make_soccer());
        world.scoring.configure_soccer();
        world.clock.configure_soccer();
        return world;
    }

    static SportPhysicsWorld create_basketball() {
        SportPhysicsWorld world;
        world.initialize(SportConfig::make_basketball());
        world.create_ball(BallFactory::make_basketball());
        world.scoring.configure_basketball();
        world.clock.configure_basketball();
        return world;
    }

    static SportPhysicsWorld create_american_football() {
        SportPhysicsWorld world;
        world.initialize(SportConfig::make_american_football());
        world.create_ball(BallFactory::make_american_football());
        world.scoring.configure_american_football();
        world.clock.configure_american_football();
        return world;
    }

    static SportPhysicsWorld create_rugby() {
        SportPhysicsWorld world;
        world.initialize(SportConfig::make_rugby());
        world.create_ball(BallFactory::make_rugby());
        world.scoring.configure_rugby_union();
        return world;
    }

    static SportPhysicsWorld create_tennis() {
        SportPhysicsWorld world;
        world.initialize(SportConfig::make_tennis());
        world.create_ball(BallFactory::make_tennis());
        return world;
    }

    static SportPhysicsWorld create_ice_hockey() {
        SportPhysicsWorld world;
        world.initialize(SportConfig::make_ice_hockey());
        world.create_ball();
        return world;
    }
};

} // namespace apc
