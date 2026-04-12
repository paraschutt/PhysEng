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
#include "apc_app/apc_sport_config.h"
#include "apc_entity/apc_entity_types.h"
#include "apc_entity/apc_entity_manager.h"
#include "apc_ai/apc_ai_formation.h"
#include "apc_ai/apc_ai_motor.h"
#include "apc_ai/apc_ai_decision.h"
#include "apc_ai/apc_ai_perception.h"  // PerceptionRingBuffer (Phase 11b Action 6)
#include "apc_ai/apc_ai_influence_map.h" // InfluenceMap (Phase 11b Action 7)
#include "apc_ai/apc_ai_steering.h"
#include "apc_input/apc_input_types.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_math_common.h"
#include <cstdint>
#include <cmath>

namespace apc {

// =============================================================================
// SeededXorShift32 — Minimal deterministic PRNG for seed-driven variation
// =============================================================================
// Zero-allocation, header-only. Same seed always produces the same sequence.
struct SeededXorShift32 {
    uint32_t state = 1u;

    explicit SeededXorShift32(uint32_t seed = 1u)
    {
        state = seed ? seed : 1u;
    }

    // Returns a pseudo-random float in [-1, 1]
    float next_f() {
        state ^= state << 13u;
        state ^= state >> 17u;
        state ^= state << 5u;
        // Map uint32 to [-1, 1] float
        int32_t signed_val = static_cast<int32_t>(state);
        return static_cast<float>(signed_val) / static_cast<float>(0x7FFFFFFF);
    }
};

// =============================================================================
// SportType — Reuse from apc_sport_field.h (pulled via apc_sport_config.h)
// =============================================================================
// Uses the comprehensive 28-value SportType from the field system.
// Scene-level mappings:
//   SOCCER = 0, BASKETBALL = 1, AMERICAN_FOOTBALL = 2,
//   RUGBY_UNION = 3, ICE_HOCKEY = 9
// =============================================================================

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
    uint32_t      seed                = 42u; // Seed for deterministic variation
    float         position_jitter     = 3.0f; // Max meters of random offset per player

    // =====================================================================
    // Static factory methods — sport-specific match presets
    // =====================================================================
    // Phase 12.5: The Multi-Sport Sandbox. These presets provide the exact
    // physical and semantic differences between Soccer and Basketball so the
    // engine can rebuild the world on command via switch_sport().
    //
    // Soccer:  68x105m field, 11v11, 2 halves of 45 min, offside active
    // Basketball: 15x28m court, 11v11 (engine-internal), 4 quarters, no offside
    // =====================================================================

    static MatchConfig soccer_sandbox()
    {
        MatchConfig cfg;
        cfg.sport               = SportType::SOCCER;
        cfg.match_duration_seconds = 5400.0f; // 90 minutes
        cfg.halves              = 2;
        cfg.half_duration       = 2700.0f;    // 45 min per half
        cfg.players_per_team    = 11;
        cfg.home_formation      = FormationType::FORMATION_4_4_2;
        cfg.away_formation      = FormationType::FORMATION_4_4_2;
        cfg.field_length        = 105.0f;     // Standard FIFA pitch
        cfg.field_width         = 68.0f;
        cfg.offside_enabled     = 1;
        cfg.position_jitter     = 3.0f;
        // Team names
        const char* hn = "Home FC";
        const char* an = "Away United";
        for (uint32_t i = 0u; hn[i] && i < 31u; ++i) { cfg.home_team_name[i] = hn[i]; }
        for (uint32_t i = 0u; an[i] && i < 31u; ++i) { cfg.away_team_name[i] = an[i]; }
        return cfg;
    }

    static MatchConfig basketball_sandbox()
    {
        MatchConfig cfg;
        cfg.sport               = SportType::BASKETBALL;
        cfg.match_duration_seconds = 2400.0f; // 40 minutes (4 × 10 min quarters)
        cfg.halves              = 4;
        cfg.half_duration       = 600.0f;     // 10 min per quarter
        cfg.players_per_team    = 5;          // Basketball 5v5 (capped by formation)
        cfg.home_formation      = FormationType::FORMATION_4_4_2;
        cfg.away_formation      = FormationType::FORMATION_4_4_2;
        cfg.field_length        = 28.0f;      // FIBA half-court × 2 = full court
        cfg.field_width         = 15.0f;      // Standard FIBA court width
        cfg.offside_enabled     = 0;          // No offside in basketball
        cfg.position_jitter     = 1.0f;       // Less jitter on smaller court
        const char* hn = "Home Bears";
        const char* an = "Away Wolves";
        for (uint32_t i = 0u; hn[i] && i < 31u; ++i) { cfg.home_team_name[i] = hn[i]; }
        for (uint32_t i = 0u; an[i] && i < 31u; ++i) { cfg.away_team_name[i] = an[i]; }
        return cfg;
    }
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

    // --- Perception ring buffer for AI reaction delay (Phase 11b Action 6) ---
    // Snapshots the world state every 60Hz tick; AI queries delayed frames
    // based on each athlete's reaction_frames stat (default: 12 = 200ms).
    PerceptionRingBuffer perception_buffer;

    // --- Semantic influence map for AI spatial awareness (Phase 11b Action 7) ---
    // Low-res 32x16 grid: threat = opponent proximity, control = friendly presence.
    // Built from delayed perception snapshots to match AI's reaction delay.
    // AI queries find_best_open_space() to navigate using semantic field rules.
    InfluenceMap influence_map;

    // --- Match configuration ---
    MatchConfig      config;

    // --- Match flow & clock state (Phase 15 Action 1) ---
    // The referee / clock system. SemanticPlayState is only set by
    // update_match_flow() or external hot-key triggers; the AI reads
    // rules.current_state on its next perception tick to decide whether
    // to chase the ball or hold position.
    SportRulesConfig  rules;
    uint32_t          current_period      = 1u;
    float             period_time_seconds = 0.0f;
    float             max_period_duration = 300.0f; // 5 min per half / quarter

    // --- Sport field with semantic zones (Phase 14 Action 1) ---
    // Populated by setup_field() during load_match(). Accessible at render
    // time for debug visualization and at AI time for spatial queries.
    SportField       field;

    // --- Seed-driven PRNG for initial position jitter ---
    SeededXorShift32 rng;

    // --- State ---
    uint8_t  is_loaded         = 0;
    uint32_t home_score        = 0u;
    uint32_t away_score        = 0u;
    float    match_time_seconds = 0.0f; // Global continuous tracker (only ticks during LIVE_PLAY)

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

        // Initialize seed-driven PRNG for position jitter
        rng = SeededXorShift32(config.seed);

        // --- Build the SportField with geometry + semantic zones ---
        setup_field();

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
        field = SportField();
        config = MatchConfig();
        is_loaded           = 0;
        home_score          = 0u;
        away_score          = 0u;
        match_time_seconds  = 0.0f;
        current_period      = 1u;
        period_time_seconds = 0.0f;
        max_period_duration = 300.0f;
        rules               = SportRulesConfig();
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
        perception_buffer.clear();
        influence_map.clear();
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

            // Apply seed-driven position jitter for variation
            if (config.position_jitter > 0.0f) {
                world_pos.x += rng.next_f() * config.position_jitter;
                world_pos.z += rng.next_f() * config.position_jitter;
            }

            entity_manager.spawn_athlete(team, pos.compatible_role,
                                          world_pos, i + 1);

            // Phase 15 Action 2: Store home position for deterministic reset.
            // The last spawned athlete is at index athlete_count - 1.
            uint32_t idx = entity_manager.athlete_count - 1u;
            entity_manager.athletes[idx].home_position = world_pos;
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
        case SportType::RUGBY_UNION:      ball_type = 3; break;
        default: ball_type = 0; break;
        }
        entity_manager.spawn_ball(ball_type, Vec3(0.0f, 0.11f, 0.0f));
    }

    // =========================================================================
    // setup_field — Initialize SportField geometry + semantic zones (Phase 14)
    // =========================================================================
    // Reads config.sport / config.field_length / config.field_width to build
    // a SportField with proper geometry and semantic zone overlay.
    //
    // Semantic zones enable:
    //   - AI spatial reasoning (RESTRICTED_DEFENSE penalty in influence map)
    //   - Debug visualization (zone wireframes in SDL2 renderer)
    //   - Rule enforcement (OUT_OF_BOUNDS detection)
    //
    // Currently supports: SOCCER, BASKETBALL, AMERICAN_FOOTBALL, RUGBY.
    // Other sports get a generic NEUTRAL_PLAYING_FIELD + boundary zones.
    // =========================================================================
    void setup_field()
    {
        FieldGeometry geo;
        geo.sport = config.sport;
        geo.length = config.field_length;
        geo.width  = config.field_width;

        switch (config.sport) {
        case SportType::SOCCER:
            geo.name              = "soccer";
            geo.center_radius     = 9.15f;
            geo.goal_width        = 7.32f;
            geo.goal_height       = 2.44f;
            geo.goal_depth        = 2.0f;
            geo.penalty_area_width = 40.32f;
            geo.penalty_area_depth = 16.5f;
            geo.goal_area_width   = 18.32f;
            geo.goal_area_depth   = 5.5f;
            break;

        case SportType::BASKETBALL:
            geo.name              = "basketball";
            geo.center_radius     = 1.83f;  // 6ft center circle
            geo.goal_width        = 0.457f; // 18-inch rim diameter
            geo.goal_height       = 3.05f;  // 10ft rim height
            geo.goal_depth        = 0.0f;
            geo.penalty_area_width = 4.88f;  // 16ft lane width
            geo.penalty_area_depth = 5.79f;  // 19ft free-throw distance
            geo.goal_area_width   = 0.0f;
            geo.goal_area_depth   = 0.0f;
            break;

        case SportType::AMERICAN_FOOTBALL:
            geo.name              = "american_football";
            geo.center_radius     = 0.0f;
            geo.goal_width        = 5.64f;  // 18ft 6in goal width
            geo.goal_height       = 3.05f;
            geo.goal_depth        = 0.0f;
            geo.penalty_area_width = 0.0f;
            geo.penalty_area_depth = 0.0f;
            geo.goal_area_width   = 0.0f;
            geo.goal_area_depth   = 0.0f;
            break;

        case SportType::RUGBY_UNION:
            geo.name              = "rugby";
            geo.center_radius     = 0.0f;
            geo.goal_width        = 5.6f;
            geo.goal_height       = 3.0f;
            geo.goal_depth        = 0.0f;
            geo.penalty_area_width = 22.0f;  // 22m in-goal width
            geo.penalty_area_depth = 22.0f;
            geo.goal_area_width   = 0.0f;
            geo.goal_area_depth   = 0.0f;
            break;

        default:
            geo.name = "generic";
            geo.center_radius = 5.0f;
            geo.goal_width    = 7.0f;
            geo.goal_height   = 2.5f;
            geo.goal_depth    = 1.5f;
            break;
        }

        field.setup(geo);

        // --- Build semantic zones based on sport geometry ---
        float half_l = geo.length * 0.5f;
        float half_w = geo.width  * 0.5f;

        // OUT_OF_BOUNDS: perimeter strip around the field
        float bw = geo.boundary_width;
        field.add_semantic_zone(ZoneSemantic::OUT_OF_BOUNDS,
            Vec3(-half_l - bw, 0.0f, -half_w - bw),
            Vec3(half_l + bw,    0.0f, -half_w));
        field.add_semantic_zone(ZoneSemantic::OUT_OF_BOUNDS,
            Vec3(-half_l - bw, 0.0f,  half_w),
            Vec3(half_l + bw,    0.0f,  half_w + bw));
        field.add_semantic_zone(ZoneSemantic::OUT_OF_BOUNDS,
            Vec3(-half_l - bw, 0.0f, -half_w),
            Vec3(-half_l,       0.0f,  half_w));
        field.add_semantic_zone(ZoneSemantic::OUT_OF_BOUNDS,
            Vec3(half_l,        0.0f, -half_w),
            Vec3(half_l + bw,   0.0f,  half_w));

        // Sport-specific semantic zones
        switch (config.sport) {
        case SportType::SOCCER: {
            float pa_hw = geo.penalty_area_width * 0.5f;
            float pa_d  = geo.penalty_area_depth;
            float ga_hw = geo.goal_area_width * 0.5f;
            float ga_d  = geo.goal_area_depth;
            float gw    = geo.goal_width * 0.5f;
            float gd    = geo.goal_depth;

            // Home penalty area
            field.add_semantic_zone(ZoneSemantic::RESTRICTED_DEFENSE,
                Vec3(-half_l, 0.0f, -pa_hw),
                Vec3(-half_l + pa_d, 0.0f, pa_hw));
            // Away penalty area
            field.add_semantic_zone(ZoneSemantic::RESTRICTED_DEFENSE,
                Vec3(half_l - pa_d, 0.0f, -pa_hw),
                Vec3(half_l,        0.0f, pa_hw));
            // Home goal area (6-yard box)
            field.add_semantic_zone(ZoneSemantic::RESTRICTED_DEFENSE,
                Vec3(-half_l, 0.0f, -ga_hw),
                Vec3(-half_l + ga_d, 0.0f, ga_hw));
            // Away goal area
            field.add_semantic_zone(ZoneSemantic::RESTRICTED_DEFENSE,
                Vec3(half_l - ga_d, 0.0f, -ga_hw),
                Vec3(half_l,        0.0f, ga_hw));
            // Home goal
            field.add_semantic_zone(ZoneSemantic::SCORING_TARGET,
                Vec3(-half_l - gd, 0.0f, -gw),
                Vec3(-half_l,       0.0f, gw));
            // Away goal
            field.add_semantic_zone(ZoneSemantic::SCORING_TARGET,
                Vec3(half_l,        0.0f, -gw),
                Vec3(half_l + gd,   0.0f, gw));
            break;
        }
        case SportType::BASKETBALL: {
            float lane_hw = geo.penalty_area_width * 0.5f;
            float lane_d  = geo.penalty_area_depth;

            // Home paint (restricted defense)
            field.add_semantic_zone(ZoneSemantic::RESTRICTED_DEFENSE,
                Vec3(-half_l, 0.0f, -lane_hw),
                Vec3(-half_l + lane_d, 0.0f, lane_hw));
            // Away paint
            field.add_semantic_zone(ZoneSemantic::RESTRICTED_DEFENSE,
                Vec3(half_l - lane_d, 0.0f, -lane_hw),
                Vec3(half_l,          0.0f, lane_hw));
            // Home hoop
            field.add_semantic_zone(ZoneSemantic::SCORING_TARGET,
                Vec3(-half_l - 0.5f, 0.0f, -geo.goal_width * 0.5f),
                Vec3(-half_l,         0.0f,  geo.goal_width * 0.5f));
            // Away hoop
            field.add_semantic_zone(ZoneSemantic::SCORING_TARGET,
                Vec3(half_l,          0.0f, -geo.goal_width * 0.5f),
                Vec3(half_l + 0.5f,   0.0f,  geo.goal_width * 0.5f));
            break;
        }
        case SportType::AMERICAN_FOOTBALL: {
            // Home endzone
            field.add_semantic_zone(ZoneSemantic::SCORING_TARGET,
                Vec3(-half_l - 10.0f, 0.0f, -half_w),
                Vec3(-half_l,         0.0f,  half_w));
            // Away endzone
            field.add_semantic_zone(ZoneSemantic::SCORING_TARGET,
                Vec3(half_l,           0.0f, -half_w),
                Vec3(half_l + 10.0f,  0.0f,  half_w));
            break;
        }
        case SportType::RUGBY_UNION: {
            float try_d = geo.penalty_area_depth;
            // Home try zone (in-goal area)
            field.add_semantic_zone(ZoneSemantic::SCORING_TARGET,
                Vec3(-half_l - try_d, 0.0f, -half_w),
                Vec3(-half_l,         0.0f,  half_w));
            // Away try zone
            field.add_semantic_zone(ZoneSemantic::SCORING_TARGET,
                Vec3(half_l,           0.0f, -half_w),
                Vec3(half_l + try_d,   0.0f,  half_w));
            break;
        }
        default:
            break;
        }
    }

    // =========================================================================
    // build_influence_map — Populate influence grid from delayed perception
    // =========================================================================
    // Phase 11b Action 7: Builds the InfluenceMap once per AI tick using the
    // delayed perception buffer. A baseline 12-frame delay (~200ms) represents
    // the "average" team perception of the battlefield geometry.
    //
    // Each athlete's historical position (from the delayed snapshot) is
    // injected as either threat (opponent team) or control (own team).
    //
    // Parameters:
    //   team_perspective  — Which team is evaluating (its opponents are threats)
    //
    // Call this from the Application layer after pushing a perception snapshot
    // but before evaluating individual AI decisions.
    // =========================================================================
    void build_influence_map(uint8_t team_perspective)
    {
        influence_map.initialize(field);
        influence_map.clear();

        // If no perception history yet, nothing to build from
        if (perception_buffer.get_count() == 0u) return;

        // Use a baseline 12-frame delay (~200ms) for the spatial map
        const PerceptionSnapshot& delayed = perception_buffer.get_delayed_state(12u);

        for (uint32_t i = 0u; i < delayed.athlete_count; ++i) {
            const AthletePercept& p = delayed.athletes[i];
            if (!p.is_active) continue;

            // Athletes on the opposing team are "threats"
            bool is_threat = (p.team != team_perspective);
            influence_map.inject_influence(p.position, 1.0f, 8.0f, is_threat);
        }
    }

    // =========================================================================
    // evaluate_ai_decisions — AI decision + steering pipeline (Phase 12)
    // =========================================================================
    // Phase 12 Action 1: The monolithic AI from Application::tick() has been
    // moved here. This method owns the complete AI cognitive pipeline:
    //
    //   1. Chase budget computation (limits chasers per team)
    //   2. Per-athlete context factor evaluation (8 inputs)
    //   3. Utility AI evaluation with hysteresis (Phase 11b Action 5)
    //   4. Action -> steering target mapping (16 action types)
    //   5. Composite steering (arrive + separation blend)
    //   6. Steering -> MotorIntent conversion via AI motor controller
    //
    // Parameters:
    //   dt — Physics timestep (1/240s) for motor controller integration
    // =========================================================================
    void evaluate_ai_decisions(float dt)
    {
        // Phase 14 Action 2: Snapshot current world into perception buffer.
        // Each athlete queries get_delayed_state(own reaction_frames) to
        // see the world as it was N ticks ago — not the live state.
        {
            const BallEntity* snap_ball = entity_manager.find_ball();
            perception_buffer.push_state(
                snap_ball ? snap_ball->position : Vec3(0.0f, 0.0f, 0.0f),
                snap_ball ? snap_ball->velocity : Vec3(0.0f, 0.0f, 0.0f),
                snap_ball ? snap_ball->possession_team : TEAM_NONE,
                ball_in_play,
                entity_manager.athletes, entity_manager.athlete_count);
        }

        // Build spatial influence map (baseline 12-frame team perspective)
        build_influence_map(static_cast<uint8_t>(TEAM_HOME));

        // Live ball for chase budget (game-mechanics, not AI perception)
        const BallEntity* ball = entity_manager.find_ball();
        Vec3 live_ball_pos = ball ? ball->position : Vec3(0.0f, 0.0f, 0.0f);
        float half_field = config.field_length * 0.5f;

        uint32_t home_player_idx = 0u;
        uint32_t away_player_idx = 0u;

        // --- Chase budget: only nearest N players per team may chase ---
        struct ChaseRank { uint32_t idx; float dist; };
        ChaseRank home_ranks[MAX_ATHLETES];
        ChaseRank away_ranks[MAX_ATHLETES];
        uint32_t home_rank_count = 0u, away_rank_count = 0u;
        for (uint32_t ii = 0u; ii < entity_manager.athlete_count; ++ii) {
            const AthleteEntity& aa = entity_manager.athletes[ii];
            if (!aa.id.is_valid() || aa.is_human_controlled) continue;
            float d = Vec3::length(Vec3::sub(aa.position, live_ball_pos));
            if (aa.team == TEAM_HOME && home_rank_count < MAX_ATHLETES) {
                home_ranks[home_rank_count++] = { ii, d };
            } else if (aa.team == TEAM_AWAY && away_rank_count < MAX_ATHLETES) {
                away_ranks[away_rank_count++] = { ii, d };
            }
        }
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

        uint32_t max_chasers_home = 2u;
        uint32_t max_chasers_away = 2u;
        if (ball) {
            if (ball->possession_team == TEAM_NONE) {
                max_chasers_home = 2u;
                max_chasers_away = 2u;
            } else if (ball->possession_team == TEAM_HOME) {
                max_chasers_home = 1u;
                max_chasers_away = 2u;
            } else {
                max_chasers_home = 2u;
                max_chasers_away = 1u;
            }
        }
        uint8_t home_can_chase[MAX_ATHLETES] = {};
        uint8_t away_can_chase[MAX_ATHLETES] = {};
        for (uint32_t ii = 0u; ii < home_rank_count && ii < max_chasers_home; ++ii) {
            home_can_chase[home_ranks[ii].idx] = 1u;
        }
        for (uint32_t ii = 0u; ii < away_rank_count && ii < max_chasers_away; ++ii) {
            away_can_chase[away_ranks[ii].idx] = 1u;
        }

        for (uint32_t i = 0u; i < entity_manager.athlete_count; ++i) {
            AthleteEntity& a = entity_manager.athletes[i];
            if (!a.id.is_valid() || a.is_human_controlled) continue;
            if (i >= MAX_AI_CONTROLLERS) continue;

            // Phase 14 Action 2: Per-athlete delayed perception.
            // A veteran with reaction_frames=6 sees the present;
            // a fatigued rookie with reaction_frames=18 sees 300ms ago.
            const PerceptionSnapshot& snap = perception_buffer.get_delayed_state(a.reaction_frames);
            Vec3 ball_pos = snap.ball_position;  // Delayed ball position
            Vec3 ball_vel = snap.ball_velocity;  // Delayed ball velocity

            float dist_to_ball = Vec3::length(Vec3::sub(a.position, ball_pos));
            float own_goal_x = (a.team == TEAM_HOME) ? -half_field : half_field;
            float opp_goal_x = (a.team == TEAM_HOME) ?  half_field : -half_field;
            float dist_opp_goal = std::abs(a.position.x - opp_goal_x);

            // Phase 14 Action 2: possession rank from DELAYED teammate positions
            float has_possession = 0.0f;
            if (last_possession_team == a.team && possession_timer > 0.0f) {
                float own_rank = 0u;
                for (uint32_t j = 0u; j < snap.athlete_count; ++j) {
                    const AthletePercept& tm = snap.athletes[j];
                    if (tm.entity_index == a.id.index) continue;
                    if (!tm.is_active || tm.team != a.team) continue;
                    float other_dist = Vec3::length(Vec3::sub(tm.position, ball_pos));
                    if (other_dist < dist_to_ball) ++own_rank;
                }
                has_possession = (own_rank == 0u) ? 1.0f : (own_rank == 1u) ? 0.5f : 0.2f;
            }

            uint8_t can_chase = (a.team == TEAM_HOME)
                ? home_can_chase[i] : away_can_chase[i];

            uint32_t form_idx_pq = (a.team == TEAM_HOME)
                ? home_player_idx : away_player_idx;
            float team_possession_pq = 0.0f;
            if (last_possession_team == a.team && possession_timer > 0.0f) {
                team_possession_pq = 1.0f;
            }
            float possession_factor_pq = 0.2f + 0.8f * team_possession_pq;
            Vec3 form_pos_pq = formation_system.get_formation_position(
                static_cast<uint8_t>(form_idx_pq),
                ball_pos,
                Vec3(own_goal_x, 0.0f, 0.0f),
                Vec3(opp_goal_x, 0.0f, 0.0f),
                possession_factor_pq);
            form_pos_pq.x *= half_field;
            form_pos_pq.z *= config.field_width * 0.5f;
            form_pos_pq.y = 0.0f;
            if (a.team == TEAM_AWAY) { form_pos_pq.x = -form_pos_pq.x; }
            float dist_to_formation = Vec3::length(Vec3::sub(a.position, form_pos_pq));
            float position_quality = 1.0f - std::min(dist_to_formation / 20.0f, 1.0f);

            // Phase 14 Action 2: nearest opponent from DELAYED perception
            float nearest_opp_dist = 999.0f;
            for (uint32_t j = 0u; j < snap.athlete_count; ++j) {
                const AthletePercept& opp = snap.athletes[j];
                if (!opp.is_active) continue;
                if (opp.entity_index == a.id.index) continue;
                if (opp.team == a.team) continue;
                float od = Vec3::length(Vec3::sub(opp.position, a.position));
                if (od < nearest_opp_dist) nearest_opp_dist = od;
            }

            float context_inputs[8] = {
                dist_to_ball,
                dist_opp_goal,
                nearest_opp_dist,
                a.stamina,
                has_possession,
                1.0f,
                0.0f,
                position_quality
            };

            uint32_t team_idx = (a.team == TEAM_HOME) ? 0u : 1u;
            utility_ai[team_idx].configure_role(a.role);

            // Phase 11b Action 5: hysteresis evaluation (writes entity.active_action_id)
            UtilityScore decision = utility_ai[team_idx].evaluate_with_hysteresis(
                context_inputs, 8u, a);

            if (!can_chase &&
                (decision.action == AIActionType::CHASE_BALL ||
                 decision.action == AIActionType::SHOOT_BALL ||
                 decision.action == AIActionType::PRESS ||
                 decision.action == AIActionType::TACKLE)) {
                decision.action = AIActionType::FORMATION_HOLD;
                decision.score *= 0.5f;
            }

            uint32_t form_idx = (a.team == TEAM_HOME)
                ? home_player_idx : away_player_idx;
            float team_possession = 0.0f;
            if (last_possession_team == a.team && possession_timer > 0.0f) {
                team_possession = 1.0f;
            }
            float possession_factor = 0.2f + 0.8f * team_possession;

            Vec3 formation_pos = formation_system.get_formation_position(
                static_cast<uint8_t>(form_idx),
                ball_pos,
                Vec3(own_goal_x, 0.0f, 0.0f),
                Vec3(opp_goal_x, 0.0f, 0.0f),
                possession_factor);
            formation_pos.x *= half_field;
            formation_pos.z *= config.field_width * 0.5f;
            formation_pos.y = 0.0f;
            if (a.team == TEAM_AWAY) { formation_pos.x = -formation_pos.x; }

            Vec3 steer_target = formation_pos;
            float max_speed = ai_controllers[i].steering_config.max_speed;
            if (max_speed < APC_EPSILON) max_speed = 7.0f;
            float urgency = 0.2f;

            switch (decision.action) {
            case AIActionType::CHASE_BALL:
                steer_target = ball_pos;
                urgency = 0.9f;
                if (dist_to_ball < 3.0f) {
                    Vec3 predicted = Vec3::add(ball_pos,
                        Vec3::scale(ball_vel, 0.3f));
                    predicted.y = 0.0f;
                    steer_target = predicted;
                }
                break;
            case AIActionType::SHOOT_BALL:
                if (dist_to_ball < 1.5f) {
                    steer_target = ball_pos;
                    urgency = 1.0f;
                } else {
                    steer_target = formation_pos;
                    urgency = 0.3f;
                }
                break;
            case AIActionType::MOVE_TO_POSITION:
                steer_target = formation_pos;
                urgency = 0.5f;
                break;
            case AIActionType::SUPPORT_RUN:
                steer_target = Vec3(
                    (ball_pos.x + opp_goal_x) * 0.5f,
                    0.0f,
                    ball_pos.z + ((a.position.z > ball_pos.z) ? 5.0f : -5.0f)
                );
                urgency = 0.6f;
                break;
            case AIActionType::PRESS:
                steer_target = ball_pos;
                urgency = 0.95f;
                break;
            case AIActionType::INTERCEPT:
                {
                    float predict_time = dist_to_ball / (max_speed + APC_EPSILON);
                    if (predict_time > 1.0f) predict_time = 1.0f;
                    steer_target = Vec3::add(ball_pos,
                        Vec3::scale(ball_vel, predict_time));
                    steer_target.y = 0.0f;
                }
                urgency = 0.8f;
                break;
            case AIActionType::FORMATION_HOLD:
            case AIActionType::TACKLE:
                {
                    // Phase 14 Action 2: nearest opponent from DELAYED perception
                    float tackle_opp_dist = 999.0f;
                    Vec3 tackle_opp_pos = a.position;
                    for (uint32_t j = 0u; j < snap.athlete_count; ++j) {
                        const AthletePercept& opp = snap.athletes[j];
                        if (!opp.is_active) continue;
                        if (opp.entity_index == a.id.index) continue;
                        if (opp.team == a.team) continue;
                        float d = Vec3::length(Vec3::sub(opp.position, a.position));
                        if (d < tackle_opp_dist) {
                            tackle_opp_dist = d;
                            tackle_opp_pos = opp.position;
                        }
                    }
                    static constexpr float TACKLE_RANGE = 3.0f;
                    if (tackle_opp_dist < TACKLE_RANGE) {
                        steer_target = tackle_opp_pos;
                        urgency = 0.95f;
                    } else {
                        steer_target = formation_pos;
                        urgency = 0.3f;
                    }
                }
                break;
            case AIActionType::PASS_BALL:
                if (dist_to_ball < 2.0f) {
                    // Phase 14 Action 2: find best forward from DELAYED teammates
                    float best_forward = -999.0f;
                    Vec3 best_teammate_pos = a.position;
                    for (uint32_t j = 0u; j < snap.athlete_count; ++j) {
                        const AthletePercept& tm = snap.athletes[j];
                        if (tm.entity_index == a.id.index) continue;
                        if (!tm.is_active || tm.team != a.team) continue;
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
                steer_target = Vec3(
                    (ball_pos.x + own_goal_x) * 0.5f,
                    0.0f,
                    ball_pos.z
                );
                urgency = 0.85f;
                break;
            case AIActionType::MARK_OPPONENT:
                {
                    // Phase 14 Action 2: mark target from DELAYED perception
                    float mark_dist = 999.0f;
                    Vec3 mark_pos = a.position;
                    for (uint32_t j = 0u; j < snap.athlete_count; ++j) {
                        const AthletePercept& opp = snap.athletes[j];
                        if (!opp.is_active) continue;
                        if (opp.entity_index == a.id.index) continue;
                        if (opp.team == a.team) continue;
                        float d = Vec3::length(Vec3::sub(opp.position, a.position));
                        if (d < mark_dist) {
                            mark_dist = d;
                            mark_pos = opp.position;
                        }
                    }
                    steer_target = Vec3(
                        (mark_pos.x + own_goal_x) * 0.45f,
                        0.0f,
                        mark_pos.z
                    );
                    urgency = 0.6f;
                }
                break;
            case AIActionType::CROSS:
                if (dist_to_ball < 2.0f) {
                    steer_target = Vec3(opp_goal_x * 0.6f, 0.0f, 0.0f);
                    urgency = 0.9f;
                } else {
                    steer_target = ball_pos;
                    urgency = 0.7f;
                }
                break;
            case AIActionType::HEADER:
                steer_target = ball_pos;
                urgency = 0.85f;
                break;
            case AIActionType::DIVE_SAVE:
                {
                    Vec3 dive_target = Vec3::add(ball_pos,
                        Vec3::scale(ball_vel, 0.15f));
                    dive_target.x = own_goal_x + (a.team == TEAM_HOME ? 2.0f : -2.0f);
                    dive_target.y = 0.0f;
                    steer_target = dive_target;
                }
                urgency = 1.0f;
                break;
            case AIActionType::PUNT:
                steer_target = ball_pos;
                urgency = 0.8f;
                break;
            default:
                steer_target = formation_pos;
                urgency = 0.2f;
                break;
            }

            SteeringOutput primary = SteeringSystem::arrive(
                a.position, steer_target, max_speed, 0.5f, 4.0f);
            primary.urgency = urgency;

            Vec3 all_neighbors[MAX_NEIGHBOR_COUNT * 2];
            uint32_t all_neighbor_count = 0u;
            for (uint32_t j = 0u; j < entity_manager.athlete_count && all_neighbor_count < MAX_NEIGHBOR_COUNT * 2; ++j) {
                if (j == i) continue;
                const AthleteEntity& other = entity_manager.athletes[j];
                if (!other.id.is_valid()) continue;
                float ndist = Vec3::length(Vec3::sub(a.position, other.position));
                if (ndist < 2.5f) {
                    all_neighbors[all_neighbor_count++] = other.position;
                }
            }
            SteeringOutput sep = SteeringSystem::separation(
                all_neighbors, all_neighbor_count, a.position,
                1.5f, 20.0f);

            WeightedSteering blended[2];
            blended[0].behavior = SteeringBehavior::ARRIVE;
            blended[0].weight = 1.0f;
            blended[0].output = primary;
            blended[1].behavior = SteeringBehavior::SEPARATION;
            blended[1].weight = 1.5f;
            blended[1].output = sep;

            SteeringOutput final_steering = SteeringSystem::blend(blended, 2u);

            a.current_intent = ai_controllers[i].update(
                final_steering, a.position, a.orientation, dt);

            // Store chosen action for ball interaction (reads a.flags & 0xFF)
            a.flags = static_cast<uint16_t>(
                a.flags & 0xFF00u) | static_cast<uint16_t>(decision.action);

            if (a.team == TEAM_HOME) ++home_player_idx;
            else ++away_player_idx;
        }
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
    // assign_human — Mark athlete as human-controlled (updates bitmask)
    // =========================================================================
    void assign_human(EntityId athlete_id)
    {
        if (!athlete_id.is_valid()) return;
        AthleteEntity* a = entity_manager.get_athlete(athlete_id);
        if (a) {
            a->is_human_controlled = 1;
            a->controller = ControllerType::PLAYER;
            // Update bitmasks: move from AI to human-controlled
            entity_manager.clear_bit(entity_manager.mask_ai_controlled, athlete_id.index);
            entity_manager.set_bit(entity_manager.mask_human_controlled, athlete_id.index);
        }
    }

    // =========================================================================
    // load_sport_configuration — Inject sport-specific AI utility actions
    // =========================================================================
    // Clears the AI's action memory and injects only the actions relevant
    // to the selected sport. This guarantees the AI will never try to
    // execute a CROSS or SLIDE_TACKLE on a basketball court.
    //
    // Also wires up the SportRulesConfig from the module config so the AI
    // knows what physics actions are legal (offside, contact severity, etc.).
    //
    // Called from assign_all_ai() based on config.sport.
    // =========================================================================
    void load_sport_configuration(const SportModuleConfig& module_config)
    {
        // 1. Store the Rules Engine (from Phase 11a Action 1)
        //    The AI can query rules.current_state,
        //    rules.offside_rule_active, etc.
        rules = module_config.rules;

        // 2. Clear AI action memory for both teams
        utility_ai[0].clear_actions();
        utility_ai[1].clear_actions();

        // 3. Inject Sport-Specific Utility Actions
        //    In a full production environment, this would read from a data
        //    file or function pointers. For our deterministic core, we
        //    hard-bind the exact subset of actions allowed per sport.
        //
        //    Mapping: SportModuleType -> AIActionType subset
        //
        //    SOCCER:           PASS, SHOOT, CROSS, HEADER, TACKLE, DIVE_SAVE, PUNT, BLOCK
        //    BASKETBALL:       PASS, SHOOT, DRIVE, SCREEN, BOX_OUT (no tackle/cross)
        //    HOCKEY:           PASS, SHOOT, BLOCK, PRESS (body check = tackle)
        //    AMERICAN_FOOTBALL: PASS, SHOOT, TACKLE, BLOCK, PRESS
        //    RUGBY:            PASS, SHOOT, TACKLE, BLOCK, SUPPORT_RUN, PRESS

        // Register actions for both teams (same subset per team)
        switch (module_config.module_type) {
        case SportModuleType::SOCCER:
 utility_ai[0].add_action(AIActionType::PASS_BALL);
 utility_ai[0].add_action(AIActionType::SHOOT_BALL);
 utility_ai[0].add_action(AIActionType::CROSS);
 utility_ai[0].add_action(AIActionType::HEADER);
 utility_ai[0].add_action(AIActionType::TACKLE);
 utility_ai[0].add_action(AIActionType::DIVE_SAVE);
 utility_ai[0].add_action(AIActionType::PUNT);
 utility_ai[0].add_action(AIActionType::BLOCK);
 utility_ai[0].add_action(AIActionType::INTERCEPT);
 utility_ai[0].add_action(AIActionType::PRESS);
 utility_ai[0].add_action(AIActionType::SUPPORT_RUN);
 utility_ai[0].add_action(AIActionType::MARK_OPPONENT);
 utility_ai[0].add_action(AIActionType::MOVE_TO_POSITION);
 utility_ai[0].add_action(AIActionType::CHASE_BALL);
 utility_ai[0].add_action(AIActionType::FORMATION_HOLD);
            break;

        case SportModuleType::BASKETBALL:
 // No TACKLE, CROSS, HEADER, PUNT, DIVE_SAVE on a basketball court
 utility_ai[0].add_action(AIActionType::PASS_BALL);
 utility_ai[0].add_action(AIActionType::SHOOT_BALL);
 utility_ai[0].add_action(AIActionType::BLOCK);
 utility_ai[0].add_action(AIActionType::INTERCEPT);
 utility_ai[0].add_action(AIActionType::SUPPORT_RUN);
 utility_ai[0].add_action(AIActionType::PRESS);
 utility_ai[0].add_action(AIActionType::MOVE_TO_POSITION);
 utility_ai[0].add_action(AIActionType::CHASE_BALL);
 utility_ai[0].add_action(AIActionType::FORMATION_HOLD);
            break;

        case SportModuleType::HOCKEY:
 utility_ai[0].add_action(AIActionType::PASS_BALL);
 utility_ai[0].add_action(AIActionType::SHOOT_BALL);
 utility_ai[0].add_action(AIActionType::BLOCK);
 utility_ai[0].add_action(AIActionType::TACKLE);
 utility_ai[0].add_action(AIActionType::INTERCEPT);
 utility_ai[0].add_action(AIActionType::PRESS);
 utility_ai[0].add_action(AIActionType::SUPPORT_RUN);
 utility_ai[0].add_action(AIActionType::MOVE_TO_POSITION);
 utility_ai[0].add_action(AIActionType::CHASE_BALL);
 utility_ai[0].add_action(AIActionType::FORMATION_HOLD);
            break;

        case SportModuleType::AMERICAN_FOOTBALL:
 utility_ai[0].add_action(AIActionType::PASS_BALL);
 utility_ai[0].add_action(AIActionType::SHOOT_BALL);
 utility_ai[0].add_action(AIActionType::TACKLE);
 utility_ai[0].add_action(AIActionType::BLOCK);
 utility_ai[0].add_action(AIActionType::INTERCEPT);
 utility_ai[0].add_action(AIActionType::PRESS);
 utility_ai[0].add_action(AIActionType::SUPPORT_RUN);
 utility_ai[0].add_action(AIActionType::MOVE_TO_POSITION);
 utility_ai[0].add_action(AIActionType::CHASE_BALL);
 utility_ai[0].add_action(AIActionType::FORMATION_HOLD);
            break;

        case SportModuleType::RUGBY:
 utility_ai[0].add_action(AIActionType::PASS_BALL);
 utility_ai[0].add_action(AIActionType::SHOOT_BALL);
 utility_ai[0].add_action(AIActionType::TACKLE);
 utility_ai[0].add_action(AIActionType::BLOCK);
 utility_ai[0].add_action(AIActionType::INTERCEPT);
 utility_ai[0].add_action(AIActionType::SUPPORT_RUN);
 utility_ai[0].add_action(AIActionType::PRESS);
 utility_ai[0].add_action(AIActionType::MOVE_TO_POSITION);
 utility_ai[0].add_action(AIActionType::CHASE_BALL);
 utility_ai[0].add_action(AIActionType::FORMATION_HOLD);
            break;

        default:
            // Fallback: basic movement only
 utility_ai[0].add_action(AIActionType::CHASE_BALL);
 utility_ai[0].add_action(AIActionType::MOVE_TO_POSITION);
 utility_ai[0].add_action(AIActionType::FORMATION_HOLD);
            break;
        }

        // Mirror actions to away team
        for (uint32_t i = 0u; i < utility_ai[0].action_count; ++i) {
            utility_ai[1].add_action(utility_ai[0].actions[i]);
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

        // Inject sport-specific actions via load_sport_configuration
        SportModuleConfig module_config;
        switch (config.sport) {
        case SportType::SOCCER:           module_config.module_type = SportModuleType::SOCCER; break;
        case SportType::BASKETBALL:       module_config.module_type = SportModuleType::BASKETBALL; break;
        case SportType::ICE_HOCKEY:       module_config.module_type = SportModuleType::HOCKEY; break;
        case SportType::AMERICAN_FOOTBALL: module_config.module_type = SportModuleType::AMERICAN_FOOTBALL; break;
        case SportType::RUGBY_UNION:      module_config.module_type = SportModuleType::RUGBY; break;
        default:                          module_config.module_type = SportModuleType::SOCCER; break;
        }
        load_sport_configuration(module_config);

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
                return; // Skip athlete interactions after goal reset
            } else if (oob) {
                // Out of bounds — stop ball and reset after brief pause
                ball->velocity = Vec3(0.0f, 0.0f, 0.0f);
                ball_in_play = 0;
                out_of_bounds_timer = 0.5f; // 0.5s pause then reset
                return; // Skip athlete interactions while ball is dead
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
                if (bz < -half_width) bz = -(half_width - 1.0f);
                else if (bz > half_width) bz = (half_width - 1.0f);
                // If beyond goal line, reset to center instead
                if (std::abs(bx) >= half_field) {
                    bx = 0.0f; bz = 0.0f;
                } else {
                    // Touchline OOB — place ball 1m inside the field
                    // to prevent immediate re-OOB by nearby athletes
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

            } // else: SUPPORT_RUN, MOVE, FORMATION_HOLD, etc. do NOT touch the ball

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
    // update_match_flow — Referee / clock system (Phase 15 Action 1)
    // =========================================================================
    // The clock only advances during LIVE_PLAY. When the period expires,
    // the state transitions to INTERMISSION automatically. The AI reads
    // rules.current_state on its next perception tick and naturally stops
    // chasing the ball (CHASE_BALL scores 0.0f during non-live states).
    //
    // The match_time_seconds counter is a global continuous tracker that
    // only increments while the clock is running — this ensures accurate
    // game time regardless of frame rate or physics micro-stutters.
    // =========================================================================
    void update_match_flow(float dt)
    {
        auto& state = rules.current_state;

        if (state == SemanticPlayState::LIVE_PLAY) {
            period_time_seconds += dt;
            match_time_seconds  += dt;

            // End of period detection
            if (period_time_seconds >= max_period_duration) {
                uint32_t total_periods = config.halves;
                if (current_period >= total_periods) {
                    // Full time — all periods played
                    state = SemanticPlayState::INTERMISSION;
                } else {
                    // Between periods (half-time, quarter break, etc.)
                    state = SemanticPlayState::INTERMISSION;
                }
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

        // 1. Tick the referee / clock logic
        update_match_flow(dt);

        // 2. Step kinematics: motor intent -> velocity -> position
        //    (always ticks so bodies settle naturally, even during intermission)
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

        // Note: match_time_seconds is now managed by update_match_flow()
        // and only ticks during LIVE_PLAY. The old unconditional increment
        // and empty half-time detection block were removed in Phase 15.
    }

    // =========================================================================
    // reset_match_positions — Deterministic teleport routine (Phase 15 Action 2)
    // =========================================================================
    // Three-phase reset used after goals, period breaks, or manual hot-key:
    //
    //   1. Spatial Teleport:  Ball to center, athletes to home_position
    //   2. Kinematic Wipe:    Zero all linear/angular velocities
    //   3. Cognitive Wipe:    Clear intents, action IDs, perception buffer,
    //                         influence map so AI evaluates the new layout
    //                         cleanly instead of resuming stale plans.
    //
    // Does NOT reset scores or clock — use reset_match() for that.
    // =========================================================================
    void reset_match_positions()
    {
        if (!is_loaded) return;

        // 1. Ball: teleport to center, wipe kinematics + possession
        BallEntity* ball = entity_manager.find_ball();
        if (ball && ball->id.is_valid()) {
            ball->position        = { 0.0f, ball->radius, 0.0f };
            ball->velocity        = { 0.0f, 0.0f, 0.0f };
            ball->angular_velocity = { 0.0f, 0.0f, 0.0f };
            ball->orientation     = Quat::identity();
            ball->last_toucher    = EntityId::make_invalid();
            ball->possession_team = TEAM_NONE;
            ball->is_in_play      = 0;
        }

        // 2. Athletes: spatial teleport + kinematic + cognitive wipe
        for (uint32_t i = 0u; i < entity_manager.athlete_count; ++i) {
            AthleteEntity& a = entity_manager.athletes[i];
            if (!a.id.is_valid() || !a.is_active) continue;

            // A. Spatial Teleport: return to formation home position
            a.position    = a.home_position;
            a.orientation = Quat::identity();

            // B. Kinematic Wipe: zero all motion state
            a.velocity     = { 0.0f, 0.0f, 0.0f };
            a.acceleration = { 0.0f, 0.0f, 0.0f };

            // C. Cognitive Wipe: clear motor intent and AI action state
            a.current_intent.reset();
            a.previous_intent.reset();
            a.flags           = 0u;
            a.active_action_id = 0xFFFFFFFFu;
            a.stamina = 1.0f;
            a.health  = 1.0f;
            a.sprint_cooldown = 0.0f;
            a.tackle_cooldown = 0.0f;
        }

        // 3. Clear scene-level possession tracking
        last_ball_toucher   = EntityId::make_invalid();
        last_possession_team = TEAM_NONE;
        possession_timer     = 0.0f;
        ball_in_play         = 0u; // Ball not in play until next kickoff
        out_of_bounds_timer  = 0.0f;

        // 4. Clear AI spatial memory so perception evaluates fresh layout
        perception_buffer.clear();
        influence_map.clear();

        // 5. Reset AI controllers to avoid stale steering targets
        for (uint32_t i = 0u; i < MAX_AI_CONTROLLERS; ++i) {
            ai_controllers[i].reset();
        }
    }

    // =========================================================================
    // reset_match — Reset positions, scores, time, and full state
    // =========================================================================
    void reset_match()
    {
        // Reset scoreboard and clock
        home_score          = 0u;
        away_score          = 0u;
        match_time_seconds  = 0.0f;
        current_period      = 1u;
        period_time_seconds = 0.0f;
        rules.current_state = SemanticPlayState::PRE_GAME;
        // Note: GameLoop reset is done by Application, not here.

        // Delegate physical + cognitive reset to the teleport routine
        reset_match_positions();
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
        cfg.sport               = SportType::RUGBY_UNION;
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
