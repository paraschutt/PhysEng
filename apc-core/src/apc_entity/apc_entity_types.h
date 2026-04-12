#pragma once
// =============================================================================
// apc_entity_types.h — Type enumerations, IDs, and core entity structs
// =============================================================================
//
// Defines the fundamental data structures for all game entities:
//
//   - EntityType: classification of entity kinds (athlete, ball, etc.)
//   - SportRole: tactical position/role per sport
//   - TeamId: team identifier
//   - EntityId: typed handle with generation for safe entity references
//   - AthleteEntity: full athlete state (identity, physics, transform, motor,
//                    sport state, stamina, timers)
//   - BallEntity: ball state (identity, physics, transform, possession)
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation (fixed-size arrays with MAX_* constants)
//   - Deterministic: fixed-order iteration, generation-checked lookups
//   - C++17
//
// =============================================================================

#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include "apc_math/apc_math_common.h"
#include "apc_input/apc_input_types.h"
#include <cstdint>
#include <cmath>

namespace apc {

// =============================================================================
// Capacity constants
// =============================================================================
static constexpr uint32_t MAX_ENTITIES = 256;  // Scaled up: 4 chunks of 64 bits
static constexpr uint32_t CHUNK_COUNT  = MAX_ENTITIES / 64;
static constexpr uint32_t MAX_ATHLETES = MAX_ENTITIES; // Up to 256 entities (crowds, expanded modes)
static constexpr uint32_t MAX_BALLS    = 3;
static constexpr uint32_t MAX_TEAMS    = 4;    // Main + extras for training

// =============================================================================
// ControllerType — Who controls an entity (used for bitmask queries)
// =============================================================================
enum class ControllerType : uint8_t {
    AI     = 0,
    PLAYER = 1,
    NONE   = 2
};

// =============================================================================
// EntityType — Classification of entity kinds
// =============================================================================
enum class EntityType : uint8_t {
    ATHLETE        = 0,
    BALL           = 1,
    GOAL_POST      = 2,
    FIELD_SURFACE  = 3,
    BOUNDARY       = 4,
    EQUIPMENT      = 5,
    CAMERA         = 6
};

// =============================================================================
// SportRole — Tactical position/role across supported sports
// =============================================================================
enum class SportRole : uint8_t {
    // Soccer roles
    SOCCER_GK   = 0,
    SOCCER_CB   = 1,
    SOCCER_LB   = 2,
    SOCCER_RB   = 3,
    SOCCER_CDM  = 4,
    SOCCER_CM   = 5,
    SOCCER_CAM  = 6,
    SOCCER_LW   = 7,
    SOCCER_RW   = 8,
    SOCCER_ST   = 9,
    SOCCER_SUB  = 10,

    // Basketball roles
    BASKETBALL_PG = 11,
    BASKETBALL_SG = 12,
    BASKETBALL_SF = 13,
    BASKETBALL_PF = 14,
    BASKETBALL_C  = 15,
    BASKETBALL_SUB = 16,

    // American Football roles
    FOOTBALL_QB  = 17,
    FOOTBALL_RB  = 18,
    FOOTBALL_WR  = 19,
    FOOTBALL_TE  = 20,
    FOOTBALL_OL  = 21,
    FOOTBALL_DL  = 22,
    FOOTBALL_LB  = 23,
    FOOTBALL_CB  = 24,
    FOOTBALL_S   = 25,
    FOOTBALL_K   = 26,
    FOOTBALL_P   = 27,
    FOOTBALL_SUB = 28,

    // Rugby roles
    RUGBY_PROP   = 29,
    RUGBY_HOOKER = 30,
    RUGBY_LOCK   = 31,
    RUGBY_FLANKER = 32,
    RUGBY_NO8    = 33,
    RUGBY_SH     = 34,
    RUGBY_FH     = 35,
    RUGBY_OC     = 36,
    RUGBY_WING   = 37,
    RUGBY_FB     = 38,
    RUGBY_SUB    = 39
};

// =============================================================================
// TeamId — Team identifier (0 = no team, 1 = home, 2 = away)
// =============================================================================
using TeamId = uint8_t;

static constexpr TeamId TEAM_NONE  = 0;
static constexpr TeamId TEAM_HOME  = 1;
static constexpr TeamId TEAM_AWAY  = 2;

// =============================================================================
// EntityId — Typed handle with generation counter for safe entity references
// =============================================================================
struct EntityId {
    uint32_t index      = 0xFFFFFFFFu;
    uint32_t generation = 0u;

    APC_FORCEINLINE uint8_t is_valid() const {
        return (index != 0xFFFFFFFFu) ? 1u : 0u;
    }

    APC_FORCEINLINE static EntityId make_invalid() {
        return EntityId{0xFFFFFFFFu, 0u};
    }

    APC_FORCEINLINE uint8_t operator==(const EntityId& other) const {
        return (index == other.index && generation == other.generation) ? 1u : 0u;
    }

    APC_FORCEINLINE uint8_t operator!=(const EntityId& other) const {
        return (index != other.index || generation != other.generation) ? 1u : 0u;
    }
};

// =============================================================================
// AthleteEntity — Full state for an athlete (player-controlled or AI)
// =============================================================================
struct AthleteEntity {
    // --- Identity ---
    EntityId   id            = EntityId::make_invalid();
    EntityType type          = EntityType::ATHLETE;
    TeamId     team          = TEAM_NONE;
    SportRole  role          = SportRole::SOCCER_CM;
    uint8_t    jersey_number = 0;

    // --- Transform ---
    Vec3 position    = { 0.0f, 0.0f, 0.0f };
    Vec3 velocity    = { 0.0f, 0.0f, 0.0f };
    Vec3 acceleration = { 0.0f, 0.0f, 0.0f };
    Quat orientation = Quat::identity();

    // --- Body dimensions ---
    float mass   = 80.0f;   // Default 80 kg
    float height = 1.80f;   // Default 1.80 m
    float radius = 0.3f;    // Capsule radius, default 0.3 m

    // --- Motor intent ---
    MotorIntent     current_intent;
    MotorIntent     previous_intent;
    uint16_t        flags = 0u; // MotorIntentFlags bitmask

    // --- Controller type (used by EntityManager bitmask queries) ---
    ControllerType  controller = ControllerType::AI;

    // --- Sport state ---
    float stamina        = 1.0f;  // 0.0-1.0
    float max_stamina    = 1.0f;
    float health         = 1.0f;  // 0.0-1.0
    uint8_t is_human_controlled = 0;
    uint8_t is_active            = 0;

    // --- Cooldowns ---
    float sprint_cooldown  = 0.0f;
    float tackle_cooldown  = 0.0f;

    // --- Formation home position (Phase 15 Action 2) ---
    // Set during spawn_team() from formation system. Used by
    // reset_match_positions() to teleport athletes back to their
    // sport-specific starting coordinates after a goal or period break.
    Vec3 home_position = { 0.0f, 0.0f, 0.0f };

    // --- AI Hysteresis: Action commitment tracking (Phase 11b Action 5) ---
    // Records the AIActionType of the action chosen on the previous frame.
    // The UtilityAI evaluate_with_hysteresis() method applies a 15%
    // commitment bonus to this action, preventing oscillation between
    // two equally-scored decisions.
    uint32_t active_action_id = 0xFFFFFFFFu;

    // --- Perception Delay (Phase 11b Action 6, pre-allocated) ---
    // Number of frames the AI waits before reacting to a new stimulus.
    // At 60Hz AI tick: 12 frames = 200ms reaction time.
    uint32_t reaction_frames = 12u;

    // --- Deterministic ordering ---
    uint32_t unique_id = 0u; // Hash for deterministic ordering

    // --- Reset all fields to default ---
    APC_FORCEINLINE void reset() {
        id              = EntityId::make_invalid();
        type            = EntityType::ATHLETE;
        team            = TEAM_NONE;
        role            = SportRole::SOCCER_CM;
        jersey_number   = 0;
        position        = { 0.0f, 0.0f, 0.0f };
        velocity        = { 0.0f, 0.0f, 0.0f };
        acceleration    = { 0.0f, 0.0f, 0.0f };
        orientation     = Quat::identity();
        mass            = 80.0f;
        height          = 1.80f;
        radius          = 0.3f;
        current_intent.reset();
        previous_intent.reset();
        flags           = 0u;
        controller      = ControllerType::AI;
        stamina         = 1.0f;
        max_stamina     = 1.0f;
        health          = 1.0f;
        is_human_controlled = 0;
        is_active       = 0;
        sprint_cooldown  = 0.0f;
        tackle_cooldown  = 0.0f;
        home_position    = { 0.0f, 0.0f, 0.0f };
        active_action_id = 0xFFFFFFFFu;
        reaction_frames  = 12u;
        unique_id        = 0u;
    }

    // --- Query: is entity active? ---
    APC_FORCEINLINE uint8_t active() const {
        return is_active;
    }

    // --- Apply stamina drain ---
    APC_FORCEINLINE void apply_stamina_drain(float amount, float dt) {
        stamina -= amount * dt;
        if (stamina < 0.0f) stamina = 0.0f;
    }

    // --- Decrement cooldowns ---
    APC_FORCEINLINE void update_cooldowns(float dt) {
        if (sprint_cooldown > 0.0f) {
            sprint_cooldown -= dt;
            if (sprint_cooldown < 0.0f) sprint_cooldown = 0.0f;
        }
        if (tackle_cooldown > 0.0f) {
            tackle_cooldown -= dt;
            if (tackle_cooldown < 0.0f) tackle_cooldown = 0.0f;
        }
    }
};

// =============================================================================
// BallEntity — State for a game ball
// =============================================================================
struct BallEntity {
    // --- Identity ---
    EntityId   id   = EntityId::make_invalid();
    EntityType type = EntityType::BALL;

    // --- Transform ---
    Vec3 position        = { 0.0f, 0.0f, 0.0f };
    Vec3 velocity        = { 0.0f, 0.0f, 0.0f };
    Vec3 angular_velocity = { 0.0f, 0.0f, 0.0f };
    Quat orientation     = Quat::identity();

    // --- Ball properties ---
    float mass = 0.43f;  // Sport-dependent default (soccer = 0.43 kg)
    float radius = 0.11f; // Sport-dependent default (soccer = 0.11 m)

    // --- Ball type ---
    // 0 = soccer, 1 = basketball, 2 = football, 3 = rugby
    uint8_t ball_type = 0;

    // --- Physics tuning ---
    float air_resistance     = 0.01f;  // Drag coefficient
    float ground_friction    = 0.4f;   // Rolling/sliding friction
    float bounce_restitution = 0.75f;  // Bounciness

    // --- Spin ---
    uint8_t spin_axis_locked = 0; // 1 = Magnus effect simplified

    // --- Possession ---
    EntityId last_toucher    = EntityId::make_invalid();
    TeamId  possession_team  = TEAM_NONE;
    uint8_t is_in_play       = 0;

    // --- Reset all fields to default ---
    APC_FORCEINLINE void reset() {
        id                = EntityId::make_invalid();
        type              = EntityType::BALL;
        position          = { 0.0f, 0.0f, 0.0f };
        velocity          = { 0.0f, 0.0f, 0.0f };
        angular_velocity  = { 0.0f, 0.0f, 0.0f };
        orientation       = Quat::identity();
        mass              = 0.43f;
        radius            = 0.11f;
        ball_type         = 0;
        air_resistance    = 0.01f;
        ground_friction   = 0.4f;
        bounce_restitution = 0.75f;
        spin_axis_locked  = 0;
        last_toucher      = EntityId::make_invalid();
        possession_team   = TEAM_NONE;
        is_in_play        = 0;
    }

    // --- Set ball properties by sport type ---
    APC_FORCEINLINE void configure_for_sport(uint8_t sport) {
        ball_type = sport;
        switch (sport) {
        case 0: // Soccer
            mass    = 0.43f;
            radius  = 0.11f;
            bounce_restitution = 0.75f;
            air_resistance    = 0.01f;
            break;
        case 1: // Basketball
            mass    = 0.62f;
            radius  = 0.12f;
            bounce_restitution = 0.80f;
            air_resistance    = 0.008f;
            break;
        case 2: // American Football
            mass    = 0.45f;
            radius  = 0.085f;
            bounce_restitution = 0.30f;
            air_resistance    = 0.005f;
            break;
        case 3: // Rugby
            mass    = 0.44f;
            radius  = 0.09f;
            bounce_restitution = 0.50f;
            air_resistance    = 0.008f;
            break;
        default:
            mass    = 0.43f;
            radius  = 0.11f;
            break;
        }
    }
};

} // namespace apc
