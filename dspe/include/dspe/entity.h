#pragma once
// ============================================================================
// DSPE Entity / Component Mask Definitions
// MAX_ENTITIES = 64
// Index layout: 0-29 players, 30 ball, 31-63 environment & triggers
// ============================================================================
#include <cstdint>
#include <cstddef>

namespace dspe {

// ---------------------------------------------------------------------------
// Entity identity
// ---------------------------------------------------------------------------
using EntityId = uint8_t;
static constexpr EntityId INVALID_ENTITY  = 255;
static constexpr EntityId MAX_ENTITIES    = 64;
static constexpr EntityId PLAYER_BEGIN    = 0;
static constexpr EntityId PLAYER_END      = 30;   // exclusive
static constexpr EntityId BALL_ENTITY     = 30;
static constexpr EntityId ENV_BEGIN       = 31;
static constexpr EntityId ENV_END         = 64;   // exclusive

// Team assignment (players 0-14 = team A, 15-29 = team B)
inline bool is_team_a(EntityId id) { return id < 15; }
inline bool is_team_b(EntityId id) { return id >= 15 && id < 30; }
inline bool is_player(EntityId id) { return id < 30; }
inline bool is_ball  (EntityId id) { return id == BALL_ENTITY; }
inline bool is_env   (EntityId id) { return id >= ENV_BEGIN && id < ENV_END; }

// ---------------------------------------------------------------------------
// Component mask (bitflags) — which components an entity has
// ---------------------------------------------------------------------------
enum ComponentFlag : uint32_t {
    COMP_NONE            = 0,
    COMP_RIGIDBODY       = 1 << 0,   // Position, velocity, mass, forces
    COMP_COLLIDER        = 1 << 1,   // Collision shape
    COMP_SKELETON        = 1 << 2,   // 15-bone skeleton rig
    COMP_BALL_PROPERTIES = 1 << 3,   // Spin, Magnus, surface interaction
    COMP_TRIGGER         = 1 << 4,   // Trigger volume (no collision response)
    COMP_STATIC          = 1 << 5,   // Immovable geometry (stadium)
    COMP_SLEEP           = 1 << 6,   // Entity is currently sleeping
    COMP_CCD             = 1 << 7,   // Entity requires swept CCD this substep

    // Archetype shortcuts
    ARCH_PLAYER = COMP_RIGIDBODY | COMP_COLLIDER | COMP_SKELETON,
    ARCH_BALL   = COMP_RIGIDBODY | COMP_COLLIDER | COMP_BALL_PROPERTIES | COMP_CCD,
    ARCH_STATIC = COMP_COLLIDER  | COMP_STATIC,
    ARCH_TRIGGER= COMP_TRIGGER,
};

// ---------------------------------------------------------------------------
// Collision shape types
// ---------------------------------------------------------------------------
enum class ShapeType : uint8_t {
    CAPSULE,   // Player limbs, main player proxy
    SPHERE,    // Ball
    BOX,       // Stadium walls, goalposts (static)
    AABB_TRIGGER, // Trigger volume
};

// ---------------------------------------------------------------------------
// Deterministic ordering helpers
// When sorting entity pairs (for constraint solve determinism):
//   canonical pair = (min(a,b), max(a,b))
// ---------------------------------------------------------------------------
struct EntityPair {
    EntityId a, b; // a < b always
    bool operator<(EntityPair o) const {
        if (a != o.a) return a < o.a;
        return b < o.b;
    }
    bool operator==(EntityPair o) const { return a == o.a && b == o.b; }
};

inline EntityPair make_pair(EntityId x, EntityId y) {
    return x < y ? EntityPair{x, y} : EntityPair{y, x};
}

} // namespace dspe
