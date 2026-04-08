#pragma once
// =============================================================================
// apc_entity_manager.h — Spawning, destruction, queries, and update for all entities
// =============================================================================
//
// Central manager for all game entities (athletes and balls):
//
//   - Spawn/destroy athletes and balls with generation-checked EntityIds
//   - Lookup by EntityId (generation check for stale references)
//   - Team queries: count, find by role
//   - Per-frame update: cooldowns, stamina drain
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation (fixed-size arrays with MAX_* constants)
//   - Deterministic: fixed-order iteration, no sqrt in distance comparisons
//   - SlotMap-style indexing with generation counters
//   - C++17
//
// =============================================================================

#include "apc_entity/apc_entity_types.h"
#include <cstdint>
#include <cmath>

namespace apc {

// =============================================================================
// EntityManager — Central entity storage and query interface
// =============================================================================
struct EntityManager {
    // --- Data ---
    AthleteEntity athletes[MAX_ATHLETES];
    BallEntity    balls[MAX_BALLS];
    uint32_t      athlete_count = 0u;
    uint32_t      ball_count    = 0u;
    uint32_t      next_athlete_generation = 0u;
    uint32_t      next_ball_generation    = 0u;

    // =========================================================================
    // Spawn / Destroy
    // =========================================================================

    // --- Spawn an athlete into the first available slot ---
    EntityId spawn_athlete(TeamId team, SportRole role, Vec3 position,
                           uint8_t jersey)
    {
        for (uint32_t i = 0u; i < MAX_ATHLETES; ++i) {
            AthleteEntity& a = athletes[i];
            if (!a.id.is_valid()) {
                ++next_athlete_generation;
                a.reset();
                a.id.index      = i;
                a.id.generation = next_athlete_generation;
                a.team          = team;
                a.role          = role;
                a.position      = position;
                a.jersey_number = jersey;
                a.is_active     = 1;
                a.unique_id     = compute_unique_id(team, role, jersey, i);

                if (i >= athlete_count) {
                    athlete_count = i + 1u;
                }
                return a.id;
            }
        }
        return EntityId::make_invalid();
    }

    // --- Spawn a ball into the first available slot ---
    EntityId spawn_ball(uint8_t ball_type, Vec3 position)
    {
        for (uint32_t i = 0u; i < MAX_BALLS; ++i) {
            BallEntity& b = balls[i];
            if (!b.id.is_valid()) {
                ++next_ball_generation;
                b.reset();
                b.id.index      = i;
                b.id.generation = next_ball_generation;
                b.position      = position;
                b.configure_for_sport(ball_type);
                b.is_in_play    = 1;

                if (i >= ball_count) {
                    ball_count = i + 1u;
                }
                return b.id;
            }
        }
        return EntityId::make_invalid();
    }

    // --- Destroy an entity by EntityId ---
    uint8_t despawn(EntityId id)
    {
        // Try athletes
        if (id.index < MAX_ATHLETES) {
            AthleteEntity& a = athletes[id.index];
            if (a.id == id) {
                a.id = EntityId::make_invalid();
                a.is_active = 0;
                return 1;
            }
        }
        // Try balls
        if (id.index < MAX_BALLS) {
            BallEntity& b = balls[id.index];
            if (b.id == id) {
                b.id = EntityId::make_invalid();
                b.is_in_play = 0;
                return 1;
            }
        }
        return 0;
    }

    // --- Clear all entities ---
    void reset()
    {
        for (uint32_t i = 0u; i < MAX_ATHLETES; ++i) {
            athletes[i].reset();
        }
        for (uint32_t i = 0u; i < MAX_BALLS; ++i) {
            balls[i].reset();
        }
        athlete_count = 0u;
        ball_count    = 0u;
        next_athlete_generation = 0u;
        next_ball_generation    = 0u;
    }

    // =========================================================================
    // Getters
    // =========================================================================

    // --- Get athlete by EntityId (nullptr if invalid or generation mismatch) ---
    AthleteEntity* get_athlete(EntityId id)
    {
        if (id.index >= MAX_ATHLETES) return nullptr;
        AthleteEntity& a = athletes[id.index];
        if (a.id.generation != id.generation) return nullptr;
        if (!a.id.is_valid()) return nullptr;
        return &a;
    }

    const AthleteEntity* get_athlete(EntityId id) const
    {
        if (id.index >= MAX_ATHLETES) return nullptr;
        const AthleteEntity& a = athletes[id.index];
        if (a.id.generation != id.generation) return nullptr;
        if (!a.id.is_valid()) return nullptr;
        return &a;
    }

    // --- Get ball by EntityId ---
    BallEntity* get_ball(EntityId id)
    {
        if (id.index >= MAX_BALLS) return nullptr;
        BallEntity& b = balls[id.index];
        if (b.id.generation != id.generation) return nullptr;
        if (!b.id.is_valid()) return nullptr;
        return &b;
    }

    const BallEntity* get_ball(EntityId id) const
    {
        if (id.index >= MAX_BALLS) return nullptr;
        const BallEntity& b = balls[id.index];
        if (b.id.generation != id.generation) return nullptr;
        if (!b.id.is_valid()) return nullptr;
        return &b;
    }

    // --- Find athlete by role (first match, deterministic order) ---
    AthleteEntity* find_athlete_by_role(TeamId team, SportRole role)
    {
        for (uint32_t i = 0u; i < athlete_count; ++i) {
            AthleteEntity& a = athletes[i];
            if (a.id.is_valid() && a.team == team && a.role == role) {
                return &a;
            }
        }
        return nullptr;
    }

    // --- Find first active ball ---
    BallEntity* find_ball()
    {
        for (uint32_t i = 0u; i < ball_count; ++i) {
            BallEntity& b = balls[i];
            if (b.id.is_valid() && b.is_in_play) {
                return &b;
            }
        }
        return nullptr;
    }

    const BallEntity* find_ball() const
    {
        for (uint32_t i = 0u; i < ball_count; ++i) {
            const BallEntity& b = balls[i];
            if (b.id.is_valid() && b.is_in_play) {
                return &b;
            }
        }
        return nullptr;
    }

    // =========================================================================
    // Queries
    // =========================================================================

    // --- Count athletes on a team ---
    uint32_t get_team_athlete_count(TeamId team) const
    {
        uint32_t count = 0u;
        for (uint32_t i = 0u; i < athlete_count; ++i) {
            if (athletes[i].id.is_valid() && athletes[i].team == team) {
                ++count;
            }
        }
        return count;
    }

    // =========================================================================
    // Update
    // =========================================================================

    // --- Per-frame update: cooldowns, stamina regen, friction ---
    void update_all(float dt)
    {
        // --- Update athletes ---
        for (uint32_t i = 0u; i < athlete_count; ++i) {
            AthleteEntity& a = athletes[i];
            if (!a.id.is_valid() || !a.is_active) continue;

            // Update cooldowns
            a.update_cooldowns(dt);

            // Stamina regen (when not sprinting)
            if (a.stamina < a.max_stamina) {
                a.stamina += 0.05f * dt; // Regen rate
                if (a.stamina > a.max_stamina) {
                    a.stamina = a.max_stamina;
                }
            }
        }

        // --- Update balls ---
        for (uint32_t i = 0u; i < ball_count; ++i) {
            BallEntity& b = balls[i];
            if (!b.id.is_valid()) continue;

            // Simple ground friction
            if (b.velocity.y < APC_EPSILON && b.position.y <= b.radius + APC_EPSILON) {
                // On ground: apply friction
                float friction = b.ground_friction * dt;
                if (friction > 1.0f) friction = 1.0f;
                float speed_xz = std::sqrt(b.velocity.x * b.velocity.x +
                                            b.velocity.z * b.velocity.z);
                if (speed_xz > APC_EPSILON) {
                    float new_speed = speed_xz * (1.0f - friction);
                    if (new_speed < 0.0f) new_speed = 0.0f;
                    float scale = new_speed / speed_xz;
                    b.velocity.x *= scale;
                    b.velocity.z *= scale;
                }
            }

            // Air resistance (linear drag)
            float drag = b.air_resistance * dt;
            if (drag > 0.99f) drag = 0.99f;
            b.velocity.x *= (1.0f - drag);
            b.velocity.y *= (1.0f - drag);
            b.velocity.z *= (1.0f - drag);
        }
    }

private:
    // --- Compute deterministic unique ID from entity properties ---
    static uint32_t compute_unique_id(TeamId team, SportRole role,
                                      uint8_t jersey, uint32_t index)
    {
        // Simple deterministic hash combining all identity fields
        uint32_t h = static_cast<uint32_t>(team);
        h = h * 31u + static_cast<uint32_t>(role);
        h = h * 31u + static_cast<uint32_t>(jersey);
        h = h * 31u + index;
        return h;
    }
};

} // namespace apc
