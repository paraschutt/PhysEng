#pragma once
// =============================================================================
// apc_entity_manager.h — Bitmask-driven entity management
// =============================================================================
//
// Central manager for all game entities (athletes and balls):
//
//   - Spawn/destroy athletes and balls with generation-checked EntityIds
//   - Bitmask state queries (SoA layout): active, human/AI controlled,
//     ball carrier, team membership — all cache-friendly 64-bit chunks
//   - O(1) lookup via __builtin_ctzll (count trailing zeros) instead of
//     O(N) linear scans
//   - Up to 256 entities (4 chunks of 64 bits)
//   - Lookup by EntityId (generation check for stale references)
//   - Team queries: count, find by role
//   - Per-frame update: cooldowns, stamina drain
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation (fixed-size arrays, bitmask state)
//   - Deterministic: fixed-order iteration, no sqrt in distance comparisons
//   - SlotMap-style indexing with generation counters
//   - C++17
//
// =============================================================================

#include "apc_entity/apc_entity_types.h"
#include <cstdint>
#include <cmath>
#include <cassert>

// Compiler compatibility for bit scan forward (count trailing zeros)
#ifdef _MSC_VER
#include <intrin.h>
#define APC_CTZ64(x) _tzcnt_u64(x)
#define APC_POPCOUNT64(x) __popcnt64(x)
#else
#define APC_CTZ64(x) __builtin_ctzll(x)
#define APC_POPCOUNT64(x) __builtin_popcountll(x)
#endif

namespace apc {

// =============================================================================
// EntityManager — Central entity storage and query interface (bitmask-driven)
// =============================================================================
struct EntityManager {
    // --- Data ---
    AthleteEntity athletes[MAX_ENTITIES];
    BallEntity    balls[MAX_BALLS];
    uint32_t      athlete_count = 0u;
    uint32_t      ball_count    = 0u;
    uint32_t      next_athlete_generation = 0u;
    uint32_t      next_ball_generation    = 0u;

    // =========================================================================
    // Bitwise state masks (SoA layout for cache-friendly queries)
    // =========================================================================
    // Each mask is an array of 4 x uint64_t (CHUNK_COUNT = 4).
    // Bit i in chunk c represents entity index (c * 64 + i).
    uint64_t mask_active[CHUNK_COUNT]          = {};
    uint64_t mask_human_controlled[CHUNK_COUNT] = {};
    uint64_t mask_ai_controlled[CHUNK_COUNT]    = {};
    uint64_t mask_ball_carrier[CHUNK_COUNT]     = {};
    uint64_t mask_team_home[CHUNK_COUNT]        = {};
    uint64_t mask_team_away[CHUNK_COUNT]        = {};

    // =========================================================================
    // Internal bitmask helpers
    // =========================================================================

    // Set bit in a mask array for a given entity index
    APC_FORCEINLINE void set_bit(uint64_t* mask, uint32_t index) {
        uint32_t chunk = index / 64;
        uint32_t bit   = index % 64;
        mask[chunk] |= (1ULL << bit);
    }

    // Clear bit in a mask array for a given entity index
    APC_FORCEINLINE void clear_bit(uint64_t* mask, uint32_t index) {
        uint32_t chunk = index / 64;
        uint32_t bit   = index % 64;
        mask[chunk] &= ~(1ULL << bit);
    }

    // Check bit in a mask array for a given entity index
    APC_FORCEINLINE uint8_t test_bit(const uint64_t* mask, uint32_t index) const {
        uint32_t chunk = index / 64;
        uint32_t bit   = index % 64;
        return (mask[chunk] & (1ULL << bit)) ? 1u : 0u;
    }

    // =========================================================================
    // Spawn / Destroy
    // =========================================================================

    // --- Spawn an athlete into the first available slot (bitmask scan) ---
    // Uses CTZ (count trailing zeros) to find the first 0-bit in mask_active,
    // yielding O(1) slot allocation instead of O(N) linear scan.
    EntityId spawn_athlete(TeamId team, SportRole role, Vec3 position,
                           uint8_t jersey)
    {
        // Find first available slot: first 0-bit in mask_active
        for (uint32_t c = 0u; c < CHUNK_COUNT; ++c) {
            uint64_t inverted = ~mask_active[c];
            if (inverted != 0) {
                uint32_t bit_idx = static_cast<uint32_t>(APC_CTZ64(inverted));
                uint32_t entity_idx = (c * 64) + bit_idx;

                // Activate bitmask state
                set_bit(mask_active, entity_idx);

                if (entity_idx >= athlete_count) {
                    athlete_count = entity_idx + 1u;
                }

                ++next_athlete_generation;
                AthleteEntity& a = athletes[entity_idx];
                a.reset();
                a.id.index      = entity_idx;
                a.id.generation = next_athlete_generation;
                a.team          = team;
                a.role          = role;
                a.position      = position;
                a.jersey_number = jersey;
                a.is_active     = 1;
                a.unique_id     = compute_unique_id(team, role, jersey, entity_idx);

                // Controller type: default AI (set to PLAYER later via assign_human)
                a.controller = ControllerType::AI;
                a.is_human_controlled = 0;
                set_bit(mask_ai_controlled, entity_idx);

                // Team masks
                if (team == TEAM_HOME) {
                    set_bit(mask_team_home, entity_idx);
                } else if (team == TEAM_AWAY) {
                    set_bit(mask_team_away, entity_idx);
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

    // --- Destroy an entity by EntityId (clears all bitmask state in one sweep) ---
    uint8_t despawn(EntityId id)
    {
        // Try athletes
        if (id.index < MAX_ENTITIES) {
            AthleteEntity& a = athletes[id.index];
            if (a.id == id) {
                a.id = EntityId::make_invalid();
                a.is_active = 0;

                // Clear ALL state bits in a single sweep
                clear_bit(mask_active, id.index);
                clear_bit(mask_human_controlled, id.index);
                clear_bit(mask_ai_controlled, id.index);
                clear_bit(mask_ball_carrier, id.index);
                clear_bit(mask_team_home, id.index);
                clear_bit(mask_team_away, id.index);

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

    // --- Destroy entity by index (alias for despawn, bitwise sweep) ---
    void destroy_entity(EntityId id)
    {
        despawn(id);
    }

    // --- Clear all entities and bitmask state ---
    void reset()
    {
        for (uint32_t i = 0u; i < MAX_ENTITIES; ++i) {
            athletes[i].reset();
        }
        for (uint32_t i = 0u; i < MAX_BALLS; ++i) {
            balls[i].reset();
        }
        athlete_count = 0u;
        ball_count    = 0u;
        next_athlete_generation = 0u;
        next_ball_generation    = 0u;

        // Clear all bitmask state
        for (uint32_t c = 0u; c < CHUNK_COUNT; ++c) {
            mask_active[c]          = 0;
            mask_human_controlled[c] = 0;
            mask_ai_controlled[c]    = 0;
            mask_ball_carrier[c]     = 0;
            mask_team_home[c]        = 0;
            mask_team_away[c]        = 0;
        }
    }

    // =========================================================================
    // Getters
    // =========================================================================

    // --- Get athlete by EntityId (nullptr if invalid or generation mismatch) ---
    AthleteEntity* get_athlete(EntityId id)
    {
        if (id.index >= MAX_ENTITIES) return nullptr;
        AthleteEntity& a = athletes[id.index];
        if (a.id.generation != id.generation) return nullptr;
        if (!a.id.is_valid()) return nullptr;
        return &a;
    }

    const AthleteEntity* get_athlete(EntityId id) const
    {
        if (id.index >= MAX_ENTITIES) return nullptr;
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
    // Bitmask Queries (O(1) or O(popcount) — no full-array iteration)
    // =========================================================================

    // --- O(1) query: find ball carrier on a given team using CTZ ---
    // Uses bitwise AND to intersect mask_ball_carrier with the team mask,
    // then CTZ to jump directly to the first set bit.
    AthleteEntity* get_ball_carrier(TeamId team)
    {
        for (uint32_t c = 0u; c < CHUNK_COUNT; ++c) {
            uint64_t target = mask_ball_carrier[c];
            if (team == TEAM_HOME) {
                target &= mask_team_home[c];
            } else if (team == TEAM_AWAY) {
                target &= mask_team_away[c];
            }
            if (target != 0) {
                uint32_t bit_idx = static_cast<uint32_t>(APC_CTZ64(target));
                uint32_t entity_idx = (c * 64) + bit_idx;
                return &athletes[entity_idx];
            }
        }
        return nullptr;
    }

    // --- Set ball carrier status for an entity ---
    void set_ball_carrier(uint32_t entity_idx, uint8_t is_carrier)
    {
        if (is_carrier) {
            set_bit(mask_ball_carrier, entity_idx);
        } else {
            clear_bit(mask_ball_carrier, entity_idx);
        }
    }

    // --- Fast iteration over only active AI entities (bitwise) ---
    // Uses iter_mask &= iter_mask - 1 to clear lowest set bit each loop,
    // yielding O(popcount) iterations instead of O(MAX_ENTITIES).
    template<typename Func>
    void for_each_ai(Func callback)
    {
        for (uint32_t c = 0u; c < CHUNK_COUNT; ++c) {
            uint64_t iter_mask = mask_active[c] & mask_ai_controlled[c];
            while (iter_mask != 0) {
                uint32_t bit_idx = static_cast<uint32_t>(APC_CTZ64(iter_mask));
                callback(athletes[(c * 64) + bit_idx]);
                iter_mask &= iter_mask - 1; // Clear the lowest set bit
            }
        }
    }

    // --- Fast iteration over all active entities (bitwise) ---
    template<typename Func>
    void for_each_active(Func callback)
    {
        for (uint32_t c = 0u; c < CHUNK_COUNT; ++c) {
            uint64_t iter_mask = mask_active[c];
            while (iter_mask != 0) {
                uint32_t bit_idx = static_cast<uint32_t>(APC_CTZ64(iter_mask));
                callback(athletes[(c * 64) + bit_idx]);
                iter_mask &= iter_mask - 1;
            }
        }
    }

    // --- Fast iteration over active entities on a team (bitwise) ---
    template<typename Func>
    void for_each_team(TeamId team, Func callback)
    {
        for (uint32_t c = 0u; c < CHUNK_COUNT; ++c) {
            uint64_t iter_mask = mask_active[c];
            if (team == TEAM_HOME) {
                iter_mask &= mask_team_home[c];
            } else if (team == TEAM_AWAY) {
                iter_mask &= mask_team_away[c];
            }
            while (iter_mask != 0) {
                uint32_t bit_idx = static_cast<uint32_t>(APC_CTZ64(iter_mask));
                callback(athletes[(c * 64) + bit_idx]);
                iter_mask &= iter_mask - 1;
            }
        }
    }

    // --- O(CHUNK_COUNT) active entity count via POPCOUNT ---
    // Uses hardware POPCOUNT instruction instead of iterating the array.
    uint32_t get_active_count() const
    {
        uint32_t count = 0u;
        for (uint32_t c = 0u; c < CHUNK_COUNT; ++c) {
            count += static_cast<uint32_t>(APC_POPCOUNT64(mask_active[c]));
        }
        return count;
    }

    // =========================================================================
    // Legacy Queries (kept for backward compatibility)
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

    // --- Per-frame update: cooldowns, stamina regen, kinematics ---
    void update_all(float dt)
    {
        // --- Update athletes ---
        for (uint32_t i = 0u; i < athlete_count; ++i) {
            AthleteEntity& a = athletes[i];
            if (!a.id.is_valid() || !a.is_active) continue;

            // Update cooldowns
            a.update_cooldowns(dt);

            // --- Apply MotorIntent to kinematics ---
            apply_motor_intent(a, dt);

            // --- Integrate velocity -> position ---
            a.position.x += a.velocity.x * dt;
            a.position.y += a.velocity.y * dt;
            a.position.z += a.velocity.z * dt;

            // --- Ground clamp (Y = 0, keep on field) ---
            if (a.position.y < 0.0f) {
                a.position.y = 0.0f;
                a.velocity.y = 0.0f;
            }

            // --- Velocity damping (ground friction) ---
            float friction = 3.0f * dt;
            if (friction > 0.95f) friction = 0.95f;
            float speed_xz = std::sqrt(a.velocity.x * a.velocity.x +
                                        a.velocity.z * a.velocity.z);
            // Only apply friction when there's no active locomotion intent
            if (!a.current_intent.has_locomotion() && speed_xz > APC_EPSILON) {
                float new_speed = speed_xz * (1.0f - friction);
                if (new_speed < 0.0f) new_speed = 0.0f;
                float scale = new_speed / speed_xz;
                a.velocity.x *= scale;
                a.velocity.z *= scale;
            }

            // Stamina drain from sprinting
            if (a.current_intent.sprint_intensity > APC_EPSILON) {
                a.apply_stamina_drain(0.15f * a.current_intent.sprint_intensity, dt);
                if (a.stamina <= 0.01f) {
                    a.current_intent.sprint_intensity = 0.0f;
                    a.current_intent.action_type = ACTION_MOVE;
                }
            }

            // Stamina regen (when not sprinting)
            if (a.stamina < a.max_stamina && a.current_intent.sprint_intensity < APC_EPSILON) {
                a.stamina += 0.08f * dt; // Regen rate
                if (a.stamina > a.max_stamina) {
                    a.stamina = a.max_stamina;
                }
            }
        }

        // --- Athlete-athlete collision resolution ---
        // Push overlapping athletes apart at the physics level.
        // This prevents athletes from stacking on top of each other
        // and creating gridlocks around the ball. Two passes for stability.
        for (uint32_t pass = 0u; pass < 2u; ++pass) {
            for (uint32_t i = 0u; i < athlete_count; ++i) {
                AthleteEntity& ai = athletes[i];
                if (!ai.id.is_valid() || !ai.is_active) continue;
                for (uint32_t j = i + 1u; j < athlete_count; ++j) {
                    AthleteEntity& aj = athletes[j];
                    if (!aj.id.is_valid() || !aj.is_active) continue;

                    float dx = ai.position.x - aj.position.x;
                    float dz = ai.position.z - aj.position.z;
                    float dist_sq = dx * dx + dz * dz;
                    float min_dist = ai.radius + aj.radius; // 0.6m for two default athletes

                    if (dist_sq < min_dist * min_dist && dist_sq > APC_EPSILON) {
                        float dist = std::sqrt(dist_sq);
                        float overlap = min_dist - dist;
                        float nx = dx / dist;
                        float nz = dz / dist;

                        // Push each athlete away by half the overlap
                        float push = overlap * 0.55f; // Slightly more than half to separate faster
                        ai.position.x += nx * push;
                        ai.position.z += nz * push;
                        aj.position.x -= nx * push;
                        aj.position.z -= nz * push;

                        // Kill velocity component moving into the other athlete
                        float rel_vn = (ai.velocity.x - aj.velocity.x) * nx +
                                       (ai.velocity.z - aj.velocity.z) * nz;
                        if (rel_vn < 0.0f) { // Only if approaching
                            ai.velocity.x -= nx * rel_vn * 0.5f;
                            ai.velocity.z -= nz * rel_vn * 0.5f;
                            aj.velocity.x += nx * rel_vn * 0.5f;
                            aj.velocity.z += nz * rel_vn * 0.5f;
                        }
                    }
                }
            }
        }

        // --- Update balls (full physics: gravity, friction, drag) ---
        for (uint32_t i = 0u; i < ball_count; ++i) {
            BallEntity& b = balls[i];
            if (!b.id.is_valid()) continue;

            // Integrate ball physics (gravity + bounce + position)
            integrate_ball_physics(b, dt);

            // On ground: apply rolling friction
            if (b.position.y <= b.radius + APC_EPSILON) {
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

    // --- Apply MotorIntent to athlete velocity (acceleration model) ---
    // Converts desired move_direction + move_speed into actual velocity.
    // Uses a spring-damper approach: accelerate toward desired velocity,
    // clamped by max run speed, with sprint modifier.
    static void apply_motor_intent(AthleteEntity& a, float dt)
    {
        (void)dt;

        const MotorIntent& intent = a.current_intent;
        float dir_mag = Vec3::length(intent.move_direction);

        // Max speeds (m/s)
        float max_walk  = 3.0f;
        float max_run   = 7.0f;
        float max_sprint = 10.0f;

        // Determine target speed from intent
        float target_speed = 0.0f;
        if (dir_mag > APC_EPSILON) {
            if (intent.sprint_intensity > APC_EPSILON && a.stamina > 0.01f) {
                target_speed = max_run + (max_sprint - max_run) * intent.sprint_intensity;
            } else if (intent.move_speed > APC_EPSILON) {
                target_speed = max_walk + (max_run - max_walk) * intent.move_speed;
            } else {
                target_speed = max_walk * intent.move_speed;
            }
        }

        // Target velocity in XZ plane
        Vec3 target_vel(0.0f, 0.0f, 0.0f);
        if (dir_mag > APC_EPSILON) {
            target_vel = Vec3::scale(intent.move_direction, target_speed);
        }

        // Smooth acceleration (spring-like: 12.0 = responsiveness)
        float accel_rate = 12.0f;
        if (dir_mag < APC_EPSILON) {
            // Decelerate when no input
            accel_rate = 8.0f;
        }

        // Lerp current velocity toward target
        a.velocity.x += (target_vel.x - a.velocity.x) * accel_rate * (1.0f / 240.0f);
        a.velocity.z += (target_vel.z - a.velocity.z) * accel_rate * (1.0f / 240.0f);
        // Keep Y velocity from intent (for jumps)
        a.velocity.y += intent.jump_impulse.y * (1.0f / 240.0f);

        // Clamp horizontal speed
        float horiz_speed = std::sqrt(a.velocity.x * a.velocity.x + a.velocity.z * a.velocity.z);
        if (horiz_speed > max_sprint) {
            float clamp = max_sprint / horiz_speed;
            a.velocity.x *= clamp;
            a.velocity.z *= clamp;
        }

        // Update orientation to face move direction
        if (dir_mag > APC_EPSILON) {
            Vec3 look = intent.look_direction;
            float look_mag = Vec3::length(look);
            if (look_mag > APC_EPSILON) {
                look = Vec3::scale(look, 1.0f / look_mag);
            } else {
                look = Vec3::scale(intent.move_direction, 1.0f / dir_mag);
            }
            // Y-axis rotation from direction
            float angle = std::atan2(look.x, look.z);
            a.orientation = Quat::from_axis_angle(Vec3(0.0f, 1.0f, 0.0f), angle);
        }
    }

    // --- Apply ball physics: gravity + ground bounce ---
    static void integrate_ball_physics(BallEntity& b, float dt)
    {
        // Gravity
        b.velocity.y -= 9.81f * dt;

        // Integrate position
        b.position.x += b.velocity.x * dt;
        b.position.y += b.velocity.y * dt;
        b.position.z += b.velocity.z * dt;

        // Ground bounce
        if (b.position.y < b.radius) {
            b.position.y = b.radius;
            b.velocity.y = -b.velocity.y * b.bounce_restitution;
            // Kill tiny bounces
            if (b.velocity.y < 0.3f) {
                b.velocity.y = 0.0f;
            }
        }
    }
};

} // namespace apc
