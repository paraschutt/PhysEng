#pragma once
// ============================================================================
// DSPE Sleep System
// Rules:
//   - Entity sleeps when |v| < 0.01 m/s AND |ω| < 0.01 rad/s for 30 ticks
//   - Wake: collision, external force, or proximity (within 1.0m of mover)
//   - Ball never sleeps
//   - Sleeping entities: skip integrator, CCD, narrow phase
//   - Still present in AABB tree for wake proximity checks
// ============================================================================
#include "../components.h"
#include "event_system.h"
#include <array>

namespace dspe {

class SleepSystem {
public:
    // Called once per outer tick (after constraint solve)
    void update(std::array<Entity, MAX_ENTITIES>& entities,
                EventQueue& events,
                uint32_t frame);

    // External code calls this to force-wake an entity (e.g. on kick)
    static void wake_entity(Entity& e, EventQueue& events, uint32_t frame,
                            EntityId waker = INVALID_ENTITY);

    // Check if entity should be simulated this substep
    static bool is_active(const Entity& e) {
        return !e.has(COMP_SLEEP);
    }

private:
    // Test whether entity velocity is below sleep threshold
    static bool below_threshold(const Entity& e);

    // Proximity wake: if any entity within WAKE_RADIUS is awake, wake e
    void check_proximity_wake(std::array<Entity, MAX_ENTITIES>& entities,
                               EntityId target_id,
                               EventQueue& events,
                               uint32_t frame);
};

} // namespace dspe
