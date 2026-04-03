// ============================================================================
// DSPE Sleep System Implementation
// ============================================================================
#include "dspe/systems/sleep_system.h"
#include <cstdlib>

namespace dspe {

bool SleepSystem::below_threshold(const Entity& e) {
    if (!e.has(COMP_RIGIDBODY)) return false;
    const RigidBody& rb = e.rigidbody;
    FpVel lin_sq  = rb.velocity.length_sq();
    FpVel ang_sq  = rb.angular_velocity.length_sq();
    FpVel lin_thr = SleepState::LINEAR_THRESHOLD * SleepState::LINEAR_THRESHOLD;
    FpVel ang_thr = SleepState::ANGULAR_THRESHOLD * SleepState::ANGULAR_THRESHOLD;
    return lin_sq <= lin_thr && ang_sq <= ang_thr;
}

void SleepSystem::wake_entity(Entity& e, EventQueue& events,
                               uint32_t frame, EntityId waker) {
    if (!e.has(COMP_SLEEP)) return;
    e.remove(COMP_SLEEP);
    e.sleep.ticks_below_threshold = 0;

    PhysicsEvent ev;
    ev.type     = EventType::ENTITY_WOKE;
    ev.frame    = frame;
    ev.entity_a = e.skeleton.entity_id;
    ev.entity_b = waker;
    events.push(ev);
}

void SleepSystem::check_proximity_wake(
    std::array<Entity, MAX_ENTITIES>& entities,
    EntityId target_id, EventQueue& events, uint32_t frame) {

    Entity& target = entities[target_id];
    if (!target.has(COMP_SLEEP | COMP_RIGIDBODY)) return;

    Vec3Pos target_pos = target.rigidbody.position;
    FpPos   wake_r2    = SleepState::WAKE_RADIUS * SleepState::WAKE_RADIUS;

    for (EntityId id = 0; id < MAX_ENTITIES; ++id) {
        if (id == target_id) continue;
        const Entity& other = entities[id];
        if (!other.has(COMP_RIGIDBODY)) continue;
        if (other.has(COMP_SLEEP))      continue;  // Other is also sleeping
        if (other.has(COMP_STATIC))     continue;  // Static never wakes

        // Distance check (Q24.8 arithmetic)
        FpPos dx = (target_pos.x - other.rigidbody.position.x).abs();
        FpPos dy = (target_pos.y - other.rigidbody.position.y).abs();
        FpPos dz = (target_pos.z - other.rigidbody.position.z).abs();
        FpPos dist2 = fp_mul<8,8,8>(dx,dx) + fp_mul<8,8,8>(dy,dy) + fp_mul<8,8,8>(dz,dz);

        if (dist2 <= wake_r2) {
            wake_entity(target, events, frame, id);
            return;
        }
    }
}

void SleepSystem::update(std::array<Entity, MAX_ENTITIES>& entities,
                          EventQueue& events, uint32_t frame) {
    for (EntityId id = 0; id < MAX_ENTITIES; ++id) {
        Entity& e = entities[id];
        if (!e.has(COMP_RIGIDBODY)) continue;
        if (e.has(COMP_STATIC))     continue;
        if (is_ball(id))            continue;  // Ball never sleeps

        if (e.has(COMP_SLEEP)) {
            // Already sleeping — check if should wake due to proximity
            check_proximity_wake(entities, id, events, frame);
            continue;
        }

        // Check sleep threshold
        if (below_threshold(e)) {
            e.sleep.ticks_below_threshold++;
            if (e.sleep.ticks_below_threshold >= SleepState::SLEEP_TICKS) {
                // Enter sleep
                e.add(COMP_SLEEP);
                // Zero out residual velocity to prevent drift on wake
                e.rigidbody.velocity         = Vec3Vel::zero();
                e.rigidbody.angular_velocity = Vec3Vel::zero();
                e.rigidbody.force_accum      = Vec3Vel::zero();
                e.rigidbody.torque_accum     = Vec3Vel::zero();

                PhysicsEvent ev;
                ev.type     = EventType::ENTITY_SLEPT;
                ev.frame    = frame;
                ev.entity_a = id;
                events.push(ev);
            }
        } else {
            e.sleep.ticks_below_threshold = 0;
        }
    }
}

} // namespace dspe
