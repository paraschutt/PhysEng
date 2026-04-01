// ============================================================================
// DSPE Trigger System Implementation
// ============================================================================
#include "dspe/systems/trigger_system.h"

namespace dspe {

// ============================================================================
// Allocate a trigger entity from the reserved ENV range (IDs 31-63)
// ============================================================================
EntityId TriggerSystem::alloc_trigger_entity(
    std::array<Entity, MAX_ENTITIES>& entities) {
    for (EntityId id = ENV_BEGIN; id < ENV_END; ++id) {
        if (entities[id].component_mask == COMP_NONE) {
            entities[id].add(ARCH_TRIGGER);
            return id;
        }
    }
    return INVALID_ENTITY;
}

// ============================================================================
// Initialise pitch triggers from layout
// ============================================================================
void TriggerSystem::init_pitch_triggers(
    std::array<Entity, MAX_ENTITIES>& entities,
    const PitchLayout& layout) {

    layout_ = layout;
    trigger_count_ = 0;

    FpPos half_len   = { layout.length.raw >> 1 };
    FpPos half_width = { layout.width.raw  >> 1 };
    FpPos half_goal  = { layout.goal_width.raw >> 1 };
    FpPos goal_h     = layout.goal_height;
    FpPos goal_d     = layout.goal_depth;

    auto make_trigger = [&](TriggerEventType type, AABB bounds, bool inverted = false) {
        EntityId id = alloc_trigger_entity(entities);
        if (id == INVALID_ENTITY) return;
        entities[id].trigger.bounds     = bounds;
        entities[id].trigger.event_type = type;
        entities[id].trigger.inverted   = inverted;
        entities[id].trigger.ball_only  = (type != TriggerEventType::KICKOFF_INFRINGEMENT);
        trigger_ids_[trigger_count_++]  = id;
    };

    // ── Left goal mouth (x = -half_len, behind goal line)
    make_trigger(TriggerEventType::GOAL_LEFT, AABB{
        { -half_len - goal_d,              FpPos::zero(),       -half_goal },
        { -half_len,                        goal_h,               half_goal  }
    });

    // ── Right goal mouth (x = +half_len)
    make_trigger(TriggerEventType::GOAL_RIGHT, AABB{
        { half_len,                          FpPos::zero(),       -half_goal },
        { { half_len.raw + goal_d.raw },     goal_h,               half_goal  }
    });

    // ── Pitch boundary (inverted — fires when ball is OUTSIDE)
    make_trigger(TriggerEventType::OUT_OF_BOUNDS, AABB{
        { -half_len, FpPos::zero(), -half_width },
        {  half_len, FpPos::from_float(30.0f), half_width }
    }, /*inverted=*/true);

    // ── Left penalty box
    FpPos pa_len  = layout.penalty_area_length;
    FpPos pa_half = { layout.penalty_area_width.raw >> 1 };
    make_trigger(TriggerEventType::BALL_IN_BOX_LEFT, AABB{
        { -half_len,              FpPos::zero(), -pa_half },
        { { -half_len.raw + pa_len.raw }, FpPos::from_float(5.0f),  pa_half  }
    });

    // ── Right penalty box
    make_trigger(TriggerEventType::BALL_IN_BOX_RIGHT, AABB{
        { { half_len.raw - pa_len.raw }, FpPos::zero(), -pa_half },
        {  half_len,                     FpPos::from_float(5.0f),  pa_half  }
    });

    // ── Kickoff circle (approximated as AABB square for simplicity)
    FpPos kr = layout.kickoff_radius;
    make_trigger(TriggerEventType::KICKOFF_INFRINGEMENT, AABB{
        { -kr, FpPos::zero(), -kr },
        {  kr, FpPos::from_float(2.0f),  kr }
    });
}

// ============================================================================
// Ball-in-goal check uses full containment of ball centroid
// ============================================================================
bool TriggerSystem::ball_in_goal(const AABB& trigger,
                                  Vec3Pos ball_pos,
                                  FpPos ball_radius) {
    // The ball is "in goal" when its centre passes the goal line.
    // Check centroid only (brief spec: "ball centroid fully inside bounds")
    return trigger.contains(ball_pos);
}

// ============================================================================
// Update — check triggers against ball (and players for kickoff)
// ============================================================================
void TriggerSystem::update(const std::array<Entity, MAX_ENTITIES>& entities,
                            EventQueue& events, uint32_t frame) {
    const Entity& ball = entities[BALL_ENTITY];
    if (!ball.has(COMP_RIGIDBODY)) return;

    Vec3Pos ball_pos    = ball.rigidbody.position;
    FpPos   ball_radius = ball.has(COMP_COLLIDER)
                        ? ball.collider.sphere.radius
                        : FpPos::from_float(0.11f);

    for (int ti = 0; ti < trigger_count_; ++ti) {
        EntityId tid = trigger_ids_[ti];
        const Entity& te = entities[tid];
        if (!te.has(COMP_TRIGGER)) continue;
        const TriggerVolume& tv = te.trigger;

        // ── Ball-only triggers
        if (tv.ball_only) {
            bool fired;
            if (tv.event_type == TriggerEventType::GOAL_LEFT ||
                tv.event_type == TriggerEventType::GOAL_RIGHT) {
                fired = ball_in_goal(tv.bounds, ball_pos, ball_radius);
            } else {
                fired = tv.test(ball_pos);
            }

            if (fired) {
                PhysicsEvent ev;
                ev.frame    = frame;
                ev.entity_a = BALL_ENTITY;
                ev.entity_b = tid;
                ev.contact_point = { FpVel{ball_pos.x.raw << 8},
                                     FpVel{ball_pos.y.raw << 8},
                                     FpVel{ball_pos.z.raw << 8} };

                switch (tv.event_type) {
                    case TriggerEventType::GOAL_LEFT:
                        ev.type = EventType::GOAL_LEFT;   break;
                    case TriggerEventType::GOAL_RIGHT:
                        ev.type = EventType::GOAL_RIGHT;  break;
                    case TriggerEventType::OUT_OF_BOUNDS:
                        ev.type = EventType::OUT_OF_BOUNDS; break;
                    case TriggerEventType::BALL_IN_BOX_LEFT:
                    case TriggerEventType::BALL_IN_BOX_RIGHT:
                        // Not a separate event type; raise generic collision
                        ev.type = EventType::COLLISION; break;
                    default: break;
                }
                events.push(ev);
            }
        }

        // ── Kickoff infringement: check each opposing player
        if (tv.event_type == TriggerEventType::KICKOFF_INFRINGEMENT) {
            for (EntityId pid = PLAYER_BEGIN; pid < PLAYER_END; ++pid) {
                const Entity& player = entities[pid];
                if (!player.has(COMP_RIGIDBODY)) continue;
                if (tv.test(player.rigidbody.position)) {
                    PhysicsEvent ev;
                    ev.type     = EventType::KICKOFF_INFRINGEMENT;
                    ev.frame    = frame;
                    ev.entity_a = pid;
                    events.push(ev);
                }
            }
        }
    }
}

} // namespace dspe