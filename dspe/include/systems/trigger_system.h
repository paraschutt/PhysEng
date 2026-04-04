#pragma once
// ============================================================================
// DSPE Trigger System — checked once per outer tick (not per substep)
// Triggers: goal, out-of-bounds, penalty area (left/right), kickoff circle
// Entity IDs 31-63 reserved for environment and trigger volumes
// ============================================================================
#include "../components.h"
#include "event_system.h"
#include <array>

namespace dspe {

// ---------------------------------------------------------------------------
// Standard pitch dimensions (FIFA standard, configurable)
// ---------------------------------------------------------------------------
struct PitchLayout {
    // Pitch bounds (y=0 is ground)
    FpPos length{FpPos::from_float(105.0f)};   // x axis
    FpPos width {FpPos::from_float( 68.0f)};   // z axis

    // Goal position (centred on each end line)
    FpPos goal_width  {FpPos::from_float( 7.32f)};
    FpPos goal_height {FpPos::from_float( 2.44f)};
    FpPos goal_depth  {FpPos::from_float( 2.44f)};  // behind goal line

    // Penalty area
    FpPos penalty_area_length{FpPos::from_float(16.5f)};
    FpPos penalty_area_width {FpPos::from_float(40.3f)};

    // Kickoff circle radius
    FpPos kickoff_radius{FpPos::from_float(9.15f)};
};

// ---------------------------------------------------------------------------
// Trigger system
// ---------------------------------------------------------------------------
class TriggerSystem {
public:
    // Populate trigger entities from pitch layout
    // Should be called once at match init
    void init_pitch_triggers(std::array<Entity, MAX_ENTITIES>& entities,
                             const PitchLayout& layout = {});

    // Check all triggers once per outer tick
    // Ball entity is always checked; player entities checked for kickoff
    void update(const std::array<Entity, MAX_ENTITIES>& entities,
                EventQueue& events,
                uint32_t frame);

    // Get the current pitch layout
    const PitchLayout& layout() const { return layout_; }

private:
    PitchLayout layout_{};
    EntityId    trigger_ids_[8]{};  // Registered trigger entity IDs
    int         trigger_count_{0};

    EntityId alloc_trigger_entity(std::array<Entity, MAX_ENTITIES>& entities);

    static bool ball_in_goal(const AABB& goal_trigger,
                              Vec3Pos ball_pos,
                              FpPos ball_radius);
};

} // namespace dspe
