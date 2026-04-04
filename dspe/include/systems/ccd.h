#pragma once
// ============================================================================
// DSPE Continuous Collision Detection (CCD)
// Algorithm: Swept sphere vs capsule, swept sphere vs box
// Applied per substep to ball entity (always) and any entity where:
//   |v| * dt > collider.radius * 0.5
// TOI (time of impact) integrated into substep loop
// ============================================================================
#include "../components.h"
#include "broad_phase.h"
#include "event_system.h"
#include <array>
#include <vector>

namespace dspe {

// ---------------------------------------------------------------------------
// CCD hit result
// ---------------------------------------------------------------------------
struct CCDHit {
    EntityId    other_entity{INVALID_ENTITY};
    FpVel       toi{};            // Time of impact ∈ [0, 1] (fraction of dt)
    Vec3Vel     normal{};         // Contact normal at impact
    Vec3Vel     contact_point{};  // World position of contact
    bool        valid{false};
};

// ---------------------------------------------------------------------------
// CCD system
// ---------------------------------------------------------------------------
class ContinuousCollisionDetection {
public:
    // Test all CCD-flagged entities against all colliders
    // Returns earliest hit per entity; caller advances entity to toi
    void test_all(std::array<Entity, MAX_ENTITIES>& entities,
                  const BroadPhase& broad_phase,
                  FpVel substep_dt,
                  std::vector<std::pair<EntityId, CCDHit>>& hits_out);

    // Single entity sweep (for manual testing)
    CCDHit sweep_entity(const Entity& mover,
                        const std::array<Entity, MAX_ENTITIES>& entities,
                        EntityId skip_id,
                        FpVel substep_dt) const;

    // Determine if entity needs CCD this substep
    // Threshold: |v| * dt > collider.radius * 0.5
    static bool needs_ccd(const Entity& e, FpVel dt);

    // Flag entities that need CCD this substep
    static void flag_ccd_entities(std::array<Entity, MAX_ENTITIES>& entities,
                                  FpVel dt);

private:
    // Swept sphere vs static capsule
    // Returns TOI in [0,1] or -1 if no hit
    static FpVel swept_sphere_capsule(
        Vec3Vel sphere_start, Vec3Vel sphere_velocity, FpVel sphere_radius,
        Vec3Vel cap_base, Vec3Vel cap_tip, FpVel cap_radius,
        Vec3Vel& out_normal);

    // Swept sphere vs AABB box (for static geometry)
    static FpVel swept_sphere_box(
        Vec3Vel sphere_start, Vec3Vel sphere_velocity, FpVel sphere_radius,
        const AABB& box,
        Vec3Vel& out_normal);

    // Closest point on segment to sphere, return t [0,1] along segment
    static FpVel closest_t_on_segment(Vec3Vel a, Vec3Vel b, Vec3Vel p);

    // Quadratic solver: at^2 + bt + c = 0, returns smallest t >= 0 or -1
    static FpVel solve_quadratic_min_positive(FpVel a, FpVel b, FpVel c);
};

} // namespace dspe
