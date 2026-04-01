#pragma once
// ============================================================================
// DSPE Constraint Solver — Sequential Impulse
// Parameters: 10 iterations, Baumgarte β=0.2, slop=0.005m
// Friction cone: |λ_t| ≤ μ * λ_n (2D, both tangent axes clamped)
// Warm-starting: contacts and joints
// Ordering: deterministic by EntityPair (min, max entity ID)
// ============================================================================
#include "../components.h"
#include "../materials.h"
#include "narrow_phase.h"
#include "event_system.h"
#include <array>
#include <vector>
#include <algorithm>

namespace dspe {

// ---------------------------------------------------------------------------
// Solver parameters (data-driven, extern for tuning)
// ---------------------------------------------------------------------------
struct SolverParams {
    int     iterations       = 10;
    FpVel   baumgarte_beta   = FpVel::from_float(0.2f);   // Position correction
    FpVel   baumgarte_slop   = FpVel::from_float(0.005f); // Penetration tolerance
    FpVel   warm_start_scale = FpVel::from_float(0.8f);   // Impulse decay between frames
    FpVel   restitution_slop = FpVel::from_float(0.5f);   // Min relative vel for bounce
    FpVel   max_impulse      = FpVel::from_float(10000.0f); // Safety clamp
};

extern SolverParams g_solver_params;

// ---------------------------------------------------------------------------
// Contact constraint (one per contact point)
// ---------------------------------------------------------------------------
struct ContactConstraint {
    EntityId ea{}, eb{};
    Vec3Vel  normal{};           // From B toward A
    Vec3Vel  tangent1{};
    Vec3Vel  tangent2{};
    Vec3Vel  ra{}, rb{};         // Contact offset from COM
    FpVel    effective_mass_n{}; // 1 / (J M^{-1} J^T) for normal
    FpVel    effective_mass_t1{};
    FpVel    effective_mass_t2{};
    FpVel    bias{};             // Baumgarte + restitution velocity bias
    FpVel    lambda_n{};         // Accumulated normal impulse
    FpVel    lambda_t1{};
    FpVel    lambda_t2{};
    FpVel    combined_restitution{};
    FpVel    combined_mu{};      // Friction coefficient
    FpVel    depth{};
};

// ---------------------------------------------------------------------------
// Joint constraint (one per joint in all skeletons)
// ---------------------------------------------------------------------------
struct JointConstraint {
    EntityId  bone_a{};         // Entity owning parent bone
    EntityId  bone_b{};         // Entity owning child bone
    uint8_t   bone_index{};     // Bone index in skeleton
    JointType joint_type{};
    Vec3Vel   anchor_a{};       // Joint anchor in bone A local space
    Vec3Vel   anchor_b{};       // Joint anchor in bone B local space
    Vec3Vel   axis{};           // Hinge axis (local to A)
    FpVel     lambda[6]{};      // Accumulated impulses for warm-starting
    FpVel     motor_torque{};   // Animation drive torque
    JointLimit limit{};
};

// ---------------------------------------------------------------------------
// Constraint solver
// ---------------------------------------------------------------------------
class ConstraintSolver {
public:
    explicit ConstraintSolver(const SolverParams& params = g_solver_params);

    // Main solve: takes entities array, sorted contact manifolds, surface state
    void solve(std::array<Entity, MAX_ENTITIES>& entities,
               std::vector<ContactManifold>& manifolds,
               const SurfaceState& surface,
               EventQueue& events,
               uint32_t frame);

private:
    const SolverParams& params_;

    // Contact constraint pool (no heap allocations)
    std::array<ContactConstraint, 800> contact_pool_{};
    int contact_count_{0};

    // Joint constraint pool
    std::array<JointConstraint, 300> joint_pool_{};
    int joint_count_{0};

    // Per-frame methods
    void build_contact_constraints(
        std::array<Entity, MAX_ENTITIES>& entities,
        const std::vector<ContactManifold>& manifolds,
        const SurfaceState& surface);

    void build_joint_constraints(
        std::array<Entity, MAX_ENTITIES>& entities);

    void warm_start(std::array<Entity, MAX_ENTITIES>& entities);

    void solve_contacts(std::array<Entity, MAX_ENTITIES>& entities);

    void solve_joints(std::array<Entity, MAX_ENTITIES>& entities);

    void emit_contact_events(std::array<Entity, MAX_ENTITIES>& entities,
                             EventQueue& events, uint32_t frame) const;

    // Per-constraint helpers
    void prepare_contact(ContactConstraint& c,
                         const Entity& ea, const Entity& eb,
                         const ContactPoint& cp,
                         const SurfaceState& surface);

    void solve_contact_velocity(ContactConstraint& c,
                                Entity& ea, Entity& eb);

    void solve_joint_velocity(JointConstraint& j,
                              Entity& ea);

    // Tangent basis from normal (deterministic)
    static void compute_tangents(Vec3Vel normal,
                                 Vec3Vel& t1, Vec3Vel& t2);

    // Effective mass: 1 / (invMa + invMb + (ra×n)·Ia^{-1}·(ra×n) + ...)
    static FpVel effective_mass(const RigidBody& a, const RigidBody& b,
                                Vec3Vel ra, Vec3Vel rb, Vec3Vel n);
};

} // namespace dspe