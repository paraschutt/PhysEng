// ============================================================================
// DSPE Constraint Solver — Sequential Impulse Implementation
// 10 iterations, β=0.2, slop=0.005m, friction cone, joint warm-starting
// Entities processed in deterministic order (sort by EntityPair)
// ============================================================================
#include "dspe/systems/constraint_solver.h"
#include "dspe/systems/skeleton.h"
#include <algorithm>
#include <cmath>

namespace dspe {

SolverParams g_solver_params{};

ConstraintSolver::ConstraintSolver(const SolverParams& params)
    : params_(params) {}

// ---------------------------------------------------------------------------
// Tangent basis — deterministic (no random, same result for same normal)
// ---------------------------------------------------------------------------
void ConstraintSolver::compute_tangents(Vec3Vel n, Vec3Vel& t1, Vec3Vel& t2) {
    // Choose tangent using least-perpendicular axis
    FpVel abs_x = n.x.abs(), abs_y = n.y.abs(), abs_z = n.z.abs();
    Vec3Vel perp;
    if (abs_x <= abs_y && abs_x <= abs_z) {
        perp = {FpVel::one(), FpVel::zero(), FpVel::zero()};
    } else if (abs_y <= abs_z) {
        perp = {FpVel::zero(), FpVel::one(), FpVel::zero()};
    } else {
        perp = {FpVel::zero(), FpVel::zero(), FpVel::one()};
    }
    t1 = n.cross(perp).normalized();
    t2 = n.cross(t1).normalized();
}

// ---------------------------------------------------------------------------
// Effective mass: 1 / (inv_ma + inv_mb + (ra×n)·Ia⁻¹·(ra×n) + ...)
// ---------------------------------------------------------------------------
FpVel ConstraintSolver::effective_mass(const RigidBody& a, const RigidBody& b,
                                        Vec3Vel ra, Vec3Vel rb, Vec3Vel n) {
    FpVel lin_mass = a.inv_mass + b.inv_mass;

    auto angular_term = [](const RigidBody& body, Vec3Vel r, Vec3Vel axis) -> FpVel {
        Vec3Vel rxn = r.cross(axis);
        // Diagonal inertia tensor: (r×n)·I⁻¹·(r×n)
        return (rxn.x * rxn.x * body.inv_inertia_x) +
               (rxn.y * rxn.y * body.inv_inertia_y) +
               (rxn.z * rxn.z * body.inv_inertia_z);
    };

    FpVel total = lin_mass + angular_term(a, ra, n) + angular_term(b, rb, n);
    if (total.raw == 0) return FpVel::zero();
    return FpVel::one() / total;
}

// ---------------------------------------------------------------------------
// Build contact constraints from manifolds
// ---------------------------------------------------------------------------
void ConstraintSolver::build_contact_constraints(
    std::array<Entity, MAX_ENTITIES>& entities,
    const std::vector<ContactManifold>& manifolds,
    const SurfaceState& surface) {

    contact_count_ = 0;
    for (const auto& manifold : manifolds) {
        if (!manifold.valid()) continue;
        EntityId ida = manifold.pair.a;
        EntityId idb = manifold.pair.b;
        if (ida >= MAX_ENTITIES || idb >= MAX_ENTITIES) continue;
        Entity& ea = entities[ida];
        Entity& eb = entities[idb];
        if (!ea.has(COMP_RIGIDBODY) || !eb.has(COMP_RIGIDBODY)) continue;

        FpVel e_combined  = combined_restitution(manifold.mat_a, manifold.mat_b);
        FpVel mu_combined = combined_kinetic_mu(manifold.mat_a, manifold.mat_b);
        mu_combined = surface.apply_wetness(mu_combined);

        for (uint8_t ci = 0; ci < manifold.count; ++ci) {
            if (contact_count_ >= (int)contact_pool_.size()) break;
            const ContactPoint& cp = manifold.points[ci];
            ContactConstraint& c = contact_pool_[contact_count_++];

            c.ea = ida; c.eb = idb;
            c.normal = cp.normal;
            compute_tangents(c.normal, c.tangent1, c.tangent2);

            // Contact offsets from COM (pos in Q15.16)
            auto to_vel = [](Vec3Pos p) -> Vec3Vel {
                return { FpVel{p.x.raw << 8}, FpVel{p.y.raw << 8}, FpVel{p.z.raw << 8} };
            };
            Vec3Vel pa = to_vel(ea.rigidbody.position);
            Vec3Vel pb = to_vel(eb.rigidbody.position);
            c.ra = cp.world_pos_a - pa;
            c.rb = cp.world_pos_b - pb;

            c.depth               = cp.depth;
            c.combined_restitution= e_combined;
            c.combined_mu         = mu_combined;

            // Effective masses
            c.effective_mass_n  = effective_mass(ea.rigidbody, eb.rigidbody,
                                                  c.ra, c.rb, c.normal);
            c.effective_mass_t1 = effective_mass(ea.rigidbody, eb.rigidbody,
                                                  c.ra, c.rb, c.tangent1);
            c.effective_mass_t2 = effective_mass(ea.rigidbody, eb.rigidbody,
                                                  c.ra, c.rb, c.tangent2);

            // Baumgarte bias: β/dt * max(0, depth - slop)
            FpVel excess = cp.depth - params_.baumgarte_slop;
            if (excess.raw < 0) excess = FpVel::zero();
            // Use dt_inv ≈ 1/substep_dt = 240 Hz
            FpVel dt_inv = FpVel::from_float(240.0f);
            c.bias = params_.baumgarte_beta * dt_inv * excess;

            // Restitution: only if relative velocity > slop
            Vec3Vel rel_vel = ea.rigidbody.velocity_at(c.ra) -
                              eb.rigidbody.velocity_at(c.rb);
            FpVel vn = rel_vel.dot(c.normal);
            if ((-vn) > params_.restitution_slop) {
                c.bias += e_combined * (-vn);
            }

            // Warm-start: initialise from cached impulses
            c.lambda_n  = cp.cached_impulse_n  * params_.warm_start_scale;
            c.lambda_t1 = cp.cached_impulse_t1 * params_.warm_start_scale;
            c.lambda_t2 = cp.cached_impulse_t2 * params_.warm_start_scale;
        }
    }
}

// ---------------------------------------------------------------------------
// Build joint constraints from all skeletons
// ---------------------------------------------------------------------------
void ConstraintSolver::build_joint_constraints(
    std::array<Entity, MAX_ENTITIES>& entities) {
    joint_count_ = 0;
    for (EntityId id = PLAYER_BEGIN; id < PLAYER_END; ++id) {
        Entity& e = entities[id];
        if (!e.has(COMP_SKELETON | COMP_RIGIDBODY)) continue;
        Skeleton& sk = e.skeleton;

        for (uint8_t b = 1; b < BONE_COUNT; ++b) { // skip root (PELVIS)
            if (joint_count_ >= (int)joint_pool_.size()) break;
            JointConstraint& jc = joint_pool_[joint_count_++];
            jc.bone_a      = id;
            jc.bone_b      = id;  // Same entity, different bone
            jc.bone_index  = b;
            jc.joint_type  = (b == BONE_HEAD) ? JointType::SPHERICAL : JointType::REVOLUTE;
            jc.limit       = BONE_LIMITS[b];
            jc.motor_torque= sk.bones[b].motor_torque;
            // Warm-start: carry lambda from previous frame
            // (jc.lambda[] is preserved between frames for joints)
        }
    }
}

// ---------------------------------------------------------------------------
// Warm start — apply cached impulses before iteration
// ---------------------------------------------------------------------------
void ConstraintSolver::warm_start(std::array<Entity, MAX_ENTITIES>& entities) {
    for (int i = 0; i < contact_count_; ++i) {
        ContactConstraint& c = contact_pool_[i];
        Entity& ea = entities[c.ea];
        Entity& eb = entities[c.eb];

        Vec3Vel impulse_n  = c.normal   * c.lambda_n;
        Vec3Vel impulse_t1 = c.tangent1 * c.lambda_t1;
        Vec3Vel impulse_t2 = c.tangent2 * c.lambda_t2;
        Vec3Vel total      = impulse_n + impulse_t1 + impulse_t2;

        ea.rigidbody.apply_impulse( total, c.ra);
        eb.rigidbody.apply_impulse(-total, c.rb);
    }
}

// ---------------------------------------------------------------------------
// Solve one contact constraint (velocity impulse)
// ---------------------------------------------------------------------------
void ConstraintSolver::solve_contact_velocity(ContactConstraint& c,
                                               Entity& ea, Entity& eb) {
    // ─── Normal impulse ──────────────────────────────────────────────────────
    Vec3Vel rel_vel = ea.rigidbody.velocity_at(c.ra) -
                      eb.rigidbody.velocity_at(c.rb);
    FpVel vn = rel_vel.dot(c.normal);

    // λ_n = -(vn - bias) * effective_mass_n
    FpVel lambda_delta = (-(vn - c.bias)) * c.effective_mass_n;

    // Clamp accumulated: λ_n >= 0 (no pulling)
    FpVel old_n = c.lambda_n;
    c.lambda_n  = fp_clamp(old_n + lambda_delta, FpVel::zero(),
                            params_.max_impulse);
    lambda_delta = c.lambda_n - old_n;

    Vec3Vel impulse_n = c.normal * lambda_delta;
    ea.rigidbody.apply_impulse( impulse_n, c.ra);
    eb.rigidbody.apply_impulse(-impulse_n, c.rb);

    // ─── Friction impulse (tangent 1) ─────────────────────────────────────
    rel_vel = ea.rigidbody.velocity_at(c.ra) - eb.rigidbody.velocity_at(c.rb);
    FpVel vt1 = rel_vel.dot(c.tangent1);
    FpVel lambda_t1_delta = (-vt1) * c.effective_mass_t1;
    FpVel old_t1 = c.lambda_t1;
    FpVel max_friction = c.combined_mu * c.lambda_n;
    c.lambda_t1 = fp_clamp(old_t1 + lambda_t1_delta, -max_friction, max_friction);
    lambda_t1_delta = c.lambda_t1 - old_t1;

    Vec3Vel impulse_t1 = c.tangent1 * lambda_t1_delta;
    ea.rigidbody.apply_impulse( impulse_t1, c.ra);
    eb.rigidbody.apply_impulse(-impulse_t1, c.rb);

    // ─── Friction impulse (tangent 2) ─────────────────────────────────────
    rel_vel = ea.rigidbody.velocity_at(c.ra) - eb.rigidbody.velocity_at(c.rb);
    FpVel vt2 = rel_vel.dot(c.tangent2);
    FpVel lambda_t2_delta = (-vt2) * c.effective_mass_t2;
    FpVel old_t2 = c.lambda_t2;
    // 2D friction cone: |λ_t| ≤ μ * λ_n (both tangent components together)
    FpVel total_t_sq = c.lambda_t1 * c.lambda_t1 + c.lambda_t2 * c.lambda_t2;
    FpVel max_t_sq   = max_friction * max_friction;
    c.lambda_t2 = fp_clamp(old_t2 + lambda_t2_delta, -max_friction, max_friction);
    // Rescale if combined exceeds cone
    FpVel combined_t = c.lambda_t1 * c.lambda_t1 + c.lambda_t2 * c.lambda_t2;
    if (combined_t > max_t_sq && max_t_sq.raw > 0) {
        FpVel scale = max_friction / fp_sqrt(combined_t);
        c.lambda_t1 = c.lambda_t1 * scale;
        c.lambda_t2 = c.lambda_t2 * scale;
    }
    lambda_t2_delta = c.lambda_t2 - old_t2;

    Vec3Vel impulse_t2 = c.tangent2 * lambda_t2_delta;
    ea.rigidbody.apply_impulse( impulse_t2, c.ra);
    eb.rigidbody.apply_impulse(-impulse_t2, c.rb);
}

// ---------------------------------------------------------------------------
// Solve joint: spring-damper style angular constraint
// ---------------------------------------------------------------------------
void ConstraintSolver::solve_joint_velocity(JointConstraint& j, Entity& ea) {
    Skeleton& sk = ea.skeleton;
    BoneState& bone = sk.bones[j.bone_index];

    // Current angle
    FpVel angle = bone.joint_angle;

    // Velocity at joint: simplified as angular vel component along joint axis
    FpVel av = ea.rigidbody.angular_velocity.dot(j.axis);

    // Motor drive: proportional + derivative controller
    FpVel stiffness  = FpVel::from_float(500.0f);  // N·m/rad
    FpVel damping    = FpVel::from_float(40.0f);   // N·m·s/rad
    FpVel angle_err  = bone.target_angle - angle;
    FpVel torque     = stiffness * angle_err - damping * av;

    // Clamp to max torque
    FpVel max_t = BONE_MAX_TORQUE[j.bone_index];
    torque = fp_clamp(torque, -max_t, max_t);

    // Apply as angular impulse (torque * dt)
    Vec3Vel ang_impulse = j.axis * (torque * SUBSTEP_DT);
    ea.rigidbody.angular_velocity += {
        ang_impulse.x * ea.rigidbody.inv_inertia_x,
        ang_impulse.y * ea.rigidbody.inv_inertia_y,
        ang_impulse.z * ea.rigidbody.inv_inertia_z,
    };

    // Hard joint limit enforcement
    if (j.joint_type == JointType::REVOLUTE) {
        if (angle < j.limit.min_angle && av < FpVel::zero()) {
            // Clamp to prevent motion past limit
            ea.rigidbody.angular_velocity = Vec3Vel::zero();
        }
        if (angle > j.limit.max_angle && av > FpVel::zero()) {
            ea.rigidbody.angular_velocity = Vec3Vel::zero();
        }
    }
}

// ---------------------------------------------------------------------------
// Emit events for significant contacts
// ---------------------------------------------------------------------------
void ConstraintSolver::emit_contact_events(
    std::array<Entity, MAX_ENTITIES>& entities,
    EventQueue& events, uint32_t frame) const {
    for (int i = 0; i < contact_count_; ++i) {
        const ContactConstraint& c = contact_pool_[i];
        if (c.lambda_n.raw < FpVel::from_float(0.5f).raw) continue; // low impulse, skip
        PhysicsEvent ev;
        ev.type              = EventType::COLLISION;
        ev.frame             = frame;
        ev.entity_a          = c.ea;
        ev.entity_b          = c.eb;
        ev.contact_normal    = c.normal;
        ev.impulse_magnitude = c.lambda_n;
        events.push(ev);
    }
}

// ---------------------------------------------------------------------------
// Main solve
// ---------------------------------------------------------------------------
void ConstraintSolver::solve(std::array<Entity, MAX_ENTITIES>& entities,
                              std::vector<ContactManifold>& manifolds,
                              const SurfaceState& surface,
                              EventQueue& events,
                              uint32_t frame) {
    // Manifolds are already sorted by EntityPair (from broad phase sort)
    build_contact_constraints(entities, manifolds, surface);
    build_joint_constraints(entities);

    // Warm start
    warm_start(entities);

    // Iterate (Sequential Impulse)
    for (int iter = 0; iter < params_.iterations; ++iter) {
        // Contacts — deterministic order (already sorted by EntityPair)
        for (int i = 0; i < contact_count_; ++i) {
            ContactConstraint& c = contact_pool_[i];
            if (c.ea >= MAX_ENTITIES || c.eb >= MAX_ENTITIES) continue;
            solve_contact_velocity(c, entities[c.ea], entities[c.eb]);
        }

        // Joints — deterministic order (by entity ID, then bone index)
        for (int i = 0; i < joint_count_; ++i) {
            JointConstraint& j = joint_pool_[i];
            if (j.bone_a >= MAX_ENTITIES) continue;
            solve_joint_velocity(j, entities[j.bone_a]);
        }
    }

    // Write back cached impulses to manifolds for next frame warm-start
    int ci = 0;
    for (auto& manifold : manifolds) {
        if (!manifold.valid()) continue;
        for (uint8_t pi = 0; pi < manifold.count && ci < contact_count_; ++pi, ++ci) {
            manifold.points[pi].cached_impulse_n  = contact_pool_[ci].lambda_n;
            manifold.points[pi].cached_impulse_t1 = contact_pool_[ci].lambda_t1;
            manifold.points[pi].cached_impulse_t2 = contact_pool_[ci].lambda_t2;
        }
    }

    emit_contact_events(entities, events, frame);
}

} // namespace dspe