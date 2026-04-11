#pragma once
// =============================================================================
// apc_si_solver_3d.h — Sequential Impulse solver with warmstart, ball-CCD,
//                        dynamic iterations, and sleep awareness
// =============================================================================
//
// Solves contact constraints using the Sequential Impulse (SI) method with:
//   - Warmstart support (reads accumulated impulses from ContactManager)
//   - Dynamic solver iterations (extra passes for ball contacts)
//   - CCD (Continuous Collision Detection) for ball tunneling prevention
//   - Sleep-aware solve/integrate (skips sleeping bodies)
//
// Pipeline per frame:
//   1. ContactManager::update()           — match contacts, warmstart impulses
//   2. Solver3D::prepare() or prepare_warmstarted()  — build VelocityConstraints
//   3. Solver3D::solve()                  — SI iterations (base + ball extra)
//   4. Solver3D::integrate()              — Euler integration + sleep check
//   5. ContactManager::store_impulse()    — write back impulses for next frame
//
// Design:
//   - Header-only, apc:: namespace
//   - Deterministic: same inputs -> same outputs
//   - C++17
//
// =============================================================================

#include "apc_solver/apc_rigid_body.h"
#include "apc_solver/apc_contact_manager.h"
#include "apc_collision/apc_sphere_sphere.h" // Pulls in canonical ContactPoint definition
#include "apc_math/apc_vec3.h"
#include <vector>
#include <cmath>
#include <cstdint>

namespace apc {

struct VelocityConstraint {
    uint32_t id_a;
    uint32_t id_b;
    Vec3 normal;
    Vec3 contact_point;
    Vec3 r_a;
    Vec3 r_b;
    float penetration;
    float normal_mass;

    // Normal impulse accumulation (clamped >= 0)
    float accumulated_normal_impulse;

    // Tangential (friction) impulse accumulation
    Vec3 accumulated_friction_impulse;
    float friction_mass;  // Effective mass for tangential direction
};

class Solver3D {
public:
    // Friction coefficient (Coulomb model). 0.0 = frictionless, 1.0 = high grip.
    float friction_coefficient = 0.4f;

    // Velocity threshold below which friction impulse is zeroed (prevents jitter)
    float friction_velocity_threshold = 0.01f;

    // Solver configuration
    float baumgarte_factor = 0.2f;     // Position correction strength (0-1)
    float baumgarte_slop = 0.005f;     // Penetration threshold before correction
    float restitution = 0.0f;          // Coefficient of restitution (0=inelastic, 1=elastic)
    uint32_t velocity_iterations = 8;  // Sequential impulse iterations per frame
    float linear_damping = 0.999f;     // Per-frame velocity damping
    float angular_damping = 0.998f;    // Per-frame angular damping

    // --- Dynamic ball iterations ---
    // Extra SI iterations applied only to contacts involving a ball body.
    // Balls have high mass disparity with athletes and high velocity,
    // requiring more solver cycles to converge stably.
    uint32_t ball_extra_iterations = 4;

    // --- CCD (Continuous Collision Detection) configuration ---
    // When a ball moves more than (bounding_radius * ccd_factor) per frame,
    // a swept-sphere check is performed against the ground plane to prevent
    // tunneling. The ball velocity is clamped to the time-of-impact if a
    // hit is detected.
    float ccd_factor = 0.8f;  // Movement threshold = bounding_radius * ccd_factor

    // --- Sleep system thresholds ---
    float sleep_linear_vel_sq  = 0.01f * 0.01f;   // 0.01 m/s
    float sleep_angular_vel_sq = 0.05f * 0.05f;    // 0.05 rad/s
    uint32_t sleep_frame_threshold = 60u;           // ~1 second at 60Hz

    // =========================================================================
    // prepare — Build a VelocityConstraint from a contact point
    // =========================================================================
    // Cold start: accumulated impulses are zeroed.
    // For warmstarted preparation, use prepare_warmstarted() instead.
    // =========================================================================
    void prepare(const ContactPoint& contact, uint32_t id_a, uint32_t id_b, const std::vector<RigidBody>& bodies) {
        VelocityConstraint c;
        c.id_a = id_a;
        c.id_b = id_b;
        c.normal = contact.normal;
        c.contact_point = contact.point_on_a;
        c.penetration = contact.penetration;
        c.accumulated_normal_impulse = 0.0f;
        c.accumulated_friction_impulse = Vec3(0.0f, 0.0f, 0.0f);

        const RigidBody& a = bodies[id_a];
        const RigidBody& b = bodies[id_b];

        c.r_a = Vec3::sub(c.contact_point, a.position);
        c.r_b = Vec3::sub(c.contact_point, b.position);

        // Normal effective mass
        compute_normal_mass(a, b, c.normal, c.r_a, c.r_b, c.normal_mass);

        // Tangential (friction) effective mass — approximated as normal mass
        c.friction_mass = c.normal_mass;

        constraints.push_back(c);
    }

    // =========================================================================
    // prepare_warmstarted — Build a VelocityConstraint with warmstart impulses
    // =========================================================================
    // Reads accumulated impulses from the ContactManager's persistent manifold
    // for this body pair. If no warmstart data exists, falls back to cold start.
    // =========================================================================
    void prepare_warmstarted(const ContactPoint& contact, uint32_t id_a, uint32_t id_b,
                             const std::vector<RigidBody>& bodies,
                             const ContactManager& contact_mgr)
    {
        VelocityConstraint c;
        c.id_a = id_a;
        c.id_b = id_b;
        c.normal = contact.normal;
        c.contact_point = contact.point_on_a;
        c.penetration = contact.penetration;

        const RigidBody& a = bodies[id_a];
        const RigidBody& b = bodies[id_b];

        c.r_a = Vec3::sub(c.contact_point, a.position);
        c.r_b = Vec3::sub(c.contact_point, b.position);

        // Normal effective mass
        compute_normal_mass(a, b, c.normal, c.r_a, c.r_b, c.normal_mass);

        // Tangential effective mass
        c.friction_mass = c.normal_mass;

        // Attempt warmstart from ContactManager
        bool warmstarted = contact_mgr.apply_warmstart_to_constraint(
            id_a, id_b,
            contact.point_on_a,
            c.accumulated_normal_impulse,
            c.accumulated_friction_impulse
        );

        if (!warmstarted) {
            c.accumulated_normal_impulse = 0.0f;
            c.accumulated_friction_impulse = Vec3(0.0f, 0.0f, 0.0f);
        }

        constraints.push_back(c);
    }

    // =========================================================================
    // solve — Sequential impulse solver with sleep awareness + ball extras
    // =========================================================================
    void solve(std::vector<RigidBody>& bodies, float dt) {
        // Phase 1: Base iterations for ALL contacts
        for (uint32_t iter = 0u; iter < velocity_iterations; ++iter) {
            for (auto& c : constraints) {
                solve_single_constraint(c, bodies, dt);
            }
        }

        // Phase 2: Extra iterations for ball contacts only
        // Balls flagged STATE_IS_BALL get additional solver passes to handle
        // mass disparity and high-speed impacts more stably.
        for (uint32_t iter = 0u; iter < ball_extra_iterations; ++iter) {
            for (auto& c : constraints) {
                if (c.id_a < bodies.size() && c.id_b < bodies.size()) {
                    const RigidBody& a = bodies[c.id_a];
                    const RigidBody& b = bodies[c.id_b];
                    if (a.is_ball() || b.is_ball()) {
                        solve_single_constraint(c, bodies, dt);
                    }
                }
            }
        }
    }

    // =========================================================================
    // integrate — Euler integration with sleep system + CCD for balls
    // =========================================================================
    void integrate(std::vector<RigidBody>& bodies, float dt) {
        for (auto& body : bodies) {
            // Skip kinematic bodies (infinite mass, externally driven)
            if (body.inverse_mass == 0.0f) continue;

            // --- CCD for balls ---
            // Swept-sphere check against ground plane (y=0) to prevent
            // tunneling on high-speed shots. This is the most common
            // tunneling scenario in sport simulations.
            if (body.is_ball() && body.is_ccd_enabled()) {
                perform_ccd_ground(body, dt);
            }

            // --- Euler integration ---
            body.position = Vec3::add(body.position, Vec3::scale(body.linear_velocity, dt));

            Vec3 half_ang = Vec3::scale(body.angular_velocity, dt * 0.5f);
            Quat delta_q(half_ang.x, half_ang.y, half_ang.z, 0.0f);
            body.orientation = Quat::normalize(Quat::multiply(delta_q, body.orientation));

            body.update_world_inertia();

            body.linear_velocity = Vec3::scale(body.linear_velocity, linear_damping);
            body.angular_velocity = Vec3::scale(body.angular_velocity, angular_damping);

            // --- Sleep check ---
            // Only attempt sleep for active (non-kinematic) bodies
            if (body.is_active() && !body.is_kinematic()) {
                body.try_sleep(sleep_linear_vel_sq, sleep_angular_vel_sq,
                               sleep_frame_threshold);
            }
        }
    }

    // Prepare multiple contacts from a manifold (cold start)
    void prepare_manifold(const ContactPoint* contacts, uint32_t count,
                          uint32_t id_a, uint32_t id_b, const std::vector<RigidBody>& bodies) {
        for (uint32_t i = 0u; i < count; ++i) {
            prepare(contacts[i], id_a, id_b, bodies);
        }
    }

    // Prepare multiple contacts from a manifold (warmstarted)
    void prepare_manifold_warmstarted(const ContactPoint* contacts, uint32_t count,
                                       uint32_t id_a, uint32_t id_b,
                                       const std::vector<RigidBody>& bodies,
                                       const ContactManager& contact_mgr) {
        for (uint32_t i = 0u; i < count; ++i) {
            prepare_warmstarted(contacts[i], id_a, id_b, bodies, contact_mgr);
        }
    }

    void clear() { constraints.clear(); }

private:
    std::vector<VelocityConstraint> constraints;

    // =========================================================================
    // compute_normal_mass — Normal effective mass calculation
    // =========================================================================
    APC_FORCEINLINE static void compute_normal_mass(
        const RigidBody& a, const RigidBody& b,
        const Vec3& normal, const Vec3& r_a, const Vec3& r_b,
        float& out_normal_mass)
    {
        Vec3 cross_a = Vec3::cross(r_a, normal);
        Vec3 cross_b = Vec3::cross(r_b, normal);

        const Mat3& inv_I_a = a.world_inverse_inertia;
        Vec3 rot_cross_a = inv_I_a.transform_vec(cross_a);
        float term_a = Vec3::dot(rot_cross_a, cross_a);

        const Mat3& inv_I_b = b.world_inverse_inertia;
        Vec3 rot_cross_b = inv_I_b.transform_vec(cross_b);
        float term_b = Vec3::dot(rot_cross_b, cross_b);

        float inv_mass_sum = a.inverse_mass + b.inverse_mass + term_a + term_b;
        out_normal_mass = (inv_mass_sum > 0.0f) ? (1.0f / inv_mass_sum) : 0.0f;
    }

    // =========================================================================
    // solve_single_constraint — One SI iteration for one contact
    // =========================================================================
    // Handles: normal impulse + Baumgarte position correction + restitution
    //          + Coulomb cone friction.
    // Skips if either body is sleeping (unless woken by a previous impulse).
    // =========================================================================
    APC_FORCEINLINE void solve_single_constraint(
        VelocityConstraint& c, std::vector<RigidBody>& bodies, float dt)
    {
        if (c.id_a >= bodies.size() || c.id_b >= bodies.size()) return;

        RigidBody& a = bodies[c.id_a];
        RigidBody& b = bodies[c.id_b];

        // Skip if both bodies are sleeping
        if (a.is_sleeping() && b.is_sleeping()) return;

        // Relative velocity at contact point
        Vec3 vel_a = Vec3::add(a.linear_velocity, Vec3::cross(a.angular_velocity, c.r_a));
        Vec3 vel_b = Vec3::add(b.linear_velocity, Vec3::cross(b.angular_velocity, c.r_b));
        Vec3 rel_vel = Vec3::sub(vel_a, vel_b);

        // ---- NORMAL IMPULSE ----
        float vel_along_normal = Vec3::dot(rel_vel, c.normal);
        float positional_error = std::max(c.penetration - baumgarte_slop, 0.0f) * baumgarte_factor / dt;

        // Restitution bias: bounce when approaching with small penetration
        float restitution_bias = 0.0f;
        if (restitution > 0.0f) {
            if (vel_along_normal < -APC_EPSILON && c.penetration < baumgarte_slop * 2.0f) {
                restitution_bias = -restitution * vel_along_normal;
            }
        }

        float delta_normal = c.normal_mass * (-vel_along_normal + positional_error + restitution_bias);
        float new_normal_impulse = std::max(c.accumulated_normal_impulse + delta_normal, 0.0f);
        delta_normal = new_normal_impulse - c.accumulated_normal_impulse;
        c.accumulated_normal_impulse = new_normal_impulse;

        Vec3 normal_impulse = Vec3::scale(c.normal, delta_normal);
        a.apply_impulse(normal_impulse, c.contact_point);
        b.apply_impulse(Vec3::scale(normal_impulse, -1.0f), c.contact_point);

        // ---- FRICTION IMPULSE (Coulomb cone model) ----
        // Recompute relative velocity after normal impulse
        vel_a = Vec3::add(a.linear_velocity, Vec3::cross(a.angular_velocity, c.r_a));
        vel_b = Vec3::add(b.linear_velocity, Vec3::cross(b.angular_velocity, c.r_b));
        rel_vel = Vec3::sub(vel_a, vel_b);

        float vel_n = Vec3::dot(rel_vel, c.normal);
        Vec3 vel_tangent = Vec3::sub(rel_vel, Vec3::scale(c.normal, vel_n));

        float tangent_len_sq = Vec3::length_sq(vel_tangent);

        if (tangent_len_sq > friction_velocity_threshold * friction_velocity_threshold) {
            float tangent_len = std::sqrt(tangent_len_sq);
            Vec3 tangent = Vec3::scale(vel_tangent, 1.0f / tangent_len);

            float delta_friction_mag = c.friction_mass * tangent_len;
            float max_friction = friction_coefficient * c.accumulated_normal_impulse;
            float new_friction_mag = std::min(delta_friction_mag + Vec3::length(c.accumulated_friction_impulse), max_friction);
            float applied_friction = new_friction_mag - Vec3::length(c.accumulated_friction_impulse);

            if (applied_friction > 0.0f) {
                Vec3 friction_impulse = Vec3::scale(tangent, -applied_friction);
                c.accumulated_friction_impulse = Vec3::add(c.accumulated_friction_impulse, friction_impulse);
                a.apply_impulse(friction_impulse, c.contact_point);
                b.apply_impulse(Vec3::scale(friction_impulse, -1.0f), c.contact_point);
            }
        }
    }

    // =========================================================================
    // perform_ccd_ground — Swept-sphere CCD against ground plane (y=0)
    // =========================================================================
    // Checks if the ball would tunnel through the ground plane during this
    // timestep. If so, clamps the ball's position and velocity to the
    // time-of-impact (TOI).
    //
    // The ground plane is at y=0 with normal (0,1,0).
    // Ball is treated as a sphere of radius bounding_radius.
    //
    // This prevents the classic "ball through floor" bug on high-speed
    // shots where discrete collision detection misses the contact because
    // the ball moves more than its radius per frame.
    // =========================================================================
    APC_FORCEINLINE void perform_ccd_ground(RigidBody& ball, float dt) {
        if (dt <= 0.0f) return;

        // Ground plane: y=0, normal = (0,1,0)
        // Ball swept sphere: center moves from P0 to P0 + V*dt
        // Closest approach to plane: P0.y + t * V.y (for t in [0,1])

        float p0_y = ball.position.y;
        float v_y = ball.linear_velocity.y;
        float radius = ball.bounding_radius;

        // Check if ball would pass through ground plane
        // Condition: ball starts above ground and ends below (or at) ground - radius
        if (p0_y > radius && v_y < 0.0f) {
            float p1_y = p0_y + v_y * dt;

            if (p1_y <= radius) {
                // Ball would tunnel — compute time of impact
                // p0_y + t * v_y * dt = radius  =>  t = (radius - p0_y) / (v_y * dt)
                float denom = v_y * dt;
                if (denom < -APC_EPSILON) { // Moving downward
                    float t = (radius - p0_y) / denom;
                    if (t < 0.0f) t = 0.0f;
                    if (t > 1.0f) t = 1.0f;

                    // Clamp position to TOI
                    ball.position.x = ball.position.x + ball.linear_velocity.x * dt * t;
                    ball.position.y = radius;  // Place exactly on ground surface
                    ball.position.z = ball.position.z + ball.linear_velocity.z * dt * t;

                    // Reflect Y velocity with restitution
                    if (ball.linear_velocity.y < 0.0f) {
                        ball.linear_velocity.y = -ball.linear_velocity.y * restitution;
                        // Kill tiny bounces
                        if (ball.linear_velocity.y < 0.3f) {
                            ball.linear_velocity.y = 0.0f;
                        }
                    }
                }
            }
        }
    }
};

} // namespace apc
