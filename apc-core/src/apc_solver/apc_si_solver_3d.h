#pragma once
#include "apc_solver/apc_rigid_body.h"
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
    // Can be tuned per-scenario. Default provides noticeable but not extreme friction.
    float friction_coefficient = 0.4f;

    // Solver configuration
    float baumgarte_factor = 0.2f;     // Position correction strength (0-1)
    float baumgarte_slop = 0.005f;     // Penetration threshold before correction
    float restitution = 0.0f;          // Coefficient of restitution (0=inelastic, 1=elastic)
    uint32_t velocity_iterations = 8;  // Sequential impulse iterations per frame
    float linear_damping = 0.999f;     // Per-frame velocity damping
    float angular_damping = 0.998f;    // Per-frame angular damping

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

        // --- Normal effective mass (same as before) ---
        Vec3 cross_a = Vec3::cross(c.r_a, c.normal);
        Vec3 cross_b = Vec3::cross(c.r_b, c.normal);

        const Mat3& inv_I_a = a.world_inverse_inertia;
        Vec3 rot_cross_a = inv_I_a.transform_vec(cross_a);
        float term_a = Vec3::dot(rot_cross_a, cross_a);

        const Mat3& inv_I_b = b.world_inverse_inertia;
        Vec3 rot_cross_b = inv_I_b.transform_vec(cross_b);
        float term_b = Vec3::dot(rot_cross_b, cross_b);

        float inv_mass_sum = a.inverse_mass + b.inverse_mass + term_a + term_b;
        c.normal_mass = (inv_mass_sum > 0.0f) ? (1.0f / inv_mass_sum) : 0.0f;

        // --- Tangential (friction) effective mass ---
        // For the tangential direction, we use an average of the two tangent basis vectors.
        // Since we don't know the sliding direction yet, we compute mass for an arbitrary
        // tangent and store it. The actual friction solve picks the tangent from the
        // relative velocity at solve time.
        //
        // Simplification: tangential mass ≈ normal mass for uniform bodies.
        // For accuracy we'd recompute per-tangent-direction, but the sequential impulse
        // solver converges with this approximation across iterations.
        c.friction_mass = c.normal_mass;

        constraints.push_back(c);
    }

    void solve(std::vector<RigidBody>& bodies, float dt) {
        for (uint32_t iter = 0; iter < velocity_iterations; ++iter) {
            for (auto& c : constraints) {
                RigidBody& a = bodies[c.id_a];
                RigidBody& b = bodies[c.id_b];

                // Relative velocity at contact point
                Vec3 vel_a = Vec3::add(a.linear_velocity, Vec3::cross(a.angular_velocity, c.r_a));
                Vec3 vel_b = Vec3::add(b.linear_velocity, Vec3::cross(b.angular_velocity, c.r_b));
                Vec3 rel_vel = Vec3::sub(vel_a, vel_b);

                // ---- NORMAL IMPULSE ----
                float vel_along_normal = Vec3::dot(rel_vel, c.normal);
                float positional_error = std::max(c.penetration - baumgarte_slop, 0.0f) * baumgarte_factor / dt;

                // Restitution: add bounce bias when approaching (vel_along_normal < 0)
                // Only apply when objects are approaching and penetration is small
                // (don't bounce when deeply penetrating — that causes explosions)
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
                //
                // The friction impulse opposes the tangential component of the relative
                // velocity. It is clamped to a cone defined by:
                //   |friction_impulse| <= mu * normal_impulse
                //
                // This is the standard sequential impulse friction approach used in
                // Box2D, Bullet, and most game physics engines.

                // Recompute relative velocity after normal impulse was applied
                vel_a = Vec3::add(a.linear_velocity, Vec3::cross(a.angular_velocity, c.r_a));
                vel_b = Vec3::add(b.linear_velocity, Vec3::cross(b.angular_velocity, c.r_b));
                rel_vel = Vec3::sub(vel_a, vel_b);

                // Decompose relative velocity into normal and tangential components
                float vel_n = Vec3::dot(rel_vel, c.normal);
                Vec3 vel_tangent = Vec3::sub(rel_vel, Vec3::scale(c.normal, vel_n));

                float tangent_len_sq = Vec3::length_sq(vel_tangent);

                if (tangent_len_sq > APC_EPSILON_SQ) {
                    // Normalize the tangential velocity to get the friction direction
                    float tangent_len = std::sqrt(tangent_len_sq);
                    Vec3 tangent = Vec3::scale(vel_tangent, 1.0f / tangent_len);

                    // Compute the impulse needed to zero out tangential velocity
                    // delta_friction = -friction_mass * tangent_speed
                    float delta_friction_mag = c.friction_mass * tangent_len;

                    // Coulomb cone clamp: friction impulse <= mu * normal impulse
                    float max_friction = friction_coefficient * c.accumulated_normal_impulse;
                    float new_friction_mag = std::min(delta_friction_mag + Vec3::length(c.accumulated_friction_impulse), max_friction);
                    float applied_friction = new_friction_mag - Vec3::length(c.accumulated_friction_impulse);

                    // If the accumulated friction is already at the cone boundary, skip
                    if (applied_friction > 0.0f) {
                        // Friction impulse opposes the sliding direction
                        Vec3 friction_impulse = Vec3::scale(tangent, -applied_friction);

                        c.accumulated_friction_impulse = Vec3::add(c.accumulated_friction_impulse, friction_impulse);

                        a.apply_impulse(friction_impulse, c.contact_point);
                        b.apply_impulse(Vec3::scale(friction_impulse, -1.0f), c.contact_point);
                    }
                }
            }
        }
    }

    void integrate(std::vector<RigidBody>& bodies, float dt) {
        for (auto& body : bodies) {
            if (body.inverse_mass == 0.0f) continue;

            body.position = Vec3::add(body.position, Vec3::scale(body.linear_velocity, dt));

            Vec3 half_ang = Vec3::scale(body.angular_velocity, dt * 0.5f);
            Quat delta_q(half_ang.x, half_ang.y, half_ang.z, 0.0f);
            body.orientation = Quat::normalize(Quat::multiply(delta_q, body.orientation));

            body.update_world_inertia();

            body.linear_velocity = Vec3::scale(body.linear_velocity, linear_damping);
            body.angular_velocity = Vec3::scale(body.angular_velocity, angular_damping);
        }
    }

    // Prepare multiple contacts from a manifold
    void prepare_manifold(const ContactPoint* contacts, uint32_t count,
                          uint32_t id_a, uint32_t id_b, const std::vector<RigidBody>& bodies) {
        for (uint32_t i = 0; i < count; ++i) {
            prepare(contacts[i], id_a, id_b, bodies);
        }
    }

    void clear() { constraints.clear(); }

private:
    std::vector<VelocityConstraint> constraints;
};

} // namespace apc
