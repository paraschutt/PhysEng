#pragma once
#include "apc_solver/apc_rigid_body.h"
#include "apc_collision/apc_sphere_sphere.h" // Pulls in canonical ContactPoint definition
#include "apc_math/apc_vec3.h"
#include <vector>

namespace apc {

// REMOVED duplicate ContactPoint struct here

struct VelocityConstraint {
    uint32_t id_a;
    uint32_t id_b;
    Vec3 normal;
    Vec3 contact_point;
    Vec3 r_a;
    Vec3 r_b;
    float penetration;
    float normal_mass;
    float accumulated_impulse;
};

class Solver3D {
public:
    void prepare(const ContactPoint& contact, uint32_t id_a, uint32_t id_b, const std::vector<RigidBody>& bodies) {
        VelocityConstraint c;
        c.id_a = id_a;
        c.id_b = id_b;
        c.normal = contact.normal;
        c.contact_point = contact.point_on_a;
        c.penetration = contact.penetration;
        c.accumulated_impulse = 0.0f;

        const RigidBody& a = bodies[id_a];
        const RigidBody& b = bodies[id_b];

        c.r_a = Vec3::sub(c.contact_point, a.position);
        c.r_b = Vec3::sub(c.contact_point, b.position);

        Vec3 cross_a = Vec3::cross(c.r_a, c.normal);
        Vec3 cross_b = Vec3::cross(c.r_b, c.normal);
        
        // Manual Mat3 * Vec3 multiply for term_a
        const Mat3& inv_I_a = a.world_inverse_inertia;
        Vec3 rot_cross_a(
            cross_a.x * inv_I_a.m[0] + cross_a.y * inv_I_a.m[3] + cross_a.z * inv_I_a.m[6],
            cross_a.x * inv_I_a.m[1] + cross_a.y * inv_I_a.m[4] + cross_a.z * inv_I_a.m[7],
            cross_a.x * inv_I_a.m[2] + cross_a.y * inv_I_a.m[5] + cross_a.z * inv_I_a.m[8]
        );
        float term_a = Vec3::dot(rot_cross_a, cross_a);

        // Manual Mat3 * Vec3 multiply for term_b
        const Mat3& inv_I_b = b.world_inverse_inertia;
        Vec3 rot_cross_b(
            cross_b.x * inv_I_b.m[0] + cross_b.y * inv_I_b.m[3] + cross_b.z * inv_I_b.m[6],
            cross_b.x * inv_I_b.m[1] + cross_b.y * inv_I_b.m[4] + cross_b.z * inv_I_b.m[7],
            cross_b.x * inv_I_b.m[2] + cross_b.y * inv_I_b.m[5] + cross_b.z * inv_I_b.m[8]
        );
        float term_b = Vec3::dot(rot_cross_b, cross_b);
        
        float inv_mass_sum = a.inverse_mass + b.inverse_mass + term_a + term_b;
        c.normal_mass = (inv_mass_sum > 0.0f) ? (1.0f / inv_mass_sum) : 0.0f;

        constraints.push_back(c);
    }

    void solve(std::vector<RigidBody>& bodies, float dt) {
        const float BAUMGARTE_FACTOR = 0.2f;
        const float SLOP = 0.005f;

        for (auto& c : constraints) {
            RigidBody& a = bodies[c.id_a];
            RigidBody& b = bodies[c.id_b];

            Vec3 vel_a = Vec3::add(a.linear_velocity, Vec3::cross(a.angular_velocity, c.r_a));
            Vec3 vel_b = Vec3::add(b.linear_velocity, Vec3::cross(b.angular_velocity, c.r_b));
            Vec3 rel_vel = Vec3::sub(vel_a, vel_b);
            
            float vel_along_normal = Vec3::dot(rel_vel, c.normal);
            float positional_error = std::max(c.penetration - SLOP, 0.0f) * BAUMGARTE_FACTOR / dt;

            float delta_impulse = c.normal_mass * (-vel_along_normal + positional_error);
            float new_impulse = std::max(c.accumulated_impulse + delta_impulse, 0.0f);
            delta_impulse = new_impulse - c.accumulated_impulse;
            c.accumulated_impulse = new_impulse;

            Vec3 impulse = Vec3::scale(c.normal, delta_impulse);
            a.apply_impulse(impulse, c.contact_point);
            b.apply_impulse(Vec3::scale(impulse, -1.0f), c.contact_point);
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
            
            body.linear_velocity = Vec3::scale(body.linear_velocity, 0.999f);
            body.angular_velocity = Vec3::scale(body.angular_velocity, 0.998f);
        }
    }

    void clear() { constraints.clear(); }

private:
    std::vector<VelocityConstraint> constraints;
};

} // namespace apc