#pragma once
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include "apc_math/apc_mat3.h"
#include <cstdint>

namespace apc {

struct RigidBody {
    Vec3 position;
    Vec3 linear_velocity;
    float inverse_mass;

    Quat orientation;
    Vec3 angular_velocity;
    
    Mat3 local_inverse_inertia;
    Mat3 world_inverse_inertia;

    // Collision filtering (bitfield)
    uint32_t collision_layer = 1;       // Which layer(s) this body is ON (default: layer 0)
    uint32_t collision_mask  = 0xFFFFFFFF; // Which layer(s) this body collides WITH (default: all)

    // Check if two bodies should collide based on their layer/mask
    static bool should_collide(const RigidBody& a, const RigidBody& b) {
        return (a.collision_layer & b.collision_mask) != 0 &&
               (b.collision_layer & a.collision_mask) != 0;
    }

    void update_world_inertia() {
        Mat3 rot = Mat3::from_quat(orientation);
        Mat3 inv_rot = rot.transpose();
        world_inverse_inertia = Mat3::multiply(rot, Mat3::multiply(local_inverse_inertia, inv_rot));
    }

    void apply_impulse(const Vec3& impulse, const Vec3& contact_point) {
        linear_velocity = Vec3::add(linear_velocity, Vec3::scale(impulse, inverse_mass));
        
        Vec3 r = Vec3::sub(contact_point, position);
        Vec3 torque_impulse = Vec3::cross(r, impulse);
        
        // Manual Mat3 * Vec3 multiply (avoids needing operator overloads)
        const Mat3& m = world_inverse_inertia;
        Vec3 angular_impulse(
            torque_impulse.x * m.m[0] + torque_impulse.y * m.m[3] + torque_impulse.z * m.m[6],
            torque_impulse.x * m.m[1] + torque_impulse.y * m.m[4] + torque_impulse.z * m.m[7],
            torque_impulse.x * m.m[2] + torque_impulse.y * m.m[5] + torque_impulse.z * m.m[8]
        );
        
        angular_velocity = Vec3::add(angular_velocity, angular_impulse);
    }
};

} // namespace apc