#pragma once
#include "apc_math/apc_vec3.h"
#include <cmath>

namespace apc {

struct SphereCollider {
    float radius;
};

struct ContactPoint {
    Vec3 point_on_a;
    Vec3 point_on_b;
    Vec3 normal; // Points from A to B
    float penetration;
};

// Fast path for sphere-sphere. Returns true if overlapping.
inline bool detect_sphere_sphere(
    const Vec3& pos_a, const SphereCollider& col_a,
    const Vec3& pos_b, const SphereCollider& col_b,
    ContactPoint& out_contact) 
{
    Vec3 diff = Vec3::sub(pos_b, pos_a);
    float dist_sq = Vec3::length_sq(diff);
    float radius_sum = col_a.radius + col_b.radius;
    
    // FIX 1: Changed >= to > to allow exact touching state
    if (dist_sq > radius_sum * radius_sum) {
        return false; // Separated
    }

    float dist = std::sqrt(dist_sq);
    if (dist < APC_EPSILON) {
        out_contact.normal = Vec3(0.0f, 1.0f, 0.0f);
        out_contact.penetration = radius_sum;
    } else {
        // FIX 2: Flipped the sign! Normal must point from B to A (away from surface)
        out_contact.normal = Vec3::scale(diff, -1.0f / dist);
        out_contact.penetration = radius_sum - dist;
    }

    // With normal pointing B->A, point_on_a is pushed AWAY from B (correct surface point)
    out_contact.point_on_a = Vec3::add(pos_a, Vec3::scale(out_contact.normal, col_a.radius));
    
    // point_on_b is pushed TOWARDS A (correct surface point)
    out_contact.point_on_b = Vec3::sub(pos_b, Vec3::scale(out_contact.normal, col_b.radius));
    
    return true;
}

} // namespace apc