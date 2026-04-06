#pragma once
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include "apc_math/apc_mat3.h"

namespace apc {

struct OBB {
    Vec3 center;
    Vec3 extents;     // Half-lengths (x, y, z)
    Quat orientation; 
    Mat3 rotation;    // Cached from orientation for fast transforms

    void update_cache() {
        rotation = Mat3::from_quat(orientation);
    }

    // Transform a point from local OBB space to world space
    Vec3 local_to_world(const Vec3& local) const {
        return Vec3::add(center, rotation.rotate(local));
    }

    // Transform a world direction to local OBB space (no translation)
    Vec3 world_to_local_dir(const Vec3& world) const {
        // Fast inverse rotation = transpose
        Mat3 inv_rot = rotation.transpose();
        return inv_rot.rotate(world);
    }
};

// Strict OBB-vs-OBB overlap test using SAT.
// Returns true if overlapping. NO early exits based on heuristic—strict axis order.
inline bool obb_intersect(const OBB& a, const OBB& b) {
    Vec3 d = Vec3::sub(b.center, a.center);
    
    // Mat3 stores columns. Transpose gives rows (local axes).
    Mat3 a_rot = a.rotation.transpose();
    Mat3 b_rot = b.rotation.transpose();

    Vec3 a_axes[3] = { {a_rot.m[0], a_rot.m[3], a_rot.m[6]}, 
                       {a_rot.m[1], a_rot.m[4], a_rot.m[7]}, 
                       {a_rot.m[2], a_rot.m[5], a_rot.m[8]} };
    Vec3 b_axes[3] = { {b_rot.m[0], b_rot.m[3], b_rot.m[6]}, 
                       {b_rot.m[1], b_rot.m[4], b_rot.m[7]}, 
                       {b_rot.m[2], b_rot.m[5], b_rot.m[8]} };

    float a_ext[3] = {a.extents.x, a.extents.y, a.extents.z};
    float b_ext[3] = {b.extents.x, b.extents.y, b.extents.z};

    // Helper lambda for single axis test
    auto test_axis = [&](const Vec3& axis) -> bool {
        float len_sq = Vec3::length_sq(axis);
        if (len_sq < APC_EPSILON_SQ) return true; // Degenerate axis, skip
        
        float ra = std::abs(Vec3::dot(a_axes[0], axis)) * a_ext[0] +
                   std::abs(Vec3::dot(a_axes[1], axis)) * a_ext[1] +
                   std::abs(Vec3::dot(a_axes[2], axis)) * a_ext[2];
                   
        float rb = std::abs(Vec3::dot(b_axes[0], axis)) * b_ext[0] +
                   std::abs(Vec3::dot(b_axes[1], axis)) * b_ext[1] +
                   std::abs(Vec3::dot(b_axes[2], axis)) * b_ext[2];

        float dist = std::abs(Vec3::dot(d, axis));
        return dist <= (ra + rb); // True if overlapping on this axis
    };

    // STRICT TESTING ORDER: 15 axes total.
    // 1-3: A's face normals
    if (!test_axis(a_axes[0])) return false;
    if (!test_axis(a_axes[1])) return false;
    if (!test_axis(a_axes[2])) return false;
    
    // 4-6: B's face normals
    if (!test_axis(b_axes[0])) return false;
    if (!test_axis(b_axes[1])) return false;
    if (!test_axis(b_axes[2])) return false;

    // 7-15: Edge-edge cross products (9 axes)
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            Vec3 cross = Vec3::cross(a_axes[i], b_axes[j]);
            if (!test_axis(cross)) return false;
        }
    }

    return true; // No separating axis found
}

} // namespace apc