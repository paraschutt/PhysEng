#pragma once
#include "apc_vec3.h"
#include <cmath>

namespace apc {

// Forward declaration: apc_mat3.h includes apc_quat.h, 
// so we must forward declare Mat3 here to avoid circular includes.
struct Mat3;

struct Quat {
    float x, y, z, w;

    APC_FORCEINLINE Quat() : x(0), y(0), z(0), w(1.0f) {} // Identity
    APC_FORCEINLINE Quat(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}

    APC_FORCEINLINE static Quat identity() { return Quat(0, 0, 0, 1.0f); }

    // Deterministic axis-angle to quaternion
    APC_FORCEINLINE static Quat from_axis_angle(const Vec3& axis, float angle) {
        float half_angle = angle * 0.5f;
        float s = std::sin(half_angle);
        float c = std::cos(half_angle);
        return Quat(axis.x * s, axis.y * s, axis.z * s, c);
    }

    // Declaration only! Implementation is in apc_quat.cpp (which includes apc_mat3.h)
    static Quat from_rotation_matrix(const Mat3& m);

    // Extract rotation matrix from quaternion
    Mat3 to_mat3() const;

    // Hamilton product (strict ordering: x,y,z,w)
    APC_FORCEINLINE static Quat multiply(const Quat& a, const Quat& b) {
        return Quat(
            a.x * b.w + a.w * b.x + a.y * b.z - a.z * b.y,
            a.y * b.w + a.w * b.y + a.z * b.x - a.x * b.z,
            a.z * b.w + a.w * b.z + a.x * b.y - a.y * b.x,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
        );
    }

    // Rotate a vector by this quaternion
    APC_FORCEINLINE Vec3 rotate(const Vec3& v) const {
        Vec3 qv = Vec3(x, y, z);
        Vec3 t = Vec3::scale(Vec3::cross(qv, v), 2.0f);
        return Vec3::add(v, Vec3::add(Vec3::scale(t, w), Vec3::cross(qv, t)));
    }

    APC_FORCEINLINE static Quat normalize(const Quat& q) {
        float len_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        if (len_sq < APC_EPSILON_SQ) return identity();
        float inv_len = 1.0f / std::sqrt(len_sq);
        return Quat(q.x * inv_len, q.y * inv_len, q.z * inv_len, q.w * inv_len);
    }

    // Inverse of a normalized quaternion (just flips the axis)
    APC_FORCEINLINE static Quat inverse(const Quat& q) {
        return Quat(-q.x, -q.y, -q.z, q.w);
    }
};

} // namespace apc