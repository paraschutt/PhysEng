#pragma once
#include "apc_vec3.h"
#include <cmath>

namespace apc {

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
        // Optimized formula: t = 2 * cross(q.xyz, v), result = v + q.w * t + cross(q.xyz, t)
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
};

} // namespace apc