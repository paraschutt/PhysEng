#pragma once
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include <cmath>

namespace apc {

// Column-major 3x3 matrix
struct Mat3 {
    float m[9];

    APC_FORCEINLINE static Mat3 identity() {
        return Mat3{{
            1, 0, 0,
            0, 1, 0,
            0, 0, 1
        }};
    }

    APC_FORCEINLINE static Mat3 from_quat(const Quat& q) {
        float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
        float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
        float wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;

        return Mat3{{
            1.0f - 2.0f * (yy + zz), 2.0f * (xy + wz),         2.0f * (xz - wy),
            2.0f * (xy - wz),         1.0f - 2.0f * (xx + zz), 2.0f * (yz + wx),
            2.0f * (xz + wy),         2.0f * (yz - wx),         1.0f - 2.0f * (xx + yy)
        }};
    }

    // Transpose
    APC_FORCEINLINE Mat3 transpose() const {
        return Mat3{{
            m[0], m[3], m[6],
            m[1], m[4], m[7],
            m[2], m[5], m[8]
        }};
    }

    // Declarations only - Implementations moved to apc_mat3.cpp to avoid MSVC duplicate body errors
    static Mat3 multiply(const Mat3& a, const Mat3& b);

    // Element-wise addition
    static Mat3 add(const Mat3& a, const Mat3& b);
    Mat3 inverse() const;

    // Extract diagonal (used for world-space inertia scaling)
    APC_FORCEINLINE Vec3 diagonal() const {
        return Vec3(m[0], m[4], m[8]);
    }

    // Transform a vector by this matrix: out = M * v
    // Column-major layout: out.x = v.x*m[0] + v.y*m[3] + v.z*m[6], etc.
    // Explicitly ordered to prevent compiler FP reordering.
    APC_FORCEINLINE Vec3 transform_vec(const Vec3& v) const {
        return Vec3(
            v.x * m[0] + v.y * m[3] + v.z * m[6],
            v.x * m[1] + v.y * m[4] + v.z * m[7],
            v.x * m[2] + v.y * m[5] + v.z * m[8]
        );
    }
};

} // namespace apc