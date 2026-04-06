#pragma once
#include "apc_vec3.h"
#include "apc_quat.h"
#include <cmath>

namespace apc {

// Column-major 3x3 matrix (standard for graphics/physics)
// m[0..2] = Col0, m[3..5] = Col1, m[6..8] = Col2
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

    // 3x3 Inverse via Cofactors (Deterministic: no row-pivoting variance like Gauss-Jordan)
    APC_FORCEINLINE Mat3 inverse() const {
        float a = m[0], b = m[1], c = m[2];
        float d = m[3], e = m[4], f = m[5];
        float g = m[6], h = m[7], i = m[8];

        float det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
        
        if (std::abs(det) < APC_EPSILON) return identity(); // Degenerate fallback
        
        float inv_det = 1.0f / det;

        return Mat3{{
            (e * i - f * h) * inv_det, (c * h - b * i) * inv_det, (b * f - c * e) * inv_det,
            (f * g - d * i) * inv_det, (a * i - c * g) * inv_det, (c * d - a * f) * inv_det,
            (d * h - e * g) * inv_det, (b * g - a * h) * inv_det, (a * e - b * d) * inv_det
        }};
    }

    // Extract diagonal (used for world-space inertia scaling)
    APC_FORCEINLINE Vec3 diagonal() const {
        return Vec3(m[0], m[4], m[8]);
    }
};

} // namespace apc