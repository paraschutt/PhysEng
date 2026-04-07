#include "apc_quat.h"
#include "apc_mat3.h" 
#include "apc_math_common.h"
#include <cmath>

namespace apc {

// Extract quaternion from a rotation matrix.
// Uses Shepperd's method for numerical stability (finds largest diagonal element first).
Quat Quat::from_rotation_matrix(const Mat3& m) {
    float trace = m.m[0] + m.m[4] + m.m[8];
    
    if (trace > 0.0f) {
        float s = 0.5f / std::sqrt(trace + 1.0f);
        return Quat(
            (m.m[5] - m.m[7]) * s,
            (m.m[6] - m.m[2]) * s,
            (m.m[1] - m.m[3]) * s,
            0.25f / s
        );
    } else {
        // Find the largest diagonal element
        if (m.m[0] > m.m[4] && m.m[0] > m.m[8]) {
            float s = 2.0f * std::sqrt(1.0f + m.m[0] - m.m[4] - m.m[8]);
            return Quat(
                0.25f * s,
                (m.m[1] + m.m[3]) / s,
                (m.m[6] + m.m[2]) / s,
                (m.m[5] - m.m[7]) / s
            );
        } else if (m.m[4] > m.m[8]) {
            float s = 2.0f * std::sqrt(1.0f + m.m[4] - m.m[0] - m.m[8]);
            return Quat(
                (m.m[1] + m.m[3]) / s,
                0.25f * s,
                (m.m[5] + m.m[7]) / s,
                (m.m[6] - m.m[2]) / s
            );
        } else {
            float s = 2.0f * std::sqrt(1.0f + m.m[8] - m.m[0] - m.m[4]);
            return Quat(
                (m.m[6] + m.m[2]) / s,
                (m.m[5] + m.m[7]) / s,
                0.25f * s,
                (m.m[1] - m.m[3]) / s
            );
        }
    }
}

Mat3 Quat::to_mat3() const {
    return Mat3::from_quat(*this);
}

} // namespace apc