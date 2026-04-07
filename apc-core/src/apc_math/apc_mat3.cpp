#include "apc_mat3.h"
#include "apc_math_common.h"
#include <cmath>

namespace apc {

// Standard 3x3 matrix multiplication C = A * B
// Explicitly ordered to prevent compiler auto-vectorization from reordering FP ops
Mat3 Mat3::multiply(const Mat3& a, const Mat3& b) {
    Mat3 out;
    
    // Column 0 of out
    out.m[0] = a.m[0]*b.m[0] + a.m[3]*b.m[1] + a.m[6]*b.m[2];
    out.m[1] = a.m[1]*b.m[0] + a.m[4]*b.m[1] + a.m[7]*b.m[2];
    out.m[2] = a.m[2]*b.m[0] + a.m[5]*b.m[1] + a.m[8]*b.m[2];
    
    // Column 1 of out
    out.m[3] = a.m[0]*b.m[3] + a.m[3]*b.m[4] + a.m[6]*b.m[5];
    out.m[4] = a.m[1]*b.m[3] + a.m[4]*b.m[4] + a.m[7]*b.m[5];
    out.m[5] = a.m[2]*b.m[3] + a.m[5]*b.m[4] + a.m[8]*b.m[5];
    
    // Column 2 of out
    out.m[6] = a.m[0]*b.m[6] + a.m[3]*b.m[7] + a.m[6]*b.m[8];
    out.m[7] = a.m[1]*b.m[6] + a.m[4]*b.m[7] + a.m[7]*b.m[8];
    out.m[8] = a.m[2]*b.m[6] + a.m[5]*b.m[7] + a.m[8]*b.m[8];
    
    return out;
}

// 3x3 Inverse via Cofactors.
// We use this instead of Gauss-Jordan because GJ requires row swapping based on 
// epsilon comparisons, which leads to divergent code paths across platforms.
Mat3 Mat3::inverse() const {
    // Cache matrix elements for cleaner math
    float a = m[0], b = m[1], c = m[2];
    float d = m[3], e = m[4], f = m[5];
    float g = m[6], h = m[7], i = m[8];

    // Calculate determinant
    float det = a * (e * i - f * h) 
              - b * (d * i - f * g) 
              + c * (d * h - e * g);

    // Deterministic fallback: If matrix is degenerate, do NOT return NaN. 
    // Return identity. The physics engine will handle identity inertia poorly,
    // but it won't instantly NaN-propagate and corrupt the state hash.
    if (std::abs(det) < APC_EPSILON) {
        return Mat3::identity();
    }
    
    float inv_det = 1.0f / det;

    // Calculate inverse using adjugate matrix (cofactor transpose)
    // Order of operations is strictly defined
    return Mat3{{
        (e * i - f * h) * inv_det, (c * h - b * i) * inv_det, (b * f - c * e) * inv_det,
        (f * g - d * i) * inv_det, (a * i - c * g) * inv_det, (c * d - a * f) * inv_det,
        (d * h - e * g) * inv_det, (b * g - a * h) * inv_det, (a * e - b * d) * inv_det
    }};
}

Mat3 Mat3::add(const Mat3& a, const Mat3& b) {
    return Mat3{{
        a.m[0] + b.m[0], a.m[1] + b.m[1], a.m[2] + b.m[2],
        a.m[3] + b.m[3], a.m[4] + b.m[4], a.m[5] + b.m[5],
        a.m[6] + b.m[6], a.m[7] + b.m[7], a.m[8] + b.m[8]
    }};
}

} // namespace apc