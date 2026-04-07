#include "apc_math/apc_mat3.h"
#include "apc_math/apc_vec3.h"
#include <cassert>
#include <cmath>

void test_mat3_basic() {
    apc::Mat3 id = apc::Mat3::identity();
    assert(id.m[0] == 1.0f && id.m[4] == 1.0f && id.m[8] == 1.0f);

    // Transpose of identity is identity
    apc::Mat3 t = id.transpose();
    assert(t.m[0] == 1.0f && t.m[4] == 1.0f && t.m[8] == 1.0f);

    // Diagonal extraction
    apc::Vec3 d = id.diagonal();
    assert(d.x == 1.0f && d.y == 1.0f && d.z == 1.0f);
}

void test_mat3_inverse() {
    // Scale matrix [2, 0, 0, 0, 4, 0, 0, 0, 8]
    apc::Mat3 scale;
    for(int i=0; i<9; ++i) scale.m[i] = 0.0f;
    scale.m[0] = 2.0f; scale.m[4] = 4.0f; scale.m[8] = 8.0f;

    apc::Mat3 inv = scale.inverse();
    
    // Should be [0.5, 0, 0, 0, 0.25, 0, 0, 0, 0.125]
    assert(std::abs(inv.m[0] - 0.5f) < 0.0001f);
    assert(std::abs(inv.m[4] - 0.25f) < 0.0001f);
    assert(std::abs(inv.m[8] - 0.125f) < 0.0001f);

    // Multiply scale * inv should equal identity
    apc::Mat3 result = apc::Mat3::multiply(scale, inv);
    assert(std::abs(result.m[0] - 1.0f) < 0.0001f);
    assert(std::abs(result.m[4] - 1.0f) < 0.0001f);
    assert(std::abs(result.m[8] - 1.0f) < 0.0001f);
}

void test_mat3_degenerate_inverse() {
    // BRANCH COVERAGE: Zero matrix triggers fallback to Identity
    apc::Mat3 zero;
    for(int i=0; i<9; ++i) zero.m[i] = 0.0f;

    apc::Mat3 inv = zero.inverse();
    
    // Verify it returns identity, NOT NaN
    assert(std::abs(inv.m[0] - 1.0f) < 0.0001f);
    assert(std::abs(inv.m[4] - 1.0f) < 0.0001f);
    assert(std::abs(inv.m[8] - 1.0f) < 0.0001f);
    assert(inv.m[1] == 0.0f); // Check off-diagonals are zero
}

int main() {
    test_mat3_basic();
    test_mat3_inverse();
    test_mat3_degenerate_inverse();
    return 0;
}