#include "apc_math/apc_quat.h"
#include "apc_math/apc_mat3.h"
#include "apc_math/apc_vec3.h"
#include <cassert>
#include <cmath>

void test_quat_basic() {
    apc::Quat id = apc::Quat::identity();
    assert(id.x == 0.0f && id.y == 0.0f && id.z == 0.0f && id.w == 1.0f);

    // Identity * Identity = Identity
    apc::Quat res = apc::Quat::multiply(id, id);
    assert(res.w == 1.0f);
}

void test_quat_rotations() {
    // 90 degree rotation around Z axis
    apc::Quat qz = apc::Quat::from_axis_angle(apc::Vec3(0, 0, 1.0f), apc::APC_HALF_PI);
    
    // Rotate X axis (1,0,0) by 90 deg around Z -> (0,1,0)
    apc::Vec3 rotated = qz.rotate(apc::Vec3(1.0f, 0.0f, 0.0f));
    assert(std::abs(rotated.x) < 0.001f);
    assert(std::abs(rotated.y - 1.0f) < 0.001f);
    assert(std::abs(rotated.z) < 0.001f);
}

void test_quat_from_matrix() {
    // Create a known rotation matrix (90 deg around Z) in COLUMN-MAJOR layout.
    //
    // +90° Z rotation matrix (row-major):
    //   [ cos90  -sin90  0 ]   [ 0  -1  0 ]
    //   [ sin90   cos90  0 ] = [ 1   0  0 ]
    //   [   0       0    1 ]   [ 0   0  1 ]
    //
    // Column-major storage (m[col*3 + row]):
    //   Col 0 (X-axis): (cos90, sin90, 0) = (0, 1, 0) → m[0]=0, m[1]=1, m[2]=0
    //   Col 1 (Y-axis): (-sin90, cos90, 0) = (-1, 0, 0) → m[3]=-1, m[4]=0, m[5]=0
    //   Col 2 (Z-axis): (0, 0, 1) → m[6]=0, m[7]=0, m[8]=1
    apc::Mat3 m;
    m.m[0] = 0.0f;  m.m[1] = 1.0f;  m.m[2] = 0.0f;
    m.m[3] = -1.0f; m.m[4] = 0.0f;  m.m[5] = 0.0f;
    m.m[6] = 0.0f;  m.m[7] = 0.0f;  m.m[8] = 1.0f;

    apc::Quat q = apc::Quat::from_rotation_matrix(m);
    apc::Vec3 test_vec(1.0f, 0.0f, 0.0f);
    apc::Vec3 rotated = q.rotate(test_vec);

    assert(std::abs(rotated.x) < 0.001f);
    assert(std::abs(rotated.y - 1.0f) < 0.001f);
    assert(std::abs(rotated.z) < 0.001f);
}

int main() {
    test_quat_basic();
    test_quat_rotations();
    test_quat_from_matrix();
    return 0;
}