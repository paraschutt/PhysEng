#include "apc_math/apc_vec3.h"
#include <cassert>
#include <cmath>

void test_vec3_basic_algebra() {
    apc::Vec3 a(1.0f, 2.0f, 3.0f);
    apc::Vec3 b(4.0f, 5.0f, 6.0f);

    // Add
    apc::Vec3 c = apc::Vec3::add(a, b);
    assert(c.x == 5.0f && c.y == 7.0f && c.z == 9.0f);

    // Sub
    c = apc::Vec3::sub(b, a);
    assert(c.x == 3.0f && c.y == 3.0f && c.z == 3.0f);

    // Scale
    c = apc::Vec3::scale(a, 2.0f);
    assert(c.x == 2.0f && c.y == 4.0f && c.z == 6.0f);

    // Dot
    assert(apc::Vec3::dot(a, b) == 32.0f); // 1*4 + 2*5 + 3*6

    // Cross
    apc::Vec3 cross = apc::Vec3::cross(apc::Vec3(1,0,0), apc::Vec3(0,1,0));
    assert(cross.x == 0.0f && cross.y == 0.0f && cross.z == 1.0f);

    // Cross orthogonality
    cross = apc::Vec3::cross(a, b);
    assert(std::abs(apc::Vec3::dot(cross, a)) < 0.0001f);
    assert(std::abs(apc::Vec3::dot(cross, b)) < 0.0001f);
}

void test_vec3_edge_cases() {
    // BRANCH COVERAGE: Zero-length normalize returns (0,0,0)
    apc::Vec3 zero(0.0f, 0.0f, 0.0f);
    apc::Vec3 normalized = apc::Vec3::normalize(zero);
    assert(normalized.x == 0.0f && normalized.y == 0.0f && normalized.z == 0.0f);

    // BRANCH COVERAGE: Safe normalize returns fallback
    apc::Vec3 fallback(0.0f, 1.0f, 0.0f);
    normalized = apc::Vec3::safe_normalize(zero, fallback);
    assert(normalized.y == 1.0f);

    // Bitwise equality: +0.0f is NOT equal to -0.0f bitwise
    apc::Vec3 pos_zero(0.0f, 0.0f, 0.0f);
    apc::Vec3 neg_zero(-0.0f, -0.0f, -0.0f);
    assert(!pos_zero.equals_exact(neg_zero)); 
    assert(pos_zero.equals_approx(neg_zero)); // Approximate doesn't care about sign of zero

    // Length squared
    assert(apc::Vec3::length_sq(apc::Vec3(3.0f, 4.0f, 0.0f)) == 25.0f);
}

int main() {
    test_vec3_basic_algebra();
    test_vec3_edge_cases();
    return 0;
}