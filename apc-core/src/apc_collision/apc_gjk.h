#pragma once
#include "apc_math/apc_vec3.h"
#include <cstdint>

namespace apc {

struct ConvexHull {
    // Function pointer to get vertex in local space
    // MUST return deterministic tie-breaks if dots are equal (handled inside GJK)
    using SupportFunc = Vec3 (*)(const void* shape, const Vec3& dir, uint32_t& out_vertex_id);
    
    const void* user_data;
    SupportFunc support;
};

struct GJKResult {
    bool intersecting;
    Vec3 separation_vector; // Valid only if intersecting == false
};

class GJKBoolean {
public:
    static constexpr uint32_t MAX_ITERATIONS = 32;

    static GJKResult query(const ConvexHull& hull_a, const ConvexHull& hull_b) {
        Vec3 direction = Vec3(1.0f, 0.0f, 0.0f); // Initial arbitrary direction
        
        Simplex simplex;
        uint32_t discard_id_a, discard_id_b;
        
        // Get initial point
        Vec3 point = support(hull_a, hull_b, direction, discard_id_a, discard_id_b);
        simplex.add(point);
        
        // Flip direction for next search
        direction = Vec3::scale(direction, -1.0f);
        
        for (uint32_t i = 0; i < MAX_ITERATIONS; ++i) {
            point = support(hull_a, hull_b, direction, discard_id_a, discard_id_b);
            
            // If the furthest point in the opposite direction isn't past the origin,
            // the shapes do not intersect.
            if (Vec3::dot(point, direction) < 0.0f) {
                return { false, direction }; // Separated
            }
            
            simplex.add(point);
            
            if (do_simplex(simplex, direction)) {
                return { true, Vec3(0.0f, 0.0f, 0.0f) }; // Intersecting!
            }
        }
        
        // Failed to converge (degenerate shapes floating point noise)
        // Fallback: Assume separated to prevent physics explosions
        return { false, direction };
    }

private:
    struct Simplex {
        Vec3 points[4];
        int count = 0;

        void add(const Vec3& p) {
            if (count < 4) points[count++] = p;
        }
    };

    static Vec3 triple_product(const Vec3& a, const Vec3& b, const Vec3& c) {
        // (A x B) x C = B(A.C) - A(B.C)
        float ac = Vec3::dot(a, c);
        float bc = Vec3::dot(b, c);
        return Vec3::sub(Vec3::scale(b, ac), Vec3::scale(a, bc));
    }

    static Vec3 support(const ConvexHull& a, const ConvexHull& b, const Vec3& dir, uint32_t& out_id_a, uint32_t& out_id_b) {
        Vec3 dir_neg = Vec3::scale(dir, -1.0f);
        Vec3 p_a = a.support(a.user_data, dir, out_id_a);
        Vec3 p_b = b.support(b.user_data, dir_neg, out_id_b);
        return Vec3::sub(p_a, p_b); // Minkowski difference
    }

    static bool do_simplex(Simplex& simplex, Vec3& direction) {
        switch (simplex.count) {
            case 2: return line_case(simplex, direction);
            case 3: return triangle_case(simplex, direction);
            case 4: return tetrahedron_case(simplex, direction);
            default: return false;
        }
    }

    static bool line_case(Simplex& s, Vec3& dir) {
        Vec3 A = s.points[1];
        Vec3 B = s.points[0];
        Vec3 AB = Vec3::sub(B, A);
        Vec3 AO = Vec3::scale(A, -1.0f);

        if (Vec3::dot(AB, AO) > 0.0f) {
            dir = triple_product(AB, AO, AB);
        } else {
            s.points[0] = A;
            s.count = 1;
            dir = AO;
        }
        return false;
    }

    static bool triangle_case(Simplex& s, Vec3& dir) {
        Vec3 A = s.points[2];
        Vec3 B = s.points[1];
        Vec3 C = s.points[0];
        
        Vec3 AB = Vec3::sub(B, A);
        Vec3 AC = Vec3::sub(C, A);
        Vec3 AO = Vec3::scale(A, -1.0f);
        Vec3 ABC = Vec3::cross(AB, AC);

        if (Vec3::dot(Vec3::cross(ABC, AC), AO) > 0.0f) {
            if (Vec3::dot(AC, AO) > 0.0f) {
                s.points[0] = A; s.points[1] = C; s.count = 2;
                dir = triple_product(AC, AO, AC);
            } else {
                s.points[0] = A; s.points[1] = B; s.count = 2;
                return line_case(s, dir);
            }
        } else {
            if (Vec3::dot(Vec3::cross(AB, ABC), AO) > 0.0f) {
                s.points[0] = A; s.points[1] = B; s.count = 2;
                return line_case(s, dir);
            } else {
                if (Vec3::dot(ABC, AO) > 0.0f) {
                    dir = ABC;
                } else {
                    s.points[1] = A; s.points[2] = B; s.points[0] = C;
                    dir = Vec3::scale(ABC, -1.0f);
                }
            }
        }
        return false;
    }

    static bool tetrahedron_case(Simplex& s, Vec3& dir) {
        Vec3 A = s.points[3];
        Vec3 B = s.points[2];
        Vec3 C = s.points[1];
        Vec3 D = s.points[0];
        
        Vec3 AB = Vec3::sub(B, A);
        Vec3 AC = Vec3::sub(C, A);
        Vec3 AD = Vec3::sub(D, A);
        Vec3 AO = Vec3::scale(A, -1.0f);

        Vec3 ABC = Vec3::cross(AB, AC);
        Vec3 ACD = Vec3::cross(AC, AD);
        Vec3 ADB = Vec3::cross(AD, AB);

        if (Vec3::dot(ABC, AO) > 0.0f) {
            s.points[0] = B; s.points[1] = C; s.points[2] = A; s.count = 3;
            return triangle_case(s, dir);
        }
        if (Vec3::dot(ACD, AO) > 0.0f) {
            s.points[0] = C; s.points[1] = D; s.points[2] = A; s.count = 3;
            return triangle_case(s, dir);
        }
        if (Vec3::dot(ADB, AO) > 0.0f) {
            s.points[0] = D; s.points[1] = B; s.points[2] = A; s.count = 3;
            return triangle_case(s, dir);
        }
        
        return true; // Origin is inside tetrahedron
    }
};

} // namespace apc