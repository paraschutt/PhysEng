#pragma once
// ============================================================================
// DSPE Narrow Phase — GJK distance/EPA penetration depth for contact generation
// Shapes: Capsule-Capsule, Sphere-Capsule, Sphere-Box, Box-Box
// Contact manifold: persisted between frames for warm-starting
// ============================================================================
#include "../math_types.h"
#include "../components.h"
#include "../entity.h"
#include <array>
#include <vector>

namespace dspe {

// ---------------------------------------------------------------------------
// Contact point (single feature)
// ---------------------------------------------------------------------------
struct ContactPoint {
    Vec3Vel world_pos_a{};     // Contact point on body A world space
    Vec3Vel world_pos_b{};     // Contact point on body B world space
    Vec3Vel normal{};          // Contact normal, pointing from B to A (outward A)
    FpVel   depth{};           // Penetration depth (positive = overlapping)
    FpVel   cached_impulse_n{};// Warm-start: normal impulse from previous frame
    FpVel   cached_impulse_t1{};
    FpVel   cached_impulse_t2{};
    bool    is_new{true};
};

// ---------------------------------------------------------------------------
// Contact manifold — up to 4 contact points per pair
// Persisted across frames for warm-starting
// ---------------------------------------------------------------------------
static constexpr int MAX_CONTACTS_PER_MANIFOLD = 4;

struct ContactManifold {
    EntityPair pair{};
    std::array<ContactPoint, MAX_CONTACTS_PER_MANIFOLD> points{};
    uint8_t    count{0};
    MaterialId mat_a{MAT_DRY_GRASS};
    MaterialId mat_b{MAT_DRY_GRASS};
    uint32_t   last_frame{0};  // For staleness detection

    void clear() { count = 0; }
    bool valid()  const { return count > 0; }
};

// ---------------------------------------------------------------------------
// GJK support function types
// ---------------------------------------------------------------------------
using SupportFn = Vec3Vel(*)(const void* shape, const QuatVel& rot,
                              const Vec3Pos& pos, Vec3Vel dir);

// ---------------------------------------------------------------------------
// GJK Simplex (up to 4 points in 3D)
// ---------------------------------------------------------------------------
struct Simplex {
    Vec3Vel pts[4]{};
    int     n{0};

    void clear()           { n = 0; }
    void push(Vec3Vel v)   { pts[n++] = v; }
    Vec3Vel& operator[](int i) { return pts[i]; }
    const Vec3Vel& operator[](int i) const { return pts[i]; }
};

// ---------------------------------------------------------------------------
// Narrow phase interface
// ---------------------------------------------------------------------------
class NarrowPhase {
public:
    NarrowPhase();

    // Generate contacts between a pair of entities
    // Returns true if contact was found
    bool generate_contact(const Entity& ea, const Entity& eb,
                          ContactManifold& out_manifold) const;

    // Update persistent manifold (remove stale contacts, add new)
    void update_manifold(ContactManifold& manifold,
                         const ContactManifold& new_contacts) const;

    // Per-shape pair specialisations (public for testing)
    bool capsule_capsule(const Collider& ca, const Vec3Pos& pa, const QuatVel& qa,
                         const Collider& cb, const Vec3Pos& pb, const QuatVel& qb,
                         ContactManifold& out) const;

    bool sphere_capsule (const Collider& cs, const Vec3Pos& ps,
                         const Collider& cc, const Vec3Pos& pc, const QuatVel& qc,
                         ContactManifold& out) const;

    bool sphere_box     (const Collider& cs, const Vec3Pos& ps,
                         const Collider& cb, const Vec3Pos& pb, const QuatVel& qb,
                         ContactManifold& out) const;

    bool sphere_sphere  (const Collider& ca, const Vec3Pos& pa,
                         const Collider& cb, const Vec3Pos& pb,
                         ContactManifold& out) const;

private:
    // GJK: compute closest points between two convex shapes
    // Returns false if shapes overlap (call EPA for depth)
    bool gjk(SupportFn sa, const void* shapea, const QuatVel& rota, const Vec3Pos& pa,
              SupportFn sb, const void* shapeb, const QuatVel& rotb, const Vec3Pos& pb,
              Simplex& simplex, Vec3Vel& closest_on_a, Vec3Vel& closest_on_b) const;

    // EPA: compute penetration depth from overlapping GJK simplex
    bool epa(SupportFn sa, const void* shapea, const QuatVel& rota, const Vec3Pos& pa,
             SupportFn sb, const void* shapeb, const QuatVel& rotb, const Vec3Pos& pb,
             Simplex& simplex, Vec3Vel& normal, FpVel& depth) const;

    // Closest point on line segment to point P
    static Vec3Vel closest_point_on_segment(Vec3Vel a, Vec3Vel b, Vec3Vel p);

    // Closest pair of points on two line segments
    static void closest_points_segment_segment(
        Vec3Vel a0, Vec3Vel a1, Vec3Vel b0, Vec3Vel b1,
        Vec3Vel& out_pa, Vec3Vel& out_pb);

    // Update simplex direction for GJK (returns false if origin enclosed)
    bool update_simplex(Simplex& s, Vec3Vel& dir) const;

    // Support functions for each shape
    static Vec3Vel support_capsule(const void* shape, const QuatVel& rot,
                                   const Vec3Pos& pos, Vec3Vel dir);
    static Vec3Vel support_sphere (const void* shape, const QuatVel& rot,
                                   const Vec3Pos& pos, Vec3Vel dir);
    static Vec3Vel support_box    (const void* shape, const QuatVel& rot,
                                   const Vec3Pos& pos, Vec3Vel dir);
};

} // namespace dspe