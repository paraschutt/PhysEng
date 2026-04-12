#pragma once
// =============================================================================
// Collision Dispatcher — Central routing for shape-pair collision detection
// =============================================================================
//
// Provides:
//   - ContactManifold: multi-contact result (up to 4 contacts per pair)
//   - CollisionShape: tagged union of all supported shape types
//   - Shape-type-specific fast-path detectors (sphere-box, sphere-sphere,
//     sphere-plane)
//   - Generic convex-convex detector (GJK boolean -> EPA penetration)
//   - Generic sphere-convex detector (GJK + positioned sphere support + EPA)
//   - dispatch_detect(): routes any shape pair to the correct detector
//
// Normal convention: B->A (push A away from B). All detectors must adhere.
// Determinism: index-order tie-breaking everywhere, no random selection.
//
// =============================================================================

#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include "apc_math/apc_mat3.h"
#include "apc_math/apc_math_common.h"
#include "apc_sphere_sphere.h"     // detect_sphere_sphere, ContactPoint, SphereCollider
#include "apc_gjk.h"              // GJKBoolean, ConvexHull, GJKSimplex
#include "apc_epa.h"              // EPA, EPAResult
#include "apc_obb.h"              // OBB, obb_intersect
#include "apc_plane.h"            // detect_sphere_plane, PlaneCollider
#include "apc_capsule.h"          // CapsuleShapeData, capsule_support, detect_*_capsule*
#include "apc_cylinder.h"         // CylinderShapeData, cylinder_support, detect_sphere_cylinder
#include "apc_support.h"          // Support functions, shape data structs
#include "apc_broadphase.h"       // AABB
#include "apc_capsule.h"          // CapsuleShapeData, capsule support/detection
#include "apc_cylinder.h"         // CylinderShapeData, cylinder support/detection
#include <cstdint>
#include <cmath>

namespace apc {

// ---------------------------------------------------------------------------
// Maximum contacts per collision pair
// ---------------------------------------------------------------------------
static constexpr uint32_t MAX_CONTACTS_PER_PAIR = 4u;

// ---------------------------------------------------------------------------
// ContactManifold — result of a collision query between two shapes
// ---------------------------------------------------------------------------
struct ContactManifold {
    uint32_t id_a;                          // Body A identifier
    uint32_t id_b;                          // Body B identifier (id_a < id_b)
    ContactPoint contacts[MAX_CONTACTS_PER_PAIR];
    uint32_t contact_count;                 // Number of valid contacts (0 = separated)
    bool is_block_solved = false;           // Set by solver: true if ball-athlete pair was block-solved

    void reset() {
        id_a = 0u;
        id_b = 0u;
        contact_count = 0u;
        is_block_solved = false;
    }

    void add_contact(const ContactPoint& cp) {
        if (contact_count < MAX_CONTACTS_PER_PAIR) {
            contacts[contact_count] = cp;
            ++contact_count;
        }
    }
};

// ---------------------------------------------------------------------------
// Shape type enumeration
// ---------------------------------------------------------------------------
enum class ShapeType : uint8_t {
    Sphere      = 0,
    Box         = 1,
    ConvexPiece = 2,   // Single convex hull from decomposition
    Plane       = 3,
    Capsule     = 4,
    Cylinder    = 5,
    Count       = 6
};

// ---------------------------------------------------------------------------
// CollisionShape — tagged union wrapping any supported shape type
// ---------------------------------------------------------------------------
struct CollisionShape {
    ShapeType type;
    Vec3      position;           // World position (center) for all shapes
    Quat      orientation;        // World orientation (identity for sphere/plane)
    Mat3      rotation;           // Cached rotation matrix

    // --- Type-specific data ---
    float     sphere_radius;         // Sphere only
    Vec3      box_half_extents;      // Box only
    const Vec3* convex_vertices;     // ConvexPiece only (non-owning pointer)
    uint32_t  convex_vertex_count;   // ConvexPiece only
    Vec3      plane_normal;          // Plane only (point_on_plane = position)

    // --- Capsule / Cylinder ---
    float     capsule_radius;        // Capsule only
    float     capsule_half_height;   // Capsule only
    float     cylinder_radius;       // Cylinder only
    float     cylinder_half_height;  // Cylinder only

    void update_cache() {
        rotation = Mat3::from_quat(orientation);
    }

    // --- Factory methods ---

    static CollisionShape make_sphere(float radius, const Vec3& pos) {
        CollisionShape s;
        s.type = ShapeType::Sphere;
        s.position = pos;
        s.orientation = Quat::identity();
        s.rotation = Mat3::identity();
        s.sphere_radius = radius;
        s.box_half_extents = Vec3(0.0f, 0.0f, 0.0f);
        s.convex_vertices = nullptr;
        s.convex_vertex_count = 0u;
        s.plane_normal = Vec3(0.0f, 0.0f, 0.0f);
        s.capsule_radius = 0.0f;
        s.capsule_half_height = 0.0f;
        s.cylinder_radius = 0.0f;
        s.cylinder_half_height = 0.0f;
        return s;
    }

    static CollisionShape make_box(const Vec3& half_extents,
                                   const Vec3& pos,
                                   const Quat& orient)
    {
        CollisionShape s;
        s.type = ShapeType::Box;
        s.position = pos;
        s.orientation = orient;
        s.rotation = Mat3::from_quat(orient);
        s.box_half_extents = half_extents;
        s.sphere_radius = 0.0f;
        s.convex_vertices = nullptr;
        s.convex_vertex_count = 0u;
        s.plane_normal = Vec3(0.0f, 0.0f, 0.0f);
        s.capsule_radius = 0.0f;
        s.capsule_half_height = 0.0f;
        s.cylinder_radius = 0.0f;
        s.cylinder_half_height = 0.0f;
        return s;
    }

    static CollisionShape make_convex_piece(
        const Vec3* vertices, uint32_t count,
        const Vec3& pos, const Quat& orient)
    {
        CollisionShape s;
        s.type = ShapeType::ConvexPiece;
        s.position = pos;
        s.orientation = orient;
        s.rotation = Mat3::from_quat(orient);
        s.convex_vertices = vertices;
        s.convex_vertex_count = count;
        s.sphere_radius = 0.0f;
        s.box_half_extents = Vec3(0.0f, 0.0f, 0.0f);
        s.plane_normal = Vec3(0.0f, 0.0f, 0.0f);
        s.capsule_radius = 0.0f;
        s.capsule_half_height = 0.0f;
        s.cylinder_radius = 0.0f;
        s.cylinder_half_height = 0.0f;
        return s;
    }

    static CollisionShape make_plane(const Vec3& point, const Vec3& normal) {
        CollisionShape s;
        s.type = ShapeType::Plane;
        s.position = point;
        s.orientation = Quat::identity();
        s.rotation = Mat3::identity();
        s.plane_normal = normal;
        s.sphere_radius = 0.0f;
        s.box_half_extents = Vec3(0.0f, 0.0f, 0.0f);
        s.convex_vertices = nullptr;
        s.convex_vertex_count = 0u;
        s.capsule_radius = 0.0f;
        s.capsule_half_height = 0.0f;
        s.cylinder_radius = 0.0f;
        s.cylinder_half_height = 0.0f;
        return s;
    }

    static CollisionShape make_capsule(float radius, float half_height,
                                       const Vec3& pos, const Quat& orient)
    {
        CollisionShape s;
        s.type = ShapeType::Capsule;
        s.position = pos;
        s.orientation = orient;
        s.rotation = Mat3::from_quat(orient);
        s.capsule_radius = radius;
        s.capsule_half_height = half_height;
        s.sphere_radius = 0.0f;
        s.box_half_extents = Vec3(0.0f, 0.0f, 0.0f);
        s.convex_vertices = nullptr;
        s.convex_vertex_count = 0u;
        s.plane_normal = Vec3(0.0f, 0.0f, 0.0f);
        s.cylinder_radius = 0.0f;
        s.cylinder_half_height = 0.0f;
        return s;
    }

    static CollisionShape make_cylinder(float radius, float half_height,
                                        const Vec3& pos, const Quat& orient)
    {
        CollisionShape s;
        s.type = ShapeType::Cylinder;
        s.position = pos;
        s.orientation = orient;
        s.rotation = Mat3::from_quat(orient);
        s.cylinder_radius = radius;
        s.cylinder_half_height = half_height;
        s.sphere_radius = 0.0f;
        s.box_half_extents = Vec3(0.0f, 0.0f, 0.0f);
        s.convex_vertices = nullptr;
        s.convex_vertex_count = 0u;
        s.plane_normal = Vec3(0.0f, 0.0f, 0.0f);
        s.capsule_radius = 0.0f;
        s.capsule_half_height = 0.0f;
        return s;
    }

    // -----------------------------------------------------------------------
    // get_aabb — approximate AABB for broadphase
    // -----------------------------------------------------------------------
    AABB get_aabb() const {
        AABB aabb;
        switch (type) {
        case ShapeType::Sphere: {
            aabb.min = Vec3(position.x - sphere_radius,
                             position.y - sphere_radius,
                             position.z - sphere_radius);
            aabb.max = Vec3(position.x + sphere_radius,
                             position.y + sphere_radius,
                             position.z + sphere_radius);
            break;
        }
        case ShapeType::Box: {
            Vec3 he = box_half_extents;
            float corners[8][3] = {
                {-he.x, -he.y, -he.z}, { he.x, -he.y, -he.z},
                {-he.x,  he.y, -he.z}, { he.x,  he.y, -he.z},
                {-he.x, -he.y,  he.z}, { he.x, -he.y,  he.z},
                {-he.x,  he.y,  he.z}, { he.x,  he.y,  he.z}
            };
            aabb.min = Vec3(1e30f, 1e30f, 1e30f);
            aabb.max = Vec3(-1e30f, -1e30f, -1e30f);
            for (int i = 0; i < 8; ++i) {
                Vec3 local(corners[i][0], corners[i][1], corners[i][2]);
                Vec3 world = Vec3::add(position, rotation.transform_vec(local));
                if (world.x < aabb.min.x) aabb.min.x = world.x;
                if (world.y < aabb.min.y) aabb.min.y = world.y;
                if (world.z < aabb.min.z) aabb.min.z = world.z;
                if (world.x > aabb.max.x) aabb.max.x = world.x;
                if (world.y > aabb.max.y) aabb.max.y = world.y;
                if (world.z > aabb.max.z) aabb.max.z = world.z;
            }
            break;
        }
        case ShapeType::ConvexPiece: {
            aabb.min = Vec3(1e30f, 1e30f, 1e30f);
            aabb.max = Vec3(-1e30f, -1e30f, -1e30f);
            for (uint32_t i = 0u; i < convex_vertex_count; ++i) {
                Vec3 world = Vec3::add(position,
                    rotation.transform_vec(convex_vertices[i]));
                if (world.x < aabb.min.x) aabb.min.x = world.x;
                if (world.y < aabb.min.y) aabb.min.y = world.y;
                if (world.z < aabb.min.z) aabb.min.z = world.z;
                if (world.x > aabb.max.x) aabb.max.x = world.x;
                if (world.y > aabb.max.y) aabb.max.y = world.y;
                if (world.z > aabb.max.z) aabb.max.z = world.z;
            }
            break;
        }
        case ShapeType::Plane: {
            // Approximate as a very large flat box oriented along the normal
            const float E = 500.0f;
            float anx = apc::math::abs(plane_normal.x);
            float any = apc::math::abs(plane_normal.y);
            float anz = apc::math::abs(plane_normal.z);
            if (anx >= any && anx >= anz) {
                // Normal is roughly X-aligned
                aabb.min = Vec3(position.x - 0.1f, position.y - E, position.z - E);
                aabb.max = Vec3(position.x + 0.1f, position.y + E, position.z + E);
            } else if (any >= anz) {
                // Normal is roughly Y-aligned
                aabb.min = Vec3(position.x - E, position.y - 0.1f, position.z - E);
                aabb.max = Vec3(position.x + E, position.y + 0.1f, position.z + E);
            } else {
                // Normal is roughly Z-aligned
                aabb.min = Vec3(position.x - E, position.y - E, position.z - 0.1f);
                aabb.max = Vec3(position.x + E, position.y + E, position.z + 0.1f);
            }
            break;
        }
        case ShapeType::Capsule: {
            CapsuleShapeData cap_data;
            cap_data.radius = capsule_radius;
            cap_data.half_height = capsule_half_height;
            cap_data.position = position;
            cap_data.orientation = orientation;
            cap_data.rotation = rotation;
            cap_data.update_cache();
            aabb = capsule_get_aabb(cap_data);
            break;
        }
        case ShapeType::Cylinder: {
            CylinderShapeData cyl_data;
            cyl_data.radius = cylinder_radius;
            cyl_data.half_height = cylinder_half_height;
            cyl_data.position = position;
            cyl_data.orientation = orientation;
            cyl_data.rotation = rotation;
            cyl_data.update_cache();
            aabb = cylinder_get_aabb(cyl_data);
            break;
        }
        default:
            aabb.min = Vec3(0.0f, 0.0f, 0.0f);
            aabb.max = Vec3(0.0f, 0.0f, 0.0f);
            break;
        }
        return aabb;
    }
};

// ---------------------------------------------------------------------------
// detect_sphere_box() — dedicated fast path
// Normal convention: B->A (from box toward sphere).
// ---------------------------------------------------------------------------
inline bool detect_sphere_box(
    const Vec3& sphere_pos, float sphere_radius,
    const Vec3& box_pos, const Vec3& box_half_extents,
    const Mat3& box_rot,
    ContactPoint& out_contact)
{
    // Transform sphere center into box local space
    Vec3 diff = Vec3::sub(sphere_pos, box_pos);
    Mat3 inv_rot = box_rot.transpose();
    Vec3 local = inv_rot.transform_vec(diff);

    // Clamp to box AABB to find closest point on box to sphere center
    float cx = (local.x < -box_half_extents.x) ? -box_half_extents.x :
               (local.x >  box_half_extents.x) ?  box_half_extents.x : local.x;
    float cy = (local.y < -box_half_extents.y) ? -box_half_extents.y :
               (local.y >  box_half_extents.y) ?  box_half_extents.y : local.y;
    float cz = (local.z < -box_half_extents.z) ? -box_half_extents.z :
               (local.z >  box_half_extents.z) ?  box_half_extents.z : local.z;

    Vec3 closest_local(cx, cy, cz);
    Vec3 delta_local = Vec3::sub(local, closest_local);
    float dist_sq = Vec3::length_sq(delta_local);

    // Radius-squared comparison for early out
    float radius_sq = sphere_radius * sphere_radius;
    if (dist_sq > radius_sq) {
        return false; // No collision
    }

    // --- OUTSIDE CASE: sphere center is outside the box surface ---
    if (dist_sq > APC_EPSILON_SQ) {
        float dist = std::sqrt(dist_sq);
        float inv_dist = 1.0f / dist;

        // Normal in local space: from closest point toward sphere center (B->A)
        Vec3 local_normal = Vec3::scale(delta_local, inv_dist);

        // Transform to world space
        out_contact.normal = box_rot.transform_vec(local_normal);
        out_contact.penetration = sphere_radius - dist;

        // Contact points
        // point_on_a: sphere surface facing the box
        out_contact.point_on_a = Vec3::sub(sphere_pos,
            Vec3::scale(out_contact.normal, sphere_radius));
        // point_on_b: closest point on box surface (world space)
        out_contact.point_on_b = Vec3::add(box_pos,
            box_rot.transform_vec(closest_local));

        return true;
    }

    // --- INSIDE CASE: sphere center is inside the box ---
    // Find the axis with minimum penetration depth to the nearest face.
    float px = box_half_extents.x - apc::math::abs(local.x);
    float py = box_half_extents.y - apc::math::abs(local.y);
    float pz = box_half_extents.z - apc::math::abs(local.z);

    // DETERMINISM: x first, then y, then z for equal values.
    Vec3 local_normal(0.0f, 0.0f, 0.0f);
    float pen = 0.0f;

    if (px <= py && px <= pz) {
        local_normal.x = (local.x >= 0.0f) ? 1.0f : -1.0f;
        pen = px + sphere_radius;
    } else if (py <= pz) {
        local_normal.y = (local.y >= 0.0f) ? 1.0f : -1.0f;
        pen = py + sphere_radius;
    } else {
        local_normal.z = (local.z >= 0.0f) ? 1.0f : -1.0f;
        pen = pz + sphere_radius;
    }

    // Transform normal to world space — B->A (from box toward sphere)
    out_contact.normal = box_rot.transform_vec(local_normal);
    out_contact.penetration = pen;

    // Contact points
    // point_on_a: sphere surface facing the box (toward box center)
    out_contact.point_on_a = Vec3::sub(sphere_pos,
        Vec3::scale(out_contact.normal, sphere_radius));

    // point_on_b: box face nearest to the sphere center
    // = sphere_pos + normal * (penetration - sphere_radius)
    // = sphere_pos + normal * px (distance from sphere center to face)
    out_contact.point_on_b = Vec3::add(sphere_pos,
        Vec3::scale(out_contact.normal, pen - sphere_radius));

    return true;
}

// ---------------------------------------------------------------------------
// swap_contact() — negate normal and swap A/B contact points.
// Used when the dispatcher swaps argument order to match a detector.
// ---------------------------------------------------------------------------
inline void swap_contact(ContactPoint& cp) {
    cp.normal = Vec3::scale(cp.normal, -1.0f);
    Vec3 tmp = cp.point_on_a;
    cp.point_on_a = cp.point_on_b;
    cp.point_on_b = tmp;
}

// ---------------------------------------------------------------------------
// Helper: build a ConvexHull from a CollisionShape (box or convex piece).
// The shape_data must outlive the returned ConvexHull (stack allocation).
// ---------------------------------------------------------------------------
namespace detail {

inline ConvexHull make_hull_from_shape(const CollisionShape& shape,
                                       BoxShapeData& box_buf,
                                       ConvexPieceShapeData& convex_buf)
{
    ConvexHull hull;
    if (shape.type == ShapeType::Box) {
        box_buf.half_extents = shape.box_half_extents;
        box_buf.position      = shape.position;
        box_buf.orientation   = shape.orientation;
        box_buf.rotation      = shape.rotation;  // Already cached
        hull.user_data = &box_buf;
        hull.support   = box_support;
    } else {
        // ConvexPiece
        convex_buf.vertices      = shape.convex_vertices;
        convex_buf.vertex_count  = shape.convex_vertex_count;
        convex_buf.position      = shape.position;
        convex_buf.orientation   = shape.orientation;
        convex_buf.rotation      = shape.rotation;  // Already cached
        hull.user_data = &convex_buf;
        hull.support   = convex_piece_support;
    }
    return hull;
}

inline ConvexHull make_sphere_hull(const CollisionShape& shape,
                                   PositionedSphereData& sphere_buf)
{
    sphere_buf.radius   = shape.sphere_radius;
    sphere_buf.position = shape.position;
    ConvexHull hull;
    hull.user_data = &sphere_buf;
    hull.support   = positioned_sphere_support;
    return hull;
}

inline ConvexHull make_capsule_hull(const CollisionShape& shape,
                                     CapsuleShapeData& cap_buf)
{
    cap_buf.radius = shape.capsule_radius;
    cap_buf.half_height = shape.capsule_half_height;
    cap_buf.position = shape.position;
    cap_buf.orientation = shape.orientation;
    cap_buf.rotation = shape.rotation;
    cap_buf.update_cache();
    ConvexHull hull;
    hull.user_data = &cap_buf;
    hull.support = capsule_support;
    return hull;
}

inline ConvexHull make_cylinder_hull(const CollisionShape& shape,
                                      CylinderShapeData& cyl_buf)
{
    cyl_buf.radius = shape.cylinder_radius;
    cyl_buf.half_height = shape.cylinder_half_height;
    cyl_buf.position = shape.position;
    cyl_buf.orientation = shape.orientation;
    cyl_buf.rotation = shape.rotation;
    cyl_buf.update_cache();
    ConvexHull hull;
    hull.user_data = &cyl_buf;
    hull.support = cylinder_support;
    return hull;
}

    // -----------------------------------------------------------------------
    // rebuild_simplex() — build a non-degenerate simplex from 4 tetrahedral
    // directions. Used when GJK produces a degenerate (coplanar/collinear)
    // simplex that EPA cannot use.
    // -----------------------------------------------------------------------
    static void rebuild_simplex(const ConvexHull& hull_a, const ConvexHull& hull_b,
                                 Vec3 out_points[4])
    {
        static const Vec3 dirs[4] = {
            Vec3( 1.0f,  1.0f,  1.0f),
            Vec3( 1.0f, -1.0f, -1.0f),
            Vec3(-1.0f,  1.0f, -1.0f),
            Vec3(-1.0f, -1.0f,  1.0f)
        };
        for (int i = 0; i < 4; ++i) {
            uint32_t id_a = 0, id_b = 0;
            Vec3 pa = hull_a.support(hull_a.user_data, dirs[i], id_a);
            Vec3 pb = hull_b.support(hull_b.user_data, Vec3::scale(dirs[i], -1.0f), id_b);
            out_points[i] = Vec3::sub(pa, pb);
        }
    }

    // -----------------------------------------------------------------------
    // simplex_volume() — 6x signed volume of tetrahedron (0 if degenerate)
    // -----------------------------------------------------------------------
    static float simplex_volume(const Vec3 pts[4]) {
        Vec3 a = pts[3]; // Reference vertex
        Vec3 ab = Vec3::sub(pts[0], a);
        Vec3 ac = Vec3::sub(pts[1], a);
        Vec3 ad = Vec3::sub(pts[2], a);
        return Vec3::dot(Vec3::cross(ab, ac), ad);
    }

    // -----------------------------------------------------------------------
    // gjk_epa_generic() — GJK+EPA for generic convex hull pairs.
    // Returns B->A normal convention.
    // -----------------------------------------------------------------------
    inline bool gjk_epa_generic(const ConvexHull& hull_a, const ConvexHull& hull_b,
                                ContactPoint& out_contact)
    {
        GJKSimplex simplex;
        GJKResult gjk = GJKBoolean::query_with_simplex(hull_a, hull_b, simplex);

        if (!gjk.intersecting) {
            return false;
        }

        EPAResult epa;
        if (simplex.count >= 4 &&
            apc::math::abs(simplex_volume(simplex.points)) > APC_EPSILON)
        {
            epa = EPA::query(simplex.points, simplex.count, hull_a, hull_b);
        } else {
            Vec3 rebuilt[4];
            rebuild_simplex(hull_a, hull_b, rebuilt);
            epa = EPA::query(rebuilt, 4, hull_a, hull_b);
        }

        if (!epa.success) {
            return false;
        }

        // EPA normal is A->B. We need B->A.
        out_contact.normal      = Vec3::scale(epa.normal, -1.0f);
        out_contact.penetration = epa.penetration;
        out_contact.point_on_a  = epa.point_on_a;
        out_contact.point_on_b  = epa.point_on_b;
        return true;
    }

} // namespace detail

// ---------------------------------------------------------------------------
// detect_sphere_convex() — GJK + EPA for sphere vs any convex shape.
//
// Args: sphere_shape is the sphere (first), convex_shape is the box or convex
//       piece (second).
//
// Returns: ContactPoint with normal B->A (from convex toward sphere).
//          point_on_a = point on sphere surface.
//          point_on_b = point on convex surface.
// ---------------------------------------------------------------------------
inline bool detect_sphere_convex(
    const CollisionShape& sphere_shape,
    const CollisionShape& convex_shape,
    ContactPoint& out_contact)
{
    if (convex_shape.type != ShapeType::Box &&
        convex_shape.type != ShapeType::ConvexPiece)
    {
        return false;
    }

    // Build hulls — buffers on stack, valid for this scope
    PositionedSphereData sphere_buf;
    ConvexHull hull_a = detail::make_sphere_hull(sphere_shape, sphere_buf);

    BoxShapeData box_buf;
    ConvexPieceShapeData convex_buf;
    ConvexHull hull_b = detail::make_hull_from_shape(convex_shape, box_buf, convex_buf);

    // GJK boolean + simplex
    GJKSimplex simplex;
    GJKResult gjk = GJKBoolean::query_with_simplex(hull_a, hull_b, simplex);

    if (!gjk.intersecting) {
        return false;
    }

    // EPA for penetration depth. If the GJK simplex is degenerate (coplanar
    // or collinear — common when shapes overlap along a single axis), rebuild
    // a proper non-degenerate simplex from tetrahedral directions.
    EPAResult epa;
    if (simplex.count >= 4 &&
        apc::math::abs(detail::simplex_volume(simplex.points)) > APC_EPSILON)
    {
        epa = EPA::query(simplex.points, simplex.count, hull_a, hull_b);
    } else {
        Vec3 rebuilt[4];
        detail::rebuild_simplex(hull_a, hull_b, rebuilt);
        epa = EPA::query(rebuilt, 4, hull_a, hull_b);
    }

    if (!epa.success) {
        return false;
    }

    // EPA normal is A->B (sphere->convex). We need B->A (convex->sphere).
    out_contact.normal      = Vec3::scale(epa.normal, -1.0f);
    out_contact.penetration = epa.penetration;
    out_contact.point_on_a  = epa.point_on_a;
    out_contact.point_on_b  = epa.point_on_b;

    return true;
}

// ---------------------------------------------------------------------------
// detect_convex_convex() — GJK + EPA for two convex shapes (box, convex piece).
//
// Args: shape_a and shape_b must be Box or ConvexPiece.
//
// Returns: ContactPoint with normal B->A (from shape_b toward shape_a).
// ---------------------------------------------------------------------------
inline bool detect_convex_convex(
    const CollisionShape& shape_a,
    const CollisionShape& shape_b,
    ContactPoint& out_contact)
{
    if ((shape_a.type != ShapeType::Box && shape_a.type != ShapeType::ConvexPiece) ||
        (shape_b.type != ShapeType::Box && shape_b.type != ShapeType::ConvexPiece))
    {
        return false;
    }

    // Build hulls — buffers on stack, valid for this scope
    BoxShapeData box_a_buf;
    ConvexPieceShapeData convex_a_buf;
    ConvexHull hull_a = detail::make_hull_from_shape(shape_a, box_a_buf, convex_a_buf);

    BoxShapeData box_b_buf;
    ConvexPieceShapeData convex_b_buf;
    ConvexHull hull_b = detail::make_hull_from_shape(shape_b, box_b_buf, convex_b_buf);

    // GJK boolean + simplex
    GJKSimplex simplex;
    GJKResult gjk = GJKBoolean::query_with_simplex(hull_a, hull_b, simplex);

    if (!gjk.intersecting) {
        return false;
    }

    // EPA for penetration depth. If the GJK simplex is degenerate (coplanar
    // or collinear — common when shapes overlap along a single axis), rebuild
    // a proper non-degenerate simplex from tetrahedral directions.
    EPAResult epa;
    if (simplex.count >= 4 &&
        apc::math::abs(detail::simplex_volume(simplex.points)) > APC_EPSILON)
    {
        epa = EPA::query(simplex.points, simplex.count, hull_a, hull_b);
    } else {
        Vec3 rebuilt[4];
        detail::rebuild_simplex(hull_a, hull_b, rebuilt);
        epa = EPA::query(rebuilt, 4, hull_a, hull_b);
    }

    if (!epa.success) {
        return false;
    }

    // EPA normal is A->B (shape_a->shape_b). We need B->A (shape_b->shape_a).
    out_contact.normal      = Vec3::scale(epa.normal, -1.0f);
    out_contact.penetration = epa.penetration;
    out_contact.point_on_a  = epa.point_on_a;
    out_contact.point_on_b  = epa.point_on_b;

    return true;
}

// ---------------------------------------------------------------------------
// detect_plane_box() — plane vs oriented box collision detection.
// Normal convention: B->A (plane->box, i.e., plane normal direction).
//
// Algorithm: Transform all 8 box corners to world space, compute signed
// distance of each to the plane. The most deeply penetrating corner becomes
// the contact point. If no corner is below the plane, no collision.
// ---------------------------------------------------------------------------
inline bool detect_plane_box(
    const Vec3& plane_point,
    const Vec3& plane_normal,
    const Vec3& box_pos,
    const Vec3& box_half_extents,
    const Mat3& box_rot,
    ContactPoint& out_contact)
{
    // All 8 box corners in local space
    const float hx = box_half_extents.x;
    const float hy = box_half_extents.y;
    const float hz = box_half_extents.z;
    Vec3 corners[8] = {
        Vec3(-hx, -hy, -hz), Vec3( hx, -hy, -hz),
        Vec3(-hx,  hy, -hz), Vec3( hx,  hy, -hz),
        Vec3(-hx, -hy,  hz), Vec3( hx, -hy,  hz),
        Vec3(-hx,  hy,  hz), Vec3( hx,  hy,  hz)
    };

    float min_dist = 1e30f;
    uint32_t min_idx = 0u;

    // Find the corner with the minimum (most negative) signed distance
    for (uint32_t i = 0u; i < 8u; ++i) {
        Vec3 world_corner = Vec3::add(box_pos, box_rot.transform_vec(corners[i]));
        float d = Vec3::dot(Vec3::sub(world_corner, plane_point), plane_normal);
        if (d < min_dist) {
            min_dist = d;
            min_idx = i;
        }
    }

    // min_dist > 0 means all corners are on the outside — no collision
    if (min_dist > 0.0f) {
        return false;
    }

    // Contact: the deepest penetrating corner
    Vec3 deepest_corner = Vec3::add(box_pos, box_rot.transform_vec(corners[min_idx]));

    out_contact.normal = plane_normal;  // B->A (plane->box)
    out_contact.penetration = -min_dist; // positive penetration depth
    out_contact.point_on_a = deepest_corner;               // box surface (deepest corner)
    out_contact.point_on_b = Vec3::sub(deepest_corner,      // plane surface projection
        Vec3::scale(plane_normal, min_dist));

    return true;
}

// ---------------------------------------------------------------------------
// detect_plane_convex() — plane vs convex piece collision detection.
// Normal convention: B->A (plane->convex).
//
// Algorithm: Transform all convex vertices to world space, compute signed
// distance of each to the plane. Find the most deeply penetrating vertex.
// ---------------------------------------------------------------------------
inline bool detect_plane_convex(
    const Vec3& plane_point,
    const Vec3& plane_normal,
    const Vec3& convex_pos,
    const Mat3& convex_rot,
    const Vec3* convex_vertices,
    uint32_t convex_vertex_count,
    ContactPoint& out_contact)
{
    float min_dist = 1e30f;
    uint32_t min_idx = 0u;

    for (uint32_t i = 0u; i < convex_vertex_count; ++i) {
        Vec3 world_v = Vec3::add(convex_pos, convex_rot.transform_vec(convex_vertices[i]));
        float d = Vec3::dot(Vec3::sub(world_v, plane_point), plane_normal);
        if (d < min_dist) {
            min_dist = d;
            min_idx = i;
        }
    }

    if (min_dist > 0.0f) {
        return false;
    }

    Vec3 deepest_v = Vec3::add(convex_pos, convex_rot.transform_vec(convex_vertices[min_idx]));

    out_contact.normal = plane_normal;
    out_contact.penetration = -min_dist;
    out_contact.point_on_a = deepest_v;
    out_contact.point_on_b = Vec3::sub(deepest_v, Vec3::scale(plane_normal, min_dist));

    return true;
}

// ---------------------------------------------------------------------------
// dispatch_detect() — Central collision dispatcher.
//
// Takes two CollisionShapes and body IDs, routes to the appropriate detector,
// and populates a ContactManifold.
//
// Normal convention: all contacts have normal pointing B->A
//   (from body B toward body A, i.e., pushing A away from B).
//
// Returns true if any contact was generated.
// ---------------------------------------------------------------------------
inline bool dispatch_detect(
    const CollisionShape& shape_a,
    const CollisionShape& shape_b,
    uint32_t id_a,
    uint32_t id_b,
    ContactManifold& out_manifold)
{
    out_manifold.reset();
    out_manifold.id_a = id_a;
    out_manifold.id_b = id_b;

    ContactPoint contact;
    bool hit = false;

    const ShapeType ta = shape_a.type;
    const ShapeType tb = shape_b.type;

    // =======================================================================
    // SPHERE – SPHERE
    // =======================================================================
    if (ta == ShapeType::Sphere && tb == ShapeType::Sphere) {
        SphereCollider ca;
        ca.radius = shape_a.sphere_radius;
        SphereCollider cb;
        cb.radius = shape_b.sphere_radius;
        hit = detect_sphere_sphere(shape_a.position, ca,
                                  shape_b.position, cb, contact);
    }
    // =======================================================================
    // SPHERE – BOX  (A=sphere, B=box → normal B->A = box->sphere)
    // =======================================================================
    else if (ta == ShapeType::Sphere && tb == ShapeType::Box) {
        hit = detect_sphere_box(
            shape_a.position, shape_a.sphere_radius,
            shape_b.position, shape_b.box_half_extents,
            shape_b.rotation, contact);
    }
    // =======================================================================
    // SPHERE – CONVEX PIECE  (A=sphere, B=convex)
    // =======================================================================
    else if (ta == ShapeType::Sphere && tb == ShapeType::ConvexPiece) {
        hit = detect_sphere_convex(shape_a, shape_b, contact);
    }
    // =======================================================================
    // SPHERE – PLANE  (A=sphere, B=plane → normal B->A = plane->sphere)
    // =======================================================================
    else if (ta == ShapeType::Sphere && tb == ShapeType::Plane) {
        SphereCollider col;
        col.radius = shape_a.sphere_radius;
        PlaneCollider plane;
        plane.point  = shape_b.position;
        plane.normal = shape_b.plane_normal;
        hit = detect_sphere_plane(shape_a.position, col, plane, contact);
    }
    // =======================================================================
    // SPHERE – CAPSULE  (A=sphere, B=capsule → normal B->A = capsule->sphere)
    // =======================================================================
    else if (ta == ShapeType::Sphere && tb == ShapeType::Capsule) {
        CapsuleShapeData cap_buf;
        CapsuleShapeData& cap = cap_buf;
        cap.radius = shape_b.capsule_radius;
        cap.half_height = shape_b.capsule_half_height;
        cap.position = shape_b.position;
        cap.orientation = shape_b.orientation;
        cap.rotation = shape_b.rotation;
        cap.update_cache();
        hit = detect_sphere_capsule(shape_a.position, shape_a.sphere_radius,
                                     cap, contact);
    }
    // =======================================================================
    // SPHERE – CYLINDER  (A=sphere, B=cylinder → normal B->A = cylinder->sphere)
    // =======================================================================
    else if (ta == ShapeType::Sphere && tb == ShapeType::Cylinder) {
        CylinderShapeData cyl_buf;
        CylinderShapeData& cyl = cyl_buf;
        cyl.radius = shape_b.cylinder_radius;
        cyl.half_height = shape_b.cylinder_half_height;
        cyl.position = shape_b.position;
        cyl.orientation = shape_b.orientation;
        cyl.rotation = shape_b.rotation;
        cyl.update_cache();
        hit = detect_sphere_cylinder(shape_a.position, shape_a.sphere_radius,
                                      cyl, contact);
    }
    // =======================================================================
    // BOX – SPHERE  (A=box, B=sphere → need B->A = sphere->box → negate)
    // =======================================================================
    else if (ta == ShapeType::Box && tb == ShapeType::Sphere) {
        hit = detect_sphere_box(
            shape_b.position, shape_b.sphere_radius,
            shape_a.position, shape_a.box_half_extents,
            shape_a.rotation, contact);
        if (hit) { swap_contact(contact); }
    }
    // =======================================================================
    // BOX – BOX  (GJK+EPA)
    // =======================================================================
    else if (ta == ShapeType::Box && tb == ShapeType::Box) {
        hit = detect_convex_convex(shape_a, shape_b, contact);
    }
    // =======================================================================
    // BOX – CONVEX PIECE  (GJK+EPA)
    // =======================================================================
    else if (ta == ShapeType::Box && tb == ShapeType::ConvexPiece) {
        hit = detect_convex_convex(shape_a, shape_b, contact);
    }
    // =======================================================================
    // BOX – PLANE  (A=box, B=plane → need B->A = plane->box)
    // =======================================================================
    else if (ta == ShapeType::Box && tb == ShapeType::Plane) {
        hit = detect_plane_box(
            shape_b.position, shape_b.plane_normal,
            shape_a.position, shape_a.box_half_extents,
            shape_a.rotation, contact);
    }
    // =======================================================================
    // BOX – CAPSULE  (A=box, B=capsule → GJK+EPA, negate)
    // =======================================================================
    else if (ta == ShapeType::Box && tb == ShapeType::Capsule) {
        CapsuleShapeData cap_buf;
        ConvexHull hull_a = detail::make_capsule_hull(shape_b, cap_buf);
        BoxShapeData box_buf;
        ConvexPieceShapeData convex_buf;
        ConvexHull hull_b = detail::make_hull_from_shape(shape_a, box_buf, convex_buf);
        hit = detail::gjk_epa_generic(hull_a, hull_b, contact);
        if (hit) { swap_contact(contact); }
    }
    // =======================================================================
    // BOX – CYLINDER  (A=box, B=cylinder → GJK+EPA, negate)
    // =======================================================================
    else if (ta == ShapeType::Box && tb == ShapeType::Cylinder) {
        CylinderShapeData cyl_buf;
        ConvexHull hull_a = detail::make_cylinder_hull(shape_b, cyl_buf);
        BoxShapeData box_buf;
        ConvexPieceShapeData convex_buf;
        ConvexHull hull_b = detail::make_hull_from_shape(shape_a, box_buf, convex_buf);
        hit = detail::gjk_epa_generic(hull_a, hull_b, contact);
        if (hit) { swap_contact(contact); }
    }
    // =======================================================================
    // CONVEX PIECE – SPHERE  (A=convex, B=sphere → need B->A = sphere->convex)
    // =======================================================================
    else if (ta == ShapeType::ConvexPiece && tb == ShapeType::Sphere) {
        hit = detect_sphere_convex(shape_b, shape_a, contact);
        if (hit) { swap_contact(contact); }
    }
    // =======================================================================
    // CONVEX PIECE – BOX  (GJK+EPA)
    // =======================================================================
    else if (ta == ShapeType::ConvexPiece && tb == ShapeType::Box) {
        hit = detect_convex_convex(shape_a, shape_b, contact);
    }
    // =======================================================================
    // CONVEX PIECE – CONVEX PIECE  (GJK+EPA)
    // =======================================================================
    else if (ta == ShapeType::ConvexPiece && tb == ShapeType::ConvexPiece) {
        hit = detect_convex_convex(shape_a, shape_b, contact);
    }
    // =======================================================================
    // CONVEX PIECE – PLANE  (A=convex, B=plane → need B->A = plane->convex)
    // =======================================================================
    else if (ta == ShapeType::ConvexPiece && tb == ShapeType::Plane) {
        hit = detect_plane_convex(
            shape_b.position, shape_b.plane_normal,
            shape_a.position, shape_a.rotation,
            shape_a.convex_vertices, shape_a.convex_vertex_count, contact);
    }
    // =======================================================================
    // CONVEX PIECE – CAPSULE  (A=convex, B=capsule → GJK+EPA, negate)
    // =======================================================================
    else if (ta == ShapeType::ConvexPiece && tb == ShapeType::Capsule) {
        CapsuleShapeData cap_buf;
        ConvexHull hull_a = detail::make_capsule_hull(shape_b, cap_buf);
        BoxShapeData box_buf;
        ConvexPieceShapeData convex_buf;
        ConvexHull hull_b = detail::make_hull_from_shape(shape_a, box_buf, convex_buf);
        hit = detail::gjk_epa_generic(hull_a, hull_b, contact);
        if (hit) { swap_contact(contact); }
    }
    // =======================================================================
    // CONVEX PIECE – CYLINDER  (A=convex, B=cylinder → GJK+EPA, negate)
    // =======================================================================
    else if (ta == ShapeType::ConvexPiece && tb == ShapeType::Cylinder) {
        CylinderShapeData cyl_buf;
        ConvexHull hull_a = detail::make_cylinder_hull(shape_b, cyl_buf);
        BoxShapeData box_buf;
        ConvexPieceShapeData convex_buf;
        ConvexHull hull_b = detail::make_hull_from_shape(shape_a, box_buf, convex_buf);
        hit = detail::gjk_epa_generic(hull_a, hull_b, contact);
        if (hit) { swap_contact(contact); }
    }
    // =======================================================================
    // PLANE – SPHERE  (A=plane, B=sphere → need B->A = sphere->plane)
    // =======================================================================
    else if (ta == ShapeType::Plane && tb == ShapeType::Sphere) {
        SphereCollider col;
        col.radius = shape_b.sphere_radius;
        PlaneCollider plane;
        plane.point  = shape_a.position;
        plane.normal = shape_a.plane_normal;
        hit = detect_sphere_plane(shape_b.position, col, plane, contact);
        if (hit) { swap_contact(contact); }
    }
    // =======================================================================
    // PLANE – BOX  (A=plane, B=box → need B->A = box->plane → negate)
    // =======================================================================
    else if (ta == ShapeType::Plane && tb == ShapeType::Box) {
        hit = detect_plane_box(
            shape_a.position, shape_a.plane_normal,
            shape_b.position, shape_b.box_half_extents,
            shape_b.rotation, contact);
        if (hit) { swap_contact(contact); }
    }
    // =======================================================================
    // PLANE – CONVEX PIECE  (A=plane, B=convex → need B->A = convex->plane → negate)
    // =======================================================================
    else if (ta == ShapeType::Plane && tb == ShapeType::ConvexPiece) {
        hit = detect_plane_convex(
            shape_a.position, shape_a.plane_normal,
            shape_b.position, shape_b.rotation,
            shape_b.convex_vertices, shape_b.convex_vertex_count, contact);
        if (hit) { swap_contact(contact); }
    }
    // =======================================================================
    // CAPSULE – SPHERE  (A=capsule, B=sphere → need B->A = sphere->capsule → negate)
    // =======================================================================
    else if (ta == ShapeType::Capsule && tb == ShapeType::Sphere) {
        CapsuleShapeData cap_buf;
        CapsuleShapeData& cap = cap_buf;
        cap.radius = shape_a.capsule_radius;
        cap.half_height = shape_a.capsule_half_height;
        cap.position = shape_a.position;
        cap.orientation = shape_a.orientation;
        cap.rotation = shape_a.rotation;
        cap.update_cache();
        hit = detect_sphere_capsule(shape_b.position, shape_b.sphere_radius,
                                     cap, contact);
        if (hit) { swap_contact(contact); }
    }
    // =======================================================================
    // CAPSULE – CAPSULE  (detect_capsule_capsule returns B->A ✓)
    // =======================================================================
    else if (ta == ShapeType::Capsule && tb == ShapeType::Capsule) {
        CapsuleShapeData cap_a_buf;
        cap_a_buf.radius = shape_a.capsule_radius;
        cap_a_buf.half_height = shape_a.capsule_half_height;
        cap_a_buf.position = shape_a.position;
        cap_a_buf.orientation = shape_a.orientation;
        cap_a_buf.rotation = shape_a.rotation;
        cap_a_buf.update_cache();

        CapsuleShapeData cap_b_buf;
        cap_b_buf.radius = shape_b.capsule_radius;
        cap_b_buf.half_height = shape_b.capsule_half_height;
        cap_b_buf.position = shape_b.position;
        cap_b_buf.orientation = shape_b.orientation;
        cap_b_buf.rotation = shape_b.rotation;
        cap_b_buf.update_cache();

        hit = detect_capsule_capsule(cap_a_buf, cap_b_buf, contact);
    }
    // =======================================================================
    // CAPSULE – BOX  (A=capsule, B=box → GJK+EPA)
    // =======================================================================
    else if (ta == ShapeType::Capsule && tb == ShapeType::Box) {
        CapsuleShapeData cap_buf;
        ConvexHull hull_a = detail::make_capsule_hull(shape_a, cap_buf);
        BoxShapeData box_buf;
        ConvexPieceShapeData convex_buf;
        ConvexHull hull_b = detail::make_hull_from_shape(shape_b, box_buf, convex_buf);
        hit = detail::gjk_epa_generic(hull_a, hull_b, contact);
    }
    // =======================================================================
    // CAPSULE – CONVEX PIECE  (GJK+EPA)
    // =======================================================================
    else if (ta == ShapeType::Capsule && tb == ShapeType::ConvexPiece) {
        CapsuleShapeData cap_buf;
        ConvexHull hull_a = detail::make_capsule_hull(shape_a, cap_buf);
        BoxShapeData box_buf;
        ConvexPieceShapeData convex_buf;
        ConvexHull hull_b = detail::make_hull_from_shape(shape_b, box_buf, convex_buf);
        hit = detail::gjk_epa_generic(hull_a, hull_b, contact);
    }
    // =======================================================================
    // CAPSULE – PLANE  (A=capsule, B=plane → normal B->A = plane->capsule)
    // =======================================================================
    else if (ta == ShapeType::Capsule && tb == ShapeType::Plane) {
        CapsuleShapeData cap_buf;
        cap_buf.radius = shape_a.capsule_radius;
        cap_buf.half_height = shape_a.capsule_half_height;
        cap_buf.position = shape_a.position;
        cap_buf.orientation = shape_a.orientation;
        cap_buf.rotation = shape_a.rotation;
        cap_buf.update_cache();
        hit = detect_capsule_plane(cap_buf, shape_b.position, shape_b.plane_normal, contact);
    }
    // =======================================================================
    // CAPSULE – CYLINDER  (GJK+EPA)
    // =======================================================================
    else if (ta == ShapeType::Capsule && tb == ShapeType::Cylinder) {
        CapsuleShapeData cap_buf;
        ConvexHull hull_a = detail::make_capsule_hull(shape_a, cap_buf);
        CylinderShapeData cyl_buf;
        ConvexHull hull_b = detail::make_cylinder_hull(shape_b, cyl_buf);
        hit = detail::gjk_epa_generic(hull_a, hull_b, contact);
    }
    // =======================================================================
    // CYLINDER – SPHERE  (A=cylinder, B=sphere → negate)
    // =======================================================================
    else if (ta == ShapeType::Cylinder && tb == ShapeType::Sphere) {
        CylinderShapeData cyl_buf;
        cyl_buf.radius = shape_a.cylinder_radius;
        cyl_buf.half_height = shape_a.cylinder_half_height;
        cyl_buf.position = shape_a.position;
        cyl_buf.orientation = shape_a.orientation;
        cyl_buf.rotation = shape_a.rotation;
        cyl_buf.update_cache();
        hit = detect_sphere_cylinder(shape_b.position, shape_b.sphere_radius,
                                      cyl_buf, contact);
        if (hit) { swap_contact(contact); }
    }
    // =======================================================================
    // CYLINDER – BOX  (A=cylinder, B=box → GJK+EPA)
    // =======================================================================
    else if (ta == ShapeType::Cylinder && tb == ShapeType::Box) {
        CylinderShapeData cyl_buf;
        ConvexHull hull_a = detail::make_cylinder_hull(shape_a, cyl_buf);
        BoxShapeData box_buf;
        ConvexPieceShapeData convex_buf;
        ConvexHull hull_b = detail::make_hull_from_shape(shape_b, box_buf, convex_buf);
        hit = detail::gjk_epa_generic(hull_a, hull_b, contact);
    }
    // =======================================================================
    // CYLINDER – CONVEX PIECE  (GJK+EPA)
    // =======================================================================
    else if (ta == ShapeType::Cylinder && tb == ShapeType::ConvexPiece) {
        CylinderShapeData cyl_buf;
        ConvexHull hull_a = detail::make_cylinder_hull(shape_a, cyl_buf);
        BoxShapeData box_buf;
        ConvexPieceShapeData convex_buf;
        ConvexHull hull_b = detail::make_hull_from_shape(shape_b, box_buf, convex_buf);
        hit = detail::gjk_epa_generic(hull_a, hull_b, contact);
    }
    // =======================================================================
    // CYLINDER – PLANE  (GJK+EPA)
    // =======================================================================
    else if (ta == ShapeType::Cylinder && tb == ShapeType::Plane) {
        CylinderShapeData cyl_buf;
        ConvexHull hull_a = detail::make_cylinder_hull(shape_a, cyl_buf);
        PlaneShapeData plane_data;
        plane_data.normal = shape_b.plane_normal;
        plane_data.point = shape_b.position;
        plane_data.extent = 1000.0f;
        ConvexHull hull_b;
        hull_b.user_data = &plane_data;
        hull_b.support = plane_support;
        hit = detail::gjk_epa_generic(hull_a, hull_b, contact);
    }
    // =======================================================================
    // CYLINDER – CAPSULE  (GJK+EPA)
    // =======================================================================
    else if (ta == ShapeType::Cylinder && tb == ShapeType::Capsule) {
        CylinderShapeData cyl_buf;
        ConvexHull hull_a = detail::make_cylinder_hull(shape_a, cyl_buf);
        CapsuleShapeData cap_buf;
        ConvexHull hull_b = detail::make_capsule_hull(shape_b, cap_buf);
        hit = detail::gjk_epa_generic(hull_a, hull_b, contact);
    }
    // =======================================================================
    // CYLINDER – CYLINDER  (GJK+EPA)
    // =======================================================================
    else if (ta == ShapeType::Cylinder && tb == ShapeType::Cylinder) {
        CylinderShapeData cyl_a_buf;
        ConvexHull hull_a = detail::make_cylinder_hull(shape_a, cyl_a_buf);
        CylinderShapeData cyl_b_buf;
        ConvexHull hull_b = detail::make_cylinder_hull(shape_b, cyl_b_buf);
        hit = detail::gjk_epa_generic(hull_a, hull_b, contact);
    }
    // =======================================================================
    // PLANE – CAPSULE  (A=plane, B=capsule → negate)
    // =======================================================================
    else if (ta == ShapeType::Plane && tb == ShapeType::Capsule) {
        CapsuleShapeData cap_buf;
        cap_buf.radius = shape_b.capsule_radius;
        cap_buf.half_height = shape_b.capsule_half_height;
        cap_buf.position = shape_b.position;
        cap_buf.orientation = shape_b.orientation;
        cap_buf.rotation = shape_b.rotation;
        cap_buf.update_cache();
        hit = detect_capsule_plane(cap_buf, shape_a.position, shape_a.plane_normal, contact);
        if (hit) { swap_contact(contact); }
    }
    // =======================================================================
    // PLANE – CYLINDER  (A=plane, B=cylinder → GJK+EPA, negate)
    // =======================================================================
    else if (ta == ShapeType::Plane && tb == ShapeType::Cylinder) {
        CylinderShapeData cyl_buf;
        ConvexHull hull_a = detail::make_cylinder_hull(shape_b, cyl_buf);
        PlaneShapeData plane_data;
        plane_data.normal = shape_a.plane_normal;
        plane_data.point = shape_a.position;
        plane_data.extent = 1000.0f;
        ConvexHull hull_b;
        hull_b.user_data = &plane_data;
        hull_b.support = plane_support;
        hit = detail::gjk_epa_generic(hull_a, hull_b, contact);
        if (hit) { swap_contact(contact); }
    }
    // =======================================================================
    // Unknown pair — not implemented
    // =======================================================================
    else {
        return false;
    }

    if (hit) {
        out_manifold.add_contact(contact);
    }
    return hit;
}

} // namespace apc
