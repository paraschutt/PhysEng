#pragma once
// =============================================================================
// Capsule Collision Primitive for APC Deterministic Physics Engine
// =============================================================================
//
// Provides:
//   - CapsuleShapeData: capsule description (radius, half-height, position,
//     orientation, cached rotation matrix)
//   - capsule_support(): GJK-compatible support function matching
//     ConvexHull::SupportFunc signature
//   - detect_sphere_capsule(): dedicated fast-path sphere vs capsule
//   - detect_capsule_capsule(): dedicated fast-path capsule vs capsule
//   - detect_capsule_plane(): capsule vs infinite half-space
//   - capsule_get_aabb(): AABB for broadphase via 6-axis support evaluation
//
// Normal convention: B->A for ALL detect_ functions (normal points from B
// toward A, i.e., pushing A away from B).
//
// Determinism: no FMA, index-order tie-breaking, consistent FP ordering.
//
// =============================================================================

#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include "apc_math/apc_mat3.h"
#include "apc_math/apc_math_common.h"
#include "apc_sphere_sphere.h"   // ContactPoint, SphereCollider
#include "apc_broadphase.h"     // AABB
#include <cstdint>
#include <cmath>

namespace apc {

// ---------------------------------------------------------------------------
// CapsuleShapeData -- capsule collider description
//
// Local-space convention: the capsule's cylindrical axis is along local Y,
// extending from -half_height to +half_height. Hemispherical caps of the
// given radius sit on each end of the cylinder.
//
// Total geometric height = 2 * half_height + 2 * radius
// ---------------------------------------------------------------------------
struct CapsuleShapeData {
    float radius;          // Radius of hemispherical caps and cylindrical body
    float half_height;     // Half the distance between the two sphere centers
    Vec3  position;        // World position (center of the capsule)
    Quat  orientation;     // World orientation
    Mat3  rotation;        // Cached rotation matrix (set via update_cache)

    APC_FORCEINLINE void update_cache() {
        rotation = Mat3::from_quat(orientation);
    }
};

// ---------------------------------------------------------------------------
// capsule_support() -- GJK-compatible support function
//
// Matches ConvexHull::SupportFunc signature:
//   Vec3 (*)(const void* user_data, const Vec3& dir, uint32_t& out_vertex_id)
//
// user_data must point to a CapsuleShapeData (update_cache must have been
// called).
//
// Algorithm (Minkowski sum of line segment + sphere):
//   1. Transform direction into local space (transpose of rotation)
//   2. Pick the axis endpoint toward which the direction points (Y axis)
//   3. Normalize direction, multiply by radius for the sphere contribution
//   4. Sum axis endpoint + sphere offset, transform back to world space
//
// Vertex ID encoding (deterministic tie-breaking):
//   0 = top cap (+Y end)
//   1 = bottom cap (-Y end)
//   2 = cylinder body (tangential direction dominates)
// ---------------------------------------------------------------------------
inline Vec3 capsule_support(const void* user_data, const Vec3& dir,
                            uint32_t& out_vertex_id)
{
    const CapsuleShapeData* data =
        static_cast<const CapsuleShapeData*>(user_data);

    // 1. Transform direction into local space (inverse rotation = transpose)
    Mat3 inv_rot = data->rotation.transpose();
    Vec3 local_dir = inv_rot.transform_vec(dir);

    // Handle degenerate zero direction
    float dir_len_sq = Vec3::length_sq(local_dir);
    if (dir_len_sq < APC_EPSILON_SQ) {
        // Fallback: point on top cap along +X (deterministic)
        Vec3 local_pt(data->radius, data->half_height, 0.0f);
        out_vertex_id = 0u;
        return Vec3::add(data->position, data->rotation.transform_vec(local_pt));
    }

    // 2. Normalize the direction
    float inv_dir_len = 1.0f / std::sqrt(dir_len_sq);
    Vec3 norm_dir = Vec3::scale(local_dir, inv_dir_len);

    // 3. Axis endpoint contribution (line segment support along Y)
    float y_contrib;
    uint32_t cap_id;
    if (local_dir.y >= 0.0f) {
        y_contrib = data->half_height;
        cap_id = 0u; // top cap
    } else {
        y_contrib = -data->half_height;
        cap_id = 1u; // bottom cap
    }

    // 4. Sphere contribution (normalized direction * radius)
    float sx = norm_dir.x * data->radius;
    float sy = norm_dir.y * data->radius;
    float sz = norm_dir.z * data->radius;

    // 5. Combine: Minkowski sum = line segment support + sphere support
    Vec3 local_point(sx, y_contrib + sy, sz);

    // 6. Vertex ID: determine if the support point is on the cap or cylinder.
    //    When the tangential component dominates the Y component, the point
    //    lies on or near the cylinder wall (equatorial region of a cap).
    float abs_ny = apc::math::abs(norm_dir.y);
    float tangent_len_sq = norm_dir.x * norm_dir.x + norm_dir.z * norm_dir.z;
    if (tangent_len_sq > APC_EPSILON_SQ && abs_ny * abs_ny < tangent_len_sq) {
        out_vertex_id = 2u; // cylinder body
    } else {
        out_vertex_id = cap_id; // hemispherical cap
    }

    // 7. Transform local support point to world space
    return Vec3::add(data->position, data->rotation.transform_vec(local_point));
}

// ---------------------------------------------------------------------------
// detect_sphere_capsule() -- dedicated fast-path for sphere vs capsule
//
// Args:
//   sphere_pos, sphere_radius: sphere description (body A)
//   capsule: capsule shape data (body B)
//   out_contact: populated on collision
//
// Normal convention: B->A (capsule -> sphere)
// Contact-point convention: surface points closest to each other at contact.
// ---------------------------------------------------------------------------
inline bool detect_sphere_capsule(
    const Vec3& sphere_pos,
    float sphere_radius,
    const CapsuleShapeData& capsule,
    ContactPoint& out_contact)
{
    // 1. Transform sphere center into capsule local space
    Vec3 diff = Vec3::sub(sphere_pos, capsule.position);
    Mat3 inv_rot = capsule.rotation.transpose();
    Vec3 local_sphere = inv_rot.transform_vec(diff);

    // 2. Find closest point on the capsule's Y-axis segment to the sphere center.
    //    Project the local Y component and clamp to [-half_height, +half_height].
    float hh = capsule.half_height;
    float clamped_y = local_sphere.y;
    if (clamped_y < -hh) {
        clamped_y = -hh;
    } else if (clamped_y > hh) {
        clamped_y = hh;
    }

    // 3. Closest point on axis in world space
    Vec3 local_axis_point(0.0f, clamped_y, 0.0f);
    Vec3 closest_on_axis = Vec3::add(
        capsule.position, capsule.rotation.transform_vec(local_axis_point));

    // 4. Distance from sphere center to closest point on axis
    Vec3 sep = Vec3::sub(sphere_pos, closest_on_axis);
    float dist_sq = Vec3::length_sq(sep);
    float radius_sum = sphere_radius + capsule.radius;

    // 5. Early-out if separated
    float rs_sq = radius_sum * radius_sum;
    if (dist_sq > rs_sq) {
        return false;
    }

    float dist = std::sqrt(dist_sq);

    if (dist < APC_EPSILON) {
        // Degenerate: sphere center exactly on capsule axis.
        // Use capsule's local up direction as fallback normal.
        Vec3 capsule_up = capsule.rotation.transform_vec(Vec3(0.0f, 1.0f, 0.0f));
        out_contact.normal = capsule_up;
        out_contact.penetration = radius_sum;
    } else {
        float inv_dist = 1.0f / dist;
        // Normal: B->A (capsule -> sphere)
        out_contact.normal = Vec3::scale(sep, inv_dist);
        out_contact.penetration = radius_sum - dist;
    }

    // Contact points: surface points at the contact location.
    // point_on_a = sphere surface toward capsule
    out_contact.point_on_a = Vec3::sub(
        sphere_pos, Vec3::scale(out_contact.normal, sphere_radius));
    // point_on_b = capsule surface toward sphere
    out_contact.point_on_b = Vec3::add(
        closest_on_axis, Vec3::scale(out_contact.normal, capsule.radius));

    return true;
}

// ---------------------------------------------------------------------------
// Detail namespace -- internal helpers (not part of public API)
// ---------------------------------------------------------------------------
namespace detail {

// ---------------------------------------------------------------------------
// closest_points_segments() -- minimum-distance points between two segments
//
// Given segment 1 from p1 to q1 and segment 2 from p2 to q2, finds
// parameters s,t in [0,1] such that:
//   |p1 + s*(q1-p1) - p2 - t*(q2-p2)|
// is minimized.
//
// Algorithm: Ericson, "Real-Time Collision Detection", pp. 149-151.
// Determinism: when segments are parallel, s defaults to 0.0f (start of
// segment 1). All clamping uses strict ordered comparisons.
// ---------------------------------------------------------------------------
inline void closest_points_segments(
    const Vec3& p1, const Vec3& q1,
    const Vec3& p2, const Vec3& q2,
    float& s_out, float& t_out)
{
    Vec3 d1 = Vec3::sub(q1, p1); // direction of segment 1
    Vec3 d2 = Vec3::sub(q2, p2); // direction of segment 2
    Vec3 r  = Vec3::sub(p1, p2); // start-to-start vector

    float a = Vec3::dot(d1, d1); // squared length of segment 1
    float e = Vec3::dot(d2, d2); // squared length of segment 2
    float f = Vec3::dot(d2, r);

    // Both segments degenerate to points
    if (a <= APC_EPSILON && e <= APC_EPSILON) {
        s_out = 0.0f;
        t_out = 0.0f;
        return;
    }

    // First segment degenerates to a point
    if (a <= APC_EPSILON) {
        s_out = 0.0f;
        t_out = (e > APC_EPSILON) ? (f / e) : 0.0f;
        if (t_out < 0.0f) t_out = 0.0f;
        else if (t_out > 1.0f) t_out = 1.0f;
        return;
    }

    float c = Vec3::dot(d1, r);

    // Second segment degenerates to a point
    if (e <= APC_EPSILON) {
        t_out = 0.0f;
        s_out = -c / a;
        if (s_out < 0.0f) s_out = 0.0f;
        else if (s_out > 1.0f) s_out = 1.0f;
        return;
    }

    float b = Vec3::dot(d1, d2);
    float denom = a * e - b * b;

    // General case: segments not parallel
    if (apc::math::abs(denom) > APC_EPSILON) {
        s_out = (b * f - c * e) / denom;
        if (s_out < 0.0f) s_out = 0.0f;
        else if (s_out > 1.0f) s_out = 1.0f;
    } else {
        // Parallel segments: DETERMINISM -- pick start of segment 1
        s_out = 0.0f;
    }

    // Compute optimal t for the (possibly clamped) s
    t_out = (b * s_out + f) / e;

    // Clamp t and recompute s if t was pushed out of [0,1]
    if (t_out < 0.0f) {
        t_out = 0.0f;
        s_out = -c / a;
        if (s_out < 0.0f) s_out = 0.0f;
        else if (s_out > 1.0f) s_out = 1.0f;
    } else if (t_out > 1.0f) {
        t_out = 1.0f;
        s_out = (b - c) / a;
        if (s_out < 0.0f) s_out = 0.0f;
        else if (s_out > 1.0f) s_out = 1.0f;
    }
}

} // namespace detail

// ---------------------------------------------------------------------------
// detect_capsule_capsule() -- dedicated fast-path for capsule vs capsule
//
// Args:
//   cap_a: first capsule (body A)
//   cap_b: second capsule (body B)
//   out_contact: populated on collision
//
// Normal convention: B->A (cap_b -> cap_a)
// Contact-point convention: surface points closest to each other at contact.
// ---------------------------------------------------------------------------
inline bool detect_capsule_capsule(
    const CapsuleShapeData& cap_a,
    const CapsuleShapeData& cap_b,
    ContactPoint& out_contact)
{
    // 1. Compute axis endpoints for each capsule in world space
    Vec3 a_top = Vec3::add(cap_a.position,
        cap_a.rotation.transform_vec(Vec3(0.0f, cap_a.half_height, 0.0f)));
    Vec3 a_bot = Vec3::add(cap_a.position,
        cap_a.rotation.transform_vec(Vec3(0.0f, -cap_a.half_height, 0.0f)));

    Vec3 b_top = Vec3::add(cap_b.position,
        cap_b.rotation.transform_vec(Vec3(0.0f, cap_b.half_height, 0.0f)));
    Vec3 b_bot = Vec3::add(cap_b.position,
        cap_b.rotation.transform_vec(Vec3(0.0f, -cap_b.half_height, 0.0f)));

    // 2. Find closest points between the two axis segments
    float s = 0.0f;
    float t = 0.0f;
    detail::closest_points_segments(a_bot, a_top, b_bot, b_top, s, t);

    Vec3 a_dir = Vec3::sub(a_top, a_bot);
    Vec3 b_dir = Vec3::sub(b_top, b_bot);
    Vec3 point_on_a_seg = Vec3::add(a_bot, Vec3::scale(a_dir, s));
    Vec3 point_on_b_seg = Vec3::add(b_bot, Vec3::scale(b_dir, t));

    // 3. Distance between closest points on the axes
    Vec3 sep = Vec3::sub(point_on_a_seg, point_on_b_seg);
    float dist_sq = Vec3::length_sq(sep);
    float radius_sum = cap_a.radius + cap_b.radius;

    // 4. Early-out if separated
    float rs_sq = radius_sum * radius_sum;
    if (dist_sq > rs_sq) {
        return false;
    }

    float dist = std::sqrt(dist_sq);

    if (dist < APC_EPSILON) {
        // Degenerate: axis segments intersect or are coincident.
        // Deterministic fallback: cross product of the two axis directions.
        Vec3 a_axis_n = Vec3::normalize(a_dir);
        Vec3 b_axis_n = Vec3::normalize(b_dir);
        Vec3 fallback = Vec3::cross(a_axis_n, b_axis_n);
        fallback = Vec3::safe_normalize(fallback, Vec3(0.0f, 1.0f, 0.0f));
        out_contact.normal = fallback;
        out_contact.penetration = radius_sum;
    } else {
        float inv_dist = 1.0f / dist;
        // Normal: B->A (cap_b -> cap_a)
        out_contact.normal = Vec3::scale(sep, inv_dist);
        out_contact.penetration = radius_sum - dist;
    }

    // Contact points: surface points at the contact location.
    // point_on_a = capsule A surface toward capsule B
    out_contact.point_on_a = Vec3::sub(
        point_on_a_seg, Vec3::scale(out_contact.normal, cap_a.radius));
    // point_on_b = capsule B surface toward capsule A
    out_contact.point_on_b = Vec3::add(
        point_on_b_seg, Vec3::scale(out_contact.normal, cap_b.radius));

    return true;
}

// ---------------------------------------------------------------------------
// detect_capsule_plane() -- capsule vs infinite half-space
//
// Args:
//   capsule: capsule shape data (body A)
//   plane_point: any point on the plane (body B)
//   plane_normal: outward normal of the plane (points away from solid side)
//   out_contact: populated on collision
//
// Normal convention: B->A (plane -> capsule)
//   The plane normal already points from the plane toward the capsule
//   (outward from the solid half-space), so out_contact.normal = plane_normal.
//
// Contact-point convention: surface points closest to each other at contact.
// ---------------------------------------------------------------------------
inline bool detect_capsule_plane(
    const CapsuleShapeData& capsule,
    const Vec3& plane_point,
    const Vec3& plane_normal,
    ContactPoint& out_contact)
{
    // 1. Compute capsule axis endpoints in world space
    Vec3 p_top = Vec3::add(capsule.position,
        capsule.rotation.transform_vec(Vec3(0.0f, capsule.half_height, 0.0f)));
    Vec3 p_bot = Vec3::add(capsule.position,
        capsule.rotation.transform_vec(Vec3(0.0f, -capsule.half_height, 0.0f)));

    // 2. Signed distance of each endpoint to the plane (positive = outside)
    float dist_top = Vec3::dot(Vec3::sub(p_top, plane_point), plane_normal);
    float dist_bot = Vec3::dot(Vec3::sub(p_bot, plane_point), plane_normal);

    // 3. The capsule surface closest to the plane has signed distance
    //    = min(dist_top, dist_bot) - radius
    float min_dist;
    Vec3 closest_endpoint;
    if (dist_top <= dist_bot) {
        min_dist = dist_top;
        closest_endpoint = p_top;
    } else {
        min_dist = dist_bot;
        closest_endpoint = p_bot;
    }

    float surface_dist = min_dist - capsule.radius;

    // 4. No collision if the capsule surface is entirely above the plane
    if (surface_dist >= 0.0f) {
        return false;
    }

    // 5. Contact data
    out_contact.normal = plane_normal;       // B->A (plane -> capsule)
    out_contact.penetration = -surface_dist; // positive penetration depth

    // 6. Contact points
    // point_on_a: capsule surface point closest to the plane
    out_contact.point_on_a = Vec3::sub(
        closest_endpoint, Vec3::scale(plane_normal, capsule.radius));

    // point_on_b: projection of closest endpoint onto the plane surface
    out_contact.point_on_b = Vec3::sub(
        closest_endpoint, Vec3::scale(plane_normal, min_dist));

    return true;
}

// ---------------------------------------------------------------------------
// capsule_get_aabb() -- approximate AABB for broadphase
//
// Evaluates the 6 axis-aligned extreme support directions and takes the
// enclosing box. Because the capsule is a convex shape, this produces the
// tightest possible AABB.
// ---------------------------------------------------------------------------
inline AABB capsule_get_aabb(const CapsuleShapeData& capsule) {
    // 6 axis-aligned directions for extreme support evaluation
    static const Vec3 axes[6] = {
        Vec3( 1.0f,  0.0f,  0.0f),
        Vec3(-1.0f,  0.0f,  0.0f),
        Vec3( 0.0f,  1.0f,  0.0f),
        Vec3( 0.0f, -1.0f,  0.0f),
        Vec3( 0.0f,  0.0f,  1.0f),
        Vec3( 0.0f,  0.0f, -1.0f)
    };

    uint32_t discard = 0u;
    Vec3 p = capsule_support(&capsule, axes[0], discard);

    AABB aabb;
    aabb.min = p;
    aabb.max = p;

    for (int i = 1; i < 6; ++i) {
        p = capsule_support(&capsule, axes[i], discard);
        if (p.x < aabb.min.x) aabb.min.x = p.x;
        if (p.y < aabb.min.y) aabb.min.y = p.y;
        if (p.z < aabb.min.z) aabb.min.z = p.z;
        if (p.x > aabb.max.x) aabb.max.x = p.x;
        if (p.y > aabb.max.y) aabb.max.y = p.y;
        if (p.z > aabb.max.z) aabb.max.z = p.z;
    }

    return aabb;
}

} // namespace apc
