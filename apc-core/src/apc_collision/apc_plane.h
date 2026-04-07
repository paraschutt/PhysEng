#pragma once
// =============================================================================
// Ground Plane (Infinite Half-Space) Collision Shape
// =============================================================================
//
// Defines a plane collider and a sphere-vs-plane detector. The plane represents
// an infinite half-space. Convention: normal points AWAY from the solid side
// (e.g., +Y for a floor). A sphere collides with the plane when it penetrates
// into the solid half-space (below the plane surface).
//
// Normal convention for detect_sphere_plane: B→A (from plane toward sphere).
//
// =============================================================================

#include "apc_math/apc_vec3.h"
#include "apc_math/apc_math_common.h"
#include "apc_sphere_sphere.h" // ContactPoint
#include <cstdint>
#include <cmath>

namespace apc {

// ---------------------------------------------------------------------------
// Infinite ground plane defined by a point on the plane and an outward normal.
// The half-space is on the opposite side of the normal (the "solid" side).
// Convention: normal points AWAY from the solid (upward for a floor).
// ---------------------------------------------------------------------------
struct PlaneCollider {
    Vec3 point;      // Any point on the plane (e.g., origin for y=0 floor)
    Vec3 normal;     // Outward normal (e.g., +Y for a floor)
};

// ---------------------------------------------------------------------------
// Detect collision between a sphere and a plane.
// Returns true if the sphere penetrates the half-space.
// Normal convention: B→A (points from plane toward sphere center).
// ---------------------------------------------------------------------------
inline bool detect_sphere_plane(
    const Vec3& pos,
    const SphereCollider& col,
    const PlaneCollider& plane,
    ContactPoint& out_contact)
{
    // Signed distance from sphere center to plane (positive = outside)
    float dist = Vec3::dot(Vec3::sub(pos, plane.point), plane.normal);

    if (dist > col.radius) {
        return false; // Sphere is fully above the plane
    }

    // Penetration depth
    float penetration = col.radius - dist;

    // Contact normal: plane's outward normal (points toward sphere = B→A for solver)
    // The plane is "B" and the sphere is "A"
    out_contact.normal = plane.normal;
    out_contact.penetration = penetration;

    // Contact points
    // Point on sphere (A): closest point on sphere surface toward the plane
    out_contact.point_on_a = Vec3::sub(pos, Vec3::scale(plane.normal, col.radius));

    // Point on plane (B): closest point on the plane to the sphere
    out_contact.point_on_b = Vec3::sub(pos, Vec3::scale(plane.normal, dist));

    return true;
}

// ---------------------------------------------------------------------------
// Support function data for plane (treat as a box with very large extents).
// This allows planes to participate in GJK if needed.
// ---------------------------------------------------------------------------
struct PlaneShapeData {
    Vec3  normal;     // Outward normal
    Vec3  point;      // Point on the plane
    float extent;     // Half-extent for box approximation (e.g., 1000.0f)
};

// ---------------------------------------------------------------------------
// Box-shaped support function for a plane (GJK-compatible approximation).
// Treats the plane as an axis-aligned box with very large tangential extents
// and a thin solid side.
// ---------------------------------------------------------------------------
inline Vec3 plane_support(const void* user_data, const Vec3& dir, uint32_t& out_vertex_id) {
    const PlaneShapeData* data = static_cast<const PlaneShapeData*>(user_data);

    float dot_val = Vec3::dot(dir, data->normal);

    // Simple two-sided support for GJK compatibility.
    out_vertex_id = (dot_val >= 0.0f) ? 1u : 0u;

    float offset = (dot_val >= 0.0f) ? 0.0f : -0.001f;
    return Vec3::add(data->point, Vec3::scale(data->normal, offset));
}

} // namespace apc
