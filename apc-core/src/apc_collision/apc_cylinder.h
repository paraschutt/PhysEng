#pragma once
// =============================================================================
// Cylinder Collision Primitive — APC Deterministic Physics Engine
// =============================================================================
//
// Provides:
//   - CylinderShapeData : shape parameters + cached transform
//   - cylinder_support() : GJK-compatible support function
//   - cylinder_get_aabb(): analytical AABB for broadphase
//   - detect_sphere_cylinder(): dedicated sphere-cylinder fast-path detector
//
// Dependencies: apc_vec3, apc_quat, apc_mat3, apc_math_common, apc_broadphase
//
// Determinism: no FMA, index-order tie-breaking, deterministic fallback for
// degenerate support directions. All FP operations use explicit multiply-add
// ordering.
//
// =============================================================================

#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include "apc_math/apc_mat3.h"
#include "apc_math/apc_math_common.h"
#include "apc_sphere_sphere.h"  // ContactPoint canonical definition
#include "apc_broadphase.h"
#include <cstdint>
#include <cmath>

namespace apc {

// ---------------------------------------------------------------------------
// CylinderShapeData — parameters and cached transform for a cylinder.
//
// The cylinder axis is the local Y axis. The shape extends from -half_height
// to +half_height along Y with the given circular radius.
//
// update_cache() must be called after modifying orientation/rotation.
// ---------------------------------------------------------------------------
struct CylinderShapeData {
    float radius;          // Radius of the circular cross-section
    float half_height;     // Half the height along the local Y axis
    Vec3  position;        // World position (center of the cylinder)
    Quat  orientation;     // World orientation (Y = cylinder axis in local space)
    Mat3  rotation;        // Cached rotation matrix (set via update_cache)

    void update_cache() {
        rotation = Mat3::from_quat(orientation);
    }
};

// ---------------------------------------------------------------------------
// cylinder_support() — GJK-compatible support function.
//
// Returns the farthest point on the cylinder surface in the given world-space
// direction.  user_data must point to a CylinderShapeData whose update_cache()
// has already been called.
//
// Algorithm:
//   1. Transform direction into local space via transpose(rotation).
//   2. Axis component: sign(local_dir.y) * half_height.
//   3. Radial component: project local_dir onto the XZ plane, normalize,
//      scale by radius.  If the XZ projection is degenerate (nearly zero),
//      fall back to (radius, 0, 0) for determinism.
//   4. Compose local support point (radial_x, axis_y, radial_z).
//   5. Transform to world: position + rotation * local_point.
//
// DETERMINISM:
//   - Axis sign uses >= so positive half wins ties (consistent with
//     box_support and convex_piece_support tie-breaking).
//   - Degenerate XZ direction: deterministic fallback (radius, 0, 0).
//   - vertex_id encodes the selected vertex deterministically:
//       bit 0 : axis sign   (0 = negative, 1 = positive)
//       bit 1 : X sign of local_dir  (0 = negative, 1 = non-negative)
//       bit 2 : Z sign of local_dir  (0 = negative, 1 = non-negative)
//
// Signature matches ConvexHull::SupportFunc (see apc_gjk.h).
// ---------------------------------------------------------------------------
inline Vec3 cylinder_support(const void* user_data, const Vec3& dir,
                             uint32_t& out_vertex_id)
{
    const CylinderShapeData* data =
        static_cast<const CylinderShapeData*>(user_data);

    // 1. Transform direction into local cylinder space
    Vec3 local_dir = data->rotation.transpose().transform_vec(dir);

    // 2. Axis component: choose top or bottom cap based on Y direction
    float sy = (local_dir.y >= 0.0f) ? data->half_height
                                     : -data->half_height;

    // 3. Radial component: project onto XZ plane, normalize, scale by radius
    float xz_len_sq = local_dir.x * local_dir.x + local_dir.z * local_dir.z;
    float rx;
    float rz;

    if (xz_len_sq > APC_EPSILON_SQ) {
        float inv_xz_len = 1.0f / std::sqrt(xz_len_sq);
        rx = local_dir.x * inv_xz_len * data->radius;
        rz = local_dir.z * inv_xz_len * data->radius;
    } else {
        // Degenerate: direction nearly aligned with cylinder axis.
        // Deterministic fallback: pick +X direction.
        rx = data->radius;
        rz = 0.0f;
    }

    // 4. Compose local support point
    Vec3 local_point(rx, sy, rz);

    // 5. Encode deterministic vertex_id
    uint32_t axis_bit = (local_dir.y >= 0.0f) ? 1u : 0u;
    uint32_t x_bit    = (local_dir.x >= 0.0f) ? 1u : 0u;
    uint32_t z_bit    = (local_dir.z >= 0.0f) ? 1u : 0u;
    out_vertex_id = axis_bit | (x_bit << 1u) | (z_bit << 2u);

    // 6. Transform to world space: position + rotation * local_point
    return Vec3::add(data->position, data->rotation.transform_vec(local_point));
}

// ---------------------------------------------------------------------------
// cylinder_get_aabb() — analytical axis-aligned bounding box.
//
// Computes a tight (slightly conservative) AABB by decomposing the cylinder
// into its axis extent and radial extent:
//
//   For each world axis component i (x=0, y=1, z=2):
//     extent_i = |axis[i]| * half_height
//              + (|perp_0[i]| + |perp_2[i]|) * radius
//
// Where axis = column 1 of rotation (local Y → world), and perp_0 / perp_2
// are columns 0 and 2 of the rotation matrix respectively.
//
// The abs-sum bound for the radial part is conservative:
//   sqrt(a^2 + b^2) * radius  <=  (|a| + |b|) * radius
// This is standard practice for broadphase AABBs and avoids expensive sqrts.
// ---------------------------------------------------------------------------
inline AABB cylinder_get_aabb(const CylinderShapeData& cyl) {
    const float* m = cyl.rotation.m;

    // Column-major layout:
    //   column 0 (local X → world): m[0], m[1], m[2]
    //   column 1 (local Y → world): m[3], m[4], m[5]  ← cylinder axis
    //   column 2 (local Z → world): m[6], m[7], m[8]

    // Axis extent (cylinder axis = local Y = column 1)
    float ax_x = std::abs(m[3]);
    float ax_y = std::abs(m[4]);
    float ax_z = std::abs(m[5]);

    // Radial extent (columns 0 and 2 — abs-sum conservative bound)
    float rad_x = (std::abs(m[0]) + std::abs(m[6])) * cyl.radius;
    float rad_y = (std::abs(m[1]) + std::abs(m[7])) * cyl.radius;
    float rad_z = (std::abs(m[2]) + std::abs(m[8])) * cyl.radius;

    // Total half-extent per world axis
    float ext_x = ax_x * cyl.half_height + rad_x;
    float ext_y = ax_y * cyl.half_height + rad_y;
    float ext_z = ax_z * cyl.half_height + rad_z;

    AABB aabb;
    aabb.min = Vec3(cyl.position.x - ext_x,
                     cyl.position.y - ext_y,
                     cyl.position.z - ext_z);
    aabb.max = Vec3(cyl.position.x + ext_x,
                     cyl.position.y + ext_y,
                     cyl.position.z + ext_z);
    return aabb;
}

// ---------------------------------------------------------------------------
// detect_sphere_cylinder() — dedicated sphere-vs-cylinder collision detector.
//
// Tests a sphere (shape A) against a cylinder (shape B) and outputs a single
// contact if they overlap.  This is an optimisation over the generic
// GJK+EPA path for the common sphere-cylinder pair.
//
// Normal convention: B->A (from cylinder toward sphere).
//
// Algorithm:
//   1. Transform sphere center into cylinder local space.
//   2. Determine which surface feature (barrel / top cap / bottom cap) is
//      closest to the sphere center, then compute the closest point on that
//      feature.
//   3. If the sphere center is inside the cylinder, find the minimum-
//      penetration feature with deterministic tie-breaking:
//        barrel (radial) > top cap > bottom cap.
//   4. Check signed distance against sphere radius.
//   5. Transform normal and contact points to world space.
//
// update_cache() must have been called on cylinder before calling this.
//
// Returns true if colliding (penetration >= 0).
// ---------------------------------------------------------------------------
inline bool detect_sphere_cylinder(
    const Vec3& sphere_pos,
    float sphere_radius,
    const CylinderShapeData& cylinder,
    ContactPoint& out_contact)
{
    // -----------------------------------------------------------------------
    // 1. Transform sphere center into cylinder local space
    // -----------------------------------------------------------------------
    Vec3 diff  = Vec3::sub(sphere_pos, cylinder.position);
    Vec3 local = cylinder.rotation.transpose().transform_vec(diff);

    const float sy     = local.y;
    const float sx     = local.x;
    const float sz     = local.z;
    const float half_h = cylinder.half_height;
    const float rad    = cylinder.radius;

    // Radial distance in the XZ plane
    float sxz_len_sq = sx * sx + sz * sz;
    float sxz_len    = std::sqrt(sxz_len_sq);

    // Classification flags
    bool y_inside  = (sy >= -half_h && sy <= half_h);
    bool xz_inside = (sxz_len <= rad);

    Vec3  local_closest;
    Vec3  local_normal;
    float penetration;

    // -----------------------------------------------------------------------
    // Case 1: Sphere center is INSIDE the cylinder volume.
    //         Find the surface feature with minimum penetration depth.
    // -----------------------------------------------------------------------
    if (y_inside && xz_inside) {
        float d_barrel = rad   - sxz_len;   // distance to barrel wall
        float d_top    = half_h - sy;       // distance to top cap face
        float d_bot    = sy + half_h;       // distance to bottom cap face

        // Deterministic tie-breaking: barrel first, then top cap, then bottom
        if (d_barrel <= d_top && d_barrel <= d_bot) {
            // Closest to barrel — normal points radially outward
            if (sxz_len > APC_EPSILON) {
                float inv = 1.0f / sxz_len;
                local_normal = Vec3(sx * inv, 0.0f, sz * inv);
            } else {
                // On the axis: deterministic fallback +X
                local_normal = Vec3(1.0f, 0.0f, 0.0f);
            }
            local_closest = Vec3(local_normal.x * rad, sy, local_normal.z * rad);
            penetration   = d_barrel + sphere_radius;
        } else if (d_top <= d_bot) {
            // Closest to top cap face
            local_normal  = Vec3(0.0f, 1.0f, 0.0f);
            local_closest = Vec3(sx, half_h, sz);
            penetration   = d_top + sphere_radius;
        } else {
            // Closest to bottom cap face
            local_normal  = Vec3(0.0f, -1.0f, 0.0f);
            local_closest = Vec3(sx, -half_h, sz);
            penetration   = d_bot + sphere_radius;
        }
    }
    // -----------------------------------------------------------------------
    // Case 2: Sphere center is ABOVE or BELOW the cylinder, but within the
    //         cap disk projection (radially inside).
    //         Closest feature is the cap face.
    // -----------------------------------------------------------------------
    else if (!y_inside && xz_inside) {
        float sign_y = (sy >= 0.0f) ? 1.0f : -1.0f;
        float cap_y  = sign_y * half_h;
        // Signed distance from sphere center to the nearest cap plane
        float dist_to_cap = (sy >= 0.0f) ? (sy - half_h) : (-half_h - sy);

        if (dist_to_cap > sphere_radius) {
            return false; // Separated — sphere beyond cap face
        }

        local_normal  = Vec3(0.0f, sign_y, 0.0f);
        local_closest = Vec3(sx, cap_y, sz);
        penetration   = sphere_radius - dist_to_cap;
    }
    // -----------------------------------------------------------------------
    // Case 3: Sphere center is outside the cylinder's radial extent.
    //         Closest feature is on the barrel (or cap edge if Y is also
    //         outside). Compute closest point on the barrel at the clamped Y
    //         level, then measure Euclidean distance.
    // -----------------------------------------------------------------------
    else {
        // Clamp Y to cylinder height range
        float y_c = (sy < -half_h) ? -half_h : ((sy > half_h) ? half_h : sy);

        // Radial direction (normalized projection onto XZ)
        float rx, rz;
        if (sxz_len > APC_EPSILON) {
            float inv = 1.0f / sxz_len;
            rx = sx * inv * rad;
            rz = sz * inv * rad;
        } else {
            // Degenerate: sphere center exactly on the cylinder axis
            rx = rad;
            rz = 0.0f;
        }

        // Closest point on barrel at clamped Y
        local_closest = Vec3(rx, y_c, rz);

        // Distance from sphere center to closest point
        Vec3  delta   = Vec3::sub(local, local_closest);
        float dist_sq = Vec3::length_sq(delta);

        if (dist_sq > sphere_radius * sphere_radius) {
            return false; // Separated — sphere beyond barrel
        }

        float dist = std::sqrt(dist_sq);
        if (dist > APC_EPSILON) {
            // Normal: from cylinder surface toward sphere center (B->A)
            float inv = 1.0f / dist;
            local_normal = Vec3(delta.x * inv, delta.y * inv, delta.z * inv);
            penetration  = sphere_radius - dist;
        } else {
            // Degenerate: sphere center exactly on the cylinder surface.
            // Normal points radially outward from the barrel.
            local_normal = Vec3(rx / rad, 0.0f, rz / rad);
            penetration  = sphere_radius;
        }
    }

    // -----------------------------------------------------------------------
    // Transform results to world space
    // -----------------------------------------------------------------------

    // Normal: B->A (cylinder → sphere) — already correct from local computation
    out_contact.normal      = cylinder.rotation.transform_vec(local_normal);
    out_contact.penetration = penetration;

    // Contact point on cylinder surface (B)
    out_contact.point_on_b = Vec3::add(
        cylinder.position,
        cylinder.rotation.transform_vec(local_closest));

    // Contact point on sphere surface (A) — sphere center pushed toward B
    out_contact.point_on_a = Vec3::sub(
        sphere_pos,
        Vec3::scale(out_contact.normal, sphere_radius));

    return true;
}

} // namespace apc
