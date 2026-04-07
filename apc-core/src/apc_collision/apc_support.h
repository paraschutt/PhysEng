#pragma once
// =============================================================================
// Support Function Implementations for GJK Integration
// =============================================================================
//
// Provides shape-specific support functions that match the ConvexHull::SupportFunc
// signature. These take raw shape data (via user_data) and return the farthest
// point on the shape boundary in a given direction.
//
// Each shape has:
//   1. A ShapeData struct (passed as ConvexHull::user_data)
//   2. A support function implementing the ConvexHull::SupportFunc signature
//
// Determinism: all tie-breaking is index-order (first-found wins for convex
// piece scan; lexicographic sign combination for box vertices).
//
// =============================================================================

#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include "apc_math/apc_mat3.h"
#include "apc_math/apc_math_common.h"
#include "apc_gjk.h"  // ConvexHull::SupportFunc signature, GJKSimplex
#include <cstdint>

namespace apc {

// ---------------------------------------------------------------------------
// Shape data wrappers passed as ConvexHull::user_data
// ---------------------------------------------------------------------------

struct SphereShapeData {
    float radius;
};

struct BoxShapeData {
    Vec3 half_extents;  // half-widths (x, y, z)
    Vec3 position;      // world position
    Quat orientation;   // world orientation (must be normalized)
    Mat3 rotation;      // cached rotation matrix (set via update_cache)

    void update_cache() {
        rotation = Mat3::from_quat(orientation);
    }
};

struct ConvexPieceShapeData {
    const Vec3* vertices;   // pointer to vertex array
    uint32_t vertex_count;
    Vec3 position;          // world position
    Quat orientation;       // world orientation
    Mat3 rotation;          // cached rotation matrix

    void update_cache() {
        rotation = Mat3::from_quat(orientation);
    }
};

// ---------------------------------------------------------------------------
// Support function for a sphere centered at origin.
// user_data must point to SphereShapeData.
// ---------------------------------------------------------------------------
inline Vec3 sphere_support(const void* user_data, const Vec3& dir, uint32_t& out_vertex_id) {
    (void)out_vertex_id;
    const SphereShapeData* data = static_cast<const SphereShapeData*>(user_data);
    float len = std::sqrt(Vec3::length_sq(dir));
    if (len < APC_EPSILON) {
        return Vec3(data->radius, 0.0f, 0.0f);
    }
    float inv_len = 1.0f / len;
    return Vec3(dir.x * inv_len * data->radius,
                dir.y * inv_len * data->radius,
                dir.z * inv_len * data->radius);
}

// ---------------------------------------------------------------------------
// Support function for an oriented box.
// user_data must point to BoxShapeData (update_cache() must have been called).
//
// DETERMINISM: if two vertices have equal dot products, we pick the one with
// the lowest lexicographic index of its sign combination (++, +-, -+, etc.)
// ---------------------------------------------------------------------------
inline Vec3 box_support(const void* user_data, const Vec3& dir, uint32_t& out_vertex_id) {
    const BoxShapeData* data = static_cast<const BoxShapeData*>(user_data);

    // Transform direction into local box space
    Vec3 local_dir = data->rotation.transpose().transform_vec(dir);

    // Support point in local space: sign of each component determines which face
    float sx = (local_dir.x >= 0.0f) ? data->half_extents.x : -data->half_extents.x;
    float sy = (local_dir.y >= 0.0f) ? data->half_extents.y : -data->half_extents.y;
    float sz = (local_dir.z >= 0.0f) ? data->half_extents.z : -data->half_extents.z;

    // Encode vertex as a 3-bit id: bit0 = x sign, bit1 = y sign, bit2 = z sign
    out_vertex_id = ((local_dir.x >= 0.0f) ? 1u : 0u) |
                    ((local_dir.y >= 0.0f) ? 2u : 0u) |
                    ((local_dir.z >= 0.0f) ? 4u : 0u);

    // Transform back to world space
    Vec3 local_point(sx, sy, sz);
    return Vec3::add(data->position, data->rotation.transform_vec(local_point));
}

// ---------------------------------------------------------------------------
// Support function for a convex piece (raw vertex cloud).
// user_data must point to ConvexPieceShapeData (update_cache() must have been
// called).
//
// DETERMINISM: among vertices with equal maximum dot products, we pick the
// one with the lowest index (first-found-wins via linear scan).
// ---------------------------------------------------------------------------
inline Vec3 convex_piece_support(const void* user_data, const Vec3& dir, uint32_t& out_vertex_id) {
    const ConvexPieceShapeData* data = static_cast<const ConvexPieceShapeData*>(user_data);

    // Transform direction into local space
    Vec3 local_dir = data->rotation.transpose().transform_vec(dir);

    float max_dot = -1e30f;
    uint32_t best_idx = 0u;

    // Linear scan — DETERMINISM: first index wins ties
    for (uint32_t i = 0u; i < data->vertex_count; ++i) {
        const Vec3& v = data->vertices[i];
        float d = v.x * local_dir.x + v.y * local_dir.y + v.z * local_dir.z;
        if (d > max_dot) {
            max_dot = d;
            best_idx = i;
        }
    }

    out_vertex_id = best_idx;

    // Transform local vertex to world space
    Vec3 world_local = data->rotation.transform_vec(data->vertices[best_idx]);
    return Vec3::add(data->position, world_local);
}

// ---------------------------------------------------------------------------
// Helper: positioned sphere support (sphere with explicit world position).
// Used by the collision dispatcher for sphere-vs-convex GJK queries.
// ---------------------------------------------------------------------------
struct PositionedSphereData {
    float radius;
    Vec3  position;
};

inline Vec3 positioned_sphere_support(const void* user_data, const Vec3& dir, uint32_t& out_vertex_id) {
    (void)out_vertex_id;
    const PositionedSphereData* data = static_cast<const PositionedSphereData*>(user_data);
    float len = std::sqrt(Vec3::length_sq(dir));
    if (len < APC_EPSILON) {
        return Vec3::add(data->position, Vec3(data->radius, 0.0f, 0.0f));
    }
    float inv_len = 1.0f / len;
    Vec3 offset(dir.x * inv_len * data->radius,
                dir.y * inv_len * data->radius,
                dir.z * inv_len * data->radius);
    return Vec3::add(data->position, offset);
}

} // namespace apc
