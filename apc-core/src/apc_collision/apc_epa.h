#pragma once
// =============================================================================
// EPA (Expanding Polytope Algorithm) — Deterministic Penetration Depth Solver
// =============================================================================
//
// Given a GJK simplex that encloses the origin (indicating convex shape
// intersection), EPA expands the simplex into a polytope approximating the
// Minkowski difference boundary, then finds the closest face to the origin.
// That face's normal and distance yield the contact normal and penetration
// depth. Barycentric interpolation over the face provides approximate contact
// points on each original shape.
//
// Determinism guarantees:
//   - All floating-point operations use explicit, fixed evaluation order
//   - No SIMD, no FMA instructions
//   - Face selection uses stable index-based tie-breaking (lowest index wins)
//   - Horizon edges are discovered in a deterministic face-order scan
//   - Convergence tolerance is APC_EPSILON (1e-6f)
//
// =============================================================================

#include "apc_math/apc_vec3.h"
#include "apc_math/apc_math_common.h"
#include "apc_gjk.h"          // ConvexHull (support-function wrapper), GJKResult
#include <cstdint>
#include <cmath>

namespace apc {

// ---------------------------------------------------------------------------
// Result of an EPA query. Fields are valid only when `success == true`.
// ---------------------------------------------------------------------------
struct EPAResult {
    bool   success;       // false if degenerate geometry or convergence failure
    Vec3   normal;        // Contact normal from A to B (matches ContactPoint convention)
    float  penetration;   // Depth of interpenetration (always >= 0 on success)
    Vec3   point_on_a;    // Approximate deepest contact point on surface of A
    Vec3   point_on_b;    // Approximate deepest contact point on surface of B
};

// ---------------------------------------------------------------------------
// EPA — Expanding Polytope Algorithm
// ---------------------------------------------------------------------------
class EPA {
public:
    // Hard iteration / vertex caps to guarantee bounded execution time.
    static constexpr uint32_t MAX_ITERATIONS = 64;
    static constexpr uint32_t MAX_VERTICES   = 64;
    // A convex polyhedron with V vertices and all-tri faces has at most
    // F = 2V − 4 faces (Euler: V − E + F = 2, E = 3F/2).  With V = 64
    // that gives F = 124.  We use 192 to leave headroom during expansion.
    static constexpr uint32_t MAX_FACES      = 192;

private:
    // -----------------------------------------------------------------------
    // Internal vertex: stores the Minkowski-difference point together with
    // the individual support points from hull A and hull B.  These are
    // needed to reconstruct approximate contact points via barycentric
    // interpolation at convergence.
    // -----------------------------------------------------------------------
    struct Vertex {
        Vec3 point;      // Minkowski difference vertex: support_A(dir) − support_B(−dir)
        Vec3 support_a;  // Corresponding support point on hull A
        Vec3 support_b;  // Corresponding support point on hull B
    };

    // -----------------------------------------------------------------------
    // Internal face (triangle).  The normal is *always* oriented outward
    // (away from the polytope interior / towards the origin side).
    // `distance` is the signed distance from the origin to the face plane:
    //   distance = dot(normal, any vertex of the face)
    // It is always >= 0 for valid polytopes that contain the origin.
    // -----------------------------------------------------------------------
    struct Face {
        uint32_t i[3];   // Indices into the vertex array (CCW when viewed from outside)
        Vec3     normal; // Unit outward normal
        float    distance;
    };

    // -----------------------------------------------------------------------
    // query() — main entry point
    // -----------------------------------------------------------------------
public:
    static EPAResult query(
        const Vec3       simplex_points[4],
        int              simplex_count,
        const ConvexHull& hull_a,
        const ConvexHull& hull_b)
    {
        EPAResult result;
        result.success     = false;
        result.normal      = Vec3(0.0f, 0.0f, 0.0f);
        result.penetration = 0.0f;
        result.point_on_a  = Vec3(0.0f, 0.0f, 0.0f);
        result.point_on_b  = Vec3(0.0f, 0.0f, 0.0f);

        // ---- Fixed-size storage ------------------------------------------------
        Vertex  vertices[MAX_VERTICES];
        uint32_t vertex_count = 0;

        Face    faces[MAX_FACES];
        uint32_t face_count   = 0;

        // ---- Helper: evaluate the Minkowski-difference support function -----------
        // Returns a − b, and writes the individual support points into out_a, out_b.
        auto minkowski_support = [&](const Vec3& dir,
                                     Vec3& out_a,
                                     Vec3& out_b) -> Vec3
        {
            Vec3 dir_neg = Vec3::scale(dir, -1.0f);
            uint32_t id_a = 0;
            uint32_t id_b = 0;
            out_a = hull_a.support(hull_a.user_data, dir,     id_a);
            out_b = hull_b.support(hull_b.user_data, dir_neg, id_b);
            return Vec3::sub(out_a, out_b);
        };

        // ---- Helper: check whether a Minkowski point is too close to an -----------
        //            existing vertex (indicates a degenerate / duplicate support).
        auto is_duplicate = [&](const Vec3& p) -> bool {
            for (uint32_t vi = 0; vi < vertex_count; ++vi) {
                Vec3 diff = Vec3::sub(vertices[vi].point, p);
                if (Vec3::length_sq(diff) < APC_EPSILON_SQ) {
                    return true;
                }
            }
            return false;
        };

        // ---- Helper: compute face normal & distance, orient outward ----------------
        // Returns false for degenerate (zero-area) faces.
        auto compute_face = [&](Face& f) -> bool {
            const Vec3& a = vertices[f.i[0]].point;
            const Vec3& b = vertices[f.i[1]].point;
            const Vec3& c = vertices[f.i[2]].point;

            Vec3 ab = Vec3::sub(b, a);
            Vec3 ac = Vec3::sub(c, a);
            Vec3 n  = Vec3::cross(ab, ac);

            float len_sq = Vec3::length_sq(n);
            if (len_sq < APC_EPSILON_SQ) {
                return false;  // Degenerate triangle
            }

            float len      = std::sqrt(len_sq);
            float inv_len  = 1.0f / len;
            f.normal       = Vec3::scale(n, inv_len);
            f.distance     = Vec3::dot(f.normal, a);

            // The origin is inside the polytope, so the outward-facing normal
            // must satisfy dot(normal, vertex) > 0.  Flip if needed.
            if (f.distance < 0.0f) {
                f.normal   = Vec3::scale(f.normal, -1.0f);
                f.distance = -f.distance;
                // Reverse winding to stay consistent
                uint32_t tmp = f.i[1];
                f.i[1] = f.i[2];
                f.i[2] = tmp;
            }

            return true;
        };

        // ---- Helper: append a validated face to the face array --------------------
        auto add_face = [&](uint32_t i0, uint32_t i1, uint32_t i2) -> bool {
            if (face_count >= MAX_FACES) {
                return false;
            }
            Face& f = faces[face_count];
            f.i[0] = i0;
            f.i[1] = i1;
            f.i[2] = i2;
            if (!compute_face(f)) {
                return false;  // Degenerate — silently skip
            }
            ++face_count;
            return true;
        };

        // ---- Helper: append a vertex ---------------------------------------------
        auto add_vertex = [&](const Vec3& pt,
                              const Vec3& sa,
                              const Vec3& sb) -> uint32_t
        {
            if (vertex_count >= MAX_VERTICES) {
                return UINT32_MAX;
            }
            vertices[vertex_count].point     = pt;
            vertices[vertex_count].support_a = sa;
            vertices[vertex_count].support_b = sb;
            return vertex_count++;
        };

        // =========================================================================
        // STEP 1 — Seed the polytope from the GJK simplex
        // =========================================================================
        //
        // The GJK simplex stores Minkowski-difference points but NOT the
        // individual A / B support points.  To recover them we re-evaluate
        // the support functions.  Using the simplex point's direction from
        // the origin as the search direction gives us the exact (or
        // numerically equivalent) support pair, because the support function
        // returns the farthest point in that direction — which is the same
        // point GJK originally found.
        // =========================================================================

        for (int si = 0; si < simplex_count && si < 4; ++si) {
            const Vec3& p = simplex_points[si];

            // Direction from origin to the simplex point
            Vec3 dir = Vec3::normalize(p);

            // If the point is at (or extremely close to) the origin, the
            // support direction is degenerate.  Use an arbitrary fallback.
            if (Vec3::length_sq(dir) < APC_EPSILON_SQ) {
                // Deterministic per-index fallback axes
                float fx = 0.0f, fy = 0.0f, fz = 0.0f;
                switch (si) {
                    case 0: fx = 1.0f; break;
                    case 1: fy = 1.0f; break;
                    case 2: fz = 1.0f; break;
                    default: fx = -1.0f; break;
                }
                dir = Vec3(fx, fy, fz);
                dir = Vec3::normalize(dir);
            }

            Vec3 sup_a, sup_b;
            // Evaluate the support functions to recover the individual A/B
            // support points.  The returned Minkowski point should match
            // (or be numerically equivalent to) the GJK simplex point.
            minkowski_support(dir, sup_a, sup_b);

            // Use the original GJK simplex point for the Minkowski vertex
            // (numerical fidelity to the GJK result) but record the recovered
            // A / B support points for contact interpolation.
            add_vertex(p, sup_a, sup_b);
        }

        // =========================================================================
        // STEP 1b — Promote triangle → tetrahedron if needed
        // =========================================================================
        //
        // GJK's do_simplex() only returns `true` (intersection) from the
        // tetrahedron case, so simplex_count should always be 4.  However,
        // we defensively handle the degenerate triangle case by adding a
        // fourth support point to form a valid volume.
        // =========================================================================

        if (vertex_count == 3) {
            Vec3 ab  = Vec3::sub(vertices[1].point, vertices[0].point);
            Vec3 ac  = Vec3::sub(vertices[2].point, vertices[0].point);
            Vec3 tn  = Vec3::cross(ab, ac);

            if (Vec3::length_sq(tn) < APC_EPSILON_SQ) {
                return result;  // Degenerate triangle — abort
            }

            Vec3 dir_pos = Vec3::normalize(tn);
            Vec3 dir_neg = Vec3::scale(dir_pos, -1.0f);

            Vec3 sa_p, sb_p, sa_n, sb_n;
            Vec3 pt_p = minkowski_support(dir_pos, sa_p, sb_p);
            Vec3 pt_n = minkowski_support(dir_neg, sa_n, sb_n);

            // Pick the side that gives a larger tetrahedron volume
            float d0 = Vec3::dot(Vec3::sub(pt_p, vertices[0].point), dir_pos);
            float d1 = Vec3::dot(Vec3::sub(pt_n, vertices[0].point), dir_pos);

            if (d0 >= d1) {
                add_vertex(pt_p, sa_p, sb_p);
            } else {
                add_vertex(pt_n, sa_n, sb_n);
            }
        }

        if (vertex_count < 4) {
            return result;  // Insufficient geometry
        }

        // =========================================================================
        // STEP 1c — Orient the base triangle and build the initial tetrahedron
        // =========================================================================
        //
        // We have four vertices (indices 0-3).  First, orient the base
        // triangle (0,1,2) so that its normal points AWAY from vertex 3.
        // Then the four outward-facing faces are:
        //   Bottom : (0, 1, 2)  — normal points away from v3
        //   Side 0 : (0, 3, 1)
        //   Side 1 : (1, 3, 2)
        //   Side 2 : (2, 3, 0)
        //
        // These windings produce normals that all face outward from the
        // tetrahedron.  compute_face() will verify and correct if needed.
        // =========================================================================

        {
            Vec3 ab  = Vec3::sub(vertices[1].point, vertices[0].point);
            Vec3 ac  = Vec3::sub(vertices[2].point, vertices[0].point);
            Vec3 tn  = Vec3::cross(ab, ac);
            Vec3 to3 = Vec3::sub(vertices[3].point, vertices[0].point);

            if (Vec3::dot(tn, to3) > 0.0f) {
                // Normal currently points TOWARD vertex 3 — flip winding
                // so it points AWAY from vertex 3 (outward for bottom face).
                Vertex tmp_v = vertices[1];
                vertices[1]  = vertices[2];
                vertices[2]  = tmp_v;
            }
        }

        // Build the four tetrahedron faces
        if (!add_face(0, 1, 2)) return result;  // Bottom
        if (!add_face(0, 3, 1)) return result;  // Side A
        if (!add_face(1, 3, 2)) return result;  // Side B
        if (!add_face(2, 3, 0)) return result;  // Side C

        // =========================================================================
        // STEP 2 — Main EPA expansion loop
        // =========================================================================

        for (uint32_t iter = 0; iter < MAX_ITERATIONS; ++iter) {

            // ---- 2a. Find the face closest to the origin -------------------------
            //
            // DETERMINISM: we scan faces in index order and keep the first
            // face that achieves the minimum distance.  This ensures that
            // ties are always broken the same way across runs / platforms.
            // --------------------------------------------------------------------
            uint32_t closest_idx = 0;
            float    closest_dist = faces[0].distance;

            for (uint32_t fi = 1; fi < face_count; ++fi) {
                if (faces[fi].distance < closest_dist) {
                    closest_dist = faces[fi].distance;
                    closest_idx  = fi;
                }
            }

            const Face& closest = faces[closest_idx];

            // ---- 2b. Get the Minkowski support in the direction of the ----------
            //      closest face's outward normal.
            // --------------------------------------------------------------------
            Vec3 sup_a, sup_b;
            Vec3 new_point = minkowski_support(closest.normal, sup_a, sup_b);

            // ---- 2c. Convergence test -------------------------------------------
            //
            // If the new support point is not significantly farther from the
            // origin than the closest face, the polytope has expanded as far
            // as the Minkowski difference boundary allows in this direction.
            // We have found the penetration feature.
            //
            // new_dist = dot(new_point, normal) is the distance of the new
            // support from the origin along the face normal.  If it barely
            // exceeds the current face distance, we're on the boundary.
            // --------------------------------------------------------------------
            float new_dist = Vec3::dot(new_point, closest.normal);

            if ((new_dist - closest_dist) < APC_EPSILON) {
                // ---- CONVERGED — extract the result ----------------------------
                result.success     = true;
                result.normal      = closest.normal;
                result.penetration = closest_dist;

                // ---- Contact point estimation via barycentric coordinates ------
                //
                // The origin projects onto the closest face's plane at:
                //   P = normal * distance
                // We compute barycentric coords (w, u, v) of P w.r.t. the
                // face triangle, then interpolate the tracked support_a /
                // support_b points to get approximate contact locations.
                //
                // DETERMINISM: the barycentric computation uses explicit
                // fixed-order FP ops (dot products computed xx+yy+zz).
                // ---------------------------------------------------------------
                compute_contact_points(
                    vertices, closest,
                    result.point_on_a, result.point_on_b);

                return result;
            }

            // ---- 2d. Duplicate vertex check -------------------------------------
            //
            // If the new support point coincides with an existing vertex,
            // we cannot meaningfully expand.  Treat as convergence.
            // --------------------------------------------------------------------
            if (is_duplicate(new_point)) {
                result.success     = true;
                result.normal      = closest.normal;
                result.penetration = closest_dist;
                compute_contact_points(
                    vertices, closest,
                    result.point_on_a, result.point_on_b);
                return result;
            }

            // ---- 2e. Add the new vertex -----------------------------------------
            uint32_t new_idx = add_vertex(new_point, sup_a, sup_b);
            if (new_idx == UINT32_MAX) {
                // Hard vertex limit reached — fail gracefully
                return result;
            }

            // ---- 2f. Classify faces as visible / non-visible --------------------
            //
            // A face is "visible" from the new point if:
            //   dot(face_normal, new_point − face_vertex) > 0
            //
            // i.e. the new point is on the outward side of the face.
            // DETERMINISM: the classification depends only on the face's
            // stored normal (which was deterministically computed) and the
            // new vertex position.
            // --------------------------------------------------------------------
            bool visible[MAX_FACES];
            for (uint32_t fi = 0; fi < face_count; ++fi) {
                Vec3 to_new = Vec3::sub(new_point,
                                        vertices[faces[fi].i[0]].point);
                visible[fi] = (Vec3::dot(faces[fi].normal, to_new) > 0.0f);
            }

            // ---- 2g. Find horizon edges ----------------------------------------
            //
            // An edge is on the horizon if it is shared between exactly one
            // visible face and one non-visible face.  In a closed polytope
            // each edge belongs to exactly two faces.
            //
            // DETERMINISM: we scan visible faces in index order, and for
            // each visible face scan its three edges in fixed order.
            // Non-visible adjacency is checked in index order.
            //
            // We record each horizon edge with canonical ordering
            // (smallest index first) to prevent duplicates (though in a
            // proper polytope duplicates should not arise).
            // --------------------------------------------------------------------
            struct HorizonEdge {
                uint32_t a;
                uint32_t b;
            };

            HorizonEdge horizon[MAX_FACES * 3];
            uint32_t    horizon_count = 0;

            for (uint32_t fi = 0; fi < face_count; ++fi) {
                if (!visible[fi]) continue;

                const Face& f = faces[fi];
                // Three edges of the face, wound consistently with the face
                static const int edge_idx[3][2] = {
                    {0, 1}, {1, 2}, {2, 0}
                };

                for (uint32_t ei = 0; ei < 3; ++ei) {
                    uint32_t ea = f.i[edge_idx[ei][0]];
                    uint32_t eb = f.i[edge_idx[ei][1]];

                    // Check all NON-visible faces for adjacency
                    bool on_horizon = false;
                    for (uint32_t fj = 0; fj < face_count; ++fj) {
                        if (fj == fi || visible[fj]) continue;

                        const Face& nf = faces[fj];
                        for (uint32_t k = 0; k < 3; ++k) {
                            uint32_t na = nf.i[k];
                            uint32_t nb = nf.i[(k + 1) % 3];
                            if ((na == ea && nb == eb) ||
                                (na == eb && nb == ea))
                            {
                                on_horizon = true;
                                break;
                            }
                        }
                        if (on_horizon) break;
                    }

                    if (on_horizon) {
                        if (horizon_count < MAX_FACES * 3) {
                            horizon[horizon_count].a = ea;
                            horizon[horizon_count].b = eb;
                            ++horizon_count;
                        }
                    }
                }
            }

            // A valid horizon must have at least 3 edges (closed ring)
            if (horizon_count < 3) {
                return result;  // Degenerate topology
            }

            // ---- 2h. Remove visible faces (compact the array) -------------------
            uint32_t new_face_count = 0;
            for (uint32_t fi = 0; fi < face_count; ++fi) {
                if (!visible[fi]) {
                    faces[new_face_count] = faces[fi];
                    ++new_face_count;
                }
            }
            face_count = new_face_count;

            // ---- 2i. Create new faces from horizon edges → new vertex -----------
            //
            // Each horizon edge (ea, eb) plus the new vertex new_idx forms a
            // triangle.  compute_face() will orient the normal outward.
            //
            // DETERMINISM: edges are appended in discovery order (index scan).
            // --------------------------------------------------------------------
            for (uint32_t hi = 0; hi < horizon_count; ++hi) {
                if (!add_face(horizon[hi].a, horizon[hi].b, new_idx)) {
                    // Face limit or degenerate face — fail gracefully.
                    // The solver will skip this contact pair.
                    return result;
                }
            }
        }

        // =========================================================================
        // STEP 3 — Iteration limit reached without convergence
        // =========================================================================
        // Return failure.  The calling collision solver should skip this
        // contact pair to prevent physics instabilities from a bad normal.
        // =========================================================================
        return result;
    }

private:
    // ===========================================================================
    // compute_contact_points()
    // ===========================================================================
    //
    // Given the closest face and the full vertex array, compute approximate
    // contact points on hulls A and B by:
    //   1. Projecting the origin onto the face plane → point P
    //   2. Computing barycentric coordinates of P within the face triangle
    //   3. Interpolating the tracked support_a / support_b using those coords
    //
    // Falls back to the face centroid if the triangle is degenerate.
    // ===========================================================================
    static void compute_contact_points(
        const Vertex vertices[MAX_VERTICES],
        const Face&  face,
        Vec3&        out_point_on_a,
        Vec3&        out_point_on_b)
    {
        const Vec3& pa = vertices[face.i[0]].point;  // V0
        const Vec3& pb = vertices[face.i[1]].point;  // V1
        const Vec3& pc = vertices[face.i[2]].point;  // V2

        // Projection of the origin onto the face plane
        Vec3 proj = Vec3::scale(face.normal, face.distance);

        // Edge vectors from V0
        Vec3 v0 = Vec3::sub(pc, pa);   // V2 − V0  (u-axis)
        Vec3 v1 = Vec3::sub(pb, pa);   // V1 − V0  (v-axis)
        Vec3 v2 = Vec3::sub(proj, pa); // P  − V0

        // Dot products (explicit xx+yy+zz order for determinism)
        float d00 = Vec3::dot(v0, v0);
        float d01 = Vec3::dot(v0, v1);
        float d02 = Vec3::dot(v0, v2);
        float d11 = Vec3::dot(v1, v1);
        float d12 = Vec3::dot(v1, v2);

        float denom = d00 * d11 - d01 * d01;

        if (denom < APC_EPSILON_SQ) {
            // Degenerate (zero-area) triangle — use centroid
            const float third = 1.0f / 3.0f;
            Vec3 ca = Vec3::add(
                          Vec3::add(vertices[face.i[0]].support_a,
                                    vertices[face.i[1]].support_a),
                          vertices[face.i[2]].support_a);
            out_point_on_a = Vec3::scale(ca, third);

            Vec3 cb = Vec3::add(
                          Vec3::add(vertices[face.i[0]].support_b,
                                    vertices[face.i[1]].support_b),
                          vertices[face.i[2]].support_b);
            out_point_on_b = Vec3::scale(cb, third);
            return;
        }

        // Barycentric coordinates of P within triangle (V0, V1, V2):
        //   u → weight for V2 (along v0)
        //   v → weight for V1 (along v1)
        //   w = 1 − u − v → weight for V0
        float inv_denom = 1.0f / denom;
        float u = (d11 * d02 - d01 * d12) * inv_denom;
        float v = (d00 * d12 - d01 * d02) * inv_denom;
        float w = 1.0f - u - v;

        // Clamp to [0, ∞) and renormalize to handle slight FP overshoot
        if (w < 0.0f) w = 0.0f;
        if (u < 0.0f) u = 0.0f;
        if (v < 0.0f) v = 0.0f;
        float sum = w + u + v;
        if (sum > APC_EPSILON) {
            float inv_sum = 1.0f / sum;
            w *= inv_sum;
            u *= inv_sum;
            v *= inv_sum;
        }

        // Interpolate the individual support points from A and B
        const Vec3& sa0 = vertices[face.i[0]].support_a;
        const Vec3& sa1 = vertices[face.i[1]].support_a;
        const Vec3& sa2 = vertices[face.i[2]].support_a;
        const Vec3& sb0 = vertices[face.i[0]].support_b;
        const Vec3& sb1 = vertices[face.i[1]].support_b;
        const Vec3& sb2 = vertices[face.i[2]].support_b;

        // point_on_a = w·sa0 + v·sa1 + u·sa2
        // (deterministic: each scale then two adds)
        out_point_on_a = Vec3::add(
                             Vec3::add(Vec3::scale(sa0, w),
                                       Vec3::scale(sa1, v)),
                             Vec3::scale(sa2, u));

        // point_on_b = w·sb0 + v·sb1 + u·sb2
        out_point_on_b = Vec3::add(
                             Vec3::add(Vec3::scale(sb0, w),
                                       Vec3::scale(sb1, v)),
                             Vec3::scale(sb2, u));
    }
};

} // namespace apc
