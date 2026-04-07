# Collision System — Adaptive Physics Core (APC)

> **Phase 1 core subsystem** — deterministic collision detection for a C++17 physics engine.
> All algorithms are designed for bit-exact cross-platform reproducibility: no SIMD, no FMA,
> `std::stable_sort` for ordering, index-first tie-breaking everywhere.

---

## Table of Contents

1. [Pipeline Overview](#1-pipeline-overview)
2. [Broadphase — Sweep-and-Prune](#2-broadphase--sweep-and-prune)
3. [Midphase — OBB Trees](#3-midphase--obb-trees)
4. [Narrowphase — GJK + EPA](#4-narrowphase--gjk--epa)
5. [Dedicated Fast-Path Detectors](#5-dedicated-fast-path-detectors)
6. [Collision Dispatch](#6-collision-dispatch)
7. [Supported Shapes](#7-supported-shapes)
8. [Contact Data](#8-contact-data)
9. [Asset Pipeline](#9-asset-pipeline)

---

## 1. Pipeline Overview

APC uses a classical three-tier collision detection pipeline:

```
Proxies (AABBs)  ──►  Broadphase (SAP)  ──►  BroadphasePairs
                                                       │
                                                       ▼
                     Midphase (OBB Tree)  ◄──  Pair lookup
                            │
                            ▼
                   Narrowphase (GJK/EPA or fast-path)
                            │
                            ▼
                      ContactManifold
```

| Tier | Purpose | Source | Complexity |
|------|---------|--------|------------|
| **Broadphase** | Cull impossible pairs using AABB overlap | `apc_broadphase.h` | O(N log N) sort + O(N) sweep |
| **Midphase** | Refine complex shapes (convex decompositions) against each other | `apc_obb_tree.h` | O(log K) per query, K = piece count |
| **Narrowphase** | Generate precise contact geometry (normal, penetration, contact points) | `apc_gjk.h`, `apc_epa.h`, dedicated detectors | O(1) for fast-paths, O(iter) for GJK/EPA |

### Data Flow Per Frame

1. **Update proxies** — each body's `CollisionShape::get_aabb()` produces an `AABB` proxy.
2. **Broadphase** — `BroadphaseSAP::update()` sorts endpoints; `generate_pairs()` sweeps and produces `BroadphasePair` candidates (normalized `id_a < id_b`).
3. **Midphase** (for multi-piece convex assets) — `OBBTree::query_overlap()` tests individual convex pieces against the other body's AABB/OBB. Only overlapping pieces proceed.
4. **Narrowphase** — `dispatch_detect()` routes each surviving pair to the appropriate detector, producing a `ContactManifold` with up to 4 contacts.
5. **Contact persistence** — `ContactManager::update()` matches new contacts against previous-frame contacts for solver warmstarting.

---

## 2. Broadphase — Sweep-and-Prune

**File:** `apc_broadphase.h`

The broadphase performs AABB-based pair culling using the Sweep-and-Prune (SAP) algorithm
on the X axis, with Y/Z overlap rejection.

### Algorithm

```cpp
// 1. Create min/max endpoints on X for each proxy
for (const auto& p : proxies) {
    endpoints.push_back({p.aabb.min.x, true,  p.id});   // min endpoint
    endpoints.push_back({p.aabb.max.x, false, p.id});   // max endpoint
}

// 2. Deterministic sort
std::stable_sort(endpoints.begin(), endpoints.end(),
    [](const Endpoint& a, const Endpoint& b) {
        if (a.x != b.x) return a.x < b.x;    // primary: X coordinate
        if (a.is_min != b.is_min) return a.is_min;  // min before max
        return a.id < b.id;                    // tie-break: entity ID
    });

// 3. O(N) sweep — maintain active set, test Y/Z overlap on insert
for (const auto& ep : endpoints) {
    if (ep.is_min) {
        for (active : active_set) {
            if (!aabb_overlap_yz(active, ep)) continue;  // early out
            if (id_a > id_b) swap(id_a, id_b);            // normalize
            if (filter && !filter(id_a, id_b, ud)) continue;
            pairs.push_back({id_a, id_b});
        }
        active_set.insert(ep.id);
    } else {
        active_set.remove(ep.id);
    }
}
```

### Key Design Decisions

| Decision | Rationale |
|----------|-----------|
| `std::stable_sort` | Preserves insertion order for equal X values — deterministic across platforms |
| X-axis sweep only | Y/Z tested as cheap overlap rejection; avoids 3-axis sort overhead |
| Stack-allocated active set (`uint32_t[512]`) | Avoids heap allocation in the hot path |
| `FlatMap<uint32_t, AABB>` for lookup | O(log N) AABB retrieval per overlap test; avoids hashmap nondeterminism |
| Pair normalization (`id_a < id_b`) | Guarantees each unordered pair appears exactly once |
| `FilterFunc` callback | Layer-specific filtering (e.g., ignore bodies on the same ragdoll) |

### Pair Normalization

Every `BroadphasePair` is guaranteed to satisfy `id_a < id_b`. This is enforced both during sweep insertion and in the final `std::stable_sort` on the output vector. The `BroadphasePair::operator<` provides lexicographic ordering for deterministic solver iteration:

```cpp
bool operator<(const BroadphasePair& other) const {
    if (id_a != other.id_a) return id_a < other.id_a;
    return id_b < other.id_b;
}
```

---

## 3. Midphase — OBB Trees

**File:** `apc_obb_tree.h`

The midphase accelerates collision detection for complex shapes that are represented as a
convex decomposition (multiple `ConvexPiece` hulls from V-HACD). Rather than testing every
piece against every other body, an OBB tree culls pieces whose bounding volumes don't overlap.

### Tree Construction

```
Phase 1:  Create leaf nodes — tight AABB for each ConvexPiece
Phase 2:  Top-down binary tree — median split on longest axis
```

The split heuristic picks the **longest axis** of the combined child bounding box. Ties are
broken deterministically: X > Y > Z. Leaves are sorted by their center coordinate on the split
axis using `std::stable_sort`, then split at the midpoint. Internal node OBBs are computed as
AABB enclosures of their children.

### 15-Axis SAT for OBB-OBB Overlap

**File:** `apc_obb.h` — `obb_intersect()`

The overlap test uses the full Separating Axis Theorem with **all 15 potential separating
axes** and **no heuristic early exits**. This is critical for determinism — heuristic
reordering based on projected overlap magnitudes would produce different axis test orders
across platforms.

```cpp
// Strict testing order: 15 axes total
// Axes 1-3:   A's face normals (a_axes[0..2])
// Axes 4-6:   B's face normals (b_axes[0..2])
// Axes 7-15:  Edge-edge cross products (a_axes[i] x b_axes[j])

if (!test_axis(a_axes[0])) return false;
if (!test_axis(a_axes[1])) return false;
if (!test_axis(a_axes[2])) return false;
if (!test_axis(b_axes[0])) return false;
if (!test_axis(b_axes[1])) return false;
if (!test_axis(b_axes[2])) return false;
for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
        if (!test_axis(cross(a_axes[i], b_axes[j]))) return false;
return true;  // No separating axis found → overlapping
```

### Refit for Dynamic Transforms

When a body moves, `OBBTree::refit(pos, rot)` transforms all node OBBs to world space
by applying the root rigid-body transform:

```cpp
void refit(const Vec3& position, const Quat& rotation) {
    Mat3 rot_mat = Mat3::from_quat(rotation);
    for (auto& node : nodes) {
        node.box.center = position + rot_mat * node.box.center;
        node.box.orientation = rotation * node.box.orientation;
        node.box.update_cache();
    }
}
```

### Deterministic Traversal

`query_overlap()` traverses the tree **left-before-right** at every internal node,
ensuring that the first overlapping leaf found is always the same regardless of platform:

```cpp
bool query_recursive(int node_idx, const OBB& query) const {
    if (!obb_intersect(nodes[node_idx].box, query)) return false;
    if (nodes[node_idx].hull_index >= 0) return true;  // leaf
    if (query_recursive(nodes[node_idx].left_child, query))  return true;
    if (query_recursive(nodes[node_idx].right_child, query)) return true;
    return false;
}
```

---

## 4. Narrowphase — GJK + EPA

### 4.1 GJK Boolean Query

**File:** `apc_gjk.h`

The Gilbert-Johnson-Keerthi algorithm determines whether two convex shapes intersect
by searching for the origin in their Minkowski difference.

| Parameter | Value |
|-----------|-------|
| Max iterations | 32 |
| Initial direction | `(1, 0, 0)` |
| Support function | `ConvexHull::SupportFunc` — function pointer abstraction |
| Simplex sizes | 1→2→3→4 (point→line→triangle→tetrahedron) |

The `query_with_simplex()` variant returns both the boolean result and the final
GJK simplex for EPA seeding.

**Support function abstraction:**

```cpp
struct ConvexHull {
    using SupportFunc = Vec3 (*)(const void* shape, const Vec3& dir, uint32_t& out_vertex_id);
    const void* user_data;
    SupportFunc support;
};
```

The Minkowski difference support is computed as:
```
support_Minkowski(dir) = support_A(dir) - support_B(-dir)
```

**Determinism in support functions:**
- **Box:** sign of each local-space axis component selects the face (ties: positive wins via `>=`).
- **Convex piece:** linear scan, **first-found-wins** for equal dot products.
- **Sphere/Capsule/Cylinder:** normalized direction * radius; degenerate fallback is `(+radius, 0, 0)`.

**Degenerate simplex handling:** GJK iterates up to 32 times. If the origin cannot be enclosed,
the shapes are reported as separated (safe fallback to prevent physics explosions).

### 4.2 EPA — Expanding Polytope Algorithm

**File:** `apc_epa.h`

Given a GJK simplex that contains the origin, EPA expands it into a polytope that
approximates the Minkowski difference boundary, then finds the face closest to the origin.

| Limit | Value | Rationale |
|-------|-------|-----------|
| `MAX_ITERATIONS` | 64 | Bounded convergence time |
| `MAX_VERTICES` | 64 | Euler's formula: F = 2V - 4 = 124 for all-tri faces |
| `MAX_FACES` | 192 | Headroom during expansion (124 theoretical max at V=64) |
| Convergence tolerance | `APC_EPSILON` (1e-6) | `new_dist - closest_dist < epsilon` |

**Algorithm steps:**

1. **Seed** the polytope from the GJK simplex (re-evaluate support functions to recover individual A/B support points for contact interpolation).
2. **Promote triangle → tetrahedron** if GJK returned a degenerate simplex (defensive).
3. **Orient** the base triangle so its normal points away from vertex 3.
4. **Main loop** (up to 64 iterations):
   - Find closest face to origin (index-order scan, first-found wins ties).
   - Get Minkowski support in the face normal direction.
   - **Convergence test:** if `new_dist - closest_dist < epsilon`, extract result.
   - **Duplicate check:** skip if new vertex coincides with existing (within epsilon).
   - Classify faces as visible/non-visible from the new vertex.
   - Find the **horizon** (boundary between visible and non-visible faces).
   - Remove visible faces; create new faces from horizon edges + new vertex.
5. On convergence, compute **barycentric contact points** by projecting the origin onto
   the closest face, computing barycentric coordinates, and interpolating the tracked
   `support_a`/`support_b` points.

**Determinism guarantees:**
- Face selection: lowest index wins ties on distance.
- Horizon discovery: visible faces scanned in index order; edges within each face in fixed order.
- Barycentric computation: explicit `dot(xx+yy+zz)` evaluation order; no FMA.
- Degenerate fallback: centroid averaging when triangle area is zero.

### 4.3 Degenerate Simplex Recovery

**File:** `apc_collision_dispatch.h` — `detail::rebuild_simplex()`

When GJK produces a coplanar or collinear simplex (common with face-aligned overlaps),
EPA cannot form a valid tetrahedron. The dispatcher detects this via a volume check:

```cpp
static float simplex_volume(const Vec3 pts[4]) {
    Vec3 a = pts[3];
    return dot(cross(pts[0]-a, pts[1]-a), pts[2]-a);  // 6× signed volume
}
```

If `abs(volume) < APC_EPSILON`, the simplex is rebuilt from four tetrahedral directions:

```cpp
static const Vec3 dirs[4] = {
    { 1,  1,  1}, { 1, -1, -1},
    {-1,  1, -1}, {-1, -1,  1}
};
```

This guarantees a non-degenerate starting polytope for EPA.

---

## 5. Dedicated Fast-Path Detectors

Fast-path detectors bypass GJK/EPA for common shape pairs where a closed-form solution
is both faster and more numerically stable.

### 5.1 Sphere–Sphere (`detect_sphere_sphere`)

**File:** `apc_sphere_sphere.h`

Simplest detector. Computes center distance, compares against sum of radii.

```cpp
float dist = length(pos_b - pos_a);
if (dist > radius_a + radius_b) return false;  // separated
normal = (pos_a - pos_b) / dist;  // B->A
penetration = radius_sum - dist;
```

Degenerate case (coincident centers): normal defaults to `(0, 1, 0)`.

### 5.2 Sphere–Box (`detect_sphere_box`)

**File:** `apc_collision_dispatch.h`

Transforms sphere center into box local space, clamps to the box AABB to find the closest
surface point, then measures distance.

- **Outside case:** normal points from closest point toward sphere center.
- **Inside case:** finds the face with minimum penetration depth; tie-break order: X, Y, Z.

### 5.3 Sphere–Plane (`detect_sphere_plane`)

**File:** `apc_plane.h`

Computes signed distance from sphere center to the plane (positive = outside half-space).
Collides when distance < radius.

```cpp
float dist = dot(sphere_pos - plane_point, plane_normal);
if (dist > radius) return false;
penetration = radius - dist;
normal = plane_normal;  // B->A (plane toward sphere)
```

### 5.4 Sphere–Capsule (`detect_sphere_capsule`)

**File:** `apc_capsule.h`

Transforms sphere center into capsule local space, projects onto the capsule's Y-axis segment
(clamped to `[-half_height, +half_height]`), and measures distance from sphere center to
the closest axis point.

### 5.5 Capsule–Capsule (`detect_capsule_capsule`)

**File:** `apc_capsule.h`

Uses Ericson's segment closest-points algorithm (`detail::closest_points_segments()`)
to find minimum distance between the two capsule axis segments. Parallel segment fallback:
`s` defaults to `0.0f` (start of segment A).

### 5.6 Capsule–Plane (`detect_capsule_plane`)

**File:** `apc_capsule.h`

Computes signed distance of both capsule endpoints to the plane. The endpoint closer to
the plane determines contact. If `min(dist_top, dist_bot) - radius < 0`, the capsule penetrates.

### 5.7 Sphere–Cylinder (`detect_sphere_cylinder`)

**File:** `apc_cylinder.h`

Three-case handler based on sphere center location relative to the cylinder:

| Case | Condition | Closest Feature |
|------|-----------|-----------------|
| Inside | `y_inside && xz_inside` | Minimum of barrel/top-cap/bottom-cap penetration |
| Above/Below cap | `!y_inside && xz_inside` | Nearest cap face |
| Outside barrel | `xz_inside == false` | Barrel surface at clamped Y |

Tie-breaking for inside case: barrel > top cap > bottom cap.

### 5.8 Plane–Box (`detect_plane_box`)

**File:** `apc_collision_dispatch.h`

Transforms all 8 box corners to world space, computes signed distance of each to the plane,
and finds the most deeply penetrating corner.

### 5.9 Plane–Convex (`detect_plane_convex`)

**File:** `apc_collision_dispatch.h`

Same algorithm as plane-box but iterates over all convex piece vertices instead of 8 box corners.

---

## 6. Collision Dispatch

**File:** `apc_collision_dispatch.h`

### `dispatch_detect()` — Central Router

The dispatcher accepts two `CollisionShape` objects and body IDs, routes to the correct
detector, and populates a `ContactManifold`. It handles all **36 ordered pairs** of the 6 shape types.

**Normal convention:** all contacts have the normal pointing **B→A** (from body B toward body A,
i.e., pushing A away from B). When a detector is written for the opposite argument order,
`swap_contact()` negates the normal and swaps `point_on_a`/`point_on_b`:

```cpp
inline void swap_contact(ContactPoint& cp) {
    cp.normal = Vec3::scale(cp.normal, -1.0f);
    Vec3 tmp = cp.point_on_a;
    cp.point_on_a = cp.point_on_b;
    cp.point_on_b = tmp;
}
```

### Dispatch Routing Table

| Shape A | Shape B | Detector | Notes |
|---------|---------|----------|-------|
| Sphere | Sphere | `detect_sphere_sphere` | Direct |
| Sphere | Box | `detect_sphere_box` | Direct |
| Sphere | ConvexPiece | `detect_sphere_convex` | GJK+EPA |
| Sphere | Plane | `detect_sphere_plane` | Direct |
| Sphere | Capsule | `detect_sphere_capsule` | Direct |
| Sphere | Cylinder | `detect_sphere_cylinder` | Direct |
| Box | Box | `detect_convex_convex` | GJK+EPA |
| Box | ConvexPiece | `detect_convex_convex` | GJK+EPA |
| Box | Capsule | `gjk_epa_generic` | + `swap_contact` |
| Box | Cylinder | `gjk_epa_generic` | + `swap_contact` |
| Box | Plane | `detect_plane_box` | Direct |
| ConvexPiece | ConvexPiece | `detect_convex_convex` | GJK+EPA |
| ConvexPiece | Capsule | `gjk_epa_generic` | + `swap_contact` |
| ConvexPiece | Cylinder | `gjk_epa_generic` | + `swap_contact` |
| ConvexPiece | Plane | `detect_plane_convex` | Direct |
| Capsule | Capsule | `detect_capsule_capsule` | Direct |
| Capsule | Plane | `detect_capsule_plane` | Direct |
| Capsule | Cylinder | `gjk_epa_generic` | + `swap_contact` |
| Cylinder | Cylinder | `gjk_epa_generic` | Direct |
| Cylinder | Plane | `detect_capsule_plane` | Reused (cyl as capsule approximation not used; falls to GJK path) |

For reversed orderings (e.g., Box→Sphere), the dispatcher swaps arguments, calls the same detector,
and applies `swap_contact()`.

### Hull Construction Helpers

The `detail::` namespace provides factory functions that wrap any `CollisionShape` into a
`ConvexHull` for the generic GJK+EPA path:

| Helper | Shapes |
|--------|--------|
| `make_hull_from_shape()` | Box, ConvexPiece |
| `make_sphere_hull()` | Sphere |
| `make_capsule_hull()` | Capsule |
| `make_cylinder_hull()` | Cylinder |

Each allocates a stack-local shape data buffer (`BoxShapeData`, `CapsuleShapeData`, etc.)
that must outlive the returned `ConvexHull`. The buffers are populated from the
`CollisionShape`'s cached fields.

### Generic GJK+EPA Wrapper

`detail::gjk_epa_generic()` is the unified path for arbitrary convex-convex pairs:

```cpp
inline bool gjk_epa_generic(const ConvexHull& hull_a, const ConvexHull& hull_b,
                            ContactPoint& out_contact)
{
    GJKSimplex simplex;
    GJKResult gjk = GJKBoolean::query_with_simplex(hull_a, hull_b, simplex);
    if (!gjk.intersecting) return false;

    // Degenerate simplex recovery
    EPAResult epa;
    if (simplex.count >= 4 && abs(simplex_volume(simplex.points)) > APC_EPSILON)
        epa = EPA::query(simplex.points, simplex.count, hull_a, hull_b);
    else {
        Vec3 rebuilt[4];
        rebuild_simplex(hull_a, hull_b, rebuilt);
        epa = EPA::query(rebuilt, 4, hull_a, hull_b);
    }
    if (!epa.success) return false;

    // EPA normal is A->B; flip to B->A convention
    out_contact.normal = scale(epa.normal, -1.0f);
    out_contact.penetration = epa.penetration;
    out_contact.point_on_a = epa.point_on_a;
    out_contact.point_on_b = epa.point_on_b;
    return true;
}
```

---

## 7. Supported Shapes

APC supports **6 primitive shape types**, enumerated in `ShapeType`:

```cpp
enum class ShapeType : uint8_t {
    Sphere      = 0,   // radius
    Box         = 1,   // half_extents, orientation
    ConvexPiece = 2,   // vertex cloud from V-HACD decomposition
    Plane       = 3,   // point + normal (infinite half-space)
    Capsule     = 4,   // radius, half_height, axis along local Y
    Cylinder    = 5,   // radius, half_height, axis along local Y
    Count       = 6
};
```

### Shape Descriptions

| Shape | Parameters | Local-Space Geometry |
|-------|-----------|---------------------|
| **Sphere** | `radius` | Centered at origin, trivial rotation |
| **Box** | `half_extents (x,y,z)` | Centered at origin, axis-aligned in local space |
| **ConvexPiece** | `vertices[]`, `vertex_count` | Vertex cloud; no face connectivity needed (GJK/EPA use support functions only) |
| **Plane** | `point`, `normal` | Infinite half-space; normal points away from solid side |
| **Capsule** | `radius`, `half_height` | Y-axis cylinder from `-half_height` to `+half_height` with hemispherical caps |
| **Cylinder** | `radius`, `half_height` | Y-axis cylinder from `-half_height` to `+half_height` with flat caps |

### CollisionShape Tagged Union

All shapes are wrapped in `CollisionShape`, a tagged union that stores common transform data
(`position`, `orientation`, cached `rotation` matrix) alongside type-specific parameters.
Factory methods (`make_sphere()`, `make_box()`, etc.) ensure all unused fields are zeroed.

`get_aabb()` provides a broadphase AABB for every shape type. Planes use a large flat-box
approximation (±500 units along the tangent plane).

---

## 8. Contact Data

### ContactPoint

```cpp
struct ContactPoint {
    Vec3  point_on_a;   // Contact location on body A's surface
    Vec3  point_on_b;   // Contact location on body B's surface
    Vec3  normal;       // Contact normal pointing B->A
    float penetration;  // Depth of interpenetration (>= 0)
};
```

- **Normal convention:** always points from B toward A (pushes A away from B).
- **Contact points:** approximate surface locations where the two bodies touch.
  For GJK/EPA contacts, these are computed via barycentric interpolation over the closest face.
  For fast-path contacts, they are computed analytically.

### ContactManifold

```cpp
static constexpr uint32_t MAX_CONTACTS_PER_PAIR = 4u;

struct ContactManifold {
    uint32_t id_a;                                 // Body A identifier
    uint32_t id_b;                                 // Body B identifier (id_a < id_b)
    ContactPoint contacts[MAX_CONTACTS_PER_PAIR];
    uint32_t contact_count;                        // 0 = separated
};
```

### Persistent Contact Tracking

The `ContactManager` (in `apc_contact_manager.h`) tracks manifolds across frames for
solver warmstarting:

- **Contact matching:** new contacts are matched against previous-frame contacts by
  `point_on_a` proximity (threshold: 0.1 world units, first-found wins).
- **Impulse inheritance:** matched contacts inherit accumulated impulses scaled by
  `APC_WARMSTART_FACTOR` (0.8).
- **Manifold sorting:** insertion sort by `(id_a, id_b)` pair key after each update.
- **Capacity:** 256 concurrent pairs, 4 contacts per pair (fixed-size, no allocation).

---

## 9. Asset Pipeline

### V-HACD Convex Decomposition

Complex meshes (e.g., character limbs) are decomposed into convex pieces using
[V-HACD](https://github.com/kmammou/v-hacd). The CLI tool at `tools/vhacd_cli/main.cpp`
produces `.apccol` binary files.

### `.apccol` Binary Format

```
Offset  Size    Field
0       4       Magic: "APCC" (0x43435041 LE)
4       2       Version (uint16, must be 1)
6       2       Hull count (uint16)

Per hull:
+0      4       Vertex count (uint32)
+4      4       Triangle count (uint32)
+8      V×12    Vertices (3 × float32 per vertex: x, y, z)
+8+V×12 T×12    Triangle indices (3 × uint32 per triangle)
```

**Key detail:** triangle indices are **skipped during loading**. GJK/EPA operate exclusively on
vertex clouds via support functions — face connectivity is not needed for collision detection.

### AssetLoader API

**File:** `apc_asset_loader.h`

```cpp
struct AssetLoader {
    // Load from file
    static bool load_apccol(const char* filepath, ConvexAsset& out_asset);

    // Load from in-memory buffer
    static bool load_apccol_memory(const uint8_t* data, uint32_t size,
                                   ConvexAsset& out_asset);
};
```

Both methods populate a `ConvexAsset` containing a vector of `ConvexPiece` structs,
each holding a `std::vector<Vec3>` of vertices. The asset is then paired with an
`OBBTree` for midphase acceleration.

### Typical Usage

```cpp
ConvexAsset asset;
AssetLoader::load_apccol("character_arm.apccol", asset);

OBBTree tree;
tree.build(asset);          // Build BVH over convex pieces
tree.refit(position, rotation);  // Update to world space each frame

// Midphase: test individual pieces
for (int i = 0; i < tree.get_node_count(); ++i) {
    const auto& node = tree.get_node(i);
    if (node.hull_index < 0) continue;  // skip internal nodes
    if (!obb_intersect(node.box, query_obb)) continue;
    // Narrowphase: test this specific piece
    CollisionShape piece = CollisionShape::make_convex_piece(
        asset.get_piece(node.hull_index).vertices.data(),
        asset.get_piece(node.hull_index).vertices.size(),
        position, rotation);
    dispatch_detect(piece, other_shape, id_a, id_b, manifold);
}
```

---

## File Index

| File | Contents |
|------|----------|
| `apc_broadphase.h` | `BroadphaseSAP`, `AABB`, `BroadphasePair` |
| `apc_obb_tree.h` | `OBBTree`, `OBBTreeNode` |
| `apc_obb.h` | `OBB`, `obb_intersect()` (15-axis SAT) |
| `apc_gjk.h` | `GJKBoolean`, `ConvexHull`, `GJKSimplex`, `GJKResult` |
| `apc_epa.h` | `EPA`, `EPAResult` |
| `apc_support.h` | Support functions (`box_support`, `convex_piece_support`, `positioned_sphere_support`) |
| `apc_sphere_sphere.h` | `detect_sphere_sphere()`, `SphereCollider`, `ContactPoint` |
| `apc_plane.h` | `detect_sphere_plane()`, `PlaneCollider` |
| `apc_capsule.h` | `detect_sphere_capsule()`, `detect_capsule_capsule()`, `detect_capsule_plane()`, `capsule_support()`, `capsule_get_aabb()` |
| `apc_cylinder.h` | `detect_sphere_cylinder()`, `cylinder_support()`, `cylinder_get_aabb()` |
| `apc_collision_dispatch.h` | `dispatch_detect()`, `CollisionShape`, `ContactManifold`, `detect_sphere_box()`, `detect_plane_box()`, `detect_plane_convex()`, `detect_sphere_convex()`, `detect_convex_convex()`, `gjk_epa_generic()` |
| `apc_convex_asset.h` | `ConvexAsset`, `ConvexPiece` |
| `apc_asset_loader.h` | `AssetLoader` (file + memory loading) |
| `apc_contact_manager.h` | `ContactManager`, `PersistentManifold`, `PersistentContact` |
