# APC Shape Support and Extension Guide

This guide covers the collision shape system in the Adaptive Physics Core (APC), a C++17
deterministic physics engine. It documents the six built-in shape types, explains how the
support-function abstraction drives GJK/EPA, describes the AABB broadphase strategy,
and provides a step-by-step walkthrough for adding a new shape.

---

## 1. Overview of Shape Types

APC supports six collision shape types, declared in the `ShapeType` enum inside
`src/apc_collision/apc_collision_dispatch.h`:

```cpp
enum class ShapeType : uint8_t {
    Sphere      = 0,   // radius only — fastest collision path
    Box         = 1,   // half_extents + orientation — GJK-compatible
    ConvexPiece = 2,   // vertex cloud from V-HACD decomposition — GJK/EPA
    Plane       = 3,   // infinite half-space (point + normal)
    Capsule     = 4,   // radius + half_height, axis along local Y
    Cylinder    = 5,   // radius + half_height, axis along local Y
    Count       = 6    // sentinel — always equal to the number of shapes
};
```

| Shape | Description | Primary Detection |
|-------|-------------|-------------------|
| Sphere | Single radius value. No orientation. | Analytical fast-path |
| Box | Oriented bounding box via half_extents and quaternion. | GJK/EPA (convex-convex) |
| ConvexPiece | Arbitrary convex hull from mesh decomposition (V-HACD). Non-owning vertex pointer. | GJK/EPA |
| Plane | Infinite half-space defined by a point on the plane and an outward normal. | Analytical fast-path |
| Capsule | Cylinder with hemispherical caps. Axis is local Y. | Mixed: sphere/capsule/plane fast-paths; GJK/EPA for box/convex/cylinder |
| Cylinder | Solid cylinder (flat caps). Axis is local Y. | Mixed: sphere fast-path; GJK/EPA for most other pairs |

---

## 2. Shape Data Structs

Each shape type has a corresponding data struct that is passed as `user_data` to GJK
support functions. Oriented shapes all cache their `Mat3 rotation` via `update_cache()`.

### SphereShapeData

```cpp
struct SphereShapeData {
    float radius;
};
```

### BoxShapeData

```cpp
struct BoxShapeData {
    Vec3 half_extents;  // half-widths (x, y, z)
    Vec3 position;
    Quat orientation;
    Mat3 rotation;      // cached via update_cache()

    void update_cache() { rotation = Mat3::from_quat(orientation); }
};
```

### ConvexPieceShapeData

```cpp
struct ConvexPieceShapeData {
    const Vec3* vertices;   // non-owning pointer to vertex array
    uint32_t vertex_count;
    Vec3 position;
    Quat orientation;
    Mat3 rotation;

    void update_cache() { rotation = Mat3::from_quat(orientation); }
};
```

### PositionedSphereData

Used by the dispatcher to wrap a sphere with world position for GJK queries:

```cpp
struct PositionedSphereData {
    float radius;
    Vec3  position;
};
```

### PlaneShapeData

```cpp
struct PlaneShapeData {
    Vec3  normal;   // outward normal
    Vec3  point;    // any point on the plane
    float extent;   // half-extent for box approximation (e.g. 1000.0f)
};
```

### CapsuleShapeData

```cpp
struct CapsuleShapeData {
    float radius;          // hemispherical cap + cylindrical body radius
    float half_height;     // half the distance between the two sphere centers
    Vec3  position;        // center of the capsule
    Quat  orientation;     // world orientation (local Y = capsule axis)
    Mat3  rotation;

    void update_cache() { rotation = Mat3::from_quat(orientation); }
};
```

### CylinderShapeData

```cpp
struct CylinderShapeData {
    float radius;
    float half_height;     // half height along local Y axis
    Vec3  position;
    Quat  orientation;     // local Y = cylinder axis
    Mat3  rotation;

    void update_cache() { rotation = Mat3::from_quat(orientation); }
};
```

---

## 3. CollisionShape — The Unified Interface

`CollisionShape` is a tagged union that wraps all shape types into a single object.
Defined in `apc_collision_dispatch.h`, it stores the common fields (`type`, `position`,
`orientation`, `rotation`) alongside all type-specific fields.

### Factory Methods

```cpp
auto sphere = CollisionShape::make_sphere(1.0f, Vec3(0, 5, 0));
auto box    = CollisionShape::make_box(
    Vec3(1, 2, 3), Vec3(0, 0, 0), Quat::identity());
auto convex = CollisionShape::make_convex_piece(
    vertices, count, Vec3(0, 0, 0), Quat::identity());
auto plane  = CollisionShape::make_plane(
    Vec3(0, 0, 0), Vec3(0, 1, 0));
auto cap    = CollisionShape::make_capsule(
    0.3f, 0.5f, Vec3(0, 2, 0), Quat::identity());
auto cyl    = CollisionShape::make_cylinder(
    0.5f, 1.0f, Vec3(0, 1, 0), Quat::identity());
```

### AABB Broadphase

Call `shape.get_aabb()` to get an axis-aligned bounding box for the SAP broadphase.
The implementation switches on `type` and delegates to the appropriate strategy
(see Section 5).

### Dispatch Table

`dispatch_detect(shape_a, shape_b, id_a, id_b, manifold)` is the central routing
function. It handles all 36 ordered pairs (6 x 6) by selecting either a dedicated
analytical detector or the generic GJK+EPA fallback. All contacts use the B->A
normal convention (normal points from body B toward body A).

---

## 4. Support Functions

The GJK and EPA algorithms operate on shapes through the **support function**
abstraction. A support function returns the farthest point on a shape boundary in
a given direction:

```cpp
using SupportFunc = Vec3 (*)(const void* user_data, const Vec3& dir,
                             uint32_t& out_vertex_id);
```

APC provides one support function per shape:

| Function | Defined In | Key Algorithm |
|----------|-----------|---------------|
| `sphere_support` | `apc_support.h` | Normalize direction, scale by radius |
| `box_support` | `apc_support.h` | Sign of local-space direction selects corner |
| `convex_piece_support` | `apc_support.h` | Linear scan over all vertices |
| `positioned_sphere_support` | `apc_support.h` | Same as sphere, offset by world position |
| `capsule_support` | `apc_capsule.h` | Minkowski sum of line segment + sphere |
| `cylinder_support` | `apc_cylinder.h` | Axis sign + radial XZ projection |
| `plane_support` | `apc_plane.h` | Two-sided box approximation |

### Determinism Requirement

Tie-breaking must be deterministic — when multiple vertices have equal dot products,
the result must be identical across platforms and runs. APC uses **first-index-wins**:
- **Box**: lexicographic sign combination (`>= 0` selects the positive face).
- **ConvexPiece**: lowest vertex index wins (linear scan with strict `>` comparison).
- **Capsule/Cylinder**: `>= 0` tie-breaking on axis sign, with deterministic
  fallbacks for degenerate directions.

### Example: Box Support Function

```cpp
inline Vec3 box_support(const void* user_data, const Vec3& dir,
                        uint32_t& out_vertex_id) {
    const BoxShapeData* data = static_cast<const BoxShapeData*>(user_data);
    Vec3 local_dir = data->rotation.transpose().transform_vec(dir);

    // Select corner by sign of each component
    float sx = (local_dir.x >= 0.0f) ?  data->half_extents.x : -data->half_extents.x;
    float sy = (local_dir.y >= 0.0f) ?  data->half_extents.y : -data->half_extents.y;
    float sz = (local_dir.z >= 0.0f) ?  data->half_extents.z : -data->half_extents.z;

    // Encode vertex deterministically: 3-bit id
    out_vertex_id = ((local_dir.x >= 0.0f) ? 1u : 0u) |
                    ((local_dir.y >= 0.0f) ? 2u : 0u) |
                    ((local_dir.z >= 0.0f) ? 4u : 0u);

    Vec3 local_point(sx, sy, sz);
    return Vec3::add(data->position, data->rotation.transform_vec(local_point));
}
```

---

## 5. AABB Computation

Each shape type uses a different strategy to produce its broadphase AABB:

**Sphere** — Trivial: `center +/- radius` on each axis.

**Box** — Evaluate all 8 transformed corners and take min/max.

**ConvexPiece** — Linear scan over all transformed vertices.

**Plane** — Approximate as a flat box with 500-unit extent perpendicular to the normal
and 0.1-unit thickness along the normal. The dominant normal axis is detected to orient
the box.

**Capsule** — Tight AABB via 6-axis support evaluation. Calls `capsule_support()` for
each of the six axis-aligned directions (`+X`, `-X`, `+Y`, `-Y`, `+Z`, `-Z`) and
takes the bounding box of the resulting points.

**Cylinder** — Analytical abs-sum bound. Decomposes the rotation matrix into its column
vectors (axis = column 1 for local Y), then computes:
```
extent_i = |axis[i]| * half_height + (|perp_0[i]| + |perp_2[i]|) * radius
```
This is conservative (`|a| + |b| >= sqrt(a^2 + b^2)`) but avoids expensive square roots.

---

## 6. Dedicated Fast-Paths vs GJK/EPA

Some shape pairs have hand-written analytical detectors that are faster and more robust
than the generic GJK+EPA pipeline:

### Analytical Fast-Path Pairs

| Pair | Detector Function | Location |
|------|-------------------|----------|
| sphere-sphere | `detect_sphere_sphere` | `apc_sphere_sphere.h` |
| sphere-box | `detect_sphere_box` | `apc_collision_dispatch.h` |
| sphere-plane | `detect_sphere_plane` | `apc_plane.h` |
| sphere-capsule | `detect_sphere_capsule` | `apc_capsule.h` |
| capsule-capsule | `detect_capsule_capsule` | `apc_capsule.h` |
| capsule-plane | `detect_capsule_plane` | `apc_capsule.h` |
| sphere-cylinder | `detect_sphere_cylinder` | `apc_cylinder.h` |
| plane-box | `detect_plane_box` | `apc_collision_dispatch.h` |
| plane-convex | `detect_plane_convex` | `apc_collision_dispatch.h` |

### GJK/EPA Fallback Pairs

All remaining pairs use `detail::gjk_epa_generic()`:

| Pair | Notes |
|------|-------|
| box-box | Direct GJK+EPA |
| box-convex | Direct GJK+EPA |
| convex-convex | Direct GJK+EPA |
| capsule-box | GJK+EPA with `swap_contact` |
| capsule-convex | GJK+EPA with `swap_contact` |
| capsule-cylinder | GJK+EPA with `swap_contact` |
| cylinder-box | GJK+EPA with `swap_contact` |
| cylinder-convex | GJK+EPA with `swap_contact` |
| cylinder-cylinder | GJK+EPA |
| cylinder-plane | GJK+EPA |
| plane-capsule | `detect_capsule_plane` with swapped args + `swap_contact` |
| plane-cylinder | GJK+EPA |

The GJK pipeline runs up to 32 iterations. If the resulting simplex is degenerate
(coplanar or collinear), the dispatcher rebuilds a non-degenerate simplex from 4
tetrahedral directions before running EPA.

---

## 7. How to Add a New Shape (Step-by-Step)

Follow these steps to integrate a new convex shape (e.g. "Cone") into APC. This
example assumes the shape is convex and can participate in GJK/EPA.

### Step 1: Create the Shape Header

Create `src/apc_collision/apc_cone.h`:

```cpp
#pragma once
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include "apc_math/apc_mat3.h"
#include "apc_math/apc_math_common.h"
#include "apc_sphere_sphere.h"   // ContactPoint
#include "apc_broadphase.h"      // AABB
#include "apc_gjk.h"             // ConvexHull::SupportFunc
#include <cstdint>
#include <cmath>

namespace apc {

struct ConeShapeData {
    float radius;          // base radius
    float half_height;     // half height along local Y (tip at +Y, base at -Y)
    Vec3  position;
    Quat  orientation;
    Mat3  rotation;

    void update_cache() { rotation = Mat3::from_quat(orientation); }
};

// GJK-compatible support function
inline Vec3 cone_support(const void* user_data, const Vec3& dir,
                         uint32_t& out_vertex_id) {
    const ConeShapeData* data = static_cast<const ConeShapeData*>(user_data);
    Vec3 local_dir = data->rotation.transpose().transform_vec(dir);

    // Tip at +Y, base circle at -Y
    float sy = (local_dir.y >= 0.0f) ? data->half_height : -data->half_height;

    // Radial component on base circle
    float xz_len_sq = local_dir.x * local_dir.x + local_dir.z * local_dir.z;
    float rx, rz;
    if (xz_len_sq > APC_EPSILON_SQ) {
        float inv_xz_len = 1.0f / std::sqrt(xz_len_sq);
        rx = local_dir.x * inv_xz_len * data->radius;
        rz = local_dir.z * inv_xz_len * data->radius;
    } else {
        // Degenerate: deterministic fallback
        rx = data->radius;
        rz = 0.0f;
    }

    out_vertex_id = (local_dir.y >= 0.0f) ? 1u : 0u;

    Vec3 local_point(rx, sy, rz);
    return Vec3::add(data->position, data->rotation.transform_vec(local_point));
}

// AABB for broadphase (6-axis support evaluation — tight for convex shapes)
inline AABB cone_get_aabb(const ConeShapeData& cone) {
    static const Vec3 axes[6] = {
        Vec3( 1, 0, 0), Vec3(-1, 0, 0),
        Vec3( 0, 1, 0), Vec3( 0,-1, 0),
        Vec3( 0, 0, 1), Vec3( 0, 0,-1)
    };
    uint32_t discard = 0u;
    Vec3 p = cone_support(&cone, axes[0], discard);
    AABB aabb = { p, p };
    for (int i = 1; i < 6; ++i) {
        p = cone_support(&cone, axes[i], discard);
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
```

### Step 2: Add to the ShapeType Enum

In `apc_collision_dispatch.h`, update the enum:

```cpp
enum class ShapeType : uint8_t {
    Sphere      = 0,
    Box         = 1,
    ConvexPiece = 2,
    Plane       = 3,
    Capsule     = 4,
    Cylinder    = 5,
    Cone        = 6,   // <-- NEW
    Count       = 7    // <-- increment
};
```

### Step 3: Add Fields to CollisionShape

Add type-specific fields to the tagged union:

```cpp
float cone_radius;       // Cone only
float cone_half_height;  // Cone only
```

### Step 4: Add Factory Method

```cpp
static CollisionShape make_cone(float radius, float half_height,
                                const Vec3& pos, const Quat& orient) {
    CollisionShape s;
    s.type = ShapeType::Cone;
    s.position = pos;
    s.orientation = orient;
    s.rotation = Mat3::from_quat(orient);
    s.cone_radius = radius;
    s.cone_half_height = half_height;
    // Zero-initialize all other fields...
    return s;
}
```

### Step 5: Add AABB Case

In `get_aabb()`, add a case for `ShapeType::Cone` that constructs a `ConeShapeData`,
calls `update_cache()`, and delegates to `cone_get_aabb()`.

### Step 6: Wire the Dispatch Table

In `apc_collision_dispatch.h`:

1. `#include "apc_cone.h"`
2. Add a `make_cone_hull()` helper in the `detail` namespace (analogous to
   `make_capsule_hull`).
3. For fast-path pairs (e.g. `detect_sphere_cone`), add dedicated branches.
4. For all other pairs, use `detail::gjk_epa_generic()` with `swap_contact()`
   when argument order is reversed.
5. Handle **both orderings**: `Cone-X` and `X-Cone`. For example:
   ```cpp
   // CONE – BOX
   else if (ta == ShapeType::Cone && tb == ShapeType::Box) {
       ConeShapeData cone_buf;
       ConvexHull hull_a = detail::make_cone_hull(shape_a, cone_buf);
       BoxShapeData box_buf;
       ConvexPieceShapeData convex_buf;
       ConvexHull hull_b = detail::make_hull_from_shape(shape_b, box_buf, convex_buf);
       hit = detail::gjk_epa_generic(hull_a, hull_b, contact);
   }
   // BOX – CONE  (swapped order)
   else if (ta == ShapeType::Box && tb == ShapeType::Cone) {
       ConeShapeData cone_buf;
       ConvexHull hull_a = detail::make_cone_hull(shape_b, cone_buf);
       BoxShapeData box_buf;
       ConvexPieceShapeData convex_buf;
       ConvexHull hull_b = detail::make_hull_from_shape(shape_a, box_buf, convex_buf);
       hit = detail::gjk_epa_generic(hull_a, hull_b, contact);
       if (hit) { swap_contact(contact); }
   }
   ```

### Step 7: Write Tests

Create `tests/correctness/test_cone.cpp` covering:

- **Support function**: test multiple directions (axis-aligned, diagonal, negative)
  and verify the returned point lies on the expected surface feature.
- **AABB computation**: verify the AABB encloses known extreme points.
- **Factory**: verify `make_cone()` sets all fields correctly.
- **Dispatch routing**: test at least one collision pair (e.g. cone-box) through
  `dispatch_detect()` and verify a contact is generated with valid normal direction.
- **Normal convention**: confirm all contacts use B->A normal convention.

---

## 8. Normal Convention

All detectors in APC use the **B->A normal convention**: the contact normal points
from body B toward body A. This means the normal pushes A away from B.

```cpp
// The solver applies an impulse proportional to:
//   J = penetration * normal    (applied to A, pushing A away from B)
//  -J = penetration * -normal   (applied to B, pushing B away from A)
```

When the dispatcher swaps argument order to match a detector (e.g. calling
`detect_sphere_box` with the box as A and sphere as B), it uses `swap_contact()`
to restore the correct convention:

```cpp
inline void swap_contact(ContactPoint& cp) {
    cp.normal = Vec3::scale(cp.normal, -1.0f);
    Vec3 tmp = cp.point_on_a;
    cp.point_on_a = cp.point_on_b;
    cp.point_on_b = tmp;
}
```

This negates the normal and swaps `point_on_a`/`point_on_b`. Every new detector
must follow this convention. If your detector is written as "A vs B" where A is the
first argument and B is the second, the normal must point from B toward A.

---

## Summary

| Concept | Key File |
|---------|----------|
| Shape types + CollisionShape + dispatch table | `apc_collision_dispatch.h` |
| Shape data structs + sphere/box/convex support | `apc_support.h` |
| Capsule support + fast-path detectors | `apc_capsule.h` |
| Cylinder support + sphere-cylinder detector | `apc_cylinder.h` |
| Plane support + sphere-plane detector | `apc_plane.h` |
| GJK boolean query + ConvexHull type | `apc_gjk.h` |
| EPA penetration depth | `apc_epa.h` |
| AABB + SAP broadphase | `apc_broadphase.h` |
| ContactPoint struct | `apc_sphere_sphere.h` |
