# APC API Reference

> **Adaptive Physics Core** — C++17 header-only deterministic physics engine.
> All public symbols live in the `apc::` namespace.

---

## Module: `apc_math`

Core linear-algebra primitives and constants. All arithmetic uses explicit
evaluation order (no FMA, no SIMD) to guarantee bitwise cross-platform
determinism.

### Constants (`apc_math_common.h`)

```cpp
constexpr float APC_EPSILON      = 1.0e-6f;
constexpr float APC_EPSILON_SQ   = APC_EPSILON * APC_EPSILON;  // 1.0e-12f
constexpr float APC_PI           = 3.14159265358979323846f;
constexpr float APC_TWO_PI       = APC_PI * 2.0f;
constexpr float APC_HALF_PI      = APC_PI * 0.5f;
```

```cpp
#define APC_FORCEINLINE   // __forceinline on MSVC, __attribute__((always_inline)) inline on GCC/Clang
```

### `Vec3` — 3-component vector (`apc_vec3.h`)

```cpp
struct Vec3 {
    float x, y, z;

    Vec3();                          // zero-initialised (0,0,0)
    Vec3(float x_, float y_, float z_);
    explicit Vec3(float s);          // broadcast scalar
};
```

All operations are **static named methods** (no operator overloads on the
simulation hot-path) to make FP evaluation order explicit and compiler-proof.

**Static methods**

| Method | Signature | Description |
|--------|-----------|-------------|
| `add` | `static Vec3 add(const Vec3& a, const Vec3& b)` | Component-wise addition |
| `sub` | `static Vec3 sub(const Vec3& a, const Vec3& b)` | Component-wise subtraction |
| `scale` | `static Vec3 scale(const Vec3& v, float s)` | Scalar multiply |
| `scaled_add` | `static Vec3 scaled_add(const Vec3& a, const Vec3& b, float s)` | `a + b*s` — avoids intermediate |
| `mul_comp` | `static Vec3 mul_comp(const Vec3& a, const Vec3& b)` | Component-wise multiply (inertia tensor transforms) |
| `dot` | `static float dot(const Vec3& a, const Vec3& b)` | Dot product; explicit `(xx+yy+zz)` order |
| `length_sq` | `static float length_sq(const Vec3& v)` | Squared length |
| `cross` | `static Vec3 cross(const Vec3& a, const Vec3& b)` | Cross product |
| `length` | `static float length(const Vec3& v)` | Euclidean length via `std::sqrt` |
| `normalize` | `static Vec3 normalize(const Vec3& v)` | Returns zero vector if `len_sq < APC_EPSILON_SQ` |
| `safe_normalize` | `static Vec3 safe_normalize(const Vec3& v, const Vec3& fallback)` | Returns `fallback` on degenerate input |
| `lerp` | `static Vec3 lerp(const Vec3& a, const Vec3& b, float t)` | Linear interpolation |

**Instance methods**

| Method | Signature | Description |
|--------|-----------|-------------|
| `equals_exact` | `bool equals_exact(const Vec3& other) const` | Bitwise `memcmp` comparison |
| `equals_approx` | `bool equals_approx(const Vec3& other, float eps = APC_EPSILON) const` | Per-component tolerance |
| `hash` | `uint32_t hash() const` | Deterministic hash for replay tools |

**Operators** *(only available when `APC_SIM_HOT_PATH` is NOT defined)*

```cpp
bool  operator==(const Vec3& a, const Vec3& b);
bool  operator!=(const Vec3& a, const Vec3& b);
Vec3  operator+(const Vec3& a, const Vec3& b);
Vec3  operator-(const Vec3& a, const Vec3& b);
Vec3  operator*(const Vec3& v, float s);
Vec3  operator*(float s, const Vec3& v);
```

**`std::hash<Vec3>` specialisation** — uses boost-style hash combining for
`unordered_set`/`unordered_map` use in tests and tools.

### `Quat` — Quaternion (`apc_quat.h`)

```cpp
struct Quat {
    float x, y, z, w;

    Quat();                         // identity (0,0,0,1)
    Quat(float x_, float y_, float z_, float w_);
};
```

| Method | Signature | Description |
|--------|-----------|-------------|
| `identity` | `static Quat identity()` | Returns identity quaternion |
| `from_axis_angle` | `static Quat from_axis_angle(const Vec3& axis, float angle)` | Axis-angle conversion |
| `from_rotation_matrix` | `static Quat from_rotation_matrix(const Mat3& m)` | Extract quaternion from rotation matrix |
| `multiply` | `static Quat multiply(const Quat& a, const Quat& b)` | Hamilton product; strict `x,y,z,w` ordering |
| `normalize` | `static Quat normalize(const Quat& q)` | Safe normalisation (returns identity on degenerate) |
| `inverse` | `static Quat inverse(const Quat& q)` | Conjugate for unit quaternions (`-x,-y,-z,w`) |
| `to_mat3` | `Mat3 to_mat3() const` | Convert to 3x3 rotation matrix |
| `rotate` | `Vec3 rotate(const Vec3& v) const` | Rotate a vector by this quaternion |

### `Mat3` — 3x3 column-major matrix (`apc_mat3.h`)

```cpp
struct Mat3 {
    float m[9];   // column-major: columns 0,1,2 stored at m[0..2], m[3..5], m[6..8]
};
```

| Method | Signature | Description |
|--------|-----------|-------------|
| `identity` | `static Mat3 identity()` | Identity matrix |
| `from_quat` | `static Mat3 from_quat(const Quat& q)` | Quaternion-to-matrix conversion |
| `multiply` | `static Mat3 multiply(const Mat3& a, const Mat3& b)` | Matrix multiplication |
| `add` | `static Mat3 add(const Mat3& a, const Mat3& b)` | Element-wise addition |
| `transpose` | `Mat3 transpose() const` | Matrix transpose |
| `inverse` | `Mat3 inverse() const` | 3x3 matrix inverse |
| `diagonal` | `Vec3 diagonal() const` | Extract diagonal elements (`m[0], m[4], m[8]`) |
| `transform_vec` | `Vec3 transform_vec(const Vec3& v) const` | Matrix-vector product `M*v` with explicit FP order |

---

## Module: `apc_collision`

Broadphase, narrow-phase detection, and collision asset loading.
Supports six primitive types: **Sphere**, **Box**, **ConvexPiece**, **Plane**,
**Capsule**, **Cylinder** — yielding 36 possible pair combinations, all routed
through `dispatch_detect()`.

### Contact Point & Sphere (`apc_sphere_sphere.h`)

```cpp
struct ContactPoint {
    Vec3  point_on_a;    // closest surface point on body A
    Vec3  point_on_b;    // closest surface point on body B
    Vec3  normal;        // B→A convention everywhere except sphere_sphere
    float penetration;   // depth of overlap (≥ 0)
};

struct SphereCollider {
    float radius;
};
```

```cpp
bool detect_sphere_sphere(
    const Vec3& pos_a, const SphereCollider& col_a,
    const Vec3& pos_b, const SphereCollider& col_b,
    ContactPoint& out_contact);
```

> **Normal convention note:** `detect_sphere_sphere` uses B→A (from sphere B
> toward sphere A). All other detectors in the engine use B→A as well, making
> the solver interface uniform.

### GJK Boolean (`apc_gjk.h`)

Gilbert-Johnson-Keerthi intersection test with Minkowski-difference support.

```cpp
struct ConvexHull {
    using SupportFunc = Vec3 (*)(const void* shape, const Vec3& dir, uint32_t& out_vertex_id);
    const void* user_data;
    SupportFunc support;
};

struct GJKResult {
    bool intersecting;
    Vec3 separation_vector;   // valid only when !intersecting
};

struct GJKSimplex {
    Vec3 points[4];
    int  count = 0;
    void add(const Vec3& p);
};

class GJKBoolean {
public:
    static constexpr uint32_t MAX_ITERATIONS = 32;

    static GJKResult query(const ConvexHull& hull_a, const ConvexHull& hull_b);
    static GJKResult query_with_simplex(const ConvexHull& hull_a,
                                        const ConvexHull& hull_b,
                                        GJKSimplex& simplex_out);
};
```

`query_with_simplex` populates `simplex_out` on intersection for EPA seeding.
Uses deterministic initial direction `(1,0,0)` and index-ordered tie-breaking.

### EPA — Expanding Polytope Algorithm (`apc_epa.h`)

Computes penetration depth, contact normal, and approximate contact points from
a GJK simplex enclosing the origin.

```cpp
struct EPAResult {
    bool   success;        // false on degenerate geometry or convergence failure
    Vec3   normal;         // A→B normal convention
    float  penetration;    // ≥ 0 on success
    Vec3   point_on_a;     // approximate deepest contact point on A
    Vec3   point_on_b;     // approximate deepest contact point on B
};

class EPA {
public:
    static constexpr uint32_t MAX_ITERATIONS = 64;
    static constexpr uint32_t MAX_VERTICES   = 64;
    static constexpr uint32_t MAX_FACES      = 192;

    static EPAResult query(const Vec3 simplex_points[4], int simplex_count,
                           const ConvexHull& hull_a, const ConvexHull& hull_b);
};
```

Face selection uses stable index-based tie-breaking (lowest index wins).
Contact points are computed via barycentric interpolation of tracked A/B
support points.

### Broadphase — Sweep-and-Prune (`apc_broadphase.h`)

```cpp
struct AABB {
    Vec3 min, max;
};

struct BroadphasePair {
    uint32_t id_a, id_b;   // always id_a < id_b
    bool operator<(const BroadphasePair&) const;   // strict total order
    bool operator==(const BroadphasePair&) const;
};

class BroadphaseSAP {
public:
    struct Proxy { uint32_t id; AABB aabb; };

    using FilterFunc = bool (*)(uint32_t id_a, uint32_t id_b, void* user_data);
    void*       filter_user_data = nullptr;
    FilterFunc  filter_func      = nullptr;

    void update(const std::vector<Proxy>& proxies);
    void generate_pairs();
    const std::vector<BroadphasePair>& get_potential_pairs() const;
};
```

Endpoints are sorted with `std::stable_sort` for deterministic pair output.
Active set uses a stack-allocated array (max 512 entities). Y/Z overlap is
tested before emitting a pair. Output pairs are stably sorted by `(id_a, id_b)`.

### OBB & SAT (`apc_obb.h`)

```cpp
struct OBB {
    Vec3 center, extents;   // extents = half-lengths
    Quat orientation;
    Mat3 rotation;          // cached from orientation

    void update_cache();
    Vec3 local_to_world(const Vec3& local) const;
    Vec3 world_to_local_dir(const Vec3& world) const;
};

bool obb_intersect(const OBB& a, const OBB& b);   // 15-axis SAT
```

Tests all 15 separating axes in strict order (3 face normals from A, 3 from B,
9 edge-edge cross products). No heuristic early exits.

### OBB Tree (`apc_obb_tree.h`)

Binary bounding-volume hierarchy over `ConvexAsset` pieces, built with
top-down median splits on the longest axis.

```cpp
struct OBBTreeNode {
    OBB  box;
    int  left_child, right_child;   // child indices
    int  hull_index;                // ≥ 0 for leaves, -1 for internal nodes
};

class OBBTree {
public:
    void build(const ConvexAsset& asset);
    void refit(const Vec3& position, const Quat& rotation);
    int  get_root() const;
    const OBBTreeNode& get_node(int index) const;
    int  get_node_count() const;
    bool query_overlap(const OBB& query) const;
};
```

Traversal order is deterministic: left child before right child. Split axis
tie-breaking prefers X > Y > Z.

### Convex Asset (`apc_convex_asset.h`)

```cpp
struct ConvexPiece {
    std::vector<Vec3> vertices;     // vertex cloud (no triangle connectivity)
};

struct ConvexAsset {
    std::vector<ConvexPiece> pieces;
    int get_piece_count() const;
    const ConvexPiece& get_piece(int index) const;
};
```

### Shape Data & Support Functions (`apc_support.h`)

Each shape type has a data struct (cast to `void*` for `ConvexHull::user_data`)
and a support function matching the `ConvexHull::SupportFunc` signature.

```cpp
struct SphereShapeData { float radius; };

struct BoxShapeData {
    Vec3 half_extents;
    Vec3 position;
    Quat orientation;
    Mat3 rotation;
    void update_cache();
};

struct ConvexPieceShapeData {
    const Vec3* vertices;
    uint32_t    vertex_count;
    Vec3        position;
    Quat        orientation;
    Mat3        rotation;
    void update_cache();
};

struct PositionedSphereData { float radius; Vec3 position; };
```

```cpp
Vec3 sphere_support(const void* user_data, const Vec3& dir, uint32_t& out_vertex_id);
Vec3 box_support(const void* user_data, const Vec3& dir, uint32_t& out_vertex_id);
Vec3 convex_piece_support(const void* user_data, const Vec3& dir, uint32_t& out_vertex_id);
Vec3 positioned_sphere_support(const void* user_data, const Vec3& dir, uint32_t& out_vertex_id);
```

Tie-breaking: index-order for `convex_piece_support`, lexicographic sign bits
for `box_support`.

### Plane (`apc_plane.h`)

```cpp
struct PlaneCollider {
    Vec3 point;     // any point on the plane
    Vec3 normal;    // outward normal (away from solid side)
};

bool detect_sphere_plane(const Vec3& pos, const SphereCollider& col,
                         const PlaneCollider& plane, ContactPoint& out_contact);

struct PlaneShapeData { Vec3 normal; Vec3 point; float extent; };

Vec3 plane_support(const void* user_data, const Vec3& dir, uint32_t& out_vertex_id);
```

### Capsule (`apc_capsule.h`)

Local-space axis is Y. Total height = `2 * half_height + 2 * radius`.

```cpp
struct CapsuleShapeData {
    float radius, half_height;
    Vec3  position;
    Quat  orientation;
    Mat3  rotation;
    void update_cache();
};

Vec3 capsule_support(const void* user_data, const Vec3& dir, uint32_t& out_vertex_id);
AABB capsule_get_aabb(const CapsuleShapeData& capsule);

bool detect_sphere_capsule(const Vec3& sphere_pos, float sphere_radius,
                           const CapsuleShapeData& capsule,
                           ContactPoint& out_contact);

bool detect_capsule_capsule(const CapsuleShapeData& cap_a,
                            const CapsuleShapeData& cap_b,
                            ContactPoint& out_contact);

bool detect_capsule_plane(const CapsuleShapeData& capsule,
                          const Vec3& plane_point, const Vec3& plane_normal,
                          ContactPoint& out_contact);
```

### Cylinder (`apc_cylinder.h`)

Local-space axis is Y, extending from `-half_height` to `+half_height`.

```cpp
struct CylinderShapeData {
    float radius, half_height;
    Vec3  position;
    Quat  orientation;
    Mat3  rotation;
    void update_cache();
};

Vec3 cylinder_support(const void* user_data, const Vec3& dir, uint32_t& out_vertex_id);
AABB cylinder_get_aabb(const CylinderShapeData& cyl);

bool detect_sphere_cylinder(const Vec3& sphere_pos, float sphere_radius,
                            const CylinderShapeData& cylinder,
                            ContactPoint& out_contact);
```

### Collision Shape & Dispatch (`apc_collision_dispatch.h`)

```cpp
enum class ShapeType : uint8_t {
    Sphere = 0, Box = 1, ConvexPiece = 2, Plane = 3, Capsule = 4, Cylinder = 5, Count = 6
};

struct CollisionShape {
    ShapeType type;
    Vec3  position, orientation (as Quat), rotation (as Mat3);
    // type-specific fields: sphere_radius, box_half_extents, convex_vertices,
    //   convex_vertex_count, plane_normal, capsule_radius, capsule_half_height,
    //   cylinder_radius, cylinder_half_height

    void update_cache();
    AABB get_aabb() const;

    static CollisionShape make_sphere(float radius, const Vec3& pos);
    static CollisionShape make_box(const Vec3& half_extents, const Vec3& pos, const Quat& orient);
    static CollisionShape make_convex_piece(const Vec3* vertices, uint32_t count,
                                           const Vec3& pos, const Quat& orient);
    static CollisionShape make_plane(const Vec3& point, const Vec3& normal);
    static CollisionShape make_capsule(float radius, float half_height,
                                       const Vec3& pos, const Quat& orient);
    static CollisionShape make_cylinder(float radius, float half_height,
                                        const Vec3& pos, const Quat& orient);
};
```

```cpp
struct ContactManifold {
    uint32_t id_a, id_b;
    ContactPoint contacts[4];   // MAX_CONTACTS_PER_PAIR
    uint32_t    contact_count;

    void reset();
    void add_contact(const ContactPoint& cp);
};
```

**Free-function detectors**

```cpp
bool detect_sphere_box(const Vec3& sphere_pos, float sphere_radius,
                       const Vec3& box_pos, const Vec3& box_half_extents,
                       const Mat3& box_rot, ContactPoint& out_contact);

void swap_contact(ContactPoint& cp);   // negate normal, swap point_on_a/b

bool detect_sphere_convex(const CollisionShape& sphere, const CollisionShape& convex,
                          ContactPoint& out_contact);

bool detect_convex_convex(const CollisionShape& a, const CollisionShape& b,
                          ContactPoint& out_contact);

bool detect_plane_box(const Vec3& plane_point, const Vec3& plane_normal,
                      const Vec3& box_pos, const Vec3& box_half_extents,
                      const Mat3& box_rot, ContactPoint& out_contact);

bool detect_plane_convex(const Vec3& plane_point, const Vec3& plane_normal,
                         const Vec3& convex_pos, const Mat3& convex_rot,
                         const Vec3* convex_vertices, uint32_t convex_vertex_count,
                         ContactPoint& out_contact);
```

**Central dispatcher** — routes all 36 shape-type pairs:

```cpp
bool dispatch_detect(const CollisionShape& shape_a, const CollisionShape& shape_b,
                     uint32_t id_a, uint32_t id_b,
                     ContactManifold& out_manifold);
```

All contacts produced by `dispatch_detect` use **B→A** normal convention.

### Asset Loader (`apc_asset_loader.h`)

```cpp
struct AssetLoader {
    static constexpr uint32_t MAGIC = 0x43435041u;   // "APCC" little-endian

    static bool load_apccol(const char* filepath, ConvexAsset& out_asset);
    static bool load_apccol_memory(const uint8_t* data, uint32_t size,
                                   ConvexAsset& out_asset);
};
```

Loads V-HACD `.apccol` binary format (header: magic + version + hull count,
per hull: vertex count + triangle count + vertex data). Triangle indices are
skipped — GJK/EPA operate on vertex clouds only.

---

## Module: `apc_solver`

Rigid-body dynamics and sequential-impulse constraint solver with warmstarting.

### Rigid Body (`apc_rigid_body.h`)

```cpp
struct RigidBody {
    Vec3  position, linear_velocity;
    float inverse_mass;
    Quat  orientation;
    Vec3  angular_velocity;
    Mat3  local_inverse_inertia, world_inverse_inertia;

    uint32_t collision_layer = 1;           // which layer(s) this body is ON
    uint32_t collision_mask  = 0xFFFFFFFF;  // which layer(s) it collides WITH

    static bool should_collide(const RigidBody& a, const RigidBody& b);
    void update_world_inertia();
    void apply_impulse(const Vec3& impulse, const Vec3& contact_point);
};
```

`apply_impulse` updates both linear and angular velocity using the world-space
inverse inertia tensor.

### Velocity Constraint & Solver (`apc_si_solver_3d.h`)

```cpp
struct VelocityConstraint {
    uint32_t id_a, id_b;
    Vec3  normal, contact_point, r_a, r_b;
    float penetration, normal_mass, friction_mass;
    float accumulated_normal_impulse;
    Vec3  accumulated_friction_impulse;
};
```

```cpp
class Solver3D {
public:
    // Tunable parameters
    float  friction_coefficient = 0.4f;
    float  baumgarte_factor     = 0.2f;
    float  baumgarte_slop       = 0.005f;
    float  restitution          = 0.0f;
    uint32_t velocity_iterations = 8;
    float  linear_damping       = 0.999f;
    float  angular_damping      = 0.998f;

    void prepare(const ContactPoint& contact, uint32_t id_a, uint32_t id_b,
                 const std::vector<RigidBody>& bodies);
    void prepare_manifold(const ContactPoint* contacts, uint32_t count,
                          uint32_t id_a, uint32_t id_b,
                          const std::vector<RigidBody>& bodies);
    void solve(std::vector<RigidBody>& bodies, float dt);
    void integrate(std::vector<RigidBody>& bodies, float dt);
    void clear();
};
```

`solve()` runs sequential impulse iterations with Baumgarte position
correction and Coulomb friction cone clamping. `integrate()` applies
semi-implicit Euler integration with per-frame damping.

### Contact Manager (`apc_contact_manager.h`)

Persistent contact tracking for solver warmstarting. Matches contacts across
frames by proximity and carries forward accumulated impulses.

```cpp
static constexpr float APC_WARMSTART_FACTOR        = 0.8f;
static constexpr float APC_CONTACT_MATCH_DISTANCE  = 0.1f;

struct PersistentContact {
    uint32_t     contact_id;
    ContactPoint contact;
    float        accumulated_normal_impulse;
    Vec3         accumulated_friction_impulse;
    uint32_t     age;
    void reset();
};

struct PersistentManifold {
    uint32_t id_a, id_b;
    PersistentContact contacts[4];
    uint32_t contact_count, next_contact_id;
    void reset();
    void add_or_update(const ContactPoint& new_contact, float distance_threshold);
    void prune(float max_distance);
};

class ContactManager {
public:
    static constexpr uint32_t MAX_PAIRS = 256;

    void update(const ContactManifold* new_manifolds, uint32_t new_count);
    const PersistentManifold* get_persistent(uint32_t id_a, uint32_t id_b) const;
    PersistentManifold*       get_persistent_mut(uint32_t id_a, uint32_t id_b);
    void clear();
    uint32_t pair_count() const;
    const PersistentManifold* manifolds() const;
    PersistentManifold*       manifolds_mut();
    bool store_impulse(uint32_t id_a, uint32_t id_b, uint32_t contact_index,
                       float normal_impulse, const Vec3& friction_impulse);
};
```

Manifolds are kept sorted by `(id_a, id_b)` using insertion sort after each
`update()`. Contact matching uses index-order first-match-wins scanning.

---

## Module: `apc_skeleton`

Articulated body dynamics for skeletal character physics, supporting animation-
driven, physics-driven, and blended blend modes.

### Types (`apc_skeleton_types.h`)

```cpp
struct Transform {
    Vec3 translation;
    Quat rotation;
    static Transform identity();
    static Transform multiply(const Transform& parent, const Transform& local);
};

enum class PhysicsBlendMode {
    ANIM_DRIVEN,      // 0% physics
    PHYSICS_DRIVEN,   // 100% ragdoll
    BLENDED           // spring-damper back to animation target
};

struct BonePhysicsState {
    PhysicsBlendMode mode    = PhysicsBlendMode::PHYSICS_DRIVEN;
    float stiffness          = 0.0f;
    float damping            = 0.0f;
    float max_deviation      = 0.0f;
};

struct Bone {
    uint32_t parent_index;
    Transform bind_pose;
    Vec3      joint_to_com;
    Mat3      local_inverse_inertia;
    float     inverse_mass;
    BonePhysicsState physics;
};

struct SkeletalAsset {
    std::vector<Bone> bones;
    uint32_t get_bone_count() const;
    bool is_root(uint32_t bone_index) const;
};
```

### Pose (`apc_skeletal_pose.h`)

```cpp
struct SkeletalAnimTarget {
    std::vector<Transform> target_local_transforms;
    std::vector<Vec3>      target_world_coms;
    std::vector<float>     target_joint_accelerations;
    bool has_target = false;
    void allocate(uint32_t bone_count);
};

struct SkeletalPose {
    std::vector<Transform> local_transforms, world_transforms;
    SkeletalAnimTarget target;
    void allocate(uint32_t bone_count);
    void set_to_bind_pose(const SkeletalAsset& asset);
};
```

### Forward Kinematics (`apc_skeletal_fk.h`)

```cpp
class SkeletalFK {
public:
    static void calculate_world_transforms(const SkeletalAsset& asset,
                                           const SkeletalPose& local_pose,
                                           SkeletalPose& out_world_pose);
    static Vec3  get_world_com(const SkeletalAsset& asset,
                               const SkeletalPose& world_pose,
                               uint32_t bone_index);
};
```

Iterates strictly in array order (0 to N); parent index must be less than
child index.

### Articulated Body (`apc_skeleton_apc.h`)

ABA (Articulated Body Algorithm) forward-dynamics stepper for single-DOF
joints.

```cpp
struct SkeletalDynamicState {
    std::vector<float> joint_velocities;
    std::vector<Vec3>  world_omegas, world_v_coms, a_coms, alphas;
    void allocate(uint32_t bone_count);
};

class ArticulatedBody {
public:
    static void step(const SkeletalAsset& asset,
                     const std::vector<Vec3>& local_joint_axes,
                     SkeletalPose& pose,
                     SkeletalDynamicState& state,
                     float dt, const Vec3& gravity);
};
```

Performs forward pass (velocity propagation), backward pass (composite
inertia/bias force accumulation), acceleration computation, and semi-implicit
Euler integration of joint angles.

---

## Module: `apc_containers`

Deterministic data structures used throughout the engine. `std::unordered_map`
is **banned** — all associative lookups use `FlatMap` instead.

### `FlatMap<TKey, TValue>` (`apc_flat_map.h`)

Sorted-vector map with O(log N) lookup via `std::lower_bound`. Iteration order
is strictly sorted by key.

```cpp
template<typename TKey, typename TValue>
class FlatMap {
public:
    TValue*       find(const TKey& key);
    const TValue* find(const TKey& key) const;
    void          insert(const TKey& key, const TValue& value);  // upsert
    size_t        size() const;
    bool          empty() const;
    // begin(), end() — sorted iteration
};
```

### `SlotMap<TEntityId, TValue>` (`apc_slot_map.h`)

Deterministic sparse set. Iteration order matches insertion order. Uses
`FlatMap` internally for the sparse lookup table.

```cpp
template<typename TEntityId, typename TValue>
class SlotMap {
public:
    TValue&              insert(TEntityId id, const TValue& value);
    void                 erase(TEntityId id);         // swap-and-pop
    bool                 contains(TEntityId id) const;
    TValue*              get(TEntityId id);
    const std::vector<TEntityId>& get_ids() const;     // insertion-order IDs
    size_t               size() const;
};
```

---

## Module: `apc_platform`

Cross-platform FPU control for enforcing deterministic floating-point behaviour.

```cpp
struct FPUState {
    uint64_t raw_state;
    bool     is_valid;
};

struct FPCapabilities {
    bool supports_flush_denormals_to_zero;   // DAZ
    bool supports_denormals_are_zero;        // FTZ
    bool supports_strict_rounding;           // round-to-nearest-even
    bool has_fma_that_breaks_determinism;    // FMA fused ops
};
```

```cpp
FPUState      enforce_deterministic_fp_mode();          // call once at startup
bool          verify_fp_mode(const FPUState& expected);  // debug integrity check
void          restore_fp_mode(const FPUState& state);    // before DLL boundary
FPCapabilities query_fp_capabilities();                  // informational
```

Supported platforms: x86-64 (SSE/AVX control), ARM64 (FPCR), and consoles
(`APC_PLATFORM_CONSOLE`). `enforce_deterministic_fp_mode()` crashes if the
required mode cannot be achieved.
