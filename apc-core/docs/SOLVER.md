# Solver & Rigid Body Dynamics

This document covers the rigid body representation, sequential impulse solver, and contact persistence system in the Adaptive Physics Core (APC). All components are deterministic (no FMA, sorted iteration, index-order tie-breaking) and run at a fixed 240 Hz internal timestep.

**Source files:**
- `src/apc_solver/apc_rigid_body.h` — RigidBody struct
- `src/apc_solver/apc_si_solver_3d.h` — Solver3D class
- `src/apc_solver/apc_contact_manager.h` — ContactManager + PersistentManifold

---

## 1. Rigid Body (`apc_rigid_body.h`)

### Struct Definition

```cpp
struct RigidBody {
    Vec3  position;
    Vec3  linear_velocity;
    float inverse_mass;         // 0.0 = static (infinite mass)

    Quat  orientation;
    Vec3  angular_velocity;
    Mat3  local_inverse_inertia;
    Mat3  world_inverse_inertia;

    uint32_t collision_layer = 1;          // Which layers this body occupies
    uint32_t collision_mask  = 0xFFFFFFFF; // Which layers it collides with
};
```

APC stores inverse mass (not mass) so that impulse application becomes a single multiply, and static bodies are represented naturally as `inverse_mass == 0`.

### Static Bodies

A body is static when `inverse_mass == 0.0f`. The solver and integrator both skip static bodies entirely:

```cpp
// In integrate():
if (body.inverse_mass == 0.0f) continue;

// In apply_impulse():
linear_velocity += impulse * inverse_mass;  // 0 * anything = 0, no-op for statics
```

To create a static body, set both `inverse_mass` and `local_inverse_inertia` to zero:

```cpp
RigidBody ground;
ground.position         = Vec3(0, -5, 0);
ground.inverse_mass     = 0.0f;
ground.local_inverse_inertia = Mat3{{0,0,0, 0,0,0, 0,0,0}};
ground.world_inverse_inertia = Mat3{{0,0,0, 0,0,0, 0,0,0}};
ground.orientation      = Quat::identity();
```

### World Inertia Update

The inverse inertia tensor must be recomputed whenever the orientation changes. `update_world_inertia()` rotates the local-space tensor into world space:

```cpp
void update_world_inertia() {
    Mat3 rot     = Mat3::from_quat(orientation);
    Mat3 inv_rot = rot.transpose();
    world_inverse_inertia = rot * local_inverse_inertia * inv_rot;
}
```

This is called during `integrate()` after the orientation quaternion is updated. It must also be called once at body creation if the initial orientation is not identity.

### Impulse Application

`apply_impulse` modifies both linear and angular velocity using manual `Mat3 * Vec3` arithmetic (no operator overloads) to guarantee deterministic results across compilers:

```cpp
void apply_impulse(const Vec3& impulse, const Vec3& contact_point) {
    // Linear: delta_v = impulse / mass
    linear_velocity += impulse * inverse_mass;

    // Angular: delta_w = I^-1 * (r x impulse)
    Vec3 r = contact_point - position;
    Vec3 torque_impulse = cross(r, impulse);

    // Manual matrix-vector multiply (deterministic, no FMA)
    const Mat3& m = world_inverse_inertia;
    Vec3 angular_impulse(
        torque_impulse.x * m.m[0] + torque_impulse.y * m.m[3] + torque_impulse.z * m.m[6],
        torque_impulse.x * m.m[1] + torque_impulse.y * m.m[4] + torque_impulse.z * m.m[7],
        torque_impulse.x * m.m[2] + torque_impulse.y * m.m[5] + torque_impulse.z * m.m[8]
    );

    angular_velocity += angular_impulse;
}
```

### Collision Filtering

Collision filtering uses a bitmask layer/mask system:

```cpp
static bool should_collide(const RigidBody& a, const RigidBody& b) {
    return (a.collision_layer & b.collision_mask) != 0 &&
           (b.collision_layer & a.collision_mask) != 0;
}
```

A body's `collision_layer` declares which layers it belongs to; `collision_mask` declares which layers it can collide with. The default (`layer=1, mask=0xFFFFFFFF`) makes every body collide with every other body.

Typical setup for a three-layer system:

```cpp
// Player collides with everything
player.collision_layer = 0x01;       // Layer 0
player.collision_mask  = 0xFFFFFFFF;

// Projectile collides with environment and enemies, not other projectiles
projectile.collision_layer = 0x02;   // Layer 1
projectile.collision_mask  = 0x01 | 0x04; // Layers 0 and 2

// Debris only collides with environment
debris.collision_layer = 0x04;       // Layer 2
debris.collision_mask  = 0x04;       // Layer 2 only (self-collision)
```

---

## 2. Sequential Impulse Solver (`apc_si_solver_3d.h`)

### Overview

The `Solver3D` class implements a sequential impulse (SI) solver — the same approach used in Box2D and Bullet Physics. It iterates over contact constraints, computing and clamping impulse magnitudes until the relative velocities converge toward the contact conditions.

### Configurable Parameters

| Parameter | Default | Description |
|---|---|---|
| `friction_coefficient` | 0.4 | Coulomb friction coefficient. 0.0 = frictionless, 1.0 = high grip. |
| `baumgarte_factor` | 0.2 | Position correction strength (0–1). Higher = more aggressive penetration resolution. |
| `baumgarte_slop` | 0.005 | Penetration threshold before Baumgarte correction activates. Prevents jitter for near-contact. |
| `restitution` | 0.0 | Coefficient of restitution. 0.0 = perfectly inelastic, 1.0 = perfectly elastic. |
| `velocity_iterations` | 8 | Number of SI iterations per solve() call. More iterations = better convergence, more cost. |
| `linear_damping` | 0.999 | Per-frame multiplier applied to linear velocity after integration. |
| `angular_damping` | 0.998 | Per-frame multiplier applied to angular velocity after integration. |

### VelocityConstraint

Each contact point is converted into a `VelocityConstraint`:

```cpp
struct VelocityConstraint {
    uint32_t id_a, id_b;
    Vec3 normal;
    Vec3 contact_point;
    Vec3 r_a, r_b;               // Vectors from body centers to contact point
    float penetration;

    float normal_mass;            // Effective mass along contact normal
    float accumulated_normal_impulse;      // Clamped >= 0

    float friction_mass;          // Effective mass along tangent direction
    Vec3  accumulated_friction_impulse;
};
```

### Constraint Preparation

`prepare()` builds a single constraint from a `ContactPoint`. It computes:
- Lever arms `r_a` and `r_b` from body centers to the contact point
- Effective normal mass using the standard formula:

```
K = (1/m_a + 1/m_b) + (I_a^-1 * (r_a x n)) . (r_a x n)
                             + (I_b^-1 * (r_b x n)) . (r_b x n)
normal_mass = 1 / K
```

`prepare_manifold()` wraps `prepare()` to handle multi-contact manifolds (up to `MAX_CONTACTS_PER_PAIR = 4` contacts per pair):

```cpp
void prepare_manifold(const ContactPoint* contacts, uint32_t count,
                      uint32_t id_a, uint32_t id_b,
                      const std::vector<RigidBody>& bodies) {
    for (uint32_t i = 0; i < count; ++i) {
        prepare(contacts[i], id_a, id_b, bodies);
    }
}
```

### The Solve Loop

`solve()` runs `velocity_iterations` passes over all accumulated constraints. Each pass applies two impulse stages:

**1. Normal impulse with Baumgarte stabilization:**

```cpp
// Relative velocity at contact
float vel_along_normal = dot(rel_vel, normal);

// Positional correction bias (Baumgarte)
float positional_error = max(penetration - baumgarte_slop, 0.0f) * baumgarte_factor / dt;

// Restitution: only bounce when approaching and shallowly penetrating
float restitution_bias = 0.0f;
if (restitution > 0.0f && vel_along_normal < -EPSILON && penetration < baumgarte_slop * 2.0f) {
    restitution_bias = -restitution * vel_along_normal;
}

// Compute delta and clamp accumulated impulse >= 0
float delta = normal_mass * (-vel_along_normal + positional_error + restitution_bias);
float new_impulse = max(accumulated_normal_impulse + delta, 0.0f);
delta = new_impulse - accumulated_normal_impulse;
```

The restitution guard (`penetration < baumgarte_slop * 2.0f`) prevents energy explosions when objects are deeply interpenetrating — bounce is only applied during shallow contact.

**2. Coulomb friction (tangential impulse):**

After applying the normal impulse, the solver recomputes relative velocity and extracts the tangential component:

```cpp
Vec3 vel_tangent = rel_vel - normal * dot(rel_vel, normal);

if (length_sq(vel_tangent) > EPSILON_SQ) {
    Vec3 tangent = normalize(vel_tangent);
    float delta_friction = friction_mass * length(vel_tangent);

    // Coulomb cone clamp: |friction| <= mu * |normal_impulse|
    float max_friction = friction_coefficient * accumulated_normal_impulse;
    float new_friction = min(delta_friction + length(accumulated_friction_impulse),
                             max_friction);
    float applied = new_friction - length(accumulated_friction_impulse);

    if (applied > 0.0f) {
        Vec3 friction_impulse = tangent * (-applied);
        // Apply to both bodies (equal and opposite)
    }
}
```

### Integration

`integrate()` uses semi-implicit Euler: velocities are updated first (during solve), then positions are advanced:

```cpp
void integrate(std::vector<RigidBody>& bodies, float dt) {
    for (auto& body : bodies) {
        if (body.inverse_mass == 0.0f) continue;  // Skip statics

        // Position update
        body.position += body.linear_velocity * dt;

        // Orientation update via quaternion integration
        Vec3 half_ang = body.angular_velocity * (dt * 0.5f);
        Quat delta_q(half_ang.x, half_ang.y, half_ang.z, 0.0f);
        body.orientation = normalize(delta_q * body.orientation);

        // Recompute world-space inertia tensor
        body.update_world_inertia();

        // Per-frame damping
        body.linear_velocity  *= linear_damping;
        body.angular_velocity *= angular_damping;
    }
}
```

### Clearing Constraints

Call `clear()` at the start of each frame before preparing new constraints. Constraints do not persist across frames — persistence is handled by `ContactManager` (see Section 3).

---

## 3. Contact Persistence & Warmstarting (`apc_contact_manager.h`)

### Why Warmstarting?

A cold-start solver (all impulses = 0) typically needs 8–12 iterations to converge. With warmstarting, the accumulated impulses from the previous frame provide a near-correct initial guess, reducing the iterations needed for convergence by 2–3x.

### Key Constants

| Constant | Value | Purpose |
|---|---|---|
| `APC_WARMSTART_FACTOR` | 0.8 | Scale applied to inherited impulses (0.0 = cold, 1.0 = full) |
| `APC_CONTACT_MATCH_DISTANCE` | 0.1 | World units; contacts closer than this are the "same" contact |
| `MAX_PAIRS` | 256 | Maximum tracked body pairs |
| `MAX_CONTACTS_PER_PAIR` | 4 | Contacts per pair manifold |

### PersistentManifold

Each body pair has a `PersistentManifold` that tracks up to 4 contact points across frames:

```cpp
struct PersistentManifold {
    uint32_t id_a, id_b;   // Normalized: id_a <= id_b
    PersistentContact contacts[MAX_CONTACTS_PER_PAIR];
    uint32_t contact_count;
};
```

**`add_or_update(contact, threshold)`** matches a new contact against existing ones by `point_on_a` proximity. On match:
- Positions are averaged (`lerp(0.5)`)
- Normal is replaced with the latest value from collision detection
- Penetration is `max(old, new)`
- Accumulated impulses are **preserved** (not scaled here — the caller/ContactManager handles scaling)
- Age is incremented

On no match (new contact): zero impulses, age 0.

**`prune(max_distance)`** removes outlier contacts whose `point_on_a` is too far from the manifold centroid. This cleans up stale contacts when geometry shifts.

### ContactManager

`ContactManager` manages all persistent manifolds across the simulation:

```cpp
class ContactManager {
    static constexpr uint32_t MAX_PAIRS = 256u;

    void update(const ContactManifold* new_manifolds, uint32_t new_count);
    const PersistentManifold* get_persistent(uint32_t id_a, uint32_t id_b) const;
    PersistentManifold* get_persistent_mut(uint32_t id_a, uint32_t id_b);
    bool store_impulse(uint32_t id_a, uint32_t id_b,
                       uint32_t contact_index,
                       float normal_impulse, const Vec3& friction_impulse);
    void clear();
    uint32_t pair_count() const;
};
```

### The `update()` Algorithm

1. **Normalize pair IDs** — swap so `lo <= hi` for deterministic lookup
2. **Match** each new manifold against existing persistent manifolds
3. **Match contacts** within a manifold by proximity. Matched contacts inherit impulses scaled by `APC_WARMSTART_FACTOR`:
   ```cpp
   out.accumulated_normal_impulse = old.accumulated_normal_impulse * 0.8f;
   out.accumulated_friction_impulse = scale(old.accumulated_friction_impulse, 0.8f);
   ```
4. **Remove** persistent manifolds not matched by any new manifold (pair separated)
5. **Sort** remaining manifolds by pair key using insertion sort (stable, O(N) for nearly-sorted data, no allocation)

### Impulse Storage

After the solver finishes, call `store_impulse()` to write back the accumulated impulses from `VelocityConstraint` into the persistent manifold. These become the warmstart values for the next frame:

```cpp
contact_manager.store_impulse(id_a, id_b, contact_index,
                              constraint.accumulated_normal_impulse,
                              constraint.accumulated_friction_impulse);
```

---

## 4. Typical Simulation Loop

Below is a complete frame step showing how all components integrate. This matches the pattern used in the integration tests (`test_solver_v2.cpp`, `test_friction.cpp`).

### Initialization (once)

```cpp
enforce_deterministic_fp_mode();

// Create bodies
std::vector<RigidBody> bodies;
bodies.push_back(make_dynamic_body(0, Vec3(0, 5, 0), /* mass */ 1.0f));
bodies.push_back(make_static_body(1, Vec3(0, -1, 0)));

// Create collision shapes and broadphase proxies
std::vector<CollisionShape> shapes = { /* ... */ };
std::vector<BroadphaseSAP::Proxy> proxies = { /* ... */ };

// Create solver and contact manager
Solver3D solver;
solver.friction_coefficient = 0.4f;
solver.velocity_iterations  = 8;
solver.restitution          = 0.2f;

ContactManager contact_manager;
```

### Per-Frame Step

```cpp
const float dt = 1.0f / 240.0f;

// --- 1. Broadphase ---
broadphase.update(proxies);
broadphase.generate_pairs();
const auto& pairs = broadphase.get_potential_pairs();

// --- 2. Narrowphase collision detection ---
ContactManifold manifolds[256];
uint32_t manifold_count = 0;

for (const auto& pair : pairs) {
    // Collision filtering
    if (!RigidBody::should_collide(bodies[pair.id_a], bodies[pair.id_b]))
        continue;

    // Update shape positions from body state
    shapes[pair.id_a].position = bodies[pair.id_a].position;
    shapes[pair.id_b].position = bodies[pair.id_b].position;

    // Dispatch to correct narrowphase detector
    ContactManifold& m = manifolds[manifold_count++];
    if (dispatch_detect(shapes[pair.id_a], shapes[pair.id_b],
                        pair.id_a, pair.id_b, m)) {
        // m now contains up to 4 ContactPoints
    }
}

// --- 3. Contact manager: match persistent contacts, warmstart ---
contact_manager.update(manifolds, manifold_count);

// --- 4. Prepare solver constraints from persistent manifolds ---
solver.clear();
for (uint32_t i = 0; i < contact_manager.pair_count(); ++i) {
    const PersistentManifold* pm = contact_manager.manifolds();
    for (uint32_t c = 0; c < pm->contact_count; ++c) {
        // Prepare constraint and seed with warmstarted impulses
        solver.prepare(pm->contacts[c].contact, pm->id_a, pm->id_b, bodies);

        // Apply warmstart: set accumulated impulses from persistent contact
        // (constraints is private — either use prepare_manifold or seed
        //  the VelocityConstraint after prepare returns)
    }
}

// --- 5. Solve ---
solver.solve(bodies, dt);

// --- 6. Store impulses back for next-frame warmstart ---
// (Iterate constraints and call contact_manager.store_impulse for each)

// --- 7. Integrate ---
solver.integrate(bodies, dt);

// --- 8. Update broadphase proxies from new body positions ---
for (uint32_t i = 0; i < bodies.size(); ++i) {
    if (bodies[i].inverse_mass == 0.0f) continue;
    proxies[i].aabb = shapes[i].get_aabb();
}
```

### Helper: Creating a Dynamic Sphere Body

```cpp
RigidBody make_sphere_body(const Vec3& pos, float radius, float mass) {
    RigidBody b;
    b.position       = pos;
    b.linear_velocity = Vec3(0, 0, 0);
    b.angular_velocity = Vec3(0, 0, 0);
    b.inverse_mass   = (mass > 0.0f) ? (1.0f / mass) : 0.0f;
    b.orientation    = Quat::identity();

    // Solid sphere: I = 2/5 * m * r^2
    float I     = 0.4f * mass * radius * radius;
    float inv_I = (I > 0.0f) ? (1.0f / I) : 0.0f;
    b.local_inverse_inertia = Mat3{{inv_I,0,0, 0,inv_I,0, 0,0,inv_I}};

    b.update_world_inertia();
    return b;
}
```

---

## Tuning Guide

### Common Scenarios

**Stacking stability** — increase `velocity_iterations` to 12–16 and decrease `baumgarte_slop` to 0.001. This reduces jitter in tall stacks at the cost of CPU time.

**Bouncy objects** — set `restitution` between 0.3–0.8. Keep `baumgarte_slop * 2.0` larger than expected penetration depth to avoid the restitution guard suppressing bounces.

**Ice-like sliding** — set `friction_coefficient` to 0.05 or lower. The solver will allow near-free tangential motion at contacts.

**Heavy damping (underwater)** — set `linear_damping` to 0.95–0.98 and `angular_damping` to 0.93–0.96. These are per-frame multipliers applied after each integrate step.

### Determinism Checklist

- Always call `enforce_deterministic_fp_mode()` before any simulation work
- Compile with `-ffp-contract=off` (GCC/Clang) or `/fp:strict` (MSVC) — handled by `APCDeterminism.cmake`
- Sort bodies by ID; never iterate by pointer order
- Use `ContactManager` for deterministic warmstarting (insertion sort, index-order matching)
- `apply_impulse()` uses manual `Mat3 * Vec3` — do not replace with operator overloads unless they avoid FMA
