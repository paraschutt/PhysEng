// ============================================================================
// DSPE World Implementation
// ============================================================================
#include "dspe/world.h"
#include <algorithm>
#include <cstdio>

namespace dspe {

// ============================================================================
// Construction
// ============================================================================
World::World(const WorldConfig& config)
    : solver_(config.solver), surface_(config.surface) {
    manifolds_.reserve(400);
    trigger_system_.init_pitch_triggers(entities_, config.pitch);

    // Create implicit ground plane: a large static box at y = -0.5m (half-extent 0.5m)
    // so its top face is exactly at y = 0. Ball resting on ground touches at ball_radius.
    create_static_box(
        { FpPos::zero(), FpPos::from_float(-0.5f), FpPos::zero() },
        { FpPos::from_float(200.0f), FpPos::from_float(0.5f), FpPos::from_float(200.0f) },
        MAT_DRY_GRASS
    );
}

// ============================================================================
// Entity factories
// ============================================================================
EntityId World::create_player(uint8_t player_index, Vec3Pos start_pos, bool /*team_b*/) {
    EntityId id = PLAYER_BEGIN + player_index;
    if (id >= PLAYER_END) return INVALID_ENTITY;

    Entity& e = entities_[id];
    e.component_mask = ARCH_PLAYER;

    // RigidBody
    RigidBody& rb    = e.rigidbody;
    rb.position      = start_pos;
    rb.orientation   = QuatVel::identity();
    rb.mass          = PLAYER_MASS;
    rb.inv_mass      = FpVel::one() / PLAYER_MASS;
    rb.drag_cd       = FpVel::from_float(1.0f);
    // Cross section: πr² ≈ π * 0.18² ≈ 0.1 m²
    rb.cross_section_area = FpVel::from_float(0.1f);
    // Inertia (cylinder approximation): I = 0.5 * m * r²
    FpVel I_inv = FpVel::one() / (PLAYER_MASS * FpVel::from_float(0.5f)
                                 * FpVel::from_float(0.18f * 0.18f));
    rb.inv_inertia_x = I_inv;
    rb.inv_inertia_y = I_inv;
    rb.inv_inertia_z = I_inv;

    // Collider: single capsule proxy for the full body
    e.collider.shape_type    = ShapeType::CAPSULE;
    e.collider.material_id   = MAT_DRY_GRASS;
    e.collider.skeleton_owner= id;
    // Capsule: base at feet, tip at head (~1.8m tall, 0.18m radius)
    e.collider.capsule.local_base = Vec3Pos::zero();
    e.collider.capsule.local_tip  = { FpPos::zero(),
                                       FpPos::from_float(1.62f), // 1.8 - 2*0.09
                                       FpPos::zero() };
    e.collider.capsule.radius = FpPos::from_float(0.18f);

    // Skeleton
    e.skeleton.entity_id = id;
    for (uint8_t b = 0; b < BONE_COUNT; ++b) {
        e.skeleton.bones[b].joint_angle  = FpVel::zero();
        e.skeleton.bones[b].target_angle = FpVel::zero();
    }

    // Register in broad phase
    AABB aabb{
        Vec3Pos{start_pos.x - FpPos::from_float(0.2f),
                start_pos.y,
                start_pos.z - FpPos::from_float(0.2f)},
        Vec3Pos{start_pos.x + FpPos::from_float(0.2f),
                FpPos{start_pos.y.raw + FpPos::from_float(1.9f).raw},
                start_pos.z + FpPos::from_float(0.2f)}
    };
    broad_phase_.insert(id, aabb);

    return id;
}

EntityId World::create_ball(Vec3Pos start_pos) {
    EntityId id = BALL_ENTITY;
    Entity& e   = entities_[id];
    e.component_mask = ARCH_BALL;

    // RigidBody
    RigidBody& rb    = e.rigidbody;
    rb.position      = start_pos;
    rb.orientation   = QuatVel::identity();
    rb.mass          = BALL_MASS;
    rb.inv_mass      = FpVel::one() / BALL_MASS;
    rb.drag_cd       = FpVel::from_float(0.25f);
    FpVel r2 = BALL_RADIUS * BALL_RADIUS;
    rb.cross_section_area = r2 * FpVel::from_float(3.14159f); // π*r²
    // Sphere inertia: I = 2/5 * m * r²
    FpVel I_ball = BALL_MASS * r2 * FpVel::from_float(0.4f);
    FpVel I_ball_inv = FpVel::one() / I_ball;
    rb.inv_inertia_x = I_ball_inv;
    rb.inv_inertia_y = I_ball_inv;
    rb.inv_inertia_z = I_ball_inv;

    // Collider: sphere
    e.collider.shape_type  = ShapeType::SPHERE;
    e.collider.material_id = MAT_DRY_GRASS;
    e.collider.sphere.radius = FpPos{BALL_RADIUS.raw >> 8}; // Q15.16 -> Q24.8

    // Ball properties
    e.ball = BallProperties::make_default();

    // Register in broad phase
    FpPos r = e.collider.sphere.radius;
    AABB aabb{
        { start_pos.x - r, start_pos.y - r, start_pos.z - r },
        { start_pos.x + r, start_pos.y + r, start_pos.z + r }
    };
    broad_phase_.insert(id, aabb);

    return id;
}

EntityId World::create_static_box(Vec3Pos centre, Vec3Pos half_extents,
                                   MaterialId mat) {
    // Find free ENV slot
    EntityId id = INVALID_ENTITY;
    for (EntityId eid = ENV_BEGIN; eid < ENV_END; ++eid) {
        if (entities_[eid].component_mask == COMP_NONE) { id = eid; break; }
    }
    if (id == INVALID_ENTITY) return INVALID_ENTITY;

    Entity& e = entities_[id];
    e.component_mask = ARCH_STATIC;

    // RigidBody (static: inv_mass = 0)
    e.rigidbody.position  = centre;
    e.rigidbody.mass      = FpVel::max_val();
    e.rigidbody.inv_mass  = FpVel::zero();

    // Collider: box
    e.collider.shape_type  = ShapeType::BOX;
    e.collider.material_id = mat;
    e.collider.box.half_extents = half_extents;

    // AABB
    AABB aabb{
        { centre.x - half_extents.x, centre.y - half_extents.y, centre.z - half_extents.z },
        { centre.x + half_extents.x, centre.y + half_extents.y, centre.z + half_extents.z }
    };
    e.collider.world_aabb = aabb;
    broad_phase_.insert(id, aabb);

    return id;
}

// ============================================================================
// AABB tree update
// ============================================================================
void World::update_aabb_tree() {
    for (EntityId id = 0; id < MAX_ENTITIES; ++id) {
        Entity& e = entities_[id];
        if (!e.has(COMP_COLLIDER | COMP_RIGIDBODY)) continue;
        if (e.has(COMP_STATIC))   continue;
        if (e.has(COMP_TRIGGER))  continue;

        Vec3Pos pos = e.rigidbody.position;
        AABB tight;
        if (e.collider.shape_type == ShapeType::SPHERE) {
            FpPos r = e.collider.sphere.radius;
            tight = {
                Vec3Pos{pos.x - r, pos.y - r, pos.z - r},
                Vec3Pos{pos.x + r, pos.y + r, pos.z + r}
            };
        } else if (e.collider.shape_type == ShapeType::CAPSULE) {
            FpPos r   = e.collider.capsule.radius;
            FpPos len{e.collider.capsule.local_tip.y.raw -
                      e.collider.capsule.local_base.y.raw};
            tight = {
                Vec3Pos{pos.x - r, pos.y - r, pos.z - r},
                Vec3Pos{pos.x + r, FpPos{pos.y.raw + len.raw + r.raw}, pos.z + r}
            };
        } else {
            tight = e.collider.world_aabb;
        }
        e.collider.world_aabb = tight;
        broad_phase_.update(id, tight);
    }
}

// ============================================================================
// Narrow phase contact generation
// ============================================================================
void World::generate_contacts() {
    // Broad phase candidate pairs
    static std::vector<EntityPair> pairs;
    pairs.clear();
    broad_phase_.query_pairs(pairs);  // Already sorted by (min, max) entity ID

    // Grow manifold cache to fit
    std::vector<ContactManifold> new_manifolds;
    new_manifolds.reserve(pairs.size());

    for (const EntityPair& pair : pairs) {
        EntityId ida = pair.a;
        EntityId idb = pair.b;
        Entity& ea = entities_[ida];
        Entity& eb = entities_[idb];

        // Skip inactive pairs
        if (!ea.has(COMP_COLLIDER | COMP_RIGIDBODY)) continue;
        if (!eb.has(COMP_COLLIDER | COMP_RIGIDBODY)) continue;
        if (ea.has(COMP_SLEEP) && eb.has(COMP_SLEEP)) continue;

        // Skip same-skeleton pairs (no self-collision)
        if (ea.collider.skeleton_owner != INVALID_ENTITY &&
            ea.collider.skeleton_owner == eb.collider.skeleton_owner) continue;

        // Generate contacts
        ContactManifold fresh;
        fresh.pair  = pair;
        fresh.mat_a = ea.collider.material_id;
        fresh.mat_b = eb.collider.material_id;

        if (!narrow_phase_.generate_contact(ea, eb, fresh)) {
            // No contact — discard (no manifold)
            continue;
        }

        // Find existing manifold for warm-starting
        auto it = std::lower_bound(manifolds_.begin(), manifolds_.end(), pair,
                                    [](const ContactManifold& m, EntityPair p) {
                                        return m.pair < p;
                                    });

        if (it != manifolds_.end() && it->pair == pair) {
            // Update existing manifold (preserves warm-start impulses)
            narrow_phase_.update_manifold(*it, fresh);
            new_manifolds.push_back(*it);
        } else {
            fresh.last_frame = frame_;
            new_manifolds.push_back(fresh);
        }
    }

    manifolds_ = std::move(new_manifolds);
    // Manifolds are already sorted by EntityPair (from sorted broad-phase pairs)
}

// ============================================================================
// One substep of the Verlet integrator
// ============================================================================
void World::run_substep(FpVel dt) {
    // 1. Flag CCD entities
    ContinuousCollisionDetection::flag_ccd_entities(entities_, dt);

    // 2. CCD sweep — find earliest TOI hits
    static std::vector<std::pair<EntityId, CCDHit>> ccd_hits;
    ccd_hits.clear();
    ccd_.test_all(entities_, broad_phase_, dt, ccd_hits);

    // 3. Velocity Verlet integration
    integrator_.substep(entities_, surface_, dt);

    // 4. For each CCD hit, back-correct entity to TOI position
    //    (advance to TOI, resolve normal impulse, continue for remaining dt)
    for (auto& [eid, hit] : ccd_hits) {
        Entity& e = entities_[eid];
        if (!e.has(COMP_RIGIDBODY)) continue;

        // Reflect velocity along contact normal (simplified CCD response)
        // Full resolution is handled by constraint solver; here we prevent tunnelling
        FpVel vn = e.rigidbody.velocity.dot(hit.normal);
        if (vn.raw < 0) {
            // Remove normal component (inelastic CCD correction)
            e.rigidbody.velocity = e.rigidbody.velocity - hit.normal * vn;
            // Reposition to TOI contact surface
            // Parentheses required: (expr).raw >> 8  not  expr.raw >> 8
            e.rigidbody.position = {
                FpPos{e.rigidbody.position.x.raw + ((hit.contact_point.x * hit.toi).raw >> 8)},
                FpPos{e.rigidbody.position.y.raw + ((hit.contact_point.y * hit.toi).raw >> 8)},
                FpPos{e.rigidbody.position.z.raw + ((hit.contact_point.z * hit.toi).raw >> 8)},
            };
        }
    }
}

// ============================================================================
// Main tick
// ============================================================================
uint64_t World::tick(const TickInput& inputs) {
    events_.clear();

    FpVel substep_dt = SUBSTEP_DT;

    // ── 1. Input → Force accumulation ──────────────────────────────────────
    for (EntityId id = PLAYER_BEGIN; id < PLAYER_END; ++id) {
        Entity& e = entities_[id];
        if (!e.has(COMP_RIGIDBODY | COMP_SKELETON)) continue;
        if (e.has(COMP_SLEEP)) continue;
        InputSystem::apply(e, inputs.inputs[id], surface_, events_, frame_);
    }

    // ── 2. Skeleton controller (motor drives, FK, foot sensors) ────────────
    skeleton_ctrl_.update(entities_, substep_dt, events_, frame_);

    // ── 3. Physics substeps ─────────────────────────────────────────────────
    for (int sub = 0; sub < SUBSTEPS; ++sub) {
        run_substep(substep_dt);
    }

    // ── 4. Broad phase update ───────────────────────────────────────────────
    update_aabb_tree();

    // ── 5. Narrow phase contact generation ─────────────────────────────────
    generate_contacts();

    // ── 6. Trigger system ──────────────────────────────────────────────────
    trigger_system_.update(entities_, events_, frame_);

    // ── 7. Constraint solver ────────────────────────────────────────────────
    solver_.solve(entities_, manifolds_, surface_, events_, frame_);

    // ── 8. Sleep system ─────────────────────────────────────────────────────
    sleep_system_.update(entities_, events_, frame_);

    // ── 9. Event dispatch ───────────────────────────────────────────────────
    dispatch_events();

    // ── State hash (every frame in debug; every 60 in release) ─────────────
    ++frame_;

    bool should_hash = false;
#ifdef NDEBUG
    should_hash = (frame_ % 60 == 0);
#else
    should_hash = true;
#endif
    if (should_hash) {
        last_hash_ = compute_state_hash(entities_);
    }

    return last_hash_;
}

void World::dispatch_events() {
    if (!event_callback_) return;
    PhysicsEvent ev;
    // Drain a copy of the queue for the callback (preserves queue for caller)
    EventQueue tmp = events_;
    while (tmp.pop(ev)) {
        event_callback_(ev);
    }
}

void World::restore_state(const std::array<Entity, MAX_ENTITIES>& snapshot,
                           uint32_t frame) {
    entities_ = snapshot;
    frame_    = frame;
    manifolds_.clear();
    events_.clear();
    // Rebuild broad phase from restored state
    broad_phase_.clear();
    for (EntityId id = 0; id < MAX_ENTITIES; ++id) {
        const Entity& e = entities_[id];
        if (e.has(COMP_COLLIDER | COMP_RIGIDBODY)) {
            broad_phase_.insert(id, e.collider.world_aabb);
        }
    }
}

void World::debug_print_entity(EntityId id) const {
    if (id >= MAX_ENTITIES) return;
    const Entity& e = entities_[id];
    const RigidBody& rb = e.rigidbody;
    printf("[Entity %u] pos=(%.3f, %.3f, %.3f) vel=(%.3f, %.3f, %.3f) mask=%u sleep=%s\n",
           id,
           rb.position.x.to_float(), rb.position.y.to_float(), rb.position.z.to_float(),
           rb.velocity.x.to_float(),  rb.velocity.y.to_float(),  rb.velocity.z.to_float(),
           e.component_mask,
           e.has(COMP_SLEEP) ? "YES" : "NO");
}

} // namespace dspe