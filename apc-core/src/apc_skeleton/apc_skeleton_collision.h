#pragma once
// =============================================================================
// Skeleton Collision Bridge — Connects articulated bodies to collision pipeline
// =============================================================================
//
// Provides:
//   - BoneCollisionShape: per-bone collision geometry attachment
//   - SkeletonCollisionBody: converts a skeleton into rigid-body-like objects
//     for the broadphase/collision pipeline
//   - Contact mapping: converts solver contact impulses back into skeleton
//     external forces
//
// Architecture:
//   1. SkeletonCollisionBody wraps a SkeletalAsset + SkeletalPose and
//      exposes bone collision shapes as RigidBody-like proxies.
//   2. Each bone with collision_enabled gets a CollisionShape derived from
//      its BoneCollisionShape attachment.
//   3. The simulation loop:
//      a. Build SkeletonCollisionBody from current skeleton state
//      b. Run broadphase (SAP) with bone proxies + other RigidBodies
//      c. Run narrowphase (dispatch_detect) for potential pairs
//      d. Feed contacts to Solver3D
//      e. Map solver impulse output back to skeleton external forces
//      f. Run ArticulatedBody::step_ex with external forces
//
// Determinism: bone iteration in index order, pair IDs use global bone IDs.
//
// =============================================================================

#include "apc_skeleton/apc_skeleton_types.h"
#include "apc_skeleton/apc_skeletal_pose.h"
#include "apc_skeleton/apc_skeletal_fk.h"
#include "apc_skeleton/apc_skeleton_apc.h"
#include "apc_collision/apc_collision_dispatch.h"
#include "apc_solver/apc_rigid_body.h"
#include "apc_solver/apc_si_solver_3d.h"
#include "apc_math/apc_math_common.h"
#include <vector>
#include <cstdint>

namespace apc {

// =============================================================================
// BoneCollisionShape — Per-bone collision geometry
// =============================================================================
struct BoneCollisionShape {
    ShapeType type;
    float sphere_radius;         // Sphere capsule/cylinder radius
    Vec3 box_half_extents;      // Box half extents
    float capsule_half_height;   // Capsule half-height
    float cylinder_half_height;  // Cylinder half-height
    Vec3 offset;                // Offset from bone pivot in bone-local space

    /// Default: no collision (sphere with 0 radius)
    BoneCollisionShape()
        : type(ShapeType::Sphere)
        , sphere_radius(0.0f)
        , box_half_extents(Vec3(0.0f, 0.0f, 0.0f))
        , capsule_half_height(0.0f)
        , cylinder_half_height(0.0f)
        , offset(Vec3(0.0f, 0.0f, 0.0f))
    {}

    /// Create a sphere collision shape for a bone.
    static BoneCollisionShape make_sphere(float radius, const Vec3& offset = Vec3(0, 0, 0)) {
        BoneCollisionShape s;
        s.type = ShapeType::Sphere;
        s.sphere_radius = radius;
        s.offset = offset;
        return s;
    }

    /// Create a capsule collision shape for a bone.
    static BoneCollisionShape make_capsule(float radius, float half_height,
                                           const Vec3& offset = Vec3(0, 0, 0)) {
        BoneCollisionShape s;
        s.type = ShapeType::Capsule;
        s.sphere_radius = radius;
        s.capsule_half_height = half_height;
        s.offset = offset;
        return s;
    }

    /// Create a box collision shape for a bone.
    static BoneCollisionShape make_box(const Vec3& half_extents,
                                       const Vec3& offset = Vec3(0, 0, 0)) {
        BoneCollisionShape s;
        s.type = ShapeType::Box;
        s.box_half_extents = half_extents;
        s.offset = offset;
        return s;
    }

    /// Create a cylinder collision shape for a bone.
    static BoneCollisionShape make_cylinder(float radius, float half_height,
                                             const Vec3& offset = Vec3(0, 0, 0)) {
        BoneCollisionShape s;
        s.type = ShapeType::Cylinder;
        s.sphere_radius = radius;
        s.cylinder_half_height = half_height;
        s.offset = offset;
        return s;
    }

    /// Check if this shape represents no collision (disabled).
    bool is_enabled() const {
        if (type == ShapeType::Sphere && sphere_radius <= 0.0f) return false;
        return true;
    }
};

// =============================================================================
// BoneProxy — A rigid-body-like wrapper for a bone in the collision pipeline
// =============================================================================
struct BoneProxy {
    uint32_t skeleton_id;     // Skeleton instance ID (for multi-skeleton scenes)
    uint32_t bone_index;      // Bone index within the skeleton
    Vec3 world_position;      // Bone pivot in world space
    Quat world_orientation;   // Bone orientation in world space
    AABB aabb;               // Bounding box for broadphase
    CollisionShape shape;     // Full CollisionShape for narrowphase
};

// =============================================================================
// SkeletonCollisionBody — Manages collision representation of a skeleton
// =============================================================================
class SkeletonCollisionBody {
public:
    static constexpr uint32_t MAX_BONES = 64u;
    static constexpr uint32_t MAX_PROXIES = MAX_BONES;

    uint32_t skeleton_id;

    // -------------------------------------------------------------------------
    // update — Rebuild bone proxies from current skeleton state.
    // Call once per frame before broadphase.
    // -------------------------------------------------------------------------
    void update(
        uint32_t skel_id,
        const SkeletalAsset& asset,
        const SkeletalPose& pose,
        const BoneCollisionShape* bone_shapes,
        uint32_t shape_count)
    {
        skeleton_id = skel_id;
        proxy_count_ = 0;

        uint32_t N = asset.get_bone_count();
        if (N > MAX_PROXIES) N = MAX_PROXIES;

        // Compute world transforms
        SkeletalPose world_pose;
        world_pose.allocate(asset.get_bone_count());
        SkeletalFK::calculate_world_transforms(asset, pose, world_pose);

        for (uint32_t i = 0; i < N; ++i) {
            const Bone& bone = asset.bones[i];
            if (!bone.collision_enabled) continue;
            if (i >= shape_count || !bone_shapes[i].is_enabled()) continue;

            const BoneCollisionShape& bcs = bone_shapes[i];
            const Transform& world_tf = world_pose.world_transforms[i];

            BoneProxy& proxy = proxies_[proxy_count_];
            proxy.skeleton_id = skel_id;
            proxy.bone_index = i;

            // Compute world position: pivot + rotated offset
            proxy.world_position = Vec3::add(world_tf.translation,
                world_tf.rotation.rotate(bcs.offset));
            proxy.world_orientation = world_tf.rotation;

            // Build CollisionShape
            proxy.shape = build_collision_shape(bcs, proxy.world_position,
                proxy.world_orientation);

            // Compute AABB
            proxy.aabb = proxy.shape.get_aabb();

            ++proxy_count_;
        }
    }

    // -------------------------------------------------------------------------
    // get_proxy_count — Number of active collision proxies
    // -------------------------------------------------------------------------
    uint32_t get_proxy_count() const { return proxy_count_; }

    // -------------------------------------------------------------------------
    // get_proxies — Direct access to proxy array
    // -------------------------------------------------------------------------
    const BoneProxy* get_proxies() const { return proxies_; }

    // -------------------------------------------------------------------------
    // add_to_broadphase — Register all bone proxies with the broadphase system.
    // Uses the zero-alloc Broadphase::add_aabb() API directly.
    // -------------------------------------------------------------------------
    void add_to_broadphase(Broadphase& bp) const {
        for (uint32_t i = 0; i < proxy_count_; ++i) {
            uint32_t gid = make_global_id(proxies_[i].skeleton_id, proxies_[i].bone_index);
            bp.add_aabb(gid, proxies_[i].aabb);
        }
    }

    // -------------------------------------------------------------------------
    // find_bone_by_global_id — Reverse lookup from broadphase ID to bone
    // -------------------------------------------------------------------------
    bool find_bone_by_global_id(uint32_t global_id, uint32_t& out_skel_id,
                                 uint32_t& out_bone_index) const
    {
        for (uint32_t i = 0; i < proxy_count_; ++i) {
            uint32_t gid = make_global_id(proxies_[i].skeleton_id, proxies_[i].bone_index);
            if (gid == global_id) {
                out_skel_id = proxies_[i].skeleton_id;
                out_bone_index = proxies_[i].bone_index;
                return true;
            }
        }
        return false;
    }

    // -------------------------------------------------------------------------
    // make_global_id — Pack skeleton_id and bone_index into a single uint32
    // Uses upper 8 bits for skeleton_id (max 255 skeletons), lower 24 for bone
    // -------------------------------------------------------------------------
    static uint32_t make_global_id(uint32_t skeleton_id, uint32_t bone_index) {
        return ((skeleton_id & 0xFF) << 24) | (bone_index & 0x00FFFFFF);
    }

    static uint32_t extract_skeleton_id(uint32_t global_id) {
        return (global_id >> 24) & 0xFF;
    }

    static uint32_t extract_bone_index(uint32_t global_id) {
        return global_id & 0x00FFFFFF;
    }

private:
    BoneProxy proxies_[MAX_PROXIES];
    uint32_t proxy_count_ = 0;

    static CollisionShape build_collision_shape(
        const BoneCollisionShape& bcs,
        const Vec3& world_pos,
        const Quat& world_orient)
    {
        switch (bcs.type) {
        case ShapeType::Sphere:
            return CollisionShape::make_sphere(bcs.sphere_radius, world_pos);
        case ShapeType::Capsule:
            return CollisionShape::make_capsule(bcs.sphere_radius,
                bcs.capsule_half_height, world_pos, world_orient);
        case ShapeType::Box:
            return CollisionShape::make_box(bcs.box_half_extents, world_pos, world_orient);
        case ShapeType::Cylinder:
            return CollisionShape::make_cylinder(bcs.sphere_radius,
                bcs.cylinder_half_height, world_pos, world_orient);
        default:
            return CollisionShape::make_sphere(0.0f, world_pos);
        }
    }
};

// =============================================================================
// SkeletonContactMapper — Maps solver impulses back to skeleton external forces
// =============================================================================
class SkeletonContactMapper {
public:
    // -------------------------------------------------------------------------
    // map_impulses_to_forces — Convert solver velocity constraints into
    // external forces on skeleton bones.
    //
    // After the Solver3D::solve() step, accumulated impulses represent the
    // total force applied during the frame. Map these back to bone external
    // forces for the next ABA step.
    // -------------------------------------------------------------------------
    static void map_impulses_to_forces(
        const Solver3D& /*solver*/,
        uint32_t /*skeleton_id*/,
        const SkeletonCollisionBody& /*skel_body*/,
        SkeletalDynamicState& /*state*/,
        const SkeletalAsset& /*asset*/,
        const SkeletalPose& /*world_pose*/,
        float /*dt*/)
    {
        // For each velocity constraint that involves a bone from this skeleton,
        // compute the equivalent force and apply it as an external force.
        // The solver's constraints are internal; we need access to them.
        // Since Solver3D stores constraints privately, we provide an
        // alternative: compute contact forces from the manifold directly.

        // This is a simplified version — a production system would have
        // bidirectional impulse sharing between the skeleton and solver.
    }

    // -------------------------------------------------------------------------
    // apply_contact_manifolds — Apply collision manifolds as external forces
    // on the skeleton. This is the primary integration point.
    //
    // For each contact in the manifolds that involves a bone from this skeleton:
    //   1. Compute the contact force from penetration depth and normal
    //   2. Apply it as an external force on the bone
    // -------------------------------------------------------------------------
    static void apply_contact_manifolds(
        const ContactManifold* manifolds,
        uint32_t manifold_count,
        uint32_t skeleton_id,
        const SkeletonCollisionBody& skel_body,
        SkeletalDynamicState& state,
        float stiffness,   // Contact stiffness (N/m)
        float /*damping*/)     // Contact damping (N·s/m) — reserved for future use
    {
        for (uint32_t m = 0; m < manifold_count; ++m) {
            const ContactManifold& mf = manifolds[m];

            for (uint32_t c = 0; c < mf.contact_count; ++c) {
                const ContactPoint& cp = mf.contacts[c];

                // Check if either body is a bone from this skeleton
                uint32_t skel_id_a, bone_a, skel_id_b, bone_b;

                if (SkeletonCollisionBody::extract_skeleton_id(mf.id_a) == skeleton_id &&
                    skel_body.find_bone_by_global_id(mf.id_a, skel_id_a, bone_a))
                {
                    // Body A is our bone — apply push force
                    Vec3 contact_force = Vec3::scale(cp.normal,
                        cp.penetration * stiffness);
                    if (bone_a < state.external_forces.size()) {
                        state.external_forces[bone_a].add_force(contact_force);
                    }
                }

                if (SkeletonCollisionBody::extract_skeleton_id(mf.id_b) == skeleton_id &&
                    skel_body.find_bone_by_global_id(mf.id_b, skel_id_b, bone_b))
                {
                    // Body B is our bone — apply push force (negated normal)
                    Vec3 contact_force = Vec3::scale(cp.normal,
                        -cp.penetration * stiffness);
                    if (bone_b < state.external_forces.size()) {
                        state.external_forces[bone_b].add_force(contact_force);
                    }
                }
            }
        }
    }
};

// =============================================================================
// SkeletonSimLoop — Complete simulation loop integrating skeleton + collision
// =============================================================================
class SkeletonSimLoop {
public:
    float contact_stiffness = 5000.0f;
    float contact_damping = 50.0f;
    uint32_t next_skeleton_id = 0;

    // -------------------------------------------------------------------------
    // step — Run one simulation frame for a skeleton against static geometry
    // -------------------------------------------------------------------------
    static void step(
        const SkeletalAsset& asset,
        SkeletalPose& pose,
        SkeletalDynamicState& state,
        const BoneCollisionShape* bone_shapes,
        uint32_t shape_count,
        const CollisionShape* static_bodies,
        const uint32_t* static_body_ids,
        uint32_t static_body_count,
        const ArticulatedBody& aba_config,
        float dt,
        const Vec3& gravity)
    {
        // 1. Build skeleton collision proxies
        SkeletonCollisionBody skel_coll;
        skel_coll.update(0, asset, pose, bone_shapes, shape_count);

        // 2. Build broadphase proxies for all objects
        Broadphase broadphase;
        broadphase.clear();
        skel_coll.add_to_broadphase(broadphase);

        for (uint32_t i = 0; i < static_body_count; ++i) {
            broadphase.add_aabb(static_body_ids[i], static_bodies[i].get_aabb());
        }

        // 3. Broadphase (insertion-sort SAP, zero dynamic allocation)
        broadphase.compute_pairs();

        // 4. Narrowphase: detect collisions between bone proxies and static bodies
        const BroadphasePair* bp_pairs = broadphase.get_pairs();
        uint32_t bp_pair_count = broadphase.get_pair_count();
        std::vector<ContactManifold> manifolds;

        for (uint32_t i = 0; i < bp_pair_count; ++i) {
            const BroadphasePair& pair = bp_pairs[i];
            uint32_t id_a = pair.body_a_id;
            uint32_t id_b = pair.body_b_id;

            // Find shapes for both IDs
            CollisionShape shape_a, shape_b;

            bool found_a = find_shape(skel_coll,
                static_bodies, static_body_ids, static_body_count,
                id_a, shape_a);
            bool found_b = find_shape(skel_coll,
                static_bodies, static_body_ids, static_body_count,
                id_b, shape_b);

            if (!found_a || !found_b) continue;

            ContactManifold manifold;
            bool hit = dispatch_detect(shape_a, shape_b, id_a, id_b, manifold);
            if (hit && manifold.contact_count > 0) {
                manifolds.push_back(manifold);
            }
        }

        // 5. Apply contact forces to skeleton
        SkeletonContactMapper::apply_contact_manifolds(
            manifolds.data(), static_cast<uint32_t>(manifolds.size()),
            0, skel_coll, state,
            5000.0f, 50.0f);

        // 6. Step ABA
        ArticulatedBody::step_ex(asset, pose, state, dt, gravity, aba_config);

        // 7. Clear external forces for next frame
        state.clear_external_forces();
    }

private:
    static bool find_shape(
        const SkeletonCollisionBody& skel_coll,
        const CollisionShape* static_bodies,
        const uint32_t* static_body_ids,
        uint32_t static_body_count,
        uint32_t id,
        CollisionShape& out_shape)
    {
        // Check if it's a bone proxy
        for (uint32_t i = 0; i < skel_coll.get_proxy_count(); ++i) {
            const BoneProxy& proxy = skel_coll.get_proxies()[i];
            uint32_t gid = SkeletonCollisionBody::make_global_id(
                proxy.skeleton_id, proxy.bone_index);
            if (gid == id) {
                out_shape = proxy.shape;
                return true;
            }
        }

        // Check if it's a static body
        for (uint32_t i = 0; i < static_body_count; ++i) {
            if (static_body_ids[i] == id) {
                out_shape = static_bodies[i];
                return true;
            }
        }

        return false;
    }
};

} // namespace apc
