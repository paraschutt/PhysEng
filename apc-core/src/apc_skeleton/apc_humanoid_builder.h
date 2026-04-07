#pragma once
// =============================================================================
// Humanoid Builder — Creates standard humanoid skeleton configurations
// =============================================================================
//
// Provides utilities to create common humanoid skeleton layouts:
//   - Standard humanoid: pelvis, spine, chest, head, arms (upper/lower),
//     legs (upper/lower), hands, feet
//   - Simplified ragdoll: fewer bones for performance
//   - Per-bone mass distribution based on anthropometric data
//
// All skeletons follow the APC convention:
//   - Root bone (index 0) is the pelvis/hips
//   - Parent index always < child index
//   - Y-up coordinate system
//   - Bind pose is T-pose (arms horizontal)
//
// =============================================================================

#include "apc_skeleton/apc_skeleton_types.h"
#include "apc_skeleton/apc_skeletal_pose.h"
#include "apc_skeleton/apc_skeleton_collision.h"
#include <vector>
#include <cstdint>

namespace apc {

// =============================================================================
// HumanoidLayout — Configuration for humanoid skeleton generation
// =============================================================================
struct HumanoidLayout {
    float total_mass;          // Total body mass in kg (default: 80.0f)
    float total_height;        // Standing height in meters (default: 1.8f)
    float joint_limit_shoulder;// Shoulder joint limit (radians, default: π)
    float joint_limit_elbow;   // Elbow joint limit (radians, default: 2.5)
    float joint_limit_hip;     // Hip joint limit (radians, default: 1.5)
    float joint_limit_knee;    // Knee joint limit (radians, default: 2.5)
    float joint_limit_spine;   // Spine joint limit (radians, default: 0.5)
    float joint_limit_neck;    // Neck joint limit (radians, default: 0.8)
    float joint_damping;       // Default joint damping (default: 0.02)

    HumanoidLayout()
        : total_mass(80.0f)
        , total_height(1.8f)
        , joint_limit_shoulder(3.14159f)
        , joint_limit_elbow(2.5f)
        , joint_limit_hip(1.5f)
        , joint_limit_knee(2.5f)
        , joint_limit_spine(0.5f)
        , joint_limit_neck(0.8f)
        , joint_damping(0.02f)
    {}
};

// =============================================================================
// Bone indices for standard humanoid
// =============================================================================
enum HumanoidBoneIndex : uint32_t {
    // Core
    HB_PELVIS = 0,

    // Spine chain
    HB_SPINE_LOWER = 1,
    HB_SPINE_UPPER = 2,
    HB_CHEST = 3,
    HB_NECK = 4,
    HB_HEAD = 5,

    // Left arm
    HB_L_SHOULDER = 6,
    HB_L_UPPER_ARM = 7,
    HB_L_FOREARM = 8,
    HB_L_HAND = 9,

    // Right arm
    HB_R_SHOULDER = 10,
    HB_R_UPPER_ARM = 11,
    HB_R_FOREARM = 12,
    HB_R_HAND = 13,

    // Left leg
    HB_L_HIP = 14,
    HB_L_UPPER_LEG = 15,
    HB_L_LOWER_LEG = 16,
    HB_L_FOOT = 17,

    // Right leg
    HB_R_HIP = 18,
    HB_R_UPPER_LEG = 19,
    HB_R_LOWER_LEG = 20,
    HB_R_FOOT = 21,

    HB_BONE_COUNT = 22
};

// =============================================================================
// HumanoidBuilder — Creates humanoid skeletons
// =============================================================================
class HumanoidBuilder {
public:
    // -------------------------------------------------------------------------
    // create_standard — Create a full 22-bone humanoid skeleton.
    //
    // Bone layout:
    //   0:  Pelvis (root, free-floating body)
    //   1:  Lower spine (revolute, Z-axis)
    //   2:  Upper spine (revolute, Z-axis)
    //   3:  Chest (revolute, Z-axis)
    //   4:  Neck (revolute, Z-axis)
    //   5:  Head (revolute, Z-axis)
    //   6:  L shoulder (spherical, 3-DOF)
    //   7:  L upper arm (revolute, Z-axis)
    //   8:  L forearm (revolute, Z-axis)
    //   9:  L hand (revolute, Z-axis)
    //   10: R shoulder (spherical, 3-DOF)
    //   11: R upper arm (revolute, Z-axis)
    //   12: R forearm (revolute, Z-axis)
    //   13: R hand (revolute, Z-axis)
    //   14: L hip (spherical, 3-DOF)
    //   15: L upper leg (revolute, Z-axis)
    //   16: L lower leg (revolute, Z-axis)
    //   17: L foot (revolute, X-axis)
    //   18: R hip (spherical, 3-DOF)
    //   19: R upper leg (revolute, Z-axis)
    //   20: R lower leg (revolute, Z-axis)
    //   21: R foot (revolute, X-axis)
    // -------------------------------------------------------------------------
    static void create_standard(
        const HumanoidLayout& layout,
        SkeletalAsset& asset)
    {
        asset.bones.clear();

        float scale = layout.total_height / 1.8f;

        // --- Mass distribution (approximate anthropometric percentages) ---
        // Total: 100%
        // Pelvis: 12%, Spine: 8%, Chest: 16%, Neck+Head: 8%
        // Each upper arm: 3%, each forearm: 2%, each hand: 1%
        // Each upper leg: 10%, each lower leg: 5%, each foot: 1.5%
        float m_total = layout.total_mass;

        // Helper lambda
        auto rev = [&](uint32_t parent, Vec3 offset, Vec3 axis,
                       float mass_pct, float inertia_factor,
                       float limit, uint8_t collision = 1) -> Bone {
            float m = m_total * mass_pct;
            float I = m * scale * scale * inertia_factor * 0.01f;
            Bone b = Bone::make_revolute(parent, Vec3::scale(offset, scale),
                axis, m, I > 0.01f ? I : 0.01f, limit, layout.joint_damping);
            b.collision_enabled = collision;
            b.joint_to_com = Vec3(0.0f, 0.0f, 0.0f);
            return b;
        };

        auto sph = [&](uint32_t parent, Vec3 offset,
                       float mass_pct, float inertia_factor,
                       float limit, uint8_t collision = 1) -> Bone {
            float m = m_total * mass_pct;
            float I = m * scale * scale * inertia_factor * 0.01f;
            Bone b = Bone::make_spherical(parent, Vec3::scale(offset, scale),
                m, I > 0.01f ? I : 0.01f, limit, layout.joint_damping);
            b.collision_enabled = collision;
            b.joint_to_com = Vec3(0.0f, 0.0f, 0.0f);
            return b;
        };

        // --- Core (Pelvis as root) ---
        // Root: 6-DOF free body — we approximate with a spherical joint (3 DOF)
        // for rotation and let position be handled externally.
        Bone pelvis = Bone::make_spherical(0xFFFFFFFF,
            Vec3(0.0f, 0.9f * scale, 0.0f),
            m_total * 0.12f, 5.0f, 3.14159f, layout.joint_damping);
        pelvis.joint_to_com = Vec3(0.0f, -0.05f * scale, 0.0f);
        asset.bones.push_back(pelvis);

        // --- Spine chain ---
        asset.bones.push_back(rev(HB_PELVIS, Vec3(0, 0.05f, 0),
            Vec3(0, 0, 1), 0.08f, 3.0f, layout.joint_limit_spine));
        asset.bones.push_back(rev(HB_SPINE_LOWER, Vec3(0, 0.15f, 0),
            Vec3(0, 0, 1), 0.06f, 2.5f, layout.joint_limit_spine));
        asset.bones.push_back(rev(HB_SPINE_UPPER, Vec3(0, 0.15f, 0),
            Vec3(0, 0, 1), 0.10f, 4.0f, layout.joint_limit_spine));
        asset.bones.push_back(rev(HB_CHEST, Vec3(0, 0.12f, 0),
            Vec3(0, 0, 1), 0.03f, 1.0f, layout.joint_limit_neck));
        asset.bones.push_back(rev(HB_NECK, Vec3(0, 0.08f, 0),
            Vec3(0, 0, 1), 0.05f, 2.0f, layout.joint_limit_neck, 0)); // Head no collision

        // --- Left arm ---
        asset.bones.push_back(sph(HB_CHEST, Vec3(0.18f, 0.10f, 0),
            0.03f, 2.0f, layout.joint_limit_shoulder));
        asset.bones.push_back(rev(HB_L_SHOULDER, Vec3(0, 0.28f, 0),
            Vec3(0, 0, 1), 0.025f, 2.0f, layout.joint_limit_elbow));
        asset.bones.push_back(rev(HB_L_UPPER_ARM, Vec3(0, 0.25f, 0),
            Vec3(0, 0, 1), 0.02f, 1.5f, layout.joint_limit_elbow));
        asset.bones.push_back(rev(HB_L_FOREARM, Vec3(0, 0.20f, 0),
            Vec3(0, 0, 1), 0.008f, 0.5f, layout.joint_limit_elbow, 0));

        // --- Right arm ---
        asset.bones.push_back(sph(HB_CHEST, Vec3(-0.18f, 0.10f, 0),
            0.03f, 2.0f, layout.joint_limit_shoulder));
        asset.bones.push_back(rev(HB_R_SHOULDER, Vec3(0, 0.28f, 0),
            Vec3(0, 0, 1), 0.025f, 2.0f, layout.joint_limit_elbow));
        asset.bones.push_back(rev(HB_R_UPPER_ARM, Vec3(0, 0.25f, 0),
            Vec3(0, 0, 1), 0.02f, 1.5f, layout.joint_limit_elbow));
        asset.bones.push_back(rev(HB_R_FOREARM, Vec3(0, 0.20f, 0),
            Vec3(0, 0, 1), 0.008f, 0.5f, layout.joint_limit_elbow, 0));

        // --- Left leg ---
        asset.bones.push_back(sph(HB_PELVIS, Vec3(0.10f, -0.05f, 0),
            0.10f, 4.0f, layout.joint_limit_hip));
        asset.bones.push_back(rev(HB_L_HIP, Vec3(0, 0.40f, 0),
            Vec3(0, 0, 1), 0.05f, 3.0f, layout.joint_limit_knee));
        asset.bones.push_back(rev(HB_L_UPPER_LEG, Vec3(0, 0.40f, 0),
            Vec3(0, 0, 1), 0.015f, 1.5f, layout.joint_limit_knee));
        asset.bones.push_back(rev(HB_L_LOWER_LEG, Vec3(0, 0.05f, 0),
            Vec3(1, 0, 0), 0.015f, 1.0f, 1.0f, 1));

        // --- Right leg ---
        asset.bones.push_back(sph(HB_PELVIS, Vec3(-0.10f, -0.05f, 0),
            0.10f, 4.0f, layout.joint_limit_hip));
        asset.bones.push_back(rev(HB_R_HIP, Vec3(0, 0.40f, 0),
            Vec3(0, 0, 1), 0.05f, 3.0f, layout.joint_limit_knee));
        asset.bones.push_back(rev(HB_R_UPPER_LEG, Vec3(0, 0.40f, 0),
            Vec3(0, 0, 1), 0.015f, 1.5f, layout.joint_limit_knee));
        asset.bones.push_back(rev(HB_R_LOWER_LEG, Vec3(0, 0.05f, 0),
            Vec3(1, 0, 0), 0.015f, 1.0f, 1.0f, 1));
    }

    // -------------------------------------------------------------------------
    // create_collision_shapes — Create collision shapes for a humanoid.
    // Uses capsule primitives for limbs, sphere for head, box for torso.
    // -------------------------------------------------------------------------
    static void create_collision_shapes(
        const HumanoidLayout& layout,
        BoneCollisionShape* out_shapes,
        uint32_t max_shapes)
    {
        float s = layout.total_height / 1.8f;

        auto clear = [&](uint32_t idx) {
            if (idx < max_shapes) out_shapes[idx] = BoneCollisionShape();
        };

        // Clear all
        for (uint32_t i = 0; i < max_shapes; ++i) clear(i);

        // Pelvis: box
        if (HB_PELVIS < max_shapes)
            out_shapes[HB_PELVIS] = BoneCollisionShape::make_box(
                Vec3(0.15f * s, 0.10f * s, 0.08f * s));

        // Spine/chest: capsule
        if (HB_SPINE_LOWER < max_shapes)
            out_shapes[HB_SPINE_LOWER] = BoneCollisionShape::make_capsule(
                0.08f * s, 0.12f * s);
        if (HB_SPINE_UPPER < max_shapes)
            out_shapes[HB_SPINE_UPPER] = BoneCollisionShape::make_capsule(
                0.09f * s, 0.12f * s);
        if (HB_CHEST < max_shapes)
            out_shapes[HB_CHEST] = BoneCollisionShape::make_capsule(
                0.12f * s, 0.14f * s);

        // Head: sphere
        if (HB_HEAD < max_shapes)
            out_shapes[HB_HEAD] = BoneCollisionShape::make_sphere(
                0.10f * s, Vec3(0, 0.08f * s, 0));

        // Upper arms: capsule
        if (HB_L_UPPER_ARM < max_shapes)
            out_shapes[HB_L_UPPER_ARM] = BoneCollisionShape::make_capsule(
                0.04f * s, 0.12f * s);
        if (HB_R_UPPER_ARM < max_shapes)
            out_shapes[HB_R_UPPER_ARM] = BoneCollisionShape::make_capsule(
                0.04f * s, 0.12f * s);

        // Forearms: capsule
        if (HB_L_FOREARM < max_shapes)
            out_shapes[HB_L_FOREARM] = BoneCollisionShape::make_capsule(
                0.035f * s, 0.10f * s);
        if (HB_R_FOREARM < max_shapes)
            out_shapes[HB_R_FOREARM] = BoneCollisionShape::make_capsule(
                0.035f * s, 0.10f * s);

        // Upper legs: capsule
        if (HB_L_UPPER_LEG < max_shapes)
            out_shapes[HB_L_UPPER_LEG] = BoneCollisionShape::make_capsule(
                0.07f * s, 0.18f * s);
        if (HB_R_UPPER_LEG < max_shapes)
            out_shapes[HB_R_UPPER_LEG] = BoneCollisionShape::make_capsule(
                0.07f * s, 0.18f * s);

        // Lower legs: capsule
        if (HB_L_LOWER_LEG < max_shapes)
            out_shapes[HB_L_LOWER_LEG] = BoneCollisionShape::make_capsule(
                0.05f * s, 0.18f * s);
        if (HB_R_LOWER_LEG < max_shapes)
            out_shapes[HB_R_LOWER_LEG] = BoneCollisionShape::make_capsule(
                0.05f * s, 0.18f * s);

        // Feet: box
        if (HB_L_FOOT < max_shapes)
            out_shapes[HB_L_FOOT] = BoneCollisionShape::make_box(
                Vec3(0.05f * s, 0.04f * s, 0.10f * s),
                Vec3(0, 0, 0.05f * s));
        if (HB_R_FOOT < max_shapes)
            out_shapes[HB_R_FOOT] = BoneCollisionShape::make_box(
                Vec3(0.05f * s, 0.04f * s, 0.10f * s),
                Vec3(0, 0, 0.05f * s));
    }

    // -------------------------------------------------------------------------
    // create_ragdoll_config — Configure all bones for ragdoll mode.
    // Sets blend mode to PHYSICS_DRIVEN for all bones except pelvis
    // (which stays as is since it's the root).
    // -------------------------------------------------------------------------
    static void configure_ragdoll(SkeletalAsset& asset) {
        for (uint32_t i = 0; i < asset.get_bone_count(); ++i) {
            asset.bones[i].physics.mode = PhysicsBlendMode::PHYSICS_DRIVEN;
            asset.bones[i].physics.stiffness = 0.0f;
            asset.bones[i].physics.damping = 0.0f;
            asset.bones[i].physics.max_deviation = 0.0f;
        }
    }

    // -------------------------------------------------------------------------
    // configure_anim_driven — Set all bones to animation-driven.
    // -------------------------------------------------------------------------
    static void configure_anim_driven(SkeletalAsset& asset) {
        for (uint32_t i = 0; i < asset.get_bone_count(); ++i) {
            asset.bones[i].physics.mode = PhysicsBlendMode::ANIM_DRIVEN;
        }
    }

    // -------------------------------------------------------------------------
    // configure_partial_ragdoll — Upper body physics, lower body animation.
    // Common for tackle/impact scenarios.
    // -------------------------------------------------------------------------
    static void configure_partial_ragdoll(
        SkeletalAsset& asset,
        float stiffness = 30.0f,
        float damping = 3.0f)
    {
        // Upper body: physics
        for (uint32_t i = HB_SPINE_LOWER; i <= HB_HEAD; ++i) {
            asset.bones[i].physics.mode = PhysicsBlendMode::PHYSICS_DRIVEN;
        }
        // Arms: blended (flail but recover)
        for (uint32_t i = HB_L_SHOULDER; i <= HB_R_HAND; ++i) {
            asset.bones[i].physics.mode = PhysicsBlendMode::BLENDED;
            asset.bones[i].physics.stiffness = stiffness;
            asset.bones[i].physics.damping = damping;
            asset.bones[i].physics.max_deviation = 2.0f;
        }
        // Legs: anim driven (planted)
        for (uint32_t i = HB_L_HIP; i <= HB_R_FOOT; ++i) {
            asset.bones[i].physics.mode = PhysicsBlendMode::ANIM_DRIVEN;
        }
    }
};

} // namespace apc
