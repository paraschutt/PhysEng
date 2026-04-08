#pragma once
// =============================================================================
// Sprint 18: Skeleton Renderer — Bone visualization, blend mode colors
// =============================================================================
//
// Provides:
//   - BoneVisualConfig: visual parameters for bone rendering (thickness, colors,
//     joint display options)
//   - BoneDrawCommand: preprocessed draw command per bone for rendering
//   - SkeletonRenderer: processes SkeletalAsset + SkeletalPose into draw commands
//     and renders bones, joints, COMs, axes, hierarchy
//   - BlendModeColorCoding: standalone utility for blend-mode color mapping
//   - VelocityVisualizer: draws linear/angular velocity arrows on RigidBody
//   - ForceVisualizer: draws force/torque arcs on bones
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation (fixed-capacity arrays)
//   - C++17
//
// =============================================================================

#include "apc_render_types.h"
#include "apc_debug_draw.h"
#include "apc_skeleton/apc_skeleton_types.h"
#include "apc_skeleton/apc_skeletal_pose.h"
#include "apc_solver/apc_rigid_body.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include "apc_math/apc_mat3.h"
#include "apc_math/apc_math_common.h"
#include <cstdint>
#include <cmath>

namespace apc {

// =============================================================================
// BoneVisualConfig — Visual parameters for bone rendering
// =============================================================================
struct BoneVisualConfig {
    float thickness = 0.02f;                  // Bone line thickness (visual only)
    float joint_sphere_radius = 0.03f;        // Sphere radius at each joint
    bool show_com = false;                     // Show center of mass points
    bool show_joint_axes = false;              // Show joint axis indicators
    bool show_joint_limits = false;            // Show joint limit arcs

    RenderColor color_anim_driven = RenderColor::BLUE();
    RenderColor color_physics_driven = RenderColor::RED();
    RenderColor color_blended = RenderColor::MAGENTA();
    float blend_gradient_weight = 0.0f;        // If BLENDED, interpolate by physics_weight

    /// Get the display color for a bone based on its blend mode.
    RenderColor get_bone_color(PhysicsBlendMode mode, float physics_weight = 0.0f) const {
        switch (mode) {
        case PhysicsBlendMode::ANIM_DRIVEN:
            return color_anim_driven;
        case PhysicsBlendMode::PHYSICS_DRIVEN:
            return color_physics_driven;
        case PhysicsBlendMode::BLENDED:
            return RenderColor(
                color_anim_driven.r + (color_physics_driven.r - color_anim_driven.r) * physics_weight,
                color_anim_driven.g + (color_physics_driven.g - color_anim_driven.g) * physics_weight,
                color_anim_driven.b + (color_physics_driven.b - color_anim_driven.b) * physics_weight,
                color_anim_driven.a + (color_physics_driven.a - color_anim_driven.a) * physics_weight
            );
        default:
            return color_physics_driven;
        }
    }

    /// Factory returning reasonable defaults.
    static BoneVisualConfig make_default() {
        BoneVisualConfig cfg;
        cfg.thickness = 0.02f;
        cfg.joint_sphere_radius = 0.03f;
        cfg.show_com = false;
        cfg.show_joint_axes = false;
        cfg.show_joint_limits = false;
        cfg.color_anim_driven = RenderColor::BLUE();
        cfg.color_physics_driven = RenderColor::RED();
        cfg.color_blended = RenderColor::MAGENTA();
        cfg.blend_gradient_weight = 0.0f;
        return cfg;
    }
};

// =============================================================================
// BoneDrawCommand — Preprocessed draw command per bone
// =============================================================================
struct BoneDrawCommand {
    uint32_t bone_index = 0u;
    Vec3 start_position;            // Joint pivot position (world space)
    Vec3 end_position;              // End of bone (world space)
    Quat orientation;               // Bone orientation
    uint32_t parent_index = 0xFFFFFFFFu;
    PhysicsBlendMode blend_mode = PhysicsBlendMode::PHYSICS_DRIVEN;
    JointType joint_type = JointType::Fixed;
    float physics_weight = 0.0f;    // For BLENDED mode gradient
    RenderColor color = RenderColor::WHITE();
};

// =============================================================================
// SkeletonRenderer — Processes skeleton into draw commands, renders bones
// =============================================================================
struct SkeletonRenderer {
    static constexpr uint32_t MAX_BONES = 80u;

    BoneVisualConfig config;
    BoneDrawCommand commands[MAX_BONES];
    uint32_t command_count = 0u;

    /// Set visual configuration.
    void set_config(const BoneVisualConfig& cfg) {
        config = cfg;
    }

    /// Clear all draw commands.
    void clear() {
        command_count = 0u;
    }

    /// Process a skeleton into draw commands.
    /// For each bone: start = world_transform translation.
    /// End = first child's world_transform translation if children exist,
    ///       otherwise a guessed direction using rotation.rotate(Vec3(0, 0.1f, 0)).
    void process_skeleton(const SkeletalAsset& asset, const SkeletalPose& pose) {
        clear();
        uint32_t bone_count = asset.get_bone_count();
        if (bone_count == 0u) return;
        uint32_t count = (bone_count < MAX_BONES) ? bone_count : MAX_BONES;

        for (uint32_t i = 0u; i < count; ++i) {
            BoneDrawCommand& cmd = commands[i];
            cmd.bone_index = i;
            cmd.start_position = pose.world_transforms[i].translation;
            cmd.orientation = pose.world_transforms[i].rotation;
            cmd.parent_index = asset.bones[i].parent_index;
            cmd.blend_mode = asset.bones[i].physics.mode;
            cmd.joint_type = asset.bones[i].joint_type;
            cmd.physics_weight = (asset.bones[i].physics.stiffness < 0.0f) ? 0.0f :
                                 (asset.bones[i].physics.stiffness > 1.0f) ? 1.0f :
                                 asset.bones[i].physics.stiffness;
            cmd.color = config.get_bone_color(cmd.blend_mode, cmd.physics_weight);

            // Find first child bone
            bool found_child = false;
            for (uint32_t j = i + 1u; j < count; ++j) {
                if (asset.bones[j].parent_index == i) {
                    cmd.end_position = pose.world_transforms[j].translation;
                    found_child = true;
                    break;
                }
            }
            if (!found_child) {
                // No children: guess direction from bone's rotation
                cmd.end_position = Vec3::add(cmd.start_position,
                    pose.world_transforms[i].rotation.rotate(Vec3(0.0f, 0.1f, 0.0f)));
            }

            command_count++;
        }
    }

    /// Alias for process_skeleton.
    void generate_bone_draw_commands(const SkeletalAsset& asset, const SkeletalPose& pose) {
        process_skeleton(asset, pose);
    }

    /// Draw bone lines from start to end for each command.
    void draw_bones(DebugDraw& dd) {
        for (uint32_t i = 0u; i < command_count; ++i) {
            const BoneDrawCommand& cmd = commands[i];
            dd.list.add_line(cmd.start_position, cmd.end_position, cmd.color);
        }
    }

    /// Draw joint spheres (points) at each bone's start position.
    void draw_joint_spheres(DebugDraw& dd) {
        float size = config.joint_sphere_radius * 100.0f;
        for (uint32_t i = 0u; i < command_count; ++i) {
            const BoneDrawCommand& cmd = commands[i];
            dd.list.add_point(cmd.start_position, cmd.color, size);
        }
    }

    /// Draw center of mass points for each bone in CYAN.
    void draw_com_points(DebugDraw& dd, const SkeletalAsset& asset) {
        RenderColor com_color = RenderColor::CYAN();
        float size = config.joint_sphere_radius * 50.0f;
        uint32_t count = (command_count < asset.get_bone_count()) ? command_count : asset.get_bone_count();
        for (uint32_t i = 0u; i < count; ++i) {
            Vec3 world_com = Vec3::add(
                commands[i].start_position,
                commands[i].orientation.rotate(asset.bones[i].joint_to_com)
            );
            dd.list.add_point(world_com, com_color, size);
        }
    }

    /// Draw transform axes at each joint (short RGB axes).
    void draw_joint_axes(DebugDraw& dd, const SkeletalPose& pose) {
        float scale = 0.05f;
        for (uint32_t i = 0u; i < command_count; ++i) {
            dd.draw_transform_axes(
                pose.world_transforms[i].translation,
                pose.world_transforms[i].rotation,
                scale
            );
        }
    }

    /// Draw joint limit arcs for non-Fixed joints (3-line fan pattern in YELLOW).
    void draw_joint_limits(DebugDraw& dd, const SkeletalAsset& asset, const SkeletalPose& pose) {
        RenderColor limit_color = RenderColor::YELLOW();
        float arc_radius = 0.04f;
        float arc_angle = 0.5f; // ~28.6 degrees half-spread

        for (uint32_t i = 0u; i < command_count; ++i) {
            if (commands[i].joint_type == JointType::Fixed) continue;

            const Vec3& joint_pos = pose.world_transforms[i].translation;
            const Quat& joint_rot = pose.world_transforms[i].rotation;

            // Draw a small 3-line fan around the joint's first axis
            Mat3 rot = Mat3::from_quat(joint_rot);
            Vec3 axis = rot.transform_vec(Vec3(1.0f, 0.0f, 0.0f));

            // Find two perpendicular vectors to axis
            Vec3 up = Vec3(0.0f, 1.0f, 0.0f);
            if (std::abs(Vec3::dot(axis, up)) > 0.99f) {
                up = Vec3(0.0f, 0.0f, 1.0f);
            }
            Vec3 perp1 = Vec3::normalize(Vec3::cross(axis, up));
            Vec3 perp2 = Vec3::cross(axis, perp1);

            // 3 line segments approximating a partial circle
            Vec3 p0 = Vec3::add(joint_pos, Vec3::scale(perp1, arc_radius));
            float a1 = arc_angle;
            Vec3 p1 = Vec3::add(joint_pos, Vec3::add(
                Vec3::scale(perp1, arc_radius * std::cos(a1)),
                Vec3::scale(perp2, arc_radius * std::sin(a1))
            ));
            float a2 = arc_angle * 2.0f;
            Vec3 p2 = Vec3::add(joint_pos, Vec3::add(
                Vec3::scale(perp1, arc_radius * std::cos(a2)),
                Vec3::scale(perp2, arc_radius * std::sin(a2))
            ));

            dd.list.add_line(p0, p1, limit_color);
            dd.list.add_line(p1, p2, limit_color);
            dd.list.add_line(p0, p2, limit_color);
        }
    }

    /// Draw hierarchy lines: thin gray lines from each bone's start to parent start.
    void draw_hierarchy_lines(DebugDraw& dd) {
        RenderColor hier_color = RenderColor::GRAY();
        for (uint32_t i = 0u; i < command_count; ++i) {
            uint32_t pi = commands[i].parent_index;
            if (pi == 0xFFFFFFFFu || pi >= command_count) continue;
            dd.list.add_line(commands[i].start_position, commands[pi].start_position, hier_color);
        }
    }
};

// =============================================================================
// BlendModeColorCoding — Standalone utility for blend-mode color mapping
// =============================================================================
struct BlendModeColorCoding {
    /// Get the display color for a given blend mode.
    static RenderColor get_color(PhysicsBlendMode mode, float physics_weight = 0.0f) {
        RenderColor anim = RenderColor::BLUE();
        RenderColor physics = RenderColor::RED();
        switch (mode) {
        case PhysicsBlendMode::ANIM_DRIVEN:
            return anim;
        case PhysicsBlendMode::PHYSICS_DRIVEN:
            return physics;
        case PhysicsBlendMode::BLENDED:
            return RenderColor(
                anim.r + (physics.r - anim.r) * physics_weight,
                anim.g + (physics.g - anim.g) * physics_weight,
                anim.b + (physics.b - anim.b) * physics_weight,
                anim.a + (physics.a - anim.a) * physics_weight
            );
        default:
            return physics;
        }
    }

    /// Get a human-readable name for the blend mode.
    static const char* get_mode_name(PhysicsBlendMode mode) {
        switch (mode) {
        case PhysicsBlendMode::ANIM_DRIVEN:    return "ANIM_DRIVEN";
        case PhysicsBlendMode::PHYSICS_DRIVEN: return "PHYSICS_DRIVEN";
        case PhysicsBlendMode::BLENDED:        return "BLENDED";
        default:                                return "UNKNOWN";
        }
    }
};

// =============================================================================
// VelocityVisualizer — Draws linear/angular velocity arrows on RigidBody
// =============================================================================
struct VelocityVisualizer {
    float scale = 0.1f;
    RenderColor linear_color = RenderColor::GREEN();
    RenderColor angular_color = RenderColor::CYAN();

    /// Draw a velocity arrow from body position.
    void draw_velocity(uint32_t body_id, const RigidBody& body, DebugDraw& dd) {
        (void)body_id;
        dd.draw_velocity_arrow(body.position, body.linear_velocity, linear_color, scale);
    }

    /// Draw an angular velocity arc around the angular velocity axis at body position.
    /// If angular speed > epsilon, draws 3 line segments in a circular arc.
    void draw_angular_velocity(uint32_t body_id, const RigidBody& body, DebugDraw& dd) {
        (void)body_id;
        float speed = Vec3::length(body.angular_velocity);
        if (speed < APC_EPSILON) return;

        Vec3 axis = Vec3::scale(body.angular_velocity, 1.0f / speed);
        float radius = 0.05f;

        // Find two perpendicular vectors
        Vec3 up = Vec3(0.0f, 1.0f, 0.0f);
        if (std::abs(Vec3::dot(axis, up)) > 0.99f) {
            up = Vec3(0.0f, 0.0f, 1.0f);
        }
        Vec3 perp1 = Vec3::normalize(Vec3::cross(axis, up));
        Vec3 perp2 = Vec3::cross(axis, perp1);

        // 3-line arc (approx 120 degrees)
        float arc_step = APC_TWO_PI / 3.0f;
        Vec3 center = body.position;

        Vec3 p0 = Vec3::add(center, Vec3::scale(perp1, radius));
        Vec3 p1 = Vec3::add(center, Vec3::add(
            Vec3::scale(perp1, radius * std::cos(arc_step)),
            Vec3::scale(perp2, radius * std::sin(arc_step))
        ));
        Vec3 p2 = Vec3::add(center, Vec3::add(
            Vec3::scale(perp1, radius * std::cos(arc_step * 2.0f)),
            Vec3::scale(perp2, radius * std::sin(arc_step * 2.0f))
        ));

        dd.list.add_line(p0, p1, angular_color);
        dd.list.add_line(p1, p2, angular_color);
        dd.list.add_line(p2, p0, angular_color);
    }
};

// =============================================================================
// ForceVisualizer — Draws force/torque arcs on bones
// =============================================================================
struct ForceVisualizer {
    float scale = 0.01f;
    RenderColor force_color = RenderColor::ORANGE();
    RenderColor torque_color = RenderColor::YELLOW();

    /// Draw a force arrow from position.
    void draw_force(uint32_t bone_id, const Vec3& position, const ExternalBoneForce& ef, DebugDraw& dd) {
        (void)bone_id;
        dd.draw_force_arrow(position, ef.force, force_color, scale);
    }

    /// Draw a torque arc (3-line circle) around the torque axis at position.
    void draw_torque(uint32_t bone_id, const Vec3& position, const ExternalBoneForce& ef, DebugDraw& dd) {
        (void)bone_id;
        float mag = Vec3::length(ef.torque);
        if (mag < APC_EPSILON) return;

        Vec3 axis = Vec3::scale(ef.torque, 1.0f / mag);
        float radius = 0.04f;

        // Find two perpendicular vectors
        Vec3 up = Vec3(0.0f, 1.0f, 0.0f);
        if (std::abs(Vec3::dot(axis, up)) > 0.99f) {
            up = Vec3(0.0f, 0.0f, 1.0f);
        }
        Vec3 perp1 = Vec3::normalize(Vec3::cross(axis, up));
        Vec3 perp2 = Vec3::cross(axis, perp1);

        // 3-line arc
        float arc_step = APC_TWO_PI / 3.0f;
        Vec3 p0 = Vec3::add(position, Vec3::scale(perp1, radius));
        Vec3 p1 = Vec3::add(position, Vec3::add(
            Vec3::scale(perp1, radius * std::cos(arc_step)),
            Vec3::scale(perp2, radius * std::sin(arc_step))
        ));
        Vec3 p2 = Vec3::add(position, Vec3::add(
            Vec3::scale(perp1, radius * std::cos(arc_step * 2.0f)),
            Vec3::scale(perp2, radius * std::sin(arc_step * 2.0f))
        ));

        dd.list.add_line(p0, p1, torque_color);
        dd.list.add_line(p1, p2, torque_color);
        dd.list.add_line(p2, p0, torque_color);
    }
};

} // namespace apc
