// =============================================================================
// Sprint 18 Tests — Skeletal & Contact Visualization
// =============================================================================
//
// Tests for the APC render visualization headers (apc_render/ headers):
//   1. BoneVisualConfig defaults and factory
//   2. BoneVisualConfig get_bone_color (ANIM_DRIVEN, PHYSICS_DRIVEN, BLENDED)
//   3. SkeletonRenderer with 22-bone humanoid
//   4. SkeletonRenderer draw_bones produces correct line count
//   5. SkeletonRenderer hierarchy (3-bone chain)
//   6. BlendModeColorCoding utility
//   7. VelocityVisualizer (linear + angular velocity)
//   8. ForceVisualizer (force + torque)
//   9. ContactVisualizerConfig defaults
//   10. ImpulseHeatmap (green→yellow→red)
//   11. ContactVisualizer process and draw
//   12. Contact age fade (alpha decreases with age)
//
// Pattern: int main() + assert(), no test framework.
// =============================================================================

#include "apc_render/apc_skeleton_renderer.h"
#include "apc_render/apc_contact_visualizer.h"
#include "apc_render/apc_debug_draw.h"
#include "apc_skeleton/apc_skeleton_types.h"
#include "apc_skeleton/apc_skeletal_pose.h"
#include "apc_solver/apc_rigid_body.h"
#include "apc_solver/apc_contact_manager.h"
#include "apc_collision/apc_collision_dispatch.h"
#include <cassert>
#include <cstdio>
#include <cmath>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static constexpr float EPS = 1e-4f;

static bool approx_eq(float a, float b, float eps = EPS) {
    return std::abs(a - b) < eps;
}

// =============================================================================
// TEST 1: BoneVisualConfig defaults
// =============================================================================
static int test_bone_visual_config_defaults() {
    std::printf("  [Test 1] BoneVisualConfig defaults...\n");

    apc::BoneVisualConfig cfg;

    // Defaults
    assert(approx_eq(cfg.thickness, 0.02f) && "thickness = 0.02");
    assert(approx_eq(cfg.joint_sphere_radius, 0.03f) && "joint_sphere_radius = 0.03");
    assert(cfg.show_com == false && "show_com = false");
    assert(cfg.show_joint_axes == false && "show_joint_axes = false");
    assert(cfg.show_joint_limits == false && "show_joint_limits = false");
    assert(approx_eq(cfg.color_anim_driven.b, 1.0f) && "color_anim_driven = BLUE");
    assert(approx_eq(cfg.color_physics_driven.r, 1.0f) && "color_physics_driven = RED");
    assert(approx_eq(cfg.color_blended.r, 1.0f) && "color_blended = MAGENTA");
    assert(approx_eq(cfg.blend_gradient_weight, 0.0f) && "blend_gradient_weight = 0.0");

    // make_default factory
    apc::BoneVisualConfig def = apc::BoneVisualConfig::make_default();
    assert(approx_eq(def.thickness, 0.02f) && "make_default thickness");
    assert(approx_eq(def.joint_sphere_radius, 0.03f) && "make_default radius");
    assert(def.show_com == false && "make_default show_com");
    assert(approx_eq(def.color_anim_driven.b, 1.0f) && "make_default anim color");

    std::printf("    [PASS] BoneVisualConfig defaults and make_default verified\n");
    return 0;
}

// =============================================================================
// TEST 2: BoneVisualConfig get_bone_color
// =============================================================================
static int test_bone_visual_config_get_bone_color() {
    std::printf("  [Test 2] BoneVisualConfig get_bone_color...\n");

    apc::BoneVisualConfig cfg;

    // ANIM_DRIVEN → BLUE
    apc::RenderColor c_anim = cfg.get_bone_color(apc::PhysicsBlendMode::ANIM_DRIVEN);
    assert(approx_eq(c_anim.r, 0.0f) && "ANIM_DRIVEN r=0 (blue)");
    assert(approx_eq(c_anim.g, 0.0f) && "ANIM_DRIVEN g=0 (blue)");
    assert(approx_eq(c_anim.b, 1.0f) && "ANIM_DRIVEN b=1 (blue)");

    // PHYSICS_DRIVEN → RED
    apc::RenderColor c_phys = cfg.get_bone_color(apc::PhysicsBlendMode::PHYSICS_DRIVEN);
    assert(approx_eq(c_phys.r, 1.0f) && "PHYSICS_DRIVEN r=1 (red)");
    assert(approx_eq(c_phys.g, 0.0f) && "PHYSICS_DRIVEN g=0 (red)");
    assert(approx_eq(c_phys.b, 0.0f) && "PHYSICS_DRIVEN b=0 (red)");

    // BLENDED with weight=0 → anim color (blue)
    apc::RenderColor c_blend0 = cfg.get_bone_color(apc::PhysicsBlendMode::BLENDED, 0.0f);
    assert(approx_eq(c_blend0.r, 0.0f) && "BLENDED weight=0 r=0");
    assert(approx_eq(c_blend0.b, 1.0f) && "BLENDED weight=0 b=1");

    // BLENDED with weight=1 → physics color (red)
    apc::RenderColor c_blend1 = cfg.get_bone_color(apc::PhysicsBlendMode::BLENDED, 1.0f);
    assert(approx_eq(c_blend1.r, 1.0f) && "BLENDED weight=1 r=1");
    assert(approx_eq(c_blend1.b, 0.0f) && "BLENDED weight=1 b=0");

    // BLENDED with weight=0.5 → interpolated (midpoint)
    apc::RenderColor c_blend05 = cfg.get_bone_color(apc::PhysicsBlendMode::BLENDED, 0.5f);
    assert(approx_eq(c_blend05.r, 0.5f) && "BLENDED weight=0.5 r=0.5");
    assert(approx_eq(c_blend05.g, 0.0f) && "BLENDED weight=0.5 g=0");
    assert(approx_eq(c_blend05.b, 0.5f) && "BLENDED weight=0.5 b=0.5");

    // Custom colors
    cfg.color_anim_driven = apc::RenderColor::GREEN();
    cfg.color_physics_driven = apc::RenderColor::CYAN();
    apc::RenderColor c_custom = cfg.get_bone_color(apc::PhysicsBlendMode::ANIM_DRIVEN);
    assert(approx_eq(c_custom.g, 1.0f) && "Custom anim = GREEN");

    apc::RenderColor c_custom_blend = cfg.get_bone_color(apc::PhysicsBlendMode::BLENDED, 0.5f);
    // GREEN → CYAN blend: g=1.0 for both, r: 0→0 = 0, b: 0→1 = 0.5
    assert(approx_eq(c_custom_blend.r, 0.0f) && "Custom blend r=0");
    assert(approx_eq(c_custom_blend.g, 1.0f) && "Custom blend g=1.0");
    assert(approx_eq(c_custom_blend.b, 0.5f) && "Custom blend b=0.5");

    std::printf("    [PASS] BoneVisualConfig get_bone_color verified for all modes\n");
    return 0;
}

// =============================================================================
// TEST 3: SkeletonRenderer with 22-bone humanoid
// =============================================================================
static int test_skeleton_renderer_22_bones() {
    std::printf("  [Test 3] SkeletonRenderer with 22-bone humanoid...\n");

    apc::SkeletalAsset asset;
    apc::SkeletalPose pose;

    // Build a 22-bone humanoid as a chain from root
    for (uint32_t i = 0u; i < 22u; ++i) {
        apc::Bone bone;
        bone.parent_index = (i == 0u) ? 0xFFFFFFFFu : (i - 1u);
        bone.bind_pose = apc::Transform{apc::Vec3(0.0f, 0.1f, 0.0f), apc::Quat::identity()};
        bone.physics.mode = (i % 3 == 0) ? apc::PhysicsBlendMode::ANIM_DRIVEN :
                           (i % 3 == 1) ? apc::PhysicsBlendMode::PHYSICS_DRIVEN :
                                          apc::PhysicsBlendMode::BLENDED;
        bone.physics.stiffness = static_cast<float>(i) * 0.05f; // 0.0, 0.05, 0.10, ...
        bone.joint_type = (i == 0u) ? apc::JointType::Fixed : apc::JointType::Spherical3D;
        asset.bones.push_back(bone);
    }

    pose.allocate(22u);
    // Set world transforms as a vertical chain
    for (uint32_t i = 0u; i < 22u; ++i) {
        pose.world_transforms[i].translation = apc::Vec3(0.0f, static_cast<float>(i) * 0.1f, 0.0f);
        pose.world_transforms[i].rotation = apc::Quat::identity();
    }

    apc::SkeletonRenderer renderer;
    renderer.process_skeleton(asset, pose);

    assert(renderer.command_count == 22u && "command_count == 22");
    assert(renderer.command_count <= apc::SkeletonRenderer::MAX_BONES && "Within MAX_BONES");

    // Verify all commands have valid bone_index
    for (uint32_t i = 0u; i < 22u; ++i) {
        assert(renderer.commands[i].bone_index == i && "bone_index matches");
    }

    // Root bone (index 0) should have parent_index == 0xFFFFFFFF
    assert(renderer.commands[0].parent_index == 0xFFFFFFFFu && "Root parent = 0xFFFFFFFF");

    // Verify blend_mode propagation
    assert(renderer.commands[0].blend_mode == apc::PhysicsBlendMode::ANIM_DRIVEN && "Bone 0 ANIM_DRIVEN");
    assert(renderer.commands[1].blend_mode == apc::PhysicsBlendMode::PHYSICS_DRIVEN && "Bone 1 PHYSICS_DRIVEN");
    assert(renderer.commands[2].blend_mode == apc::PhysicsBlendMode::BLENDED && "Bone 2 BLENDED");

    // Verify colors match blend modes
    assert(approx_eq(renderer.commands[0].color.b, 1.0f) && "Bone 0 color = BLUE");
    assert(approx_eq(renderer.commands[1].color.r, 1.0f) && "Bone 1 color = RED");

    // Verify start positions
    assert(approx_eq(renderer.commands[0].start_position.y, 0.0f) && "Bone 0 start y=0");
    assert(approx_eq(renderer.commands[1].start_position.y, 0.1f) && "Bone 1 start y=0.1");

    // Verify end positions (chain: bone i's end = bone i+1's start)
    for (uint32_t i = 0u; i < 21u; ++i) {
        assert(approx_eq(renderer.commands[i].end_position.x,
                         renderer.commands[i + 1u].start_position.x, 0.001f)
               && "Chain: end[i] ≈ start[i+1] x");
        assert(approx_eq(renderer.commands[i].end_position.y,
                         renderer.commands[i + 1u].start_position.y, 0.001f)
               && "Chain: end[i] ≈ start[i+1] y");
        assert(approx_eq(renderer.commands[i].end_position.z,
                         renderer.commands[i + 1u].start_position.z, 0.001f)
               && "Chain: end[i] ≈ start[i+1] z");
    }

    // Verify physics_weight = stiffness (clamped to [0,1])
    assert(approx_eq(renderer.commands[0].physics_weight, 0.0f) && "Bone 0 physics_weight = 0.0");
    assert(approx_eq(renderer.commands[1].physics_weight, 0.05f) && "Bone 1 physics_weight = 0.05");

    std::printf("    [PASS] 22-bone humanoid processing verified\n");
    return 0;
}

// =============================================================================
// TEST 4: SkeletonRenderer draw_bones produces correct line count
// =============================================================================
static int test_skeleton_renderer_draw_bones() {
    std::printf("  [Test 4] SkeletonRenderer draw_bones produces correct line count...\n");

    apc::SkeletalAsset asset;
    apc::SkeletalPose pose;

    // 5-bone chain
    for (uint32_t i = 0u; i < 5u; ++i) {
        apc::Bone bone;
        bone.parent_index = (i == 0u) ? 0xFFFFFFFFu : (i - 1u);
        bone.bind_pose = apc::Transform{apc::Vec3(0.0f, 0.1f, 0.0f), apc::Quat::identity()};
        bone.physics.mode = apc::PhysicsBlendMode::PHYSICS_DRIVEN;
        asset.bones.push_back(bone);
    }

    pose.allocate(5u);
    for (uint32_t i = 0u; i < 5u; ++i) {
        pose.world_transforms[i].translation = apc::Vec3(0.0f, static_cast<float>(i) * 0.1f, 0.0f);
        pose.world_transforms[i].rotation = apc::Quat::identity();
    }

    apc::SkeletonRenderer renderer;
    renderer.process_skeleton(asset, pose);

    apc::DebugDraw dd;
    renderer.draw_bones(dd);
    assert(dd.list.get_line_count() >= 5u && "draw_bones: at least 5 lines");

    // draw_joint_spheres
    dd.clear();
    renderer.draw_joint_spheres(dd);
    assert(dd.list.get_point_count() == 5u && "draw_joint_spheres: 5 points");

    // draw_com_points
    dd.clear();
    renderer.draw_com_points(dd, asset);
    assert(dd.list.get_point_count() == 5u && "draw_com_points: 5 points");

    // draw_joint_axes (3 lines per bone = 15 lines)
    dd.clear();
    renderer.draw_joint_axes(dd, pose);
    assert(dd.list.get_line_count() == 5u * 3u && "draw_joint_axes: 15 lines");

    // draw_hierarchy_lines (4 parent-child connections for a 5-bone chain)
    dd.clear();
    renderer.draw_hierarchy_lines(dd);
    assert(dd.list.get_line_count() == 4u && "draw_hierarchy_lines: 4 lines (chain of 5)");

    std::printf("    [PASS] draw_bones, joint_spheres, com, axes, hierarchy verified\n");
    return 0;
}

// =============================================================================
// TEST 5: SkeletonRenderer hierarchy (3-bone chain)
// =============================================================================
static int test_skeleton_renderer_hierarchy() {
    std::printf("  [Test 5] SkeletonRenderer hierarchy (3-bone chain)...\n");

    apc::SkeletalAsset asset;
    apc::SkeletalPose pose;

    // 3-bone chain: root → child1 → child2
    for (uint32_t i = 0u; i < 3u; ++i) {
        apc::Bone bone;
        bone.parent_index = (i == 0u) ? 0xFFFFFFFFu : (i - 1u);
        bone.bind_pose = apc::Transform{apc::Vec3(0.0f, 0.15f, 0.0f), apc::Quat::identity()};
        bone.physics.mode = apc::PhysicsBlendMode::PHYSICS_DRIVEN;
        asset.bones.push_back(bone);
    }

    pose.allocate(3u);
    pose.world_transforms[0].translation = apc::Vec3(0.0f, 0.0f, 0.0f);
    pose.world_transforms[0].rotation = apc::Quat::identity();
    pose.world_transforms[1].translation = apc::Vec3(0.0f, 0.15f, 0.0f);
    pose.world_transforms[1].rotation = apc::Quat::identity();
    pose.world_transforms[2].translation = apc::Vec3(0.0f, 0.30f, 0.0f);
    pose.world_transforms[2].rotation = apc::Quat::identity();

    apc::SkeletonRenderer renderer;
    renderer.process_skeleton(asset, pose);

    assert(renderer.command_count == 3u && "3 commands");

    // Root bone: end = child1's start = (0, 0.15, 0)
    assert(approx_eq(renderer.commands[0].end_position.x, 0.0f) && "Root end x");
    assert(approx_eq(renderer.commands[0].end_position.y, 0.15f) && "Root end y = child1 start");
    assert(approx_eq(renderer.commands[0].end_position.z, 0.0f) && "Root end z");

    // Verify root end == child1 start
    assert(approx_eq(renderer.commands[0].end_position.x, renderer.commands[1].start_position.x) && "root end == child1 start x");
    assert(approx_eq(renderer.commands[0].end_position.y, renderer.commands[1].start_position.y) && "root end == child1 start y");
    assert(approx_eq(renderer.commands[0].end_position.z, renderer.commands[1].start_position.z) && "root end == child1 start z");

    // Child1 end == child2 start
    assert(approx_eq(renderer.commands[1].end_position.x, renderer.commands[2].start_position.x) && "child1 end == child2 start x");
    assert(approx_eq(renderer.commands[1].end_position.y, renderer.commands[2].start_position.y) && "child1 end == child2 start y");
    assert(approx_eq(renderer.commands[1].end_position.z, renderer.commands[2].start_position.z) && "child1 end == child2 start z");

    // Child2 has no children → fallback direction
    // Default fallback: start + rotation.rotate(Vec3(0, 0.1, 0)) = (0, 0.3) + (0, 0.1, 0) = (0, 0.4, 0)
    assert(approx_eq(renderer.commands[2].end_position.y, 0.40f, 0.001f) && "Child2 end uses fallback = (0, 0.4, 0)");

    // Parent indices
    assert(renderer.commands[0].parent_index == 0xFFFFFFFFu && "Root parent = invalid");
    assert(renderer.commands[1].parent_index == 0u && "Child1 parent = 0");
    assert(renderer.commands[2].parent_index == 1u && "Child2 parent = 1");

    std::printf("    [PASS] 3-bone chain hierarchy verified\n");
    return 0;
}

// =============================================================================
// TEST 6: BlendModeColorCoding utility
// =============================================================================
static int test_blend_mode_color_coding() {
    std::printf("  [Test 6] BlendModeColorCoding utility...\n");

    // ANIM_DRIVEN → BLUE
    apc::RenderColor c_anim = apc::BlendModeColorCoding::get_color(apc::PhysicsBlendMode::ANIM_DRIVEN);
    assert(approx_eq(c_anim.b, 1.0f) && "ANIM_DRIVEN = BLUE");

    // PHYSICS_DRIVEN → RED
    apc::RenderColor c_phys = apc::BlendModeColorCoding::get_color(apc::PhysicsBlendMode::PHYSICS_DRIVEN);
    assert(approx_eq(c_phys.r, 1.0f) && "PHYSICS_DRIVEN = RED");

    // BLENDED weight=0 → BLUE
    apc::RenderColor c_b0 = apc::BlendModeColorCoding::get_color(apc::PhysicsBlendMode::BLENDED, 0.0f);
    assert(approx_eq(c_b0.b, 1.0f) && "BLENDED w=0 = BLUE");

    // BLENDED weight=1 → RED
    apc::RenderColor c_b1 = apc::BlendModeColorCoding::get_color(apc::PhysicsBlendMode::BLENDED, 1.0f);
    assert(approx_eq(c_b1.r, 1.0f) && "BLENDED w=1 = RED");

    // BLENDED weight=0.5 → midpoint
    apc::RenderColor c_b05 = apc::BlendModeColorCoding::get_color(apc::PhysicsBlendMode::BLENDED, 0.5f);
    assert(approx_eq(c_b05.r, 0.5f) && "BLENDED w=0.5 r=0.5");
    assert(approx_eq(c_b05.b, 0.5f) && "BLENDED w=0.5 b=0.5");

    // get_mode_name
    assert(std::strcmp(apc::BlendModeColorCoding::get_mode_name(apc::PhysicsBlendMode::ANIM_DRIVEN),
                       "ANIM_DRIVEN") == 0 && "mode name ANIM_DRIVEN");
    assert(std::strcmp(apc::BlendModeColorCoding::get_mode_name(apc::PhysicsBlendMode::PHYSICS_DRIVEN),
                       "PHYSICS_DRIVEN") == 0 && "mode name PHYSICS_DRIVEN");
    assert(std::strcmp(apc::BlendModeColorCoding::get_mode_name(apc::PhysicsBlendMode::BLENDED),
                       "BLENDED") == 0 && "mode name BLENDED");

    std::printf("    [PASS] BlendModeColorCoding colors and names verified\n");
    return 0;
}

// =============================================================================
// TEST 7: VelocityVisualizer
// =============================================================================
static int test_velocity_visualizer() {
    std::printf("  [Test 7] VelocityVisualizer...\n");

    apc::VelocityVisualizer viz;
    apc::DebugDraw dd;

    // Non-zero velocity: should produce 3 lines (shaft + 2 arrowhead)
    apc::RigidBody body;
    body.position = apc::Vec3(0.0f, 0.0f, 0.0f);
    body.linear_velocity = apc::Vec3(0.0f, 5.0f, 0.0f);
    body.orientation = apc::Quat::identity();
    body.angular_velocity = apc::Vec3(0.0f, 0.0f, 0.0f);

    viz.draw_velocity(0u, body, dd);
    assert(dd.list.get_line_count() == 3u && "Non-zero velocity: 3 lines (arrow)");

    dd.clear();

    // Zero velocity: should produce 0 lines
    apc::RigidBody zero_body;
    zero_body.position = apc::Vec3(1.0f, 2.0f, 3.0f);
    zero_body.linear_velocity = apc::Vec3(0.0f, 0.0f, 0.0f);
    zero_body.angular_velocity = apc::Vec3(0.0f, 0.0f, 0.0f);

    viz.draw_velocity(1u, zero_body, dd);
    assert(dd.list.get_line_count() == 0u && "Zero velocity: 0 lines");

    dd.clear();

    // Angular velocity: should produce 3 lines (arc)
    body.angular_velocity = apc::Vec3(1.0f, 0.0f, 0.0f);
    viz.draw_angular_velocity(0u, body, dd);
    assert(dd.list.get_line_count() == 3u && "Non-zero angular velocity: 3 lines (arc)");

    dd.clear();

    // Zero angular velocity: should produce 0 lines
    viz.draw_angular_velocity(1u, zero_body, dd);
    assert(dd.list.get_line_count() == 0u && "Zero angular velocity: 0 lines");

    // Verify arrow uses correct color
    dd.clear();
    viz.draw_velocity(0u, body, dd);
    assert(approx_eq(dd.list.lines[0].a.color.g, 1.0f) && "Linear velocity color = GREEN");

    std::printf("    [PASS] VelocityVisualizer linear/angular velocity verified\n");
    return 0;
}

// =============================================================================
// TEST 8: ForceVisualizer
// =============================================================================
static int test_force_visualizer() {
    std::printf("  [Test 8] ForceVisualizer...\n");

    apc::ForceVisualizer viz;
    apc::DebugDraw dd;

    // Non-zero force: should produce 3 lines (arrow)
    apc::ExternalBoneForce ef;
    ef.force = apc::Vec3(10.0f, 0.0f, 0.0f);
    ef.torque = apc::Vec3(0.0f, 5.0f, 0.0f);
    apc::Vec3 pos(0.0f, 1.0f, 0.0f);

    viz.draw_force(0u, pos, ef, dd);
    assert(dd.list.get_line_count() == 3u && "Non-zero force: 3 lines (arrow)");

    dd.clear();

    // Zero force: should produce 0 lines (or very small)
    apc::ExternalBoneForce zero_ef;
    zero_ef.force = apc::Vec3(0.0f, 0.0f, 0.0f);
    zero_ef.torque = apc::Vec3(0.0f, 0.0f, 0.0f);

    viz.draw_force(1u, pos, zero_ef, dd);
    assert(dd.list.get_line_count() == 0u && "Zero force: 0 lines");

    dd.clear();

    // Non-zero torque: should produce 3 lines (arc)
    viz.draw_torque(0u, pos, ef, dd);
    assert(dd.list.get_line_count() == 3u && "Non-zero torque: 3 lines (arc)");

    dd.clear();

    // Zero torque: should produce 0 lines
    viz.draw_torque(1u, pos, zero_ef, dd);
    assert(dd.list.get_line_count() == 0u && "Zero torque: 0 lines");

    // Verify force arrow color = ORANGE
    dd.clear();
    viz.draw_force(0u, pos, ef, dd);
    assert(approx_eq(dd.list.lines[0].a.color.r, 1.0f) && "Force color r=1");
    assert(approx_eq(dd.list.lines[0].a.color.g, 0.5f) && "Force color g=0.5 (ORANGE)");

    std::printf("    [PASS] ForceVisualizer force/torque verified\n");
    return 0;
}

// =============================================================================
// TEST 9: ContactVisualizerConfig defaults
// =============================================================================
static int test_contact_visualizer_config() {
    std::printf("  [Test 9] ContactVisualizerConfig defaults...\n");

    apc::ContactVisualizerConfig cfg;

    assert(cfg.show_contacts == true && "show_contacts = true");
    assert(cfg.show_normals == true && "show_normals = true");
    assert(cfg.show_penetration == true && "show_penetration = true");
    assert(cfg.show_impulse_heatmap == false && "show_impulse_heatmap = false");
    assert(cfg.show_friction == false && "show_friction = false");
    assert(cfg.contact_age_threshold == 100u && "contact_age_threshold = 100");
    assert(approx_eq(cfg.impulse_color_scale, 100.0f) && "impulse_color_scale = 100");
    assert(approx_eq(cfg.normal_arrow_length, 0.2f) && "normal_arrow_length = 0.2");
    assert(approx_eq(cfg.penetration_depth_scale, 1.0f) && "penetration_depth_scale = 1.0");
    assert(approx_eq(cfg.contact_point_size, 0.05f) && "contact_point_size = 0.05");

    // make_default factory
    apc::ContactVisualizerConfig def = apc::ContactVisualizerConfig::make_default();
    assert(def.show_contacts == true && "make_default show_contacts");
    assert(def.contact_age_threshold == 100u && "make_default age_threshold");
    assert(approx_eq(def.impulse_color_scale, 100.0f) && "make_default impulse_scale");

    std::printf("    [PASS] ContactVisualizerConfig defaults and make_default verified\n");
    return 0;
}

// =============================================================================
// TEST 10: ImpulseHeatmap
// =============================================================================
static int test_impulse_heatmap() {
    std::printf("  [Test 10] ImpulseHeatmap...\n");

    apc::ImpulseHeatmap hm;

    // impulse = 0 → green (low)
    apc::RenderColor c0 = hm.evaluate(0.0f);
    assert(approx_eq(c0.r, 0.0f) && "impulse=0 r=0");
    assert(approx_eq(c0.g, 1.0f) && "impulse=0 g=1 (GREEN)");
    assert(approx_eq(c0.b, 0.0f) && "impulse=0 b=0");

    // impulse = scale/2 → yellow (mid)
    apc::RenderColor c50 = hm.evaluate(50.0f);
    assert(approx_eq(c50.r, 1.0f) && "impulse=50 r=1 (YELLOW)");
    assert(approx_eq(c50.g, 1.0f) && "impulse=50 g=1 (YELLOW)");
    assert(approx_eq(c50.b, 0.0f) && "impulse=50 b=0 (YELLOW)");

    // impulse = scale → red (high)
    apc::RenderColor c100 = hm.evaluate(100.0f);
    assert(approx_eq(c100.r, 1.0f) && "impulse=100 r=1 (RED)");
    assert(approx_eq(c100.g, 0.0f) && "impulse=100 g=0 (RED)");
    assert(approx_eq(c100.b, 0.0f) && "impulse=100 b=0 (RED)");

    // impulse = 2*scale → clamped to red
    apc::RenderColor c200 = hm.evaluate(200.0f);
    assert(approx_eq(c200.r, 1.0f) && "impulse=200 clamped r=1");
    assert(approx_eq(c200.g, 0.0f) && "impulse=200 clamped g=0");
    assert(approx_eq(c200.b, 0.0f) && "impulse=200 clamped b=0");

    // impulse = negative → clamped to green
    apc::RenderColor cneg = hm.evaluate(-10.0f);
    assert(approx_eq(cneg.r, 0.0f) && "impulse=-10 clamped r=0");
    assert(approx_eq(cneg.g, 1.0f) && "impulse=-10 clamped g=1");
    assert(approx_eq(cneg.b, 0.0f) && "impulse=-10 clamped b=0");

    // impulse = scale/4 → between green and yellow
    apc::RenderColor c25 = hm.evaluate(25.0f);
    // t = 0.25, in low half → s = 0.5, lerp(green, yellow, 0.5)
    assert(approx_eq(c25.r, 0.5f) && "impulse=25 r=0.5");
    assert(approx_eq(c25.g, 1.0f) && "impulse=25 g=1.0");
    assert(approx_eq(c25.b, 0.0f) && "impulse=25 b=0");

    // impulse = 75 → between yellow and red
    apc::RenderColor c75 = hm.evaluate(75.0f);
    // t = 0.75, in high half → s = 0.5, lerp(yellow, red, 0.5)
    assert(approx_eq(c75.r, 1.0f) && "impulse=75 r=1");
    assert(approx_eq(c75.g, 0.5f) && "impulse=75 g=0.5");
    assert(approx_eq(c75.b, 0.0f) && "impulse=75 b=0");

    std::printf("    [PASS] ImpulseHeatmap green→yellow→red ramp verified\n");
    return 0;
}

// =============================================================================
// TEST 11: ContactVisualizer process and draw
// =============================================================================
static int test_contact_visualizer_process_and_draw() {
    std::printf("  [Test 11] ContactVisualizer process and draw...\n");

    // Create a ContactManager and populate it with 2 manifolds (3 contacts total)
    apc::ContactManager cm;

    // Manifold 1: 2 contacts
    apc::ContactManifold m1;
    m1.id_a = 0;
    m1.id_b = 1;
    m1.contact_count = 2;
    m1.contacts[0].point_on_a = apc::Vec3(0.0f, 0.0f, 0.0f);
    m1.contacts[0].point_on_b = apc::Vec3(0.0f, 0.1f, 0.0f);
    m1.contacts[0].normal = apc::Vec3(0.0f, 1.0f, 0.0f);
    m1.contacts[0].penetration = 0.05f;
    m1.contacts[1].point_on_a = apc::Vec3(0.1f, 0.0f, 0.0f);
    m1.contacts[1].point_on_b = apc::Vec3(0.1f, 0.08f, 0.0f);
    m1.contacts[1].normal = apc::Vec3(0.0f, 1.0f, 0.0f);
    m1.contacts[1].penetration = 0.03f;

    // Manifold 2: 1 contact
    apc::ContactManifold m2;
    m2.id_a = 1;
    m2.id_b = 2;
    m2.contact_count = 1;
    m2.contacts[0].point_on_a = apc::Vec3(1.0f, 0.0f, 0.0f);
    m2.contacts[0].point_on_b = apc::Vec3(1.0f, 0.05f, 0.0f);
    m2.contacts[0].normal = apc::Vec3(0.0f, 1.0f, 0.0f);
    m2.contacts[0].penetration = 0.02f;

    apc::ContactManifold manifolds[2] = {m1, m2};
    cm.update(manifolds, 2);

    // Process contacts
    apc::ContactVisualizer viz;
    viz.process_contacts(cm);

    assert(viz.command_count == 3u && "command_count == 3");

    // Verify contact data
    assert(viz.commands[0].body_a == 0u && "Contact 0 body_a = 0");
    assert(viz.commands[0].body_b == 1u && "Contact 0 body_b = 1");
    assert(approx_eq(viz.commands[0].penetration, 0.05f) && "Contact 0 penetration = 0.05");
    assert(approx_eq(viz.commands[1].penetration, 0.03f) && "Contact 1 penetration = 0.03");
    assert(approx_eq(viz.commands[2].penetration, 0.02f) && "Contact 2 penetration = 0.02");

    // Verify point positions
    assert(approx_eq(viz.commands[0].point_a.x, 0.0f) && "Contact 0 point_a.x = 0");
    assert(approx_eq(viz.commands[0].point_a.y, 0.0f) && "Contact 0 point_a.y = 0");
    assert(approx_eq(viz.commands[1].point_a.x, 0.1f) && "Contact 1 point_a.x = 0.1");
    assert(approx_eq(viz.commands[2].point_a.x, 1.0f) && "Contact 2 point_a.x = 1.0");

    // Verify age = 0 (new contacts)
    assert(viz.commands[0].age == 0u && "Contact 0 age = 0");
    assert(viz.commands[1].age == 0u && "Contact 1 age = 0");
    assert(viz.commands[2].age == 0u && "Contact 2 age = 0");

    // Verify get_contact_count and get_commands
    assert(viz.get_contact_count() == 3u && "get_contact_count = 3");
    assert(viz.get_commands() != nullptr && "get_commands != nullptr");

    // draw_contact_points: each contact = 4 lines (X marker)
    apc::DebugDraw dd;
    viz.draw_contact_points(dd);
    assert(dd.list.get_line_count() == 3u * 4u && "draw_contact_points: 12 lines");

    // draw_normal_arrows: each contact = 1 line
    dd.clear();
    viz.draw_normal_arrows(dd);
    assert(dd.list.get_line_count() == 3u && "draw_normal_arrows: 3 lines");

    // draw_penetration_indicators: 3 contacts with penetration > 0 → 3 lines
    dd.clear();
    viz.draw_penetration_indicators(dd);
    assert(dd.list.get_line_count() == 3u && "draw_penetration_indicators: 3 lines");

    // draw_impulse_heatmap: 3 points
    dd.clear();
    viz.draw_impulse_heatmap(dd);
    assert(dd.list.get_point_count() == 3u && "draw_impulse_heatmap: 3 points");

    // draw_friction_vectors: 0 friction impulses → 0 lines
    dd.clear();
    viz.draw_friction_vectors(dd);
    assert(dd.list.get_line_count() == 0u && "draw_friction_vectors: 0 lines (no friction)");

    std::printf("    [PASS] ContactVisualizer process and draw verified\n");
    return 0;
}

// =============================================================================
// TEST 12: Contact age fade
// =============================================================================
static int test_contact_age_fade() {
    std::printf("  [Test 12] Contact age fade...\n");

    // Create a ContactManager with contacts at different ages
    apc::ContactManager cm;

    // Manifold with 3 contacts: age 0, 50, 100 (via multiple updates)
    apc::ContactManifold m1;
    m1.id_a = 0;
    m1.id_b = 1;
    m1.contact_count = 3;
    m1.contacts[0].point_on_a = apc::Vec3(0.0f, 0.0f, 0.0f);
    m1.contacts[0].point_on_b = apc::Vec3(0.0f, 0.1f, 0.0f);
    m1.contacts[0].normal = apc::Vec3(0.0f, 1.0f, 0.0f);
    m1.contacts[0].penetration = 0.01f;
    m1.contacts[1].point_on_a = apc::Vec3(0.5f, 0.0f, 0.0f);
    m1.contacts[1].point_on_b = apc::Vec3(0.5f, 0.08f, 0.0f);
    m1.contacts[1].normal = apc::Vec3(0.0f, 1.0f, 0.0f);
    m1.contacts[1].penetration = 0.02f;
    m1.contacts[2].point_on_a = apc::Vec3(1.0f, 0.0f, 0.0f);
    m1.contacts[2].point_on_b = apc::Vec3(1.0f, 0.07f, 0.0f);
    m1.contacts[2].normal = apc::Vec3(0.0f, 1.0f, 0.0f);
    m1.contacts[2].penetration = 0.03f;

    // First update creates contacts with age 0
    cm.update(&m1, 1);

    // Process — all contacts should have age 0, alpha = 1.0
    apc::ContactVisualizer viz;
    viz.process_contacts(cm);
    assert(viz.command_count == 3u && "Initial: 3 contacts");

    float alpha_age0 = viz.commands[0].color.a;
    assert(approx_eq(alpha_age0, 1.0f) && "Age 0: alpha = 1.0");

    // Second update — ages become 1
    cm.update(&m1, 1);
    viz.process_contacts(cm);
    assert(viz.command_count == 3u && "After update: 3 contacts");
    float alpha_age1 = viz.commands[0].color.a;
    // alpha = 1.0 - 1/100 = 0.99
    assert(approx_eq(alpha_age1, 0.99f, 0.01f) && "Age 1: alpha ≈ 0.99");

    // Test with different age thresholds
    viz.config.contact_age_threshold = 50u;
    viz.process_contacts(cm);
    assert(viz.command_count == 3u && "Threshold 50: 3 contacts");

    float alpha_thresh = viz.commands[0].color.a;
    // alpha = 1.0 - 1/50 = 0.98
    assert(approx_eq(alpha_thresh, 0.98f, 0.01f) && "Threshold 50: alpha ≈ 0.98");

    // Test with very low threshold (should filter out aged contacts)
    // Need contacts with age > threshold to test filtering
    viz.config.contact_age_threshold = 0u;
    viz.process_contacts(cm);
    // All contacts have age 1 > 0, so they should be filtered out
    assert(viz.command_count == 0u && "Threshold 0: all contacts filtered (age > 0)");

    // Verify alpha ordering: newer contacts have higher alpha
    viz.config.contact_age_threshold = 100u;
    // Manually set up a ContactManager with different ages
    apc::ContactManager cm2;
    apc::ContactManifold m2;
    m2.id_a = 0;
    m2.id_b = 1;
    m2.contact_count = 2;
    m2.contacts[0].point_on_a = apc::Vec3(0.0f, 0.0f, 0.0f);
    m2.contacts[0].point_on_b = apc::Vec3(0.0f, 0.1f, 0.0f);
    m2.contacts[0].normal = apc::Vec3(0.0f, 1.0f, 0.0f);
    m2.contacts[0].penetration = 0.01f;
    m2.contacts[1].point_on_a = apc::Vec3(0.5f, 0.0f, 0.0f);
    m2.contacts[1].point_on_b = apc::Vec3(0.5f, 0.08f, 0.0f);
    m2.contacts[1].normal = apc::Vec3(0.0f, 1.0f, 0.0f);
    m2.contacts[1].penetration = 0.02f;

    cm2.update(&m2, 1);

    // After 25 updates, age should be 25
    for (int i = 0; i < 25; ++i) {
        cm2.update(&m2, 1);
    }

    viz.process_contacts(cm2);
    assert(viz.command_count == 2u && "After 26 updates: 2 contacts");

    float alpha_26 = viz.commands[0].color.a;
    // alpha = 1.0 - 26/100 = 0.74
    assert(alpha_26 < 1.0f && "Aged contact alpha < 1.0");
    assert(alpha_26 > 0.5f && "Aged contact alpha > 0.5 (age=26, threshold=100)");

    // After 50 more updates, total age ~76
    for (int i = 0; i < 50; ++i) {
        cm2.update(&m2, 1);
    }
    viz.process_contacts(cm2);
    assert(viz.command_count == 2u && "After 76 updates: 2 contacts");

    float alpha_76 = viz.commands[0].color.a;
    assert(alpha_76 < alpha_26 && "Older contact has lower alpha");
    assert(alpha_76 > 0.0f && "Alpha still > 0 (age 76 < threshold 100)");

    std::printf("    [PASS] Contact age fade (alpha decreases with age) verified\n");
    return 0;
}

// =============================================================================
// MAIN
// =============================================================================
int main() {
    std::printf("=== Sprint 18: Skeletal & Contact Visualization ===\n\n");

    int result = 0;
    result |= test_bone_visual_config_defaults();
    result |= test_bone_visual_config_get_bone_color();
    result |= test_skeleton_renderer_22_bones();
    result |= test_skeleton_renderer_draw_bones();
    result |= test_skeleton_renderer_hierarchy();
    result |= test_blend_mode_color_coding();
    result |= test_velocity_visualizer();
    result |= test_force_visualizer();
    result |= test_contact_visualizer_config();
    result |= test_impulse_heatmap();
    result |= test_contact_visualizer_process_and_draw();
    result |= test_contact_age_fade();

    std::printf("\n");
    if (result == 0) {
        std::printf("Sprint 18: ALL TESTS PASSED\n");
    } else {
        std::printf("Sprint 18: SOME TESTS FAILED\n");
    }

    return result;
}
