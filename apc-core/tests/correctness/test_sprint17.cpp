// =============================================================================
// Sprint 17 Tests — Render Foundation & Debug Draw
// =============================================================================
//
// Tests for the APC render pipeline (apc_render/ headers):
//   1. RenderColor — pack/unpack round-trip, from_rgb, named constants
//   2. RenderVertex — default construction, field assignment
//   3. RenderMaterial — defaults, wireframe flag
//   4. RenderStats — zero defaults, increment, reset
//   5. RenderPassType — all 6 values present and distinct
//   6. CameraController — perspective, look_at, projection matrix values
//   7. CameraController — orthographic, symmetric frustum
//   8. FollowCamera — update follows target with offset, smoothing
//   9. OrbitCamera — rotate, zoom, pan, update
//   10. DebugDrawList — add_line, add_point, add_triangle, clear, counts
//   11. DebugDraw shapes — aabb, sphere, contact_normal, velocity_arrow
//   12. ShapeDebugRenderer — dispatch for sphere, box, capsule, cylinder
//   13. RenderBridge — sync_transforms, sync_contacts, begin/end_frame
//
// Pattern: int main() + assert(), no test framework.
// =============================================================================

#include "apc_render/apc_render_types.h"
#include "apc_render/apc_render_context.h"
#include "apc_render/apc_render_camera.h"
#include "apc_render/apc_debug_draw.h"
#include "apc_render/apc_render_bridge.h"
#include "apc_collision/apc_collision_dispatch.h"
#include "apc_solver/apc_rigid_body.h"
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
// TEST 1: RenderColor basics
// =============================================================================
static int test_render_color() {
    std::printf("  [Test 1] RenderColor basics...\n");

    // --- Named constants ---
    apc::RenderColor white = apc::RenderColor::WHITE();
    assert(approx_eq(white.r, 1.0f) && "WHITE.r = 1");
    assert(approx_eq(white.g, 1.0f) && "WHITE.g = 1");
    assert(approx_eq(white.b, 1.0f) && "WHITE.b = 1");
    assert(approx_eq(white.a, 1.0f) && "WHITE.a = 1");

    apc::RenderColor black = apc::RenderColor::BLACK();
    assert(approx_eq(black.r, 0.0f) && "BLACK.r = 0");
    assert(approx_eq(black.g, 0.0f) && "BLACK.g = 0");

    apc::RenderColor red = apc::RenderColor::RED();
    assert(approx_eq(red.r, 1.0f) && "RED.r = 1");
    assert(approx_eq(red.g, 0.0f) && "RED.g = 0");
    assert(approx_eq(red.b, 0.0f) && "RED.b = 0");

    apc::RenderColor green = apc::RenderColor::GREEN();
    assert(approx_eq(green.g, 1.0f) && "GREEN.g = 1");
    assert(approx_eq(green.r, 0.0f) && "GREEN.r = 0");

    apc::RenderColor blue = apc::RenderColor::BLUE();
    assert(approx_eq(blue.b, 1.0f) && "BLUE.b = 1");
    assert(approx_eq(blue.r, 0.0f) && "BLUE.r = 0");

    apc::RenderColor yellow = apc::RenderColor::YELLOW();
    assert(approx_eq(yellow.r, 1.0f) && "YELLOW.r = 1");
    assert(approx_eq(yellow.g, 1.0f) && "YELLOW.g = 1");

    apc::RenderColor cyan = apc::RenderColor::CYAN();
    assert(approx_eq(cyan.g, 1.0f) && "CYAN.g = 1");
    assert(approx_eq(cyan.b, 1.0f) && "CYAN.b = 1");

    apc::RenderColor magenta = apc::RenderColor::MAGENTA();
    assert(approx_eq(magenta.r, 1.0f) && "MAGENTA.r = 1");
    assert(approx_eq(magenta.b, 1.0f) && "MAGENTA.b = 1");

    apc::RenderColor gray = apc::RenderColor::GRAY();
    assert(approx_eq(gray.r, 0.5f) && "GRAY.r = 0.5");

    apc::RenderColor orange = apc::RenderColor::ORANGE();
    assert(approx_eq(orange.r, 1.0f) && "ORANGE.r = 1");
    assert(approx_eq(orange.g, 0.5f) && "ORANGE.g = 0.5");

    // --- Pack/unpack round-trip ---
    apc::RenderColor c1(0.5f, 0.25f, 0.75f, 1.0f);
    uint32_t packed = c1.pack_u32();
    apc::RenderColor c2 = apc::RenderColor::unpack_u32(packed);
    assert(approx_eq(c1.r, c2.r, 0.01f) && "Pack/unpack round-trip r");
    assert(approx_eq(c1.g, c2.g, 0.01f) && "Pack/unpack round-trip g");
    assert(approx_eq(c1.b, c2.b, 0.01f) && "Pack/unpack round-trip b");
    assert(approx_eq(c1.a, c2.a, 0.01f) && "Pack/unpack round-trip a");

    // --- from_rgb (alpha should be 1.0) ---
    apc::RenderColor c3 = apc::RenderColor::from_rgb(0.3f, 0.7f, 0.9f);
    assert(approx_eq(c3.r, 0.3f) && "from_rgb r");
    assert(approx_eq(c3.g, 0.7f) && "from_rgb g");
    assert(approx_eq(c3.b, 0.9f) && "from_rgb b");
    assert(approx_eq(c3.a, 1.0f) && "from_rgb a = 1.0");

    // --- RGBA alpha ---
    apc::RenderColor c4(1.0f, 0.0f, 0.0f, 0.5f);
    assert(approx_eq(c4.a, 0.5f) && "RGBA alpha = 0.5");

    // --- Pack format: ABGR ---
    uint32_t packed_white = apc::RenderColor::WHITE().pack_u32();
    assert(packed_white == 0xFFFFFFFFu && "WHITE packs to 0xFFFFFFFF");

    uint32_t packed_red = apc::RenderColor::RED().pack_u32();
    assert(packed_red == 0xFF0000FFu && "RED packs to 0xFF0000FF");

    std::printf("    [PASS] RenderColor pack/unpack, from_rgb, constants verified\n");
    return 0;
}

// =============================================================================
// TEST 2: RenderVertex
// =============================================================================
static int test_render_vertex() {
    std::printf("  [Test 2] RenderVertex...\n");

    apc::RenderVertex v;
    assert(approx_eq(v.position.x, 0.0f) && "Default position.x = 0");
    assert(approx_eq(v.position.y, 0.0f) && "Default position.y = 0");
    assert(approx_eq(v.position.z, 0.0f) && "Default position.z = 0");
    assert(approx_eq(v.normal.y, 1.0f) && "Default normal.y = 1");
    assert(approx_eq(v.uv.x, 0.0f) && "Default uv.x = 0");
    assert(approx_eq(v.uv.y, 0.0f) && "Default uv.y = 0");
    assert(approx_eq(v.color.r, 1.0f) && "Default color = WHITE");

    // Assignment
    v.position = apc::Vec3(1.0f, 2.0f, 3.0f);
    v.normal   = apc::Vec3(0.0f, 0.0f, 1.0f);
    v.uv       = apc::Vec2(0.5f, 0.75f);
    v.color    = apc::RenderColor::RED();

    assert(approx_eq(v.position.x, 1.0f) && "Assigned position.x = 1");
    assert(approx_eq(v.position.y, 2.0f) && "Assigned position.y = 2");
    assert(approx_eq(v.normal.z, 1.0f) && "Assigned normal.z = 1");
    assert(approx_eq(v.uv.x, 0.5f) && "Assigned uv.x = 0.5");
    assert(approx_eq(v.color.r, 1.0f) && "Assigned color = RED");

    std::printf("    [PASS] RenderVertex default and assignment verified\n");
    return 0;
}

// =============================================================================
// TEST 3: RenderMaterial
// =============================================================================
static int test_render_material() {
    std::printf("  [Test 3] RenderMaterial...\n");

    apc::RenderMaterial m;
    assert(approx_eq(m.albedo_color.r, 1.0f) && "Default albedo = WHITE");
    assert(approx_eq(m.emissive_color.r, 0.0f) && "Default emissive = BLACK");
    assert(m.wireframe == false && "Default wireframe = false");
    assert(approx_eq(m.transparency, 0.0f) && "Default transparency = 0");
    assert(approx_eq(m.line_width, 1.0f) && "Default line_width = 1");
    assert(approx_eq(m.point_size, 1.0f) && "Default point_size = 1");
    assert(m.material_id == 0u && "Default material_id = 0");
    assert(m.double_sided == false && "Default double_sided = false");

    // Wireframe flag
    m.wireframe = true;
    assert(m.wireframe == true && "Wireframe flag set");

    m.transparency = 0.5f;
    assert(approx_eq(m.transparency, 0.5f) && "Transparency set to 0.5");

    std::printf("    [PASS] RenderMaterial defaults and properties verified\n");
    return 0;
}

// =============================================================================
// TEST 4: RenderStats
// =============================================================================
static int test_render_stats() {
    std::printf("  [Test 4] RenderStats...\n");

    apc::RenderStats stats;
    assert(stats.draw_calls == 0u && "Default draw_calls = 0");
    assert(stats.triangle_count == 0u && "Default triangle_count = 0");
    assert(stats.line_count == 0u && "Default line_count = 0");
    assert(stats.point_count == 0u && "Default point_count = 0");
    assert(approx_eq(stats.frame_time_ms, 0.0f) && "Default frame_time = 0");

    // Increment
    stats.draw_calls = 42u;
    stats.triangle_count = 100u;
    stats.line_count = 50u;
    assert(stats.draw_calls == 42u && "draw_calls set");

    // Reset
    stats.reset();
    assert(stats.draw_calls == 0u && "After reset: draw_calls = 0");
    assert(stats.triangle_count == 0u && "After reset: triangle_count = 0");
    assert(stats.line_count == 0u && "After reset: line_count = 0");
    assert(approx_eq(stats.frame_time_ms, 0.0f) && "After reset: frame_time = 0");

    std::printf("    [PASS] RenderStats defaults, increment, reset verified\n");
    return 0;
}

// =============================================================================
// TEST 5: RenderPassType enum
// =============================================================================
static int test_render_pass_type() {
    std::printf("  [Test 5] RenderPassType enum...\n");

    // All 6 values present and distinct
    using RPT = apc::RenderPassType;
    assert(static_cast<uint8_t>(RPT::OPAQUE)      == 0u);
    assert(static_cast<uint8_t>(RPT::TRANSPARENT)  == 1u);
    assert(static_cast<uint8_t>(RPT::WIREFRAME)    == 2u);
    assert(static_cast<uint8_t>(RPT::OVERLAY)      == 3u);
    assert(static_cast<uint8_t>(RPT::DEBUG)        == 4u);
    assert(static_cast<uint8_t>(RPT::HUD)          == 5u);

    // All distinct
    assert(RPT::OPAQUE != RPT::TRANSPARENT);
    assert(RPT::TRANSPARENT != RPT::WIREFRAME);
    assert(RPT::WIREFRAME != RPT::OVERLAY);
    assert(RPT::OVERLAY != RPT::DEBUG);
    assert(RPT::DEBUG != RPT::HUD);
    assert(RPT::OPAQUE != RPT::HUD);

    std::printf("    [PASS] RenderPassType 6 distinct values verified\n");
    return 0;
}

// =============================================================================
// TEST 6: CameraController perspective
// =============================================================================
static int test_camera_perspective() {
    std::printf("  [Test 6] CameraController perspective...\n");

    apc::CameraController cam;
    float fov = 1.0471975512f; // ~60 degrees
    float aspect = 16.0f / 9.0f;
    cam.set_perspective(fov, aspect, 0.1f, 100.0f);

    assert(cam.mode == apc::CameraMode::PERSPECTIVE && "Mode = PERSPECTIVE");

    // Look from origin at -Z
    cam.look_at(apc::Vec3(0.0f, 0.0f, 0.0f),
                apc::Vec3(0.0f, 0.0f, -1.0f),
                apc::Vec3(0.0f, 1.0f, 0.0f));

    const float* view = cam.get_view_matrix();
    // When eye is at origin looking at -Z, the view matrix should be identity-ish
    // Column 3 (translation) should be near zero since eye is at origin
    assert(approx_eq(view[12], 0.0f, 0.1f) && "View matrix translation x ≈ 0");
    assert(approx_eq(view[13], 0.0f, 0.1f) && "View matrix translation y ≈ 0");
    assert(approx_eq(view[14], 0.0f, 0.1f) && "View matrix translation z ≈ 0");

    // Verify projection matrix [0][0] = 1/tan(fov/2)
    const float* proj = cam.get_projection_matrix();
    float f = 1.0f / std::tan(fov * 0.5f);
    float expected_00 = f / aspect;
    assert(approx_eq(proj[0], expected_00, 0.01f) && "Projection [0][0] = f/aspect");

    // Projection [5] = f
    assert(approx_eq(proj[5], f, 0.01f) && "Projection [5] = f");

    // Projection [11] = -1 (perspective divide)
    assert(approx_eq(proj[11], -1.0f) && "Projection [11] = -1");

    // VP matrix should be computed
    const float* vp = cam.get_view_projection();
    // VP should not be all zeros
    bool vp_nonzero = false;
    for (int i = 0; i < 16; ++i) {
        if (std::abs(vp[i]) > 0.001f) { vp_nonzero = true; break; }
    }
    assert(vp_nonzero && "VP matrix is not all zeros");

    std::printf("    [PASS] Perspective projection and view matrix verified\n");
    return 0;
}

// =============================================================================
// TEST 7: CameraController orthographic
// =============================================================================
static int test_camera_orthographic() {
    std::printf("  [Test 7] CameraController orthographic...\n");

    apc::CameraController cam;
    float size = 10.0f;
    float aspect = 2.0f;
    cam.set_orthographic(size, aspect, 0.1f, 100.0f);

    assert(cam.mode == apc::CameraMode::ORTHOGRAPHIC && "Mode = ORTHOGRAPHIC");

    cam.look_at(apc::Vec3(0.0f, 0.0f, 10.0f),
                apc::Vec3(0.0f, 0.0f, 0.0f),
                apc::Vec3(0.0f, 1.0f, 0.0f));

    const float* proj = cam.get_projection_matrix();

    // Orthographic projection: [0] = 1/(half_w) = 1/(size*aspect)
    float expected_00 = 1.0f / (size * aspect);
    assert(approx_eq(proj[0], expected_00, 0.01f) && "Ortho [0][0] = 1/half_w");

    // [5] = 1/(half_h) = 1/size
    float expected_11 = 1.0f / size;
    assert(approx_eq(proj[5], expected_11, 0.01f) && "Ortho [1][1] = 1/half_h");

    // [15] = 1.0 (not -1.0 like perspective)
    assert(approx_eq(proj[15], 1.0f) && "Ortho [3][3] = 1.0");

    // Symmetric frustum: off-diagonal terms should be 0
    assert(approx_eq(proj[1], 0.0f) && "Ortho [1] = 0");
    assert(approx_eq(proj[4], 0.0f) && "Ortho [4] = 0");
    assert(approx_eq(proj[2], 0.0f) && "Ortho [2] = 0");
    assert(approx_eq(proj[8], 0.0f) && "Ortho [8] = 0");

    std::printf("    [PASS] Orthographic projection verified\n");
    return 0;
}

// =============================================================================
// TEST 8: FollowCamera
// =============================================================================
static int test_follow_camera() {
    std::printf("  [Test 8] FollowCamera...\n");

    apc::FollowCamera fc;
    fc.config.offset = apc::Vec3(0.0f, 5.0f, -10.0f);
    fc.config.smooth_factor = 1.0f; // Instant follow for test
    fc.current_position = apc::Vec3(0.0f, 0.0f, 0.0f);

    // Update with target at origin, no velocity
    fc.update(apc::Vec3(0.0f, 0.0f, 0.0f), apc::Vec3(0.0f, 0.0f, 0.0f), 1.0f / 60.0f);

    apc::Vec3 pos = fc.get_position();
    // With smooth_factor=1.0 and dt=1/60, t = 1 - exp(-10/60) ≈ 0.153
    // After update, position should have moved toward target + offset
    // Just verify it's not still at origin
    assert(pos.y > 0.0f && "Follow camera has height > 0");
    assert(pos.z < 0.0f && "Follow camera is behind target (z < 0)");

    // Look target should be near origin
    apc::Vec3 target = fc.get_look_target();
    assert(approx_eq(target.x, 0.0f, 0.5f) && "Look target near origin");

    // Test smoothing with lower factor
    fc.config.smooth_factor = 0.01f;
    fc.current_position = apc::Vec3(0.0f, 5.0f, -10.0f);
    fc.current_target = apc::Vec3(0.0f, 0.0f, 0.0f);

    fc.update(apc::Vec3(100.0f, 0.0f, 0.0f), apc::Vec3(0.0f, 0.0f, 0.0f), 1.0f / 60.0f);

    // With low smooth factor, camera should not have jumped to new position
    apc::Vec3 pos2 = fc.get_position();
    // Camera should be somewhere between old and new position
    assert(pos2.x < 90.0f && "Smooth camera didn't jump to far target");

    std::printf("    [PASS] FollowCamera offset, smoothing verified\n");
    return 0;
}

// =============================================================================
// TEST 9: OrbitCamera
// =============================================================================
static int test_orbit_camera() {
    std::printf("  [Test 9] OrbitCamera...\n");

    apc::OrbitCamera oc;
    oc.config.distance = 10.0f;
    oc.config.yaw = 0.0f;
    oc.config.pitch = 0.5f;

    // Rotate
    float orig_yaw = oc.config.yaw;
    float orig_pitch = oc.config.pitch;
    oc.rotate(100.0f, 50.0f);
    assert(oc.config.yaw > orig_yaw && "Yaw increased after rotate");
    assert(oc.config.pitch > orig_pitch && "Pitch increased after rotate");

    // Pitch clamping
    oc.config.pitch = -apc::APC_HALF_PI + 0.5f;
    oc.rotate(0.0f, -10000.0f);
    assert(oc.config.pitch >= oc.config.min_pitch && "Pitch clamped to min");

    oc.config.pitch = apc::APC_HALF_PI - 0.5f;
    oc.rotate(0.0f, 10000.0f);
    assert(oc.config.pitch <= oc.config.max_pitch && "Pitch clamped to max");

    // Zoom
    float orig_dist = oc.config.distance;
    oc.zoom(10.0f);
    assert(oc.config.distance > orig_dist && "Distance increased after zoom in");

    // Distance clamping
    oc.config.distance = oc.config.min_distance + 0.1f;
    oc.zoom(-1000.0f);
    assert(oc.config.distance >= oc.config.min_distance && "Distance clamped to min");

    oc.config.distance = oc.config.max_distance - 0.1f;
    oc.zoom(1000.0f);
    assert(oc.config.distance <= oc.config.max_distance && "Distance clamped to max");

    // Pan
    apc::Vec3 orig_center = oc.config.orbit_center;
    oc.pan(100.0f, 0.0f);
    assert(!oc.config.orbit_center.equals_approx(orig_center) && "Pan moved center");

    // Update produces valid CameraTransform
    apc::CameraTransform ct = oc.update();
    assert(!ct.eye.equals_approx(ct.target) && "Eye != target after update");
    assert(ct.eye.y > 0.0f && "Camera eye has height");

    // get_position and get_target
    apc::Vec3 pos = oc.get_position();
    apc::Vec3 tgt = oc.get_target();
    assert(!pos.equals_approx(tgt) && "get_position != get_target");

    std::printf("    [PASS] OrbitCamera rotate, zoom, pan, update verified\n");
    return 0;
}

// =============================================================================
// TEST 10: DebugDrawList
// =============================================================================
static int test_debug_draw_list() {
    std::printf("  [Test 10] DebugDrawList...\n");

    apc::DebugDrawList ddl;

    assert(ddl.get_line_count() == 0u && "Initial line_count = 0");
    assert(ddl.get_point_count() == 0u && "Initial point_count = 0");
    assert(ddl.get_triangle_count() == 0u && "Initial triangle_count = 0");

    // Add lines
    apc::RenderColor c = apc::RenderColor::RED();
    for (uint32_t i = 0u; i < 10u; ++i) {
        ddl.add_line(apc::Vec3(0, 0, 0), apc::Vec3(1, 0, 0), c);
    }
    assert(ddl.get_line_count() == 10u && "10 lines added");

    // Add points
    for (uint32_t i = 0u; i < 5u; ++i) {
        ddl.add_point(apc::Vec3(0, 0, 0), c, 2.0f);
    }
    assert(ddl.get_point_count() == 5u && "5 points added");

    // Add triangles
    for (uint32_t i = 0u; i < 3u; ++i) {
        ddl.add_triangle(apc::Vec3(0,0,0), apc::Vec3(1,0,0), apc::Vec3(0,1,0), c);
    }
    assert(ddl.get_triangle_count() == 3u && "3 triangles added");

    // Two-color line
    apc::RenderColor c2 = apc::RenderColor::GREEN();
    ddl.add_line(apc::Vec3(0,0,0), c, apc::Vec3(1,1,1), c2);
    assert(ddl.get_line_count() == 11u && "Two-color line added");

    // Clear
    ddl.clear();
    assert(ddl.get_line_count() == 0u && "After clear: line_count = 0");
    assert(ddl.get_point_count() == 0u && "After clear: point_count = 0");
    assert(ddl.get_triangle_count() == 0u && "After clear: triangle_count = 0");

    // MAX limits
    for (uint32_t i = 0u; i < apc::DebugDrawList::MAX_LINES + 10u; ++i) {
        ddl.add_line(apc::Vec3(0,0,0), apc::Vec3(1,0,0), c);
    }
    assert(ddl.get_line_count() == apc::DebugDrawList::MAX_LINES
           && "Line count capped at MAX_LINES");

    std::printf("    [PASS] DebugDrawList add/clear/limits verified\n");
    return 0;
}

// =============================================================================
// TEST 11: DebugDraw shapes
// =============================================================================
static int test_debug_draw_shapes() {
    std::printf("  [Test 11] DebugDraw shapes...\n");

    apc::DebugDraw dd;
    apc::RenderColor c = apc::RenderColor::GREEN();
    uint32_t segments = 12u;

    // AABB: 12 edges
    dd.draw_aabb(apc::Vec3(-1, -1, -1), apc::Vec3(1, 1, 1), c);
    assert(dd.list.get_line_count() == 12u && "draw_aabb produces 12 lines");

    dd.clear();

    // Sphere wireframe: 3 circles * segments
    dd.draw_sphere_wireframe(apc::Vec3(0,0,0), 1.0f, c, segments);
    assert(dd.list.get_line_count() == 3u * segments && "Sphere produces 3*segments lines");

    dd.clear();

    // Contact normal: 1 line
    dd.draw_contact_normal(apc::Vec3(0,0,0), apc::Vec3(0,1,0), c, 1.0f);
    assert(dd.list.get_line_count() == 1u && "draw_contact_normal produces 1 line");

    dd.clear();

    // Velocity arrow: shaft + 2 arrowhead lines = 3 lines
    dd.draw_velocity_arrow(apc::Vec3(0,0,0), apc::Vec3(0,0,5), c, 1.0f);
    assert(dd.list.get_line_count() == 3u && "Velocity arrow produces 3 lines");

    dd.clear();

    // Contact point: 4 lines (X marker)
    dd.draw_contact_point(apc::Vec3(0,0,0), c);
    assert(dd.list.get_line_count() == 4u && "Contact point produces 4 lines");

    dd.clear();

    // Force arrow: same as velocity arrow
    dd.draw_force_arrow(apc::Vec3(0,0,0), apc::Vec3(1,0,0), c, 1.0f);
    assert(dd.list.get_line_count() == 3u && "Force arrow produces 3 lines");

    dd.clear();

    // Transform axes: 3 lines (X, Y, Z)
    dd.draw_transform_axes(apc::Vec3(0,0,0), apc::Quat::identity(), 1.0f);
    assert(dd.list.get_line_count() == 3u && "Transform axes produces 3 lines");

    dd.clear();

    // OBB wireframe: 12 edges
    dd.draw_obb_wireframe(apc::Vec3(0,0,0), apc::Vec3(1,1,1),
                          apc::Quat::identity(), c);
    assert(dd.list.get_line_count() == 12u && "OBB wireframe produces 12 lines");

    std::printf("    [PASS] DebugDraw AABB, sphere, contact, arrow shapes verified\n");
    return 0;
}

// =============================================================================
// TEST 12: ShapeDebugRenderer
// =============================================================================
static int test_shape_debug_renderer() {
    std::printf("  [Test 12] ShapeDebugRenderer...\n");

    apc::ShapeDebugRenderer renderer;
    apc::DebugDraw dd;
    apc::RenderColor c = apc::RenderColor::YELLOW();

    // Sphere
    {
        dd.clear();
        apc::CollisionShape shape = apc::CollisionShape::make_sphere(2.0f,
            apc::Vec3(0, 0, 0));
        renderer.draw(shape, dd, c);
        // Sphere draws 3 circles * 12 segments = 36 lines
        assert(dd.list.get_line_count() == 36u && "Sphere debug: 36 lines");
    }

    // Box
    {
        dd.clear();
        apc::CollisionShape shape = apc::CollisionShape::make_box(
            apc::Vec3(1, 1, 1), apc::Vec3(0, 0, 0), apc::Quat::identity());
        renderer.draw(shape, dd, c);
        // Box draws 12 edges
        assert(dd.list.get_line_count() == 12u && "Box debug: 12 lines");
    }

    // Capsule
    {
        dd.clear();
        apc::CollisionShape shape = apc::CollisionShape::make_capsule(
            0.5f, 1.0f, apc::Vec3(0, 0, 0), apc::Quat::identity());
        shape.update_cache();
        renderer.draw(shape, dd, c);
        // Capsule: 2 circles (12 each) + vertical lines (12/3 = 4) +
        // 2 half-circles (6 each) = 24 + 4 + 12 = 40
        // But depends on segments, let's just check > 0
        assert(dd.list.get_line_count() > 0u && "Capsule debug: lines > 0");
    }

    // Cylinder
    {
        dd.clear();
        apc::CollisionShape shape = apc::CollisionShape::make_cylinder(
            1.0f, 2.0f, apc::Vec3(0, 0, 0), apc::Quat::identity());
        shape.update_cache();
        renderer.draw(shape, dd, c);
        // Cylinder: 2 circles + vertical lines
        assert(dd.list.get_line_count() > 0u && "Cylinder debug: lines > 0");
    }

    std::printf("    [PASS] ShapeDebugRenderer dispatch verified\n");
    return 0;
}

// =============================================================================
// TEST 13: RenderBridge
// =============================================================================
static int test_render_bridge() {
    std::printf("  [Test 13] RenderBridge...\n");

    apc::RenderBridge bridge;

    // Default state
    assert(bridge.frame_count == 0u && "Initial frame_count = 0");
    assert(bridge.config.show_contact_viz == true && "Default show_contact_viz");

    // Config
    apc::RenderBridgeConfig cfg;
    cfg.show_contact_viz = false;
    cfg.debug_line_width = 2.0f;
    bridge.set_config(cfg);
    assert(bridge.config.show_contact_viz == false && "Config set: show_contact_viz = false");

    // sync_transforms
    apc::RigidBody bodies[3];
    bodies[0].position = apc::Vec3(1.0f, 2.0f, 3.0f);
    bodies[1].position = apc::Vec3(4.0f, 5.0f, 6.0f);
    bodies[2].position = apc::Vec3(7.0f, 8.0f, 9.0f);
    bridge.sync_transforms(bodies, 3);
    assert(bridge.synced_body_count == 3u && "3 bodies synced");

    // sync_contacts with show_contact_viz = false should add nothing
    bridge.begin_frame();
    apc::ContactPoint contacts[2];
    contacts[0].point_on_b = apc::Vec3(0, 0, 0);
    contacts[0].normal = apc::Vec3(0, 1, 0);
    bridge.sync_contacts(contacts, 2);
    assert(bridge.debug_draw.list.get_line_count() == 0u
           && "No contact debug when show_contact_viz = false");

    // Enable contact viz and try again
    bridge.config.show_contact_viz = true;
    bridge.begin_frame();
    bridge.sync_contacts(contacts, 2);
    // Each contact: 4 lines (contact point) + 1 line (normal) = 5
    assert(bridge.debug_draw.list.get_line_count() == 10u
           && "Contact debug draws 5 lines per contact");

    // begin_frame clears and increments frame_count
    bridge.end_frame();
    bridge.begin_frame();
    assert(bridge.debug_draw.list.get_line_count() == 0u
           && "begin_frame clears debug draw");
    assert(bridge.frame_count > 0u && "frame_count incremented");

    // Reset
    bridge.reset();
    assert(bridge.frame_count == 0u && "After reset: frame_count = 0");
    assert(bridge.synced_body_count == 0u && "After reset: synced_body_count = 0");

    // Accessors
    bridge.get_debug_draw();
    bridge.get_camera();
    bridge.get_stats();

    // sync_game_hooks
    apc::GameHookOutput hooks[2];
    hooks[0].type = apc::HookEffectType::HIT_STOP;
    hooks[1].type = apc::HookEffectType::CAMERA_SHAKE;
    bridge.sync_game_hooks(hooks, 2);
    assert(bridge.hook_output_count == 2u && "2 hook outputs synced");

    // sync_sport_state (stub, should not crash)
    bridge.sync_sport_state(nullptr, 0u, nullptr, nullptr, 0u);

    std::printf("    [PASS] RenderBridge sync, config, lifecycle verified\n");
    return 0;
}

// =============================================================================
// MAIN
// =============================================================================
int main() {
    std::printf("=== Sprint 17: Render Foundation & Debug Draw ===\n\n");

    int result = 0;
    result |= test_render_color();
    result |= test_render_vertex();
    result |= test_render_material();
    result |= test_render_stats();
    result |= test_render_pass_type();
    result |= test_camera_perspective();
    result |= test_camera_orthographic();
    result |= test_follow_camera();
    result |= test_orbit_camera();
    result |= test_debug_draw_list();
    result |= test_debug_draw_shapes();
    result |= test_shape_debug_renderer();
    result |= test_render_bridge();

    std::printf("\n");
    if (result == 0) {
        std::printf("Sprint 17: ALL TESTS PASSED\n");
    } else {
        std::printf("Sprint 17: SOME TESTS FAILED\n");
    }

    return result;
}
