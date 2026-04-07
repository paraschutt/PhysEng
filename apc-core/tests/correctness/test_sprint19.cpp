// =============================================================================
// Sprint 19 Tests — Sport Field & Ball Visualization
// =============================================================================
//
// Tests for the APC render visualization headers (Sprint 19):
//   1.  FieldSurfaceVisual defaults and surface color mapping
//   2.  FieldMarkingStyle defaults
//   3.  FieldRendererConfig defaults
//   4.  Soccer field renders correct dimensions (105x68)
//   5.  FieldRenderer generates markings for RECTANGLE type
//   6.  FieldRenderer dispatches for OVAL, DIAMOND, CIRCLE, RINK, RING
//   7.  GoalPostRenderer draws goal structure
//   8.  BoundaryEventVisualizer colors per event type
//   9.  BoundaryEventVisualizer draw with fade
//   10. BallVisualConfig defaults
//   11. BallRenderer processes BallState (sphere)
//   12. BallRenderer processes BallState (prolate)
//   13. BallRenderer draw_ball_mesh generates wireframe
//   14. BallRenderer trail circular buffer
//   15. BallRenderer draw_ball_shadow
//   16. BallRenderer draw_spin_indicator
//   17. PossessionIndicator
//   18. SportHUDConfig defaults
//   19. HUDElement
//   20. SportHUD scoreboard and clock
//   21. SportHUD score event flash
//   22. Integration: full soccer field + ball + HUD render
//
// Pattern: int main() + assert(), no test framework.
// =============================================================================

#include "apc_render/apc_field_renderer.h"
#include "apc_render/apc_ball_renderer.h"
#include "apc_render/apc_sport_hud.h"
#include "apc_render/apc_debug_draw.h"
#include "apc_sport/apc_sport_field.h"
#include "apc_sport/apc_sport_rules.h"
#include "apc_sport/apc_ball_physics.h"
#include "apc_sport/apc_ball_control.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
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
// TEST 1: FieldSurfaceVisual defaults and surface color mapping
// =============================================================================
static int test_field_surface_visual() {
    std::printf("  [Test 1] FieldSurfaceVisual defaults and surface color mapping...\n");

    apc::FieldSurfaceVisual vis;

    // Defaults
    assert(vis.surface == apc::SurfaceType::GRASS && "Default surface = GRASS");
    assert(approx_eq(vis.ground_color.g, 0.6f) && "Default grass green");
    assert(approx_eq(vis.ground_color.r, 0.2f) && "Default grass red=0.2");
    assert(approx_eq(vis.ground_color.b, 0.15f) && "Default grass blue=0.15");
    assert(approx_eq(vis.line_color.r, 1.0f) && "Default line white");
    assert(vis.grid_spacing == 0.0f && "Default grid_spacing = 0");

    // make_default factory
    apc::FieldSurfaceVisual def = apc::FieldSurfaceVisual::make_default();
    assert(def.surface == apc::SurfaceType::GRASS && "make_default = GRASS");

    // from_surface for different surfaces
    apc::FieldSurfaceVisual ice = apc::FieldSurfaceVisual::from_surface(apc::SurfaceType::ICE);
    assert(ice.surface == apc::SurfaceType::ICE && "from_surface(ICE)");
    assert(approx_eq(ice.ground_color.b, 0.98f) && "Ice color blue=0.98");
    assert(approx_eq(ice.line_color.r, 0.2f) && "Ice line = dark");

    apc::FieldSurfaceVisual wood = apc::FieldSurfaceVisual::from_surface(apc::SurfaceType::WOOD);
    assert(approx_eq(wood.ground_color.r, 0.72f) && "Wood color r=0.72");
    assert(approx_eq(wood.ground_color.g, 0.53f) && "Wood color g=0.53");

    apc::FieldSurfaceVisual sand = apc::FieldSurfaceVisual::from_surface(apc::SurfaceType::SAND);
    assert(approx_eq(sand.line_color.r, 0.2f) && "Sand line = dark");

    // get_surface_color for all types
    apc::RenderColor grass_c = apc::FieldSurfaceVisual::get_surface_color(apc::SurfaceType::GRASS);
    assert(approx_eq(grass_c.g, 0.6f) && "Grass color green");
    apc::RenderColor clay_c = apc::FieldSurfaceVisual::get_surface_color(apc::SurfaceType::CLAY);
    assert(approx_eq(clay_c.r, 0.76f) && "Clay color r=0.76");
    apc::RenderColor water_c = apc::FieldSurfaceVisual::get_surface_color(apc::SurfaceType::WATER);
    assert(approx_eq(water_c.b, 0.7f) && "Water color blue=0.7");

    std::printf("    [PASS] FieldSurfaceVisual defaults and surface colors verified\n");
    return 0;
}

// =============================================================================
// TEST 2: FieldMarkingStyle defaults
// =============================================================================
static int test_field_marking_style() {
    std::printf("  [Test 2] FieldMarkingStyle defaults...\n");

    apc::FieldMarkingStyle ms;

    assert(approx_eq(ms.line_width, 0.12f) && "line_width = 0.12");
    assert(approx_eq(ms.line_color.r, 1.0f) && "line_color = WHITE");
    assert(ms.dashed == false && "dashed = false");
    assert(ms.center_circle_segments == 24u && "center_circle_segments = 24");
    assert(ms.arc_segments == 16u && "arc_segments = 16");

    apc::FieldMarkingStyle def = apc::FieldMarkingStyle::make_default();
    assert(approx_eq(def.line_width, 0.12f) && "make_default line_width");
    assert(def.center_circle_segments == 24u && "make_default segments");

    std::printf("    [PASS] FieldMarkingStyle defaults verified\n");
    return 0;
}

// =============================================================================
// TEST 3: FieldRendererConfig defaults
// =============================================================================
static int test_field_renderer_config() {
    std::printf("  [Test 3] FieldRendererConfig defaults...\n");

    apc::FieldRendererConfig cfg;

    assert(cfg.show_surface == true && "show_surface = true");
    assert(cfg.show_zones == false && "show_zones = false");
    assert(cfg.show_markings == true && "show_markings = true");
    assert(cfg.show_goals == true && "show_goals = true");
    assert(cfg.show_corner_flags == true && "show_corner_flags = true");
    assert(cfg.show_penalty_areas == true && "show_penalty_areas = true");
    assert(approx_eq(cfg.zone_alpha, 0.3f) && "zone_alpha = 0.3");
    assert(approx_eq(cfg.marking_brightness, 1.0f) && "marking_brightness = 1.0");
    assert(approx_eq(cfg.ground_y, 0.0f) && "ground_y = 0.0");

    apc::FieldRendererConfig def = apc::FieldRendererConfig::make_default();
    assert(def.show_surface == true && "make_default show_surface");

    std::printf("    [PASS] FieldRendererConfig defaults verified\n");
    return 0;
}

// =============================================================================
// TEST 4: Soccer field renders correct dimensions
// =============================================================================
static int test_soccer_field_dimensions() {
    std::printf("  [Test 4] Soccer field renders correct dimensions (105x68)...\n");

    apc::SportField field;
    apc::FieldGeometry geo = apc::FieldGeometry::make_soccer();
    field.setup(geo);

    apc::FieldRenderer renderer;
    renderer.process_field(field);

    // Ground extents
    assert(approx_eq(renderer.draw_cmd.ground_extents.x, 52.5f) && "Half-length = 52.5");
    assert(approx_eq(renderer.draw_cmd.ground_extents.z, 34.0f) && "Half-width = 34.0");

    // Ground center
    assert(approx_eq(renderer.draw_cmd.ground_center.x, 0.0f) && "Center x = 0");
    assert(approx_eq(renderer.draw_cmd.ground_center.z, 0.0f) && "Center z = 0");

    // Ground color (GRASS)
    assert(approx_eq(renderer.draw_cmd.ground_color.g, 0.6f) && "Ground = grass green");

    // Markings should be generated (at minimum: 4 boundary + 1 halfway + center circle)
    assert(renderer.draw_cmd.marking_count > 10u && "At least 10 markings for soccer");

    // Corner flags
    assert(renderer.draw_cmd.corner_count == 4u && "4 corner flags");

    // Verify corner flag positions
    assert(approx_eq(renderer.draw_cmd.corners[0].position.x, -52.5f) && "Corner 0 x = -52.5");
    assert(approx_eq(renderer.draw_cmd.corners[0].position.z, -34.0f) && "Corner 0 z = -34.0");
    assert(approx_eq(renderer.draw_cmd.corners[1].position.x, 52.5f) && "Corner 1 x = 52.5");
    assert(approx_eq(renderer.draw_cmd.corners[2].position.x, -52.5f) && "Corner 2 x = -52.5");
    assert(approx_eq(renderer.draw_cmd.corners[2].position.z, 34.0f) && "Corner 2 z = 34.0");
    assert(approx_eq(renderer.draw_cmd.corners[3].position.x, 52.5f) && "Corner 3 x = 52.5");
    assert(approx_eq(renderer.draw_cmd.corners[3].position.z, 34.0f) && "Corner 3 z = 34.0");

    // Draw ground plane (2 triangles)
    apc::DebugDraw dd;
    renderer.draw_ground_plane(dd);
    assert(dd.list.get_triangle_count() == 2u && "Ground plane: 2 triangles");

    std::printf("    [PASS] Soccer field 105x68 dimensions verified\n");
    return 0;
}

// =============================================================================
// TEST 5: FieldRenderer generates markings for RECTANGLE type
// =============================================================================
static int test_field_renderer_rectangle_markings() {
    std::printf("  [Test 5] FieldRenderer generates markings for RECTANGLE type...\n");

    apc::SportField field;
    apc::FieldGeometry geo = apc::FieldGeometry::make_soccer();
    field.setup(geo);

    apc::FieldRenderer renderer;
    renderer.process_field(field);

    // Draw markings
    apc::DebugDraw dd;
    renderer.draw_field_markings(dd);
    uint32_t line_count = dd.list.get_line_count();
    assert(line_count > 0u && "Markings produce lines");

    // Draw all
    dd.clear();
    renderer.draw_all(dd);
    uint32_t total_lines = dd.list.get_line_count();
    uint32_t total_tris = dd.list.get_triangle_count();
    assert(total_tris == 2u && "All: 2 ground triangles");
    assert(total_lines > line_count && "All: more lines than markings alone");

    // No-markings config
    apc::FieldRenderer renderer2;
    apc::FieldRendererConfig no_marks;
    no_marks.show_markings = false;
    renderer2.set_config(no_marks);
    renderer2.process_field(field);
    apc::DebugDraw dd2;
    renderer2.draw_field_markings(dd2);
    assert(dd2.list.get_line_count() == 0u && "No-markings config: 0 lines");

    std::printf("    [PASS] Rectangle markings and config toggles verified\n");
    return 0;
}

// =============================================================================
// TEST 6: FieldRenderer dispatch for all field types
// =============================================================================
static int test_field_renderer_all_types() {
    std::printf("  [Test 6] FieldRenderer dispatch for all field types...\n");

    // OVAL
    {
        apc::SportField field;
        field.setup(apc::FieldGeometry::make_aussie_rules());
        apc::FieldRenderer renderer;
        renderer.process_field(field);
        assert(renderer.draw_cmd.marking_count > 0u && "OVAL: markings generated");
        assert(approx_eq(renderer.draw_cmd.ground_extents.x, 82.5f) && "OVAL: half-length = 82.5");
    }

    // DIAMOND
    {
        apc::SportField field;
        field.setup(apc::FieldGeometry::make_baseball_diamond());
        apc::FieldRenderer renderer;
        renderer.process_field(field);
        assert(renderer.draw_cmd.marking_count > 0u && "DIAMOND: markings generated");
    }

    // CIRCLE (beach volleyball)
    {
        apc::SportField field;
        field.setup(apc::FieldGeometry::make_beach_volleyball());
        apc::FieldRenderer renderer;
        renderer.process_field(field);
        assert(renderer.draw_cmd.marking_count > 0u && "CIRCLE: markings generated");
        assert(approx_eq(renderer.draw_cmd.ground_extents.x, 8.0f) && "Beach VB: half-length = 8");
    }

    // RINK
    {
        apc::SportField field;
        field.setup(apc::FieldGeometry::make_ice_hockey());
        apc::FieldRenderer renderer;
        renderer.process_field(field);
        assert(renderer.draw_cmd.marking_count > 0u && "RINK: markings generated");
        assert(approx_eq(renderer.draw_cmd.ground_extents.x, 30.48f) && "Hockey: half-length = 30.48");
    }

    // RING (boxing — not in factory, use generic)
    {
        apc::FieldGeometry ring_geo;
        ring_geo.type = apc::FieldType::RING;
        ring_geo.length = 6.0f;
        ring_geo.width = 6.0f;
        ring_geo.sport = apc::SportType::BOXING;
        apc::SportField field;
        field.setup(ring_geo);
        apc::FieldRenderer renderer;
        renderer.process_field(field);
        assert(renderer.draw_cmd.marking_count > 0u && "RING: markings generated");
    }

    std::printf("    [PASS] All field type dispatch verified\n");
    return 0;
}

// =============================================================================
// TEST 7: GoalPostRenderer draws goal structure
// =============================================================================
static int test_goal_post_renderer() {
    std::printf("  [Test 7] GoalPostRenderer draws goal structure...\n");

    apc::GoalPost goal;
    goal.position = apc::Vec3(-52.5f, 0.0f, 0.0f);
    goal.opening_center = apc::Vec3(-52.5f, 0.0f, 0.0f);
    goal.width = 7.32f;
    goal.height = 2.44f;
    goal.depth = 2.5f;
    goal.post_diameter = 0.12f;
    goal.facing_direction = apc::Vec3(-1.0f, 0.0f, 0.0f);
    goal.team_id = 0;

    apc::GoalPostRenderer gpr;
    apc::DebugDraw dd;
    apc::RenderColor white = apc::RenderColor::WHITE();
    gpr.draw_goal_structure(goal, dd, white);

    // 2 posts + 1 crossbar + 8 net edges = 11 lines (with depth > 0)
    uint32_t lines = dd.list.get_line_count();
    assert(lines == 11u && "Goal with net: 11 lines");

    // Goal without depth
    apc::GoalPost goal_no_net = goal;
    goal_no_net.depth = 0.0f;
    apc::DebugDraw dd2;
    gpr.draw_goal_structure(goal_no_net, dd2, white);
    assert(dd2.list.get_line_count() == 3u && "Goal without net: 3 lines");

    // Post hit
    apc::DebugDraw dd3;
    gpr.draw_post_hit(goal.opening_center, dd3);
    // Sphere wireframe with 8 segments = 3 circles × 8 = 24 lines
    assert(dd3.list.get_line_count() == 24u && "Post hit: 24 lines (wireframe sphere)");

    std::printf("    [PASS] GoalPostRenderer verified\n");
    return 0;
}

// =============================================================================
// TEST 8: BoundaryEventVisualizer colors
// =============================================================================
static int test_boundary_event_colors() {
    std::printf("  [Test 8] BoundaryEventVisualizer colors...\n");

    apc::RenderColor goal = apc::BoundaryEventVisualizer::get_event_color(apc::BoundaryEventType::GOAL_SCORED);
    assert(approx_eq(goal.g, 1.0f) && "GOAL_SCORED = GREEN");

    apc::RenderColor oob = apc::BoundaryEventVisualizer::get_event_color(apc::BoundaryEventType::OUT_OF_BOUNDS);
    assert(approx_eq(oob.r, 1.0f) && "OUT_OF_BOUNDS = RED");

    apc::RenderColor hit_post = apc::BoundaryEventVisualizer::get_event_color(apc::BoundaryEventType::HIT_POST);
    assert(approx_eq(hit_post.g, 1.0f) && "HIT_POST = YELLOW");

    apc::RenderColor crossbar = apc::BoundaryEventVisualizer::get_event_color(apc::BoundaryEventType::HIT_CROSSBAR);
    assert(approx_eq(crossbar.r, 1.0f) && "HIT_CROSSBAR = ORANGE");
    assert(approx_eq(crossbar.g, 0.5f) && "HIT_CROSSBAR g=0.5");

    apc::RenderColor td = apc::BoundaryEventVisualizer::get_event_color(apc::BoundaryEventType::TOUCHDOWN);
    assert(approx_eq(td.b, 1.0f) && "TOUCHDOWN = CYAN");

    apc::RenderColor hr = apc::BoundaryEventVisualizer::get_event_color(apc::BoundaryEventType::HOME_RUN);
    assert(approx_eq(hr.b, 1.0f) && "HOME_RUN = MAGENTA");

    std::printf("    [PASS] BoundaryEventVisualizer colors verified\n");
    return 0;
}

// =============================================================================
// TEST 9: BoundaryEventVisualizer draw with fade
// =============================================================================
static int test_boundary_event_visualizer_draw() {
    std::printf("  [Test 9] BoundaryEventVisualizer draw with fade...\n");

    apc::BoundaryEvent evt;
    evt.type = apc::BoundaryEventType::GOAL_SCORED;
    evt.position = apc::Vec3(0.0f, 0.0f, 0.0f);
    evt.ball_id = 0;
    evt.timestamp = 0.0f;

    apc::BoundaryEventVisualizer bev;
    apc::DebugDraw dd;

    // t=0: should produce lines (expanding ring)
    bev.draw_event(evt, 0.0f, dd);
    assert(dd.list.get_line_count() == 16u && "GOAL at t=0: 16 lines (ring)");

    // t=0.5: still active but smaller
    dd.clear();
    bev.draw_event(evt, 0.5f, dd);
    assert(dd.list.get_line_count() == 16u && "GOAL at t=0.5: 16 lines (smaller ring)");

    // t>1.0: expired, no lines
    dd.clear();
    bev.draw_event(evt, 1.5f, dd);
    assert(dd.list.get_line_count() == 0u && "GOAL at t=1.5: expired, 0 lines");

    // OUT_OF_BOUNDS at t=0: X marker (2 lines)
    evt.type = apc::BoundaryEventType::OUT_OF_BOUNDS;
    evt.position = apc::Vec3(55.0f, 0.0f, 35.0f);
    dd.clear();
    bev.draw_event(evt, 0.0f, dd);
    assert(dd.list.get_line_count() == 2u && "OOB at t=0: 2 lines (X marker)");

    std::printf("    [PASS] BoundaryEventVisualizer draw and fade verified\n");
    return 0;
}

// =============================================================================
// TEST 10: BallVisualConfig defaults
// =============================================================================
static int test_ball_visual_config() {
    std::printf("  [Test 10] BallVisualConfig defaults...\n");

    apc::BallVisualConfig cfg;

    assert(cfg.deformation_enabled == true && "deformation_enabled = true");
    assert(approx_eq(cfg.deformation_scale, 0.3f) && "deformation_scale = 0.3");
    assert(cfg.spin_indicator_enabled == false && "spin_indicator = false");
    assert(approx_eq(cfg.spin_arrow_length, 0.5f) && "spin_arrow_length = 0.5");
    assert(cfg.trail_enabled == true && "trail_enabled = true");
    assert(cfg.trail_max_points == 64u && "trail_max_points = 64");
    assert(cfg.shadow_enabled == true && "shadow_enabled = true");
    assert(approx_eq(cfg.shadow_radius_scale, 1.2f) && "shadow_radius_scale = 1.2");
    assert(cfg.mesh_segments == 12u && "mesh_segments = 12");

    apc::BallVisualConfig def = apc::BallVisualConfig::make_default();
    assert(def.trail_max_points == 64u && "make_default trail_max_points");

    std::printf("    [PASS] BallVisualConfig defaults verified\n");
    return 0;
}

// =============================================================================
// TEST 11: BallRenderer processes BallState (sphere)
// =============================================================================
static int test_ball_renderer_sphere() {
    std::printf("  [Test 11] BallRenderer processes BallState (sphere)...\n");

    apc::BallState ball;
    ball.config = apc::BallFactory::make_soccer();
    ball.ball_id = 0;
    ball.reset();
    ball.body.position = apc::Vec3(5.0f, 2.0f, 3.0f);
    ball.body.linear_velocity = apc::Vec3(10.0f, 0.0f, 0.0f);
    ball.on_ground = false;
    ball.in_air = true;
    ball.update_spin_info();

    apc::BallRenderer renderer;
    renderer.process_ball(ball);

    assert(approx_eq(renderer.draw_cmd.position.x, 5.0f) && "Position x = 5");
    assert(approx_eq(renderer.draw_cmd.position.y, 2.0f) && "Position y = 2");
    assert(approx_eq(renderer.draw_cmd.position.z, 3.0f) && "Position z = 3");
    assert(renderer.draw_cmd.shape == apc::BallShape::SPHERE && "Shape = SPHERE");
    assert(approx_eq(renderer.draw_cmd.radius, 0.11f) && "Radius = 0.11");
    assert(renderer.draw_cmd.in_air == true && "in_air = true");
    assert(renderer.draw_cmd.on_ground == false && "on_ground = false");
    assert(approx_eq(renderer.draw_cmd.scale_x, 1.0f) && "No deformation scale_x");
    assert(approx_eq(renderer.draw_cmd.scale_y, 1.0f) && "No deformation scale_y");

    std::printf("    [PASS] BallRenderer sphere processing verified\n");
    return 0;
}

// =============================================================================
// TEST 12: BallRenderer processes BallState (prolate)
// =============================================================================
static int test_ball_renderer_prolate() {
    std::printf("  [Test 12] BallRenderer processes BallState (prolate)...\n");

    apc::BallState ball;
    ball.config = apc::BallFactory::make_rugby();
    ball.ball_id = 0;
    ball.reset();
    ball.body.position = apc::Vec3(0.0f, 1.0f, 0.0f);
    ball.body.angular_velocity = apc::Vec3(0.0f, 30.0f, 0.0f);
    ball.update_spin_info();

    apc::BallRenderer renderer;
    renderer.process_ball(ball);

    assert(renderer.draw_cmd.shape == apc::BallShape::PROLATE && "Shape = PROLATE");
    assert(approx_eq(renderer.draw_cmd.semi_major, 0.16f) && "Semi-major = 0.16");
    assert(approx_eq(renderer.draw_cmd.semi_minor, 0.06f) && "Semi-minor = 0.06");
    assert(approx_eq(renderer.draw_cmd.spin_rate, 30.0f) && "Spin rate = 30");

    // Draw prolate mesh: 3 circles × 12 segments = 36 lines
    apc::DebugDraw dd;
    renderer.draw_ball_mesh(dd);
    assert(dd.list.get_line_count() == 36u && "Prolate mesh: 36 lines (3×12)");

    std::printf("    [PASS] BallRenderer prolate processing verified\n");
    return 0;
}

// =============================================================================
// TEST 13: BallRenderer draw_ball_mesh generates wireframe
// =============================================================================
static int test_ball_renderer_mesh() {
    std::printf("  [Test 13] BallRenderer draw_ball_mesh generates wireframe...\n");

    apc::BallState ball;
    ball.config = apc::BallFactory::make_soccer();
    ball.reset();
    ball.body.position = apc::Vec3(0.0f, 1.0f, 0.0f);

    apc::BallRenderer renderer;
    renderer.set_config(apc::BallVisualConfig::make_default());
    renderer.process_ball(ball);

    apc::DebugDraw dd;
    renderer.draw_ball_mesh(dd);

    // Sphere wireframe: 3 circles × 12 segments = 36 lines
    assert(dd.list.get_line_count() == 36u && "Sphere mesh: 36 lines (3×12)");

    // With 8 segments: 3 × 8 = 24
    apc::BallVisualConfig cfg8;
    cfg8.mesh_segments = 8u;
    renderer.set_config(cfg8);
    renderer.process_ball(ball);
    apc::DebugDraw dd8;
    renderer.draw_ball_mesh(dd8);
    assert(dd8.list.get_line_count() == 24u && "Sphere 8-seg: 24 lines (3×8)");

    std::printf("    [PASS] Ball mesh wireframe generation verified\n");
    return 0;
}

// =============================================================================
// TEST 14: BallRenderer trail circular buffer
// =============================================================================
static int test_ball_renderer_trail() {
    std::printf("  [Test 14] BallRenderer trail circular buffer...\n");

    apc::BallTrailBuffer trail;
    assert(trail.get_count() == 0u && "Empty trail");

    // Push 3 points
    trail.push(apc::Vec3(0.0f, 0.0f, 0.0f));
    trail.push(apc::Vec3(1.0f, 0.0f, 0.0f));
    trail.push(apc::Vec3(2.0f, 0.0f, 0.0f));
    assert(trail.get_count() == 3u && "3 points");

    // Verify order (0=oldest, 2=newest)
    assert(approx_eq(trail.get(0).x, 0.0f) && "Trail[0] x = 0 (oldest)");
    assert(approx_eq(trail.get(1).x, 1.0f) && "Trail[1] x = 1");
    assert(approx_eq(trail.get(2).x, 2.0f) && "Trail[2] x = 2 (newest)");

    // Reset
    trail.reset();
    assert(trail.get_count() == 0u && "Reset: 0 points");

    // Fill beyond capacity (64)
    for (uint32_t i = 0u; i < 70u; ++i) {
        trail.push(apc::Vec3(static_cast<float>(i), 0.0f, 0.0f));
    }
    assert(trail.get_count() == 64u && "Clamped to 64");
    // Oldest should be index 6 (70 - 64)
    assert(approx_eq(trail.get(0).x, 6.0f) && "Wrapped: oldest = 6");

    std::printf("    [PASS] Ball trail circular buffer verified\n");
    return 0;
}

// =============================================================================
// TEST 15: BallRenderer draw_ball_shadow
// =============================================================================
static int test_ball_renderer_shadow() {
    std::printf("  [Test 15] BallRenderer draw_ball_shadow...\n");

    apc::BallState ball;
    ball.config = apc::BallFactory::make_soccer();
    ball.reset();
    ball.body.position = apc::Vec3(0.0f, 1.0f, 0.0f);
    ball.on_ground = false;
    ball.in_air = true;

    apc::BallRenderer renderer;
    renderer.process_ball(ball);

    apc::DebugDraw dd;
    renderer.draw_ball_shadow(dd);
    // Shadow: 8-segment circle = 8 lines
    assert(dd.list.get_line_count() == 8u && "Shadow: 8 lines (circle)");

    // No shadow config
    apc::BallVisualConfig no_shadow;
    no_shadow.shadow_enabled = false;
    renderer.set_config(no_shadow);
    renderer.process_ball(ball);
    apc::DebugDraw dd2;
    renderer.draw_ball_shadow(dd2);
    assert(dd2.list.get_line_count() == 0u && "No shadow: 0 lines");

    std::printf("    [PASS] Ball shadow verified\n");
    return 0;
}

// =============================================================================
// TEST 16: BallRenderer draw_spin_indicator
// =============================================================================
static int test_ball_renderer_spin() {
    std::printf("  [Test 16] BallRenderer draw_spin_indicator...\n");

    apc::BallState ball;
    ball.config = apc::BallFactory::make_soccer();
    ball.reset();
    ball.body.position = apc::Vec3(0.0f, 1.0f, 0.0f);
    ball.body.angular_velocity = apc::Vec3(0.0f, 20.0f, 0.0f);
    ball.update_spin_info();

    apc::BallRenderer renderer;
    apc::BallVisualConfig spin_cfg;
    spin_cfg.spin_indicator_enabled = true;
    renderer.set_config(spin_cfg);
    renderer.process_ball(ball);

    apc::DebugDraw dd;
    renderer.draw_spin_indicator(dd);
    // Velocity arrow with non-zero speed: 3 lines (shaft + 2 arrowhead)
    assert(dd.list.get_line_count() == 3u && "Spin indicator: 3 lines (arrow)");

    // Zero spin: no lines
    ball.body.angular_velocity = apc::Vec3(0.0f, 0.0f, 0.0f);
    ball.update_spin_info();
    renderer.process_ball(ball);
    apc::DebugDraw dd2;
    renderer.draw_spin_indicator(dd2);
    assert(dd2.list.get_line_count() == 0u && "Zero spin: 0 lines");

    std::printf("    [PASS] Ball spin indicator verified\n");
    return 0;
}

// =============================================================================
// TEST 17: PossessionIndicator
// =============================================================================
static int test_possession_indicator() {
    std::printf("  [Test 17] PossessionIndicator...\n");

    apc::PossessionIndicator pi;
    assert(approx_eq(pi.ring_radius, 0.8f) && "ring_radius = 0.8");
    assert(approx_eq(pi.ring_height_offset, 0.05f) && "ring_height_offset = 0.05");
    assert(pi.ring_segments == 16u && "ring_segments = 16");
    assert(approx_eq(pi.home_color.b, 1.0f) && "home_color = BLUE");
    assert(approx_eq(pi.away_color.r, 1.0f) && "away_color = RED");

    apc::DebugDraw dd;
    apc::Vec3 pos(5.0f, 0.0f, 3.0f);
    pi.draw(pos, 1u, 1.0f, dd);
    // 16 segments = 16 lines
    assert(dd.list.get_line_count() == 16u && "Home possession: 16 lines (ring)");

    dd.clear();
    pi.draw(pos, 2u, 0.5f, dd);
    assert(dd.list.get_line_count() == 16u && "Away possession: 16 lines (ring)");

    std::printf("    [PASS] PossessionIndicator verified\n");
    return 0;
}

// =============================================================================
// TEST 18: SportHUDConfig defaults
// =============================================================================
static int test_sport_hud_config() {
    std::printf("  [Test 18] SportHUDConfig defaults...\n");

    apc::SportHUDConfig cfg;

    assert(cfg.show_scoreboard == true && "show_scoreboard = true");
    assert(cfg.show_clock == true && "show_clock = true");
    assert(cfg.show_period == true && "show_period = true");
    assert(cfg.show_possession == true && "show_possession = true");
    assert(cfg.show_fouls == false && "show_fouls = false");
    assert(cfg.show_play_state == true && "show_play_state = true");
    assert(cfg.show_shot_clock == false && "show_shot_clock = false");
    assert(cfg.show_play_clock == false && "show_play_clock = false");
    assert(cfg.show_timeout_count == false && "show_timeout_count = false");
    assert(cfg.show_score_event == true && "show_score_event = true");
    assert(approx_eq(cfg.font_size, 24.0f) && "font_size = 24");
    assert(approx_eq(cfg.score_event_flash_duration, 3.0f) && "flash_duration = 3.0");

    // make_basketball
    apc::SportHUDConfig bball = apc::SportHUDConfig::make_basketball();
    assert(bball.show_shot_clock == true && "Basketball: shot_clock = true");
    assert(bball.show_fouls == true && "Basketball: fouls = true");

    // make_american_football
    apc::SportHUDConfig football = apc::SportHUDConfig::make_american_football();
    assert(football.show_play_clock == true && "Football: play_clock = true");

    std::printf("    [PASS] SportHUDConfig defaults and factories verified\n");
    return 0;
}

// =============================================================================
// TEST 19: HUDElement
// =============================================================================
static int test_hud_element() {
    std::printf("  [Test 19] HUDElement...\n");

    apc::HUDElement elem;

    assert(approx_eq(elem.x, 0.0f) && "Default x = 0");
    assert(approx_eq(elem.y, 0.0f) && "Default y = 0");
    assert(approx_eq(elem.width, 100.0f) && "Default width = 100");
    assert(elem.visible == true && "Default visible = true");
    assert(elem.anchor == apc::HUDAnchor::TOP_LEFT && "Default anchor = TOP_LEFT");
    assert(elem.sort_order == 0u && "Default sort_order = 0");

    elem.set_position(50.0f, 25.0f, apc::HUDAnchor::TOP_CENTER);
    assert(approx_eq(elem.x, 50.0f) && "Set x = 50");
    assert(approx_eq(elem.y, 25.0f) && "Set y = 25");
    assert(elem.anchor == apc::HUDAnchor::TOP_CENTER && "Anchor = TOP_CENTER");

    elem.set_text("SCORE", apc::RenderColor::GREEN());
    assert(elem.text_color.g == 1.0f && "Text color = GREEN");

    elem.hide();
    assert(elem.visible == false && "Hidden");
    elem.show();
    assert(elem.visible == true && "Shown");

    std::printf("    [PASS] HUDElement verified\n");
    return 0;
}

// =============================================================================
// TEST 20: SportHUD scoreboard and clock
// =============================================================================
static int test_sport_hud_scoreboard() {
    std::printf("  [Test 20] SportHUD scoreboard and clock...\n");

    apc::ScoringSystem scoring;
    scoring.configure_soccer();
    scoring.add_score(0u, 1.0f, "goal", 1200.0f, apc::Vec3(0.0f, 0.0f, 0.0f), 5u);
    scoring.add_score(1u, 1.0f, "goal", 2400.0f, apc::Vec3(0.0f, 0.0f, 0.0f), 10u);

    apc::ClockSystem clock;
    clock.configure_soccer();
    clock.start();
    clock.update(1200.0f);

    apc::SportHUD hud;
    hud.process_sport_state(scoring, clock, 1u);

    assert(approx_eq(hud.scores[0], 1.0f) && "Home score = 1");
    assert(approx_eq(hud.scores[1], 1.0f) && "Away score = 1");
    assert(approx_eq(hud.game_time, 1200.0f) && "Game time = 1200");
    assert(hud.current_period == 1u && "Period = 1");
    assert(hud.possession_team == 1u && "Possession = team 1");

    // Draw scoreboard
    apc::DebugDraw dd;
    hud.draw_scoreboard(dd);
    assert(dd.list.get_line_count() > 0u && "Scoreboard produces lines");

    // Draw clock
    dd.clear();
    hud.draw_match_clock(dd);
    assert(dd.list.get_line_count() > 0u && "Clock produces lines");

    // Draw period
    dd.clear();
    hud.draw_period_indicator(dd);
    assert(dd.list.get_point_count() == 1u && "Period: 1 dot");

    // Draw possession arrow
    dd.clear();
    hud.draw_possession_arrow(dd);
    assert(dd.list.get_line_count() == 3u && "Possession arrow: 3 lines");

    // Draw play state
    dd.clear();
    hud.draw_play_state_indicator(dd);
    assert(dd.list.get_point_count() == 1u && "Play state: 1 point");

    // Draw all
    dd.clear();
    hud.draw_all(dd);
    assert(dd.list.get_line_count() > 0u && "All HUD produces lines");

    std::printf("    [PASS] SportHUD scoreboard and clock verified\n");
    return 0;
}

// =============================================================================
// TEST 21: SportHUD score event flash
// =============================================================================
static int test_sport_hud_score_flash() {
    std::printf("  [Test 21] SportHUD score event flash...\n");

    apc::ScoringSystem scoring;
    scoring.configure_soccer();
    scoring.add_score(0u, 2.0f, "goal", 10.0f, apc::Vec3(0.0f, 0.0f, 0.0f), 5u);

    apc::ClockSystem clock;
    clock.configure_soccer();

    apc::SportHUD hud;
    hud.process_sport_state(scoring, clock);

    // Flash should be active
    assert(hud.flash_count >= 1u && "Flash created");

    // At t=0 flash is active
    apc::DebugDraw dd;
    hud.draw_score_event(dd);
    uint32_t lines_at_0 = dd.list.get_line_count();
    assert(lines_at_0 > 0u && "Score flash at t=0: produces lines");

    // Advance time past flash duration
    hud.update(hud.config.score_event_flash_duration + 1.0f);
    dd.clear();
    hud.draw_score_event(dd);
    assert(dd.list.get_line_count() == 0u && "Score flash expired: 0 lines");

    // Reset
    hud.reset();
    assert(hud.flash_count == 0u && "Reset: 0 flashes");
    assert(approx_eq(hud.scores[0], 0.0f) && "Reset: scores cleared");

    std::printf("    [PASS] SportHUD score event flash verified\n");
    return 0;
}

// =============================================================================
// TEST 22: Integration — full soccer field + ball + HUD
// =============================================================================
static int test_integration() {
    std::printf("  [Test 22] Integration: soccer field + ball + HUD...\n");

    // Set up soccer field with goals
    apc::SportField field;
    apc::FieldGeometry geo = apc::FieldGeometry::make_soccer();
    field.setup(geo);

    // Add goals
    apc::GoalPost home_goal;
    home_goal.position = apc::Vec3(-52.5f, 0.0f, 0.0f);
    home_goal.opening_center = apc::Vec3(-52.5f, 1.22f, 0.0f);
    home_goal.width = 7.32f;
    home_goal.height = 2.44f;
    home_goal.depth = 2.5f;
    home_goal.post_diameter = 0.12f;
    home_goal.facing_direction = apc::Vec3(-1.0f, 0.0f, 0.0f);
    home_goal.team_id = 0;
    field.add_goal(home_goal);

    apc::GoalPost away_goal;
    away_goal.position = apc::Vec3(52.5f, 0.0f, 0.0f);
    away_goal.opening_center = apc::Vec3(52.5f, 1.22f, 0.0f);
    away_goal.width = 7.32f;
    away_goal.height = 2.44f;
    away_goal.depth = 2.5f;
    away_goal.post_diameter = 0.12f;
    away_goal.facing_direction = apc::Vec3(1.0f, 0.0f, 0.0f);
    away_goal.team_id = 1;
    field.add_goal(away_goal);

    // Render field
    apc::FieldRenderer field_renderer;
    field_renderer.process_field(field);

    // Set up ball
    apc::BallState ball;
    ball.config = apc::BallFactory::make_soccer();
    ball.reset();
    ball.body.position = apc::Vec3(0.0f, 0.11f, 0.0f); // Center of field
    ball.body.linear_velocity = apc::Vec3(5.0f, 0.0f, 3.0f);
    ball.on_ground = true;
    ball.in_air = false;
    ball.update_spin_info();

    apc::BallRenderer ball_renderer;
    ball_renderer.process_ball(ball);

    // Push ball through a few "frames" for trail
    for (int i = 0; i < 5; ++i) {
        ball.body.position = apc::Vec3(
            ball.body.position.x + ball.body.linear_velocity.x * 0.01f,
            ball.body.position.y,
            ball.body.position.z + ball.body.linear_velocity.z * 0.01f);
        ball_renderer.process_ball(ball);
    }
    assert(ball_renderer.trail.get_count() == 6u && "Ball trail: 6 points after 5 updates");

    // Set up HUD
    apc::ScoringSystem scoring;
    scoring.configure_soccer();
    scoring.add_score(0u, 1.0f, "goal", 45.0f, apc::Vec3(45.0f, 0.0f, 0.0f), 9u);

    apc::ClockSystem clock;
    clock.configure_soccer();
    clock.start();
    clock.update(2700.0f); // End of first half

    apc::SportHUD hud;
    hud.process_sport_state(scoring, clock, 0u);

    // Draw everything
    apc::DebugDraw dd;
    field_renderer.draw_all(dd);
    uint32_t field_lines = dd.list.get_line_count();

    ball_renderer.draw_ball_mesh(dd);
    ball_renderer.draw_ball_shadow(dd);
    ball_renderer.draw_ball_trail(dd);
    uint32_t ball_lines = dd.list.get_line_count() - field_lines;

    hud.draw_all(dd);
    uint32_t hud_lines = dd.list.get_line_count() - field_lines - ball_lines;

    assert(field_lines > 0u && "Integration: field lines rendered");
    assert(ball_lines > 0u && "Integration: ball lines rendered");
    assert(hud_lines > 0u && "Integration: HUD lines rendered");

    // Verify goal structure was processed
    assert(field_renderer.draw_cmd.goal_count == 2u && "Integration: 2 goals");

    std::printf("    [PASS] Integration: full soccer render verified\n");
    return 0;
}

// =============================================================================
// MAIN
// =============================================================================
int main() {
    std::printf("=== Sprint 19: Sport Field & Ball Visualization ===\n\n");

    int result = 0;
    result |= test_field_surface_visual();
    result |= test_field_marking_style();
    result |= test_field_renderer_config();
    result |= test_soccer_field_dimensions();
    result |= test_field_renderer_rectangle_markings();
    result |= test_field_renderer_all_types();
    result |= test_goal_post_renderer();
    result |= test_boundary_event_colors();
    result |= test_boundary_event_visualizer_draw();
    result |= test_ball_visual_config();
    result |= test_ball_renderer_sphere();
    result |= test_ball_renderer_prolate();
    result |= test_ball_renderer_mesh();
    result |= test_ball_renderer_trail();
    result |= test_ball_renderer_shadow();
    result |= test_ball_renderer_spin();
    result |= test_possession_indicator();
    result |= test_sport_hud_config();
    result |= test_hud_element();
    result |= test_sport_hud_scoreboard();
    result |= test_sport_hud_score_flash();
    result |= test_integration();

    std::printf("\n");
    if (result == 0) {
        std::printf("Sprint 19: ALL TESTS PASSED\n");
    } else {
        std::printf("Sprint 19: SOME TESTS FAILED\n");
    }

    return result;
}
