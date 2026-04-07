#pragma once
// =============================================================================
// Sprint 19: Field Renderer — Sport field visualization
// =============================================================================
//
// Provides:
//   - FieldSurfaceVisual: ground plane appearance per surface type
//   - FieldMarkingStyle: line rendering style for field markings
//   - FieldRendererConfig: toggles and parameters for field rendering
//   - FieldDrawCommand: preprocessed draw data per field element
//   - FieldRenderer: processes SportField into debug draw commands
//   - GoalPostRenderer: draws goal posts, crossbar, net outline
//   - BoundaryEventVisualizer: visual effects for boundary events
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation (fixed-capacity arrays)
//   - C++17
//
// =============================================================================

#include "apc_render_types.h"
#include "apc_debug_draw.h"
#include "apc_sport/apc_sport_field.h"
#include "apc_sport/apc_sport_rules.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_math_common.h"
#include <cstdint>
#include <cmath>

namespace apc {

// =============================================================================
// FieldSurfaceVisual — Ground plane appearance per surface type
// =============================================================================
struct FieldSurfaceVisual {
    SurfaceType surface = SurfaceType::GRASS;
    RenderColor ground_color = RenderColor::GREEN();
    RenderColor line_color = RenderColor::WHITE();
    float grid_spacing = 1.0f;

    /// Get default ground color for a surface type.
    static RenderColor get_surface_color(SurfaceType surface) {
        switch (surface) {
        case SurfaceType::GRASS:       return RenderColor(0.2f, 0.6f, 0.15f, 1.0f);
        case SurfaceType::ARTIFICIAL:  return RenderColor(0.15f, 0.55f, 0.1f, 1.0f);
        case SurfaceType::CLAY:        return RenderColor(0.76f, 0.55f, 0.33f, 1.0f);
        case SurfaceType::HARD_COURT:  return RenderColor(0.6f, 0.6f, 0.6f, 1.0f);
        case SurfaceType::WOOD:        return RenderColor(0.72f, 0.53f, 0.28f, 1.0f);
        case SurfaceType::CARPET:      return RenderColor(0.3f, 0.3f, 0.7f, 1.0f);
        case SurfaceType::SAND:        return RenderColor(0.9f, 0.82f, 0.6f, 1.0f);
        case SurfaceType::ICE:         return RenderColor(0.85f, 0.92f, 0.98f, 1.0f);
        case SurfaceType::WATER:       return RenderColor(0.1f, 0.3f, 0.7f, 1.0f);
        case SurfaceType::TRAMPOLINE:  return RenderColor(0.3f, 0.3f, 0.3f, 1.0f);
        case SurfaceType::MAT:         return RenderColor(0.25f, 0.25f, 0.35f, 1.0f);
        case SurfaceType::CONCRETE:    return RenderColor(0.5f, 0.5f, 0.5f, 1.0f);
        default:                       return RenderColor(0.3f, 0.5f, 0.2f, 1.0f);
        }
    }

    static FieldSurfaceVisual make_default() {
        FieldSurfaceVisual v;
        v.surface = SurfaceType::GRASS;
        v.ground_color = get_surface_color(SurfaceType::GRASS);
        v.line_color = RenderColor::WHITE();
        v.grid_spacing = 0.0f;
        return v;
    }

    static FieldSurfaceVisual from_surface(SurfaceType s) {
        FieldSurfaceVisual v;
        v.surface = s;
        v.ground_color = get_surface_color(s);
        v.line_color = RenderColor::WHITE();
        if (s == SurfaceType::ICE || s == SurfaceType::SAND) {
            v.line_color = RenderColor(0.2f, 0.2f, 0.2f, 1.0f);
        }
        return v;
    }
};

// =============================================================================
// FieldMarkingStyle — Line rendering style
// =============================================================================
struct FieldMarkingStyle {
    float line_width = 0.12f;
    RenderColor line_color = RenderColor::WHITE();
    RenderColor goal_color = RenderColor::WHITE();
    bool dashed = false;
    uint32_t center_circle_segments = 24u;
    uint32_t arc_segments = 16u;

    static FieldMarkingStyle make_default() {
        FieldMarkingStyle s;
        s.line_width = 0.12f;
        s.line_color = RenderColor::WHITE();
        s.goal_color = RenderColor::WHITE();
        s.dashed = false;
        s.center_circle_segments = 24u;
        s.arc_segments = 16u;
        return s;
    }
};

// =============================================================================
// FieldRendererConfig — Toggles and parameters
// =============================================================================
struct FieldRendererConfig {
    bool show_surface = true;
    bool show_zones = false;
    bool show_markings = true;
    bool show_goals = true;
    bool show_corner_flags = true;
    bool show_penalty_areas = true;
    float zone_alpha = 0.3f;
    float marking_brightness = 1.0f;
    float ground_y = 0.0f;

    static FieldRendererConfig make_default() {
        FieldRendererConfig c;
        c.show_surface = true;
        c.show_zones = false;
        c.show_markings = true;
        c.show_goals = true;
        c.show_corner_flags = true;
        c.show_penalty_areas = true;
        c.zone_alpha = 0.3f;
        c.marking_brightness = 1.0f;
        c.ground_y = 0.0f;
        return c;
    }
};

// =============================================================================
// FieldDrawCommand — Preprocessed draw data
// =============================================================================
struct FieldMarkingLine {
    Vec3 start;
    Vec3 end;
    RenderColor color;
};

struct ZoneHighlight {
    Vec3 center;
    Vec3 extents;
    RenderColor color;
    FieldZoneId zone_id;
};

struct GoalDrawData {
    Vec3 left_post_bottom;
    Vec3 left_post_top;
    Vec3 right_post_bottom;
    Vec3 right_post_top;
    Vec3 crossbar_left;
    Vec3 crossbar_right;
    Vec3 net_back_left_bottom;
    Vec3 net_back_right_bottom;
    Vec3 net_back_left_top;
    Vec3 net_back_right_top;
    RenderColor post_color;
    float post_diameter;
};

struct CornerFlagData {
    Vec3 position;
    float height;
    RenderColor color;
};

struct FieldDrawCommand {
    static constexpr uint32_t MAX_MARKINGS = 256u;
    static constexpr uint32_t MAX_ZONES = 20u;
    static constexpr uint32_t MAX_GOALS = 4u;
    static constexpr uint32_t MAX_CORNERS = 4u;

    FieldMarkingLine markings[MAX_MARKINGS];
    uint32_t marking_count = 0u;

    ZoneHighlight zones[MAX_ZONES];
    uint32_t zone_count = 0u;

    GoalDrawData goals[MAX_GOALS];
    uint32_t goal_count = 0u;

    CornerFlagData corners[MAX_CORNERS];
    uint32_t corner_count = 0u;

    Vec3 ground_center;
    Vec3 ground_extents;
    RenderColor ground_color;
};

// =============================================================================
// GoalPostRenderer — Draws goal posts, crossbar, net outline
// =============================================================================
struct GoalPostRenderer {
    float post_visual_diameter = 0.12f;

    /// Draw goal posts, crossbar, and net outline for a GoalPost.
    void draw_goal_structure(const GoalPost& goal, DebugDraw& dd,
                              const RenderColor& color) const {
        float half_w = goal.width * 0.5f;
        float h = goal.height;
        float d = goal.depth;

        Vec3 left_bottom = Vec3::add(goal.opening_center, Vec3(-half_w, 0.0f, 0.0f));
        Vec3 left_top = Vec3::add(goal.opening_center, Vec3(-half_w, h, 0.0f));
        Vec3 right_bottom = Vec3::add(goal.opening_center, Vec3(half_w, 0.0f, 0.0f));
        Vec3 right_top = Vec3::add(goal.opening_center, Vec3(half_w, h, 0.0f));

        dd.list.add_line(left_bottom, left_top, color);
        dd.list.add_line(right_bottom, right_top, color);
        dd.list.add_line(left_top, right_top, color);

        if (d > 0.0f) {
            Vec3 back_left_bottom = Vec3::add(left_bottom, Vec3::scale(goal.facing_direction, -d));
            Vec3 back_right_bottom = Vec3::add(right_bottom, Vec3::scale(goal.facing_direction, -d));
            Vec3 back_left_top = Vec3::add(left_top, Vec3::scale(goal.facing_direction, -d));
            Vec3 back_right_top = Vec3::add(right_top, Vec3::scale(goal.facing_direction, -d));

            dd.list.add_line(left_top, back_left_top, color);
            dd.list.add_line(right_top, back_right_top, color);
            dd.list.add_line(back_left_top, back_right_top, color);
            dd.list.add_line(back_left_bottom, back_left_top, color);
            dd.list.add_line(back_right_bottom, back_right_top, color);
            dd.list.add_line(left_bottom, back_left_bottom, color);
            dd.list.add_line(right_bottom, back_right_bottom, color);
            dd.list.add_line(back_left_bottom, back_right_bottom, color);
        }
    }

    /// Draw a hit indicator at a post or crossbar hit.
    void draw_post_hit(const Vec3& hit_position, DebugDraw& dd) const {
        RenderColor hit_color = RenderColor::YELLOW();
        dd.draw_sphere_wireframe(hit_position, 0.3f, hit_color, 8u);
    }
};

// =============================================================================
// BoundaryEventVisualizer — Visual effects for boundary events
// =============================================================================
struct BoundaryEventVisualizer {
    float flash_duration = 1.0f;
    float flash_radius = 2.0f;

    /// Get visual color for a boundary event type.
    static RenderColor get_event_color(BoundaryEventType type) {
        switch (type) {
        case BoundaryEventType::GOAL_SCORED:  return RenderColor::GREEN();
        case BoundaryEventType::HIT_POST:     return RenderColor::YELLOW();
        case BoundaryEventType::HIT_CROSSBAR: return RenderColor::ORANGE();
        case BoundaryEventType::HIT_NET:      return RenderColor::WHITE();
        case BoundaryEventType::OUT_OF_BOUNDS: return RenderColor::RED();
        case BoundaryEventType::TOUCHDOWN:    return RenderColor::CYAN();
        case BoundaryEventType::SAFETY:       return RenderColor::RED();
        case BoundaryEventType::BEHIND:       return RenderColor(0.5f, 0.0f, 0.5f, 1.0f);
        case BoundaryEventType::HOME_RUN:     return RenderColor::MAGENTA();
        case BoundaryEventType::FOUL_BALL:    return RenderColor::GRAY();
        default:                              return RenderColor::WHITE();
        }
    }

    /// Draw boundary event visualization with time-based fade.
    void draw_event(const BoundaryEvent& event, float time_since_event,
                    DebugDraw& dd) {
        if (time_since_event > flash_duration) return;

        float t = 1.0f - (time_since_event / flash_duration);
        RenderColor color = get_event_color(event.type);
        color.a = t;
        float radius = flash_radius * t;

        switch (event.type) {
        case BoundaryEventType::GOAL_SCORED:
            draw_expanding_ring(event.position, radius, color, 16u, dd);
            break;
        case BoundaryEventType::HIT_POST:
        case BoundaryEventType::HIT_CROSSBAR:
            dd.draw_sphere_wireframe(event.position, 0.3f * t, color, 8u);
            break;
        case BoundaryEventType::OUT_OF_BOUNDS: {
            float s = 0.5f * t;
            dd.list.add_line(Vec3::add(event.position, Vec3(-s, 0, -s)),
                            Vec3::add(event.position, Vec3(s, 0, s)), color);
            dd.list.add_line(Vec3::add(event.position, Vec3(s, 0, -s)),
                            Vec3::add(event.position, Vec3(-s, 0, s)), color);
            break;
        }
        case BoundaryEventType::TOUCHDOWN:
            draw_expanding_ring(event.position, radius * 1.5f, color, 20u, dd);
            break;
        default:
            dd.list.add_point(event.position, color, 5.0f * t);
            break;
        }
    }

private:
    static void draw_expanding_ring(const Vec3& center, float radius,
                                     const RenderColor& color, uint32_t segments,
                                     DebugDraw& dd) {
        if (radius < APC_EPSILON) return;
        for (uint32_t i = 0u; i < segments; ++i) {
            float a0 = static_cast<float>(i) / static_cast<float>(segments) * APC_TWO_PI;
            float a1 = static_cast<float>(i + 1) / static_cast<float>(segments) * APC_TWO_PI;
            Vec3 p0 = Vec3::add(center, Vec3(radius * std::cos(a0), 0.0f,
                                                  radius * std::sin(a0)));
            Vec3 p1 = Vec3::add(center, Vec3(radius * std::cos(a1), 0.0f,
                                                  radius * std::sin(a1)));
            dd.list.add_line(p0, p1, color);
        }
    }
};

// =============================================================================
// FieldRenderer — Processes SportField into debug draw commands
// =============================================================================
struct FieldRenderer {
    FieldRendererConfig config;
    FieldMarkingStyle marking_style;
    FieldSurfaceVisual surface_visual;
    GoalPostRenderer goal_renderer;
    BoundaryEventVisualizer event_visualizer;

    FieldDrawCommand draw_cmd;

    void set_config(const FieldRendererConfig& cfg) { config = cfg; }

    /// Process a SportField into draw commands.
    void process_field(const SportField& field) {
        draw_cmd = FieldDrawCommand();

        const FieldGeometry& geo = field.geometry;

        draw_cmd.ground_center = Vec3(0.0f, config.ground_y, 0.0f);
        draw_cmd.ground_extents = Vec3(geo.length * 0.5f, 0.0f, geo.width * 0.5f);
        surface_visual = FieldSurfaceVisual::from_surface(geo.default_surface);
        draw_cmd.ground_color = surface_visual.ground_color;

        switch (geo.type) {
        case FieldType::RECTANGLE: generate_rectangle_markings(geo); break;
        case FieldType::OVAL:      generate_oval_markings(geo); break;
        case FieldType::DIAMOND:   generate_diamond_markings(geo); break;
        case FieldType::CIRCLE:    generate_circle_markings(geo); break;
        case FieldType::RINK:      generate_rink_markings(geo); break;
        case FieldType::RING:      generate_ring_markings(geo); break;
        default:                   generate_rectangle_markings(geo); break;
        }

        // Zones
        if (config.show_zones) {
            for (uint32_t i = 0u; i < field.zone_count && draw_cmd.zone_count < FieldDrawCommand::MAX_ZONES; ++i) {
                const FieldZone& z = field.zones[i];
                ZoneHighlight& zh = draw_cmd.zones[draw_cmd.zone_count++];
                zh.center = z.center;
                zh.extents = z.extents;
                zh.zone_id = z.zone_id;
                if (z.team_id == 1u) {
                    zh.color = RenderColor(0.0f, 0.0f, 1.0f, config.zone_alpha);
                } else if (z.team_id == 2u) {
                    zh.color = RenderColor(1.0f, 0.0f, 0.0f, config.zone_alpha);
                } else {
                    zh.color = RenderColor(0.5f, 0.5f, 0.5f, config.zone_alpha);
                }
            }
        }

        // Goals
        if (config.show_goals) {
            for (uint32_t i = 0u; i < field.goal_count && draw_cmd.goal_count < FieldDrawCommand::MAX_GOALS; ++i) {
                const GoalPost& g = field.goals[i];
                GoalDrawData& gd = draw_cmd.goals[draw_cmd.goal_count++];
                float half_w = g.width * 0.5f;
                gd.left_post_bottom = Vec3::add(g.opening_center, Vec3(-half_w, 0.0f, 0.0f));
                gd.left_post_top = Vec3::add(g.opening_center, Vec3(-half_w, g.height, 0.0f));
                gd.right_post_bottom = Vec3::add(g.opening_center, Vec3(half_w, 0.0f, 0.0f));
                gd.right_post_top = Vec3::add(g.opening_center, Vec3(half_w, g.height, 0.0f));
                gd.crossbar_left = gd.left_post_top;
                gd.crossbar_right = gd.right_post_top;
                float d = g.depth;
                if (d > 0.0f) {
                    gd.net_back_left_bottom = Vec3::add(gd.left_post_bottom, Vec3::scale(g.facing_direction, -d));
                    gd.net_back_right_bottom = Vec3::add(gd.right_post_bottom, Vec3::scale(g.facing_direction, -d));
                    gd.net_back_left_top = Vec3::add(gd.left_post_top, Vec3::scale(g.facing_direction, -d));
                    gd.net_back_right_top = Vec3::add(gd.right_post_top, Vec3::scale(g.facing_direction, -d));
                }
                gd.post_color = RenderColor::WHITE();
                gd.post_diameter = g.post_diameter;
            }
        }

        // Corner flags
        if (config.show_corner_flags) {
            float half_l = geo.length * 0.5f;
            float half_w = geo.width * 0.5f;
            float h = geo.corner_post_height;
            RenderColor flag_color = RenderColor::YELLOW();
            if (draw_cmd.corner_count < FieldDrawCommand::MAX_CORNERS)
                draw_cmd.corners[draw_cmd.corner_count++] = {Vec3(-half_l, 0.0f, -half_w), h, flag_color};
            if (draw_cmd.corner_count < FieldDrawCommand::MAX_CORNERS)
                draw_cmd.corners[draw_cmd.corner_count++] = {Vec3(half_l, 0.0f, -half_w), h, flag_color};
            if (draw_cmd.corner_count < FieldDrawCommand::MAX_CORNERS)
                draw_cmd.corners[draw_cmd.corner_count++] = {Vec3(-half_l, 0.0f, half_w), h, flag_color};
            if (draw_cmd.corner_count < FieldDrawCommand::MAX_CORNERS)
                draw_cmd.corners[draw_cmd.corner_count++] = {Vec3(half_l, 0.0f, half_w), h, flag_color};
        }
    }

    // --- Draw methods ---

    void draw_ground_plane(DebugDraw& dd) {
        if (!config.show_surface) return;
        float y = config.ground_y;
        float hl = draw_cmd.ground_extents.x;
        float hw = draw_cmd.ground_extents.z;
        Vec3 bl(-hl, y, -hw), br(hl, y, -hw), tr(hl, y, hw), tl(-hl, y, hw);
        dd.list.add_triangle(bl, br, tr, draw_cmd.ground_color);
        dd.list.add_triangle(bl, tr, tl, draw_cmd.ground_color);
    }

    void draw_field_markings(DebugDraw& dd) {
        if (!config.show_markings) return;
        for (uint32_t i = 0u; i < draw_cmd.marking_count; ++i) {
            dd.list.add_line(draw_cmd.markings[i].start, draw_cmd.markings[i].end,
                             draw_cmd.markings[i].color);
        }
    }

    void draw_zones(DebugDraw& dd) {
        if (!config.show_zones) return;
        for (uint32_t i = 0u; i < draw_cmd.zone_count; ++i) {
            const ZoneHighlight& z = draw_cmd.zones[i];
            Vec3 mn = Vec3::sub(z.center, z.extents);
            Vec3 mx = Vec3::add(z.center, z.extents);
            dd.draw_aabb(mn, mx, z.color);
        }
    }

    void draw_goal_posts(DebugDraw& dd) {
        if (!config.show_goals) return;
        for (uint32_t i = 0u; i < draw_cmd.goal_count; ++i) {
            const GoalDrawData& gd = draw_cmd.goals[i];
            RenderColor c = gd.post_color;
            dd.list.add_line(gd.left_post_bottom, gd.left_post_top, c);
            dd.list.add_line(gd.right_post_bottom, gd.right_post_top, c);
            dd.list.add_line(gd.crossbar_left, gd.crossbar_right, c);
        }
    }

    void draw_corner_flags(DebugDraw& dd) {
        if (!config.show_corner_flags) return;
        for (uint32_t i = 0u; i < draw_cmd.corner_count; ++i) {
            const CornerFlagData& cf = draw_cmd.corners[i];
            Vec3 top(cf.position.x, cf.height, cf.position.z);
            dd.list.add_line(cf.position, top, cf.color);
            Vec3 flag_end(top.x + 0.3f, top.y, top.z);
            Vec3 flag_tip(top.x + 0.15f, top.y + 0.2f, top.z);
            dd.list.add_line(top, flag_end, cf.color);
            dd.list.add_line(top, flag_tip, cf.color);
            dd.list.add_line(flag_end, flag_tip, cf.color);
        }
    }

    void draw_penalty_areas(DebugDraw& dd) { draw_field_markings(dd); }
    void draw_center_circle(DebugDraw& dd) { draw_field_markings(dd); }

    void draw_all(DebugDraw& dd) {
        draw_ground_plane(dd);
        draw_zones(dd);
        draw_field_markings(dd);
        draw_goal_posts(dd);
        draw_corner_flags(dd);
    }

    void clear() { draw_cmd = FieldDrawCommand(); }

private:
    void add_marking(const Vec3& a, const Vec3& b) {
        if (draw_cmd.marking_count < FieldDrawCommand::MAX_MARKINGS) {
            draw_cmd.markings[draw_cmd.marking_count++] = {a, b, marking_style.line_color};
        }
    }

    void add_circle_markings(const Vec3& center, float radius, uint32_t segments) {
        float y = config.ground_y;
        for (uint32_t i = 0u; i < segments; ++i) {
            float a0 = static_cast<float>(i) / static_cast<float>(segments) * APC_TWO_PI;
            float a1 = static_cast<float>(i + 1) / static_cast<float>(segments) * APC_TWO_PI;
            add_marking(
                Vec3(center.x + radius * std::cos(a0), y, center.z + radius * std::sin(a0)),
                Vec3(center.x + radius * std::cos(a1), y, center.z + radius * std::sin(a1)));
        }
    }

    void add_arc_markings(const Vec3& center, float radius, float start_angle,
                           float end_angle, uint32_t segments) {
        float y = config.ground_y;
        for (uint32_t i = 0u; i < segments; ++i) {
            float t0 = static_cast<float>(i) / static_cast<float>(segments);
            float t1 = static_cast<float>(i + 1) / static_cast<float>(segments);
            float a0 = start_angle + t0 * (end_angle - start_angle);
            float a1 = start_angle + t1 * (end_angle - start_angle);
            add_marking(
                Vec3(center.x + radius * std::cos(a0), y, center.z + radius * std::sin(a0)),
                Vec3(center.x + radius * std::cos(a1), y, center.z + radius * std::sin(a1)));
        }
    }

    // RECTANGLE markings (soccer, rugby, basketball, tennis, volleyball)
    void generate_rectangle_markings(const FieldGeometry& geo) {
        float y = config.ground_y;
        float hl = geo.length * 0.5f;
        float hw = geo.width * 0.5f;

        // Boundary
        Vec3 bl(-hl, y, -hw), br(hl, y, -hw), tr(hl, y, hw), tl(-hl, y, hw);
        add_marking(bl, br); add_marking(br, tr); add_marking(tr, tl); add_marking(tl, bl);

        // Half-way line
        add_marking(Vec3(0.0f, y, -hw), Vec3(0.0f, y, hw));

        // Center circle
        if (geo.center_radius > 0.0f) {
            add_circle_markings(Vec3(0.0f, y, 0.0f), geo.center_radius,
                               marking_style.center_circle_segments);
            add_marking(Vec3(-0.1f, y, 0.0f), Vec3(0.1f, y, 0.0f));
        }

        // Penalty areas
        if (config.show_penalty_areas && geo.penalty_area_depth > 0.0f) {
            float paw = geo.penalty_area_width * 0.5f;
            float pad = geo.penalty_area_depth;

            // Left penalty area
            add_marking(Vec3(-hl, y, -paw), Vec3(-hl + pad, y, -paw));
            add_marking(Vec3(-hl + pad, y, -paw), Vec3(-hl + pad, y, paw));
            add_marking(Vec3(-hl + pad, y, paw), Vec3(-hl, y, paw));
            add_marking(Vec3(-hl, y, paw), Vec3(-hl, y, -paw));

            // Right penalty area
            add_marking(Vec3(hl - pad, y, -paw), Vec3(hl, y, -paw));
            add_marking(Vec3(hl, y, -paw), Vec3(hl, y, paw));
            add_marking(Vec3(hl, y, paw), Vec3(hl - pad, y, paw));
            add_marking(Vec3(hl - pad, y, paw), Vec3(hl - pad, y, -paw));

            // Goal areas
            if (geo.goal_area_depth > 0.0f) {
                float gaw = geo.goal_area_width * 0.5f;
                float gad = geo.goal_area_depth;
                add_marking(Vec3(-hl, y, -gaw), Vec3(-hl + gad, y, -gaw));
                add_marking(Vec3(-hl + gad, y, -gaw), Vec3(-hl + gad, y, gaw));
                add_marking(Vec3(-hl + gad, y, gaw), Vec3(-hl, y, gaw));
                add_marking(Vec3(-hl, y, gaw), Vec3(-hl, y, -gaw));

                add_marking(Vec3(hl - gad, y, -gaw), Vec3(hl, y, -gaw));
                add_marking(Vec3(hl, y, -gaw), Vec3(hl, y, gaw));
                add_marking(Vec3(hl, y, gaw), Vec3(hl - gad, y, gaw));
                add_marking(Vec3(hl - gad, y, gaw), Vec3(hl - gad, y, -gaw));
            }

            // Penalty spots
            if (geo.penalty_spot > 0.0f) {
                add_marking(Vec3(-hl + geo.penalty_spot - 0.1f, y, 0.0f),
                           Vec3(-hl + geo.penalty_spot + 0.1f, y, 0.0f));
                add_marking(Vec3(hl - geo.penalty_spot - 0.1f, y, 0.0f),
                           Vec3(hl - geo.penalty_spot + 0.1f, y, 0.0f));
            }
        }

        // Basketball 3pt line and key
        if (geo.three_point_line > 0.0f) {
            add_arc_markings(Vec3(-hl, y, 0.0f), geo.three_point_line,
                            -APC_PI * 0.5f, APC_PI * 0.5f, 24u);
            add_arc_markings(Vec3(hl, y, 0.0f), geo.three_point_line,
                            APC_PI * 0.5f, APC_PI * 1.5f, 24u);

            if (geo.key_width > 0.0f && geo.key_length > 0.0f) {
                float kw = geo.key_width * 0.5f;
                float kl = geo.key_length;
                add_marking(Vec3(-hl, y, -kw), Vec3(-hl + kl, y, -kw));
                add_marking(Vec3(-hl + kl, y, -kw), Vec3(-hl + kl, y, kw));
                add_marking(Vec3(-hl + kl, y, kw), Vec3(-hl, y, kw));
                add_marking(Vec3(-hl, y, kw), Vec3(-hl, y, -kw));

                add_marking(Vec3(hl - kl, y, -kw), Vec3(hl, y, -kw));
                add_marking(Vec3(hl, y, -kw), Vec3(hl, y, kw));
                add_marking(Vec3(hl, y, kw), Vec3(hl - kl, y, kw));
                add_marking(Vec3(hl - kl, y, kw), Vec3(hl - kl, y, -kw));
            }

            if (geo.free_throw_line > 0.0f) {
                float kw2 = geo.key_width * 0.5f;
                add_marking(Vec3(-hl + geo.free_throw_line, y, -kw2),
                           Vec3(-hl + geo.free_throw_line, y, kw2));
                add_marking(Vec3(hl - geo.free_throw_line, y, -kw2),
                           Vec3(hl - geo.free_throw_line, y, kw2));
            }
        }

        // Corner arcs
        if (geo.corner_radius > 0.0f) {
            float cr = geo.corner_radius;
            add_arc_markings(Vec3(-hl, y, -hw), cr, 0.0f, APC_PI * 0.5f, 8u);
            add_arc_markings(Vec3(hl, y, -hw), cr, APC_PI * 0.5f, APC_PI, 8u);
            add_arc_markings(Vec3(hl, y, hw), cr, APC_PI, APC_PI * 1.5f, 8u);
            add_arc_markings(Vec3(-hl, y, hw), cr, APC_PI * 1.5f, APC_TWO_PI, 8u);
        }
    }

    // OVAL markings (aussie rules, cricket)
    void generate_oval_markings(const FieldGeometry& geo) {
        float y = config.ground_y;
        float hl = geo.length * 0.5f;
        float hw = geo.width * 0.5f;
        uint32_t segments = 48u;
        for (uint32_t i = 0u; i < segments; ++i) {
            float a0 = static_cast<float>(i) / static_cast<float>(segments) * APC_TWO_PI;
            float a1 = static_cast<float>(i + 1) / static_cast<float>(segments) * APC_TWO_PI;
            add_marking(Vec3(hl * std::cos(a0), y, hw * std::sin(a0)),
                       Vec3(hl * std::cos(a1), y, hw * std::sin(a1)));
        }
        if (geo.center_radius > 0.0f) {
            add_circle_markings(Vec3(0.0f, y, 0.0f), geo.center_radius, 24u);
        }
        add_marking(Vec3(0.0f, y, -hw), Vec3(0.0f, y, hw));
    }

    // DIAMOND markings (baseball)
    void generate_diamond_markings(const FieldGeometry& geo) {
        float y = config.ground_y;
        float hl = geo.length * 0.5f;
        float hw = geo.width * 0.5f;
        Vec3 home(-hl, y, 0.0f);
        Vec3 first(-hl + hw, y, -hw);
        Vec3 second(-hl + 2.0f * hw, y, 0.0f);
        Vec3 third(-hl + hw, y, hw);
        add_marking(home, first); add_marking(first, second);
        add_marking(second, third); add_marking(third, home);

        float ow = hw * 1.5f;
        add_marking(Vec3(-hl, y, -ow), Vec3(hl, y, -ow));
        add_marking(Vec3(hl, y, -ow), Vec3(hl, y, ow));
        add_marking(Vec3(hl, y, ow), Vec3(-hl, y, ow));
        add_marking(Vec3(-hl, y, ow), Vec3(-hl, y, -ow));
    }

    // CIRCLE markings (volleyball — uses rectangle + center net + attack lines)
    void generate_circle_markings(const FieldGeometry& geo) {
        generate_rectangle_markings(geo);
        float y = config.ground_y;
        float hw = geo.width * 0.5f;
        add_marking(Vec3(0.0f, y, -hw), Vec3(0.0f, y, hw));
        float attack = 3.0f;
        add_marking(Vec3(-attack, y, -hw), Vec3(-attack, y, hw));
        add_marking(Vec3(attack, y, -hw), Vec3(attack, y, hw));
    }

    // RINK markings (ice hockey — rounded rectangle + blue lines)
    void generate_rink_markings(const FieldGeometry& geo) {
        float y = config.ground_y;
        float hl = geo.length * 0.5f;
        float hw = geo.width * 0.5f;
        float cr = hw;
        uint32_t cs = 8u;

        add_marking(Vec3(-hl + cr, y, -hw), Vec3(hl - cr, y, -hw));
        add_marking(Vec3(hl - cr, y, hw), Vec3(-hl + cr, y, hw));
        add_marking(Vec3(-hl, y, -hw + cr), Vec3(-hl, y, hw - cr));
        add_marking(Vec3(hl, y, hw - cr), Vec3(hl, y, -hw + cr));

        add_arc_markings(Vec3(hl - cr, y, -hw + cr), cr, -APC_PI * 0.5f, 0.0f, cs);
        add_arc_markings(Vec3(hl - cr, y, hw - cr), cr, 0.0f, APC_PI * 0.5f, cs);
        add_arc_markings(Vec3(-hl + cr, y, hw - cr), cr, APC_PI * 0.5f, APC_PI, cs);
        add_arc_markings(Vec3(-hl + cr, y, -hw + cr), cr, APC_PI, APC_PI * 1.5f, cs);

        add_marking(Vec3(0.0f, y, -hw), Vec3(0.0f, y, hw));

        if (geo.center_radius > 0.0f) {
            add_circle_markings(Vec3(0.0f, y, 0.0f), geo.center_radius, 24u);
        }

        // Blue lines
        float bl_pos = hl / 3.0f;
        RenderColor saved = marking_style.line_color;
        marking_style.line_color = RenderColor::BLUE();
        add_marking(Vec3(-bl_pos, y, -hw), Vec3(-bl_pos, y, hw));
        add_marking(Vec3(bl_pos, y, -hw), Vec3(bl_pos, y, hw));
        marking_style.line_color = saved;
    }

    // RING markings (boxing, MMA)
    void generate_ring_markings(const FieldGeometry& geo) {
        float y = config.ground_y;
        float radius = geo.length * 0.5f;
        add_circle_markings(Vec3(0.0f, y, 0.0f), radius, 32u);
        add_circle_markings(Vec3(0.0f, y, 0.0f), 0.5f, 8u);
    }
};

} // namespace apc
