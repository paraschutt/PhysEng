#pragma once
// =============================================================================
// Sprint 19: Ball Renderer — Ball visualization, trails, possession
// =============================================================================
//
// Provides:
//   - BallVisualConfig: visual parameters for ball rendering
//   - BallDrawCommand: preprocessed draw command per ball
//   - BallRenderer: processes BallState into draw commands
//   - Sphere/prolate mesh generation (procedural wireframe)
//   - Ball trail system (circular buffer with alpha fade)
//   - Spin indicator, shadow, deformation, knuckle wobble
//   - PossessionIndicator: ring around controlling athlete
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation (fixed-capacity arrays)
//   - C++17
//
// =============================================================================

#include "apc_render_types.h"
#include "apc_debug_draw.h"
#include "apc_sport/apc_ball_physics.h"
#include "apc_sport/apc_ball_control.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include "apc_math/apc_math_common.h"
#include <cstdint>
#include <cmath>

namespace apc {

// =============================================================================
// BallVisualConfig — Visual parameters for ball rendering
// =============================================================================
struct BallVisualConfig {
    bool deformation_enabled = true;
    float deformation_scale = 0.3f;           // Visual exaggeration of deformation
    bool spin_indicator_enabled = false;
    float spin_arrow_length = 0.5f;
    bool trail_enabled = true;
    uint32_t trail_max_points = 64u;
    RenderColor trail_color = RenderColor::WHITE();
    bool shadow_enabled = true;
    float shadow_radius_scale = 1.2f;
    RenderColor shadow_color = RenderColor(0.0f, 0.0f, 0.0f, 0.4f);
    uint32_t mesh_segments = 12u;              // Sphere/prolate wireframe detail

    static BallVisualConfig make_default() {
        BallVisualConfig c;
        c.deformation_enabled = true;
        c.deformation_scale = 0.3f;
        c.spin_indicator_enabled = false;
        c.spin_arrow_length = 0.5f;
        c.trail_enabled = true;
        c.trail_max_points = 64u;
        c.trail_color = RenderColor::WHITE();
        c.shadow_enabled = true;
        c.shadow_radius_scale = 1.2f;
        c.shadow_color = RenderColor(0.0f, 0.0f, 0.0f, 0.4f);
        c.mesh_segments = 12u;
        return c;
    }
};

// =============================================================================
// BallDrawCommand — Preprocessed draw command per ball
// =============================================================================
struct BallDrawCommand {
    Vec3 position = Vec3(0.0f, 0.0f, 0.0f);
    Quat orientation;
    float radius = 0.11f;
    float scale_x = 1.0f;                     // Deformation X
    float scale_y = 1.0f;                     // Deformation Y
    float scale_z = 1.0f;                     // Deformation Z
    Vec3 spin_axis = Vec3(0.0f, 1.0f, 0.0f);
    float spin_rate = 0.0f;
    float deformation = 0.0f;
    Vec3 deformation_axis = Vec3(0.0f, 1.0f, 0.0f);
    bool on_ground = false;
    bool in_air = true;
    BallShape shape = BallShape::SPHERE;
    float semi_major = 0.11f;
    float semi_minor = 0.11f;
};

// =============================================================================
// BallTrailBuffer — Circular buffer of past ball positions
// =============================================================================
struct BallTrailBuffer {
    static constexpr uint32_t MAX_POINTS = 64u;

    Vec3 points[MAX_POINTS];
    uint32_t count = 0u;
    uint32_t head = 0u;       // Next write position
    bool full = false;

    void reset() {
        count = 0u;
        head = 0u;
        full = false;
    }

    void push(const Vec3& pos) {
        points[head] = pos;
        head = (head + 1u) % MAX_POINTS;
        if (head == 0u) full = true;
        count = full ? MAX_POINTS : head;
    }

    uint32_t get_count() const { return count; }

    /// Get point by index (0 = oldest, count-1 = newest).
    const Vec3& get(uint32_t idx) const {
        uint32_t start = full ? head : 0u;
        return points[(start + idx) % MAX_POINTS];
    }
};

// =============================================================================
// BallRenderer — Processes BallState into draw commands
// =============================================================================
struct BallRenderer {
    BallVisualConfig config;
    BallDrawCommand draw_cmd;
    BallTrailBuffer trail;

    /// Set visual configuration.
    void set_config(const BallVisualConfig& cfg) {
        config = cfg;
    }

    /// Process a BallState into a draw command.
    void process_ball(const BallState& ball) {
        draw_cmd.position = ball.body.position;
        draw_cmd.orientation = ball.body.orientation;
        draw_cmd.shape = ball.config.shape;
        draw_cmd.semi_major = ball.config.semi_major;
        draw_cmd.semi_minor = ball.config.semi_minor;
        draw_cmd.spin_axis = ball.spin_axis;
        draw_cmd.spin_rate = ball.spin_rate;
        draw_cmd.deformation = ball.deformation;
        draw_cmd.deformation_axis = ball.deformation_axis;
        draw_cmd.on_ground = ball.on_ground;
        draw_cmd.in_air = ball.in_air;

        // Compute radius and deformation scale
        if (ball.config.shape == BallShape::PROLATE) {
            draw_cmd.radius = ball.config.get_effective_radius();
            draw_cmd.scale_x = 1.0f;
            draw_cmd.scale_y = 1.0f;
            draw_cmd.scale_z = 1.0f;
            // Prolate: apply non-uniform scale later in draw
        } else {
            draw_cmd.radius = ball.config.radius;
            draw_cmd.scale_x = 1.0f;
            draw_cmd.scale_y = 1.0f;
            draw_cmd.scale_z = 1.0f;
        }

        // Deformation squash
        if (config.deformation_enabled && ball.deformation > 0.0f) {
            float d = ball.deformation * config.deformation_scale;
            // Squash along deformation axis
            // For simplicity: scale deformation axis by (1 - d), others by (1 + d * 0.5)
            draw_cmd.scale_y = 1.0f - d;
            draw_cmd.scale_x = 1.0f + d * 0.5f;
            draw_cmd.scale_z = 1.0f + d * 0.5f;
        }

        // Update trail
        if (config.trail_enabled) {
            trail.push(ball.body.position);
            // Trim to max
            while (trail.get_count() > config.trail_max_points) {
                // Trail auto-trims via circular buffer
                break;
            }
        }
    }

    /// Draw ball as wireframe sphere.
    void draw_ball_mesh(DebugDraw& dd) const {
        RenderColor ball_color = RenderColor::WHITE();
        if (draw_cmd.shape == BallShape::PROLATE) {
            draw_prolate_wireframe(dd, draw_cmd.position, draw_cmd.semi_major,
                                   draw_cmd.semi_minor, draw_cmd.orientation,
                                   ball_color, config.mesh_segments);
        } else {
            dd.draw_sphere_wireframe(draw_cmd.position, draw_cmd.radius, ball_color,
                                     config.mesh_segments);
        }
    }

    /// Draw deformation indicator: extra lines showing squash direction.
    void draw_deformation(DebugDraw& dd) const {
        if (!config.deformation_enabled || draw_cmd.deformation < 0.01f) return;

        RenderColor def_color = RenderColor::YELLOW();
        float d = draw_cmd.deformation * config.deformation_scale;
        Vec3 pos = draw_cmd.position;
        Vec3 axis = draw_cmd.deformation_axis;
        float r = draw_cmd.radius;

        // Squash indicator: compressed line along deformation axis
        Vec3 top = Vec3::add(pos, Vec3::scale(axis, r * (1.0f - d)));
        Vec3 bot = Vec3::sub(pos, Vec3::scale(axis, r * (1.0f - d)));
        dd.list.add_line(top, bot, def_color);
    }

    /// Draw spin axis indicator (arrow from ball center along spin axis).
    void draw_spin_indicator(DebugDraw& dd) const {
        if (!config.spin_indicator_enabled) return;
        if (draw_cmd.spin_rate < APC_EPSILON) return;

        RenderColor spin_color = RenderColor::CYAN();
        Vec3 tip = Vec3::add(draw_cmd.position,
            Vec3::scale(draw_cmd.spin_axis, config.spin_arrow_length));
        dd.draw_velocity_arrow(draw_cmd.position,
            Vec3::scale(draw_cmd.spin_axis, draw_cmd.spin_rate),
            spin_color, 0.1f);
    }

    /// Draw ball shadow (circle projected onto ground plane Y=0).
    void draw_ball_shadow(DebugDraw& dd) const {
        if (!config.shadow_enabled) return;

        Vec3 shadow_pos(draw_cmd.position.x, 0.01f, draw_cmd.position.z);
        float shadow_r = draw_cmd.radius * config.shadow_radius_scale;

        // Height-based shadow size (higher = smaller, more spread)
        RenderColor shadow_col = config.shadow_color;
        float height = draw_cmd.position.y;
        if (height > 0.0f) {
            float height_factor = 1.0f / (1.0f + height * 0.5f);
            shadow_r *= height_factor;
            shadow_col.a = 0.4f * height_factor;
        }

        // Draw shadow as 2 circles (XZ plane at y=0.01)
        uint32_t seg = 8u;
        for (uint32_t i = 0u; i < seg; ++i) {
            float a0 = static_cast<float>(i) / static_cast<float>(seg) * APC_TWO_PI;
            float a1 = static_cast<float>(i + 1) / static_cast<float>(seg) * APC_TWO_PI;
            Vec3 p0 = Vec3::add(shadow_pos, Vec3(shadow_r * std::cos(a0), 0.0f,
                                                         shadow_r * std::sin(a0)));
            Vec3 p1 = Vec3::add(shadow_pos, Vec3(shadow_r * std::cos(a1), 0.0f,
                                                         shadow_r * std::sin(a1)));
            dd.list.add_line(p0, p1, shadow_col);
        }
    }

    /// Draw ball trail (line strip with alpha fade by age).
    void draw_ball_trail(DebugDraw& dd) const {
        if (!config.trail_enabled) return;

        uint32_t n = trail.get_count();
        if (n < 2u) return;

        for (uint32_t i = 0u; i < n - 1u; ++i) {
            float t = static_cast<float>(i) / static_cast<float>(n);
            RenderColor tc = RenderColor(
                config.trail_color.r,
                config.trail_color.g,
                config.trail_color.b,
                t * config.trail_color.a  // Alpha fade: old=transparent, new=opaque
            );
            dd.list.add_line(trail.get(i), trail.get(i + 1), tc);
        }
    }

    /// Draw knuckle wobble indicator (small oscillating lines for low-spin balls).
    void draw_knuckle_wobble(DebugDraw& dd, const BallState& ball) const {
        if (ball.knuckle_intensity < 0.01f) return;

        RenderColor wobble_color = RenderColor::MAGENTA();
        float wobble_r = draw_cmd.radius * 0.5f * ball.knuckle_intensity;
        uint32_t seg = 6u;

        for (uint32_t i = 0u; i < seg; ++i) {
            float a = static_cast<float>(i) / static_cast<float>(seg) * APC_TWO_PI;
            Vec3 p = Vec3::add(draw_cmd.position,
                Vec3(wobble_r * std::cos(a), wobble_r * std::sin(a), 0.0f));
            dd.list.add_point(p, wobble_color, 2.0f);
        }
    }

    /// Get trail buffer for external access.
    const BallTrailBuffer& get_trail() const { return trail; }

    /// Clear trail.
    void reset_trail() { trail.reset(); }

    /// Clear all state.
    void clear() {
        draw_cmd = BallDrawCommand();
        trail.reset();
    }

private:
    /// Draw prolate spheroid wireframe (ellipsoid of revolution).
    void draw_prolate_wireframe(DebugDraw& dd, const Vec3& center,
                                 float semi_major, float semi_minor,
                                 const Quat& orientation, const RenderColor& color,
                                 uint32_t segments) const {
        Mat3 rot = Mat3::from_quat(orientation);

        // XY plane circle (scaled by semi_major on X, semi_minor on Y)
        for (uint32_t i = 0u; i < segments; ++i) {
            float a0 = static_cast<float>(i) / static_cast<float>(segments) * APC_TWO_PI;
            float a1 = static_cast<float>(i + 1) / static_cast<float>(segments) * APC_TWO_PI;
            Vec3 l0(semi_major * std::cos(a0), semi_minor * std::sin(a0), 0.0f);
            Vec3 l1(semi_major * std::cos(a1), semi_minor * std::sin(a1), 0.0f);
            dd.list.add_line(Vec3::add(center, rot.transform_vec(l0)),
                            Vec3::add(center, rot.transform_vec(l1)), color);
        }

        // XZ plane circle (scaled by semi_major on X, semi_minor on Z)
        for (uint32_t i = 0u; i < segments; ++i) {
            float a0 = static_cast<float>(i) / static_cast<float>(segments) * APC_TWO_PI;
            float a1 = static_cast<float>(i + 1) / static_cast<float>(segments) * APC_TWO_PI;
            Vec3 l0(semi_major * std::cos(a0), 0.0f, semi_minor * std::sin(a0));
            Vec3 l1(semi_major * std::cos(a1), 0.0f, semi_minor * std::sin(a1));
            dd.list.add_line(Vec3::add(center, rot.transform_vec(l0)),
                            Vec3::add(center, rot.transform_vec(l1)), color);
        }

        // YZ plane circle (scaled by semi_minor on both)
        for (uint32_t i = 0u; i < segments; ++i) {
            float a0 = static_cast<float>(i) / static_cast<float>(segments) * APC_TWO_PI;
            float a1 = static_cast<float>(i + 1) / static_cast<float>(segments) * APC_TWO_PI;
            Vec3 l0(0.0f, semi_minor * std::cos(a0), semi_minor * std::sin(a0));
            Vec3 l1(0.0f, semi_minor * std::cos(a1), semi_minor * std::sin(a1));
            dd.list.add_line(Vec3::add(center, rot.transform_vec(l0)),
                            Vec3::add(center, rot.transform_vec(l1)), color);
        }
    }
};

// =============================================================================
// PossessionIndicator — Ring/circle around controlling athlete
// =============================================================================
struct PossessionIndicator {
    float ring_radius = 0.8f;                // Radius of the indicator ring
    float ring_height_offset = 0.05f;        // Slightly above ground
    uint32_t ring_segments = 16u;
    RenderColor home_color = RenderColor::BLUE();
    RenderColor away_color = RenderColor::RED();
    float pulse_speed = 3.0f;                // Pulses per second
    float min_alpha = 0.3f;
    float max_alpha = 1.0f;

    /// Draw possession ring around an athlete.
    void draw(const Vec3& athlete_position, uint32_t team_id,
              float control_strength, float time, DebugDraw& dd) const {
        RenderColor base_color = (team_id == 1u) ? home_color : away_color;

        // Pulsing alpha based on control strength and time
        float pulse = (std::sin(time * pulse_speed * APC_TWO_PI) + 1.0f) * 0.5f;
        float alpha = min_alpha + (max_alpha - min_alpha) * pulse * control_strength;
        alpha = std::min(1.0f, std::max(0.0f, alpha));

        RenderColor color(base_color.r, base_color.g, base_color.b, alpha);

        float y = athlete_position.y + ring_height_offset;
        for (uint32_t i = 0u; i < ring_segments; ++i) {
            float a0 = static_cast<float>(i) / static_cast<float>(ring_segments) * APC_TWO_PI;
            float a1 = static_cast<float>(i + 1) / static_cast<float>(ring_segments) * APC_TWO_PI;
            Vec3 p0 = Vec3::add(athlete_position,
                Vec3(ring_radius * std::cos(a0), y, ring_radius * std::sin(a0)));
            Vec3 p1 = Vec3::add(athlete_position,
                Vec3(ring_radius * std::cos(a1), y, ring_radius * std::sin(a1)));
            dd.list.add_line(p0, p1, color);
        }
    }

    /// Draw with default colors using team_id.
    void draw(const Vec3& athlete_position, uint32_t team_id,
              float control_strength, DebugDraw& dd) const {
        draw(athlete_position, team_id, control_strength, 0.0f, dd);
    }
};

} // namespace apc
