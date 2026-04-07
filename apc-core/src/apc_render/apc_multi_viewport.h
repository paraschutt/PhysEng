#pragma once
// =============================================================================
// Multi-Viewport — Split-screen, per-viewport camera, performance overlay
// =============================================================================
//
// Provides:
//   - ViewportConfig: per-viewport configuration
//   - SplitLayout: split-screen layout enumeration
//   - MultiViewport: manages up to 4 viewports with layout control
//   - RenderPerformanceOverlay: draws FPS, physics stats, render stats
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation (fixed-capacity arrays)
//   - Deterministic: same inputs produce same outputs
//   - C++17
//
// =============================================================================

#include "apc_render_types.h"
#include "apc_debug_draw.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_math_common.h"
#include <cstdint>
#include <cmath>

namespace apc {

// =============================================================================
// ViewportConfig — Per-viewport configuration
// =============================================================================
struct ViewportConfig {
    float x      = 0.0f;     // Top-left X (normalized 0-1 or pixel)
    float y      = 0.0f;     // Top-left Y
    float width  = 1.0f;     // Width
    float height = 1.0f;     // Height
    uint32_t camera_id        = 0u;    // Assigned camera
    uint32_t render_layer_mask = 0xFFu; // Which render layers to draw
    RenderColor clear_color   = RenderColor(0.1f, 0.1f, 0.15f, 1.0f);

    // Render layer bit flags
    static constexpr uint32_t LAYER_OPAQUE      = 1u << 0;
    static constexpr uint32_t LAYER_TRANSPARENT  = 1u << 1;
    static constexpr uint32_t LAYER_OVERLAY      = 1u << 2;
    static constexpr uint32_t LAYER_DEBUG        = 1u << 3;
    static constexpr uint32_t LAYER_HUD          = 1u << 4;

    // Common presets
    static uint32_t layer_main_view() {
        return LAYER_OPAQUE | LAYER_TRANSPARENT;
    }

    static uint32_t layer_debug_view() {
        return LAYER_OVERLAY | LAYER_DEBUG;
    }

    static uint32_t layer_minimap() {
        return LAYER_OPAQUE;
    }

    static uint32_t layer_all() {
        return LAYER_OPAQUE | LAYER_TRANSPARENT | LAYER_OVERLAY |
               LAYER_DEBUG | LAYER_HUD;
    }

    // --- contains_point: check if a point (normalized) is inside this viewport ---
    bool contains_point(float px, float py) const {
        return px >= x && px <= x + width &&
               py >= y && py <= y + height;
    }

    // --- get_aspect_ratio ---
    float get_aspect_ratio() const {
        if (height < APC_EPSILON) return 1.0f;
        return width / height;
    }
};

// =============================================================================
// SplitLayout — Split-screen layout type
// =============================================================================
enum class SplitLayout : uint8_t {
    FULL        = 0,   // Single viewport (no split)
    HORIZONTAL  = 1,   // Side-by-side (2-player)
    VERTICAL    = 2,   // Top-bottom (2-player)
    QUAD        = 3,   // 2x2 grid (4-player)
    CUSTOM      = 4    // Arbitrary regions
};

// =============================================================================
// MultiViewport — Manages up to 4 viewports with split-screen layouts
// =============================================================================
struct MultiViewport {
    static constexpr uint32_t MAX_VIEWPORTS = 4u;

    ViewportConfig viewports[MAX_VIEWPORTS];
    uint32_t       viewport_count = 1u;
    SplitLayout    layout         = SplitLayout::FULL;

    // --- add_viewport ---
    uint32_t add_viewport(const ViewportConfig& config) {
        if (viewport_count >= MAX_VIEWPORTS) return 0xFFFFFFFF;
        uint32_t id = viewport_count;
        viewports[viewport_count] = config;
        ++viewport_count;
        return id;
    }

    // --- set_split_layout: apply a standard split-screen layout ---
    void set_split_layout(SplitLayout new_layout) {
        layout = new_layout;

        switch (layout) {
        case SplitLayout::FULL:
            viewport_count = 1u;
            viewports[0] = ViewportConfig();
            viewports[0].x = 0.0f;
            viewports[0].y = 0.0f;
            viewports[0].width = 1.0f;
            viewports[0].height = 1.0f;
            viewports[0].render_layer_mask = ViewportConfig::layer_all();
            break;

        case SplitLayout::HORIZONTAL:
            viewport_count = 2u;
            // Left half
            viewports[0] = ViewportConfig();
            viewports[0].x = 0.0f;
            viewports[0].y = 0.0f;
            viewports[0].width = 0.5f;
            viewports[0].height = 1.0f;
            viewports[0].camera_id = 0u;
            viewports[0].render_layer_mask = ViewportConfig::layer_all();
            // Right half
            viewports[1] = ViewportConfig();
            viewports[1].x = 0.5f;
            viewports[1].y = 0.0f;
            viewports[1].width = 0.5f;
            viewports[1].height = 1.0f;
            viewports[1].camera_id = 1u;
            viewports[1].render_layer_mask = ViewportConfig::layer_all();
            break;

        case SplitLayout::VERTICAL:
            viewport_count = 2u;
            // Top half
            viewports[0] = ViewportConfig();
            viewports[0].x = 0.0f;
            viewports[0].y = 0.0f;
            viewports[0].width = 1.0f;
            viewports[0].height = 0.5f;
            viewports[0].camera_id = 0u;
            viewports[0].render_layer_mask = ViewportConfig::layer_all();
            // Bottom half
            viewports[1] = ViewportConfig();
            viewports[1].x = 0.0f;
            viewports[1].y = 0.5f;
            viewports[1].width = 1.0f;
            viewports[1].height = 0.5f;
            viewports[1].camera_id = 1u;
            viewports[1].render_layer_mask = ViewportConfig::layer_all();
            break;

        case SplitLayout::QUAD:
            viewport_count = 4u;
            // Top-left
            viewports[0] = ViewportConfig();
            viewports[0].x = 0.0f;   viewports[0].y = 0.0f;
            viewports[0].width = 0.5f; viewports[0].height = 0.5f;
            viewports[0].camera_id = 0u;
            viewports[0].render_layer_mask = ViewportConfig::layer_all();
            // Top-right
            viewports[1] = ViewportConfig();
            viewports[1].x = 0.5f;   viewports[1].y = 0.0f;
            viewports[1].width = 0.5f; viewports[1].height = 0.5f;
            viewports[1].camera_id = 1u;
            viewports[1].render_layer_mask = ViewportConfig::layer_all();
            // Bottom-left
            viewports[2] = ViewportConfig();
            viewports[2].x = 0.0f;   viewports[2].y = 0.5f;
            viewports[2].width = 0.5f; viewports[2].height = 0.5f;
            viewports[2].camera_id = 2u;
            viewports[2].render_layer_mask = ViewportConfig::layer_all();
            // Bottom-right
            viewports[3] = ViewportConfig();
            viewports[3].x = 0.5f;   viewports[3].y = 0.5f;
            viewports[3].width = 0.5f; viewports[3].height = 0.5f;
            viewports[3].camera_id = 3u;
            viewports[3].render_layer_mask = ViewportConfig::layer_all();
            break;

        case SplitLayout::CUSTOM:
            // User must configure viewports manually
            break;
        }
    }

    // --- resize: update viewport dimensions (for window resize) ---
    void resize(float total_width, float total_height) {
        (void)total_width;
        (void)total_height;
        // Viewport positions are stored in normalized [0,1] coordinates,
        // so no recalculation needed on resize.
        // If pixel coordinates were used, they would be recalculated here.
    }

    // --- get_viewport: get viewport by index ---
    const ViewportConfig* get_viewport(uint32_t index) const {
        if (index >= viewport_count) return nullptr;
        return &viewports[index];
    }

    // --- get_viewport_count ---
    uint32_t get_viewport_count() const { return viewport_count; }

    // --- get_layout ---
    SplitLayout get_layout() const { return layout; }

    // --- set_viewport_camera: assign a camera to a viewport ---
    bool set_viewport_camera(uint32_t viewport_index, uint32_t camera_id) {
        if (viewport_index >= viewport_count) return false;
        viewports[viewport_index].camera_id = camera_id;
        return true;
    }

    // --- set_viewport_layers: set render layer mask for a viewport ---
    bool set_viewport_layers(uint32_t viewport_index, uint32_t layer_mask) {
        if (viewport_index >= viewport_count) return false;
        viewports[viewport_index].render_layer_mask = layer_mask;
        return true;
    }

    // --- draw_viewport_borders: draw separator lines between viewports ---
    void draw_viewport_borders(DebugDraw& dd, float screen_w, float screen_h) const {
        if (layout == SplitLayout::FULL || viewport_count <= 1u) return;

        RenderColor border_color = RenderColor(0.3f, 0.3f, 0.3f, 1.0f);

        switch (layout) {
        case SplitLayout::HORIZONTAL: {
            // Vertical divider at center
            float mx = screen_w * 0.5f;
            Vec3 top(mx, 0.0f, 0.0f);
            Vec3 bot(mx, screen_h, 0.0f);
            dd.list.add_line(top, bot, border_color);
            break;
        }
        case SplitLayout::VERTICAL: {
            // Horizontal divider at center
            float my = screen_h * 0.5f;
            Vec3 left(0.0f, my, 0.0f);
            Vec3 right(screen_w, my, 0.0f);
            dd.list.add_line(left, right, border_color);
            break;
        }
        case SplitLayout::QUAD: {
            // Both dividers
            float mx = screen_w * 0.5f;
            float my = screen_h * 0.5f;
            dd.list.add_line(Vec3(mx, 0.0f, 0.0f), Vec3(mx, screen_h, 0.0f), border_color);
            dd.list.add_line(Vec3(0.0f, my, 0.0f), Vec3(screen_w, my, 0.0f), border_color);
            break;
        }
        default:
            break;
        }
    }

    // --- reset ---
    void reset() {
        viewport_count = 1u;
        layout = SplitLayout::FULL;
        viewports[0] = ViewportConfig();
    }
};

// =============================================================================
// RenderPerformanceOverlay — Performance statistics display
// =============================================================================
struct RenderPerformanceOverlay {
    float fps                    = 60.0f;
    float frame_time_ms          = 16.67f;
    float physics_frame_time_ms  = 5.0f;
    uint32_t contact_count       = 0u;
    uint32_t broadphase_pairs    = 0u;
    uint32_t draw_calls          = 0u;
    uint32_t triangle_count      = 0u;
    uint32_t line_count          = 0u;
    uint32_t point_count         = 0u;

    // FPS tracking
    float fps_accumulator        = 0.0f;
    uint32_t fps_frame_count     = 0u;
    float fps_update_interval    = 0.5f; // Update FPS display every 0.5s
    float fps_timer              = 0.0f;

    bool show_fps                = true;
    bool show_physics_stats      = false;
    bool show_render_stats       = false;
    bool show_memory_stats       = false;
    float position_x             = 10.0f;
    float position_y             = 10.0f;
    float line_spacing           = 15.0f;

    // --- update_frame: record per-frame stats ---
    void update_frame(float dt, const RenderStats& render_stats) {
        frame_time_ms = render_stats.frame_time_ms > 0.0f
            ? render_stats.frame_time_ms
            : dt * 1000.0f;
        draw_calls    = render_stats.draw_calls;
        triangle_count = render_stats.triangle_count;
        line_count    = render_stats.line_count;
        point_count   = render_stats.point_count;

        // FPS calculation
        fps_timer += dt;
        fps_frame_count++;
        fps_accumulator += dt;

        if (fps_timer >= fps_update_interval) {
            if (fps_accumulator > APC_EPSILON) {
                fps = static_cast<float>(fps_frame_count) / fps_accumulator;
            }
            fps_accumulator = 0.0f;
            fps_frame_count = 0u;
            fps_timer = 0.0f;
        }
    }

    // --- update_physics_stats: record physics-side stats ---
    void update_physics_stats(float physics_dt, uint32_t contacts, uint32_t broadphase) {
        physics_frame_time_ms = physics_dt * 1000.0f;
        contact_count = contacts;
        broadphase_pairs = broadphase;
    }

    // --- draw_fps: draw FPS counter ---
    void draw_fps(DebugDraw& dd) const {
        if (!show_fps) return;

        // FPS bar (simple representation as lines)
        // Green bar length proportional to FPS (60fps = full)
        float bar_length = (fps / 60.0f) * 100.0f;
        if (bar_length > 100.0f) bar_length = 100.0f;

        RenderColor fps_color;
        if (fps >= 55.0f) {
            fps_color = RenderColor::GREEN();
        } else if (fps >= 30.0f) {
            fps_color = RenderColor::YELLOW();
        } else {
            fps_color = RenderColor::RED();
        }

        // Draw bar
        Vec3 start(position_x, position_y, 0.0f);
        Vec3 end(position_x + bar_length, position_y, 0.0f);
        dd.list.add_line(start, end, fps_color);
    }

    // --- draw_physics_stats: draw physics performance info ---
    void draw_physics_stats(DebugDraw& dd) const {
        if (!show_physics_stats) return;

        float y = position_y + line_spacing;

        // Physics frame time bar (max 16ms = full)
        float phys_bar = (physics_frame_time_ms / 16.0f) * 100.0f;
        if (phys_bar > 100.0f) phys_bar = 100.0f;

        RenderColor phys_color = physics_frame_time_ms < 8.0f
            ? RenderColor::GREEN()
            : (physics_frame_time_ms < 12.0f ? RenderColor::YELLOW() : RenderColor::RED());

        Vec3 pstart(position_x, y, 0.0f);
        Vec3 pend(position_x + phys_bar, y, 0.0f);
        dd.list.add_line(pstart, pend, phys_color);

        // Contact count bar (max 100 = full)
        y += line_spacing;
        float contact_bar = (static_cast<float>(contact_count) / 100.0f) * 100.0f;
        if (contact_bar > 100.0f) contact_bar = 100.0f;

        RenderColor contact_color = RenderColor::CYAN();
        Vec3 cstart(position_x, y, 0.0f);
        Vec3 cend(position_x + contact_bar, y, 0.0f);
        dd.list.add_line(cstart, cend, contact_color);

        // Broadphase pairs bar (max 500 = full)
        y += line_spacing;
        float bp_bar = (static_cast<float>(broadphase_pairs) / 500.0f) * 100.0f;
        if (bp_bar > 100.0f) bp_bar = 100.0f;

        RenderColor bp_color = RenderColor::MAGENTA();
        Vec3 bstart(position_x, y, 0.0f);
        Vec3 bend(position_x + bp_bar, y, 0.0f);
        dd.list.add_line(bstart, bend, bp_color);
    }

    // --- draw_render_stats: draw rendering performance info ---
    void draw_render_stats(DebugDraw& dd) const {
        if (!show_render_stats) return;

        float y = position_y + line_spacing;

        // Draw calls bar (max 500 = full)
        float dc_bar = (static_cast<float>(draw_calls) / 500.0f) * 100.0f;
        if (dc_bar > 100.0f) dc_bar = 100.0f;

        RenderColor dc_color = RenderColor::ORANGE();
        Vec3 dcstart(position_x, y, 0.0f);
        Vec3 dcend(position_x + dc_bar, y, 0.0f);
        dd.list.add_line(dcstart, dcend, dc_color);

        // Triangle count bar (max 10000 = full)
        y += line_spacing;
        float tri_bar = (static_cast<float>(triangle_count) / 10000.0f) * 100.0f;
        if (tri_bar > 100.0f) tri_bar = 100.0f;

        RenderColor tri_color = RenderColor::BLUE();
        Vec3 tstart(position_x, y, 0.0f);
        Vec3 tend(position_x + tri_bar, y, 0.0f);
        dd.list.add_line(tstart, tend, tri_color);
    }

    // --- draw_memory_stats: draw memory usage info ---
    void draw_memory_stats(DebugDraw& dd) const {
        if (!show_memory_stats) return;

        // Memory stats are represented as bars
        // Since we can't query actual memory, we show fixed indicators
        float y = position_y + line_spacing;

        // Fixed memory bar (placeholder)
        RenderColor mem_color = RenderColor::GRAY();
        Vec3 mstart(position_x, y, 0.0f);
        Vec3 mend(position_x + 30.0f, y, 0.0f); // Fixed 30% placeholder
        dd.list.add_line(mstart, mend, mem_color);
    }

    // --- draw_all: draw all enabled stat bars ---
    void draw_all(DebugDraw& dd) const {
        draw_fps(dd);
        draw_physics_stats(dd);
        draw_render_stats(dd);
        draw_memory_stats(dd);
    }

    // --- get_fps ---
    float get_fps() const { return fps; }

    // --- get_frame_time ---
    float get_frame_time_ms() const { return frame_time_ms; }

    // --- reset ---
    void reset() {
        fps = 60.0f;
        frame_time_ms = 16.67f;
        physics_frame_time_ms = 5.0f;
        contact_count = 0u;
        broadphase_pairs = 0u;
        draw_calls = 0u;
        triangle_count = 0u;
        line_count = 0u;
        point_count = 0u;
        fps_accumulator = 0.0f;
        fps_frame_count = 0u;
        fps_timer = 0.0f;
    }
};

} // namespace apc
