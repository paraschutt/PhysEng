#pragma once
// =============================================================================
// Render Types — Core data types for the APC render pipeline
// =============================================================================
//
// Provides:
//   - Vec2: simple 2D vector
//   - RenderColor: RGBA float with pack/unpack to uint32
//   - RenderVertex: position, normal, uv, color
//   - PrimitiveTopology: draw mode enumeration
//   - RenderMaterial: material properties
//   - RenderPassType: render pass enumeration
//   - RenderStats: per-frame statistics
//   - RenderViewUniforms: camera/view uniforms for shaders
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation
//   - C++17
//
// =============================================================================

#include "apc_math/apc_vec3.h"
#include "apc_math/apc_math_common.h"
#include <cstdint>
#include <cmath>

namespace apc {

// ---------------------------------------------------------------------------
// Vec2 — Simple 2D vector
// ---------------------------------------------------------------------------
struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;

    Vec2() = default;
    Vec2(float x_, float y_) : x(x_), y(y_) {}

    static Vec2 add(const Vec2& a, const Vec2& b) {
        return Vec2(a.x + b.x, a.y + b.y);
    }

    static Vec2 sub(const Vec2& a, const Vec2& b) {
        return Vec2(a.x - b.x, a.y - b.y);
    }

    static Vec2 scale(const Vec2& v, float s) {
        return Vec2(v.x * s, v.y * s);
    }

    static Vec2 lerp(const Vec2& a, const Vec2& b, float t) {
        return Vec2(
            a.x + (b.x - a.x) * t,
            a.y + (b.y - a.y) * t
        );
    }
};

// ---------------------------------------------------------------------------
// RenderColor — RGBA float (0.0-1.0 per channel)
// ---------------------------------------------------------------------------
struct RenderColor {
    float r = 1.0f;
    float g = 1.0f;
    float b = 1.0f;
    float a = 1.0f;

    RenderColor() = default;
    RenderColor(float r_, float g_, float b_, float a_ = 1.0f)
        : r(r_), g(g_), b(b_), a(a_) {}

    // Pack to uint32 ABGR (common GPU format)
    uint32_t pack_u32() const {
        uint32_t rb = static_cast<uint32_t>(r * 255.0f + 0.5f);
        uint32_t gb = static_cast<uint32_t>(g * 255.0f + 0.5f);
        uint32_t bb = static_cast<uint32_t>(b * 255.0f + 0.5f);
        uint32_t ab = static_cast<uint32_t>(a * 255.0f + 0.5f);
        return (ab << 24u) | (bb << 16u) | (gb << 8u) | rb;
    }

    // Unpack from uint32 ABGR
    static RenderColor unpack_u32(uint32_t packed) {
        RenderColor c;
        c.r = static_cast<float>( packed        & 0xFFu) / 255.0f;
        c.g = static_cast<float>((packed >> 8u)  & 0xFFu) / 255.0f;
        c.b = static_cast<float>((packed >> 16u) & 0xFFu) / 255.0f;
        c.a = static_cast<float>((packed >> 24u) & 0xFFu) / 255.0f;
        return c;
    }

    // RGB factory (alpha defaults to 1.0)
    static RenderColor from_rgb(float r_, float g_, float b_) {
        return RenderColor(r_, g_, b_, 1.0f);
    }

    // --- Named constants ---
    static RenderColor WHITE()   { return RenderColor(1.0f, 1.0f, 1.0f, 1.0f); }
    static RenderColor BLACK()   { return RenderColor(0.0f, 0.0f, 0.0f, 1.0f); }
    static RenderColor RED()     { return RenderColor(1.0f, 0.0f, 0.0f, 1.0f); }
    static RenderColor GREEN()   { return RenderColor(0.0f, 1.0f, 0.0f, 1.0f); }
    static RenderColor BLUE()    { return RenderColor(0.0f, 0.0f, 1.0f, 1.0f); }
    static RenderColor YELLOW()  { return RenderColor(1.0f, 1.0f, 0.0f, 1.0f); }
    static RenderColor CYAN()    { return RenderColor(0.0f, 1.0f, 1.0f, 1.0f); }
    static RenderColor MAGENTA() { return RenderColor(1.0f, 0.0f, 1.0f, 1.0f); }
    static RenderColor GRAY()    { return RenderColor(0.5f, 0.5f, 0.5f, 1.0f); }
    static RenderColor ORANGE()  { return RenderColor(1.0f, 0.5f, 0.0f, 1.0f); }
};

// ---------------------------------------------------------------------------
// RenderVertex — Position + normal + UV + color
// ---------------------------------------------------------------------------
struct RenderVertex {
    Vec3 position = Vec3(0.0f, 0.0f, 0.0f);
    Vec3 normal   = Vec3(0.0f, 1.0f, 0.0f);
    Vec2 uv       = Vec2(0.0f, 0.0f);
    RenderColor color = RenderColor::WHITE();

    RenderVertex() = default;
};

// ---------------------------------------------------------------------------
// PrimitiveTopology — Draw mode enumeration
// ---------------------------------------------------------------------------
enum class PrimitiveTopology : uint8_t {
    TRIANGLES       = 0,
    LINES           = 1,
    POINTS          = 2,
    LINE_STRIP      = 3,
    TRIANGLE_STRIP  = 4
};

// ---------------------------------------------------------------------------
// RenderMaterial — Material properties for draw calls
// ---------------------------------------------------------------------------
struct RenderMaterial {
    RenderColor albedo_color    = RenderColor::WHITE();
    RenderColor emissive_color  = RenderColor::BLACK();
    bool wireframe              = false;
    float transparency          = 0.0f;    // 0 = opaque, 1 = fully transparent
    float line_width            = 1.0f;
    float point_size            = 1.0f;
    uint32_t material_id        = 0u;
    bool double_sided           = false;

    static RenderMaterial make_default() {
        RenderMaterial m;
        return m;
    }
};

// ---------------------------------------------------------------------------
// RenderPassType — Render pass enumeration
// ---------------------------------------------------------------------------
enum class RenderPassType : uint8_t {
    OPAQUE       = 0,
    TRANSPARENT  = 1,
    WIREFRAME    = 2,
    OVERLAY      = 3,
    DEBUG        = 4,
    HUD          = 5
};

// ---------------------------------------------------------------------------
// RenderStats — Per-frame render statistics
// ---------------------------------------------------------------------------
struct RenderStats {
    uint32_t draw_calls     = 0u;
    uint32_t triangle_count = 0u;
    uint32_t line_count     = 0u;
    uint32_t point_count    = 0u;
    float frame_time_ms     = 0.0f;
    float gpu_time_ms       = 0.0f;

    void reset() {
        draw_calls     = 0u;
        triangle_count = 0u;
        line_count     = 0u;
        point_count    = 0u;
        frame_time_ms  = 0.0f;
        gpu_time_ms    = 0.0f;
    }
};

// ---------------------------------------------------------------------------
// RenderViewUniforms — Camera/view uniforms for shaders
// ---------------------------------------------------------------------------
struct RenderViewUniforms {
    float view_matrix[16]       = {};  // column-major 4x4
    float projection_matrix[16] = {};  // column-major 4x4
    float view_projection[16]   = {};  // column-major 4x4
    Vec3  eye_position          = Vec3(0.0f, 0.0f, 0.0f);
    float viewport_x            = 0.0f;
    float viewport_y            = 0.0f;
    float viewport_w            = 0.0f;
    float viewport_h            = 0.0f;
    float near_plane            = 0.1f;
    float far_plane             = 100.0f;

    // Compute VP = view * projection (column-major 4x4 multiply)
    void compute_view_projection() {
        for (int col = 0; col < 4; ++col) {
            for (int row = 0; row < 4; ++row) {
                float sum = 0.0f;
                for (int k = 0; k < 4; ++k) {
                    sum += view_matrix[k * 4 + row] * projection_matrix[col * 4 + k];
                }
                view_projection[col * 4 + row] = sum;
            }
        }
    }
};

} // namespace apc
