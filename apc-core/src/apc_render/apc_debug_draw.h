#pragma once
// =============================================================================
// Debug Draw — Debug visualization system for physics shapes
// =============================================================================
//
// Provides:
//   - DebugDrawColor: bitmask flags for debug visualization categories
//   - DebugDrawFlags: uint16_t alias for DebugDrawColor bitmask
//   - DebugVertex, DebugLine, DebugTriangle, DebugPoint: debug primitives
//   - DebugDrawList: fixed-capacity buffer for debug primitives
//   - DebugDraw: high-level shape drawing (AABB, sphere, capsule, etc.)
//   - ShapeDebugRenderer: dispatches CollisionShape to appropriate debug draw
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation (fixed-capacity arrays)
//   - C++17
//
// =============================================================================

#include "apc_render_types.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include "apc_math/apc_mat3.h"
#include "apc_math/apc_math_common.h"
#include "apc_collision/apc_collision_dispatch.h"
#include <cstdint>
#include <cmath>

namespace apc {

// ---------------------------------------------------------------------------
// DebugDrawColor — Bitfield flags for debug visualization categories
// ---------------------------------------------------------------------------
enum class DebugDrawColor : uint16_t {
    CONTACT_NORMAL    = 1u,
    CONTACT_POINT     = 2u,
    AABB              = 4u,
    VELOCITY          = 8u,
    FORCE             = 16u,
    JOINT             = 32u,
    BONE_ANIM         = 64u,
    BONE_PHYSICS      = 128u,
    BONE_BLENDED      = 256u,
    PENETRATION       = 512u,
    BROADPHASE_PAIR   = 1024u,
    ALL               = 0xFFFFu
};

using DebugDrawFlags = uint16_t;

// ---------------------------------------------------------------------------
// DebugVertex — Vertex with position and color
// ---------------------------------------------------------------------------
struct DebugVertex {
    Vec3 position = Vec3(0.0f, 0.0f, 0.0f);
    RenderColor color = RenderColor::WHITE();

    DebugVertex() = default;
    DebugVertex(const Vec3& pos, const RenderColor& col)
        : position(pos), color(col) {}
};

// ---------------------------------------------------------------------------
// DebugLine — Line segment
// ---------------------------------------------------------------------------
struct DebugLine {
    DebugVertex a;
    DebugVertex b;

    DebugLine() = default;
    DebugLine(const DebugVertex& a_, const DebugVertex& b_) : a(a_), b(b_) {}
};

// ---------------------------------------------------------------------------
// DebugTriangle — Triangle
// ---------------------------------------------------------------------------
struct DebugTriangle {
    DebugVertex v0;
    DebugVertex v1;
    DebugVertex v2;

    DebugTriangle() = default;
};

// ---------------------------------------------------------------------------
// DebugPoint — Point with size
// ---------------------------------------------------------------------------
struct DebugPoint {
    Vec3 position = Vec3(0.0f, 0.0f, 0.0f);
    RenderColor color = RenderColor::WHITE();
    float size = 4.0f;

    DebugPoint() = default;
    DebugPoint(const Vec3& pos, const RenderColor& col, float sz)
        : position(pos), color(col), size(sz) {}
};

// ---------------------------------------------------------------------------
// DebugDrawList — Fixed-capacity buffer for debug primitives
// ---------------------------------------------------------------------------
struct DebugDrawList {
    static constexpr uint32_t MAX_LINES     = 4096u;
    static constexpr uint32_t MAX_POINTS    = 2048u;
    static constexpr uint32_t MAX_TRIANGLES = 2048u;

    DebugLine lines[MAX_LINES];
    uint32_t line_count = 0u;

    DebugPoint points[MAX_POINTS];
    uint32_t point_count = 0u;

    DebugTriangle triangles[MAX_TRIANGLES];
    uint32_t triangle_count = 0u;

    void add_line(const Vec3& a, const Vec3& b, const RenderColor& color) {
        if (line_count < MAX_LINES) {
            lines[line_count] = DebugLine(DebugVertex(a, color), DebugVertex(b, color));
            ++line_count;
        }
    }

    void add_line(const Vec3& a_pos, const RenderColor& a_color,
                  const Vec3& b_pos, const RenderColor& b_color) {
        if (line_count < MAX_LINES) {
            lines[line_count] = DebugLine(DebugVertex(a_pos, a_color), DebugVertex(b_pos, b_color));
            ++line_count;
        }
    }

    void add_point(const Vec3& pos, const RenderColor& color, float size = 4.0f) {
        if (point_count < MAX_POINTS) {
            points[point_count] = DebugPoint(pos, color, size);
            ++point_count;
        }
    }

    void add_triangle(const Vec3& v0, const Vec3& v1, const Vec3& v2,
                      const RenderColor& color) {
        if (triangle_count < MAX_TRIANGLES) {
            triangles[triangle_count] = DebugTriangle();
            triangles[triangle_count].v0 = DebugVertex(v0, color);
            triangles[triangle_count].v1 = DebugVertex(v1, color);
            triangles[triangle_count].v2 = DebugVertex(v2, color);
            ++triangle_count;
        }
    }

    void clear() {
        line_count = 0u;
        point_count = 0u;
        triangle_count = 0u;
    }

    uint32_t get_line_count() const { return line_count; }
    uint32_t get_point_count() const { return point_count; }
    uint32_t get_triangle_count() const { return triangle_count; }

    const DebugLine* get_lines() const { return lines; }
    const DebugPoint* get_points() const { return points; }
    const DebugTriangle* get_triangles() const { return triangles; }
};

// ---------------------------------------------------------------------------
// DebugDraw — High-level shape drawing
// ---------------------------------------------------------------------------
struct DebugDraw {
    DebugDrawList list;

    // Draw AABB wireframe (12 edges of a box)
    void draw_aabb(const Vec3& min, const Vec3& max, const RenderColor& color) {
        // 8 corners
        Vec3 corners[8] = {
            Vec3(min.x, min.y, min.z), Vec3(max.x, min.y, min.z),
            Vec3(min.x, max.y, min.z), Vec3(max.x, max.y, min.z),
            Vec3(min.x, min.y, max.z), Vec3(max.x, min.y, max.z),
            Vec3(min.x, max.y, max.z), Vec3(max.x, max.y, max.z)
        };
        // 12 edges
        // Bottom face
        list.add_line(corners[0], corners[1], color);
        list.add_line(corners[1], corners[3], color);
        list.add_line(corners[3], corners[2], color);
        list.add_line(corners[2], corners[0], color);
        // Top face
        list.add_line(corners[4], corners[5], color);
        list.add_line(corners[5], corners[7], color);
        list.add_line(corners[7], corners[6], color);
        list.add_line(corners[6], corners[4], color);
        // Vertical edges
        list.add_line(corners[0], corners[4], color);
        list.add_line(corners[1], corners[5], color);
        list.add_line(corners[2], corners[6], color);
        list.add_line(corners[3], corners[7], color);
    }

    // Draw sphere wireframe (3 circles in XY, XZ, YZ planes)
    void draw_sphere_wireframe(const Vec3& center, float radius,
                               const RenderColor& color,
                               uint32_t segments = 12u) {
        // XY plane circle
        for (uint32_t i = 0; i < segments; ++i) {
            float a0 = static_cast<float>(i) / static_cast<float>(segments) * APC_TWO_PI;
            float a1 = static_cast<float>(i + 1) / static_cast<float>(segments) * APC_TWO_PI;
            Vec3 p0 = Vec3::add(center, Vec3(radius * std::cos(a0),
                                              radius * std::sin(a0), 0.0f));
            Vec3 p1 = Vec3::add(center, Vec3(radius * std::cos(a1),
                                              radius * std::sin(a1), 0.0f));
            list.add_line(p0, p1, color);
        }
        // XZ plane circle
        for (uint32_t i = 0; i < segments; ++i) {
            float a0 = static_cast<float>(i) / static_cast<float>(segments) * APC_TWO_PI;
            float a1 = static_cast<float>(i + 1) / static_cast<float>(segments) * APC_TWO_PI;
            Vec3 p0 = Vec3::add(center, Vec3(radius * std::cos(a0), 0.0f,
                                              radius * std::sin(a0)));
            Vec3 p1 = Vec3::add(center, Vec3(radius * std::cos(a1), 0.0f,
                                              radius * std::sin(a1)));
            list.add_line(p0, p1, color);
        }
        // YZ plane circle
        for (uint32_t i = 0; i < segments; ++i) {
            float a0 = static_cast<float>(i) / static_cast<float>(segments) * APC_TWO_PI;
            float a1 = static_cast<float>(i + 1) / static_cast<float>(segments) * APC_TWO_PI;
            Vec3 p0 = Vec3::add(center, Vec3(0.0f, radius * std::cos(a0),
                                              radius * std::sin(a0)));
            Vec3 p1 = Vec3::add(center, Vec3(0.0f, radius * std::cos(a1),
                                              radius * std::sin(a1)));
            list.add_line(p0, p1, color);
        }
    }

    // Draw capsule wireframe (cylinder wireframe + 2 half-circles at ends)
    void draw_capsule_wireframe(const Vec3& center, float radius,
                                float half_height, const Quat& orientation,
                                const RenderColor& color,
                                uint32_t segments = 12u) {
        Mat3 rot = Mat3::from_quat(orientation);

        // Cylinder wireframe: circles at top and bottom + vertical lines
        Vec3 axis_top    = Vec3::add(center, rot.transform_vec(Vec3(0.0f,  half_height, 0.0f)));
        Vec3 axis_bottom = Vec3::add(center, rot.transform_vec(Vec3(0.0f, -half_height, 0.0f)));

        // Circles at top and bottom (in XZ plane of local space)
        for (uint32_t i = 0; i < segments; ++i) {
            float a0 = static_cast<float>(i) / static_cast<float>(segments) * APC_TWO_PI;
            float a1 = static_cast<float>(i + 1) / static_cast<float>(segments) * APC_TWO_PI;
            Vec3 local0_top    = Vec3(radius * std::cos(a0), 0.0f, radius * std::sin(a0));
            Vec3 local1_top    = Vec3(radius * std::cos(a1), 0.0f, radius * std::sin(a1));
            Vec3 local0_bottom = Vec3(radius * std::cos(a0), 0.0f, radius * std::sin(a0));
            Vec3 local1_bottom = Vec3(radius * std::cos(a1), 0.0f, radius * std::sin(a1));

            Vec3 w0t = Vec3::add(axis_top, rot.transform_vec(local0_top));
            Vec3 w1t = Vec3::add(axis_top, rot.transform_vec(local1_top));
            Vec3 w0b = Vec3::add(axis_bottom, rot.transform_vec(local0_bottom));
            Vec3 w1b = Vec3::add(axis_bottom, rot.transform_vec(local1_bottom));

            list.add_line(w0t, w1t, color);
            list.add_line(w0b, w1b, color);
        }

        // Vertical lines connecting top and bottom circles
        for (uint32_t i = 0; i < segments; i += 3) {
            float a = static_cast<float>(i) / static_cast<float>(segments) * APC_TWO_PI;
            Vec3 local = Vec3(radius * std::cos(a), 0.0f, radius * std::sin(a));
            Vec3 wt = Vec3::add(axis_top, rot.transform_vec(local));
            Vec3 wb = Vec3::add(axis_bottom, rot.transform_vec(local));
            list.add_line(wt, wb, color);
        }

        // Half-circles at top and bottom (in XY plane)
        for (uint32_t i = 0; i < segments / 2; ++i) {
            float a0 = static_cast<float>(i) / static_cast<float>(segments / 2) * APC_PI;
            float a1 = static_cast<float>(i + 1) / static_cast<float>(segments / 2) * APC_PI;

            // Top half-circle
            Vec3 local0_top = Vec3(radius * std::cos(a0), radius * std::sin(a0), 0.0f);
            Vec3 local1_top = Vec3(radius * std::cos(a1), radius * std::sin(a1), 0.0f);
            Vec3 w0t = Vec3::add(axis_top, rot.transform_vec(local0_top));
            Vec3 w1t = Vec3::add(axis_top, rot.transform_vec(local1_top));
            list.add_line(w0t, w1t, color);

            // Bottom half-circle
            Vec3 local0_bot = Vec3(radius * std::cos(a0), -radius * std::sin(a0), 0.0f);
            Vec3 local1_bot = Vec3(radius * std::cos(a1), -radius * std::sin(a1), 0.0f);
            Vec3 w0b = Vec3::add(axis_bottom, rot.transform_vec(local0_bot));
            Vec3 w1b = Vec3::add(axis_bottom, rot.transform_vec(local1_bot));
            list.add_line(w0b, w1b, color);
        }
    }

    // Draw cylinder wireframe (circles at top/bottom + vertical lines)
    void draw_cylinder_wireframe(const Vec3& center, float radius,
                                 float half_height, const Quat& orientation,
                                 const RenderColor& color,
                                 uint32_t segments = 12u) {
        Mat3 rot = Mat3::from_quat(orientation);

        Vec3 axis_top    = Vec3::add(center, rot.transform_vec(Vec3(0.0f,  half_height, 0.0f)));
        Vec3 axis_bottom = Vec3::add(center, rot.transform_vec(Vec3(0.0f, -half_height, 0.0f)));

        // Circles at top and bottom
        for (uint32_t i = 0; i < segments; ++i) {
            float a0 = static_cast<float>(i) / static_cast<float>(segments) * APC_TWO_PI;
            float a1 = static_cast<float>(i + 1) / static_cast<float>(segments) * APC_TWO_PI;
            Vec3 local0 = Vec3(radius * std::cos(a0), 0.0f, radius * std::sin(a0));
            Vec3 local1 = Vec3(radius * std::cos(a1), 0.0f, radius * std::sin(a1));

            Vec3 w0t = Vec3::add(axis_top, rot.transform_vec(local0));
            Vec3 w1t = Vec3::add(axis_top, rot.transform_vec(local1));
            Vec3 w0b = Vec3::add(axis_bottom, rot.transform_vec(local0));
            Vec3 w1b = Vec3::add(axis_bottom, rot.transform_vec(local1));

            list.add_line(w0t, w1t, color);
            list.add_line(w0b, w1b, color);
        }

        // Vertical lines
        for (uint32_t i = 0; i < segments; i += 3) {
            float a = static_cast<float>(i) / static_cast<float>(segments) * APC_TWO_PI;
            Vec3 local = Vec3(radius * std::cos(a), 0.0f, radius * std::sin(a));
            Vec3 wt = Vec3::add(axis_top, rot.transform_vec(local));
            Vec3 wb = Vec3::add(axis_bottom, rot.transform_vec(local));
            list.add_line(wt, wb, color);
        }
    }

    // Draw OBB wireframe (transformed AABB)
    void draw_obb_wireframe(const Vec3& center, const Vec3& extents,
                            const Quat& orientation, const RenderColor& color) {
        Mat3 rot = Mat3::from_quat(orientation);

        Vec3 local_corners[8] = {
            Vec3(-extents.x, -extents.y, -extents.z),
            Vec3( extents.x, -extents.y, -extents.z),
            Vec3(-extents.x,  extents.y, -extents.z),
            Vec3( extents.x,  extents.y, -extents.z),
            Vec3(-extents.x, -extents.y,  extents.z),
            Vec3( extents.x, -extents.y,  extents.z),
            Vec3(-extents.x,  extents.y,  extents.z),
            Vec3( extents.x,  extents.y,  extents.z)
        };

        Vec3 world_corners[8];
        for (int i = 0; i < 8; ++i) {
            world_corners[i] = Vec3::add(center, rot.transform_vec(local_corners[i]));
        }

        // Same edge pattern as AABB (12 edges)
        list.add_line(world_corners[0], world_corners[1], color);
        list.add_line(world_corners[1], world_corners[3], color);
        list.add_line(world_corners[3], world_corners[2], color);
        list.add_line(world_corners[2], world_corners[0], color);
        list.add_line(world_corners[4], world_corners[5], color);
        list.add_line(world_corners[5], world_corners[7], color);
        list.add_line(world_corners[7], world_corners[6], color);
        list.add_line(world_corners[6], world_corners[4], color);
        list.add_line(world_corners[0], world_corners[4], color);
        list.add_line(world_corners[1], world_corners[5], color);
        list.add_line(world_corners[2], world_corners[6], color);
        list.add_line(world_corners[3], world_corners[7], color);
    }

    // Draw plane grid
    void draw_plane_grid(const Vec3& point, const Vec3& normal, float extent,
                         uint32_t divisions, const RenderColor& color) {
        // Build local frame on the plane
        Vec3 up = Vec3(0.0f, 1.0f, 0.0f);
        if (std::abs(Vec3::dot(normal, up)) > 0.99f) {
            up = Vec3(1.0f, 0.0f, 0.0f);
        }
        Vec3 tangent = Vec3::normalize(Vec3::cross(normal, up));
        Vec3 bitangent = Vec3::cross(normal, tangent);

        float step = (2.0f * extent) / static_cast<float>(divisions);

        for (uint32_t i = 0; i <= divisions; ++i) {
            float t = -extent + static_cast<float>(i) * step;

            // Line along tangent direction
            Vec3 p0 = Vec3::add(Vec3::add(point, Vec3::scale(tangent, t)),
                                Vec3::scale(bitangent, -extent));
            Vec3 p1 = Vec3::add(Vec3::add(point, Vec3::scale(tangent, t)),
                                Vec3::scale(bitangent, extent));
            list.add_line(p0, p1, color);

            // Line along bitangent direction
            p0 = Vec3::add(Vec3::add(point, Vec3::scale(bitangent, t)),
                           Vec3::scale(tangent, -extent));
            p1 = Vec3::add(Vec3::add(point, Vec3::scale(bitangent, t)),
                           Vec3::scale(tangent, extent));
            list.add_line(p0, p1, color);
        }
    }

    // Draw contact point (small X marker — 4 short lines)
    void draw_contact_point(const Vec3& point, const RenderColor& color) {
        float s = 0.1f;
        // X marker in XZ plane
        list.add_line(Vec3::add(point, Vec3(-s, 0, -s)),
                       Vec3::add(point, Vec3( s, 0,  s)), color);
        list.add_line(Vec3::add(point, Vec3( s, 0, -s)),
                       Vec3::add(point, Vec3(-s, 0,  s)), color);
        // X marker in XY plane
        list.add_line(Vec3::add(point, Vec3(-s, -s, 0)),
                       Vec3::add(point, Vec3( s,  s, 0)), color);
        list.add_line(Vec3::add(point, Vec3( s, -s, 0)),
                       Vec3::add(point, Vec3(-s,  s, 0)), color);
    }

    // Draw contact normal (arrow from point along normal)
    void draw_contact_normal(const Vec3& point, const Vec3& normal,
                             const RenderColor& color, float length) {
        Vec3 tip = Vec3::add(point, Vec3::scale(normal, length));
        list.add_line(point, tip, color);
    }

    // Draw velocity arrow (shaft + 2 arrowhead lines at 45 degrees)
    void draw_velocity_arrow(const Vec3& position, const Vec3& velocity,
                             const RenderColor& color, float scale) {
        float speed = Vec3::length(velocity);
        if (speed < APC_EPSILON) return;

        Vec3 dir = Vec3::scale(velocity, 1.0f / speed);
        Vec3 tip = Vec3::add(position, Vec3::scale(velocity, scale));

        // Shaft
        list.add_line(position, tip, color);

        // Arrowhead (2 lines at 45 degrees from tip)
        float head_len = speed * scale * 0.2f;
        if (head_len < 0.01f) head_len = 0.01f;
        Vec3 head_base = Vec3::sub(tip, Vec3::scale(dir, head_len));

        // Find a perpendicular vector for arrowhead
        Vec3 perp;
        if (std::abs(dir.y) < 0.99f) {
            perp = Vec3::normalize(Vec3::cross(dir, Vec3(0.0f, 1.0f, 0.0f)));
        } else {
            perp = Vec3::normalize(Vec3::cross(dir, Vec3(1.0f, 0.0f, 0.0f)));
        }

        float head_width = head_len * 0.4f;
        Vec3 left  = Vec3::add(head_base, Vec3::scale(perp, head_width));
        Vec3 right = Vec3::sub(head_base, Vec3::scale(perp, head_width));

        list.add_line(tip, left, color);
        list.add_line(tip, right, color);
    }

    // Draw force arrow (same as velocity arrow)
    void draw_force_arrow(const Vec3& position, const Vec3& force,
                          const RenderColor& color, float scale) {
        draw_velocity_arrow(position, force, color, scale);
    }

    // Draw transform axes (X=red, Y=green, Z=blue)
    void draw_transform_axes(const Vec3& position, const Quat& orientation,
                             float scale) {
        Mat3 rot = Mat3::from_quat(orientation);

        Vec3 x_axis = rot.transform_vec(Vec3(1.0f, 0.0f, 0.0f));
        Vec3 y_axis = rot.transform_vec(Vec3(0.0f, 1.0f, 0.0f));
        Vec3 z_axis = rot.transform_vec(Vec3(0.0f, 0.0f, 1.0f));

        Vec3 x_tip = Vec3::add(position, Vec3::scale(x_axis, scale));
        Vec3 y_tip = Vec3::add(position, Vec3::scale(y_axis, scale));
        Vec3 z_tip = Vec3::add(position, Vec3::scale(z_axis, scale));

        list.add_line(position, x_tip, RenderColor::RED());
        list.add_line(position, y_tip, RenderColor::GREEN());
        list.add_line(position, z_tip, RenderColor::BLUE());
    }

    // Flush — returns const reference to the debug draw list
    const DebugDrawList& flush() const {
        return list;
    }

    // Clear all debug primitives
    void clear() {
        list.clear();
    }
};

// ---------------------------------------------------------------------------
// ShapeDebugRenderer — Dispatches CollisionShape to appropriate DebugDraw method
// ---------------------------------------------------------------------------
struct ShapeDebugRenderer {

    void draw(const CollisionShape& shape, DebugDraw& dd,
              const RenderColor& color) const
    {
        switch (shape.type) {
        case ShapeType::Sphere:
            dd.draw_sphere_wireframe(shape.position, shape.sphere_radius, color);
            break;

        case ShapeType::Box:
            dd.draw_obb_wireframe(shape.position, shape.box_half_extents,
                                  shape.orientation, color);
            break;

        case ShapeType::Capsule:
            dd.draw_capsule_wireframe(shape.position, shape.capsule_radius,
                                      shape.capsule_half_height,
                                      shape.orientation, color);
            break;

        case ShapeType::Cylinder:
            dd.draw_cylinder_wireframe(shape.position, shape.cylinder_radius,
                                       shape.cylinder_half_height,
                                       shape.orientation, color);
            break;

        case ShapeType::Plane: {
            // Draw a grid on the plane
            Vec3 grid_normal = shape.plane_normal;
            dd.draw_plane_grid(shape.position, grid_normal, 5.0f, 10u, color);
            break;
        }

        case ShapeType::ConvexPiece: {
            // Approximate with AABB for debug
            if (shape.convex_vertex_count > 0u) {
                Vec3 vmin(1e30f, 1e30f, 1e30f);
                Vec3 vmax(-1e30f, -1e30f, -1e30f);
                for (uint32_t i = 0u; i < shape.convex_vertex_count; ++i) {
                    const Vec3& v = shape.convex_vertices[i];
                    if (v.x < vmin.x) vmin.x = v.x;
                    if (v.y < vmin.y) vmin.y = v.y;
                    if (v.z < vmin.z) vmin.z = v.z;
                    if (v.x > vmax.x) vmax.x = v.x;
                    if (v.y > vmax.y) vmax.y = v.y;
                    if (v.z > vmax.z) vmax.z = v.z;
                }
                // Center the AABB and transform
                Vec3 center = Vec3::scale(Vec3::add(vmin, vmax), 0.5f);
                Vec3 half_ext = Vec3::scale(Vec3::sub(vmax, vmin), 0.5f);
                dd.draw_obb_wireframe(
                    Vec3::add(shape.position, shape.rotation.transform_vec(center)),
                    half_ext, shape.orientation, color);
            }
            break;
        }

        default:
            break;
        }
    }
};

} // namespace apc
