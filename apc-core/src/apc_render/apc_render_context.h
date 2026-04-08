#pragma once
// =============================================================================
// Render Context — Abstract render backend interface
// =============================================================================
//
// Provides the RenderContext struct: a virtual interface that concrete render
// backends (e.g., OpenGL, Vulkan) implement. All draw calls and state changes
// go through this interface.
//
// Design:
//   - Struct with virtual methods (abstract interface)
//   - apc:: namespace
//   - No dynamic allocation in the interface itself
//   - C++17
//
// =============================================================================

#include "apc_render_types.h"
#include <cstdint>

namespace apc {

// ---------------------------------------------------------------------------
// RenderContext — Abstract render backend interface
// ---------------------------------------------------------------------------
struct RenderContext {
    virtual ~RenderContext() = default;

    // Frame lifecycle
    virtual bool begin_frame() = 0;    // Returns false if should skip frame
    virtual void end_frame() = 0;
    virtual void submit() = 0;

    // Draw calls
    virtual void draw_lines(const RenderVertex* vertices,
                            uint32_t vertex_count) = 0;
    virtual void draw_triangles(const RenderVertex* vertices,
                                uint32_t vertex_count,
                                const uint32_t* indices,
                                uint32_t index_count) = 0;
    virtual void draw_points(const RenderVertex* vertices,
                             uint32_t vertex_count) = 0;

    // State management
    virtual void set_viewport(uint32_t x, uint32_t y,
                              uint32_t w, uint32_t h) = 0;
    virtual void set_view(const RenderViewUniforms& uniforms) = 0;
    virtual void push_render_pass(RenderPassType type) = 0;
    virtual void set_material(const RenderMaterial& material) = 0;
    virtual void clear(float r, float g, float b, float a) = 0;

    // Queries
    virtual RenderStats get_stats() const = 0;
    virtual const char* get_backend_name() const = 0;
};

} // namespace apc
