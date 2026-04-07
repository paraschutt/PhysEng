#pragma once
// =============================================================================
// Render Bridge — Physics-to-render sync layer
// =============================================================================
//
// Provides:
//   - RenderBridgeConfig: debug visualization and camera settings
//   - SyncPhase: sync timing enumeration
//   - RenderBridge: central sync point between physics and rendering
//
// Responsibilities:
//   - Sync rigid body transforms for rendering
//   - Generate debug draw calls from contacts
//   - Bridge game hook outputs for VFX
//   - Manage camera state
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation (fixed-capacity buffers)
//   - C++17
//
// =============================================================================

#include "apc_render_types.h"
#include "apc_render_camera.h"
#include "apc_debug_draw.h"
#include "apc_solver/apc_rigid_body.h"
#include "apc_collision/apc_collision_dispatch.h"
#include "apc_style/apc_game_hooks.h"
#include "apc_sport/apc_ball_physics.h"
#include "apc_sport/apc_sport_field.h"
#include "apc_sport/apc_ball_control.h"
#include <cstdint>

namespace apc {

// ---------------------------------------------------------------------------
// RenderBridgeConfig
// ---------------------------------------------------------------------------
struct RenderBridgeConfig {
    uint16_t   debug_draw_flags  = 0u;
    CameraMode camera_mode       = CameraMode::PERSPECTIVE;
    bool       show_hud           = false;
    bool       show_debug_overlay = false;
    bool       show_contact_viz   = true;
    bool       show_skeleton_viz  = false;
    bool       wireframe_overlay  = false;
    float      debug_line_width   = 1.0f;
};

// ---------------------------------------------------------------------------
// SyncPhase — When sync happens relative to physics/render
// ---------------------------------------------------------------------------
enum class SyncPhase : uint8_t {
    PRE_PHYSICS   = 0,
    POST_PHYSICS  = 1,
    PRE_RENDER    = 2
};

// ---------------------------------------------------------------------------
// SyncedBodyTransform — Stored body transform for rendering
// ---------------------------------------------------------------------------
struct SyncedBodyTransform {
    Vec3 position = Vec3(0.0f, 0.0f, 0.0f);
    Quat orientation = Quat::identity();
};

// ---------------------------------------------------------------------------
// RenderBridge — Central physics-to-render sync point
// ---------------------------------------------------------------------------
struct RenderBridge {
    static constexpr uint32_t MAX_SYNCED_BODIES    = 256u;
    static constexpr uint32_t MAX_HOOK_OUTPUTS     = 64u;

    RenderBridgeConfig  config;
    DebugDraw           debug_draw;
    RenderStats         last_stats;
    CameraController    camera;
    SyncPhase           sync_phase     = SyncPhase::POST_PHYSICS;
    uint32_t            frame_count    = 0u;

    // Internal buffers
    SyncedBodyTransform synced_bodies[MAX_SYNCED_BODIES];
    uint32_t            synced_body_count = 0u;

    GameHookOutput      hook_outputs[MAX_HOOK_OUTPUTS];
    uint32_t            hook_output_count = 0u;

    // --- Configuration ---
    void set_config(const RenderBridgeConfig& cfg) {
        config = cfg;
    }

    // --- Accessors ---
    DebugDraw& get_debug_draw() { return debug_draw; }
    CameraController& get_camera() { return camera; }
    const RenderStats& get_stats() const { return last_stats; }

    // --- Sync transforms from rigid bodies ---
    void sync_transforms(const RigidBody* bodies, uint32_t count) {
        synced_body_count = 0u;
        for (uint32_t i = 0u; i < count && i < MAX_SYNCED_BODIES; ++i) {
            synced_bodies[synced_body_count].position    = bodies[i].position;
            synced_bodies[synced_body_count].orientation = bodies[i].orientation;
            ++synced_body_count;
        }
    }

    // --- Sync contacts into debug draw ---
    void sync_contacts(const ContactPoint* contacts, uint32_t count) {
        if (!config.show_contact_viz) return;

        RenderColor contact_color = RenderColor::YELLOW();
        RenderColor normal_color  = RenderColor::CYAN();

        for (uint32_t i = 0u; i < count; ++i) {
            const ContactPoint& cp = contacts[i];
            debug_draw.draw_contact_point(cp.point_on_b, contact_color);
            debug_draw.draw_contact_normal(cp.point_on_b, cp.normal,
                                          normal_color, 0.5f);
        }
    }

    // --- Sync game hook outputs ---
    void sync_game_hooks(const GameHookOutput* outputs, uint32_t count) {
        hook_output_count = 0u;
        for (uint32_t i = 0u; i < count && i < MAX_HOOK_OUTPUTS; ++i) {
            hook_outputs[hook_output_count] = outputs[i];
            ++hook_output_count;
        }
    }

    // --- Sync sport state (stub for now) ---
    void sync_sport_state(const BallState* balls, uint32_t ball_count,
                          const SportField* field,
                          const PossessionRecord* possessions,
                          uint32_t poss_count) {
        // Minimal stub — future implementation will sync ball transforms
        // and field state for the render pipeline
        (void)balls;
        (void)ball_count;
        (void)field;
        (void)possessions;
        (void)poss_count;
    }

    // --- Frame lifecycle ---
    void begin_frame() {
        debug_draw.clear();
        ++frame_count;
    }

    void end_frame() {
        // Flush debug draw
        debug_draw.flush();
    }

    void reset() {
        debug_draw.clear();
        synced_body_count = 0u;
        hook_output_count = 0u;
        frame_count = 0u;
        last_stats.reset();
    }
};

} // namespace apc
