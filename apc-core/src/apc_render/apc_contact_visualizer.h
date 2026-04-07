#pragma once
// =============================================================================
// Sprint 18: Contact Visualizer — Contact point rendering, impulse heatmaps
// =============================================================================
//
// Provides:
//   - ContactVisualizerConfig: visual parameters for contact rendering
//   - ContactDrawCommand: preprocessed draw command per contact point
//   - ImpulseHeatmap: green→yellow→red color ramp based on impulse magnitude
//   - ContactVisualizer: processes ContactManager into draw commands, renders
//     contacts, normals, penetration, friction
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation (fixed-capacity arrays)
//   - C++17
//
// =============================================================================

#include "apc_render_types.h"
#include "apc_debug_draw.h"
#include "apc_solver/apc_contact_manager.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_math_common.h"
#include <cstdint>
#include <cmath>

namespace apc {

// =============================================================================
// ContactVisualizerConfig — Visual parameters for contact rendering
// =============================================================================
struct ContactVisualizerConfig {
    bool show_contacts = true;
    bool show_normals = true;
    bool show_penetration = true;
    bool show_impulse_heatmap = false;
    bool show_friction = false;
    uint32_t contact_age_threshold = 100u;         // Contacts older than this are hidden
    float impulse_color_scale = 100.0f;            // Impulse that maps to full red
    float normal_arrow_length = 0.2f;
    float penetration_depth_scale = 1.0f;          // Visual scale for penetration indicators
    float contact_point_size = 0.05f;

    /// Factory returning reasonable defaults.
    static ContactVisualizerConfig make_default() {
        ContactVisualizerConfig cfg;
        cfg.show_contacts = true;
        cfg.show_normals = true;
        cfg.show_penetration = true;
        cfg.show_impulse_heatmap = false;
        cfg.show_friction = false;
        cfg.contact_age_threshold = 100u;
        cfg.impulse_color_scale = 100.0f;
        cfg.normal_arrow_length = 0.2f;
        cfg.penetration_depth_scale = 1.0f;
        cfg.contact_point_size = 0.05f;
        return cfg;
    }
};

// =============================================================================
// ContactDrawCommand — Preprocessed draw command per contact point
// =============================================================================
struct ContactDrawCommand {
    Vec3 point_a;                     // Contact point on body A
    Vec3 point_b;                     // Contact point on body B
    Vec3 normal;                      // Contact normal
    float penetration = 0.0f;
    float normal_impulse = 0.0f;      // Accumulated normal impulse
    Vec3 friction_impulse;            // Accumulated friction impulse
    uint32_t age = 0u;
    uint32_t body_a = 0u;
    uint32_t body_b = 0u;
    RenderColor color = RenderColor::WHITE();
};

// =============================================================================
// ImpulseHeatmap — Green→Yellow→Red color ramp based on impulse magnitude
// =============================================================================
struct ImpulseHeatmap {
    float scale = 100.0f;                             // Impulse that maps to max (red)
    RenderColor low_color = RenderColor::GREEN();     // Zero impulse color
    RenderColor mid_color = RenderColor::YELLOW();    // Half-scale color
    RenderColor high_color = RenderColor::RED();      // Full-scale color

    /// Evaluate heatmap color for a given impulse magnitude.
    /// t = clamp(impulse / scale, 0, 1)
    /// if t < 0.5: lerp(low, mid, t * 2)
    /// else:        lerp(mid, high, (t - 0.5) * 2)
    RenderColor evaluate(float impulse) const {
        float t = impulse / scale;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;

        if (t < 0.5f) {
            float s = t * 2.0f;
            return RenderColor(
                low_color.r + (mid_color.r - low_color.r) * s,
                low_color.g + (mid_color.g - low_color.g) * s,
                low_color.b + (mid_color.b - low_color.b) * s,
                low_color.a + (mid_color.a - low_color.a) * s
            );
        } else {
            float s = (t - 0.5f) * 2.0f;
            return RenderColor(
                mid_color.r + (high_color.r - mid_color.r) * s,
                mid_color.g + (high_color.g - mid_color.g) * s,
                mid_color.b + (high_color.b - mid_color.b) * s,
                mid_color.a + (high_color.a - mid_color.a) * s
            );
        }
    }
};

// =============================================================================
// ContactVisualizer — Processes ContactManager, renders contact visualization
// =============================================================================
struct ContactVisualizer {
    static constexpr uint32_t MAX_CONTACTS = 512u;

    ContactVisualizerConfig config;
    ImpulseHeatmap heatmap;
    ContactDrawCommand commands[MAX_CONTACTS];
    uint32_t command_count = 0u;

    /// Set visual configuration.
    void set_config(const ContactVisualizerConfig& cfg) {
        config = cfg;
    }

    /// Clear all draw commands.
    void clear() {
        command_count = 0u;
    }

    /// Process contacts from a ContactManager into draw commands.
    /// Skips contacts older than contact_age_threshold.
    /// Applies alpha fade based on age.
    void process_contacts(const ContactManager& cm) {
        clear();
        uint32_t pairs = cm.pair_count();
        const PersistentManifold* manifolds = cm.manifolds();

        for (uint32_t p = 0u; p < pairs; ++p) {
            const PersistentManifold& pm = manifolds[p];
            for (uint32_t c = 0u; c < pm.contact_count; ++c) {
                const PersistentContact& pc = pm.contacts[c];

                // Skip contacts older than threshold
                if (pc.age > config.contact_age_threshold) continue;

                // Check capacity
                if (command_count >= MAX_CONTACTS) return;

                ContactDrawCommand& cmd = commands[command_count];
                cmd.point_a = pc.contact.point_on_a;
                cmd.point_b = pc.contact.point_on_b;
                cmd.normal = pc.contact.normal;
                cmd.penetration = pc.contact.penetration;
                cmd.normal_impulse = pc.accumulated_normal_impulse;
                cmd.friction_impulse = pc.accumulated_friction_impulse;
                cmd.age = pc.age;
                cmd.body_a = pm.id_a;
                cmd.body_b = pm.id_b;

                // Compute color
                if (config.show_impulse_heatmap) {
                    cmd.color = heatmap.evaluate(cmd.normal_impulse);
                } else {
                    cmd.color = RenderColor::WHITE();
                }

                // Apply age-based alpha fade
                float age_ratio = static_cast<float>(cmd.age) /
                                  static_cast<float>(config.contact_age_threshold);
                float alpha = 1.0f - age_ratio;
                if (alpha < 0.0f) alpha = 0.0f;
                cmd.color.a = alpha;

                command_count++;
            }
        }
    }

    /// Draw contact points (X markers) at point_a for each command.
    void draw_contact_points(DebugDraw& dd) {
        for (uint32_t i = 0u; i < command_count; ++i) {
            dd.draw_contact_point(commands[i].point_a, commands[i].color);
        }
    }

    /// Draw normal arrows from point_a along the contact normal.
    void draw_normal_arrows(DebugDraw& dd) {
        for (uint32_t i = 0u; i < command_count; ++i) {
            dd.draw_contact_normal(commands[i].point_a, commands[i].normal,
                                  commands[i].color, config.normal_arrow_length);
        }
    }

    /// Draw penetration indicators: lines from point_a along -normal.
    void draw_penetration_indicators(DebugDraw& dd) {
        RenderColor pen_color = RenderColor::RED();
        for (uint32_t i = 0u; i < command_count; ++i) {
            if (commands[i].penetration <= 0.0f) continue;
            float len = commands[i].penetration * config.penetration_depth_scale;
            Vec3 end = Vec3::add(commands[i].point_a,
                Vec3::scale(commands[i].normal, -len));
            dd.list.add_line(commands[i].point_a, end, pen_color);
        }
    }

    /// Draw impulse heatmap: colored points at point_a by impulse magnitude.
    void draw_impulse_heatmap(DebugDraw& dd) {
        for (uint32_t i = 0u; i < command_count; ++i) {
            RenderColor hc = heatmap.evaluate(commands[i].normal_impulse);
            // Size proportional to impulse, clamped to [0.01, 0.2]
            float imp_size = commands[i].normal_impulse * 0.001f;
            if (imp_size < 0.01f) imp_size = 0.01f;
            if (imp_size > 0.2f) imp_size = 0.2f;
            dd.list.add_point(commands[i].point_a, hc, imp_size * 100.0f);
        }
    }

    /// Draw friction vectors: arrows along friction impulse direction.
    void draw_friction_vectors(DebugDraw& dd) {
        RenderColor fric_color = RenderColor::YELLOW();
        float fric_scale = 0.001f;
        for (uint32_t i = 0u; i < command_count; ++i) {
            float flen = Vec3::length(commands[i].friction_impulse);
            if (flen < APC_EPSILON) continue;
            Vec3 end = Vec3::add(commands[i].point_a,
                Vec3::scale(commands[i].friction_impulse, fric_scale));
            dd.list.add_line(commands[i].point_a, end, fric_color);
        }
    }

    /// Get the number of processed contact commands.
    uint32_t get_contact_count() const { return command_count; }

    /// Get the command array.
    const ContactDrawCommand* get_commands() const { return commands; }
};

} // namespace apc
