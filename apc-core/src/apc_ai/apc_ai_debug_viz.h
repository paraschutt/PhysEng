#pragma once
// =============================================================================
// apc_ai_debug_viz.h — AI Debug Visualization System
// =============================================================================
//
// Provides visual debugging overlays for the AI system:
//
//   - AIDebugLayer: bitmask flags for toggling individual debug layers
//   - AIDebugConfig: visual tuning parameters (colors, scales, labels)
//   - UtilityDebugEntry: per-entity utility score snapshot
//   - SteeringDebugEntry: per-entity steering state snapshot
//   - FormationDebugEntry: per-entity formation state snapshot
//   - AIDebugVisualizer: dispatches debug drawing for all layers
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation (fixed-size arrays)
//   - Deterministic
//   - C++17
//
// =============================================================================

#include "apc_render/apc_debug_draw.h"
#include "apc_render/apc_render_types.h"
#include "apc_ai/apc_ai_steering.h"
#include "apc_ai/apc_ai_decision.h"
#include "apc_ai/apc_ai_formation.h"
#include "apc_entity/apc_entity_types.h"
#include "apc_entity/apc_entity_manager.h"
#include "apc_input/apc_input_types.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_math_common.h"
#include <cstdint>
#include <cmath>

namespace apc {

// =============================================================================
// AIDebugLayer — Bitmask flags for toggling AI debug visualization layers
// =============================================================================
enum class AIDebugLayer : uint16_t {
    STEERING_FORCES   = 1u << 0u,
    UTILITY_SCORES     = 1u << 1u,
    FORMATION_POSITIONS = 1u << 2u,
    MOTOR_INTENT       = 1u << 3u,
    DECISION_TREE      = 1u << 4u,
    PATH_PLANNING      = 1u << 5u,
    FIELD_CONTROL      = 1u << 6u,
    ALL                = 0xFFFFu
};

// =============================================================================
// AIDebugConfig — Visual tuning parameters for AI debug rendering
// =============================================================================
struct AIDebugConfig {
    uint16_t enabled_layers        = 0u;
    float    visualization_scale   = 1.0f;
    uint8_t  show_player_labels    = 0;
    uint8_t  show_utility_values   = 0;
    uint8_t  show_formation_grid   = 0;

    // --- Per-layer colors ---
    RenderColor steering_color  = RenderColor::GREEN();
    RenderColor utility_color    = RenderColor::YELLOW();
    RenderColor formation_color  = RenderColor(0.2f, 0.6f, 0.2f, 0.6f);
    RenderColor intent_color     = RenderColor(1.0f, 0.8f, 0.0f, 0.8f);
};

// =============================================================================
// UtilityDebugEntry — Per-entity utility score snapshot
// =============================================================================
struct UtilityDebugEntry {
    EntityId   entity_id;
    AIActionType chosen_action  = static_cast<AIActionType>(0);
    float      action_scores[8]  = {}; // Top 8 action scores
    uint8_t    action_types[8]   = {}; // Corresponding action types
    float      confidence        = 0.0f;
    Vec3       position           = { 0.0f, 0.0f, 0.0f };
};

// =============================================================================
// SteeringDebugEntry — Per-entity steering state snapshot
// =============================================================================
struct SteeringDebugEntry {
    EntityId   entity_id;
    Vec3       position            = { 0.0f, 0.0f, 0.0f };
    Vec3       steering_force      = { 0.0f, 0.0f, 0.0f };
    Vec3       seek_target         = { 0.0f, 0.0f, 0.0f };
    Vec3       flee_threat         = { 0.0f, 0.0f, 0.0f };
    uint8_t    active_behaviors[8] = {}; // Which behaviors are active
    float      behavior_weights[8] = {};
    uint8_t    behavior_count      = 0;
};

// =============================================================================
// FormationDebugEntry — Per-entity formation state snapshot
// =============================================================================
struct FormationDebugEntry {
    EntityId entity_id;
    Vec3     actual_position          = { 0.0f, 0.0f, 0.0f };
    Vec3     formation_position       = { 0.0f, 0.0f, 0.0f };
    Vec3     ball_influenced_position = { 0.0f, 0.0f, 0.0f };
    uint8_t  formation_type           = 0; // FormationType value
};

// =============================================================================
// AIDebugVisualizer — Dispatches AI debug drawing to the debug draw system
// =============================================================================
struct AIDebugVisualizer {
    AIDebugConfig config;

    // --- Debug data buffers ---
    UtilityDebugEntry  utility_entries[MAX_ATHLETES];
    uint32_t           utility_count   = 0u;

    SteeringDebugEntry steering_entries[MAX_ATHLETES];
    uint32_t           steering_count  = 0u;

    FormationDebugEntry formation_entries[MAX_ATHLETES];
    uint32_t           formation_count = 0u;

    // =========================================================================
    // Layer management
    // =========================================================================

    APC_FORCEINLINE void set_layer(AIDebugLayer layer, uint8_t enabled)
    {
        if (enabled) {
            config.enabled_layers |= static_cast<uint16_t>(layer);
        } else {
            config.enabled_layers &= ~static_cast<uint16_t>(layer);
        }
    }

    APC_FORCEINLINE uint8_t is_layer_enabled(AIDebugLayer layer) const
    {
        return (config.enabled_layers & static_cast<uint16_t>(layer)) != 0u ? 1u : 0u;
    }

    // =========================================================================
    // Frame lifecycle
    // =========================================================================

    // --- begin_frame: clear all entries ---
    void begin_frame()
    {
        utility_count   = 0u;
        steering_count  = 0u;
        formation_count = 0u;
    }

    // --- add_utility_entry: record utility debug data ---
    void add_utility_entry(const UtilityDebugEntry& entry)
    {
        if (utility_count < MAX_ATHLETES) {
            utility_entries[utility_count++] = entry;
        }
    }

    // --- add_steering_entry: record steering debug data ---
    void add_steering_entry(const SteeringDebugEntry& entry)
    {
        if (steering_count < MAX_ATHLETES) {
            steering_entries[steering_count++] = entry;
        }
    }

    // --- add_formation_entry: record formation debug data ---
    void add_formation_entry(const FormationDebugEntry& entry)
    {
        if (formation_count < MAX_ATHLETES) {
            formation_entries[formation_count++] = entry;
        }
    }

    // =========================================================================
    // render — Master draw dispatch
    // =========================================================================
    void render(DebugDrawList& draw_list) const
    {
        uint16_t active = config.enabled_layers;
        if (active == 0u) return;

        // --- STEERING_FORCES layer: arrows for steering forces ---
        if (is_layer_enabled(AIDebugLayer::STEERING_FORCES)) {
            for (uint32_t i = 0u; i < steering_count; ++i) {
                const SteeringDebugEntry& e = steering_entries[i];
                float force_len = Vec3::length(e.steering_force);
                if (force_len < APC_EPSILON) continue;

                float scale = config.visualization_scale;
                Vec3 tip = Vec3::add(e.position,
                    Vec3::scale(e.steering_force, scale));

                draw_list.add_line(e.position, tip, config.steering_color);

                // Arrowhead
                Vec3 dir = Vec3::scale(e.steering_force, 1.0f / force_len);
                float head_len = force_len * scale * 0.2f;
                if (head_len < 0.01f) head_len = 0.01f;
                Vec3 head_base = Vec3::sub(tip, Vec3::scale(dir, head_len));

                Vec3 perp;
                if (std::abs(dir.y) < 0.99f) {
                    perp = Vec3::normalize(Vec3::cross(dir, Vec3(0.0f, 1.0f, 0.0f)));
                } else {
                    perp = Vec3::normalize(Vec3::cross(dir, Vec3(1.0f, 0.0f, 0.0f)));
                }
                float hw = head_len * 0.4f;
                draw_list.add_line(tip, Vec3::add(head_base, Vec3::scale(perp, hw)),
                                  config.steering_color);
                draw_list.add_line(tip, Vec3::sub(head_base, Vec3::scale(perp, hw)),
                                  config.steering_color);

                // Seek target marker (blue point)
                if (Vec3::length(e.seek_target) > APC_EPSILON) {
                    draw_list.add_point(e.seek_target,
                        RenderColor::BLUE(), 4.0f);
                }
            }
        }

        // --- UTILITY_SCORES layer: points + bars for utility scores ---
        if (is_layer_enabled(AIDebugLayer::UTILITY_SCORES)) {
            for (uint32_t i = 0u; i < utility_count; ++i) {
                const UtilityDebugEntry& e = utility_entries[i];
                float score = e.confidence;
                if (score < APC_EPSILON) continue;

                // Clamp score
                if (score > 1.0f) score = 1.0f;

                // Bar above entity head
                float bar_base_y = e.position.y + 2.0f;
                float bar_top_y  = bar_base_y + 2.0f * score;

                // Color: green (low) -> yellow (mid) -> red (high)
                RenderColor bar_color;
                if (score < 0.5f) {
                    bar_color = RenderColor(2.0f * score, 1.0f, 0.0f, 0.8f);
                } else {
                    bar_color = RenderColor(1.0f, 2.0f * (1.0f - score), 0.0f, 0.8f);
                }

                // Vertical bar
                float hw = 0.15f;
                Vec3 bl(e.position.x - hw, bar_base_y, e.position.z);
                Vec3 br(e.position.x + hw, bar_base_y, e.position.z);
                Vec3 tl(e.position.x - hw, bar_top_y, e.position.z);
                Vec3 tr(e.position.x + hw, bar_top_y, e.position.z);

                draw_list.add_line(bl, tl, bar_color);
                draw_list.add_line(br, tr, bar_color);
                draw_list.add_line(tl, tr, bar_color);

                // Background bar outline (gray)
                RenderColor bg_color(0.3f, 0.3f, 0.3f, 0.5f);
                Vec3 bg_top(e.position.x - hw, bar_base_y + 2.0f, e.position.z);
                Vec3 bg_top_r(e.position.x + hw, bar_base_y + 2.0f, e.position.z);
                draw_list.add_line(bl, bg_top, bg_color);
                draw_list.add_line(br, bg_top_r, bg_color);
                draw_list.add_line(bg_top, bg_top_r, bg_color);
            }
        }

        // --- FORMATION_POSITIONS layer: circles at formation, lines to actual ---
        if (is_layer_enabled(AIDebugLayer::FORMATION_POSITIONS)) {
            for (uint32_t i = 0u; i < formation_count; ++i) {
                const FormationDebugEntry& e = formation_entries[i];

                // Formation position marker (green point)
                Vec3 form_pos = Vec3::add(e.formation_position, Vec3(0.0f, 0.1f, 0.0f));
                draw_list.add_point(form_pos, config.formation_color, 4.0f);

                // Ball influenced position (teal point)
                Vec3 ball_pos = Vec3::add(e.ball_influenced_position, Vec3(0.0f, 0.15f, 0.0f));
                draw_list.add_point(ball_pos, RenderColor(0.0f, 0.8f, 0.8f, 0.6f), 3.0f);

                // Line from formation to actual position
                draw_list.add_line(form_pos, e.actual_position, config.formation_color);
            }
        }

        // --- MOTOR_INTENT layer: direction arrows ---
        if (is_layer_enabled(AIDebugLayer::MOTOR_INTENT)) {
            // Draw from entity data (accessed via EntityManager externally)
            // For now, draw from steering entries as proxy
            for (uint32_t i = 0u; i < steering_count; ++i) {
                const SteeringDebugEntry& e = steering_entries[i];
                float force_len = Vec3::length(e.steering_force);
                if (force_len < APC_EPSILON) continue;

                Vec3 dir = Vec3::scale(e.steering_force, 1.0f / force_len);
                float scale = config.visualization_scale * 0.7f;
                Vec3 tip = Vec3::add(e.position, Vec3::scale(dir, scale));

                draw_list.add_line(e.position, tip, config.intent_color);
            }
        }

        // --- DECISION_TREE layer: placeholder ---
        if (is_layer_enabled(AIDebugLayer::DECISION_TREE)) {
            // Decision tree visualization requires external tree data
        }

        // --- PATH_PLANNING layer: placeholder ---
        if (is_layer_enabled(AIDebugLayer::PATH_PLANNING)) {
            // Path planning visualization requires external path data
        }

        // --- FIELD_CONTROL layer: placeholder ---
        if (is_layer_enabled(AIDebugLayer::FIELD_CONTROL)) {
            // Field control heatmap requires external spatial data
        }
    }
};

} // namespace apc
