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

        // --- STEERING_FORCES layer: directional arrows only ---
        if (is_layer_enabled(AIDebugLayer::STEERING_FORCES)) {
            for (uint32_t i = 0u; i < steering_count; ++i) {
                const SteeringDebugEntry& e = steering_entries[i];
                float force_len = Vec3::length(e.steering_force);
                if (force_len < APC_EPSILON) continue;

                Vec3 dir = Vec3::scale(e.steering_force, 1.0f / force_len);
                float arrow_len = 2.0f * config.visualization_scale;
                Vec3 tip = Vec3::add(e.position, Vec3::scale(dir, arrow_len));

                draw_list.add_line(e.position, tip, config.steering_color);

                // Arrowhead
                float head_len = arrow_len * 0.25f;
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
            }
        }

        // --- UTILITY_SCORES layer: disabled (not needed for current phase) ---
        if (is_layer_enabled(AIDebugLayer::UTILITY_SCORES)) {
            // Utility score bars removed for visual clarity
        }

        // --- FORMATION_POSITIONS layer: disabled (trails/clutter removed) ---
        if (is_layer_enabled(AIDebugLayer::FORMATION_POSITIONS)) {
            // Formation lines and markers removed for visual clarity
        }

        // --- MOTOR_INTENT layer: combined with STEERING_FORCES above ---
        if (is_layer_enabled(AIDebugLayer::MOTOR_INTENT)) {
            // Motor intent arrows merged into STEERING_FORCES layer
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
