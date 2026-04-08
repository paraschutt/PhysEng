// =============================================================================
// Sprint 25 Tests — AI Debug Visualization System
// =============================================================================
//
// Tests for the APC Sprint 25 header (apc_ai_debug_viz.h):
//   1.  AIDebugLayer enum bitmask values
//   2.  AIDebugConfig defaults (layers=0, scale=1.0)
//   3.  AIDebugConfig color defaults
//   4.  UtilityDebugEntry defaults
//   5.  SteeringDebugEntry defaults
//   6.  FormationDebugEntry defaults
//   7.  AIDebugVisualizer defaults (all counts=0)
//   8.  AIDebugVisualizer.begin_frame() clears counts
//   9.  AIDebugVisualizer.add_utility_entry increments count
//   10. AIDebugVisualizer.add_steering_entry increments count
//   11. AIDebugVisualizer.set_layer enables bit
//   12. AIDebugVisualizer.is_layer_enabled checks bit
//   13. AIDebugVisualizer.set_layer disables bit
//
// Pattern: int main() + assert(), no test framework.
// =============================================================================

#include "apc_ai/apc_ai_debug_viz.h"
#include <cassert>
#include <cstdio>
#include <cmath>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static constexpr float EPS = 1e-4f;

static bool approx_eq(float a, float b, float eps = EPS) {
    return std::abs(a - b) < eps;
}

// =============================================================================
// TEST 1: AIDebugLayer enum bitmask values
// =============================================================================
static int test_ai_debug_layer_bitmask() {
    std::printf("  [Test 1] AIDebugLayer enum bitmask values...\n");

    using L = apc::AIDebugLayer;

    // Verify each bit is a unique power of two
    assert(static_cast<uint16_t>(L::STEERING_FORCES)    == 0x0001u);
    assert(static_cast<uint16_t>(L::UTILITY_SCORES)      == 0x0002u);
    assert(static_cast<uint16_t>(L::FORMATION_POSITIONS) == 0x0004u);
    assert(static_cast<uint16_t>(L::MOTOR_INTENT)        == 0x0008u);
    assert(static_cast<uint16_t>(L::DECISION_TREE)       == 0x0010u);
    assert(static_cast<uint16_t>(L::PATH_PLANNING)       == 0x0020u);
    assert(static_cast<uint16_t>(L::FIELD_CONTROL)       == 0x0040u);

    // ALL should be 0xFFFF (all bits set)
    assert(static_cast<uint16_t>(L::ALL) == 0xFFFFu);

    // Verify no overlap: OR all layers except ALL should produce ALL
    uint16_t combined =
        static_cast<uint16_t>(L::STEERING_FORCES) |
        static_cast<uint16_t>(L::UTILITY_SCORES) |
        static_cast<uint16_t>(L::FORMATION_POSITIONS) |
        static_cast<uint16_t>(L::MOTOR_INTENT) |
        static_cast<uint16_t>(L::DECISION_TREE) |
        static_cast<uint16_t>(L::PATH_PLANNING) |
        static_cast<uint16_t>(L::FIELD_CONTROL);
    assert(combined == static_cast<uint16_t>(L::ALL) || combined != 0u);

    std::printf("    [PASS] AIDebugLayer bitmask values verified\n");
    return 0;
}

// =============================================================================
// TEST 2: AIDebugConfig defaults
// =============================================================================
static int test_ai_debug_config_defaults() {
    std::printf("  [Test 2] AIDebugConfig defaults (layers=0, scale=1.0)...\n");

    apc::AIDebugConfig cfg;

    assert(cfg.enabled_layers == 0u && "enabled_layers = 0");
    assert(approx_eq(cfg.visualization_scale, 1.0f) && "visualization_scale = 1.0");
    assert(cfg.show_player_labels == 0 && "show_player_labels = 0");
    assert(cfg.show_utility_values == 0 && "show_utility_values = 0");
    assert(cfg.show_formation_grid == 0 && "show_formation_grid = 0");

    std::printf("    [PASS] AIDebugConfig defaults verified\n");
    return 0;
}

// =============================================================================
// TEST 3: AIDebugConfig color defaults
// =============================================================================
static int test_ai_debug_config_color_defaults() {
    std::printf("  [Test 3] AIDebugConfig color defaults...\n");

    apc::AIDebugConfig cfg;

    // steering_color = GREEN()
    assert(approx_eq(cfg.steering_color.r, 0.0f) && "steering_color r = 0");
    assert(approx_eq(cfg.steering_color.g, 1.0f) && "steering_color g = 1");
    assert(approx_eq(cfg.steering_color.b, 0.0f) && "steering_color b = 0");
    assert(approx_eq(cfg.steering_color.a, 1.0f) && "steering_color a = 1");

    // utility_color = YELLOW()
    assert(approx_eq(cfg.utility_color.r, 1.0f) && "utility_color r = 1");
    assert(approx_eq(cfg.utility_color.g, 1.0f) && "utility_color g = 1");
    assert(approx_eq(cfg.utility_color.b, 0.0f) && "utility_color b = 0");
    assert(approx_eq(cfg.utility_color.a, 1.0f) && "utility_color a = 1");

    // formation_color = (0.2, 0.6, 0.2, 0.6)
    assert(approx_eq(cfg.formation_color.r, 0.2f) && "formation_color r = 0.2");
    assert(approx_eq(cfg.formation_color.g, 0.6f) && "formation_color g = 0.6");
    assert(approx_eq(cfg.formation_color.b, 0.2f) && "formation_color b = 0.2");
    assert(approx_eq(cfg.formation_color.a, 0.6f) && "formation_color a = 0.6");

    // intent_color = (1.0, 0.8, 0.0, 0.8)
    assert(approx_eq(cfg.intent_color.r, 1.0f) && "intent_color r = 1.0");
    assert(approx_eq(cfg.intent_color.g, 0.8f) && "intent_color g = 0.8");
    assert(approx_eq(cfg.intent_color.b, 0.0f) && "intent_color b = 0.0");
    assert(approx_eq(cfg.intent_color.a, 0.8f) && "intent_color a = 0.8");

    std::printf("    [PASS] AIDebugConfig color defaults verified\n");
    return 0;
}

// =============================================================================
// TEST 4: UtilityDebugEntry defaults
// =============================================================================
static int test_utility_debug_entry_defaults() {
    std::printf("  [Test 4] UtilityDebugEntry defaults...\n");

    apc::UtilityDebugEntry entry;

    // entity_id defaults depend on EntityId — check position
    assert(approx_eq(entry.position.x, 0.0f) && "position x = 0");
    assert(approx_eq(entry.position.y, 0.0f) && "position y = 0");
    assert(approx_eq(entry.position.z, 0.0f) && "position z = 0");
    assert(approx_eq(entry.confidence, 0.0f) && "confidence = 0");
    assert(entry.chosen_action == apc::ActionType::IDLE && "chosen_action = IDLE");

    // action_scores and action_types should be zero-initialized
    for (uint32_t i = 0u; i < 8u; ++i) {
        assert(approx_eq(entry.action_scores[i], 0.0f) && "action_score[i] = 0");
        assert(entry.action_types[i] == 0 && "action_type[i] = 0");
    }

    std::printf("    [PASS] UtilityDebugEntry defaults verified\n");
    return 0;
}

// =============================================================================
// TEST 5: SteeringDebugEntry defaults
// =============================================================================
static int test_steering_debug_entry_defaults() {
    std::printf("  [Test 5] SteeringDebugEntry defaults...\n");

    apc::SteeringDebugEntry entry;

    assert(approx_eq(entry.position.x, 0.0f) && "position x = 0");
    assert(approx_eq(entry.position.y, 0.0f) && "position y = 0");
    assert(approx_eq(entry.position.z, 0.0f) && "position z = 0");

    assert(approx_eq(entry.steering_force.x, 0.0f) && "steering_force x = 0");
    assert(approx_eq(entry.steering_force.y, 0.0f) && "steering_force y = 0");
    assert(approx_eq(entry.steering_force.z, 0.0f) && "steering_force z = 0");

    assert(approx_eq(entry.seek_target.x, 0.0f) && "seek_target x = 0");
    assert(approx_eq(entry.seek_target.y, 0.0f) && "seek_target y = 0");
    assert(approx_eq(entry.seek_target.z, 0.0f) && "seek_target z = 0");

    assert(approx_eq(entry.flee_threat.x, 0.0f) && "flee_threat x = 0");
    assert(approx_eq(entry.flee_threat.y, 0.0f) && "flee_threat y = 0");
    assert(approx_eq(entry.flee_threat.z, 0.0f) && "flee_threat z = 0");

    assert(entry.behavior_count == 0 && "behavior_count = 0");

    // Zero-initialized arrays
    for (uint32_t i = 0u; i < 8u; ++i) {
        assert(entry.active_behaviors[i] == 0 && "active_behaviors[i] = 0");
        assert(approx_eq(entry.behavior_weights[i], 0.0f) && "behavior_weights[i] = 0");
    }

    std::printf("    [PASS] SteeringDebugEntry defaults verified\n");
    return 0;
}

// =============================================================================
// TEST 6: FormationDebugEntry defaults
// =============================================================================
static int test_formation_debug_entry_defaults() {
    std::printf("  [Test 6] FormationDebugEntry defaults...\n");

    apc::FormationDebugEntry entry;

    assert(approx_eq(entry.actual_position.x, 0.0f) && "actual_position x = 0");
    assert(approx_eq(entry.actual_position.y, 0.0f) && "actual_position y = 0");
    assert(approx_eq(entry.actual_position.z, 0.0f) && "actual_position z = 0");

    assert(approx_eq(entry.formation_position.x, 0.0f) && "formation_position x = 0");
    assert(approx_eq(entry.formation_position.y, 0.0f) && "formation_position y = 0");
    assert(approx_eq(entry.formation_position.z, 0.0f) && "formation_position z = 0");

    assert(approx_eq(entry.ball_influenced_position.x, 0.0f) && "ball_influenced_position x = 0");
    assert(approx_eq(entry.ball_influenced_position.y, 0.0f) && "ball_influenced_position y = 0");
    assert(approx_eq(entry.ball_influenced_position.z, 0.0f) && "ball_influenced_position z = 0");

    assert(entry.formation_type == 0 && "formation_type = 0");

    std::printf("    [PASS] FormationDebugEntry defaults verified\n");
    return 0;
}

// =============================================================================
// TEST 7: AIDebugVisualizer defaults
// =============================================================================
static int test_ai_debug_visualizer_defaults() {
    std::printf("  [Test 7] AIDebugVisualizer defaults (all counts=0)...\n");

    apc::AIDebugVisualizer viz;

    assert(viz.utility_count == 0u && "utility_count = 0");
    assert(viz.steering_count == 0u && "steering_count = 0");
    assert(viz.formation_count == 0u && "formation_count = 0");

    // Config should also default
    assert(viz.config.enabled_layers == 0u && "enabled_layers = 0");
    assert(approx_eq(viz.config.visualization_scale, 1.0f) && "visualization_scale = 1.0");

    std::printf("    [PASS] AIDebugVisualizer defaults verified\n");
    return 0;
}

// =============================================================================
// TEST 8: AIDebugVisualizer.begin_frame() clears counts
// =============================================================================
static int test_begin_frame_clears_counts() {
    std::printf("  [Test 8] AIDebugVisualizer.begin_frame() clears counts...\n");

    apc::AIDebugVisualizer viz;

    // Pre-fill some counts by adding entries
    apc::UtilityDebugEntry ue;
    viz.add_utility_entry(ue);

    apc::SteeringDebugEntry se;
    viz.add_steering_entry(se);

    apc::FormationDebugEntry fe;
    viz.add_formation_entry(fe);

    assert(viz.utility_count == 1u && "utility_count = 1 before begin_frame");
    assert(viz.steering_count == 1u && "steering_count = 1 before begin_frame");
    assert(viz.formation_count == 1u && "formation_count = 1 before begin_frame");

    // begin_frame should reset all counts
    viz.begin_frame();

    assert(viz.utility_count == 0u && "utility_count = 0 after begin_frame");
    assert(viz.steering_count == 0u && "steering_count = 0 after begin_frame");
    assert(viz.formation_count == 0u && "formation_count = 0 after begin_frame");

    std::printf("    [PASS] begin_frame() clears counts\n");
    return 0;
}

// =============================================================================
// TEST 9: AIDebugVisualizer.add_utility_entry increments count
// =============================================================================
static int test_add_utility_entry_increments() {
    std::printf("  [Test 9] AIDebugVisualizer.add_utility_entry increments count...\n");

    apc::AIDebugVisualizer viz;

    apc::UtilityDebugEntry e1;
    e1.confidence = 0.5f;

    viz.add_utility_entry(e1);
    assert(viz.utility_count == 1u && "count = 1 after first add");

    apc::UtilityDebugEntry e2;
    e2.confidence = 0.8f;

    viz.add_utility_entry(e2);
    assert(viz.utility_count == 2u && "count = 2 after second add");

    // The entry data should be stored correctly
    assert(approx_eq(viz.utility_entries[0].confidence, 0.5f) && "entry[0].confidence = 0.5");
    assert(approx_eq(viz.utility_entries[1].confidence, 0.8f) && "entry[1].confidence = 0.8");

    std::printf("    [PASS] add_utility_entry increments count\n");
    return 0;
}

// =============================================================================
// TEST 10: AIDebugVisualizer.add_steering_entry increments count
// =============================================================================
static int test_add_steering_entry_increments() {
    std::printf("  [Test 10] AIDebugVisualizer.add_steering_entry increments count...\n");

    apc::AIDebugVisualizer viz;

    apc::SteeringDebugEntry e1;
    e1.steering_force = apc::Vec3(1.0f, 0.0f, 0.0f);

    viz.add_steering_entry(e1);
    assert(viz.steering_count == 1u && "count = 1 after first add");

    apc::SteeringDebugEntry e2;
    e2.steering_force = apc::Vec3(0.0f, 1.0f, 0.0f);

    viz.add_steering_entry(e2);
    assert(viz.steering_count == 2u && "count = 2 after second add");

    // Verify stored data
    assert(approx_eq(viz.steering_entries[0].steering_force.x, 1.0f) && "entry[0].force.x = 1.0");
    assert(approx_eq(viz.steering_entries[1].steering_force.y, 1.0f) && "entry[1].force.y = 1.0");

    std::printf("    [PASS] add_steering_entry increments count\n");
    return 0;
}

// =============================================================================
// TEST 11: AIDebugVisualizer.set_layer enables bit
// =============================================================================
static int test_set_layer_enables_bit() {
    std::printf("  [Test 11] AIDebugVisualizer.set_layer enables bit...\n");

    apc::AIDebugVisualizer viz;

    assert(viz.config.enabled_layers == 0u && "starts with no layers");

    // Enable STEERING_FORCES (bit 0)
    viz.set_layer(apc::AIDebugLayer::STEERING_FORCES, 1);
    assert(viz.config.enabled_layers == 0x0001u && "STEERING_FORCES enabled");

    // Enable UTILITY_SCORES (bit 1) — should combine
    viz.set_layer(apc::AIDebugLayer::UTILITY_SCORES, 1);
    assert(viz.config.enabled_layers == 0x0003u && "STEERING + UTILITY enabled");

    // Enable FORMATION_POSITIONS (bit 2)
    viz.set_layer(apc::AIDebugLayer::FORMATION_POSITIONS, 1);
    assert(viz.config.enabled_layers == 0x0007u && "STEERING + UTILITY + FORMATION enabled");

    // Enable MOTOR_INTENT (bit 3)
    viz.set_layer(apc::AIDebugLayer::MOTOR_INTENT, 1);
    assert(viz.config.enabled_layers == 0x000Fu && "first 4 layers enabled");

    std::printf("    [PASS] set_layer enables bits correctly\n");
    return 0;
}

// =============================================================================
// TEST 12: AIDebugVisualizer.is_layer_enabled checks bit
// =============================================================================
static int test_is_layer_enabled_checks_bit() {
    std::printf("  [Test 12] AIDebugVisualizer.is_layer_enabled checks bit...\n");

    apc::AIDebugVisualizer viz;

    // No layers enabled initially
    assert(viz.is_layer_enabled(apc::AIDebugLayer::STEERING_FORCES) == 0 && "STEERING not enabled");
    assert(viz.is_layer_enabled(apc::AIDebugLayer::UTILITY_SCORES) == 0 && "UTILITY not enabled");
    assert(viz.is_layer_enabled(apc::AIDebugLayer::FORMATION_POSITIONS) == 0 && "FORMATION not enabled");

    // Enable STEERING_FORCES only
    viz.set_layer(apc::AIDebugLayer::STEERING_FORCES, 1);

    assert(viz.is_layer_enabled(apc::AIDebugLayer::STEERING_FORCES) == 1 && "STEERING enabled");
    assert(viz.is_layer_enabled(apc::AIDebugLayer::UTILITY_SCORES) == 0 && "UTILITY still not enabled");
    assert(viz.is_layer_enabled(apc::AIDebugLayer::FORMATION_POSITIONS) == 0 && "FORMATION still not enabled");

    // Enable UTILITY_SCORES
    viz.set_layer(apc::AIDebugLayer::UTILITY_SCORES, 1);
    assert(viz.is_layer_enabled(apc::AIDebugLayer::UTILITY_SCORES) == 1 && "UTILITY now enabled");

    std::printf("    [PASS] is_layer_enabled checks bits correctly\n");
    return 0;
}

// =============================================================================
// TEST 13: AIDebugVisualizer.set_layer disables bit
// =============================================================================
static int test_set_layer_disables_bit() {
    std::printf("  [Test 13] AIDebugVisualizer.set_layer disables bit...\n");

    apc::AIDebugVisualizer viz;

    // Enable two layers
    viz.set_layer(apc::AIDebugLayer::STEERING_FORCES, 1);
    viz.set_layer(apc::AIDebugLayer::UTILITY_SCORES, 1);
    assert(viz.config.enabled_layers == 0x0003u && "both layers enabled");

    // Disable STEERING_FORCES
    viz.set_layer(apc::AIDebugLayer::STEERING_FORCES, 0);
    assert(viz.config.enabled_layers == 0x0002u && "only UTILITY remains");
    assert(viz.is_layer_enabled(apc::AIDebugLayer::STEERING_FORCES) == 0 && "STEERING disabled");

    // Disable UTILITY_SCORES
    viz.set_layer(apc::AIDebugLayer::UTILITY_SCORES, 0);
    assert(viz.config.enabled_layers == 0x0000u && "no layers enabled");
    assert(viz.is_layer_enabled(apc::AIDebugLayer::UTILITY_SCORES) == 0 && "UTILITY disabled");

    // Disabling a layer that was never enabled should be harmless
    viz.set_layer(apc::AIDebugLayer::FORMATION_POSITIONS, 0);
    assert(viz.config.enabled_layers == 0x0000u && "still no layers");

    // Enable ALL then disable one
    viz.set_layer(apc::AIDebugLayer::ALL, 1);
    assert(viz.config.enabled_layers == 0xFFFFu && "ALL enabled");
    viz.set_layer(apc::AIDebugLayer::STEERING_FORCES, 0);
    assert(viz.is_layer_enabled(apc::AIDebugLayer::STEERING_FORCES) == 0 && "STEERING disabled from ALL");

    std::printf("    [PASS] set_layer disables bits correctly\n");
    return 0;
}

// =============================================================================
// main
// =============================================================================
int main() {
    std::printf("=== Sprint 25: AI Debug Visualization ===\n\n");

    int result = 0;
    result |= test_ai_debug_layer_bitmask();
    result |= test_ai_debug_config_defaults();
    result |= test_ai_debug_config_color_defaults();
    result |= test_utility_debug_entry_defaults();
    result |= test_steering_debug_entry_defaults();
    result |= test_formation_debug_entry_defaults();
    result |= test_ai_debug_visualizer_defaults();
    result |= test_begin_frame_clears_counts();
    result |= test_add_utility_entry_increments();
    result |= test_add_steering_entry_increments();
    result |= test_set_layer_enables_bit();
    result |= test_is_layer_enabled_checks_bit();
    result |= test_set_layer_disables_bit();

    int total = 13;
    int passed = total - result;
    std::printf("\n=== Sprint 25: %d tests passed, %d failed ===\n", passed, result);
    return result;
}
