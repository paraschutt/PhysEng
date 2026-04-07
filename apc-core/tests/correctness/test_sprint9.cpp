// =============================================================================
// Sprint 9 Tests — ImpactStyleProfile system + MaterialCurve LUTs
// =============================================================================
//
// Tests for the Phase 3 Sprint 9 deliverables:
//   1. MaterialCurve::make_constant — constant value across range
//   2. MaterialCurve::make_linear — linear ramp from y_start to y_end
//   3. MaterialCurve::make_velocity_restitution — restitution curve shape
//   4. MaterialCurve::make_angle_friction — friction decreases with angle
//   5. MaterialCurve::make_step — step function around threshold
//   6. MaterialCurve evaluate with clamping — out-of-range values clamped
//   7. MaterialCurveRegistry::setup_defaults — 5 default curves
//   8. ImpactStyleProfile factories — default, light, heavy, ball, ragdoll
//   9. ProfileRegistry::setup_defaults — 5 profiles, all retrievable
//  10. ProfileRegistry::resolve — ATTACKER_WINS, HEAVIER_WINS, AVERAGE, MAX
//  11. BodyStyleAssignment — assign, get_profile, get_region
//  12. MaterialCurveRegistry::add and get — custom curve registration
//
// Pattern: int main() + assert(), no test framework.
// =============================================================================

#include "apc_style/apc_material_curve.h"
#include "apc_style/apc_impact_profile.h"
#include <cstdio>
#include <cmath>
#include <cassert>
#include <cstring>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static constexpr float EPS = 1e-5f;

static bool approx_eq(float a, float b, float eps = EPS) {
    return std::abs(a - b) < eps;
}

// =============================================================================
// TEST 1: MaterialCurve::make_constant
// =============================================================================
static int test_constant_curve() {
    std::printf("  [Test 1] MaterialCurve::make_constant...\n");

    apc::MaterialCurve c = apc::MaterialCurve::make_constant(0.75f, -10.0f, 10.0f);

    // Same value at every sample point
    for (uint32_t i = 0; i < apc::APC_CURVE_SAMPLES; ++i) {
        assert(approx_eq(c.samples[i], 0.75f));
    }

    // Evaluate at arbitrary inputs — all should return 0.75
    assert(approx_eq(c.evaluate(-100.0f), 0.75f));
    assert(approx_eq(c.evaluate(-10.0f), 0.75f));
    assert(approx_eq(c.evaluate(0.0f), 0.75f));
    assert(approx_eq(c.evaluate(5.0f), 0.75f));
    assert(approx_eq(c.evaluate(10.0f), 0.75f));
    assert(approx_eq(c.evaluate(999.0f), 0.75f));

    // Default range [0, 1]
    apc::MaterialCurve c2 = apc::MaterialCurve::make_constant(0.5f);
    assert(approx_eq(c2.input_min, 0.0f));
    assert(approx_eq(c2.input_max, 1.0f));
    assert(approx_eq(c2.evaluate(0.5f), 0.5f));

    std::printf("    [PASS] Constant curve returns same value for any input\n");
    return 0;
}

// =============================================================================
// TEST 2: MaterialCurve::make_linear
// =============================================================================
static int test_linear_curve() {
    std::printf("  [Test 2] MaterialCurve::make_linear...\n");

    // Ramp from 0.0 to 1.0 over [0, 1]
    apc::MaterialCurve c = apc::MaterialCurve::make_linear(0.0f, 1.0f);

    assert(approx_eq(c.input_min, 0.0f));
    assert(approx_eq(c.input_max, 1.0f));

    // First sample = y_start
    assert(approx_eq(c.samples[0], 0.0f));
    // Last sample = y_end
    assert(approx_eq(c.samples[apc::APC_CURVE_SAMPLES - 1], 1.0f));
    // Midpoint should be ~0.5
    assert(approx_eq(c.evaluate(0.5f), 0.5f, 0.05f));

    // At boundaries
    assert(approx_eq(c.evaluate(0.0f), 0.0f));
    assert(approx_eq(c.evaluate(1.0f), 1.0f));

    // Custom range [10, 20], ramp from 2.0 to 8.0
    apc::MaterialCurve c2 = apc::MaterialCurve::make_linear(2.0f, 8.0f, 10.0f, 20.0f);
    assert(approx_eq(c2.input_min, 10.0f));
    assert(approx_eq(c2.input_max, 20.0f));
    assert(approx_eq(c2.evaluate(10.0f), 2.0f));
    assert(approx_eq(c2.evaluate(20.0f), 8.0f));
    assert(approx_eq(c2.evaluate(15.0f), 5.0f, 0.05f)); // midpoint

    // Negative range
    apc::MaterialCurve c3 = apc::MaterialCurve::make_linear(1.0f, -1.0f, -5.0f, 5.0f);
    assert(approx_eq(c3.evaluate(-5.0f), 1.0f));
    assert(approx_eq(c3.evaluate(5.0f), -1.0f));
    assert(approx_eq(c3.evaluate(0.0f), 0.0f, 0.05f));

    std::printf("    [PASS] Linear ramp from y_start to y_end verified\n");
    return 0;
}

// =============================================================================
// TEST 3: MaterialCurve::make_velocity_restitution
// =============================================================================
static int test_velocity_restitution_curve() {
    std::printf("  [Test 3] MaterialCurve::make_velocity_restitution...\n");

    // Default: low_speed=1, peak_speed=5, high_speed=20, peak=0.6, high=0.2
    apc::MaterialCurve c = apc::MaterialCurve::make_velocity_restitution();

    assert(approx_eq(c.input_min, 0.0f));
    assert(approx_eq(c.input_max, 20.0f));

    // Low speed (< low_speed=1.0): restitution should be 0 (dead feel)
    // Note: with 8 LUT samples and range [0,20], sample[0] spans [0, 2.857]
    // so there's slight interpolation bleed — we verify "near zero" not exact zero.
    float at_zero = c.evaluate(0.0f);
    float at_05 = c.evaluate(0.5f);
    assert(at_zero < 0.01f && "Zero speed should have near-zero restitution");
    assert(at_05 < 0.1f && "Below low_speed should have low restitution");

    // At peak speed (5.0): should be near peak_value (0.6)
    // Note: with 8 samples, the peak may not land exactly at 5.0
    float at_peak = c.evaluate(5.0f);
    assert(at_peak > 0.4f && at_peak <= 0.65f && "Peak speed should have near-peak restitution");

    // At high speed (20.0): should be near high_value (0.2)
    float at_high = c.evaluate(20.0f);
    assert(approx_eq(at_high, 0.2f, 0.05f) && "High speed should have ~high_value restitution");

    // Mid-range (e.g., 10.0): between peak and high, should be descending
    float at_mid = c.evaluate(10.0f);
    assert(at_mid < at_peak && at_mid > at_high && "Mid-range should be between peak and high");

    // Custom parameters: lower peak
    apc::MaterialCurve c2 = apc::MaterialCurve::make_velocity_restitution(
        2.0f, 8.0f, 30.0f, 0.4f, 0.1f);
    assert(approx_eq(c2.input_max, 30.0f));
    assert(c2.evaluate(1.0f) < 0.01f && "Below custom low_speed = 0");
    float custom_peak = c2.evaluate(8.0f);
    assert(custom_peak > 0.35f && custom_peak <= 0.41f && "Custom peak value correct");
    float custom_high = c2.evaluate(30.0f);
    assert(approx_eq(custom_high, 0.1f, 0.05f) && "Custom high value correct");

    std::printf("    [PASS] Restitution curve shape verified (low=0, mid=peak, high=lower)\n");
    return 0;
}

// =============================================================================
// TEST 4: MaterialCurve::make_angle_friction
// =============================================================================
static int test_angle_friction_curve() {
    std::printf("  [Test 4] MaterialCurve::make_angle_friction...\n");

    // Default: 0→0.8, π/2→0.2 (head-on high friction, grazing low friction)
    apc::MaterialCurve c = apc::MaterialCurve::make_angle_friction();

    assert(approx_eq(c.input_min, 0.0f));
    assert(approx_eq(c.input_max, 1.5708f, 0.001f));

    // Head-on (angle=0): high friction
    float at_zero = c.evaluate(0.0f);
    assert(approx_eq(at_zero, 0.8f, 0.01f) && "Head-on friction should be 0.8");

    // Grazing (angle=π/2): low friction
    float at_grazing = c.evaluate(1.5708f);
    assert(approx_eq(at_grazing, 0.2f, 0.01f) && "Grazing friction should be 0.2");

    // Mid-angle: between head-on and grazing
    float at_mid = c.evaluate(0.7854f); // π/4
    assert(at_mid > 0.2f && at_mid < 0.8f && "Mid-angle friction between head-on and grazing");
    assert(approx_eq(at_mid, 0.5f, 0.05f) && "Mid-angle friction approximately 0.5");

    // Friction should monotonically decrease
    float prev = at_zero;
    for (int i = 1; i <= 10; ++i) {
        float angle = (1.5708f * static_cast<float>(i)) / 10.0f;
        float val = c.evaluate(angle);
        assert(val <= prev + EPS && "Friction should decrease (or stay same) with angle");
        prev = val;
    }

    // Custom parameters
    apc::MaterialCurve c2 = apc::MaterialCurve::make_angle_friction(
        0.0f, 3.14159f, 1.0f, 0.0f);
    assert(approx_eq(c2.evaluate(0.0f), 1.0f) && "Custom head-on friction = 1.0");
    assert(approx_eq(c2.evaluate(3.14159f), 0.0f, 0.01f) && "Custom grazing friction = 0.0");

    std::printf("    [PASS] Friction decreases with angle verified\n");
    return 0;
}

// =============================================================================
// TEST 5: MaterialCurve::make_step
// =============================================================================
static int test_step_curve() {
    std::printf("  [Test 5] MaterialCurve::make_step...\n");

    // Step at 0.5: below → 0.0, above → 1.0
    apc::MaterialCurve c = apc::MaterialCurve::make_step(0.5f, 0.0f, 1.0f);

    assert(approx_eq(c.input_min, 0.0f));
    assert(approx_eq(c.input_max, 1.0f));

    // Well below threshold
    float at_0 = c.evaluate(0.0f);
    assert(approx_eq(at_0, 0.0f) && "Well below threshold should be low_val");

    // Well above threshold
    float at_1 = c.evaluate(1.0f);
    assert(approx_eq(at_1, 1.0f) && "Well above threshold should be high_val");

    // Just below threshold
    float at_04 = c.evaluate(0.4f);
    assert(approx_eq(at_04, 0.0f) && "Just below threshold should be low_val");

    // Just above threshold
    float at_06 = c.evaluate(0.6f);
    assert(approx_eq(at_06, 1.0f) && "Just above threshold should be high_val");

    // Custom range and values
    apc::MaterialCurve c2 = apc::MaterialCurve::make_step(50.0f, -1.0f, 2.0f, 0.0f, 100.0f);
    assert(approx_eq(c2.input_min, 0.0f));
    assert(approx_eq(c2.input_max, 100.0f));
    assert(approx_eq(c2.evaluate(0.0f), -1.0f) && "Below custom threshold = -1.0");
    assert(approx_eq(c2.evaluate(49.0f), -1.0f) && "Just below custom threshold = -1.0");
    assert(approx_eq(c2.evaluate(50.0f), 2.0f) && "At custom threshold = 2.0");
    assert(approx_eq(c2.evaluate(100.0f), 2.0f) && "Above custom threshold = 2.0");

    std::printf("    [PASS] Step function behavior around threshold verified\n");
    return 0;
}

// =============================================================================
// TEST 6: MaterialCurve evaluate with clamping
// =============================================================================
static int test_curve_clamping() {
    std::printf("  [Test 6] MaterialCurve evaluate with clamping...\n");

    // Linear ramp [0, 10]: 0→100, 10→200
    apc::MaterialCurve c = apc::MaterialCurve::make_linear(100.0f, 200.0f, 0.0f, 10.0f);

    // Within range
    assert(approx_eq(c.evaluate(0.0f), 100.0f) && "At min input returns samples[0]");
    assert(approx_eq(c.evaluate(10.0f), 200.0f) && "At max input returns samples[last]");
    assert(approx_eq(c.evaluate(5.0f), 150.0f, 1.0f) && "Midpoint interpolation correct");

    // Below min — should clamp to samples[0]
    float below = c.evaluate(-1000.0f);
    assert(approx_eq(below, 100.0f) && "Far below min clamps to first sample");

    float below2 = c.evaluate(-0.001f);
    assert(approx_eq(below2, 100.0f) && "Just below min clamps to first sample");

    // Above max — should clamp to samples[last]
    float above = c.evaluate(1000.0f);
    assert(approx_eq(above, 200.0f) && "Far above max clamps to last sample");

    float above2 = c.evaluate(10.001f);
    assert(approx_eq(above2, 200.0f) && "Just above max clamps to last sample");

    // Zero-range curve (input_min == input_max)
    apc::MaterialCurve flat = apc::MaterialCurve::make_constant(42.0f, 5.0f, 5.0f);
    // range <= 0 → always returns samples[0]
    assert(approx_eq(flat.evaluate(0.0f), 42.0f) && "Zero-range returns samples[0] for below");
    assert(approx_eq(flat.evaluate(5.0f), 42.0f) && "Zero-range returns samples[0] for at");
    assert(approx_eq(flat.evaluate(100.0f), 42.0f) && "Zero-range returns samples[0] for above");

    std::printf("    [PASS] Values outside [min,max] are clamped correctly\n");
    return 0;
}

// =============================================================================
// TEST 7: MaterialCurveRegistry::setup_defaults
// =============================================================================
static int test_curve_registry_defaults() {
    std::printf("  [Test 7] MaterialCurveRegistry::setup_defaults...\n");

    apc::MaterialCurveRegistry reg;
    uint32_t count = reg.setup_defaults();

    assert(count == 5 && "setup_defaults should create exactly 5 curves");
    assert(reg.count == 5);

    // All 5 curves should be retrievable
    for (uint8_t id = 0; id < 5; ++id) {
        const apc::MaterialCurve* curve = reg.get(id);
        assert(curve != nullptr && "Default curve should be retrievable by ID");
        assert(curve->curve_id == id && "Curve ID should match registry index");
    }

    // Curve 0: velocity restitution (input range [0, 20])
    const apc::MaterialCurve* c0 = reg.get(0);
    assert(approx_eq(c0->input_min, 0.0f));
    assert(approx_eq(c0->input_max, 20.0f));
    // At peak speed (5.0) should have non-zero restitution
    assert(c0->evaluate(5.0f) > 0.3f && "Default velocity curve has peak restitution");

    // Curve 1: angle friction (input range [0, π/2])
    const apc::MaterialCurve* c1 = reg.get(1);
    assert(approx_eq(c1->input_max, 1.5708f, 0.001f));
    assert(c1->evaluate(0.0f) > c1->evaluate(1.5708f) && "Head-on friction > grazing friction");

    // Curve 2: heavy body restitution
    const apc::MaterialCurve* c2 = reg.get(2);
    assert(c2 != nullptr);
    assert(c2->evaluate(3.0f) > 0.1f && "Heavy restitution curve has values");

    // Curve 3: sticky ground friction
    const apc::MaterialCurve* c3 = reg.get(3);
    assert(c3 != nullptr);
    assert(c3->evaluate(0.0f) > 0.5f && "Sticky friction has high head-on grip");

    // Curve 4: linear momentum transfer
    const apc::MaterialCurve* c4 = reg.get(4);
    assert(c4 != nullptr);
    assert(approx_eq(c4->input_max, 30.0f));
    assert(approx_eq(c4->evaluate(0.0f), 0.5f) && "Momentum curve starts at 0.5");
    assert(approx_eq(c4->evaluate(30.0f), 1.5f) && "Momentum curve ends at 1.5");

    // Invalid ID should return nullptr
    assert(reg.get(5) == nullptr && "ID beyond count returns nullptr");
    assert(reg.get(255) == nullptr && "Out-of-range ID returns nullptr");

    std::printf("    [PASS] 5 default curves created, all retrievable by ID\n");
    return 0;
}

// =============================================================================
// TEST 8: ImpactStyleProfile factories
// =============================================================================
static int test_profile_factories() {
    std::printf("  [Test 8] ImpactStyleProfile factories...\n");

    // --- make_default ---
    apc::ImpactStyleProfile def = apc::ImpactStyleProfile::make_default();
    assert(std::strcmp(def.name, "default") == 0 && "Default profile name");
    assert(approx_eq(def.base_momentum_transfer, 1.0f));
    assert(approx_eq(def.vertical_bias, 0.0f));
    assert(approx_eq(def.spin_multiplier, 1.0f));
    assert(def.hit_stop_duration_ms < 0.01f && "Default has no hit-stop");
    assert(def.camera_shake_intensity < 0.01f && "Default has no camera shake");
    assert(def.friction_curve_id == 0xFF && "Default uses solver defaults for friction");
    assert(def.restitution_curve_id == 0xFF && "Default uses solver defaults for restitution");

    // --- make_light ---
    apc::ImpactStyleProfile light = apc::ImpactStyleProfile::make_light();
    assert(std::strcmp(light.name, "light") == 0 && "Light profile name");
    assert(approx_eq(light.base_momentum_transfer, 0.7f));
    assert(approx_eq(light.vertical_bias, 0.3f));
    assert(approx_eq(light.spin_multiplier, 2.0f));
    assert(approx_eq(light.hit_stop_duration_ms, 30.0f));
    assert(approx_eq(light.camera_shake_intensity, 0.1f));
    assert(approx_eq(light.light_impact_threshold, 1.0f));
    assert(approx_eq(light.heavy_impact_threshold, 5.0f));
    assert(approx_eq(light.critical_impact_threshold, 15.0f));

    // --- make_heavy ---
    apc::ImpactStyleProfile heavy = apc::ImpactStyleProfile::make_heavy();
    assert(std::strcmp(heavy.name, "heavy") == 0 && "Heavy profile name");
    assert(approx_eq(heavy.base_momentum_transfer, 1.5f));
    assert(approx_eq(heavy.vertical_bias, 0.0f));
    assert(approx_eq(heavy.spin_multiplier, 0.3f));
    assert(approx_eq(heavy.hit_stop_duration_ms, 80.0f));
    assert(approx_eq(heavy.camera_shake_intensity, 0.5f));
    assert(approx_eq(heavy.light_impact_threshold, 5.0f));
    assert(approx_eq(heavy.heavy_impact_threshold, 15.0f));
    assert(approx_eq(heavy.critical_impact_threshold, 40.0f));

    // --- make_ball ---
    apc::ImpactStyleProfile ball = apc::ImpactStyleProfile::make_ball();
    assert(std::strcmp(ball.name, "ball") == 0 && "Ball profile name");
    assert(approx_eq(ball.base_momentum_transfer, 1.2f));
    assert(approx_eq(ball.vertical_bias, 0.15f));
    assert(approx_eq(ball.spin_multiplier, 1.8f));
    assert(ball.hit_stop_duration_ms < 0.01f && "Ball has no hit-stop");
    assert(ball.camera_shake_intensity < 0.01f && "Ball has no camera shake");
    assert(approx_eq(ball.light_impact_threshold, 3.0f));

    // --- make_ragdoll ---
    apc::ImpactStyleProfile ragdoll = apc::ImpactStyleProfile::make_ragdoll();
    assert(std::strcmp(ragdoll.name, "ragdoll") == 0 && "Ragdoll profile name");
    assert(approx_eq(ragdoll.base_momentum_transfer, 1.3f));
    assert(approx_eq(ragdoll.vertical_bias, 0.2f));
    assert(approx_eq(ragdoll.spin_multiplier, 1.5f));
    assert(approx_eq(ragdoll.hit_stop_duration_ms, 50.0f));
    assert(approx_eq(ragdoll.camera_shake_intensity, 0.3f));
    assert(approx_eq(ragdoll.heavy_impact_threshold, 8.0f));
    assert(approx_eq(ragdoll.critical_impact_threshold, 20.0f));

    std::printf("    [PASS] All 5 profile factories produce valid profiles\n");
    return 0;
}

// =============================================================================
// TEST 9: ProfileRegistry::setup_defaults
// =============================================================================
static int test_profile_registry_defaults() {
    std::printf("  [Test 9] ProfileRegistry::setup_defaults...\n");

    apc::ProfileRegistry reg;
    uint32_t count = reg.setup_defaults();

    assert(count == 5 && "setup_defaults should create exactly 5 profiles");
    assert(reg.count == 5);

    // All 5 profiles retrievable
    const char* expected_names[] = {"default", "light", "heavy", "ball", "ragdoll"};
    for (uint16_t id = 0; id < 5; ++id) {
        const apc::ImpactStyleProfile* p = reg.get(id);
        assert(p != nullptr && "Default profile should be retrievable");
        assert(p->profile_id == id && "Profile ID matches registry index");
        assert(std::strcmp(p->name, expected_names[id]) == 0 && "Profile name matches");
    }

    // Verify each has distinctive values
    const apc::ImpactStyleProfile* p_def = reg.get(0);
    const apc::ImpactStyleProfile* p_light = reg.get(1);
    const apc::ImpactStyleProfile* p_heavy = reg.get(2);
    assert(p_def->base_momentum_transfer > p_light->base_momentum_transfer
           && "Default momentum > light momentum");
    assert(p_heavy->base_momentum_transfer > p_def->base_momentum_transfer
           && "Heavy momentum > default momentum");
    assert(p_light->spin_multiplier > p_heavy->spin_multiplier
           && "Light spins more than heavy");

    // Invalid ID
    assert(reg.get(5) == nullptr && "ID beyond count returns nullptr");
    assert(reg.get(65535) == nullptr && "Out-of-range ID returns nullptr");

    std::printf("    [PASS] 5 profiles created, all retrievable with correct names\n");
    return 0;
}

// =============================================================================
// TEST 10: ProfileRegistry::resolve with different blend strategies
// =============================================================================
static int test_profile_resolve_strategies() {
    std::printf("  [Test 10] ProfileRegistry::resolve blend strategies...\n");

    apc::ProfileRegistry reg;
    reg.setup_defaults();

    // ID 0 = default (momentum=1.0, vertical_bias=0.0, spin=1.0)
    // ID 2 = heavy   (momentum=1.5, vertical_bias=0.0, spin=0.3)

    // --- ATTACKER_WINS: faster body's profile ---
    // A=heavy (speed=10), B=default (speed=2) → heavy wins
    apc::ImpactStyleProfile r1 = reg.resolve(2, 0,
        apc::ProfileBlendStrategy::ATTACKER_WINS,
        1.0f, 1.0f, 10.0f, 2.0f);
    assert(approx_eq(r1.base_momentum_transfer, 1.5f)
           && "ATTACKER_WINS: heavy (faster) should win");

    // A=default (speed=1), B=heavy (speed=8) → heavy wins
    apc::ImpactStyleProfile r2 = reg.resolve(0, 2,
        apc::ProfileBlendStrategy::ATTACKER_WINS,
        1.0f, 1.0f, 1.0f, 8.0f);
    assert(approx_eq(r2.base_momentum_transfer, 1.5f)
           && "ATTACKER_WINS: heavy (faster) should win when it's body B");

    // Equal speeds → body A wins (speed_a >= speed_b)
    apc::ImpactStyleProfile r2b = reg.resolve(0, 2,
        apc::ProfileBlendStrategy::ATTACKER_WINS,
        1.0f, 1.0f, 5.0f, 5.0f);
    assert(approx_eq(r2b.base_momentum_transfer, 1.0f)
           && "ATTACKER_WINS: equal speed, body A (default) wins");

    // --- HEAVIER_WINS: more massive body's profile ---
    // A=heavy (mass=10), B=default (mass=1) → heavy wins
    apc::ImpactStyleProfile r3 = reg.resolve(2, 0,
        apc::ProfileBlendStrategy::HEAVIER_WINS,
        10.0f, 1.0f, 0.0f, 0.0f);
    assert(approx_eq(r3.base_momentum_transfer, 1.5f)
           && "HEAVIER_WINS: heavy (more massive) should win");

    // A=default (mass=1), B=heavy (mass=10) → heavy wins
    apc::ImpactStyleProfile r4 = reg.resolve(0, 2,
        apc::ProfileBlendStrategy::HEAVIER_WINS,
        1.0f, 10.0f, 0.0f, 0.0f);
    assert(approx_eq(r4.base_momentum_transfer, 1.5f)
           && "HEAVIER_WINS: heavy (more massive) wins when body B");

    // --- AVERAGE: blend all parameters ---
    // default(momentum=1.0) + light(momentum=0.7) → avg = 0.85
    apc::ImpactStyleProfile r5 = reg.resolve(0, 1,
        apc::ProfileBlendStrategy::AVERAGE);
    assert(approx_eq(r5.base_momentum_transfer, 0.85f)
           && "AVERAGE: momentum should be (1.0+0.7)/2 = 0.85");
    assert(approx_eq(r5.vertical_bias, 0.15f)
           && "AVERAGE: vertical_bias should be (0.0+0.3)/2 = 0.15");
    assert(approx_eq(r5.spin_multiplier, 1.5f)
           && "AVERAGE: spin should be (1.0+2.0)/2 = 1.5");
    assert(approx_eq(r5.hit_stop_duration_ms, 15.0f)
           && "AVERAGE: hit_stop should be (0+30)/2 = 15.0");
    assert(approx_eq(r5.camera_shake_intensity, 0.05f)
           && "AVERAGE: camera_shake should be (0+0.1)/2 = 0.05");
    assert(approx_eq(r5.time_scale_on_impact, 1.0f)
           && "AVERAGE: time_scale should be (1.0+1.0)/2 = 1.0");

    // --- MAX: take maximum of each parameter ---
    // default(momentum=1.0, spin=1.0) + heavy(momentum=1.5, spin=0.3) → max
    apc::ImpactStyleProfile r6 = reg.resolve(0, 2,
        apc::ProfileBlendStrategy::MAX);
    assert(approx_eq(r6.base_momentum_transfer, 1.5f)
           && "MAX: momentum should be max(1.0, 1.5) = 1.5");
    assert(approx_eq(r6.spin_multiplier, 1.0f)
           && "MAX: spin should be max(1.0, 0.3) = 1.0");

    // --- BODY_A_WINS / BODY_B_WINS ---
    apc::ImpactStyleProfile r7 = reg.resolve(0, 2,
        apc::ProfileBlendStrategy::BODY_A_WINS);
    assert(approx_eq(r7.base_momentum_transfer, 1.0f)
           && "BODY_A_WINS: returns body A profile");

    apc::ImpactStyleProfile r8 = reg.resolve(0, 2,
        apc::ProfileBlendStrategy::BODY_B_WINS);
    assert(approx_eq(r8.base_momentum_transfer, 1.5f)
           && "BODY_B_WINS: returns body B profile");

    // --- Invalid IDs → fallback to default ---
    apc::ImpactStyleProfile r9 = reg.resolve(255, 200,
        apc::ProfileBlendStrategy::AVERAGE);
    assert(approx_eq(r9.base_momentum_transfer, 1.0f)
           && "Both invalid IDs → fallback default");
    assert(std::strcmp(r9.name, "default") == 0);

    // One valid, one invalid → returns the valid one
    apc::ImpactStyleProfile r10 = reg.resolve(2, 255,
        apc::ProfileBlendStrategy::AVERAGE);
    assert(approx_eq(r10.base_momentum_transfer, 1.5f)
           && "One valid ID → returns valid profile");

    std::printf("    [PASS] ATTACKER_WINS, HEAVIER_WINS, AVERAGE, MAX all correct\n");
    return 0;
}

// =============================================================================
// TEST 11: BodyStyleAssignment
// =============================================================================
static int test_body_style_assignment() {
    std::printf("  [Test 11] BodyStyleAssignment...\n");

    apc::BodyStyleAssignment bsa;
    bsa.reset();

    // After reset, all bodies should have profile 0 and UNKNOWN region
    assert(bsa.get_profile(0) == 0 && "After reset, body 0 has profile 0");
    assert(bsa.get_profile(100) == 0 && "After reset, body 100 has profile 0");
    assert(bsa.get_profile(255) == 0 && "After reset, body 255 has profile 0");
    assert(bsa.get_region(0) == apc::ContactRegion::UNKNOWN);
    assert(bsa.get_region(50) == apc::ContactRegion::UNKNOWN);

    // Assign profiles
    bsa.assign(0, 1, apc::ContactRegion::HEAD);
    assert(bsa.get_profile(0) == 1 && "Body 0 assigned profile 1");
    assert(bsa.get_region(0) == apc::ContactRegion::HEAD && "Body 0 region = HEAD");

    bsa.assign(5, 2, apc::ContactRegion::TORSO);
    assert(bsa.get_profile(5) == 2 && "Body 5 assigned profile 2");
    assert(bsa.get_region(5) == apc::ContactRegion::TORSO && "Body 5 region = TORSO");

    bsa.assign(10, 3, apc::ContactRegion::UPPER_ARM);
    assert(bsa.get_profile(10) == 3);
    assert(bsa.get_region(10) == apc::ContactRegion::UPPER_ARM);

    // Default region (no explicit region)
    bsa.assign(20, 4);
    assert(bsa.get_profile(20) == 4 && "Body 20 assigned profile 4");
    assert(bsa.get_region(20) == apc::ContactRegion::UNKNOWN
           && "Body 20 default region = UNKNOWN");

    // Overwrite assignment
    bsa.assign(0, 3, apc::ContactRegion::FOOT);
    assert(bsa.get_profile(0) == 3 && "Body 0 reassigned to profile 3");
    assert(bsa.get_region(0) == apc::ContactRegion::FOOT && "Body 0 region updated to FOOT");

    // Out-of-range body index → silently ignored, returns default
    bsa.assign(256, 99, apc::ContactRegion::HIP);
    assert(bsa.get_profile(256) == 0 && "Out-of-range body returns profile 0");
    assert(bsa.get_region(256) == apc::ContactRegion::UNKNOWN
           && "Out-of-range body returns UNKNOWN region");

    // Unassigned bodies remain at default
    assert(bsa.get_profile(99) == 0 && "Unassigned body 99 has profile 0");
    assert(bsa.get_region(99) == apc::ContactRegion::UNKNOWN);

    // Verify all ContactRegion enum values can be stored and retrieved
    bsa.assign(30, 0, apc::ContactRegion::LOWER_LEG);
    assert(bsa.get_region(30) == apc::ContactRegion::LOWER_LEG);
    bsa.assign(31, 0, apc::ContactRegion::HAND);
    assert(bsa.get_region(31) == apc::ContactRegion::HAND);
    bsa.assign(32, 0, apc::ContactRegion::SHOULDER);
    assert(bsa.get_region(32) == apc::ContactRegion::SHOULDER);
    bsa.assign(33, 0, apc::ContactRegion::HIP);
    assert(bsa.get_region(33) == apc::ContactRegion::HIP);

    std::printf("    [PASS] assign, get_profile, get_region all work correctly\n");
    return 0;
}

// =============================================================================
// TEST 12: MaterialCurveRegistry::add and get (custom registration)
// =============================================================================
static int test_curve_registry_add_get() {
    std::printf("  [Test 12] MaterialCurveRegistry::add and get...\n");

    apc::MaterialCurveRegistry reg;

    // Add custom curves
    uint8_t id0 = reg.add(apc::MaterialCurve::make_constant(0.5f));
    assert(id0 == 0 && "First added curve gets ID 0");
    assert(reg.count == 1);

    uint8_t id1 = reg.add(apc::MaterialCurve::make_linear(0.0f, 1.0f, -5.0f, 5.0f));
    assert(id1 == 1 && "Second added curve gets ID 1");
    assert(reg.count == 2);

    uint8_t id2 = reg.add(apc::MaterialCurve::make_velocity_restitution());
    assert(id2 == 2 && "Third added curve gets ID 2");

    // Retrieve and verify each
    const apc::MaterialCurve* c0 = reg.get(id0);
    assert(c0 != nullptr);
    assert(c0->curve_id == 0);
    assert(approx_eq(c0->evaluate(999.0f), 0.5f) && "Custom constant curve evaluates correctly");

    const apc::MaterialCurve* c1 = reg.get(id1);
    assert(c1 != nullptr);
    assert(approx_eq(c1->evaluate(-5.0f), 0.0f) && "Custom linear at min");
    assert(approx_eq(c1->evaluate(5.0f), 1.0f) && "Custom linear at max");

    const apc::MaterialCurve* c2 = reg.get(id2);
    assert(c2 != nullptr);
    assert(c2->evaluate(5.0f) > 0.3f && "Custom velocity curve has peak");

    // Evaluate via registry helper
    assert(approx_eq(reg.evaluate(id0, 42.0f), 0.5f) && "Registry evaluate constant");
    assert(approx_eq(reg.evaluate(id1, 0.0f), 0.0f) && "Registry evaluate linear at min");

    // Fallback for invalid ID
    float fallback = reg.evaluate(255, 10.0f, 99.0f);
    assert(approx_eq(fallback, 99.0f) && "Invalid ID returns fallback value");

    float default_fb = reg.evaluate(10, 5.0f);
    assert(approx_eq(default_fb, 0.0f) && "Invalid ID with default fallback returns 0.0");

    // Add enough curves to approach APC_MAX_CURVES
    apc::MaterialCurveRegistry reg2;
    for (uint32_t i = 0; i < apc::APC_MAX_CURVES; ++i) {
        uint8_t id = reg2.add(apc::MaterialCurve::make_constant(static_cast<float>(i)));
        assert(id == static_cast<uint8_t>(i) && "Sequential IDs assigned");
    }
    assert(reg2.count == apc::APC_MAX_CURVES);

    // Overflow → returns 0xFF (error sentinel)
    uint8_t overflow_id = reg2.add(apc::MaterialCurve::make_constant(0.0f));
    assert(overflow_id == 0xFF && "Overflow add returns 0xFF");
    // Count should not exceed max
    assert(reg2.count == apc::APC_MAX_CURVES && "Count stays at max after overflow");

    std::printf("    [PASS] Custom curve registration and lookup work correctly\n");
    return 0;
}

// =============================================================================
// TEST 13 (Bonus): Curve interpolation accuracy
// =============================================================================
static int test_curve_interpolation_accuracy() {
    std::printf("  [Test 13] Curve interpolation accuracy...\n");

    // Linear 0→1 over [0, 7]: with 8 samples, each sample is at integer x
    // so evaluate(0)=0, evaluate(1)=1/7, ..., evaluate(7)=1
    apc::MaterialCurve c = apc::MaterialCurve::make_linear(0.0f, 7.0f, 0.0f, 7.0f);

    // Exact sample points
    assert(approx_eq(c.evaluate(0.0f), 0.0f) && "Sample 0: 0.0");
    assert(approx_eq(c.evaluate(1.0f), 1.0f) && "Sample 1: 1.0");
    assert(approx_eq(c.evaluate(7.0f), 7.0f) && "Sample 7: 7.0");

    // Midpoint between samples (e.g., x=1.5 → y=1.5)
    assert(approx_eq(c.evaluate(1.5f), 1.5f) && "Interpolation at 1.5");

    // Quarter point (x=0.875 → t=0.125 → index=0.875 → between sample 0 and 1)
    float q = c.evaluate(0.875f);
    assert(approx_eq(q, 0.875f, 0.01f) && "Quarter point interpolation");

    // All samples monotonically increase for linear ramp
    for (uint32_t i = 1; i < apc::APC_CURVE_SAMPLES; ++i) {
        assert(c.samples[i] >= c.samples[i - 1] - EPS
               && "Linear ramp samples are monotonically increasing");
    }

    std::printf("    [PASS] Interpolation accuracy verified\n");
    return 0;
}

// =============================================================================
// TEST 14 (Bonus): Profile blend MAX with light vs heavy
// =============================================================================
static int test_profile_blend_max_light_heavy() {
    std::printf("  [Test 14] Profile blend MAX light vs heavy...\n");

    apc::ProfileRegistry reg;
    reg.setup_defaults();

    // light: momentum=0.7, vertical_bias=0.3, spin=2.0, hit_stop=30, shake=0.1
    // heavy: momentum=1.5, vertical_bias=0.0, spin=0.3, hit_stop=80, shake=0.5
    apc::ImpactStyleProfile blended = reg.resolve(1, 2, apc::ProfileBlendStrategy::MAX);

    assert(approx_eq(blended.base_momentum_transfer, 1.5f)
           && "MAX: momentum = max(0.7, 1.5) = 1.5");
    assert(approx_eq(blended.vertical_bias, 0.3f)
           && "MAX: vertical_bias = max(0.3, 0.0) = 0.3");
    assert(approx_eq(blended.spin_multiplier, 2.0f)
           && "MAX: spin = max(2.0, 0.3) = 2.0");
    assert(approx_eq(blended.hit_stop_duration_ms, 80.0f)
           && "MAX: hit_stop = max(30, 80) = 80");
    assert(approx_eq(blended.camera_shake_intensity, 0.5f)
           && "MAX: shake = max(0.1, 0.5) = 0.5");

    std::printf("    [PASS] MAX blend of light+heavy picks maximums correctly\n");
    return 0;
}

// =============================================================================
// Main
// =============================================================================
int main() {
    std::printf("Running Sprint 9 Tests: ImpactStyleProfile + MaterialCurve LUTs\n");
    std::printf("================================================================\n");

    int result = 0;
    result |= test_constant_curve();
    result |= test_linear_curve();
    result |= test_velocity_restitution_curve();
    result |= test_angle_friction_curve();
    result |= test_step_curve();
    result |= test_curve_clamping();
    result |= test_curve_registry_defaults();
    result |= test_profile_factories();
    result |= test_profile_registry_defaults();
    result |= test_profile_resolve_strategies();
    result |= test_body_style_assignment();
    result |= test_curve_registry_add_get();
    result |= test_curve_interpolation_accuracy();
    result |= test_profile_blend_max_light_heavy();

    if (result == 0) {
        std::printf("\nAll Sprint 9 tests PASSED.\n");
    } else {
        std::printf("\nSome Sprint 9 tests FAILED.\n");
    }
    return result;
}
