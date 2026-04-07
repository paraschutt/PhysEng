// =============================================================================
// Sprint 11 Tests — Game Hooks: Hit-Stop, Time-Scale, Camera Shake, Blend Overrides
// =============================================================================
//
// Tests for Phase 3 Sprint 11 (Game Hooks system):
//   1. GameHookSystem — default construction and configuration
//   2. classify_impact — light/medium/heavy/critical severity bands
//   3. process_impacts: hit-stop — freeze frame on impact
//   4. process_impacts: camera shake — shake with correct intensity
//   5. process_impacts: time-scale — slow-motion on heavy impacts
//   6. process_impacts: blend override — auto-ragdoll on critical impacts
//   7. process_impacts: sound trigger — impact sound generation
//   8. GameHookAccumulator: hit-stop tick — decrease over time, expire
//   9. GameHookAccumulator: time-scale — active scale returned, decays
//  10. GameHookAccumulator: camera shake decay — intensity decreases
//  11. Multiple effects for single impact — one impact, many outputs
//  12. Game hooks determinism — same events produce same outputs
//

#include "apc_style/apc_material_curve.h"
#include "apc_style/apc_impact_profile.h"
#include "apc_style/apc_stylized_solver.h"
#include "apc_style/apc_game_hooks.h"
#include "apc_solver/apc_rigid_body.h"
#include "apc_collision/apc_sphere_sphere.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include "apc_platform/apc_fp_mode.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

using namespace apc;

// =============================================================================
// Helpers
// =============================================================================

// Create a custom profile with ALL game-feel hooks enabled
static ImpactStyleProfile make_test_profile() {
    ImpactStyleProfile p;
    p.profile_id   = 10;
    p.name         = "test_all_hooks";
    p.hit_stop_duration_ms   = 60.0f;
    p.camera_shake_intensity = 0.7f;
    p.time_scale_on_impact   = 0.3f;
    p.time_scale_duration_ms = 300.0f;
    p.light_impact_threshold   = 5.0f;
    p.heavy_impact_threshold   = 20.0f;
    p.critical_impact_threshold = 50.0f;
    p.base_momentum_transfer = 1.0f;
    p.vertical_bias          = 0.0f;
    p.spin_multiplier        = 1.0f;
    return p;
}

// Create an ImpactEvent with given force
static ImpactEvent make_impact(float force, float speed = 10.0f,
                                uint16_t profile_id = 0,
                                float timestamp = 0.016f) {
    ImpactEvent evt;
    evt.reset();
    evt.body_a              = 0;
    evt.body_b              = 1;
    evt.contact_normal      = Vec3(0, 1, 0);
    evt.contact_point       = Vec3(0, 0, 0);
    evt.impact_force        = force;
    evt.relative_speed      = speed;
    evt.resolved_profile_id = profile_id;
    evt.timestamp           = timestamp;
    evt.new_contact         = true;
    return evt;
}

// =============================================================================
// TEST 1: GameHookSystem construction — default config, can construct
// =============================================================================
static int test_hook_system_construction() {
    std::printf("  [Test 1] GameHookSystem construction...\n");

    // --- Default construction ---
    GameHookSystem sys;
    GameHookOutput outputs[4];
    uint32_t count = sys.process_impacts(nullptr, 0, nullptr, outputs, 4);
    assert(count == 0 && "No-op process should return 0");

    // --- Construction with custom config (hit-stop disabled) ---
    GameHookConfig cfg;
    cfg.enable_hit_stop    = false;
    cfg.enable_camera_shake = false;
    cfg.auto_ragdoll_threshold = 0.9f;
    cfg.max_hit_stop_ms    = 200.0f;
    GameHookSystem sys2(cfg);

    ProfileRegistry registry;
    registry.setup_defaults();

    // Heavy profile hit — hit-stop should NOT fire when disabled
    ImpactEvent evt = make_impact(500.0f, 10.0f, 2);
    count = sys2.process_impacts(&evt, 1, &registry, outputs, 4);

    bool has_hit_stop = false;
    for (uint32_t i = 0; i < count; ++i) {
        if (outputs[i].type == HookEffectType::HIT_STOP) has_hit_stop = true;
    }
    assert(!has_hit_stop && "Disabled hit-stop should not generate output");

    // --- Verify set_config updates behaviour ---
    GameHookConfig cfg2;
    cfg2.enable_sound_triggers = false;
    sys.set_config(cfg2);

    ImpactEvent evt2 = make_impact(3.0f, 5.0f, 1); // light profile, small hit
    count = sys.process_impacts(&evt2, 1, &registry, outputs, 4);

    bool has_sound = false;
    for (uint32_t i = 0; i < count; ++i) {
        if (outputs[i].type == HookEffectType::SOUND_TRIGGER) has_sound = true;
    }
    assert(!has_sound && "Disabled sound triggers should not generate output");

    std::printf("    [PASS] Default/config construction, disabled hooks respected\n");
    return 0;
}

// =============================================================================
// TEST 2: classify_impact — light/medium/heavy/critical classification
// =============================================================================
static int test_classify_impact() {
    std::printf("  [Test 2] classify_impact classification...\n");

    // Default profile thresholds: light=2.0, heavy=10.0, critical=25.0
    ImpactStyleProfile prof = ImpactStyleProfile::make_default();

    // Below light threshold -> 0.0
    {
        ImpactEvent evt = make_impact(0.5f);
        float intensity = GameHookSystem::classify_impact(evt, prof);
        assert(intensity < 0.001f && "Below light should be 0.0");
    }

    // At light threshold -> 0.0
    {
        ImpactEvent evt = make_impact(2.0f);
        float intensity = GameHookSystem::classify_impact(evt, prof);
        assert(intensity < 0.001f && "At light should be 0.0");
    }

    // Just above light -> 0.0625
    {
        ImpactEvent evt = make_impact(3.0f);
        float intensity = GameHookSystem::classify_impact(evt, prof);
        // range = 10-2 = 8; t = (3-2)/8 = 0.125; intensity = 0.5*0.125 = 0.0625
        assert(intensity > 0.0f && intensity < 0.5f && "Light range");
        assert(std::abs(intensity - 0.0625f) < 0.001f && "Exact light value");
    }

    // At heavy threshold -> 0.5
    {
        ImpactEvent evt = make_impact(10.0f);
        float intensity = GameHookSystem::classify_impact(evt, prof);
        assert(std::abs(intensity - 0.5f) < 0.001f && "At heavy = 0.5");
    }

    // Midway heavy-critical -> 0.75
    {
        ImpactEvent evt = make_impact(17.5f);
        float intensity = GameHookSystem::classify_impact(evt, prof);
        // range = 25-10 = 15; t = (17.5-10)/15 = 0.5; intensity = 0.5+0.25 = 0.75
        assert(std::abs(intensity - 0.75f) < 0.001f && "Mid heavy-critical = 0.75");
    }

    // At critical threshold -> 1.0
    {
        ImpactEvent evt = make_impact(25.0f);
        float intensity = GameHookSystem::classify_impact(evt, prof);
        assert(std::abs(intensity - 1.0f) < 0.001f && "At critical = 1.0");
    }

    // Far above critical -> capped at 1.0
    {
        ImpactEvent evt = make_impact(1000.0f);
        float intensity = GameHookSystem::classify_impact(evt, prof);
        assert(intensity <= 1.001f && "Above critical capped at 1.0");
    }

    // Zero force with speed fallback: force = speed*10 = 50, above critical
    {
        ImpactEvent evt = make_impact(0.0f, 5.0f);
        float intensity = GameHookSystem::classify_impact(evt, prof);
        assert(intensity >= 0.99f && "Speed fallback should produce high intensity");
    }

    std::printf("    [PASS] All severity bands classified correctly\n");
    return 0;
}

// =============================================================================
// TEST 3: process_impacts: hit-stop — impact above threshold generates HIT_STOP
// =============================================================================
static int test_process_hit_stop() {
    std::printf("  [Test 3] process_impacts: hit-stop...\n");

    ProfileRegistry registry;
    registry.setup_defaults();

    // Heavy profile (id=2): hit_stop=80ms, light=5, heavy=15, critical=40
    // force=500 -> intensity=1.0 -> duration = 80*1.0 = 80ms (cap 120ms)
    GameHookSystem sys;
    GameHookOutput outputs[16];

    ImpactEvent evt = make_impact(500.0f, 10.0f, 2);
    uint32_t count = sys.process_impacts(&evt, 1, &registry, outputs, 16);
    assert(count >= 1 && "Should produce at least one output");

    bool found = false;
    for (uint32_t i = 0; i < count; ++i) {
        if (outputs[i].type == HookEffectType::HIT_STOP) {
            found = true;
            assert(outputs[i].float_param > 70.0f && "Hit-stop duration > 70ms");
            assert(outputs[i].float_param <= 120.0f && "Hit-stop capped at max");
            assert(outputs[i].target_body == 1 && "Hit-stop targets receiver");
            assert(outputs[i].impact_intensity > 0.99f && "Intensity near 1.0");
            break;
        }
    }
    assert(found && "HIT_STOP output must exist");

    std::printf("    [PASS] HIT_STOP generated: duration=%.1fms\n",
               outputs[0].float_param);
    return 0;
}

// =============================================================================
// TEST 4: process_impacts: camera shake — correct intensity
// =============================================================================
static int test_process_camera_shake() {
    std::printf("  [Test 4] process_impacts: camera shake...\n");

    ProfileRegistry registry;
    registry.setup_defaults();

    // Heavy profile (id=2): camera_shake=0.5
    // force=500 -> intensity=1.0 -> shake = 0.5*1.0 = 0.5
    GameHookSystem sys;
    GameHookOutput outputs[16];

    ImpactEvent evt = make_impact(500.0f, 10.0f, 2);
    uint32_t count = sys.process_impacts(&evt, 1, &registry, outputs, 16);

    bool found = false;
    float shake_intensity = 0.0f;
    for (uint32_t i = 0; i < count; ++i) {
        if (outputs[i].type == HookEffectType::CAMERA_SHAKE) {
            found = true;
            shake_intensity = outputs[i].float_param;
            assert(shake_intensity > 0.4f && "Shake > 0.4");
            assert(shake_intensity <= 1.0f && "Shake capped at 1.0");
            assert(outputs[i].profile_id == 2 && "Profile id matches");
            break;
        }
    }
    assert(found && "CAMERA_SHAKE output must exist");

    std::printf("    [PASS] CAMERA_SHAKE generated: intensity=%.2f\n",
               shake_intensity);
    return 0;
}

// =============================================================================
// TEST 5: process_impacts: time-scale — slow-motion on heavy impact
// =============================================================================
static int test_process_time_scale() {
    std::printf("  [Test 5] process_impacts: time-scale...\n");

    ProfileRegistry registry;
    registry.setup_defaults();

    // Custom profile with time-scale enabled
    ImpactStyleProfile ts_prof = make_test_profile();
    // time_scale_on_impact=0.3, time_scale_duration_ms=300
    // light=5, heavy=20, critical=50
    uint16_t ts_id = registry.add(ts_prof);

    GameHookSystem sys;
    GameHookOutput outputs[16];

    // force=100 -> above critical -> intensity=1.0 -> passes >0.5 threshold
    ImpactEvent evt = make_impact(100.0f, 15.0f, ts_id);
    uint32_t count = sys.process_impacts(&evt, 1, &registry, outputs, 16);

    bool found = false;
    for (uint32_t i = 0; i < count; ++i) {
        if (outputs[i].type == HookEffectType::TIME_SCALE) {
            found = true;
            // float_param = scale factor = 0.3
            assert(std::abs(outputs[i].float_param - 0.3f) < 0.01f &&
                   "Time scale factor = 0.3");
            // float_param2 = duration * intensity = 300*1.0 = 300 (cap 500ms)
            assert(outputs[i].float_param2 > 290.0f &&
                   "Duration > 290ms");
            assert(outputs[i].float_param2 <= 500.0f &&
                   "Duration capped at max");
            break;
        }
    }
    assert(found && "TIME_SCALE output must exist");

    std::printf("    [PASS] TIME_SCALE generated: scale=%.2f, duration=%.0fms\n",
               0.3f, 300.0f);
    return 0;
}

// =============================================================================
// TEST 6: process_impacts: blend override — auto-ragdoll on critical impact
// =============================================================================
static int test_process_blend_override() {
    std::printf("  [Test 6] process_impacts: blend override (auto-ragdoll)...\n");

    ProfileRegistry registry;
    registry.setup_defaults();

    // Heavy profile (id=2): force=500 -> intensity=1.0 > auto_ragdoll_threshold(0.8)
    GameHookSystem sys;
    GameHookOutput outputs[16];

    ImpactEvent evt = make_impact(500.0f, 10.0f, 2);
    uint32_t count = sys.process_impacts(&evt, 1, &registry, outputs, 16);

    bool found = false;
    for (uint32_t i = 0; i < count; ++i) {
        if (outputs[i].type == HookEffectType::BLEND_OVERRIDE) {
            found = true;
            assert(outputs[i].target_body == 1 &&
                   "Blend override targets receiver");
            assert(outputs[i].blend_override.new_mode ==
                       PhysicsBlendMode::PHYSICS_DRIVEN &&
                   "Auto-ragdoll = PHYSICS_DRIVEN");
            assert(outputs[i].blend_override.bone_index == 0xFFFFFFFF &&
                   "All bones targeted");
            assert(outputs[i].float_param > 0.0f &&
                   "Duration should be positive");
            assert(outputs[i].blend_override.duration_ms > 0.0f &&
                   "Blend override duration positive");
            break;
        }
    }
    assert(found && "BLEND_OVERRIDE output must exist");

    std::printf("    [PASS] BLEND_OVERRIDE generated: body=%u, mode=PHYSICS_DRIVEN\n",
               (unsigned)1);
    return 0;
}

// =============================================================================
// TEST 7: process_impacts: sound trigger — any impact above threshold
// =============================================================================
static int test_process_sound_trigger() {
    std::printf("  [Test 7] process_impacts: sound trigger...\n");

    ProfileRegistry registry;
    registry.setup_defaults();

    // Light profile (id=1): light=1, heavy=5, critical=15
    // force=3 -> intensity = 0.5*(3-1)/(5-1) = 0.25 (>0.1 threshold)
    GameHookSystem sys;
    GameHookOutput outputs[16];

    ImpactEvent evt = make_impact(3.0f, 5.0f, 1);
    uint32_t count = sys.process_impacts(&evt, 1, &registry, outputs, 16);

    bool found = false;
    float sound_intensity = 0.0f;
    for (uint32_t i = 0; i < count; ++i) {
        if (outputs[i].type == HookEffectType::SOUND_TRIGGER) {
            found = true;
            sound_intensity = outputs[i].float_param;
            assert(sound_intensity > 0.0f && "Sound intensity positive");
            assert(sound_intensity <= 1.0f && "Sound intensity <= 1.0");
            assert(outputs[i].impact_intensity > 0.0f &&
                   "Impact intensity on sound output");
            break;
        }
    }
    assert(found && "SOUND_TRIGGER output must exist");

    std::printf("    [PASS] SOUND_TRIGGER generated: intensity=%.2f\n",
               sound_intensity);
    return 0;
}

// =============================================================================
// TEST 8: GameHookAccumulator: hit-stop tick — decreases, expires
// =============================================================================
static int test_accumulator_hit_stop() {
    std::printf("  [Test 8] GameHookAccumulator: hit-stop tick...\n");

    GameHookAccumulator acc;

    // Feed a HIT_STOP with 60ms duration
    GameHookOutput hit_out;
    hit_out.reset();
    hit_out.type         = HookEffectType::HIT_STOP;
    hit_out.float_param  = 60.0f;

    float dt = 1.0f / 60.0f; // ~16.67ms per frame
    acc.update(&hit_out, 1, dt);

    float remaining = acc.get_hit_stop_ms();
    assert(remaining > 0.0f && "Hit-stop should be active");
    assert(remaining < 60.0f && "Hit-stop should have decreased");
    assert(std::abs(remaining - (60.0f - dt * 1000.0f)) < 0.1f &&
           "Decrease matches dt");
    assert(acc.has_hit_stop() && "has_hit_stop() should be true");

    // Tick until expired (~60ms / 16.67ms ≈ 4 frames total)
    for (int i = 0; i < 10; ++i) {
        acc.update(nullptr, 0, dt);
    }

    assert(!acc.has_hit_stop() && "Hit-stop should have expired");
    assert(acc.get_hit_stop_ms() < 0.001f && "Remaining near 0");

    std::printf("    [PASS] Hit-stop 60ms -> 0 over ~4 frames\n");
    return 0;
}

// =============================================================================
// TEST 9: GameHookAccumulator: time-scale — active scale returned, decays
// =============================================================================
static int test_accumulator_time_scale() {
    std::printf("  [Test 9] GameHookAccumulator: time-scale...\n");

    GameHookAccumulator acc;

    // Feed TIME_SCALE: scale=0.3, duration=200ms
    GameHookOutput ts_out;
    ts_out.reset();
    ts_out.type         = HookEffectType::TIME_SCALE;
    ts_out.float_param  = 0.3f;
    ts_out.float_param2 = 200.0f;

    float dt = 1.0f / 60.0f;
    acc.update(&ts_out, 1, dt);

    float scale = acc.get_time_scale();
    assert(std::abs(scale - 0.3f) < 0.01f && "Time scale should be 0.3");

    // Tick many frames to expire (200ms / 16.67ms ≈ 12 frames + initial)
    for (int i = 0; i < 20; ++i) {
        acc.update(nullptr, 0, dt);
    }

    scale = acc.get_time_scale();
    assert(std::abs(scale - 1.0f) < 0.01f &&
           "Time scale should return to 1.0 after expiry");

    std::printf("    [PASS] Time-scale 0.3 -> expired -> 1.0\n");
    return 0;
}

// =============================================================================
// TEST 10: GameHookAccumulator: camera shake decay
// =============================================================================
static int test_accumulator_camera_shake() {
    std::printf("  [Test 10] GameHookAccumulator: camera shake decay...\n");

    GameHookAccumulator acc;

    // Feed CAMERA_SHAKE: intensity=0.6
    GameHookOutput shake_out;
    shake_out.reset();
    shake_out.type        = HookEffectType::CAMERA_SHAKE;
    shake_out.float_param = 0.6f;

    float dt = 1.0f / 60.0f;
    acc.update(&shake_out, 1, dt);

    // Initial shake = intensity * decay(1.0) = 0.6
    float shake0 = acc.get_camera_shake();
    assert(shake0 > 0.5f && "Initial shake should be near 0.6");

    // Tick 5 frames — decay should be noticeable
    for (int i = 0; i < 5; ++i) {
        acc.update(nullptr, 0, dt);
    }
    float shake5 = acc.get_camera_shake();
    assert(shake5 < shake0 && "Shake should decrease over time");

    // Tick until fully decayed (~0.33s ≈ 20 frames total)
    for (int i = 0; i < 25; ++i) {
        acc.update(nullptr, 0, dt);
    }
    float shake_final = acc.get_camera_shake();
    assert(shake_final < 0.001f && "Shake should be fully decayed");

    std::printf("    [PASS] Camera shake: %.2f -> %.2f -> %.4f\n",
               shake0, shake5, shake_final);
    return 0;
}

// =============================================================================
// TEST 11: Multiple effects for single impact
// =============================================================================
static int test_multiple_effects() {
    std::printf("  [Test 11] Multiple effects for single impact...\n");

    ProfileRegistry registry;
    registry.setup_defaults();

    // Custom profile with ALL hooks enabled
    ImpactStyleProfile full_prof = make_test_profile();
    uint16_t full_id = registry.add(full_prof);

    GameHookSystem sys;
    GameHookOutput outputs[32];

    // Critical impact: force=100 -> intensity=1.0
    ImpactEvent evt = make_impact(100.0f, 15.0f, full_id);
    uint32_t count = sys.process_impacts(&evt, 1, &registry, outputs, 32);

    // Tally each effect type
    int hit_stops = 0, shakes = 0, time_scales = 0;
    int sounds = 0, blends = 0;

    for (uint32_t i = 0; i < count; ++i) {
        switch (outputs[i].type) {
        case HookEffectType::HIT_STOP:       hit_stops++;   break;
        case HookEffectType::CAMERA_SHAKE:   shakes++;      break;
        case HookEffectType::TIME_SCALE:     time_scales++; break;
        case HookEffectType::SOUND_TRIGGER:  sounds++;     break;
        case HookEffectType::BLEND_OVERRIDE: blends++;     break;
        default: break;
        }
    }

    assert(hit_stops   == 1 && "Exactly 1 hit-stop");
    assert(shakes      == 1 && "Exactly 1 camera shake");
    assert(time_scales == 1 && "Exactly 1 time-scale");
    assert(sounds      == 1 && "Exactly 1 sound trigger");
    assert(blends      == 1 && "Exactly 1 blend override");
    assert(count       == 5 && "Exactly 5 total outputs");

    std::printf("    [PASS] Single impact produced %u effects: "
               "hit_stop=%d shake=%d time_scale=%d sound=%d blend=%d\n",
               count, hit_stops, shakes, time_scales, sounds, blends);
    return 0;
}

// =============================================================================
// TEST 12: Game hooks determinism — same events produce same outputs
// =============================================================================
static int test_determinism() {
    std::printf("  [Test 12] Game hooks determinism...\n");

    auto run_hooks = []() -> uint64_t {
        ProfileRegistry registry;
        registry.setup_defaults();

        ImpactStyleProfile test_prof = make_test_profile();
        uint16_t test_id = registry.add(test_prof);

        GameHookSystem sys;
        GameHookOutput outputs[32];

        // Three impacts with different forces and timestamps
        ImpactEvent events[3];
        events[0] = make_impact(100.0f, 15.0f, test_id, 0.016f);
        events[1] = make_impact(30.0f,  8.0f,  test_id, 0.033f);
        events[2] = make_impact(500.0f, 20.0f, test_id, 0.050f);

        uint32_t count = sys.process_impacts(events, 3, &registry, outputs, 32);

        // Hash all outputs
        uint64_t hash = 0;
        auto hash_u32 = [&hash](uint32_t v) {
            hash ^= v + 0x9e3779b97f4a7c15ULL + (hash << 12) + (hash >> 4);
        };

        // Hash count
        hash_u32(count);

        for (uint32_t i = 0; i < count; ++i) {
            uint32_t bits;
            std::memcpy(&bits, &outputs[i].type, sizeof(bits));
            hash_u32(bits);
            std::memcpy(&bits, &outputs[i].float_param, sizeof(bits));
            hash_u32(bits);
            std::memcpy(&bits, &outputs[i].float_param2, sizeof(bits));
            hash_u32(bits);
            std::memcpy(&bits, &outputs[i].target_body, sizeof(bits));
            hash_u32(bits);
            std::memcpy(&bits, &outputs[i].impact_intensity, sizeof(bits));
            hash_u32(bits);
            std::memcpy(&bits, &outputs[i].profile_id, sizeof(bits));
            hash_u32(bits);
        }
        return hash;
    };

    uint64_t h1 = run_hooks();
    uint64_t h2 = run_hooks();

    if (h1 == h2) {
        std::printf("    [PASS] Deterministic (hash=%016llx)\n",
                    (unsigned long long)h1);
        return 0;
    } else {
        std::printf("    [FAIL] Non-deterministic (h1=%016llx, h2=%016llx)\n",
                    (unsigned long long)h1, (unsigned long long)h2);
        return 1;
    }
}

// =============================================================================
// Main
// =============================================================================
int main() {
    apc::enforce_deterministic_fp_mode();
    std::printf("Running Sprint 11 Tests: Game Hooks System\n");
    std::printf("============================================\n");

    int result = 0;
    result |= test_hook_system_construction();
    result |= test_classify_impact();
    result |= test_process_hit_stop();
    result |= test_process_camera_shake();
    result |= test_process_time_scale();
    result |= test_process_blend_override();
    result |= test_process_sound_trigger();
    result |= test_accumulator_hit_stop();
    result |= test_accumulator_time_scale();
    result |= test_accumulator_camera_shake();
    result |= test_multiple_effects();
    result |= test_determinism();

    if (result == 0) {
        std::printf("\nAll Sprint 11 tests PASSED.\n");
    } else {
        std::printf("\nSome Sprint 11 tests FAILED.\n");
    }
    return result;
}
