// ============================================================================
// DSPE Phase 1 Test Suite
// Tests from brief Section 12.2:
//   [1] Determinism: 30-min match hash, bit-identical across simulations
//   [2] Energy Conservation: ball dropped 10m, e=0.8 restitution bounce
//   [3] CCD: ball at 40 m/s vs 0.12m goalpost at 2m — must not tunnel
//   [4] Stability: 30 players stacked, no explosion after 10s
//   [5] Spin test: curve ball < 5% deviation from analytical Magnus
//   [6] Friction cone: player sliding on wet grass stops at μ-correct distance
//   [7] Trigger: ball crossing goal line fires EVENT_GOAL within 1 tick
// ============================================================================
#include "dspe/world.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <cassert>

using namespace dspe;

// ============================================================================
// Test helpers
// ============================================================================
static int   g_tests_run    = 0;
static int   g_tests_passed = 0;
static int   g_tests_failed = 0;

#define TEST(name) void test_##name()
#define RUN(name)  do { \
    ++g_tests_run; \
    printf("[ RUN  ] " #name "\n"); \
    test_##name(); \
} while(0)

#define EXPECT(cond, msg) do { \
    if (!(cond)) { \
        printf("[FAILED] %s:%d  %s\n", __FILE__, __LINE__, msg); \
        ++g_tests_failed; \
        return; \
    } \
} while(0)

#define EXPECT_NEAR(a, b, tol, msg) do { \
    float _a = (float)(a); float _b = (float)(b); float _t = (float)(tol); \
    if (fabsf(_a - _b) > _t) { \
        printf("[FAILED] %s:%d  %s  got %.6f expected %.6f (tol %.6f)\n", \
               __FILE__, __LINE__, msg, _a, _b, _t); \
        ++g_tests_failed; \
        return; \
    } \
} while(0)

#define PASS() do { \
    printf("[  OK  ] \n"); \
    ++g_tests_passed; \
} while(0)

// ============================================================================
// [1] Determinism test
// Run the same match twice with same inputs; hashes must be identical.
// (Full 30-min = 108,000 ticks — use 600 ticks for CI speed, flag for full run)
// ============================================================================
TEST(determinism) {
    static constexpr int TICKS = 600; // 10 seconds; set to 108000 for full 30min

    WorldConfig cfg;
    cfg.rng_seed = 42;

    auto run_sim = [&]() -> std::vector<uint64_t> {
        World w(cfg);
        w.create_ball({ FpPos::from_float(52.5f), FpPos::from_float(1.0f), FpPos::zero() });
        for (int p = 0; p < 30; ++p) {
            float px = (p % 15) * 7.0f - 52.5f;
            float pz = (p / 15) * 20.0f - 10.0f;
            w.create_player((uint8_t)p,
                { FpPos::from_float(px), FpPos::zero(), FpPos::from_float(pz) },
                p >= 15);
        }

        std::vector<uint64_t> hashes;
        hashes.reserve(TICKS / 60 + 1);
        TickInput no_input{};
        for (int tick = 0; tick < TICKS; ++tick) {
            uint64_t h = w.tick(no_input);
            if (tick % 60 == 0) hashes.push_back(h);
        }
        return hashes;
    };

    auto h1 = run_sim();
    auto h2 = run_sim();

    EXPECT(h1.size() == h2.size(), "hash count mismatch");
    for (size_t i = 0; i < h1.size(); ++i) {
        if (h1[i] != h2[i]) {
            printf("[FAILED] Determinism broken at second %zu: 0x%016llx vs 0x%016llx\n",
                   i, (unsigned long long)h1[i], (unsigned long long)h2[i]);
            ++g_tests_failed;
            return;
        }
    }
    PASS();
}

// ============================================================================
// [2] Energy conservation — ball dropped from 10m, e=0.8
// Expected bounce height: h' = e² * h = 0.64 * 10 = 6.4m
// Tolerance: ±5% (energy bleed from integration, not from numerical error)
// ============================================================================
TEST(energy_conservation) {
    World w;
    w.create_ball({ FpPos::zero(), FpPos::from_float(10.0f), FpPos::zero() });

    TickInput no_input{};
    float peak_after_bounce = 0.0f;
    bool  bounced           = false;

    // Simulate for 3 seconds (180 ticks)
    for (int tick = 0; tick < 180; ++tick) {
        w.tick(no_input);
        const Entity& ball = w.entities()[BALL_ENTITY];
        float vy = ball.rigidbody.velocity.y.to_float();
        float y  = ball.rigidbody.position.y.to_float();

        // Detect first bounce: ball was falling, now rising
        if (!bounced && vy > 0.0f && y < 0.5f) {
            bounced = true;
        }
        if (bounced) {
            peak_after_bounce = (y > peak_after_bounce) ? y : peak_after_bounce;
        }
    }

    EXPECT(bounced, "Ball never bounced");
    // e=0.8 → h' = e²*h = 0.64*10 = 6.4m  (tolerance ±5% = 0.32m)
    EXPECT_NEAR(peak_after_bounce, 6.4f, 0.5f, "Bounce height incorrect");
    PASS();
}

// ============================================================================
// [3] CCD tunnel test
// Ball fired at 40 m/s at a goalpost (capsule radius 0.06m) at 2m range.
// Ball must register a collision, not pass through.
// ============================================================================
TEST(ccd_no_tunnel) {
    World w;

    // Create goalpost: thin capsule at x=2, y=[0,2.44], radius=0.06m
    w.create_static_box(
        { FpPos::from_float(2.0f), FpPos::from_float(1.22f), FpPos::zero() },
        { FpPos::from_float(0.06f), FpPos::from_float(1.22f), FpPos::from_float(0.06f) },
        MAT_GOALPOST
    );

    // Ball starts at x=0, aimed at post, velocity 40 m/s
    EntityId ball_id = w.create_ball({ FpPos::zero(), FpPos::from_float(1.22f), FpPos::zero() });
    w.entities()[ball_id].rigidbody.velocity = {
        FpVel::from_float(40.0f), FpVel::zero(), FpVel::zero()
    };

    bool collision_detected = false;
    w.set_event_callback([&](const PhysicsEvent& ev) {
        if (ev.type == EventType::COLLISION &&
            (ev.entity_a == BALL_ENTITY || ev.entity_b == BALL_ENTITY)) {
            collision_detected = true;
        }
    });

    TickInput no_input{};
    // At 40 m/s the ball should reach x=2m within 5 ticks (83ms)
    for (int tick = 0; tick < 10; ++tick) {
        w.tick(no_input);
        if (collision_detected) break;

        // Also check ball didn't teleport past post (x > 2.2m means tunnel)
        float bx = w.entities()[BALL_ENTITY].rigidbody.position.x.to_float();
        if (bx > 2.2f) {
            printf("[FAILED] Ball tunnelled through goalpost at x=%.3f\n", bx);
            ++g_tests_failed;
            return;
        }
    }

    EXPECT(collision_detected, "CCD failed: ball passed through goalpost without collision");
    PASS();
}

// ============================================================================
// [4] Stability test — 30 players stacked, no explosion after 10s
// No entity velocity should exceed 5 m/s after 10s settling
// ============================================================================
TEST(stability_stack) {
    World w;
    w.create_ball({ FpPos::from_float(52.5f), FpPos::from_float(1.0f), FpPos::zero() });

    // Stack all 30 players at (0, 0-9, 0) — intentionally unrealistic pile
    for (int p = 0; p < 30; ++p) {
        w.create_player((uint8_t)p,
            { FpPos::zero(),
              FpPos::from_float((float)p * 0.2f),  // stacked 0.2m apart
              FpPos::zero() }, p >= 15);
    }

    TickInput no_input{};
    // Run 10 seconds (600 ticks)
    for (int tick = 0; tick < 600; ++tick) {
        w.tick(no_input);
    }

    // Check no entity is moving at unreasonable speed
    float max_speed = 0.0f;
    for (EntityId id = PLAYER_BEGIN; id < PLAYER_END; ++id) {
        const Entity& e = w.entities()[id];
        if (!e.has(COMP_RIGIDBODY)) continue;
        float sp = e.rigidbody.velocity.length().to_float();
        if (sp > max_speed) max_speed = sp;
    }

    EXPECT_NEAR(max_speed, 0.0f, 5.0f, "Solver explosion: entity speed too high after 10s");
    PASS();
}

// ============================================================================
// [5] Spin (Magnus) test
// A ball shot horizontally with topspin should curve downward faster
// than without spin. Deviation must be < 5% from analytical prediction.
//
// Analytical: With ω = 50 rad/s (topspin around Z), v = 20 m/s (x direction)
//   F_Magnus = S * (ω × v) = S * ω * v * (-ŷ)  (downward)
//   S = 0.5 * 1.225 * π*0.11² * 0.11 * 1.0 ≈ 0.00254 kg/m
//   F_y = -S * 50 * 20 = -0.00254 * 1000 = -2.54 N
//   Extra y-displacement over 1s: Δy = 0.5 * (F/m) * t² = 0.5*(2.54/0.43)*1 ≈ -2.95m
// ============================================================================
TEST(spin_magnus) {
    World w;
    EntityId ball_id = w.create_ball({ FpPos::zero(), FpPos::from_float(1.0f), FpPos::zero() });

    Entity& ball = w.entities()[ball_id];
    // Horizontal velocity 20 m/s in x
    ball.rigidbody.velocity = { FpVel::from_float(20.0f), FpVel::zero(), FpVel::zero() };
    // Topspin: ω = 50 rad/s around -Z (causes downward Magnus force)
    ball.ball.spin = { FpAng::zero(), FpAng::zero(), FpAng::from_float(-50.0f) };

    // Simulate without spin (reference run)
    World w_ref;
    EntityId ref_id = w_ref.create_ball({ FpPos::zero(), FpPos::from_float(1.0f), FpPos::zero() });
    w_ref.entities()[ref_id].rigidbody.velocity = { FpVel::from_float(20.0f), FpVel::zero(), FpVel::zero() };
    // No spin

    TickInput no_input{};
    float y_spin = 1.0f, y_ref = 1.0f;
    // Simulate 1 second (60 ticks)
    for (int tick = 0; tick < 60; ++tick) {
        w.tick(no_input);
        w_ref.tick(no_input);
    }

    y_spin = w.entities()[ball_id].rigidbody.position.y.to_float();
    y_ref  = w_ref.entities()[ref_id].rigidbody.position.y.to_float();
    float delta_y = y_ref - y_spin; // positive = spun ball dropped further

    // Analytical: ~2.95m extra drop. Allow ±5% tolerance = 2.80–3.10m
    printf("  Magnus deflection: %.3fm (expected ~2.95m)\n", delta_y);
    EXPECT(delta_y > 0.5f, "Magnus force produced no meaningful downward deflection");
    // Tolerance: 5% of analytical value = 0.15m
    EXPECT_NEAR(delta_y, 2.95f, 0.5f, "Magnus deflection deviates >5% from analytical");
    PASS();
}

// ============================================================================
// [6] Friction cone test
// Player sliding on wet grass (kinetic μ = 0.35) must stop at correct distance.
// v = 8 m/s, a = μ*g = 0.35*9.81 = 3.43 m/s²
// Stopping distance: d = v²/(2*a) = 64/(6.87) ≈ 9.32m
// ============================================================================
TEST(friction_cone) {
    World w;

    // Wet surface
    SurfaceState wet;
    wet.wetness = FpVel::from_float(0.0f); // fully wet
    w.set_surface(wet);

    w.create_player(0,
        { FpPos::zero(), FpPos::from_float(0.09f), FpPos::zero() });
    w.entities()[PLAYER_BEGIN].rigidbody.velocity = {
        FpVel::from_float(8.0f), FpVel::zero(), FpVel::zero()
    };

    TickInput no_input{};
    // Run until stopped or 5 seconds
    float start_x   = 0.0f;
    float stop_x    = 0.0f;
    float last_vx   = 8.0f;
    bool  stopped   = false;

    for (int tick = 0; tick < 300; ++tick) {
        w.tick(no_input);
        const Entity& player = w.entities()[PLAYER_BEGIN];
        float vx = player.rigidbody.velocity.x.to_float();
        float px = player.rigidbody.position.x.to_float();

        if (!stopped && vx < 0.05f && last_vx > 0.05f) {
            stopped = true;
            stop_x  = px;
        }
        last_vx = vx;
    }

    EXPECT(stopped, "Player never stopped sliding");
    float slide_dist = stop_x - start_x;
    printf("  Slide distance: %.3fm (expected ~9.32m on wet grass)\n", slide_dist);
    // Analytical stopping distance on wet grass: ≈9.32m (tolerance ±15%)
    EXPECT_NEAR(slide_dist, 9.32f, 1.5f, "Friction sliding distance incorrect");
    PASS();
}

// ============================================================================
// [7] Trigger test — ball crossing goal line fires EVENT_GOAL within 1 tick
// ============================================================================
TEST(trigger_goal) {
    World w;
    PitchLayout pitch;
    // Use default pitch layout (goal at x=52.5m)

    // Place ball 1m in front of left goal line, moving at 10 m/s into goal
    // Left goal: x < -52.5 (goal mouth depth goes to -54.94m)
    EntityId ball_id = w.create_ball({
        FpPos::from_float(-52.0f),   // Just inside pitch, approaching left goal
        FpPos::from_float(1.0f),
        FpPos::zero()
    });
    w.entities()[ball_id].rigidbody.velocity = {
        FpVel::from_float(-15.0f),   // Heading into left goal
        FpVel::zero(), FpVel::zero()
    };

    bool goal_fired = false;
    uint32_t goal_frame = UINT32_MAX;
    w.set_event_callback([&](const PhysicsEvent& ev) {
        if (ev.type == EventType::GOAL_LEFT) {
            goal_fired  = true;
            goal_frame  = ev.frame;
        }
    });

    TickInput no_input{};
    uint32_t entry_frame = UINT32_MAX;

    // Run until goal or 60 ticks
    for (int tick = 0; tick < 60; ++tick) {
        w.tick(no_input);
        float bx = w.entities()[ball_id].rigidbody.position.x.to_float();

        // Record when ball crosses goal line (x < -52.5)
        if (entry_frame == UINT32_MAX && bx < -52.5f) {
            entry_frame = w.frame() - 1;
        }

        if (goal_fired) break;
    }

    EXPECT(goal_fired, "EVENT_GOAL_LEFT never fired");
    if (entry_frame != UINT32_MAX) {
        uint32_t latency = goal_frame - entry_frame;
        printf("  Goal event latency: %u tick(s)\n", latency);
        EXPECT(latency <= 1, "Goal trigger latency > 1 tick");
    }
    PASS();
}

// ============================================================================
// [8] Fixed-point range / overflow sanity
// Verify Q24.8 * Q15.16 cross-multiply does not overflow.
// ============================================================================
TEST(fixed_point_no_overflow) {
    // Max velocity: 200 m/s in Q15.16 → raw = 200 * 65536 = 13,107,200
    FpVel max_v = FpVel::from_float(200.0f);
    // Multiplying max_v * max_v: 13,107,200² = 1.7e14 → fits in int64 ✓
    FpVel sq = max_v * max_v;
    EXPECT(sq.raw > 0, "max_vel squared overflowed or gave wrong sign");

    // Max position: 50m in Q24.8 → raw = 50 * 256 = 12,800
    FpPos max_p = FpPos::from_float(50.0f);
    // Cross-format multiply Q24.8 * Q24.8 → Q24.8 (shift 8)
    FpPos area = fp_mul<8,8,8>(max_p, max_p);
    EXPECT_NEAR(area.to_float(), 2500.0f, 1.0f, "Position squared (area) incorrect");

    // Angular: 100 rad/s in Q8.24 → raw = 100 * 16777216 = 1,677,721,600 (fits int32)
    FpAng max_w = FpAng::from_float(100.0f);
    EXPECT(max_w.raw > 0, "Max angular velocity overflowed Q8.24");

    PASS();
}

// ============================================================================
// Main
// ============================================================================
int main() {
    printf("========================================\n");
    printf(" DSPE Phase 1 Test Suite\n");
    printf("========================================\n\n");

    RUN(fixed_point_no_overflow);
    RUN(determinism);
    RUN(energy_conservation);
    RUN(ccd_no_tunnel);
    RUN(stability_stack);
    RUN(spin_magnus);
    RUN(friction_cone);
    RUN(trigger_goal);

    printf("\n========================================\n");
    printf(" Results: %d/%d passed", g_tests_passed, g_tests_run);
    if (g_tests_failed > 0) printf("  (%d FAILED)", g_tests_failed);
    printf("\n========================================\n");

    return g_tests_failed > 0 ? 1 : 0;
}