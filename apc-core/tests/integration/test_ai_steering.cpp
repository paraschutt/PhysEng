// =============================================================================
// Integration Test: AI Steering Behaviors — Velocity Math & Flocking
// =============================================================================
//
// Validates the steering behaviors we fixed in Step 1 (9b):
//   1.  Pursue look-ahead predicts correct intercept point
//   2.  Pursue dynamic look-ahead proportional to distance
//   3.  Pursue caps look-ahead to max_prediction_time
//   4.  Evade produces force away from predicted threat
//   5.  Evade returns zero when threat is outside panic radius
//   6.  Alignment averages velocity vectors correctly (perpendicular)
//   7.  Alignment skips zero-velocity neighbors
//   8.  Alignment returns zero when all neighbors are stationary
//   9.  Alignment clamps to max_force
//   10. Separation strength inversely proportional to distance
//   11. Cohesion seeks centroid of neighbors
//   12. Blend of pursue + separation (real steering outputs)
//   13. Blend of flee + cohesion (real steering outputs)
//   14. Seek + arrive produce different results at close range
//   15. Pursue vs seek: pursue must deviate when target moves laterally
//
// Pattern: int main() + assert(), no test framework.
// =============================================================================

#include "apc_ai/apc_ai_steering.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_math_common.h"
#include <cassert>
#include <cstdlib>
#include <cstdio>
#include <cmath>

static constexpr float EPS = 1e-3f;
static int g_assertions = 0;

static bool approx_eq(float a, float b, float eps = EPS) {
    return std::abs(a - b) < eps;
}

static void check(bool cond, const char* msg) {
    ++g_assertions;
    if (!cond) {
        std::printf("    [FAIL] %s (assertion #%d)\n", msg, g_assertions);
        std::fflush(stdout);
        std::abort();
    }
}

// =============================================================================
// TEST 1: Pursue look-ahead predicts correct intercept point
// =============================================================================
static void test_pursue_intercept_trajectory() {
    std::printf("  [Test 1] Pursue look-ahead predicts correct intercept point...\n");

    apc::Vec3 pos(0, 0, 0);
    apc::Vec3 vel(0, 0, 0);
    apc::Vec3 target_pos(0, 0, 10);   // 10m away on Z
    apc::Vec3 target_vel(5, 0, 0);    // Moving at 5 m/s on X
    float max_speed = 10.0f;
    float max_pred  = 2.0f;

    // look_ahead = distance / max_speed = 10 / 10 = 1.0s
    // future_pos = (0,0,10) + (5,0,0)*1.0 = (5,0,10)
    // pursue delegates to seek(pos, (5,0,10), max_speed)
    // desired = normalize(5,0,10) * 10 = (10/sqrt(125), 0, 20/sqrt(125))
    apc::SteeringOutput out = apc::SteeringSystem::pursue(
        pos, vel, target_pos, target_vel, max_speed, max_pred);

    // Direction must match toward (5, 0, 10)
    float mag = apc::Vec3::length(out.linear);
    check(mag > 0.0f, "pursue: non-zero output");
    check(out.linear.x > 0.0f, "pursue: positive X component (leading target)");
    check(out.linear.z > 0.0f, "pursue: positive Z component (still approaching)");
    check(approx_eq(out.linear.y, 0.0f), "pursue: Y = 0 (XZ plane)");

    // Verify direction angle: atan2(z, x) should be close to atan2(10, 5)
    float expected_angle = std::atan2(10.0f, 5.0f);
    float actual_angle   = std::atan2(out.linear.z, out.linear.x);
    check(approx_eq(actual_angle, expected_angle, 0.01f),
          "pursue: direction angle matches predicted intercept");

    std::printf("    [PASS] Pursue look-ahead calculates correct intercept trajectory\n");
}

// =============================================================================
// TEST 2: Pursue dynamic look-ahead proportional to distance
// =============================================================================
static void test_pursue_dynamic_lookahead() {
    std::printf("  [Test 2] Pursue dynamic look-ahead proportional to distance...\n");

    apc::Vec3 vel(0, 0, 0);
    apc::Vec3 target_vel(3, 0, 0); // Moving at 3 m/s on X
    float max_speed = 6.0f;
    float max_pred  = 5.0f;

    // Close target: distance=6, look_ahead=6/6=1.0s, future=(3,0,0)+(3,0,0)*1=(6,0,0)
    apc::Vec3 pos_close(0, 0, 0);
    apc::Vec3 tgt_close(3, 0, 3);
    apc::SteeringOutput out_close = apc::SteeringSystem::pursue(
        pos_close, vel, tgt_close, target_vel, max_speed, max_pred);
    // Close → small look-ahead → slight X lead
    check(out_close.linear.x > 0.0f, "close pursue: leads target in X");

    // Far target: distance=30, look_ahead=30/6=5.0s, future=(0,0,30)+(3,0,0)*5=(15,0,30)
    apc::Vec3 pos_far(0, 0, 0);
    apc::Vec3 tgt_far(0, 0, 30);
    apc::SteeringOutput out_far = apc::SteeringSystem::pursue(
        pos_far, vel, tgt_far, target_vel, max_speed, max_pred);
    check(out_far.linear.x > 0.0f, "far pursue: leads target in X");

    // Close target: look_ahead≈0.707s. Future=(3,0,3)+(3,0,0)*0.707≈(5.12,0,3).
    // Direction has angle atan2(3,5.12)=30° from X axis.
    // Far target: look_ahead=5.0s (capped to max_pred=5.0). Future=(0,0,30)+(3,0,0)*5=(15,0,30).
    // Direction from origin to (15,0,30) has angle atan2(30,15)=63° from X axis.
    // Far target has proportionally MORE Z lead (less X fraction) due to the large Z distance.
    float close_x_ratio = out_close.linear.x / (apc::Vec3::length(out_close.linear) + EPS);
    float far_x_ratio   = out_far.linear.x / (apc::Vec3::length(out_far.linear) + EPS);
    check(close_x_ratio > far_x_ratio, "close pursue: proportionally larger X lead (closer target)");
    // Far target has larger absolute predicted X offset in world space
    check(out_far.linear.x > 0.1f, "far pursue: meaningful absolute X offset");

    std::printf("    [PASS] Dynamic look-ahead scales with distance\n");
}

// =============================================================================
// TEST 3: Pursue caps look-ahead to max_prediction_time
// =============================================================================
static void test_pursue_caps_prediction() {
    std::printf("  [Test 3] Pursue caps look-ahead to max_prediction_time...\n");

    apc::Vec3 pos(0, 0, 0);
    apc::Vec3 vel(0, 0, 0);
    apc::Vec3 target_pos(0, 0, 100); // Very far
    apc::Vec3 target_vel(10, 0, 0);
    float max_speed = 5.0f;
    float max_pred  = 0.5f; // Very short prediction

    // Without cap: look_ahead = 100/5 = 20s, future=(0,0,100)+(10,0,0)*20=(200,0,100)
    // With cap:    look_ahead = 0.5s,      future=(0,0,100)+(10,0,0)*0.5=(5,0,100)
    apc::SteeringOutput out = apc::SteeringSystem::pursue(
        pos, vel, target_pos, target_vel, max_speed, max_pred);

    float mag = apc::Vec3::length(out.linear);
    check(mag > 0.0f, "capped pursue: non-zero output");

    // Verify direction is toward (5, 0, 100), not (200, 0, 100)
    float actual_angle = std::atan2(out.linear.z, out.linear.x);
    float capped_angle = std::atan2(100.0f, 5.0f);
    check(approx_eq(actual_angle, capped_angle, 0.05f),
          "capped pursue: direction uses max_prediction_time");

    std::printf("    [PASS] Pursue respects max_prediction_time cap\n");
}

// =============================================================================
// TEST 4: Evade produces force away from predicted threat
// =============================================================================
static void test_evade_predicted_threat() {
    std::printf("  [Test 4] Evade produces force away from predicted threat...\n");

    apc::Vec3 pos(0, 0, 0);
    apc::Vec3 vel(0, 0, 0);
    apc::Vec3 threat_pos(10, 0, 0);
    apc::Vec3 threat_vel(-5, 0, 0); // Threat approaching
    float max_speed = 8.0f;
    float max_pred  = 2.0f;

    apc::SteeringOutput out = apc::SteeringSystem::evade(
        pos, vel, threat_pos, threat_vel, max_speed, max_pred);

    float mag = apc::Vec3::length(out.linear);
    check(mag > 0.0f, "evade: non-zero output");
    // Threat at (10,0,0) moving toward us → predicted threat is closer → flee away
    check(out.linear.x < 0.0f, "evade: pushes away from threat (-X)");
    check(approx_eq(out.linear.y, 0.0f), "evade: Y = 0");

    std::printf("    [PASS] Evade produces force away from predicted threat\n");
}

// =============================================================================
// TEST 5: Evade returns zero when threat is outside panic radius
// =============================================================================
static void test_evade_outside_panic() {
    std::printf("  [Test 5] Evade returns zero outside panic radius...\n");

    apc::Vec3 pos(0, 0, 0);
    apc::Vec3 vel(0, 0, 0);
    apc::Vec3 threat_pos(100, 0, 0);
    apc::Vec3 threat_vel(0, 0, 0);
    float max_speed = 8.0f;
    float max_pred  = 0.1f; // Very short prediction → small panic radius

    apc::SteeringOutput out = apc::SteeringSystem::evade(
        pos, vel, threat_pos, threat_vel, max_speed, max_pred);

    check(approx_eq(apc::Vec3::length(out.linear), 0.0f),
          "evade: zero output when threat outside panic radius");

    std::printf("    [PASS] Evade returns zero outside panic radius\n");
}

// =============================================================================
// TEST 6: Alignment averages velocity vectors (perpendicular case)
// =============================================================================
static void test_alignment_perpendicular_velocities() {
    std::printf("  [Test 6] Alignment averages perpendicular velocity vectors...\n");

    // Two neighbors: one moving +X, one moving +Z
    apc::Vec3 positions[2]  = { {5, 0, 0}, {0, 0, 5} };
    apc::Vec3 velocities[2] = { {10, 0, 0}, {0, 0, 10} };
    float max_force = 20.0f;

    // Average velocity: (10,0,0) + (0,0,10) = (10,0,10) / 2 = (5,0,10)
    // Normalized: (5/sqrt(50), 0, 10/sqrt(50)) ≈ (0.7071, 0, 0.7071)
    // Scaled by max_force: (0.7071*20, 0, 0.7071*20) ≈ (14.14, 0, 14.14)
    apc::SteeringOutput out = apc::SteeringSystem::alignment(
        positions, velocities, 2, max_force);

    float mag = apc::Vec3::length(out.linear);
    check(mag > 0.0f, "alignment perpendicular: non-zero output");
    check(out.linear.x > 0.0f, "alignment perpendicular: positive X");
    check(out.linear.z > 0.0f, "alignment perpendicular: positive Z");
    check(approx_eq(out.linear.y, 0.0f), "alignment perpendicular: Y = 0");

    // Direction should be 45 degrees (X and Z equal)
    float angle = std::atan2(out.linear.z, out.linear.x);
    check(approx_eq(angle, apc::APC_HALF_PI * 0.5f, 0.01f),
          "alignment perpendicular: direction is 45 degrees");

    std::printf("    [PASS] Alignment averages perpendicular velocities correctly\n");
}

// =============================================================================
// TEST 7: Alignment skips zero-velocity neighbors
// =============================================================================
static void test_alignment_skips_zero_velocity() {
    std::printf("  [Test 7] Alignment skips zero-velocity neighbors...\n");

    // 3 neighbors: 1 moving +X, 2 stationary
    apc::Vec3 positions[3]  = { {5, 0, 0}, {3, 0, 0}, {7, 0, 0} };
    apc::Vec3 velocities[3] = { {6, 0, 0}, {0, 0, 0}, {0, 0, 0} };

    apc::SteeringOutput out = apc::SteeringSystem::alignment(
        positions, velocities, 3, 20.0f);

    // Only 1 valid neighbor moving in +X → alignment should be +X
    check(out.linear.x > 0.0f, "alignment skip zero: X > 0");
    check(approx_eq(out.linear.z, 0.0f, 0.001f), "alignment skip zero: Z ≈ 0");

    std::printf("    [PASS] Alignment correctly skips zero-velocity neighbors\n");
}

// =============================================================================
// TEST 8: Alignment returns zero when all neighbors are stationary
// =============================================================================
static void test_alignment_all_stationary() {
    std::printf("  [Test 8] Alignment returns zero when all neighbors stationary...\n");

    apc::Vec3 positions[3]  = { {5, 0, 0}, {3, 0, 2}, {7, 0, -1} };
    apc::Vec3 velocities[3] = { {0, 0, 0}, {0, 0, 0}, {0, 0, 0} };

    apc::SteeringOutput out = apc::SteeringSystem::alignment(
        positions, velocities, 3, 20.0f);

    check(approx_eq(apc::Vec3::length(out.linear), 0.0f),
          "alignment all zero: output = 0");

    std::printf("    [PASS] Alignment returns zero for all-stationary neighbors\n");
}

// =============================================================================
// TEST 9: Alignment clamps output to max_force
// =============================================================================
static void test_alignment_clamps_max_force() {
    std::printf("  [Test 9] Alignment clamps output to max_force...\n");

    apc::Vec3 positions[2]  = { {5, 0, 0}, {0, 0, 5} };
    apc::Vec3 velocities[2] = { {100, 0, 0}, {0, 0, 100} }; // Very fast neighbors
    float max_force = 5.0f; // Small clamp

    apc::SteeringOutput out = apc::SteeringSystem::alignment(
        positions, velocities, 2, max_force);

    float mag = apc::Vec3::length(out.linear);
    check(mag <= max_force + EPS, "alignment clamp: magnitude <= max_force");
    check(mag > 0.0f, "alignment clamp: non-zero output");

    std::printf("    [PASS] Alignment clamps output to max_force\n");
}

// =============================================================================
// TEST 10: Separation strength inversely proportional to distance
// =============================================================================
static void test_separation_inverse_distance() {
    std::printf("  [Test 10] Separation strength inversely proportional to distance...\n");

    apc::Vec3 pos(0, 0, 0);

    // Close neighbor: stronger repulsion
    apc::Vec3 near_neighbor[1] = { {0.5f, 0, 0} };
    apc::SteeringOutput out_near = apc::SteeringSystem::separation(
        near_neighbor, 1, pos, 2.0f, 20.0f);

    // Far neighbor: weaker repulsion (still within radius)
    apc::Vec3 far_neighbor[1] = { {1.8f, 0, 0} };
    apc::SteeringOutput out_far = apc::SteeringSystem::separation(
        far_neighbor, 1, pos, 2.0f, 20.0f);

    float near_mag = apc::Vec3::length(out_near.linear);
    float far_mag  = apc::Vec3::length(out_far.linear);

    check(near_mag > 0.0f, "separation close: non-zero");
    check(far_mag > 0.0f,  "separation far: non-zero");
    check(near_mag > far_mag, "separation: closer neighbor produces stronger repulsion");

    std::printf("    [PASS] Separation inversely proportional to distance\n");
}

// =============================================================================
// TEST 11: Cohesion seeks centroid of neighbors
// =============================================================================
static void test_cohesion_centroid() {
    std::printf("  [Test 11] Cohesion seeks centroid of neighbors...\n");

    apc::Vec3 pos(0, 0, 0);

    // Neighbors at (10,0,0) and (0,0,10) → centroid at (5,0,5)
    apc::Vec3 neighbors[2] = { {10, 0, 0}, {0, 0, 10} };
    apc::SteeringOutput out = apc::SteeringSystem::cohesion(
        neighbors, 2, pos, 8.0f);

    float mag = apc::Vec3::length(out.linear);
    check(mag > 0.0f, "cohesion: non-zero output");
    check(out.linear.x > 0.0f, "cohesion: X > 0 (toward centroid X)");
    check(out.linear.z > 0.0f, "cohesion: Z > 0 (toward centroid Z)");

    // Direction should be toward (5,0,5), which is 45 degrees
    float angle = std::atan2(out.linear.z, out.linear.x);
    check(approx_eq(angle, apc::APC_HALF_PI * 0.5f, 0.01f),
          "cohesion: direction toward centroid at 45 degrees");

    std::printf("    [PASS] Cohesion steers toward neighbor centroid\n");
}

// =============================================================================
// TEST 12: Blend of pursue + separation (real steering outputs)
// =============================================================================
static void test_blend_pursue_separation() {
    std::printf("  [Test 12] Blend of pursue + separation (real steering)...\n");

    apc::Vec3 pos(0, 0, 0);
    apc::Vec3 vel(0, 0, 0);
    apc::Vec3 target_pos(0, 0, 20);
    apc::Vec3 target_vel(2, 0, 0);

    // Pursue output
    apc::SteeringOutput pursue_out = apc::SteeringSystem::pursue(
        pos, vel, target_pos, target_vel, 8.0f, 1.0f);

    // Separation output (neighbor at (1,0,1))
    apc::Vec3 neighbors[1] = { {1, 0, 1} };
    apc::SteeringOutput sep_out = apc::SteeringSystem::separation(
        neighbors, 1, pos, 3.0f, 20.0f);

    // Blend: pursue weight=2.0, separation weight=1.0
    apc::WeightedSteering blend[2];
    blend[0].output = pursue_out;
    blend[0].weight = 2.0f;
    blend[1].output = sep_out;
    blend[1].weight = 1.0f;

    apc::SteeringOutput result = apc::SteeringSystem::blend(blend, 2);

    float mag = apc::Vec3::length(result.linear);
    check(mag > 0.0f, "blend pursue+sep: non-zero output");

    // Pursue should dominate (weight 2.0 vs 1.0), so result should have strong +Z
    check(result.linear.z > 0.0f, "blend pursue+sep: Z > 0 (pursue dominates)");

    // Separation pushes away from (1,0,1), so result X should be negative or small
    // (pursue leads target on X, separation pushes -X/-Z)
    check(approx_eq(result.linear.y, 0.0f), "blend pursue+sep: Y = 0");

    std::printf("    [PASS] Blend correctly weights pursue and separation\n");
}

// =============================================================================
// TEST 13: Blend of flee + cohesion (real steering outputs)
// =============================================================================
static void test_blend_flee_cohesion() {
    std::printf("  [Test 13] Blend of flee + cohesion (real steering)...\n");

    apc::Vec3 pos(0, 0, 0);

    // Flee from +X threat
    apc::SteeringOutput flee_out = apc::SteeringSystem::flee(
        pos, apc::Vec3(5, 0, 0), 8.0f, 10.0f);

    // Cohesion toward -Z neighbors
    apc::Vec3 neighbors[2] = { {-5, 0, -5}, {-3, 0, -7} };
    apc::SteeringOutput coh_out = apc::SteeringSystem::cohesion(
        neighbors, 2, pos, 8.0f);

    // Blend: flee weight=3.0, cohesion weight=1.0
    apc::WeightedSteering blend[2];
    blend[0].output = flee_out;
    blend[0].weight = 3.0f;
    blend[1].output = coh_out;
    blend[1].weight = 1.0f;

    apc::SteeringOutput result = apc::SteeringSystem::blend(blend, 2);

    float mag = apc::Vec3::length(result.linear);
    check(mag > 0.0f, "blend flee+coh: non-zero output");

    // Flee dominates: should push in -X direction
    check(result.linear.x < 0.0f, "blend flee+coh: X < 0 (flee dominates)");

    std::printf("    [PASS] Blend correctly weights flee and cohesion\n");
}

// =============================================================================
// TEST 14: Seek vs arrive at close range
// =============================================================================
static void test_seek_vs_arrive_close_range() {
    std::printf("  [Test 14] Seek vs arrive at close range...\n");

    apc::Vec3 pos(0, 0, 0);
    apc::Vec3 target(2, 0, 0);
    float max_speed = 10.0f;

    // Seek at 2m: full speed toward target
    apc::SteeringOutput seek_out = apc::SteeringSystem::seek(pos, target, max_speed);
    float seek_mag = apc::Vec3::length(seek_out.linear);

    // Arrive at 2m with slow_radius=3: should slow down
    apc::SteeringOutput arrive_out = apc::SteeringSystem::arrive(
        pos, target, max_speed, 0.5f, 3.0f);
    float arrive_mag = apc::Vec3::length(arrive_out.linear);

    check(seek_mag > 0.0f, "seek: non-zero");
    check(arrive_mag > 0.0f, "arrive: non-zero");
    check(seek_mag > arrive_mag, "seek faster than arrive at close range");
    check(approx_eq(seek_mag, max_speed, 0.01f), "seek: full speed at any distance");

    std::printf("    [PASS] Seek and arrive produce different results at close range\n");
}

// =============================================================================
// TEST 15: Pursue vs seek deviation on lateral target movement
// =============================================================================
static void test_pursue_vs_seek_lateral_deviation() {
    std::printf("  [Test 15] Pursue deviates from seek when target moves laterally...\n");

    apc::Vec3 pos(0, 0, 0);
    apc::Vec3 vel(0, 0, 0);
    apc::Vec3 target_pos(10, 0, 0);
    apc::Vec3 target_vel(0, 0, 5);  // Target moving purely in +Z (lateral to approach)
    float max_speed = 8.0f;
    float max_pred  = 2.0f;

    apc::SteeringOutput pursue_out = apc::SteeringSystem::pursue(
        pos, vel, target_pos, target_vel, max_speed, max_pred);
    apc::SteeringOutput seek_out = apc::SteeringSystem::seek(pos, target_pos, max_speed);

    // Seek goes purely in +X
    check(seek_out.linear.x > 0.0f, "seek lateral: X > 0");
    check(approx_eq(seek_out.linear.z, 0.0f, 0.001f), "seek lateral: Z ≈ 0");

    // Pursue should have a Z component (leading the target's +Z movement)
    check(pursue_out.linear.z > 0.0f, "pursue lateral: Z > 0 (leading lateral movement)");

    // Pursue Z must be strictly greater than seek Z
    check(pursue_out.linear.z > seek_out.linear.z + EPS,
          "pursue lateral: Z component > seek Z component");

    std::printf("    [PASS] Pursue correctly leads target on lateral movement\n");
}

// =============================================================================
// Main
// =============================================================================
int main() {
    std::printf("=== Integration Test: AI Steering Behaviors ===\n\n");

    test_pursue_intercept_trajectory();
    test_pursue_dynamic_lookahead();
    test_pursue_caps_prediction();
    test_evade_predicted_threat();
    test_evade_outside_panic();
    test_alignment_perpendicular_velocities();
    test_alignment_skips_zero_velocity();
    test_alignment_all_stationary();
    test_alignment_clamps_max_force();
    test_separation_inverse_distance();
    test_cohesion_centroid();
    test_blend_pursue_separation();
    test_blend_flee_cohesion();
    test_seek_vs_arrive_close_range();
    test_pursue_vs_seek_lateral_deviation();

    std::printf("\n=== AI Steering: 15 tests passed (%d assertions) ===\n", g_assertions);
    return 0;
}
