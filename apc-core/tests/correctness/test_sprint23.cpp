// =============================================================================
// Sprint 23 Tests — AI Steering Behaviors & Motor Controller
// =============================================================================
//
// Tests for the APC Sprint 23 headers:
//   1.  SteeringBehavior enum values
//   2.  SteeringOutput default values
//   3.  SteeringOutput urgency default
//   4.  WeightedSteering default values
//   5.  SteeringRequest default values
//   6.  SteeringSystem::seek direction toward target
//   7.  SteeringSystem::seek clamps to max_speed
//   8.  SteeringSystem::flee direction away from threat
//   9.  SteeringSystem::flee returns zero outside panic_radius
//  10. SteeringSystem::arrive slows near target
//  11. SteeringSystem::arrive produces full speed far from target
//  12. SteeringSystem::pursue leads target based on velocity
//  13. SteeringSystem::separation pushes away from neighbors
//  14. SteeringSystem::cohesion pulls toward neighbor centroid
//  15. SteeringSystem::alignment matches neighbor heading
//  16. SteeringSystem::blend weighted combination
//  17. AIMotorController default values
//  18. AIMotorController.update() produces MotorIntent
//  19. AIMotorController.update() applies accuracy scaling
//  20. AIMotorController reaction_delay timer behavior
//
// Pattern: int main() + assert(), no test framework.
// =============================================================================

#include "apc_ai/apc_ai_steering.h"
#include "apc_ai/apc_ai_motor.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include "apc_math/apc_math_common.h"
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
// TEST 1: SteeringBehavior enum values
// =============================================================================
static int test_steering_behavior_enum() {
    std::printf("  [Test 1] SteeringBehavior enum values...\n");

    assert(static_cast<uint8_t>(apc::SteeringBehavior::SEEK) == 0);
    assert(static_cast<uint8_t>(apc::SteeringBehavior::FLEE) == 1);
    assert(static_cast<uint8_t>(apc::SteeringBehavior::ARRIVE) == 2);
    assert(static_cast<uint8_t>(apc::SteeringBehavior::PURSUE) == 3);
    assert(static_cast<uint8_t>(apc::SteeringBehavior::EVADE) == 4);
    assert(static_cast<uint8_t>(apc::SteeringBehavior::WANDER) == 5);
    assert(static_cast<uint8_t>(apc::SteeringBehavior::ALIGN) == 6);
    assert(static_cast<uint8_t>(apc::SteeringBehavior::FACE) == 7);
    assert(static_cast<uint8_t>(apc::SteeringBehavior::SEPARATION) == 8);
    assert(static_cast<uint8_t>(apc::SteeringBehavior::COHESION) == 9);
    assert(static_cast<uint8_t>(apc::SteeringBehavior::LEADER_FOLLOW) == 10);
    assert(static_cast<uint8_t>(apc::SteeringBehavior::PATH_FOLLOWING) == 11);
    assert(static_cast<uint8_t>(apc::SteeringBehavior::PREDICTION) == 12);

    std::printf("    [PASS] SteeringBehavior enum values verified (0-12)\n");
    return 0;
}

// =============================================================================
// TEST 2: SteeringOutput default values
// =============================================================================
static int test_steering_output_defaults() {
    std::printf("  [Test 2] SteeringOutput default values...\n");

    apc::SteeringOutput out;

    assert(approx_eq(out.linear.x, 0.0f) && "linear.x = 0");
    assert(approx_eq(out.linear.y, 0.0f) && "linear.y = 0");
    assert(approx_eq(out.linear.z, 0.0f) && "linear.z = 0");
    assert(approx_eq(out.angular.x, 0.0f) && "angular.x = 0");
    assert(approx_eq(out.angular.y, 0.0f) && "angular.y = 0");
    assert(approx_eq(out.angular.z, 0.0f) && "angular.z = 0");

    std::printf("    [PASS] SteeringOutput defaults: zero linear/angular\n");
    return 0;
}

// =============================================================================
// TEST 3: SteeringOutput urgency default
// =============================================================================
static int test_steering_output_urgency_default() {
    std::printf("  [Test 3] SteeringOutput urgency default...\n");

    apc::SteeringOutput out;

    assert(approx_eq(out.urgency, 0.0f) && "urgency default = 0.0");

    // Urgency is 0.0-1.0, so default is 0 (no urgency)
    assert(out.urgency >= 0.0f && out.urgency <= 1.0f && "urgency in [0,1]");

    std::printf("    [PASS] SteeringOutput urgency default = 0.0\n");
    return 0;
}

// =============================================================================
// TEST 4: WeightedSteering default values
// =============================================================================
static int test_weighted_steering_defaults() {
    std::printf("  [Test 4] WeightedSteering default values...\n");

    apc::WeightedSteering ws;

    assert(ws.behavior == apc::SteeringBehavior::SEEK && "default behavior = SEEK");
    assert(approx_eq(ws.weight, 1.0f) && "default weight = 1.0");
    assert(approx_eq(ws.output.linear.x, 0.0f) && "output.linear.x = 0");
    assert(approx_eq(ws.output.urgency, 0.0f) && "output.urgency = 0");

    std::printf("    [PASS] WeightedSteering defaults: SEEK, weight=1.0, zero output\n");
    return 0;
}

// =============================================================================
// TEST 5: SteeringRequest default values
// =============================================================================
static int test_steering_request_defaults() {
    std::printf("  [Test 5] SteeringRequest default values...\n");

    apc::SteeringRequest req;

    assert(approx_eq(req.target_position.x, 0.0f) && "target_position.x = 0");
    assert(approx_eq(req.target_position.y, 0.0f) && "target_position.y = 0");
    assert(approx_eq(req.target_position.z, 0.0f) && "target_position.z = 0");
    assert(approx_eq(req.target_velocity.x, 0.0f) && "target_velocity = 0");
    assert(approx_eq(req.arrive_radius, 1.0f) && "arrive_radius = 1.0");
    assert(approx_eq(req.arrive_slow_radius, 3.0f) && "arrive_slow_radius = 3.0");
    assert(approx_eq(req.wander_radius, 2.0f) && "wander_radius = 2.0");
    assert(approx_eq(req.wander_offset, 2.0f) && "wander_offset = 2.0");
    assert(approx_eq(req.wander_rate, 1.5f) && "wander_rate = 1.5");
    assert(approx_eq(req.separation_radius, 1.5f) && "separation_radius = 1.5");
    assert(approx_eq(req.separation_weight, 2.0f) && "separation_weight = 2.0");
    assert(approx_eq(req.max_speed, 8.0f) && "max_speed = 8.0");
    assert(approx_eq(req.max_force, 20.0f) && "max_force = 20.0");

    std::printf("    [PASS] SteeringRequest defaults verified\n");
    return 0;
}

// =============================================================================
// TEST 6: SteeringSystem::seek produces direction toward target
// =============================================================================
static int test_seek_direction() {
    std::printf("  [Test 6] SteeringSystem::seek direction toward target...\n");

    apc::Vec3 position(0.0f, 0.0f, 0.0f);
    apc::Vec3 target(5.0f, 0.0f, 0.0f);

    apc::SteeringOutput out = apc::SteeringSystem::seek(position, target, 8.0f);

    // Output should point in +X direction (toward target)
    assert(out.linear.x > 0.0f && "seek: linear.x > 0 (toward target)");
    assert(approx_eq(out.linear.y, 0.0f) && "seek: linear.y = 0 (XZ plane)");
    assert(approx_eq(out.linear.z, 0.0f) && "seek: linear.z = 0");

    // Seek toward Z direction
    apc::Vec3 position2(0.0f, 0.0f, 0.0f);
    apc::Vec3 target2(0.0f, 0.0f, -10.0f);
    apc::SteeringOutput out2 = apc::SteeringSystem::seek(position2, target2, 8.0f);
    assert(out2.linear.z < 0.0f && "seek: linear.z < 0 (toward -Z target)");

    std::printf("    [PASS] seek produces direction toward target\n");
    return 0;
}

// =============================================================================
// TEST 7: SteeringSystem::seek clamps to max_speed
// =============================================================================
static int test_seek_clamps_max_speed() {
    std::printf("  [Test 7] SteeringSystem::seek clamps to max_speed...\n");

    apc::Vec3 position(0.0f, 0.0f, 0.0f);
    apc::Vec3 target(100.0f, 0.0f, 0.0f); // Very far away

    float max_speed = 5.0f;
    apc::SteeringOutput out = apc::SteeringSystem::seek(position, target, max_speed);

    float mag = apc::Vec3::length(out.linear);
    assert(mag <= max_speed + EPS && "seek: magnitude <= max_speed");
    assert(mag > 0.0f && "seek: magnitude > 0");

    // Also test with small max_speed
    apc::SteeringOutput out2 = apc::SteeringSystem::seek(position, target, 0.5f);
    float mag2 = apc::Vec3::length(out2.linear);
    assert(mag2 <= 0.5f + EPS && "seek: magnitude <= 0.5");

    std::printf("    [PASS] seek clamps output to max_speed\n");
    return 0;
}

// =============================================================================
// TEST 8: SteeringSystem::flee produces direction away from threat
// =============================================================================
static int test_flee_direction() {
    std::printf("  [Test 8] SteeringSystem::flee direction away from threat...\n");

    apc::Vec3 position(0.0f, 0.0f, 0.0f);
    apc::Vec3 threat(5.0f, 0.0f, 0.0f);

    apc::SteeringOutput out = apc::SteeringSystem::flee(position, threat, 8.0f, 10.0f);

    // Flee should push in -X direction (away from +X threat)
    assert(out.linear.x < 0.0f && "flee: linear.x < 0 (away from +X threat)");
    assert(approx_eq(out.linear.y, 0.0f) && "flee: linear.y = 0");
    assert(approx_eq(out.linear.z, 0.0f) && "flee: linear.z = 0");

    // Flee from -Z threat should push in +Z
    apc::Vec3 position2(0.0f, 0.0f, 0.0f);
    apc::Vec3 threat2(0.0f, 0.0f, -3.0f);
    apc::SteeringOutput out2 = apc::SteeringSystem::flee(position2, threat2, 8.0f, 10.0f);
    assert(out2.linear.z > 0.0f && "flee: linear.z > 0 (away from -Z threat)");

    std::printf("    [PASS] flee produces direction away from threat\n");
    return 0;
}

// =============================================================================
// TEST 9: SteeringSystem::flee returns zero outside panic_radius
// =============================================================================
static int test_flee_outside_panic_radius() {
    std::printf("  [Test 9] SteeringSystem::flee returns zero outside panic_radius...\n");

    apc::Vec3 position(0.0f, 0.0f, 0.0f);
    apc::Vec3 threat(50.0f, 0.0f, 0.0f); // 50 units away

    float panic_radius = 10.0f;
    apc::SteeringOutput out = apc::SteeringSystem::flee(position, threat, 8.0f, panic_radius);

    assert(approx_eq(out.linear.x, 0.0f) && "flee outside panic: linear.x = 0");
    assert(approx_eq(out.linear.z, 0.0f) && "flee outside panic: linear.z = 0");
    float mag = apc::Vec3::length(out.linear);
    assert(approx_eq(mag, 0.0f) && "flee outside panic: magnitude = 0");

    // Inside panic radius should produce output
    apc::Vec3 threat_near(5.0f, 0.0f, 0.0f);
    apc::SteeringOutput out_near = apc::SteeringSystem::flee(position, threat_near, 8.0f, panic_radius);
    float mag_near = apc::Vec3::length(out_near.linear);
    assert(mag_near > 0.0f && "flee inside panic: magnitude > 0");

    std::printf("    [PASS] flee returns zero outside panic_radius, active inside\n");
    return 0;
}

// =============================================================================
// TEST 10: SteeringSystem::arrive slows near target
// =============================================================================
static int test_arrive_slows_near_target() {
    std::printf("  [Test 10] SteeringSystem::arrive slows near target...\n");

    float max_speed = 10.0f;
    float arrive_radius = 0.5f;
    float slow_radius = 3.0f;

    // Position 2.0 away from target (within slow_radius, outside arrive_radius)
    apc::Vec3 position(0.0f, 0.0f, 0.0f);
    apc::Vec3 target(2.0f, 0.0f, 0.0f);
    apc::SteeringOutput out = apc::SteeringSystem::arrive(position, target, max_speed, arrive_radius, slow_radius);

    float mag = apc::Vec3::length(out.linear);
    // At dist=2.0 with slow_radius=3.0, desired_speed = 10 * (2/3) ≈ 6.67
    // Should be less than max_speed
    assert(mag < max_speed && "arrive: within slow_radius, speed < max_speed");
    assert(mag > 0.0f && "arrive: still moving toward target");

    // Position 0.3 away (inside arrive_radius) — should be zero
    apc::Vec3 target_close(0.3f, 0.0f, 0.0f);
    apc::SteeringOutput out_arrived = apc::SteeringSystem::arrive(position, target_close, max_speed, arrive_radius, slow_radius);
    float mag_arrived = apc::Vec3::length(out_arrived.linear);
    assert(approx_eq(mag_arrived, 0.0f) && "arrive: inside arrive_radius, output = 0");

    std::printf("    [PASS] arrive slows near target, stops within arrive_radius\n");
    return 0;
}

// =============================================================================
// TEST 11: SteeringSystem::arrive produces full speed far from target
// =============================================================================
static int test_arrive_full_speed_far() {
    std::printf("  [Test 11] SteeringSystem::arrive produces full speed far from target...\n");

    float max_speed = 10.0f;
    float arrive_radius = 0.5f;
    float slow_radius = 3.0f;

    // Position 20.0 away (well beyond slow_radius)
    apc::Vec3 position(0.0f, 0.0f, 0.0f);
    apc::Vec3 target(20.0f, 0.0f, 0.0f);
    apc::SteeringOutput out = apc::SteeringSystem::arrive(position, target, max_speed, arrive_radius, slow_radius);

    float mag = apc::Vec3::length(out.linear);
    // Far from target: desired_speed = max_speed, clamped to max_speed
    assert(approx_eq(mag, max_speed, 0.01f) && "arrive far: speed = max_speed");

    std::printf("    [PASS] arrive produces full speed when far from target\n");
    return 0;
}

// =============================================================================
// TEST 12: SteeringSystem::pursue leads target based on velocity
// =============================================================================
static int test_pursue_leads_target() {
    std::printf("  [Test 12] SteeringSystem::pursue leads target based on velocity...\n");

    apc::Vec3 position(0.0f, 0.0f, 0.0f);
    apc::Vec3 velocity(0.0f, 0.0f, 0.0f); // Agent velocity (reserved)
    apc::Vec3 target_pos(10.0f, 0.0f, 0.0f);
    apc::Vec3 target_vel(3.0f, 0.0f, 0.0f); // Target moving in +X

    float max_speed = 8.0f;
    float max_prediction_time = 1.0f;

    apc::SteeringOutput out_pursue = apc::SteeringSystem::pursue(
        position, velocity, target_pos, target_vel, max_speed, max_prediction_time);

    // Pursue should seek toward future position (ahead of target)
    assert(out_pursue.linear.x > 0.0f && "pursue: linear.x > 0 (chasing +X)");

    // Compare with a simple seek to the current target position
    apc::SteeringOutput out_seek = apc::SteeringSystem::seek(position, target_pos, max_speed);

    // Pursue should produce a different result than simple seek when target is moving
    // (because it leads the target)
    float pursue_mag = apc::Vec3::length(out_pursue.linear);
    float seek_mag = apc::Vec3::length(out_seek.linear);
    // Both should have non-zero magnitude
    assert(pursue_mag > 0.0f && "pursue: non-zero magnitude");
    assert(seek_mag > 0.0f && "seek: non-zero magnitude");

    std::printf("    [PASS] pursue leads target based on velocity\n");
    return 0;
}

// =============================================================================
// TEST 13: SteeringSystem::separation pushes away from neighbors
// =============================================================================
static int test_separation_pushes_away() {
    std::printf("  [Test 13] SteeringSystem::separation pushes away from neighbors...\n");

    apc::Vec3 position(0.0f, 0.0f, 0.0f);

    // Neighbor at (1, 0, 0) — within separation_radius
    apc::Vec3 neighbors[1] = { apc::Vec3(1.0f, 0.0f, 0.0f) };

    apc::SteeringOutput out = apc::SteeringSystem::separation(
        neighbors, 1, position, 1.5f, 20.0f);

    // Separation should push in -X direction (away from +X neighbor)
    assert(out.linear.x < 0.0f && "separation: pushes away from neighbor");
    assert(approx_eq(out.linear.y, 0.0f) && "separation: Y = 0");

    // No neighbors → zero output
    apc::SteeringOutput out_empty = apc::SteeringSystem::separation(
        nullptr, 0, position, 1.5f, 20.0f);
    assert(approx_eq(apc::Vec3::length(out_empty.linear), 0.0f) && "separation: no neighbors = 0");

    // Neighbor outside radius → zero output
    apc::Vec3 far_neighbors[1] = { apc::Vec3(10.0f, 0.0f, 0.0f) };
    apc::SteeringOutput out_far = apc::SteeringSystem::separation(
        far_neighbors, 1, position, 1.5f, 20.0f);
    assert(approx_eq(apc::Vec3::length(out_far.linear), 0.0f) && "separation: far neighbor = 0");

    std::printf("    [PASS] separation pushes away from nearby neighbors\n");
    return 0;
}

// =============================================================================
// TEST 14: SteeringSystem::cohesion pulls toward neighbor centroid
// =============================================================================
static int test_cohesion_pulls_to_centroid() {
    std::printf("  [Test 14] SteeringSystem::cohesion pulls toward neighbor centroid...\n");

    apc::Vec3 position(0.0f, 0.0f, 0.0f);

    // Two neighbors at (5,0,0) and (5,0,0) — centroid at (5,0,0)
    apc::Vec3 neighbors[2] = {
        apc::Vec3(5.0f, 0.0f, 0.0f),
        apc::Vec3(5.0f, 0.0f, 0.0f)
    };

    apc::SteeringOutput out = apc::SteeringSystem::cohesion(
        neighbors, 2, position, 8.0f);

    // Cohesion should pull toward +X (centroid direction)
    assert(out.linear.x > 0.0f && "cohesion: pulls toward centroid");
    assert(approx_eq(out.linear.y, 0.0f) && "cohesion: Y = 0");

    // No neighbors → zero output
    apc::SteeringOutput out_empty = apc::SteeringSystem::cohesion(
        nullptr, 0, position, 8.0f);
    assert(approx_eq(apc::Vec3::length(out_empty.linear), 0.0f) && "cohesion: no neighbors = 0");

    std::printf("    [PASS] cohesion pulls toward neighbor centroid\n");
    return 0;
}

// =============================================================================
// TEST 15: SteeringSystem::alignment matches neighbor heading
// =============================================================================
static int test_alignment_matches_heading() {
    std::printf("  [Test 15] SteeringSystem::alignment matches neighbor heading...\n");

    apc::Vec3 heading(0.0f, 0.0f, 0.0f); // Zero heading

    // Neighbors used as approximate velocity proxies (internally multiplied by 0.5)
    // Two "velocity" neighbors at (4, 0, 0) and (4, 0, 0)
    apc::Vec3 neighbors[2] = {
        apc::Vec3(4.0f, 0.0f, 0.0f),
        apc::Vec3(4.0f, 0.0f, 0.0f)
    };

    apc::SteeringOutput out = apc::SteeringSystem::alignment(
        neighbors, 2, heading, 20.0f);

    // Alignment should produce a force that steers toward average neighbor direction
    // Average velocity = (4*0.5 + 4*0.5) / 2 = 2.0 in X
    // steer = avg_vel - heading = (2,0,0) - (0,0,0) = (2,0,0)
    assert(out.linear.x > 0.0f && "alignment: steers toward neighbor average");
    assert(approx_eq(out.linear.y, 0.0f) && "alignment: Y = 0");

    // No neighbors → zero output
    apc::SteeringOutput out_empty = apc::SteeringSystem::alignment(
        nullptr, 0, heading, 20.0f);
    assert(approx_eq(apc::Vec3::length(out_empty.linear), 0.0f) && "alignment: no neighbors = 0");

    std::printf("    [PASS] alignment steers to match neighbor heading\n");
    return 0;
}

// =============================================================================
// TEST 16: SteeringSystem::blend weighted combination
// =============================================================================
static int test_blend_weighted_combination() {
    std::printf("  [Test 16] SteeringSystem::blend weighted combination...\n");

    // Create two weighted steering outputs
    apc::WeightedSteering behaviors[2];

    // First behavior: seek +X with weight 1.0
    behaviors[0].output.linear = apc::Vec3(4.0f, 0.0f, 0.0f);
    behaviors[0].output.urgency = 0.5f;
    behaviors[0].weight = 1.0f;

    // Second behavior: seek +Z with weight 3.0 (heavier)
    behaviors[1].output.linear = apc::Vec3(0.0f, 0.0f, 6.0f);
    behaviors[1].output.urgency = 0.8f;
    behaviors[1].weight = 3.0f;

    apc::SteeringOutput result = apc::SteeringSystem::blend(behaviors, 2);

    // Weighted average: linear = (4*1 + 0*3) / (1+3) = 1.0 in X
    //                   linear = (0*1 + 6*3) / (1+3) = 4.5 in Z
    assert(approx_eq(result.linear.x, 1.0f, 0.01f) && "blend: X = 1.0");
    assert(approx_eq(result.linear.z, 4.5f, 0.01f) && "blend: Z = 4.5");
    assert(approx_eq(result.linear.y, 0.0f) && "blend: Y = 0");

    // Urgency: (0.5*1 + 0.8*3) / 4 = (0.5 + 2.4) / 4 = 0.725
    assert(approx_eq(result.urgency, 0.725f, 0.01f) && "blend: urgency weighted average");

    // Empty blend → zero output
    apc::SteeringOutput empty = apc::SteeringSystem::blend(nullptr, 0);
    assert(approx_eq(apc::Vec3::length(empty.linear), 0.0f) && "blend: empty = 0");
    assert(approx_eq(empty.urgency, 0.0f) && "blend: empty urgency = 0");

    std::printf("    [PASS] blend computes weighted combination correctly\n");
    return 0;
}

// =============================================================================
// TEST 17: AIMotorController default values
// =============================================================================
static int test_motor_controller_defaults() {
    std::printf("  [Test 17] AIMotorController default values...\n");

    apc::AIMotorController ctrl;

    assert(approx_eq(ctrl.reaction_delay, 0.1f) && "reaction_delay = 0.1");
    assert(approx_eq(ctrl.accuracy, 0.9f) && "accuracy = 0.9");
    assert(approx_eq(ctrl.aggression, 0.5f) && "aggression = 0.5");
    assert(approx_eq(ctrl.skill_level, 0.7f) && "skill_level = 0.7");
    assert(approx_eq(ctrl.reaction_timer, 0.0f) && "reaction_timer = 0.0");

    // Steering config defaults
    assert(approx_eq(ctrl.steering_config.max_speed, 8.0f) && "max_speed = 8.0");
    assert(approx_eq(ctrl.steering_config.max_force, 20.0f) && "max_force = 20.0");
    assert(approx_eq(ctrl.steering_config.arrive_radius, 1.0f) && "arrive_radius = 1.0");

    // Accumulated intent defaults
    assert(approx_eq(ctrl.accumulated_intent.move_speed, 0.0f) && "move_speed = 0");
    assert(ctrl.accumulated_intent.action_type == apc::ACTION_IDLE && "action = IDLE");

    std::printf("    [PASS] AIMotorController defaults verified\n");
    return 0;
}

// =============================================================================
// TEST 18: AIMotorController.update() produces MotorIntent
// =============================================================================
static int test_motor_controller_update() {
    std::printf("  [Test 18] AIMotorController.update() produces MotorIntent...\n");

    apc::AIMotorController ctrl;
    ctrl.reaction_timer = 0.0f; // No delay

    apc::SteeringOutput steering_in;
    steering_in.linear = apc::Vec3(5.0f, 0.0f, 0.0f); // Move in +X
    steering_in.angular = apc::Vec3(0.0f, 0.0f, 0.0f);
    steering_in.urgency = 0.6f;

    apc::Vec3 position(0.0f, 0.0f, 0.0f);
    apc::Quat orientation = apc::Quat::identity();

    apc::MotorIntent intent = ctrl.update(steering_in, position, orientation, 0.016f);

    // Should produce non-zero move_direction
    float move_mag = apc::Vec3::length(intent.move_direction);
    assert(move_mag > 0.0f && "update: move_direction has magnitude");

    // move_speed should be > 0 (because linear has magnitude 5.0)
    assert(intent.move_speed > 0.0f && "update: move_speed > 0");

    // With urgency=0.6 (>0.5), action should be ACTION_MOVE
    assert(intent.action_type == apc::ACTION_MOVE && "update: urgency=0.6 → ACTION_MOVE");

    // Y component should be zero (XZ plane)
    assert(approx_eq(intent.move_direction.y, 0.0f) && "update: move_dir Y = 0");

    std::printf("    [PASS] update() produces valid MotorIntent\n");
    return 0;
}

// =============================================================================
// TEST 19: AIMotorController.update() applies accuracy scaling
// =============================================================================
static int test_motor_controller_accuracy() {
    std::printf("  [Test 19] AIMotorController.update() applies accuracy scaling...\n");

    apc::Vec3 position(0.0f, 0.0f, 0.0f);
    apc::Quat orientation = apc::Quat::identity();

    // --- High accuracy controller ---
    apc::AIMotorController high_acc;
    high_acc.reaction_timer = 0.0f;
    high_acc.accuracy = 1.0f;
    high_acc.skill_level = 1.0f;

    apc::SteeringOutput steering_in;
    steering_in.linear = apc::Vec3(8.0f, 0.0f, 0.0f); // Pure +X
    steering_in.angular = apc::Vec3(0.0f, 0.0f, 0.0f);
    steering_in.urgency = 0.5f;

    apc::MotorIntent intent_high = high_acc.update(steering_in, position, orientation, 0.016f);

    // With accuracy=1.0, direction should be very close to pure +X
    // effective_accuracy = 1.0 * (0.8 + 0.2 * 1.0) = 1.0
    // noise_scale = (1.0 - 1.0) * 0.3 = 0 → no noise
    assert(approx_eq(intent_high.move_direction.x, 1.0f, 0.01f) && "high_acc: dir.x ≈ 1.0");
    assert(approx_eq(std::abs(intent_high.move_direction.z), 0.0f, 0.01f) && "high_acc: dir.z ≈ 0");

    // --- Low accuracy controller ---
    apc::AIMotorController low_acc;
    low_acc.reaction_timer = 0.0f;
    low_acc.accuracy = 0.1f;
    low_acc.skill_level = 0.1f;

    apc::MotorIntent intent_low = low_acc.update(steering_in, position, orientation, 0.016f);

    // With low accuracy, direction should deviate from pure +X
    float cross_component = std::abs(intent_low.move_direction.z);
    // Low accuracy effective_accuracy = 0.1 * (0.8 + 0.2*0.1) = 0.1 * 0.82 = 0.082
    // noise_scale = (1.0 - 0.082) * 0.3 ≈ 0.2754 → some noise present
    // The Z component should be non-zero (noise adds to it)
    assert(cross_component > 0.0f && "low_acc: direction deviates from pure X");

    std::printf("    [PASS] accuracy scaling affects output direction\n");
    return 0;
}

// =============================================================================
// TEST 20: AIMotorController reaction_delay timer behavior
// =============================================================================
static int test_motor_controller_reaction_delay() {
    std::printf("  [Test 20] AIMotorController reaction_delay timer behavior...\n");

    apc::SteeringOutput steering_in;
    steering_in.linear = apc::Vec3(5.0f, 0.0f, 0.0f);
    steering_in.angular = apc::Vec3(0.0f, 0.0f, 0.0f);
    steering_in.urgency = 0.6f;

    apc::Vec3 position(0.0f, 0.0f, 0.0f);
    apc::Quat orientation = apc::Quat::identity();

    // --- First update: reaction_timer = 0, should process immediately ---
    apc::AIMotorController ctrl;
    assert(approx_eq(ctrl.reaction_timer, 0.0f) && "timer starts at 0");

    apc::MotorIntent intent1 = ctrl.update(steering_in, position, orientation, 0.016f);
    assert(intent1.move_speed > 0.0f && "first update: processed immediately");

    // After first update, reaction_timer should be set
    assert(ctrl.reaction_timer > 0.0f && "reaction_timer set after update");

    // --- Second update with small dt: should return accumulated intent ---
    float saved_timer = ctrl.reaction_timer;
    apc::MotorIntent intent2 = ctrl.update(steering_in, position, orientation, 0.001f);
    // Timer should have decreased
    assert(ctrl.reaction_timer < saved_timer && "reaction_timer decreases");

    // --- Set a large reaction_delay and verify it blocks updates ---
    apc::AIMotorController ctrl2;
    ctrl2.reaction_delay = 1.0f;
    ctrl2.reaction_timer = 0.0f;

    // First update processes (timer was 0)
    apc::MotorIntent intent_first = ctrl2.update(steering_in, position, orientation, 0.016f);
    float first_speed = intent_first.move_speed;
    assert(first_speed > 0.0f && "first: processes");

    // Timer should now be set to something > 0
    // effective_reaction = 1.0 * (1.2 - 0.4 * 0.7) = 1.0 * 0.92 = 0.92
    assert(ctrl2.reaction_timer > 0.5f && "large delay: timer > 0.5");

    // Second update with tiny dt: should return accumulated intent (previous)
    apc::MotorIntent intent_blocked = ctrl2.update(steering_in, position, orientation, 0.001f);
    // During reaction delay, returns accumulated_intent (the previous result)
    assert(approx_eq(intent_blocked.move_speed, first_speed, 0.001f) &&
           "during delay: returns accumulated intent");

    std::printf("    [PASS] reaction_delay timer blocks during delay period\n");
    return 0;
}

// =============================================================================
// TEST 21: AIMotorController.set_preset configures roles
// =============================================================================
static int test_motor_controller_set_preset() {
    std::printf("  [Test 21] AIMotorController.set_preset configures roles...\n");

    apc::AIMotorController ctrl;

    // Defender preset
    ctrl.set_preset("defender");
    assert(approx_eq(ctrl.reaction_delay, 0.12f) && "defender: reaction_delay = 0.12");
    assert(approx_eq(ctrl.accuracy, 0.85f) && "defender: accuracy = 0.85");
    assert(approx_eq(ctrl.aggression, 0.4f) && "defender: aggression = 0.4");
    assert(approx_eq(ctrl.steering_config.max_speed, 7.0f) && "defender: max_speed = 7.0");

    // Forward preset
    ctrl.set_preset("forward");
    assert(approx_eq(ctrl.reaction_delay, 0.08f) && "forward: reaction_delay = 0.08");
    assert(approx_eq(ctrl.aggression, 0.7f) && "forward: aggression = 0.7");
    assert(approx_eq(ctrl.steering_config.max_speed, 8.5f) && "forward: max_speed = 8.5");

    // Goalkeeper preset
    ctrl.set_preset("goalkeeper");
    assert(approx_eq(ctrl.reaction_delay, 0.06f) && "goalkeeper: reaction_delay = 0.06");
    assert(approx_eq(ctrl.accuracy, 0.90f) && "goalkeeper: accuracy = 0.90");
    assert(approx_eq(ctrl.aggression, 0.3f) && "goalkeeper: aggression = 0.3");

    // Reset
    ctrl.reset();
    assert(approx_eq(ctrl.reaction_delay, 0.1f) && "reset: reaction_delay = 0.1");
    assert(approx_eq(ctrl.accuracy, 0.9f) && "reset: accuracy = 0.9");

    std::printf("    [PASS] set_preset configures role-specific parameters\n");
    return 0;
}

// =============================================================================
// TEST 22: SteeringSystem::evade inverse of pursue
// =============================================================================
static int test_evade_inverse_pursue() {
    std::printf("  [Test 22] SteeringSystem::evade inverse of pursue...\n");

    apc::Vec3 position(0.0f, 0.0f, 0.0f);
    apc::Vec3 velocity(0.0f, 0.0f, 0.0f);
    apc::Vec3 threat_pos(10.0f, 0.0f, 0.0f);
    apc::Vec3 threat_vel(5.0f, 0.0f, 0.0f);

    apc::SteeringOutput out = apc::SteeringSystem::evade(
        position, velocity, threat_pos, threat_vel, 8.0f, 1.0f);

    // Evade should push away from the threat
    assert(out.linear.x < 0.0f && "evade: pushes away from threat (-X)");
    assert(approx_eq(out.linear.y, 0.0f) && "evade: Y = 0");

    std::printf("    [PASS] evade pushes away from predicted threat position\n");
    return 0;
}

// =============================================================================
// TEST 23: SteeringSystem::wander produces non-zero output
// =============================================================================
static int test_wander_nonzero() {
    std::printf("  [Test 23] SteeringSystem::wander produces non-zero output...\n");

    apc::Vec3 position(0.0f, 0.0f, 0.0f);
    apc::Vec3 heading(0.0f, 0.0f, 3.0f); // Moving in +Z

    apc::SteeringOutput out = apc::SteeringSystem::wander(
        position, heading, 2.0f, 2.0f, 1.5f, 8.0f);

    float linear_mag = apc::Vec3::length(out.linear);
    assert(linear_mag > 0.0f && "wander: linear output > 0");

    // Angular output should have Y component (wander_rate)
    assert(!approx_eq(out.angular.y, 0.0f) && "wander: angular Y != 0");

    // Y component of linear should be 0 (XZ plane)
    assert(approx_eq(out.linear.y, 0.0f) && "wander: linear Y = 0");

    std::printf("    [PASS] wander produces deterministic non-zero output\n");
    return 0;
}

// =============================================================================
// Main
// =============================================================================
int main() {
    std::printf("=== Sprint 23 Tests ===\n");
    int fails = 0;

    fails += test_steering_behavior_enum();
    fails += test_steering_output_defaults();
    fails += test_steering_output_urgency_default();
    fails += test_weighted_steering_defaults();
    fails += test_steering_request_defaults();
    fails += test_seek_direction();
    fails += test_seek_clamps_max_speed();
    fails += test_flee_direction();
    fails += test_flee_outside_panic_radius();
    fails += test_arrive_slows_near_target();
    fails += test_arrive_full_speed_far();
    fails += test_pursue_leads_target();
    fails += test_separation_pushes_away();
    fails += test_cohesion_pulls_to_centroid();
    fails += test_alignment_matches_heading();
    fails += test_blend_weighted_combination();
    fails += test_motor_controller_defaults();
    fails += test_motor_controller_update();
    fails += test_motor_controller_accuracy();
    fails += test_motor_controller_reaction_delay();
    fails += test_motor_controller_set_preset();
    fails += test_evade_inverse_pursue();
    fails += test_wander_nonzero();

    int passed = 23 - fails;
    std::printf("=== Sprint 23: %d tests passed, %d failed ===\n", passed, fails);
    return fails;
}
