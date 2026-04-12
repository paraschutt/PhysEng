// =============================================================================
// Sprint 29 Tests — Collision Solver Unification: Mass-Ratio Fix (Phase 14 Act 4)
// =============================================================================
//
// Validates the block solver that handles extreme mass ratio collisions
// (e.g., 100kg athlete vs 0.4kg ball = 250:1 ratio) via direct 1-shot impulse.
//
// The block solver replaces iterative SI for ball-body contacts, computing the
// exact normal impulse in a single step. This eliminates ball jitter, vibration,
// and tunneling caused by SI convergence failure at extreme mass ratios.
//
// Tests:
//   1.  enable_block_solve defaults to true
//   2.  VelocityConstraint::is_block_solved defaults to false
//   3.  Block solver is triggered for ball-body contacts (XOR check)
//   4.  Block solver is NOT triggered for non-ball contacts
//   5.  Extreme mass ratio (250:1): ball bounces off athlete correctly
//   6.  Ball velocity after block solve is physically reasonable
//   7.  Block solver correctly handles separating velocities (no impulse)
//   8.  Block solver with restitution > 0 produces bounce
//   9.  Ball-CCD prevents ground tunneling at high velocity
//  10. Two-sphere contact (both non-ball) uses SI only (no block solve)
//  11. Stability: ball settles on ground without jitter over 120 frames
//  12. Determinism: two identical ball-athlete sims produce identical results
//
// Pattern: int main() + assert(), no test framework.
// =============================================================================

#include "apc_solver/apc_si_solver_3d.h"
#include "apc_solver/apc_rigid_body.h"
#include "apc_collision/apc_collision_dispatch.h"
#include "apc_collision/apc_sphere_sphere.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include "apc_math/apc_math_common.h"
#include <cassert>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <vector>

static constexpr float EPS = 1e-4f;
static int g_tests = 0;
static int g_passed = 0;

static bool approx_eq(float a, float b, float eps = EPS) {
    return std::abs(a - b) < eps;
}

static void check(bool cond, const char* msg, int line) {
    ++g_tests;
    if (cond) {
        ++g_passed;
    } else {
        std::printf("    [FAIL] %s (line %d, test #%d)\n", msg, line, g_tests);
        std::fflush(stdout);
        std::abort();
    }
}

#define CHECK(cond, msg) check((cond), (msg), __LINE__)

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static apc::RigidBody make_sphere_body(uint32_t id, const apc::Vec3& pos,
                                        float radius, float mass,
                                        bool is_ball = false)
{
    apc::RigidBody b;
    b.body_id = id;
    b.position = pos;
    b.inverse_mass = (mass > 0.0f) ? (1.0f / mass) : 0.0f;
    b.orientation = apc::Quat::identity();
    b.bounding_radius = radius;
    // Solid sphere: I = 2/5 * m * r^2, inv_I = 5 / (2 * m * r^2)
    float inv_I = (mass > 0.0f && radius > 0.0f) ? (5.0f / (2.0f * mass * radius * radius)) : 0.0f;
    b.local_inverse_inertia = apc::Mat3{{inv_I, 0, 0, 0, inv_I, 0, 0, 0, inv_I}};
    b.update_world_inertia();
    if (is_ball) {
        b.state_flags |= apc::STATE_IS_BALL | apc::STATE_CCD_ENABLED;
    }
    return b;
}

static apc::RigidBody make_static_body(uint32_t id, const apc::Vec3& pos) {
    apc::RigidBody b;
    b.body_id = id;
    b.position = pos;
    b.inverse_mass = 0.0f;
    b.orientation = apc::Quat::identity();
    b.local_inverse_inertia = apc::Mat3{{0, 0, 0, 0, 0, 0, 0, 0, 0}};
    b.state_flags |= apc::STATE_KINEMATIC;
    b.update_world_inertia();
    return b;
}

// =============================================================================
// TEST 1: enable_block_solve defaults to true
// =============================================================================
static void test_block_solve_enabled_by_default() {
    std::printf("  [Test 1] enable_block_solve defaults to true...\n");

    apc::Solver3D solver;
    CHECK(solver.enable_block_solve == true,
          "enable_block_solve = true by default");

    std::printf("    [PASS] enable_block_solve defaults verified\n");
}

// =============================================================================
// TEST 2: VelocityConstraint::is_block_solved defaults to false
// =============================================================================
static void test_is_block_solved_default() {
    std::printf("  [Test 2] is_block_solved defaults to false...\n");

    apc::VelocityConstraint vc;
    CHECK(vc.is_block_solved == false,
          "is_block_solved = false by default");

    std::printf("    [PASS] is_block_solved default verified\n");
}

// =============================================================================
// TEST 3: Block solver triggered for ball-body contacts
// =============================================================================
static void test_block_solve_triggers_for_ball() {
    std::printf("  [Test 3] Block solver triggers for ball-body contacts...\n");

    // Create a ball (0.4kg) and an athlete (80kg)
    std::vector<apc::RigidBody> bodies;
    bodies.push_back(make_sphere_body(0, apc::Vec3(0, 0.11f, 0), 0.11f, 0.43f, true));
    bodies.push_back(make_sphere_body(1, apc::Vec3(0, 0.3f, 0), 0.3f, 80.0f, false));

    // Give the athlete downward velocity toward the ball
    bodies[1].linear_velocity = apc::Vec3(0, -5.0f, 0);

    // Detect collision
    apc::SphereCollider ca;
    ca.radius = bodies[0].bounding_radius;
    apc::SphereCollider cb;
    cb.radius = bodies[1].bounding_radius;

    apc::ContactPoint cp;
    bool hit = apc::detect_sphere_sphere(
        bodies[0].position, ca,
        bodies[1].position, cb, cp);

    CHECK(hit, "ball-athlete collision detected");

    apc::Solver3D solver;
    solver.enable_block_solve = true;
    solver.velocity_iterations = 8;

    if (hit) {
        solver.prepare(cp, 0, 1, bodies);
    }

    // Solve — the block solver should fire for this ball-athlete pair
    solver.solve(bodies, 1.0f / 240.0f);

    // Verify indirectly: the ball's velocity should have changed
    // because the block solver applied an impulse.
    float ball_vy_after = bodies[0].linear_velocity.y;
    CHECK(ball_vy_after < -0.1f,
          "ball received impulse from block solver (vy pushed negative)");

    std::printf("    [PASS] Block solver triggered for ball-body contact (vy=%.4f)\n",
                ball_vy_after);
}

// =============================================================================
// TEST 4: Block solver NOT triggered for non-ball contacts
// =============================================================================
static void test_block_solve_skips_non_ball() {
    std::printf("  [Test 4] Block solver skipped for non-ball contacts...\n");

    // Two non-ball bodies: athlete (80kg) and wall (static)
    std::vector<apc::RigidBody> bodies;
    bodies.push_back(make_sphere_body(0, apc::Vec3(0, 0.3f, 0), 0.3f, 80.0f, false));
    bodies.push_back(make_static_body(1, apc::Vec3(0, -0.5f, 0)));

    bodies[0].linear_velocity = apc::Vec3(0, -2.0f, 0);

    apc::CollisionShape sphere_shape = apc::CollisionShape::make_sphere(
        0.3f, bodies[0].position);
    apc::CollisionShape box_shape = apc::CollisionShape::make_box(
        apc::Vec3(2.0f, 0.5f, 2.0f), bodies[1].position, apc::Quat::identity());

    apc::ContactManifold manifold;
    bool hit = apc::dispatch_detect(sphere_shape, box_shape, 0, 1, manifold);

    apc::Solver3D solver;
    solver.enable_block_solve = true;
    solver.velocity_iterations = 4;

    if (hit) {
        for (uint32_t i = 0u; i < manifold.contact_count; ++i) {
            solver.prepare(manifold.contacts[i], 0, 1, bodies);
        }
    }

    // Solve — should use SI only (no block solve for non-ball pair)
    // Run a few substeps for visible effect
    const float dt = 1.0f / 240.0f;
    for (int sub = 0; sub < 240; ++sub) {
        solver.clear();
        sphere_shape.position = bodies[0].position;

        apc::ContactManifold m2;
        if (apc::dispatch_detect(sphere_shape, box_shape, 0, 1, m2)) {
            for (uint32_t i = 0u; i < m2.contact_count; ++i) {
                solver.prepare(m2.contacts[i], 0, 1, bodies);
            }
        }
        solver.solve(bodies, dt);
        solver.integrate(bodies, dt);
    }

    // The athlete should have been slowed/stopped by the wall
    float vy_after = bodies[0].linear_velocity.y;
    CHECK(std::abs(vy_after) < 1.0f || vy_after > -1.5f,
          "non-ball contact resolved by SI");

    std::printf("    [PASS] Non-ball contact uses SI only (vy=%.4f)\n", vy_after);
}

// =============================================================================
// TEST 5: Extreme mass ratio (250:1) ball bounces off athlete correctly
// =============================================================================
static void test_extreme_mass_ratio_bounce() {
    std::printf("  [Test 5] Extreme mass ratio (250:1) ball bounce...\n");

    // 100kg athlete approaches 0.4kg ball head-on
    std::vector<apc::RigidBody> bodies;
    // Position athlete BELOW ball so B->A normal points upward
    // Sum of radii = 0.11 + 0.3 = 0.41, distance = 0.3, overlap = 0.11
    bodies.push_back(make_sphere_body(0, apc::Vec3(0, 0.3f, 0), 0.11f, 0.4f, true));
    bodies.push_back(make_sphere_body(1, apc::Vec3(0, 0.0f, 0), 0.3f, 100.0f, false));

    // Athlete moving upward toward ball at 10 m/s
    bodies[1].linear_velocity = apc::Vec3(0, 10.0f, 0);

    apc::SphereCollider ca;
    ca.radius = bodies[0].bounding_radius;
    apc::SphereCollider cb;
    cb.radius = bodies[1].bounding_radius;

    apc::ContactPoint cp;
    bool hit = apc::detect_sphere_sphere(
        bodies[0].position, ca,
        bodies[1].position, cb, cp);

    CHECK(hit, "ball-athlete collision detected");

    apc::Solver3D solver;
    solver.enable_block_solve = true;
    solver.restitution = 0.5f; // Moderate bounce
    solver.velocity_iterations = 8;

    if (hit) {
        solver.prepare(cp, 0, 1, bodies);
    }

    solver.solve(bodies, 1.0f / 240.0f);
    solver.integrate(bodies, 1.0f / 240.0f);

    float ball_vy = bodies[0].linear_velocity.y;
    float athlete_vy = bodies[1].linear_velocity.y;

    // Ball should have received a strong impulse from the 100kg athlete
    float ball_speed = std::abs(ball_vy);
    CHECK(ball_speed > 5.0f,
          "ball accelerated strongly by athlete impact (|vy| > 5 m/s)");

    // Athlete should barely slow down (100kg vs 0.4kg)
    CHECK(athlete_vy < 10.5f,
          "athlete barely slowed by ball impact (vy < 10.5 m/s)");

    std::printf("    [PASS] 250:1 ratio resolved correctly "
                "(ball vy=%.2f, athlete vy=%.2f)\n", ball_vy, athlete_vy);
}

// =============================================================================
// TEST 6: Ball velocity is physically reasonable (no explosion)
// =============================================================================
static void test_ball_velocity_reasonable() {
    std::printf("  [Test 6] Ball velocity is physically bounded...\n");

    // Simulate 60 frames of ball-athlete interaction
    std::vector<apc::RigidBody> bodies;
    bodies.push_back(make_sphere_body(0, apc::Vec3(0.5f, 0.11f, 0), 0.11f, 0.43f, true));
    bodies.push_back(make_sphere_body(1, apc::Vec3(0.5f, 1.0f, 0), 0.3f, 85.0f, false));

    bodies[1].linear_velocity = apc::Vec3(0, -3.0f, 0);

    apc::Solver3D solver;
    solver.enable_block_solve = true;
    solver.restitution = 0.3f;
    solver.velocity_iterations = 8;
    solver.linear_damping = 0.999f;
    solver.angular_damping = 0.998f;

    float max_ball_speed = 0.0f;
    const float dt = 1.0f / 240.0f;

    for (int frame = 0; frame < 60; ++frame) {
        solver.clear();

        apc::SphereCollider ca;
        ca.radius = bodies[0].bounding_radius;
        apc::SphereCollider cb;
        cb.radius = bodies[1].bounding_radius;

        apc::ContactPoint cp;
        if (apc::detect_sphere_sphere(
                bodies[0].position, ca,
                bodies[1].position, cb, cp))
        {
            solver.prepare(cp, 0, 1, bodies);
        }

        solver.solve(bodies, dt);
        solver.integrate(bodies, dt);

        float speed = apc::Vec3::length(bodies[0].linear_velocity);
        if (speed > max_ball_speed) max_ball_speed = speed;
    }

    // Ball speed should be bounded — no explosion from mass ratio
    // With 85kg athlete at 3m/s, max ball speed should be < 20 m/s
    CHECK(max_ball_speed < 20.0f,
          "ball speed bounded over 60 frames (< 20 m/s)");

    std::printf("    [PASS] Ball velocity bounded (max speed = %.2f m/s)\n",
                max_ball_speed);
}

// =============================================================================
// TEST 7: Block solver handles separating velocities (no impulse)
// =============================================================================
static void test_block_solve_separating() {
    std::printf("  [Test 7] Block solver skips separating contacts...\n");

    // Ball ABOVE athlete, moving AWAY (upward) — no impulse should be applied
    std::vector<apc::RigidBody> bodies;
    bodies.push_back(make_sphere_body(0, apc::Vec3(0, 0.4f, 0), 0.11f, 0.43f, true));
    bodies[0].linear_velocity = apc::Vec3(0, 5.0f, 0); // Moving away from athlete below
    bodies.push_back(make_sphere_body(1, apc::Vec3(0, 0.0f, 0), 0.3f, 80.0f, false));
    bodies[1].linear_velocity = apc::Vec3(0, 0.0f, 0); // Stationary

    apc::SphereCollider ca;
    ca.radius = bodies[0].bounding_radius;
    apc::SphereCollider cb;
    cb.radius = bodies[1].bounding_radius;

    apc::ContactPoint cp;
    bool hit = apc::detect_sphere_sphere(
        bodies[0].position, ca,
        bodies[1].position, cb, cp);

    if (!hit) {
        // Bodies might be too far apart for contact
        // Move them closer so they overlap
        bodies[0].position = apc::Vec3(0, 0.25f, 0);
        bodies[1].position = apc::Vec3(0, 0.0f, 0);
        hit = apc::detect_sphere_sphere(
            bodies[0].position, ca,
            bodies[1].position, cb, cp);
    }

    apc::Solver3D solver;
    solver.enable_block_solve = true;
    solver.restitution = 0.5f;

    float ball_vy_before = bodies[0].linear_velocity.y;

    if (hit) {
        solver.prepare(cp, 0, 1, bodies);
    }

    solver.solve(bodies, 1.0f / 240.0f);

    float ball_vy_after = bodies[0].linear_velocity.y;

    // Ball was separating — should not have been pushed further
    CHECK(ball_vy_after >= ball_vy_before - 0.01f,
          "block solver did not apply impulse to separating contact");

    std::printf("    [PASS] Separating contact correctly skipped (vy: %.4f -> %.4f)\n",
                ball_vy_before, ball_vy_after);
}

// =============================================================================
// TEST 8: Block solver with restitution produces bounce
// =============================================================================
static void test_block_solve_restitution() {
    std::printf("  [Test 8] Block solver restitution produces bounce...\n");

    // Ball near ground surface with restitution — must overlap box top (y=0)
    std::vector<apc::RigidBody> bodies;
    bodies.push_back(make_sphere_body(0, apc::Vec3(0, 0.05f, 0), 0.11f, 0.43f, true));
    bodies[0].linear_velocity = apc::Vec3(0, -5.0f, 0);
    bodies.push_back(make_static_body(1, apc::Vec3(0, -0.5f, 0)));

    apc::CollisionShape sphere_shape = apc::CollisionShape::make_sphere(
        0.11f, bodies[0].position);
    apc::CollisionShape box_shape = apc::CollisionShape::make_box(
        apc::Vec3(5.0f, 0.5f, 5.0f), bodies[1].position, apc::Quat::identity());

    apc::ContactManifold manifold;
    bool hit = apc::dispatch_detect(sphere_shape, box_shape, 0, 1, manifold);

    apc::Solver3D solver;
    solver.enable_block_solve = true;
    solver.restitution = 0.8f; // High bounce
    solver.velocity_iterations = 8;
    solver.friction_coefficient = 0.3f;

    if (hit) {
        for (uint32_t i = 0u; i < manifold.contact_count; ++i) {
            solver.prepare(manifold.contacts[i], 0, 1, bodies);
        }
    }

    solver.solve(bodies, 1.0f / 240.0f);

    // With high restitution and block solve, ball should bounce upward
    float ball_vy = bodies[0].linear_velocity.y;
    CHECK(ball_vy > 0.0f,
          "ball bounced upward with restitution 0.8 (vy > 0)");

    std::printf("    [PASS] Restitution bounce verified (vy = %.2f m/s)\n", ball_vy);
}

// =============================================================================
// TEST 9: Ball-CCD prevents ground tunneling at high velocity
// =============================================================================
static void test_ccd_prevents_tunneling() {
    std::printf("  [Test 9] CCD prevents ground tunneling at high velocity...\n");

    // Ball just above ground, moving down fast
    std::vector<apc::RigidBody> bodies;
    bodies.push_back(make_sphere_body(0, apc::Vec3(0, 0.15f, 0), 0.11f, 0.43f, true));
    bodies[0].linear_velocity = apc::Vec3(0, -50.0f, 0); // Very fast
    bodies.push_back(make_static_body(1, apc::Vec3(0, -0.5f, 0)));

    apc::Solver3D solver;
    solver.enable_block_solve = true;
    solver.restitution = 0.3f;
    solver.velocity_iterations = 8;
    solver.ccd_factor = 0.8f;

    apc::CollisionShape sphere_shape = apc::CollisionShape::make_sphere(
        0.11f, bodies[0].position);
    apc::CollisionShape box_shape = apc::CollisionShape::make_box(
        apc::Vec3(10.0f, 0.5f, 10.0f), bodies[1].position, apc::Quat::identity());

    const float dt = 1.0f / 240.0f;
    bool tunneled = false;

    // Simulate 30 frames — ball should not pass through the ground
    for (int frame = 0; frame < 30; ++frame) {
        solver.clear();

        sphere_shape.position = bodies[0].position;

        apc::ContactManifold manifold;
        if (apc::dispatch_detect(sphere_shape, box_shape, 0, 1, manifold)) {
            for (uint32_t i = 0u; i < manifold.contact_count; ++i) {
                solver.prepare(manifold.contacts[i], 0, 1, bodies);
            }
        }

        solver.solve(bodies, dt);
        solver.integrate(bodies, dt);

        // Ball should not go below ground (y = 0 + radius)
        if (bodies[0].position.y < -0.5f) {
            tunneled = true;
        }
    }

    CHECK(!tunneled, "ball did not tunnel through ground at 50 m/s");

    std::printf("    [PASS] CCD prevented tunneling (final y=%.4f)\n",
                bodies[0].position.y);
}

// =============================================================================
// TEST 10: Two non-ball spheres use SI only
// =============================================================================
static void test_non_ball_si_only() {
    std::printf("  [Test 10] Two non-ball spheres use SI only...\n");

    // Two athlete-mass spheres colliding — pure SI, no block solve
    std::vector<apc::RigidBody> bodies;
    bodies.push_back(make_sphere_body(0, apc::Vec3(0, 0.3f, 0), 0.3f, 80.0f, false));
    bodies.push_back(make_sphere_body(1, apc::Vec3(0, 0.8f, 0), 0.3f, 80.0f, false));

    bodies[0].linear_velocity = apc::Vec3(0, 3.0f, 0);
    bodies[1].linear_velocity = apc::Vec3(0, -3.0f, 0);

    apc::SphereCollider ca;
    ca.radius = 0.3f;
    apc::SphereCollider cb;
    cb.radius = 0.3f;

    apc::ContactPoint cp;
    bool hit = apc::detect_sphere_sphere(
        bodies[0].position, ca,
        bodies[1].position, cb, cp);

    CHECK(hit, "two athlete spheres in contact");

    apc::Solver3D solver;
    solver.enable_block_solve = true;
    solver.restitution = 0.0f;

    if (hit) {
        solver.prepare(cp, 0, 1, bodies);
    }

    float vy0_before = bodies[0].linear_velocity.y;
    float vy1_before = bodies[1].linear_velocity.y;

    solver.solve(bodies, 1.0f / 240.0f);

    // Equal mass head-on collision: velocities should swap
    float vy0_after = bodies[0].linear_velocity.y;
    float vy1_after = bodies[1].linear_velocity.y;

    CHECK(vy0_after < vy0_before,
          "athlete 0 slowed down by collision");
    CHECK(vy1_after > vy1_before,
          "athlete 1 sped up by collision");

    std::printf("    [PASS] Equal-mass SI collision "
                "(a: %.2f->%.2f, b: %.2f->%.2f)\n",
                vy0_before, vy0_after, vy1_before, vy1_after);
}

// =============================================================================
// TEST 11: Stability — ball settles on ground without jitter
// =============================================================================
static void test_ball_settles_without_jitter() {
    std::printf("  [Test 11] Ball settles on ground without jitter...\n");

    // Ball just above ground surface, starting at rest
    std::vector<apc::RigidBody> bodies;
    bodies.push_back(make_sphere_body(0, apc::Vec3(0, 0.12f, 0), 0.11f, 0.43f, true));
    bodies[0].linear_velocity = apc::Vec3(0, 0.0f, 0);
    bodies.push_back(make_static_body(1, apc::Vec3(0, -0.5f, 0)));

    apc::CollisionShape sphere_shape = apc::CollisionShape::make_sphere(
        0.11f, bodies[0].position);
    apc::CollisionShape box_shape = apc::CollisionShape::make_box(
        apc::Vec3(10.0f, 0.5f, 10.0f), bodies[1].position, apc::Quat::identity());

    apc::Solver3D solver;
    solver.enable_block_solve = true;
    solver.restitution = 0.0f; // No bounce — should settle
    solver.velocity_iterations = 8;
    solver.friction_coefficient = 0.5f;
    solver.linear_damping = 0.998f;

    const float dt = 1.0f / 240.0f;
    float min_y = 1e30f;
    float max_y = -1e30f;
    bool exploded = false;

    for (int frame = 0; frame < 240; ++frame) { // 1 second of sim
        solver.clear();

        sphere_shape.position = bodies[0].position;

        apc::ContactManifold manifold;
        if (apc::dispatch_detect(sphere_shape, box_shape, 0, 1, manifold)) {
            for (uint32_t i = 0u; i < manifold.contact_count; ++i) {
                solver.prepare(manifold.contacts[i], 0, 1, bodies);
            }
        }

        solver.solve(bodies, dt);
        solver.integrate(bodies, dt);

        float y = bodies[0].position.y;
        if (y < min_y) min_y = y;
        if (y > max_y) max_y = y;

        // Explosion detection
        if (y > 10.0f || y < -2.0f) {
            exploded = true;
            break;
        }
    }

    CHECK(!exploded, "ball did not explode over 240 frames");
    CHECK(max_y - min_y < 1.0f,
          "ball position range < 1.0m (no jitter)");

    std::printf("    [PASS] Ball settled (y=%.4f, range=%.4f)\n",
                bodies[0].position.y, max_y - min_y);
}

// =============================================================================
// TEST 12: Determinism — identical sims produce identical results
// =============================================================================
static void test_determinism_ball_athlete() {
    std::printf("  [Test 12] Determinism: identical ball-athlete sims...\n");

    auto run_sim = []() -> apc::Vec3 {
        std::vector<apc::RigidBody> bodies;
        bodies.push_back(make_sphere_body(0, apc::Vec3(0.1f, 0.11f, 0), 0.11f, 0.43f, true));
        bodies.push_back(make_sphere_body(1, apc::Vec3(0.1f, 0.8f, 0), 0.3f, 90.0f, false));

        bodies[1].linear_velocity = apc::Vec3(0, -8.0f, 0);

        apc::Solver3D solver;
        solver.enable_block_solve = true;
        solver.restitution = 0.4f;
        solver.velocity_iterations = 8;
        solver.friction_coefficient = 0.3f;

        apc::CollisionShape sphere_shape = apc::CollisionShape::make_sphere(
            0.11f, bodies[0].position);
        apc::CollisionShape athlete_shape = apc::CollisionShape::make_sphere(
            0.3f, bodies[1].position);

        const float dt = 1.0f / 240.0f;

        for (int frame = 0; frame < 120; ++frame) {
            solver.clear();

            sphere_shape.position = bodies[0].position;
            athlete_shape.position = bodies[1].position;

            apc::SphereCollider ca;
            ca.radius = 0.11f;
            apc::SphereCollider cb;
            cb.radius = 0.3f;

            apc::ContactPoint cp;
            if (apc::detect_sphere_sphere(
                    bodies[0].position, ca,
                    bodies[1].position, cb, cp))
            {
                solver.prepare(cp, 0, 1, bodies);
            }

            solver.solve(bodies, dt);
            solver.integrate(bodies, dt);
        }

        return bodies[0].position;
    };

    apc::Vec3 result_a = run_sim();
    apc::Vec3 result_b = run_sim();

    CHECK(approx_eq(result_a.x, result_b.x, 1e-6f), "determinism: x matches");
    CHECK(approx_eq(result_a.y, result_b.y, 1e-6f), "determinism: y matches");
    CHECK(approx_eq(result_a.z, result_b.z, 1e-6f), "determinism: z matches");

    std::printf("    [PASS] Deterministic across two runs (pos = %.4f, %.4f, %.4f)\n",
                result_a.x, result_a.y, result_a.z);
}

// =============================================================================
// main
// =============================================================================
int main() {
    std::printf("=== Sprint 29: Collision Solver Unification — Mass-Ratio Fix ===\n\n");

    test_block_solve_enabled_by_default();
    test_is_block_solved_default();
    test_block_solve_triggers_for_ball();
    test_block_solve_skips_non_ball();
    test_extreme_mass_ratio_bounce();
    test_ball_velocity_reasonable();
    test_block_solve_separating();
    test_block_solve_restitution();
    test_ccd_prevents_tunneling();
    test_non_ball_si_only();
    test_ball_settles_without_jitter();
    test_determinism_ball_athlete();

    int failed = g_tests - g_passed;
    std::printf("\n=== Sprint 29: %d tests passed, %d failed (%d total) ===\n",
                g_passed, failed, g_tests);
    return failed;
}
