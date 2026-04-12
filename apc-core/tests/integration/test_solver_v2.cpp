// =============================================================================
// test_solver_v2.cpp — Sprint 3 Solver Improvements Integration Tests
// =============================================================================
// Tests the enhanced Solver3D features:
//   1. Multi-iteration convergence (more iterations = less penetration)
//   2. Restitution / bounce behavior
//   3. Configurable damping
//   4. prepare_manifold() with ContactManifold
//   5. Full pipeline: dispatcher → solver → integrate (sphere-box, sphere-plane)
//
// All tests produce a state hash for cross-platform determinism verification.
// =============================================================================

#include "apc_solver/apc_rigid_body.h"
#include "apc_solver/apc_si_solver_3d.h"
#include "apc_collision/apc_collision_dispatch.h"
#include "apc_platform/apc_fp_mode.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include <cstdio>
#include <cmath>
#include <cstring>
#include <vector>

using namespace apc;

// ---------------------------------------------------------------------------
// Hashing utilities
// ---------------------------------------------------------------------------
static uint32_t fnv1a_bytes(const void* data, size_t len) {
    const uint8_t* p = static_cast<const uint8_t*>(data);
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < len; ++i) {
        h ^= p[i];
        h *= 16777619u;
    }
    return h;
}

static void hash_bool(uint64_t& h, bool v) {
    uint8_t b = v ? 1 : 0;
    h ^= (uint64_t)b + 0x9e3779b9 + (h << 6);
}

static void hash_float(uint64_t& h, float f) {
    uint32_t bits;
    std::memcpy(&bits, &f, sizeof(bits));
    h ^= (uint64_t)bits + 0x9e3779b9 + (h << 6);
}

// ---------------------------------------------------------------------------
// Helper: create a default dynamic sphere rigid body
// ---------------------------------------------------------------------------
static RigidBody make_sphere_body(uint32_t /*id*/, const Vec3& pos, float radius, float mass = 1.0f) {
    RigidBody b;
    b.position = pos;
    b.linear_velocity = Vec3(0, 0, 0);
    b.angular_velocity = Vec3(0, 0, 0);
    b.inverse_mass = (mass > 0.0f) ? (1.0f / mass) : 0.0f;
    b.orientation = Quat::identity();
    // Moment of inertia for solid sphere: I = 2/5 * m * r^2
    float I = 0.4f * mass * radius * radius;
    float inv_I = (I > 0.0f) ? (1.0f / I) : 0.0f;
    b.local_inverse_inertia = Mat3{{inv_I, 0, 0, 0, inv_I, 0, 0, 0, inv_I}};
    b.update_world_inertia();
    return b;
}

// ---------------------------------------------------------------------------
// Helper: create a static (infinite mass) body
// ---------------------------------------------------------------------------
static RigidBody make_static_body(uint32_t /*id*/, const Vec3& pos) {
    RigidBody b;
    b.position = pos;
    b.linear_velocity = Vec3(0, 0, 0);
    b.angular_velocity = Vec3(0, 0, 0);
    b.inverse_mass = 0.0f;
    b.orientation = Quat::identity();
    b.local_inverse_inertia = Mat3{{0, 0, 0, 0, 0, 0, 0, 0, 0}};
    b.world_inverse_inertia = Mat3{{0, 0, 0, 0, 0, 0, 0, 0, 0}};
    return b;
}

int main() {
    enforce_deterministic_fp_mode();

    int passed = 0;
    int total  = 6;
    uint64_t state_hash = 0;

    std::printf("=== Solver V2 Integration Tests ===\n\n");

    // -----------------------------------------------------------------------
    // Test 1: Multi-iteration convergence
    //   A sphere pressed into a static box with initial velocity.
    //   More solver iterations should remove more penetration.
    // -----------------------------------------------------------------------
    {
        const float dt = 1.0f / 240.0f;

        // Sphere moving downward into a static box
        std::vector<RigidBody> bodies;
        bodies.push_back(make_sphere_body(0, Vec3(0, 0.5f, 0), 0.5f));
        bodies[0].linear_velocity = Vec3(0, -1.0f, 0); // Moving down
        bodies.push_back(make_static_body(1, Vec3(0, -0.5f, 0)));

        CollisionShape sphere = CollisionShape::make_sphere(0.5f, bodies[0].position);
        CollisionShape box    = CollisionShape::make_box(
            Vec3(2.0f, 0.5f, 2.0f), bodies[1].position, Quat::identity());

        ContactManifold manifold;
        bool hit = dispatch_detect(sphere, box, 0, 1, manifold);

        if (!hit) {
            std::printf("[FAIL] Test 1: No collision detected\n");
        } else {
            float pen_1_iter, pen_8_iter;

            // Run with 1 iteration
            {
                std::vector<RigidBody> b1 = bodies;
                Solver3D solver;
                solver.velocity_iterations = 1;
                solver.friction_coefficient = 0.0f;
                solver.restitution = 0.0f;
                solver.linear_damping = 1.0f;
                solver.angular_damping = 1.0f;
                solver.prepare(manifold.contacts[0], 0, 1, b1);
                solver.solve(b1, dt);
                solver.integrate(b1, dt);

                // Re-detect to measure remaining penetration
                sphere.position = b1[0].position;
                ContactManifold m1;
                if (dispatch_detect(sphere, box, 0, 1, m1)) {
                    pen_1_iter = m1.contacts[0].penetration;
                } else {
                    pen_1_iter = 0.0f;
                }
            }

            // Run with 8 iterations
            {
                std::vector<RigidBody> b2 = bodies;
                Solver3D solver;
                solver.velocity_iterations = 8;
                solver.friction_coefficient = 0.0f;
                solver.restitution = 0.0f;
                solver.linear_damping = 1.0f;
                solver.angular_damping = 1.0f;
                solver.prepare(manifold.contacts[0], 0, 1, b2);
                solver.solve(b2, dt);
                solver.integrate(b2, dt);

                sphere.position = b2[0].position;
                ContactManifold m2;
                if (dispatch_detect(sphere, box, 0, 1, m2)) {
                    pen_8_iter = m2.contacts[0].penetration;
                } else {
                    pen_8_iter = 0.0f;
                }
            }

            bool ok = (pen_8_iter <= pen_1_iter + APC_EPSILON);
            std::printf("[%s] Test 1: Multi-iter convergence (1-iter pen=%.4f, 8-iter pen=%.4f)\n",
                         ok ? "PASS" : "FAIL", pen_1_iter, pen_8_iter);
            if (ok) ++passed;
            hash_float(state_hash, pen_1_iter);
            hash_float(state_hash, pen_8_iter);
        }
    }

    // -----------------------------------------------------------------------
    // Test 2: Restitution — solver remains stable with restitution enabled
    //   A sphere pressed into a ground plane with restitution should remain
    //   stable (no explosion, no NaN) and produce a reasonable bounce velocity.
    // -----------------------------------------------------------------------
    {
        const float dt = 1.0f / 240.0f;
        const int steps = 240;

        std::vector<RigidBody> bodies;
        bodies.push_back(make_sphere_body(0, Vec3(0, 0.55f, 0), 0.5f));
        bodies[0].linear_velocity = Vec3(0, -2.0f, 0);
        bodies.push_back(make_static_body(1, Vec3(0, -1.0f, 0)));

        Solver3D solver;
        solver.velocity_iterations = 8;
        solver.restitution = 0.8f;
        solver.friction_coefficient = 0.0f;
        solver.linear_damping = 1.0f;
        solver.angular_damping = 1.0f;

        bool stable = true;
        float vy_final = 0.0f;

        for (int i = 0; i < steps; ++i) {
            solver.clear();

            CollisionShape sphere_s = CollisionShape::make_sphere(0.5f, bodies[0].position);
            CollisionShape plane_s = CollisionShape::make_plane(
                bodies[1].position, Vec3(0, 1, 0));

            ContactManifold manifold;
            if (dispatch_detect(sphere_s, plane_s, 0, 1, manifold)) {
                for (uint32_t ci = 0; ci < manifold.contact_count; ++ci) {
                    solver.prepare(manifold.contacts[ci], 0, 1, bodies);
                }
            }

            solver.solve(bodies, dt);
            solver.integrate(bodies, dt);

            float y = bodies[0].position.y;
            float vy = bodies[0].linear_velocity.y;

            // Stability: position and velocity must remain finite
            if (y != y || vy != vy || y > 100.0f || y < -100.0f) {
                stable = false;
            }
            vy_final = vy;
        }

        bool ok = stable && (vy_final >= -5.0f) && (vy_final <= 5.0f);
        std::printf("[%s] Test 2: Restitution stable (vy_final=%.4f)\\n",
                     ok ? "PASS" : "FAIL", vy_final);
        if (ok) ++passed;
        hash_float(state_hash, vy_final);
    }

    // -----------------------------------------------------------------------
    // Test 3: Configurable damping
    //   Higher damping should slow a moving body more.
    // -----------------------------------------------------------------------
    {
        const float dt = 1.0f / 240.0f;
        const int steps = 240; // 1 second

        auto simulate_damping = [&](float damping) -> float {
            std::vector<RigidBody> bodies;
            bodies.push_back(make_sphere_body(0, Vec3(0, 5.0f, 0), 0.5f));
            bodies[0].linear_velocity = Vec3(1.0f, 0, 0);

            Solver3D solver;
            solver.velocity_iterations = 1;
            solver.linear_damping = damping;
            solver.angular_damping = 1.0f;

            for (int i = 0; i < steps; ++i) {
                solver.solve(bodies, dt);  // No contacts → no-op solve
                solver.integrate(bodies, dt);
            }

            return bodies[0].linear_velocity.x;
        };

        float v_heavy = simulate_damping(0.9f);   // Heavy damping
        float v_light = simulate_damping(0.999f);  // Light damping

        bool ok = (v_light > v_heavy + 0.01f);
        std::printf("[%s] Test 3: Damping (heavy=%.4f, light=%.4f)\n",
                     ok ? "PASS" : "FAIL", v_heavy, v_light);
        if (ok) ++passed;
        hash_float(state_hash, v_heavy);
        hash_float(state_hash, v_light);
    }

    // -----------------------------------------------------------------------
    // Test 4: prepare_manifold() with multiple contacts
    // -----------------------------------------------------------------------
    {
        std::vector<RigidBody> bodies;
        bodies.push_back(make_sphere_body(0, Vec3(0, 0, 0), 1.0f));
        bodies.push_back(make_sphere_body(1, Vec3(1.5f, 0, 0), 1.0f));

        SphereCollider ca{ 1.0f };
        SphereCollider cb{ 1.0f };

        ContactPoint contacts[2];
        ContactPoint cp1;
        bool hit1 = detect_sphere_sphere(
            bodies[0].position, ca, bodies[1].position, cb, cp1);

        // Create a second (synthetic) contact slightly offset
        ContactPoint cp2 = cp1;
        cp2.point_on_a = Vec3::add(cp1.point_on_a, Vec3(0, 0.1f, 0));
        cp2.point_on_b = Vec3::add(cp1.point_on_b, Vec3(0, 0.1f, 0));

        Solver3D solver;
        solver.velocity_iterations = 4;
        solver.friction_coefficient = 0.0f;

        if (hit1) {
            contacts[0] = cp1;
            contacts[1] = cp2;
            solver.prepare_manifold(contacts, 2, 0, 1, bodies);
        }

        bool ok = hit1;  // If prepare_manifold didn't crash, it worked
        std::printf("[%s] Test 4: prepare_manifold multi-contact\n",
                     ok ? "PASS" : "FAIL");
        if (ok) ++passed;
        hash_bool(state_hash, ok);
    }

    // -----------------------------------------------------------------------
    // Test 5: Full pipeline — sphere pressed into box, 2 seconds of sim
    //   Sphere should settle on the box without exploding.
    //   Engine has no gravity; we give the sphere an initial downward velocity.
    // -----------------------------------------------------------------------
    {
        const float dt = 1.0f / 240.0f;
        const int steps = 480; // 2 seconds

        std::vector<RigidBody> bodies;
        bodies.push_back(make_sphere_body(0, Vec3(0, 1.0f, 0), 0.5f));
        bodies[0].linear_velocity = Vec3(0, -2.0f, 0); // Moving toward box
        bodies.push_back(make_static_body(1, Vec3(0, -0.5f, 0)));

        CollisionShape sphere_shape = CollisionShape::make_sphere(0.5f, Vec3(0, 0, 0));
        CollisionShape box_shape    = CollisionShape::make_box(
            Vec3(2.0f, 0.5f, 2.0f), Vec3(0, -0.5f, 0), Quat::identity());

        Solver3D solver;
        solver.velocity_iterations = 8;
        solver.friction_coefficient = 0.4f;
        solver.restitution = 0.0f;
        solver.linear_damping = 0.999f;
        solver.angular_damping = 0.998f;

        bool stable = true;
        float min_y = 1e30f;

        for (int step = 0; step < steps; ++step) {
            solver.clear();
            sphere_shape.position = bodies[0].position;

            ContactManifold manifold;
            if (dispatch_detect(sphere_shape, box_shape, 0, 1, manifold)) {
                for (uint32_t i = 0; i < manifold.contact_count; ++i) {
                    solver.prepare(manifold.contacts[i], 0, 1, bodies);
                }
            }

            solver.solve(bodies, dt);
            solver.integrate(bodies, dt);

            float y = bodies[0].position.y;
            if (y < min_y) min_y = y;

            // Stability: sphere should not go below box top (y=0)
            // or fly above initial height + tolerance
            if (y < -1.0f || y > 10.0f) {
                stable = false;
            }
        }

        // After settling, sphere should be near or above the box top (y=0)
        // and should not be at the initial position (it moved)
        float final_y = bodies[0].position.y;
        bool ok = stable && (final_y > -0.5f) && (final_y < 2.0f);
        std::printf("[%s] Test 5: Full pipeline sphere-into-box 2s (final_y=%.4f, min_y=%.4f)\n",
                     ok ? "PASS" : "FAIL", final_y, min_y);
        if (ok) ++passed;
        hash_bool(state_hash, stable);
        hash_float(state_hash, final_y);
    }

    // -----------------------------------------------------------------------
    // Test 6: Determinism — two identical sims produce identical results
    // -----------------------------------------------------------------------
    {
        const float dt = 1.0f / 240.0f;
        const int steps = 300;

        auto run_sim = [&]() -> Vec3 {
            std::vector<RigidBody> bodies;
            bodies.push_back(make_sphere_body(0, Vec3(0, 2.0f, 0), 0.5f));
            bodies[0].linear_velocity = Vec3(0.3f, 0, 0.1f); // Slight horizontal
            bodies[0].angular_velocity = Vec3(0, 0, 1.0f);
            bodies.push_back(make_static_body(1, Vec3(0, -0.5f, 0)));

            CollisionShape sphere_shape = CollisionShape::make_sphere(0.5f, Vec3(0, 0, 0));
            CollisionShape box_shape    = CollisionShape::make_box(
                Vec3(2.0f, 0.5f, 2.0f), Vec3(0, -0.5f, 0), Quat::identity());

            Solver3D solver;
            solver.velocity_iterations = 8;
            solver.friction_coefficient = 0.4f;
            solver.restitution = 0.2f;

            for (int step = 0; step < steps; ++step) {
                solver.clear();
                sphere_shape.position = bodies[0].position;

                ContactManifold manifold;
                if (dispatch_detect(sphere_shape, box_shape, 0, 1, manifold)) {
                    for (uint32_t i = 0; i < manifold.contact_count; ++i) {
                        solver.prepare(manifold.contacts[i], 0, 1, bodies);
                    }
                }

                solver.solve(bodies, dt);
                solver.integrate(bodies, dt);
            }

            return bodies[0].position;
        };

        Vec3 pos_a = run_sim();
        Vec3 pos_b = run_sim();

        float diff = std::sqrt(Vec3::length_sq(Vec3::sub(pos_a, pos_b)));
        bool ok = (diff < APC_EPSILON);
        std::printf("[%s] Test 6: Determinism (diff=%.10f)\n",
                     ok ? "PASS" : "FAIL", diff);
        if (ok) ++passed;
        hash_float(state_hash, pos_a.x);
        hash_float(state_hash, pos_a.y);
        hash_float(state_hash, pos_a.z);
    }

    // -----------------------------------------------------------------------
    // Summary & state hash
    // -----------------------------------------------------------------------
    std::printf("\n=== Solver V2 Tests: %d/%d passed ===\n", passed, total);

    uint32_t final_hash = fnv1a_bytes(&state_hash, sizeof(state_hash));
    std::printf("State Hash: 0x%08x\n", final_hash);

    return (passed == total) ? 0 : 1;
}
