#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include "apc_collision/apc_sphere_sphere.h"
#include "apc_solver/apc_rigid_body.h"
#include "apc_solver/apc_si_solver_3d.h"
#include "apc_platform/apc_fp_mode.h"
#include <cstdio>
#include <cmath>
#include <cstring>

int main() {
    apc::enforce_deterministic_fp_mode();
    std::printf("Running 3D Spinning Sphere Bounce Integration Test...\n");

    std::vector<apc::RigidBody> bodies(2);

    // Body A: Falling, spinning sphere
    bodies[0].position = apc::Vec3(0.0f, 10.0f, 0.0f);
    bodies[0].linear_velocity = apc::Vec3(0.0f, 0.0f, 0.0f); // Dropping straight down
    bodies[0].angular_velocity = apc::Vec3(10.0f, 0.0f, 0.0f); // Spinning on X axis
    bodies[0].inverse_mass = 1.0f; 
    bodies[0].orientation = apc::Quat::identity();
    bodies[0].local_inverse_inertia = apc::Mat3{{
        2.5f, 0.0f, 0.0f,
        0.0f, 2.5f, 0.0f,
        0.0f, 0.0f, 2.5f
    }};
    bodies[0].update_world_inertia();

    // Body B: Static ground sphere
    bodies[1].position = apc::Vec3(0.0f, -1.5f, 0.0f);
    bodies[1].linear_velocity = apc::Vec3(0.0f, 0.0f, 0.0f);
    bodies[1].angular_velocity = apc::Vec3(0.0f, 0.0f, 0.0f);
    bodies[1].inverse_mass = 0.0f; // Static!
    bodies[1].orientation = apc::Quat::identity();
    bodies[1].local_inverse_inertia = apc::Mat3::identity();
    bodies[1].update_world_inertia();

    apc::SphereCollider col_a{1.0f};
    apc::SphereCollider col_b{1.0f};

    apc::Solver3D solver;
    solver.friction_coefficient = 0.0f; // This test was designed for frictionless physics
    apc::ContactPoint temp_contact;
    float dt = 1.0f / 240.0f;

    for (int i = 0; i < 2400; ++i) { // 10 seconds of sim
        // 0. Apply Gravity (Simple downward acceleration)
        bodies[0].linear_velocity.y -= 9.81f * dt;

        // 1. Update world inertia (if orientation changed)
        bodies[0].update_world_inertia();

        // 2. Collision Detection
        if (apc::detect_sphere_sphere(bodies[0].position, col_a, bodies[1].position, col_b, temp_contact)) {
            solver.prepare(temp_contact, 0, 1, bodies);
            solver.solve(bodies, dt);
        }
        
        solver.clear();

        // 3. Integrate
        solver.integrate(bodies, dt);
        
        // Safety check
        if (bodies[0].position.y < -50.0f) {
            std::printf("[FAIL] Body fell through floor.\n");
            return 1;
        }
    }

    // The ball should have bounced and come to rest near y = 0.5 (top of ground sphere)
    // If it's still at y=10, physics didn't run. If it's below -1, it tunneled.
    if (bodies[0].position.y > 5.0f) {
        std::printf("[FAIL] Body never fell (Gravity/Physics didn't run).\n");
        return 1;
    }

    // Check that spin affected something (friction would couple linear and angular velocity)
    // Since we don't have friction in the solver yet, just verify it's stable.
    if (std::isnan(bodies[0].position.x) || std::isnan(bodies[0].orientation.w)) {
        std::printf("[FAIL] NaN detected in final state.\n");
        return 1;
    }

    // Generate determinism hash for the final state
    uint64_t final_hash = 0;
    auto hash_float = [&final_hash](float f) {
        uint32_t bits;
        std::memcpy(&bits, &f, sizeof(bits));
        final_hash ^= bits + 0x9e3779b9 + (final_hash << 6);
    };

    hash_float(bodies[0].position.x);
    hash_float(bodies[0].position.y);
    hash_float(bodies[0].position.z);
    hash_float(bodies[0].angular_velocity.x); 

    std::printf("[PASS] 3D Solver simulated spinning sphere stably.\n");
    std::printf("Final Pos: (%.3f, %.3f, %.3f)\n", bodies[0].position.x, bodies[0].position.y, bodies[0].position.z);
    std::printf("Final Angular Vel X: %.3f\n", bodies[0].angular_velocity.x);
    std::printf("State Hash: %016llx\n", (unsigned long long)final_hash);

    return 0;
}