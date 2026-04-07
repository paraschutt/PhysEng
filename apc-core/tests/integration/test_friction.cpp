// =============================================================================
// test_friction.cpp — Friction Solver Integration Test
// =============================================================================
// Verifies that the Coulomb friction model in Solver3D correctly couples
// angular and linear velocity.  A spinning sphere drops onto a static
// ground sphere:
//   - With friction (mu > 0): the spin creates tangential velocity at the
//     contact, and friction converts it into lateral linear displacement.
//   - Without friction (mu = 0): the sphere slides freely at the contact
//     point regardless of spin, producing no lateral displacement.
//
// The test runs both scenarios and compares the final X position.
// =============================================================================

#include "apc_solver/apc_rigid_body.h"
#include "apc_solver/apc_si_solver_3d.h"
#include "apc_collision/apc_sphere_sphere.h"
#include "apc_platform/apc_fp_mode.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include "apc_math/apc_mat3.h"
#include <cstdio>
#include <cmath>
#include <cstring>
#include <vector>

using namespace apc;

// ---------------------------------------------------------------------------
// Simulation parameters
// ---------------------------------------------------------------------------
static constexpr int   SIM_STEPS       = 2400;           // 10 s at 240 Hz
static constexpr float DT              = 1.0f / 240.0f;
static constexpr float GRAVITY         = 9.81f;
static constexpr float SPHERE_RADIUS   = 1.0f;
// Use a large ground sphere so the falling sphere can roll laterally
// without escaping the collision zone.
static constexpr float GROUND_RADIUS   = 100.0f;
static constexpr float GROUND_Y        = -(GROUND_RADIUS + SPHERE_RADIUS - 0.01f);
static constexpr float DROP_HEIGHT     = 5.0f;            // Falling sphere start Y
static constexpr float SPIN_Z          = 10.0f;           // Angular vel around Z axis

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

static void hash_float(uint64_t& h, float f) {
    uint32_t bits;
    std::memcpy(&bits, &f, sizeof(bits));
    h ^= (uint64_t)bits + 0x9e3779b9 + (h << 6);
}

// ---------------------------------------------------------------------------
// Run a single physics simulation and return the final position of body 0
// ---------------------------------------------------------------------------
static Vec3 run_simulation(float friction_coeff, float spin_z) {
    std::vector<RigidBody> bodies(2);

    // Body 0: Spinning, falling sphere
    bodies[0].position         = Vec3(0.0f, DROP_HEIGHT, 0.0f);
    bodies[0].linear_velocity  = Vec3(0.0f, 0.0f, 0.0f);
    bodies[0].angular_velocity = Vec3(0.0f, 0.0f, spin_z);
    bodies[0].inverse_mass     = 1.0f;
    bodies[0].orientation      = Quat::identity();
    // Uniform sphere: I = 2/5 * m * r^2,  m = 1 → I = 0.4,  inv_I = 2.5
    bodies[0].local_inverse_inertia = Mat3{{
        2.5f, 0.0f, 0.0f,
        0.0f, 2.5f, 0.0f,
        0.0f, 0.0f, 2.5f
    }};
    bodies[0].update_world_inertia();

    // Body 1: Static ground sphere (very large → effectively flat surface)
    bodies[1].position         = Vec3(0.0f, GROUND_Y, 0.0f);
    bodies[1].linear_velocity  = Vec3(0.0f, 0.0f, 0.0f);
    bodies[1].angular_velocity = Vec3(0.0f, 0.0f, 0.0f);
    bodies[1].inverse_mass     = 0.0f;  // Static
    bodies[1].orientation      = Quat::identity();
    bodies[1].local_inverse_inertia = Mat3::identity();
    bodies[1].update_world_inertia();

    SphereCollider col_a{ SPHERE_RADIUS };
    SphereCollider col_b{ GROUND_RADIUS };

    Solver3D solver;
    solver.friction_coefficient = friction_coeff;

    ContactPoint contact;

    for (int step = 0; step < SIM_STEPS; ++step) {
        // 1. Apply gravity
        bodies[0].linear_velocity.y -= GRAVITY * DT;

        // 2. Update world inertia (orientation may have changed from last frame)
        bodies[0].update_world_inertia();

        // 3. Collision detection
        if (detect_sphere_sphere(bodies[0].position, col_a,
                                 bodies[1].position, col_b,
                                 contact)) {
            solver.prepare(contact, 0, 1, bodies);
            solver.solve(bodies, DT);
        }

        solver.clear();

        // 4. Integrate positions
        solver.integrate(bodies, DT);
    }

    return bodies[0].position;
}

int main() {
    enforce_deterministic_fp_mode();

    int passed = 0;
    int total  = 5;
    uint64_t state_hash = 0;

    std::printf("=== Friction Solver Integration Test ===\n\n");

    // -----------------------------------------------------------------------
    // Run both simulations
    // -----------------------------------------------------------------------
    Vec3 pos_friction = run_simulation(0.4f, SPIN_Z);    // With friction
    Vec3 pos_no_fric  = run_simulation(0.0f, SPIN_Z);    // Without friction

    std::printf("With friction   (mu=0.4): final pos = (%.6f, %.6f, %.6f)\n",
                pos_friction.x, pos_friction.y, pos_friction.z);
    std::printf("Without friction (mu=0.0): final pos = (%.6f, %.6f, %.6f)\n",
                pos_no_fric.x, pos_no_fric.y, pos_no_fric.z);

    float x_friction = pos_friction.x;
    float x_no_fric  = pos_no_fric.x;

    // -----------------------------------------------------------------------
    // Test 1: With friction, the sphere should have non-zero X displacement
    // -----------------------------------------------------------------------
    {
        bool ok = (std::abs(x_friction) > 0.01f);
        std::printf("[%s] Test 1: With friction, |X| = %.6f > 0.01 (spin pushes laterally)\n",
                     ok ? "PASS" : "FAIL", std::abs(x_friction));
        if (ok) ++passed;
        hash_float(state_hash, x_friction);
    }

    // -----------------------------------------------------------------------
    // Test 2: Without friction, the sphere should NOT move laterally
    // -----------------------------------------------------------------------
    {
        bool ok = (std::abs(x_no_fric) < 0.001f);
        std::printf("[%s] Test 2: Without friction, |X| = %.6f < 0.001 (no lateral drift)\n",
                     ok ? "PASS" : "FAIL", std::abs(x_no_fric));
        if (ok) ++passed;
        hash_float(state_hash, x_no_fric);
    }

    // -----------------------------------------------------------------------
    // Test 3: Friction case should have significantly MORE X displacement
    // -----------------------------------------------------------------------
    {
        bool ok = (std::abs(x_friction) > std::abs(x_no_fric) * 5.0f + 0.01f);
        std::printf("[%s] Test 3: |X_friction| (%.4f) >> |X_no_fric| (%.6f)\n",
                     ok ? "PASS" : "FAIL", std::abs(x_friction), std::abs(x_no_fric));
        if (ok) ++passed;
    }

    // -----------------------------------------------------------------------
    // Test 4: Friction with ZERO spin → sphere should NOT move laterally
    // -----------------------------------------------------------------------
    // If there is no angular velocity, friction has no tangential velocity
    // to oppose, so no lateral displacement should occur.
    {
        Vec3 pos_no_spin = run_simulation(0.4f, 0.0f);
        bool ok = (std::abs(pos_no_spin.x) < 0.001f);
        std::printf("[%s] Test 4: Friction + zero spin → |X| = %.6f < 0.001\n",
                     ok ? "PASS" : "FAIL", std::abs(pos_no_spin.x));
        if (ok) ++passed;
        hash_float(state_hash, pos_no_spin.x);
    }

    // -----------------------------------------------------------------------
    // Test 5: Friction is deterministic — run same scenario twice
    // -----------------------------------------------------------------------
    {
        Vec3 pos_run1 = run_simulation(0.4f, SPIN_Z);
        Vec3 pos_run2 = run_simulation(0.4f, SPIN_Z);

        bool ok = (pos_run1.x == pos_run2.x)
               && (pos_run1.y == pos_run2.y)
               && (pos_run1.z == pos_run2.z);
        std::printf("[%s] Test 5: Determinism — two identical runs match exactly\n",
                     ok ? "PASS" : "FAIL");
        if (ok) ++passed;
        hash_float(state_hash, pos_run1.x);
    }

    // -----------------------------------------------------------------------
    // Summary & state hash
    // -----------------------------------------------------------------------
    std::printf("\n=== Friction Test Results: %d/%d passed ===\n", passed, total);

    uint32_t final_hash = fnv1a_bytes(&state_hash, sizeof(state_hash));
    std::printf("State Hash: 0x%08x\n", final_hash);

    return (passed == total) ? 0 : 1;
}
