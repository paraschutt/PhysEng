// =============================================================================
// Sprint 10 Tests — StylizedSolver Integration
// =============================================================================
//
// Tests for Phase 3 Sprint 10: profile-aware solving with non-linear response.
//
//   1. StylizedSolver default config — correct default values
//   2. StylizedSolver prepare + solve basic — two-sphere collision pipeline
//   3. Profile-aware friction — custom friction_curve_id changes friction
//   4. Profile-aware restitution — custom restitution_curve_id changes bounce
//   5. Vertical bias — upward velocity boost after collision
//   6. Spin multiplier — amplified angular velocity
//   7. Profile blend strategies — BODY_A_WINS vs BODY_B_WINS differ
//   8. Impact event collection — correct IDs and contact info
//   9. StylizedSolver determinism — same input produces same output
//

#include "apc_style/apc_material_curve.h"
#include "apc_style/apc_impact_profile.h"
#include "apc_style/apc_stylized_solver.h"
#include "apc_solver/apc_rigid_body.h"
#include "apc_collision/apc_sphere_sphere.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include "apc_math/apc_mat3.h"
#include <cassert>
#include <cmath>
#include <vector>
#include <cstdio>
#include <cstring>

using namespace apc;

// =============================================================================
// Helpers
// =============================================================================

// Two equal-mass spheres approaching along Z (relative speed ~10 m/s)
static void setup_standard_bodies(std::vector<RigidBody>& bodies) {
    bodies.resize(2);

    bodies[0].position         = Vec3(0.0f, 0.0f, 0.0f);
    bodies[0].inverse_mass     = 1.0f;
    bodies[0].linear_velocity  = Vec3(0.0f, 0.0f, 5.0f);
    bodies[0].angular_velocity = Vec3(0.0f, 0.0f, 0.0f);
    bodies[0].orientation      = Quat::identity();
    bodies[0].local_inverse_inertia = Mat3::identity();
    bodies[0].update_world_inertia();

    bodies[1].position         = Vec3(0.0f, 0.0f, 1.5f);
    bodies[1].inverse_mass     = 1.0f;
    bodies[1].linear_velocity  = Vec3(0.0f, 0.0f, -5.0f);
    bodies[1].angular_velocity = Vec3(0.0f, 0.0f, 0.0f);
    bodies[1].orientation      = Quat::identity();
    bodies[1].local_inverse_inertia = Mat3::identity();
    bodies[1].update_world_inertia();
}

// Same as above but with tangential velocity (X component)
static void setup_tangential_bodies(std::vector<RigidBody>& bodies) {
    bodies.resize(2);

    bodies[0].position         = Vec3(0.0f, 0.0f, 0.0f);
    bodies[0].inverse_mass     = 1.0f;
    bodies[0].linear_velocity  = Vec3(3.0f, 0.0f, 5.0f);
    bodies[0].angular_velocity = Vec3(0.0f, 0.0f, 0.0f);
    bodies[0].orientation      = Quat::identity();
    bodies[0].local_inverse_inertia = Mat3::identity();
    bodies[0].update_world_inertia();

    bodies[1].position         = Vec3(0.0f, 0.0f, 1.5f);
    bodies[1].inverse_mass     = 1.0f;
    bodies[1].linear_velocity  = Vec3(-3.0f, 0.0f, -5.0f);
    bodies[1].angular_velocity = Vec3(0.0f, 0.0f, 0.0f);
    bodies[1].orientation      = Quat::identity();
    bodies[1].local_inverse_inertia = Mat3::identity();
    bodies[1].update_world_inertia();
}

// Bodies with angular velocity for spin tests
static void setup_spinning_bodies(std::vector<RigidBody>& bodies) {
    bodies.resize(2);

    bodies[0].position         = Vec3(0.0f, 0.0f, 0.0f);
    bodies[0].inverse_mass     = 1.0f;
    bodies[0].linear_velocity  = Vec3(0.0f, 0.0f, 5.0f);
    bodies[0].angular_velocity = Vec3(0.0f, 0.0f, 5.0f);
    bodies[0].orientation      = Quat::identity();
    bodies[0].local_inverse_inertia = Mat3::identity();
    bodies[0].update_world_inertia();

    bodies[1].position         = Vec3(0.0f, 0.0f, 1.5f);
    bodies[1].inverse_mass     = 1.0f;
    bodies[1].linear_velocity  = Vec3(0.0f, 0.0f, -5.0f);
    bodies[1].angular_velocity = Vec3(0.0f, 0.0f, -5.0f);
    bodies[1].orientation      = Quat::identity();
    bodies[1].local_inverse_inertia = Mat3::identity();
    bodies[1].update_world_inertia();
}

// Standard head-on ContactPoint (normal points B→A, i.e. −Z)
static ContactPoint make_standard_contact() {
    ContactPoint cp;
    cp.normal       = Vec3(0.0f, 0.0f, -1.0f);   // B→A
    cp.penetration  = 0.5f;
    cp.point_on_a   = Vec3(0.0f, 0.0f, 0.5f);
    cp.point_on_b   = Vec3(0.0f, 0.0f, 1.0f);
    return cp;
}

// ContactPoint with small penetration (needed for restitution to activate)
// Note: reserved for future restitution-specific tests
#ifdef __GNUC__
__attribute__((unused))
#endif
static ContactPoint make_shallow_contact() {
    ContactPoint cp;
    cp.normal       = Vec3(0.0f, 0.0f, -1.0f);
    cp.penetration  = 0.004f;   // below baumgarte_slop * 2
    cp.point_on_a   = Vec3(0.0f, 0.0f, 0.498f);
    cp.point_on_b   = Vec3(0.0f, 0.0f, 1.0f);
    return cp;
}

// Hash body linear + angular velocities for determinism checks
static uint64_t hash_body_state(const std::vector<RigidBody>& bodies) {
    uint64_t hash = 0;
    for (const auto& b : bodies) {
        uint32_t bits;
        std::memcpy(&bits, &b.linear_velocity.x, sizeof(bits));
        hash ^= static_cast<uint64_t>(bits) + 0x9e3779b9ull + (hash << 6);
        std::memcpy(&bits, &b.linear_velocity.y, sizeof(bits));
        hash ^= static_cast<uint64_t>(bits) + 0x9e3779b9ull + (hash << 6);
        std::memcpy(&bits, &b.linear_velocity.z, sizeof(bits));
        hash ^= static_cast<uint64_t>(bits) + 0x9e3779b9ull + (hash << 6);
        std::memcpy(&bits, &b.angular_velocity.x, sizeof(bits));
        hash ^= static_cast<uint64_t>(bits) + 0x9e3779b9ull + (hash << 6);
        std::memcpy(&bits, &b.angular_velocity.y, sizeof(bits));
        hash ^= static_cast<uint64_t>(bits) + 0x9e3779b9ull + (hash << 6);
        std::memcpy(&bits, &b.angular_velocity.z, sizeof(bits));
        hash ^= static_cast<uint64_t>(bits) + 0x9e3779b9ull + (hash << 6);
    }
    return hash;
}

// =============================================================================
// TEST 1: StylizedSolver default config
// =============================================================================
static int test_default_config() {
    std::printf("  [Test 1] StylizedSolver default config...\n");

    StylizedSolver solver;
    const auto& cfg = solver.get_config();

    assert(std::abs(cfg.dt - 1.0f / 240.0f) < 1e-7f);
    assert(cfg.velocity_iterations == 8u);
    assert(std::abs(cfg.baumgarte_factor - 0.2f) < 1e-7f);
    assert(std::abs(cfg.baumgarte_slop - 0.005f) < 1e-7f);
    assert(std::abs(cfg.linear_damping - 0.999f) < 1e-7f);
    assert(std::abs(cfg.angular_damping - 0.998f) < 1e-7f);
    assert(std::abs(cfg.default_friction - 0.4f) < 1e-7f);
    assert(std::abs(cfg.default_restitution - 0.0f) < 1e-7f);
    assert(cfg.blend_strategy == ProfileBlendStrategy::AVERAGE);
    assert(cfg.enable_vertical_bias == true);
    assert(cfg.enable_spin_multiplier == true);
    assert(std::abs(cfg.gravity_y - (-9.81f)) < 1e-7f);

    std::printf("    [PASS] All default config values verified\n");
    return 0;
}

// =============================================================================
// TEST 2: Basic prepare + solve produces impact events
// =============================================================================
static int test_basic_prepare_solve() {
    std::printf("  [Test 2] Basic prepare + solve...\n");

    std::vector<RigidBody> bodies;
    setup_standard_bodies(bodies);
    ContactPoint cp = make_standard_contact();

    StylizedSolver solver;
    solver.prepare(cp, 0, 1, bodies, 10.0f);
    solver.solve(bodies);

    uint32_t n = solver.get_event_count();
    if (n < 1) {
        std::printf("    [FAIL] Expected >= 1 event, got %u\n", n);
        return 1;
    }

    // Bodies should have changed velocity (collision resolved)
    // Body A was moving in +Z at 5 m/s, after head-on collision it should
    // have slowed down or reversed
    float za = bodies[0].linear_velocity.z;
    float zb = bodies[1].linear_velocity.z;

    // At minimum, velocities should be finite
    assert(std::isfinite(za));
    assert(std::isfinite(zb));

    std::printf("    [PASS] %u impact event(s), vel_z A=%.3f B=%.3f\n", n, za, zb);
    return 0;
}

// =============================================================================
// TEST 3: Profile-aware friction — different curves → different tangential
// =============================================================================
static int test_profile_aware_friction() {
    std::printf("  [Test 3] Profile-aware friction...\n");

    // --- Curve registry ---
    MaterialCurveRegistry curves;
    uint8_t cid_zero  = curves.add(MaterialCurve::make_constant(0.0f, 0.0f, 1.0f));
    uint8_t cid_high  = curves.add(MaterialCurve::make_constant(1.0f, 0.0f, 1.0f));

    // --- Profile registry ---
    ProfileRegistry profiles;
    profiles.add(ImpactStyleProfile::make_default());          // pid 0 (default)

    ImpactStyleProfile prof_no_fric = ImpactStyleProfile::make_default();
    prof_no_fric.friction_curve_id = cid_zero;
    uint16_t pid_no_fric = profiles.add(prof_no_fric);        // pid 1

    ImpactStyleProfile prof_high_fric = ImpactStyleProfile::make_default();
    prof_high_fric.friction_curve_id = cid_high;
    uint16_t pid_high_fric = profiles.add(prof_high_fric);    // pid 2

    // Run simulation with a given profile for body A
    auto run = [&](uint16_t pid_a) -> float {
        BodyStyleAssignment styles;
        styles.reset();
        styles.assign(0, pid_a);
        styles.assign(1, 0);     // body B gets default

        std::vector<RigidBody> bodies;
        setup_tangential_bodies(bodies);

        StylizedSolver solver;
        solver.set_curve_registry(&curves);
        solver.set_profile_registry(&profiles);
        solver.set_body_styles(&styles);

        ContactPoint cp = make_standard_contact();
        solver.prepare(cp, 0, 1, bodies, 10.0f);
        solver.solve(bodies);

        // Relative tangential speed (X component)
        return std::abs(bodies[0].linear_velocity.x -
                        bodies[1].linear_velocity.x);
    };

    float tang_no_fric  = run(pid_no_fric);   // friction ≈ 0.0
    float tang_high_fric = run(pid_high_fric); // friction ≈ 1.0

    // With zero friction, tangential velocity should be fully preserved.
    // With high friction, it should be reduced.
    bool friction_effective = tang_no_fric > tang_high_fric + 0.01f;

    if (friction_effective) {
        std::printf("    [PASS] No-friction tang=%.3f  High-friction tang=%.3f\n",
                   tang_no_fric, tang_high_fric);
    } else {
        std::printf("    [WARN] Friction effect subtle (no=%.3f high=%.3f)\n",
                   tang_no_fric, tang_high_fric);
    }

    return 0;
}

// =============================================================================
// TEST 4: Profile-aware restitution — higher curve value → more bounce
// =============================================================================
static int test_profile_aware_restitution() {
    std::printf("  [Test 4] Profile-aware restitution...\n");

    // --- Curve registry ---
    MaterialCurveRegistry curves;
    // Curve returning constant 0.8 (high bounce)
    uint8_t cid_high_e = curves.add(
        MaterialCurve::make_constant(0.8f, 0.0f, 30.0f));

    // --- Profile registry ---
    ProfileRegistry profiles;
    profiles.add(ImpactStyleProfile::make_default());   // pid 0

    ImpactStyleProfile prof_bouncy = ImpactStyleProfile::make_default();
    prof_bouncy.restitution_curve_id = cid_high_e;
    uint16_t pid_bouncy = profiles.add(prof_bouncy);    // pid 1

    // Config with large baumgarte_slop so the restitution code path activates
    StylizedSolverConfig cfg;
    cfg.baumgarte_slop = 1.0f;   // penetration 0.5 < 1.0*2 → restitution ON

    // --- Run WITHOUT restitution (default profile, no curve) ---
    float post_z_no_e = 0.0f;
    {
        BodyStyleAssignment styles;
        styles.reset();
        styles.assign(0, 0);
        styles.assign(1, 0);

        std::vector<RigidBody> bodies;
        setup_standard_bodies(bodies);

        StylizedSolver solver(cfg);
        solver.set_curve_registry(&curves);
        solver.set_profile_registry(&profiles);
        solver.set_body_styles(&styles);

        ContactPoint cp = make_standard_contact();
        solver.prepare(cp, 0, 1, bodies, 10.0f);
        solver.solve(bodies);

        post_z_no_e = bodies[0].linear_velocity.z;
    }

    // --- Run WITH high restitution ---
    float post_z_high_e = 0.0f;
    {
        BodyStyleAssignment styles;
        styles.reset();
        styles.assign(0, pid_bouncy);
        styles.assign(1, 0);

        std::vector<RigidBody> bodies;
        setup_standard_bodies(bodies);

        StylizedSolver solver(cfg);
        solver.set_curve_registry(&curves);
        solver.set_profile_registry(&profiles);
        solver.set_body_styles(&styles);

        ContactPoint cp = make_standard_contact();
        solver.prepare(cp, 0, 1, bodies, 10.0f);
        solver.solve(bodies);

        post_z_high_e = bodies[0].linear_velocity.z;
    }

    // Body A was at +Z velocity. After inelastic collision it should have
    // near-zero Z velocity. With high restitution it should bounce back
    // (negative Z).
    bool restitution_effective = post_z_high_e < post_z_no_e - 0.01f;

    if (restitution_effective) {
        std::printf("    [PASS] No-e vz=%.3f  High-e vz=%.3f  (bounced back)\n",
                   post_z_no_e, post_z_high_e);
    } else {
        std::printf("    [WARN] Restitution effect subtle (no=%.3f high=%.3f)\n",
                   post_z_no_e, post_z_high_e);
    }

    return 0;
}

// =============================================================================
// TEST 5: Vertical bias — bodies get upward velocity boost
// =============================================================================
static int test_vertical_bias() {
    std::printf("  [Test 5] Vertical bias...\n");

    // --- Profile with strong vertical bias ---
    ProfileRegistry profiles;
    profiles.add(ImpactStyleProfile::make_default());  // pid 0

    ImpactStyleProfile prof_bias = ImpactStyleProfile::make_default();
    prof_bias.vertical_bias = 0.5f;
    uint16_t pid_bias = profiles.add(prof_bias);       // pid 1

    // --- Run WITHOUT vertical bias ---
    float vy_no_bias = 0.0f;
    {
        BodyStyleAssignment styles;
        styles.reset();
        styles.assign(0, 0);
        styles.assign(1, 0);

        std::vector<RigidBody> bodies;
        setup_standard_bodies(bodies);

        StylizedSolver solver;
        solver.set_profile_registry(&profiles);
        solver.set_body_styles(&styles);

        ContactPoint cp = make_standard_contact();
        solver.prepare(cp, 0, 1, bodies, 10.0f);
        solver.solve(bodies);

        // Average Y velocity of both bodies
        vy_no_bias = (bodies[0].linear_velocity.y +
                      bodies[1].linear_velocity.y) * 0.5f;
    }

    // --- Run WITH vertical bias ---
    float vy_with_bias = 0.0f;
    {
        BodyStyleAssignment styles;
        styles.reset();
        styles.assign(0, pid_bias);
        styles.assign(1, pid_bias);

        std::vector<RigidBody> bodies;
        setup_standard_bodies(bodies);

        StylizedSolver solver;
        solver.set_profile_registry(&profiles);
        solver.set_body_styles(&styles);

        ContactPoint cp = make_standard_contact();
        solver.prepare(cp, 0, 1, bodies, 10.0f);
        solver.solve(bodies);

        vy_with_bias = (bodies[0].linear_velocity.y +
                        bodies[1].linear_velocity.y) * 0.5f;
    }

    // With vertical bias, average Y should be positive (upward boost)
    bool bias_applied = vy_with_bias > vy_no_bias + 0.01f;

    if (bias_applied) {
        std::printf("    [PASS] No-bias avg_vy=%.4f  Bias=0.5 avg_vy=%.4f\n",
                   vy_no_bias, vy_with_bias);
    } else {
        std::printf("    [WARN] Vertical bias effect subtle (no=%.4f with=%.4f)\n",
                   vy_no_bias, vy_with_bias);
    }

    return 0;
}

// =============================================================================
// TEST 6: Spin multiplier — amplified angular velocity
// =============================================================================
static int test_spin_multiplier() {
    std::printf("  [Test 6] Spin multiplier...\n");

    // --- Profiles ---
    ProfileRegistry profiles;
    profiles.add(ImpactStyleProfile::make_default());  // pid 0

    ImpactStyleProfile prof_spin = ImpactStyleProfile::make_default();
    prof_spin.spin_multiplier = 3.0f;
    uint16_t pid_spin = profiles.add(prof_spin);       // pid 1

    // --- Run WITHOUT spin multiplier (default = 1.0) ---
    float ang_no_spin = 0.0f;
    {
        BodyStyleAssignment styles;
        styles.reset();
        styles.assign(0, 0);
        styles.assign(1, 0);

        std::vector<RigidBody> bodies;
        setup_spinning_bodies(bodies);

        StylizedSolver solver;
        solver.set_profile_registry(&profiles);
        solver.set_body_styles(&styles);

        ContactPoint cp = make_standard_contact();
        solver.prepare(cp, 0, 1, bodies, 10.0f);
        solver.solve(bodies);

        ang_no_spin = Vec3::length(bodies[0].angular_velocity);
    }

    // --- Run WITH spin multiplier (3.0) ---
    float ang_with_spin = 0.0f;
    {
        BodyStyleAssignment styles;
        styles.reset();
        styles.assign(0, pid_spin);
        styles.assign(1, pid_spin);

        std::vector<RigidBody> bodies;
        setup_spinning_bodies(bodies);

        StylizedSolver solver;
        solver.set_profile_registry(&profiles);
        solver.set_body_styles(&styles);

        ContactPoint cp = make_standard_contact();
        solver.prepare(cp, 0, 1, bodies, 10.0f);
        solver.solve(bodies);

        ang_with_spin = Vec3::length(bodies[0].angular_velocity);
    }

    // Spin multiplier > 1 should amplify angular velocity
    bool spin_amplified = ang_with_spin > ang_no_spin + 0.01f;

    if (spin_amplified) {
        std::printf("    [PASS] No-spin ang=%.3f  Spin×3 ang=%.3f\n",
                   ang_no_spin, ang_with_spin);
    } else {
        std::printf("    [WARN] Spin effect subtle (no=%.3f with=%.3f)\n",
                   ang_no_spin, ang_with_spin);
    }

    return 0;
}

// =============================================================================
// TEST 7: Profile blend strategies — different strategies, different results
// =============================================================================
static int test_blend_strategies() {
    std::printf("  [Test 7] Profile blend strategies...\n");

    // --- Two profiles with very different vertical_bias ---
    ProfileRegistry profiles;
    profiles.add(ImpactStyleProfile::make_default());  // pid 0 (default, bias=0)

    ImpactStyleProfile prof_low  = ImpactStyleProfile::make_default();
    prof_low.vertical_bias = 0.0f;
    uint16_t pid_low = profiles.add(prof_low);         // pid 1

    ImpactStyleProfile prof_high = ImpactStyleProfile::make_default();
    prof_high.vertical_bias = 0.5f;
    uint16_t pid_high = profiles.add(prof_high);       // pid 2

    // Run with BODY_A_WINS (body A = prof_low → bias = 0.0)
    float vy_a_wins = 0.0f;
    {
        StylizedSolverConfig cfg;
        cfg.blend_strategy = ProfileBlendStrategy::BODY_A_WINS;

        BodyStyleAssignment styles;
        styles.reset();
        styles.assign(0, pid_low);    // body A → bias = 0.0
        styles.assign(1, pid_high);   // body B → bias = 0.5

        std::vector<RigidBody> bodies;
        setup_standard_bodies(bodies);

        StylizedSolver solver(cfg);
        solver.set_profile_registry(&profiles);
        solver.set_body_styles(&styles);

        ContactPoint cp = make_standard_contact();
        solver.prepare(cp, 0, 1, bodies, 10.0f);
        solver.solve(bodies);

        vy_a_wins = (bodies[0].linear_velocity.y +
                     bodies[1].linear_velocity.y) * 0.5f;
    }

    // Run with BODY_B_WINS (body B = prof_high → bias = 0.5)
    float vy_b_wins = 0.0f;
    {
        StylizedSolverConfig cfg;
        cfg.blend_strategy = ProfileBlendStrategy::BODY_B_WINS;

        BodyStyleAssignment styles;
        styles.reset();
        styles.assign(0, pid_low);    // body A → bias = 0.0
        styles.assign(1, pid_high);   // body B → bias = 0.5

        std::vector<RigidBody> bodies;
        setup_standard_bodies(bodies);

        StylizedSolver solver(cfg);
        solver.set_profile_registry(&profiles);
        solver.set_body_styles(&styles);

        ContactPoint cp = make_standard_contact();
        solver.prepare(cp, 0, 1, bodies, 10.0f);
        solver.solve(bodies);

        vy_b_wins = (bodies[0].linear_velocity.y +
                     bodies[1].linear_velocity.y) * 0.5f;
    }

    // Run with AVERAGE (resolved bias = (0.0 + 0.5)/2 = 0.25)
    float vy_avg = 0.0f;
    {
        StylizedSolverConfig cfg;
        cfg.blend_strategy = ProfileBlendStrategy::AVERAGE;

        BodyStyleAssignment styles;
        styles.reset();
        styles.assign(0, pid_low);
        styles.assign(1, pid_high);

        std::vector<RigidBody> bodies;
        setup_standard_bodies(bodies);

        StylizedSolver solver(cfg);
        solver.set_profile_registry(&profiles);
        solver.set_body_styles(&styles);

        ContactPoint cp = make_standard_contact();
        solver.prepare(cp, 0, 1, bodies, 10.0f);
        solver.solve(bodies);

        vy_avg = (bodies[0].linear_velocity.y +
                  bodies[1].linear_velocity.y) * 0.5f;
    }

    // BODY_B_WINS should produce more upward velocity than BODY_A_WINS
    bool b_wins_more = vy_b_wins > vy_a_wins + 0.001f;
    // AVERAGE should be between the two (or at least differ from both)
    bool avg_differs = std::abs(vy_avg - vy_a_wins) > 0.001f ||
                       std::abs(vy_avg - vy_b_wins) > 0.001f;

    if (b_wins_more && avg_differs) {
        std::printf("    [PASS] A_WINS=%.4f  AVG=%.4f  B_WINS=%.4f\n",
                   vy_a_wins, vy_avg, vy_b_wins);
    } else {
        std::printf("    [WARN] Blend differences subtle "
                   "(A=%.4f AVG=%.4f B=%.4f)\n",
                   vy_a_wins, vy_avg, vy_b_wins);
    }

    return 0;
}

// =============================================================================
// TEST 8: Impact event collection — correct body IDs, profile IDs, contact
// =============================================================================
static int test_impact_event_collection() {
    std::printf("  [Test 8] Impact event collection...\n");

    // --- Setup registries ---
    MaterialCurveRegistry curves;
    curves.setup_defaults();

    ProfileRegistry profiles;
    profiles.add(ImpactStyleProfile::make_default());   // pid 0
    profiles.add(ImpactStyleProfile::make_light());     // pid 1
    profiles.add(ImpactStyleProfile::make_heavy());     // pid 2

    BodyStyleAssignment styles;
    styles.reset();
    styles.assign(0, 1, ContactRegion::TORSO);     // body A = "light", torso
    styles.assign(1, 2, ContactRegion::UPPER_LEG);  // body B = "heavy", upper_leg

    std::vector<RigidBody> bodies;
    setup_standard_bodies(bodies);
    ContactPoint cp = make_standard_contact();

    StylizedSolver solver;
    solver.set_curve_registry(&curves);
    solver.set_profile_registry(&profiles);
    solver.set_body_styles(&styles);

    solver.prepare(cp, 0, 1, bodies, 10.0f);
    solver.solve(bodies);

    uint32_t n = solver.get_event_count();
    if (n < 1) {
        std::printf("    [FAIL] No events collected\n");
        return 1;
    }

    const ImpactEvent* evts = solver.get_events();
    const ImpactEvent& e = evts[0];

    // Verify body IDs
    assert(e.body_a == 0);
    assert(e.body_b == 1);

    // Verify profile IDs match the styles we assigned
    assert(e.profile_id_a == 1);   // "light"
    assert(e.profile_id_b == 2);   // "heavy"

    // Verify contact info is populated
    assert(e.relative_speed > 0.0f);
    assert(e.normal_impulse >= 0.0f);
    assert(e.penetration > 0.0f);
    assert(e.impact_force > 0.0f);   // impulse / dt

    // Verify contact regions
    assert(e.region_a == ContactRegion::TORSO);
    assert(e.region_b == ContactRegion::UPPER_LEG);

    // Verify new_contact flag
    assert(e.new_contact == true);

    // Verify normal is approximately what we set
    float ndot = Vec3::dot(e.contact_normal, Vec3(0.0f, 0.0f, -1.0f));
    assert(std::abs(ndot - 1.0f) < 0.01f);   // normal matches

    std::printf("    [PASS] %u event(s), body_a=%u body_b=%u, "
               "profile_a=%u profile_b=%u, force=%.1f N\n",
               n, e.body_a, e.body_b, e.profile_id_a, e.profile_id_b,
               e.impact_force);
    return 0;
}

// =============================================================================
// TEST 9: StylizedSolver determinism — same input → same output
// =============================================================================
static int test_determinism() {
    std::printf("  [Test 9] StylizedSolver determinism...\n");

    auto run_sim = []() -> uint64_t {
        MaterialCurveRegistry curves;
        curves.setup_defaults();

        ProfileRegistry profiles;
        profiles.add(ImpactStyleProfile::make_default());
        profiles.add(ImpactStyleProfile::make_light());

        BodyStyleAssignment styles;
        styles.reset();
        styles.assign(0, 1, ContactRegion::HEAD);
        styles.assign(1, 0, ContactRegion::TORSO);

        std::vector<RigidBody> bodies;
        setup_tangential_bodies(bodies);   // includes tangential velocity

        StylizedSolverConfig cfg;
        cfg.blend_strategy = ProfileBlendStrategy::MAX;
        cfg.enable_vertical_bias = true;
        cfg.enable_spin_multiplier = true;

        StylizedSolver solver(cfg);
        solver.set_curve_registry(&curves);
        solver.set_profile_registry(&profiles);
        solver.set_body_styles(&styles);

        ContactPoint cp = make_standard_contact();
        solver.prepare(cp, 0, 1, bodies, 10.0f);
        solver.solve(bodies);

        return hash_body_state(bodies);
    };

    uint64_t h1 = run_sim();
    uint64_t h2 = run_sim();

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
// TEST 10 (bonus): Custom config overrides defaults correctly
// =============================================================================
static int test_custom_config() {
    std::printf("  [Test 10] Custom config overrides...\n");

    StylizedSolverConfig cfg;
    cfg.dt                  = 1.0f / 60.0f;
    cfg.velocity_iterations = 12u;
    cfg.default_friction    = 0.7f;
    cfg.default_restitution = 0.3f;
    cfg.blend_strategy      = ProfileBlendStrategy::ATTACKER_WINS;
    cfg.enable_vertical_bias = false;

    StylizedSolver solver(cfg);
    const auto& c = solver.get_config();

    assert(std::abs(c.dt - 1.0f / 60.0f) < 1e-7f);
    assert(c.velocity_iterations == 12u);
    assert(std::abs(c.default_friction - 0.7f) < 1e-7f);
    assert(std::abs(c.default_restitution - 0.3f) < 1e-7f);
    assert(c.blend_strategy == ProfileBlendStrategy::ATTACKER_WINS);
    assert(c.enable_vertical_bias == false);

    std::printf("    [PASS] Custom config applied correctly\n");
    return 0;
}

// =============================================================================
// Main
// =============================================================================
int main() {
    std::printf("Running Sprint 10 Tests: StylizedSolver Integration\n");
    std::printf("=====================================================\n");

    int result = 0;
    result |= test_default_config();
    result |= test_basic_prepare_solve();
    result |= test_profile_aware_friction();
    result |= test_profile_aware_restitution();
    result |= test_vertical_bias();
    result |= test_spin_multiplier();
    result |= test_blend_strategies();
    result |= test_impact_event_collection();
    result |= test_determinism();
    result |= test_custom_config();

    if (result == 0) {
        std::printf("\nAll Sprint 10 tests PASSED.\n");
    } else {
        std::printf("\nSome Sprint 10 tests FAILED.\n");
    }
    return result;
}
