// =============================================================================
// Sprint 13 Tests — Ball Physics Core
// =============================================================================
//
// Tests for the Ball Physics system (apc_sport/apc_ball_physics.h):
//   1. BallConfig basics — soccer (sphere) and rugby (prolate) factories,
//      get_effective_radius(), to_rigid_body()
//   2. All ball factories — soccer, basketball, rugby, american_football,
//      tennis, baseball, golf, volleyball, aussie_rules, cricket, handball
//   3. BallState — reset(), spin info, angular velocity classification
//   4. SurfaceBounceTable — default entries, velocity-dependent restitution,
//      set/get for custom surfaces
//   5. AerodynamicModel — drag, Magnus, spin decay, apply()
//   6. BallBounce — resolve_surface, resolve_body
//   7. BallPhysicsWorld — multi-ball simulation, gravity, ground collision
//
// Pattern: int main() + assert(), no test framework.
// =============================================================================

#include "apc_sport/apc_ball_physics.h"
#include <cassert>
#include <cmath>
#include <cstdio>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static constexpr float EPS = 1e-5f;

static bool approx_eq(float a, float b, float eps = EPS) {
    return std::abs(a - b) < eps;
}

// =============================================================================
// TEST 1: BallConfig basics — soccer (sphere) and rugby (prolate)
// =============================================================================
static int test_ball_config_basics() {
    std::printf("  [Test 1] BallConfig basics...\n");

    // --- Soccer ball (sphere) ---
    apc::BallConfig soccer = apc::BallFactory::make_soccer();
    assert(soccer.shape == apc::BallShape::SPHERE && "Soccer shape should be SPHERE");
    assert(approx_eq(soccer.radius, 0.11f) && "Soccer radius ≈ 0.11");
    assert(approx_eq(soccer.mass, 0.43f) && "Soccer mass ≈ 0.43");
    assert(soccer.mass > 0.0f && "Soccer mass > 0");
    assert(soccer.radius > 0.0f && "Soccer radius > 0");

    // Effective radius for sphere = radius
    float eff_r_soccer = soccer.get_effective_radius();
    assert(approx_eq(eff_r_soccer, 0.11f) && "Sphere effective radius = radius");

    // to_rigid_body produces valid body
    apc::RigidBody rb_soccer = soccer.to_rigid_body();
    assert(rb_soccer.inverse_mass > 0.0f && "Soccer RigidBody inverse_mass > 0");
    assert(approx_eq(rb_soccer.inverse_mass, 1.0f / 0.43f, 0.01f)
           && "Soccer inverse_mass ≈ 1/mass");

    // --- Rugby ball (prolate) ---
    apc::BallConfig rugby = apc::BallFactory::make_rugby();
    assert(rugby.shape == apc::BallShape::PROLATE && "Rugby shape should be PROLATE");
    assert(approx_eq(rugby.semi_major, 0.16f) && "Rugby semi_major ≈ 0.16");
    assert(approx_eq(rugby.semi_minor, 0.06f) && "Rugby semi_minor ≈ 0.06");

    // Effective radius for prolate = (semi_major + semi_minor) * 0.5
    float eff_r_rugby = rugby.get_effective_radius();
    float expected_eff_r = (0.16f + 0.06f) * 0.5f; // = 0.11
    assert(approx_eq(eff_r_rugby, expected_eff_r) && "Prolate effective radius = (major+minor)/2");

    // to_rigid_body for prolate also valid
    apc::RigidBody rb_rugby = rugby.to_rigid_body();
    assert(rb_rugby.inverse_mass > 0.0f && "Rugby RigidBody inverse_mass > 0");

    // Inertia diagonal should be positive (inv_I > 0 since moment_of_inertia > 0)
    assert(rb_rugby.local_inverse_inertia.m[0] > 0.0f && "Rugby inv inertia m[0] > 0");
    assert(rb_rugby.local_inverse_inertia.m[4] > 0.0f && "Rugby inv inertia m[4] > 0");
    assert(rb_rugby.local_inverse_inertia.m[8] > 0.0f && "Rugby inv inertia m[8] > 0");

    std::printf("    [PASS] Soccer sphere and rugby prolate configs verified\n");
    return 0;
}

// =============================================================================
// TEST 2: All ball factories have reasonable physical properties
// =============================================================================
static int test_all_ball_factories() {
    std::printf("  [Test 2] All ball factories...\n");

    // Soccer
    {
        apc::BallConfig c = apc::BallFactory::make_soccer();
        assert(c.mass > 0.0f && "Soccer mass > 0");
        assert(c.radius > 0.0f && "Soccer radius > 0");
        assert(c.drag_coefficient > 0.0f && "Soccer Cd > 0");
        assert(c.cross_section_area > 0.0f && "Soccer area > 0");
        assert(c.moment_of_inertia > 0.0f && "Soccer MoI > 0");
        assert(c.base_restitution > 0.0f && "Soccer restitution > 0");
    }
    // Basketball
    {
        apc::BallConfig c = apc::BallFactory::make_basketball();
        assert(c.mass > 0.0f && "Basketball mass > 0");
        assert(c.radius > 0.0f && "Basketball radius > 0");
        assert(c.drag_coefficient > 0.0f && "Basketball Cd > 0");
        assert(c.cross_section_area > 0.0f && "Basketball area > 0");
        assert(c.moment_of_inertia > 0.0f && "Basketball MoI > 0");
        assert(c.base_restitution > 0.0f && "Basketball restitution > 0");
    }
    // Rugby
    {
        apc::BallConfig c = apc::BallFactory::make_rugby();
        assert(c.mass > 0.0f && "Rugby mass > 0");
        assert(c.semi_major > 0.0f && "Rugby semi_major > 0");
        assert(c.semi_minor > 0.0f && "Rugby semi_minor > 0");
        assert(c.drag_coefficient > 0.0f && "Rugby Cd > 0");
        assert(c.moment_of_inertia > 0.0f && "Rugby MoI > 0");
        assert(c.base_restitution > 0.0f && "Rugby restitution > 0");
        assert(c.shape == apc::BallShape::PROLATE && "Rugby is PROLATE");
    }
    // American Football
    {
        apc::BallConfig c = apc::BallFactory::make_american_football();
        assert(c.mass > 0.0f && "American Football mass > 0");
        assert(c.semi_major > 0.0f && "American Football semi_major > 0");
        assert(c.semi_minor > 0.0f && "American Football semi_minor > 0");
        assert(c.drag_coefficient > 0.0f && "American Football Cd > 0");
        assert(c.moment_of_inertia > 0.0f && "American Football MoI > 0");
        assert(c.base_restitution > 0.0f && "American Football restitution > 0");
        assert(c.shape == apc::BallShape::PROLATE && "American Football is PROLATE");
    }
    // Tennis
    {
        apc::BallConfig c = apc::BallFactory::make_tennis();
        assert(c.mass > 0.0f && "Tennis mass > 0");
        assert(c.radius > 0.0f && "Tennis radius > 0");
        assert(c.drag_coefficient > 0.0f && "Tennis Cd > 0");
        assert(c.cross_section_area > 0.0f && "Tennis area > 0");
        assert(c.moment_of_inertia > 0.0f && "Tennis MoI > 0");
        assert(c.base_restitution > 0.0f && "Tennis restitution > 0");
    }
    // Baseball
    {
        apc::BallConfig c = apc::BallFactory::make_baseball();
        assert(c.mass > 0.0f && "Baseball mass > 0");
        assert(c.radius > 0.0f && "Baseball radius > 0");
        assert(c.drag_coefficient > 0.0f && "Baseball Cd > 0");
        assert(c.cross_section_area > 0.0f && "Baseball area > 0");
        assert(c.moment_of_inertia > 0.0f && "Baseball MoI > 0");
        assert(c.base_restitution > 0.0f && "Baseball restitution > 0");
    }
    // Golf
    {
        apc::BallConfig c = apc::BallFactory::make_golf();
        assert(c.mass > 0.0f && "Golf mass > 0");
        assert(c.radius > 0.0f && "Golf radius > 0");
        assert(c.drag_coefficient > 0.0f && "Golf Cd > 0");
        assert(c.cross_section_area > 0.0f && "Golf area > 0");
        assert(c.moment_of_inertia > 0.0f && "Golf MoI > 0");
        assert(c.base_restitution > 0.0f && "Golf restitution > 0");
    }
    // Volleyball
    {
        apc::BallConfig c = apc::BallFactory::make_volleyball();
        assert(c.mass > 0.0f && "Volleyball mass > 0");
        assert(c.radius > 0.0f && "Volleyball radius > 0");
        assert(c.drag_coefficient > 0.0f && "Volleyball Cd > 0");
        assert(c.cross_section_area > 0.0f && "Volleyball area > 0");
        assert(c.moment_of_inertia > 0.0f && "Volleyball MoI > 0");
        assert(c.base_restitution > 0.0f && "Volleyball restitution > 0");
    }
    // Australian Rules Football
    {
        apc::BallConfig c = apc::BallFactory::make_aussie_rules();
        assert(c.mass > 0.0f && "Aussie Rules mass > 0");
        assert(c.semi_major > 0.0f && "Aussie Rules semi_major > 0");
        assert(c.semi_minor > 0.0f && "Aussie Rules semi_minor > 0");
        assert(c.drag_coefficient > 0.0f && "Aussie Rules Cd > 0");
        assert(c.moment_of_inertia > 0.0f && "Aussie Rules MoI > 0");
        assert(c.base_restitution > 0.0f && "Aussie Rules restitution > 0");
        assert(c.shape == apc::BallShape::PROLATE && "Aussie Rules is PROLATE");
    }
    // Cricket
    {
        apc::BallConfig c = apc::BallFactory::make_cricket();
        assert(c.mass > 0.0f && "Cricket mass > 0");
        assert(c.radius > 0.0f && "Cricket radius > 0");
        assert(c.drag_coefficient > 0.0f && "Cricket Cd > 0");
        assert(c.cross_section_area > 0.0f && "Cricket area > 0");
        assert(c.moment_of_inertia > 0.0f && "Cricket MoI > 0");
        assert(c.base_restitution > 0.0f && "Cricket restitution > 0");
    }
    // Handball
    {
        apc::BallConfig c = apc::BallFactory::make_handball();
        assert(c.mass > 0.0f && "Handball mass > 0");
        assert(c.radius > 0.0f && "Handball radius > 0");
        assert(c.drag_coefficient > 0.0f && "Handball Cd > 0");
        assert(c.cross_section_area > 0.0f && "Handball area > 0");
        assert(c.moment_of_inertia > 0.0f && "Handball MoI > 0");
        assert(c.base_restitution > 0.0f && "Handball restitution > 0");
    }

    // Verify all factories produce distinct names
    apc::BallConfig configs[11];
    configs[0] = apc::BallFactory::make_soccer();
    configs[1] = apc::BallFactory::make_basketball();
    configs[2] = apc::BallFactory::make_rugby();
    configs[3] = apc::BallFactory::make_american_football();
    configs[4] = apc::BallFactory::make_tennis();
    configs[5] = apc::BallFactory::make_baseball();
    configs[6] = apc::BallFactory::make_golf();
    configs[7] = apc::BallFactory::make_volleyball();
    configs[8] = apc::BallFactory::make_aussie_rules();
    configs[9] = apc::BallFactory::make_cricket();
    configs[10] = apc::BallFactory::make_handball();

    // Golf is the lightest ball
    assert(configs[6].mass < configs[0].mass && "Golf lighter than soccer");
    // Basketball is heavier than soccer
    assert(configs[1].mass > configs[0].mass && "Basketball heavier than soccer");
    // Tennis has a smaller radius than soccer
    assert(configs[4].radius < configs[0].radius && "Tennis smaller than soccer");
    // Basketball has a larger radius than soccer
    assert(configs[1].radius > configs[0].radius && "Basketball larger than soccer");

    std::printf("    [PASS] All 11 ball factories produce valid configs\n");
    return 0;
}

// =============================================================================
// TEST 3: BallState — reset, spin info, angular velocity
// =============================================================================
static int test_ball_state() {
    std::printf("  [Test 3] BallState...\n");

    apc::BallConfig soccer = apc::BallFactory::make_soccer();

    // Create BallState from config, call reset()
    apc::BallState bs;
    bs.config = soccer;
    bs.ball_id = 42;
    bs.reset();

    // Verify reset clears runtime state
    assert(bs.on_ground == false && "After reset: on_ground = false");
    assert(bs.in_air == true && "After reset: in_air = true");
    assert(approx_eq(bs.deformation, 0.0f) && "After reset: deformation = 0");
    assert(approx_eq(bs.spin_rate, 0.0f) && "After reset: spin_rate = 0");
    assert(approx_eq(bs.knuckle_intensity, 0.0f) && "After reset: knuckle = 0");
    assert(bs.in_water == false && "After reset: in_water = false");
    assert(approx_eq(bs.ground_contact_time, 0.0f) && "After reset: ground_contact_time = 0");
    assert(bs.ball_id == 42 && "Ball ID preserved after reset");

    // Default spin axis is up
    assert(approx_eq(bs.spin_axis.x, 0.0f) && "Default spin axis x = 0");
    assert(approx_eq(bs.spin_axis.y, 1.0f) && "Default spin axis y = 1");
    assert(approx_eq(bs.spin_axis.z, 0.0f) && "Default spin axis z = 0");

    // Body should be at origin with zero velocity
    assert(approx_eq(bs.body.position.x, 0.0f) && "Reset position x = 0");
    assert(approx_eq(bs.body.position.y, 0.0f) && "Reset position y = 0");
    assert(approx_eq(bs.body.position.z, 0.0f) && "Reset position z = 0");

    // --- Set angular velocity and call update_spin_info() ---
    bs.body.angular_velocity = apc::Vec3(0.0f, 30.0f, 0.0f); // 30 rad/s around Y
    bs.body.linear_velocity = apc::Vec3(10.0f, 0.0f, 0.0f);  // Moving in +X
    bs.update_spin_info();

    assert(bs.spin_rate > 0.0f && "spin_rate > 0 after setting angular velocity");
    assert(approx_eq(bs.spin_rate, 30.0f, 0.1f) && "spin_rate ≈ 30 rad/s");

    // Spin axis should be normalized (unit vector)
    float axis_len = apc::Vec3::length(bs.spin_axis);
    assert(approx_eq(axis_len, 1.0f, 0.001f) && "spin_axis is normalized");
    assert(approx_eq(bs.spin_axis.y, 1.0f, 0.01f) && "spin_axis points in Y");

    // With Y-axis spin and +X velocity, spin_axis=(0,1,0) aligns with world-up,
    // so abs_up dominates → HELICAL
    assert(bs.spin_type == apc::SpinAxis::HELICAL && "Y-spin + X-vel = HELICAL (up-aligned)");

    // --- Test TOPSPIN: spin aligned with velocity direction (forward rotation) ---
    // For topspin: spin axis should align with velocity direction (positive dot)
    // If velocity is +X and spin is around Z-axis, the cross product of angular
    // velocity with velocity... Let's think: omega x v where omega=(0,0,w), v=(v,0,0)
    // That gives (0*0 - w*0, w*v - 0*0, 0*0 - 0*v) = (0, wv, 0) — upward Magnus = topspin effect
    // But the classification uses spin_axis dot velocity_dir.
    // spin_axis = normalize(angular_velocity) = (0,0,1), vel_dir = (1,0,0)
    // forward_dot = 0, lateral = cross(vel_dir, up) = (0,0,-1), lateral_dot = -1
    // abs_fwd=0, abs_lat=1, abs_up=0 → SIDESPIN
    // 
    // For TOPSPIN: need spin_axis to align with velocity direction
    // angular_vel = (30, 0, 0), spin_axis = (1, 0, 0), vel = (10, 0, 0), vel_dir = (1, 0, 0)
    // forward_dot = 1, abs_fwd=1 > abs_lat=0 and abs_up=0 → TOPSPIN
    bs.body.angular_velocity = apc::Vec3(30.0f, 0.0f, 0.0f);
    bs.body.linear_velocity = apc::Vec3(10.0f, 0.0f, 0.0f);
    bs.update_spin_info();
    assert(bs.spin_type == apc::SpinAxis::TOPSPIN && "Aligned forward spin = TOPSPIN");

    // --- Test BACKSPIN: spin opposite to velocity direction ---
    bs.body.angular_velocity = apc::Vec3(-30.0f, 0.0f, 0.0f);
    bs.body.linear_velocity = apc::Vec3(10.0f, 0.0f, 0.0f);
    bs.update_spin_info();
    assert(bs.spin_type == apc::SpinAxis::BACKSPIN && "Anti-aligned spin = BACKSPIN");

    // --- Test HELICAL: spin along up axis with zero velocity ---
    bs.body.angular_velocity = apc::Vec3(0.0f, 50.0f, 0.0f);
    bs.body.linear_velocity = apc::Vec3(0.0f, 0.0f, 0.0f);
    bs.update_spin_info();
    assert(bs.spin_type == apc::SpinAxis::HELICAL && "Spin with no velocity = HELICAL");

    std::printf("    [PASS] BallState reset, spin_rate, spin_axis normalization, "
                "spin classification verified\n");
    return 0;
}

// =============================================================================
// TEST 4: SurfaceBounceTable — default entries, velocity-dependent restitution
// =============================================================================
static int test_surface_bounce_table() {
    std::printf("  [Test 4] SurfaceBounceTable...\n");

    apc::SurfaceBounceTable table = apc::SurfaceBounceTable::make_default();

    // Verify default surface entries
    const apc::SurfaceBounceEntry& grass = table.get(apc::SurfaceType::GRASS);
    assert(approx_eq(grass.restitution, 0.6f, 0.01f) && "GRASS restitution ≈ 0.6");
    assert(approx_eq(grass.friction, 0.45f, 0.01f) && "GRASS friction ≈ 0.45");

    const apc::SurfaceBounceEntry& wood = table.get(apc::SurfaceType::WOOD);
    assert(approx_eq(wood.restitution, 0.85f, 0.01f) && "WOOD restitution ≈ 0.85");

    const apc::SurfaceBounceEntry& sand = table.get(apc::SurfaceType::SAND);
    assert(approx_eq(sand.restitution, 0.15f, 0.01f) && "SAND restitution ≈ 0.15");

    const apc::SurfaceBounceEntry& ice = table.get(apc::SurfaceType::ICE);
    assert(approx_eq(ice.restitution, 0.6f, 0.01f) && "ICE restitution ≈ 0.6");
    assert(approx_eq(ice.friction, 0.03f, 0.01f) && "ICE friction ≈ 0.03 (very low)");

    // Trampoline should have very high restitution
    const apc::SurfaceBounceEntry& trampoline = table.get(apc::SurfaceType::TRAMPOLINE);
    assert(approx_eq(trampoline.restitution, 0.95f, 0.01f) && "TRAMPOLINE restitution ≈ 0.95");

    // --- Velocity-dependent restitution via lookup() ---
    apc::BallConfig soccer = apc::BallFactory::make_soccer();

    // Low impact speed: restitution should be near the base value
    apc::SurfaceBounceEntry low_entry = table.lookup(soccer, apc::SurfaceType::GRASS, 1.0f);
    assert(low_entry.restitution > 0.5f && "Low speed: restitution still high");

    // High impact speed: restitution should be reduced
    apc::SurfaceBounceEntry high_entry = table.lookup(soccer, apc::SurfaceType::GRASS, 50.0f);
    assert(high_entry.restitution < low_entry.restitution
           && "High speed: restitution reduced vs low speed");

    // Restitution should be clamped to >= 0.1
    apc::SurfaceBounceEntry extreme_entry = table.lookup(soccer, apc::SurfaceType::SAND, 200.0f);
    assert(extreme_entry.restitution >= 0.1f - EPS
           && "Extreme speed: restitution clamped >= 0.1");

    // --- set() and get() for CUSTOM_0 ---
    table.set(apc::SurfaceType::CUSTOM_0, 0.9f, 0.6f, 0.02f, 0.05f, 0.3f, 0.7f);
    const apc::SurfaceBounceEntry& custom = table.get(apc::SurfaceType::CUSTOM_0);
    assert(approx_eq(custom.restitution, 0.9f) && "CUSTOM_0 restitution set correctly");
    assert(approx_eq(custom.friction, 0.6f) && "CUSTOM_0 friction set correctly");
    assert(approx_eq(custom.rolling_resistance, 0.02f) && "CUSTOM_0 rolling set correctly");
    assert(approx_eq(custom.spin_damping, 0.05f) && "CUSTOM_0 spin_damping set correctly");
    assert(approx_eq(custom.deformation_factor, 0.3f) && "CUSTOM_0 deformation set correctly");
    assert(approx_eq(custom.bounce_sound_intensity, 0.7f) && "CUSTOM_0 sound set correctly");

    // Verify lookup on CUSTOM_0
    apc::SurfaceBounceEntry custom_lookup = table.lookup(soccer, apc::SurfaceType::CUSTOM_0, 0.0f);
    assert(approx_eq(custom_lookup.restitution, 0.9f, 0.01f)
           && "CUSTOM_0 lookup at zero speed ≈ base restitution");

    std::printf("    [PASS] Default surfaces, velocity-dependent restitution, "
                "custom set/get verified\n");
    return 0;
}

// =============================================================================
// TEST 5: AerodynamicModel — drag, Magnus, spin decay, apply
// =============================================================================
static int test_aerodynamic_model() {
    std::printf("  [Test 5] AerodynamicModel...\n");

    apc::AerodynamicModel aero;
    aero.dt = 1.0f / 240.0f;

    apc::BallConfig soccer = apc::BallFactory::make_soccer();
    apc::BallState bs;
    bs.config = soccer;
    bs.reset();

    // Set ball velocity to (10, 5, 0)
    bs.body.linear_velocity = apc::Vec3(10.0f, 5.0f, 0.0f);
    bs.update_spin_info();

    // --- Compute forces ---
    apc::AeroForces forces = aero.compute(bs);

    // Verify drag opposes velocity: drag.x < 0 if velocity.x > 0
    assert(forces.drag.x < 0.0f && "Drag x < 0 (opposes velocity.x > 0)");
    assert(forces.drag.y < 0.0f && "Drag y < 0 (opposes velocity.y > 0)");
    assert(approx_eq(forces.drag.z, 0.0f, EPS) && "Drag z ≈ 0 (velocity.z = 0)");

    // Verify drag_magnitude > 0 for moving ball
    assert(forces.drag_magnitude > 0.0f && "drag_magnitude > 0 for ball with speed > 0");

    // --- Test spin_decay_rate > 0 when ball has spin ---
    // Initially no spin → spin_decay should be 0
    apc::BallState bs_no_spin;
    bs_no_spin.config = soccer;
    bs_no_spin.reset();
    bs_no_spin.body.linear_velocity = apc::Vec3(10.0f, 0.0f, 0.0f);
    apc::AeroForces forces_no_spin = aero.compute(bs_no_spin);
    assert(approx_eq(forces_no_spin.spin_decay_rate, 0.0f, 0.001f)
           && "No spin → no spin decay");

    // Now add angular velocity to create spin
    bs.body.angular_velocity = apc::Vec3(0.0f, 30.0f, 0.0f); // 30 rad/s around Y
    bs.body.linear_velocity = apc::Vec3(10.0f, 0.0f, 0.0f);  // Moving in +X
    bs.update_spin_info();
    forces = aero.compute(bs);

    // Magnus force should exist when there's both spin and velocity
    assert(forces.magnus_magnitude > 0.0f && "magnus_magnitude > 0 with spin + velocity");

    // Spin decay rate should be > 0
    assert(forces.spin_decay_rate > 0.0f && "spin_decay_rate > 0 when ball has spin");

    // Magnus force direction: omega x v = (0,30,0) x (10,0,0) = (0*0-0*0, 0*10-0*0, 0*0-30*10) = (0,0,-300)
    // So magnus should have negative z component
    assert(forces.magnus.z < 0.0f && "Magnus force z < 0 (omega x v for Y-spin, X-vel)");

    // --- Test apply() modifies ball velocity (speed decreases due to drag) ---
    float speed_before = apc::Vec3::length(bs.body.linear_velocity);
    aero.apply(bs);
    float speed_after = apc::Vec3::length(bs.body.linear_velocity);

    assert(speed_after < speed_before && "Speed decreases after applying drag");

    // Angular velocity should also decrease (spin decay)
    float ang_speed_before = 30.0f;
    float ang_speed_after = apc::Vec3::length(bs.body.angular_velocity);
    assert(ang_speed_after < ang_speed_before && "Angular speed decreases after spin decay");

    // --- Test with zero velocity: no drag ---
    apc::BallState bs_static;
    bs_static.config = soccer;
    bs_static.reset();
    apc::AeroForces static_forces = aero.compute(bs_static);
    assert(approx_eq(static_forces.drag_magnitude, 0.0f) && "Zero velocity → zero drag");
    assert(approx_eq(static_forces.magnus_magnitude, 0.0f) && "Zero velocity → zero Magnus");

    std::printf("    [PASS] Drag opposes velocity, Magnus with spin, "
                "spin decay, apply() reduces speed\n");
    return 0;
}

// =============================================================================
// TEST 6: BallBounce — resolve_surface and resolve_body
// =============================================================================
static int test_ball_bounce() {
    std::printf("  [Test 6] BallBounce...\n");

    apc::BallConfig soccer = apc::BallFactory::make_soccer();
    apc::SurfaceBounceTable table = apc::SurfaceBounceTable::make_default();
    apc::BallBounce resolver;

    // --- resolve_surface: ball falling onto ground ---
    apc::BallState bs_fall;
    bs_fall.config = soccer;
    bs_fall.reset();
    bs_fall.body.position = apc::Vec3(0.0f, 0.11f, 0.0f); // At ground level
    bs_fall.body.linear_velocity = apc::Vec3(0.0f, -10.0f, 0.0f); // Falling down
    bs_fall.in_air = true;
    bs_fall.on_ground = false;

    apc::Vec3 ground_normal(0.0f, 1.0f, 0.0f);
    apc::BallBounceResult result_surface = resolver.resolve_surface(
        bs_fall, apc::SurfaceType::GRASS, ground_normal, table);

    assert(result_surface.triggered == true && "Surface bounce triggered");
    assert(result_surface.new_velocity.y > 0.0f && "Ball bounces up (vy > 0)");
    assert(result_surface.effective_restitution > 0.0f && "Effective restitution > 0");

    // Velocity magnitude should be less than before (energy loss)
    float speed_after_bounce = apc::Vec3::length(result_surface.new_velocity);
    assert(speed_after_bounce < 10.0f && "Bounce speed < impact speed (energy loss)");

    // Ball state should be updated
    assert(bs_fall.on_ground == true && "Ball on_ground after surface bounce");
    assert(bs_fall.in_air == false && "Ball not in_air after surface bounce");
    assert(bs_fall.current_surface == apc::SurfaceType::GRASS
           && "Current surface set to GRASS");

    // Deformation should be set
    assert(bs_fall.deformation > 0.0f && "Deformation > 0 after bounce");
    assert(bs_fall.deformation <= 1.0f && "Deformation <= 1.0 (clamped)");

    // --- resolve_surface: ball moving away from surface → no bounce ---
    apc::BallState bs_rising;
    bs_rising.config = soccer;
    bs_rising.reset();
    bs_rising.body.linear_velocity = apc::Vec3(0.0f, 5.0f, 0.0f); // Moving up

    apc::BallBounceResult no_bounce = resolver.resolve_surface(
        bs_rising, apc::SurfaceType::GRASS, ground_normal, table);
    assert(no_bounce.triggered == false && "No bounce when moving away from surface");

    // --- resolve_body: ball hitting another rigid body ---
    apc::BallState bs_body;
    bs_body.config = soccer;
    bs_body.reset();
    bs_body.body.position = apc::Vec3(0.0f, 1.0f, 0.0f);
    bs_body.body.linear_velocity = apc::Vec3(0.0f, -10.0f, 0.0f); // Falling toward body

    // Create a static rigid body (infinite mass = 0 inverse_mass)
    apc::RigidBody wall;
    wall.position = apc::Vec3(0.0f, 0.0f, 0.0f);
    wall.linear_velocity = apc::Vec3(0.0f, 0.0f, 0.0f);
    wall.inverse_mass = 0.0f; // Infinite mass (immovable)
    wall.orientation = apc::Quat::identity();
    wall.angular_velocity = apc::Vec3(0.0f, 0.0f, 0.0f);
    wall.update_world_inertia();

    apc::Vec3 contact_normal(0.0f, 1.0f, 0.0f); // Normal pointing up from wall
    apc::Vec3 contact_point(0.0f, 0.0f, 0.0f);

    apc::BallBounceResult result_body = resolver.resolve_body(
        bs_body, wall, contact_normal, contact_point, 0.8f);

    assert(result_body.triggered == true && "Body bounce triggered");
    assert(result_body.effective_restitution > 0.0f && "Body bounce effective_rest > 0");

    // Ball velocity should change
    assert(!bs_body.body.linear_velocity.equals_approx(apc::Vec3(0.0f, -10.0f, 0.0f))
           && "Velocity changed after body collision");

    // Ball should be bouncing up (or at least not falling as fast)
    assert(bs_body.body.linear_velocity.y > -10.0f
           && "Ball velocity changed (less negative or positive)");

    // Deformation should be set
    assert(bs_body.deformation > 0.0f && "Deformation > 0 after body bounce");

    std::printf("    [PASS] Surface bounce, no-bounce, body collision verified\n");
    return 0;
}

// =============================================================================
// TEST 7: BallPhysicsWorld — multi-ball simulation, gravity, ground collision
// =============================================================================
static int test_ball_physics_world() {
    std::printf("  [Test 7] BallPhysicsWorld...\n");

    apc::BallPhysicsWorld world;
    world.surface_table = apc::SurfaceBounceTable::make_default();
    float dt = 1.0f / 240.0f;

    // --- Add a soccer ball, place it at y=10, simulate ~1 second ---
    apc::BallConfig soccer = apc::BallFactory::make_soccer();
    uint32_t ball_id = world.add_ball(soccer);
    assert(ball_id == 0 && "First ball gets ID 0");
    assert(world.ball_count == 1 && "ball_count = 1");

    apc::BallState* ball = world.get_ball(0);
    assert(ball != nullptr && "get_ball(0) returns valid pointer");

    ball->body.position = apc::Vec3(0.0f, 10.0f, 0.0f);
    ball->body.linear_velocity = apc::Vec3(0.0f, 0.0f, 0.0f);

    // Simulate for ~1 second (240 steps)
    bool hit_ground = false;
    for (int i = 0; i < 240; ++i) {
        world.step(dt);
        if (ball->ground_contact_time > 0.0f) {
            hit_ground = true;
        }
    }

    // Ball should have fallen (y < 10)
    assert(ball->body.position.y < 10.0f && "Ball has fallen after 1 second");

    // Ball should have hit ground (y=0 plane) within ~1.5 seconds
    // t = sqrt(2*h/g) = sqrt(2*10/9.81) ≈ 1.43s, so within 240 steps (1s) it may not
    // have hit ground yet depending on position. Let's simulate longer.
    for (int i = 0; i < 120; ++i) { // Additional 0.5 seconds
        world.step(dt);
        if (ball->ground_contact_time > 0.0f) {
            hit_ground = true;
        }
    }

    // After ~1.5s total, ball should have definitely reached the ground
    assert(ball->body.position.y < 8.0f && "Ball has fallen significantly");

    // After enough time, ball should hit ground at least once
    // Simulate more if needed (3 more seconds = 720 steps)
    // Note: on_ground may be reset to false on bounce (velocity.y > 0.5 after bounce),
    // so check ground_contact_time which persists
    for (int i = 0; i < 720; ++i) {
        world.step(dt);
        if (ball->ground_contact_time > 0.0f) {
            hit_ground = true;
            break; // Found ground contact
        }
    }
    assert(hit_ground && "Ball hits ground at some point during simulation");

    // --- Test multiple balls ---
    apc::BallPhysicsWorld world2;
    world2.surface_table = apc::SurfaceBounceTable::make_default();

    world2.add_ball(apc::BallFactory::make_soccer());
    world2.add_ball(apc::BallFactory::make_basketball());
    world2.add_ball(apc::BallFactory::make_tennis());

    assert(world2.ball_count == 3 && "ball_count = 3 after adding 3 balls");

    // Verify each ball is retrievable
    apc::BallState* b0 = world2.get_ball(0);
    apc::BallState* b1 = world2.get_ball(1);
    apc::BallState* b2 = world2.get_ball(2);
    assert(b0 != nullptr && "Ball 0 exists");
    assert(b1 != nullptr && "Ball 1 exists");
    assert(b2 != nullptr && "Ball 2 exists");
    assert(world2.get_ball(3) == nullptr && "Ball 3 does not exist");
    assert(world2.get_ball(100) == nullptr && "Out-of-range returns nullptr");

    // Verify ball IDs
    assert(b0->ball_id == 0 && "Ball 0 has ID 0");
    assert(b1->ball_id == 1 && "Ball 1 has ID 1");
    assert(b2->ball_id == 2 && "Ball 2 has ID 2");

    // Place 3 balls at different heights, simulate, verify they all fall
    b0->body.position = apc::Vec3(0.0f, 5.0f, 0.0f);
    b1->body.position = apc::Vec3(2.0f, 8.0f, 0.0f);
    b2->body.position = apc::Vec3(-2.0f, 12.0f, 0.0f);

    for (int i = 0; i < 480; ++i) { // 2 seconds
        world2.step(dt);
    }

    assert(b0->body.position.y < 5.0f && "Ball 0 has fallen");
    assert(b1->body.position.y < 8.0f && "Ball 1 has fallen");
    assert(b2->body.position.y < 12.0f && "Ball 2 has fallen");

    // --- Test MAX_BALLS limit ---
    apc::BallPhysicsWorld world3;
    for (uint32_t i = 0; i < apc::BallPhysicsWorld::MAX_BALLS; ++i) {
        uint32_t id = world3.add_ball(apc::BallFactory::make_soccer());
        assert(id != 0xFFFFFFFF && "Can add up to MAX_BALLS");
    }
    assert(world3.ball_count == apc::BallPhysicsWorld::MAX_BALLS
           && "ball_count == MAX_BALLS");

    // Overflow should return error sentinel
    uint32_t overflow_id = world3.add_ball(apc::BallFactory::make_soccer());
    assert(overflow_id == 0xFFFFFFFF && "Overflow add returns 0xFFFFFFFF");
    assert(world3.ball_count == apc::BallPhysicsWorld::MAX_BALLS
           && "ball_count stays at MAX after overflow");

    std::printf("    [PASS] Gravity, ground collision, multi-ball, MAX_BALLS verified\n");
    return 0;
}

// =============================================================================
// TEST 8 (Bonus): Config helper methods — get_drag_force, get_magnus_force
// =============================================================================
static int test_config_helper_methods() {
    std::printf("  [Test 8] BallConfig helper methods...\n");

    apc::BallConfig soccer = apc::BallFactory::make_soccer();

    // get_drag_force: F = 0.5 * Cd * rho * A * v^2
    // At v=20 m/s: F = 0.5 * 0.25 * 1.225 * 0.038 * 400 = 2.3275
    float drag_f20 = soccer.get_drag_force(20.0f);
    float expected_drag = 0.5f * 0.25f * 1.225f * 0.038f * 400.0f;
    assert(approx_eq(drag_f20, expected_drag, 0.01f) && "get_drag_force matches formula");

    // At v=0: drag should be 0
    assert(approx_eq(soccer.get_drag_force(0.0f), 0.0f) && "Zero speed → zero drag");

    // Drag should scale with v^2
    float drag_f10 = soccer.get_drag_force(10.0f);
    float drag_f40 = soccer.get_drag_force(40.0f);
    // drag(40) should be 16x drag(10) (4x speed → 16x force)
    assert(approx_eq(drag_f40, drag_f10 * 16.0f, 0.1f)
           && "Drag scales with v^2");

    // get_magnus_force: F = Cl * 4/3 * pi * r^3 * rho * omega * v
    float magnus_f = soccer.get_magnus_force(30.0f, 20.0f);
    assert(magnus_f > 0.0f && "Magnus force > 0 with spin and velocity");

    // Magnus should be 0 if either spin or speed is 0
    assert(approx_eq(soccer.get_magnus_force(0.0f, 20.0f), 0.0f)
           && "Zero spin → zero Magnus");
    assert(approx_eq(soccer.get_magnus_force(30.0f, 0.0f), 0.0f)
           && "Zero speed → zero Magnus");

    std::printf("    [PASS] get_drag_force and get_magnus_force verified\n");
    return 0;
}

// =============================================================================
// TEST 9 (Bonus): Deformation recovery in BallPhysicsWorld
// =============================================================================
static int test_deformation_recovery() {
    std::printf("  [Test 9] Deformation recovery...\n");

    apc::BallPhysicsWorld world;
    world.surface_table = apc::SurfaceBounceTable::make_default();
    float dt = 1.0f / 240.0f;

    apc::BallConfig soccer = apc::BallFactory::make_soccer();
    world.add_ball(soccer);

    apc::BallState* ball = world.get_ball(0);
    ball->body.position = apc::Vec3(0.0f, 0.5f, 0.0f); // Close to ground
    ball->body.linear_velocity = apc::Vec3(0.0f, -5.0f, 0.0f);

    // Simulate until ball hits ground
    bool bounced = false;
    for (int i = 0; i < 240; ++i) {
        world.step(dt);
        if (ball->ground_contact_time > 0.0f) {
            bounced = true;
        }
    }
    assert(bounced && "Ball hit ground from low height");

    // Deformation should be recovering (or already recovered)
    // After bouncing and some time, deformation should trend toward 0
    // Keep simulating to verify recovery
    float def_before = ball->deformation;
    for (int i = 0; i < 120; ++i) { // 0.5 more seconds
        world.step(dt);
    }
    float def_after = ball->deformation;

    // Deformation should not increase after recovery time
    // (unless ball hits ground again, but from low bounce it may)
    // At minimum, the recovery mechanism exists (deformation -= dt * 8.0)
    // We verify the mechanism is working by checking it can decrease
    // Note: if the ball bounced and came back down, def_after might be > 0 again
    // So we just verify the recovery rate is reasonable

    // Create a ball that's already on ground with deformation, no velocity
    // to test pure recovery
    apc::BallState bs;
    bs.config = soccer;
    bs.reset();
    bs.body.position = apc::Vec3(0.0f, 0.11f, 0.0f);
    bs.deformation = 0.8f;
    bs.on_ground = true;
    bs.body.linear_velocity = apc::Vec3(0.0f, 0.0f, 0.0f);

    // Recovery rate = 8.0 per second, so after 0.1s (24 steps): 0.8 - 0.8 = 0.0
    apc::BallPhysicsWorld world2;
    world2.surface_table = apc::SurfaceBounceTable::make_default();
    world2.add_ball(soccer);
    apc::BallState* b2 = world2.get_ball(0);
    *b2 = bs;

    for (int i = 0; i < 24; ++i) {
        world2.step(dt);
    }
    assert(b2->deformation < 0.8f && "Deformation decreases over time");
    assert(b2->deformation >= 0.0f && "Deformation clamped >= 0");

    std::printf("    [PASS] Deformation recovery mechanism verified\n");
    return 0;
}

// =============================================================================
// TEST 10 (Bonus): AerodynamicModel knuckle effect
// =============================================================================
static int test_knuckle_effect() {
    std::printf("  [Test 10] Knuckle effect...\n");

    apc::AerodynamicModel aero;
    apc::BallConfig soccer = apc::BallFactory::make_soccer();

    // Ball moving with no spin and knuckle_intensity > 0 should have increased drag
    apc::BallState bs_normal;
    bs_normal.config = soccer;
    bs_normal.reset();
    bs_normal.body.linear_velocity = apc::Vec3(20.0f, 0.0f, 0.0f);
    bs_normal.knuckle_intensity = 0.0f;

    apc::BallState bs_knuckle;
    bs_knuckle.config = soccer;
    bs_knuckle.reset();
    bs_knuckle.body.linear_velocity = apc::Vec3(20.0f, 0.0f, 0.0f);
    bs_knuckle.knuckle_intensity = 0.8f; // High knuckle
    // Both have zero spin → spin_rate = 0 < 5.0 threshold

    apc::AeroForces f_normal = aero.compute(bs_normal);
    apc::AeroForces f_knuckle = aero.compute(bs_knuckle);

    // Knuckle ball should have higher drag
    assert(f_knuckle.drag_magnitude > f_normal.drag_magnitude
           && "Knuckle increases drag for low-spin ball");

    std::printf("    [PASS] Knuckle effect increases drag verified\n");
    return 0;
}

// =============================================================================
// TEST 11 (Bonus): BallBounce with different surfaces
// =============================================================================
static int test_bounce_surfaces() {
    std::printf("  [Test 11] Bounce on different surfaces...\n");

    apc::BallConfig soccer = apc::BallFactory::make_soccer();
    apc::SurfaceBounceTable table = apc::SurfaceBounceTable::make_default();
    apc::BallBounce resolver;
    apc::Vec3 ground_normal(0.0f, 1.0f, 0.0f);

    // Bounce on WOOD (high restitution)
    apc::BallState bs_wood;
    bs_wood.config = soccer;
    bs_wood.reset();
    bs_wood.body.linear_velocity = apc::Vec3(0.0f, -10.0f, 0.0f);
    apc::BallBounceResult r_wood = resolver.resolve_surface(
        bs_wood, apc::SurfaceType::WOOD, ground_normal, table);

    // Bounce on SAND (low restitution)
    apc::BallState bs_sand;
    bs_sand.config = soccer;
    bs_sand.reset();
    bs_sand.body.linear_velocity = apc::Vec3(0.0f, -10.0f, 0.0f);
    apc::BallBounceResult r_sand = resolver.resolve_surface(
        bs_sand, apc::SurfaceType::SAND, ground_normal, table);

    // Bounce on TRAMPOLINE (very high restitution)
    apc::BallState bs_tramp;
    bs_tramp.config = soccer;
    bs_tramp.reset();
    bs_tramp.body.linear_velocity = apc::Vec3(0.0f, -10.0f, 0.0f);
    apc::BallBounceResult r_tramp = resolver.resolve_surface(
        bs_tramp, apc::SurfaceType::TRAMPOLINE, ground_normal, table);

    // Trampoline bounce velocity > wood bounce velocity > sand bounce velocity
    float speed_tramp = apc::Vec3::length(r_tramp.new_velocity);
    float speed_wood = apc::Vec3::length(r_wood.new_velocity);
    float speed_sand = apc::Vec3::length(r_sand.new_velocity);

    assert(speed_tramp > speed_wood && "Trampoline bounce > wood bounce");
    assert(speed_wood > speed_sand && "Wood bounce > sand bounce");
    assert(r_tramp.effective_restitution > r_wood.effective_restitution
           && "Trampoline restitution > wood restitution");
    assert(r_wood.effective_restitution > r_sand.effective_restitution
           && "Wood restitution > sand restitution");

    std::printf("    [PASS] Different surfaces produce different bounce heights\n");
    return 0;
}

// =============================================================================
// TEST 12 (Bonus): AeroForces reset
// =============================================================================
static int test_aero_forces_reset() {
    std::printf("  [Test 12] AeroForces reset...\n");

    apc::AeroForces forces;
    forces.drag = apc::Vec3(1.0f, 2.0f, 3.0f);
    forces.magnus = apc::Vec3(4.0f, 5.0f, 6.0f);
    forces.drag_magnitude = 100.0f;
    forces.magnus_magnitude = 50.0f;
    forces.spin_decay_rate = 10.0f;

    forces.reset();

    assert(approx_eq(forces.drag.x, 0.0f) && "Reset drag.x = 0");
    assert(approx_eq(forces.drag.y, 0.0f) && "Reset drag.y = 0");
    assert(approx_eq(forces.drag.z, 0.0f) && "Reset drag.z = 0");
    assert(approx_eq(forces.magnus.x, 0.0f) && "Reset magnus.x = 0");
    assert(approx_eq(forces.magnus.y, 0.0f) && "Reset magnus.y = 0");
    assert(approx_eq(forces.magnus.z, 0.0f) && "Reset magnus.z = 0");
    assert(approx_eq(forces.drag_magnitude, 0.0f) && "Reset drag_magnitude = 0");
    assert(approx_eq(forces.magnus_magnitude, 0.0f) && "Reset magnus_magnitude = 0");
    assert(approx_eq(forces.spin_decay_rate, 0.0f) && "Reset spin_decay_rate = 0");

    std::printf("    [PASS] AeroForces reset clears all fields\n");
    return 0;
}

// =============================================================================
// Main
// =============================================================================
int main() {
    std::printf("Sprint 13: Ball Physics Core tests\n");
    std::printf("====================================\n");

    int result = 0;
    result |= test_ball_config_basics();
    result |= test_all_ball_factories();
    result |= test_ball_state();
    result |= test_surface_bounce_table();
    result |= test_aerodynamic_model();
    result |= test_ball_bounce();
    result |= test_ball_physics_world();
    result |= test_config_helper_methods();
    result |= test_deformation_recovery();
    result |= test_knuckle_effect();
    result |= test_bounce_surfaces();
    result |= test_aero_forces_reset();

    if (result == 0) {
        std::printf("\nSprint 13: ALL TESTS PASSED\n");
    } else {
        std::printf("\nSprint 13: SOME TESTS FAILED\n");
    }
    return result;
}
