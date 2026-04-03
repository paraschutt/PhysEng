#pragma once
// ============================================================================
// DSPE Integrator — Velocity Verlet, 4 substeps per 60Hz tick
// Force model: gravity, drag (quadratic), Magnus (ball), wind
// NOTE: drag evaluated at v(t+dt/2) — first-order approximation.
//       C_d is tuned empirically, not set to ISA theoretical values.
// ============================================================================
#include "../components.h"
#include "../materials.h"
#include "event_system.h"

namespace dspe {

// ---------------------------------------------------------------------------
// Force accumulation — computes all forces for one entity, one substep
// ---------------------------------------------------------------------------
class ForceAccumulator {
public:
    // Accumulate gravity, drag, wind into entity's force_accum
    static void accumulate(Entity& entity, const SurfaceState& surface,
                           bool half_step);

    // Ball-specific: Magnus effect and spin decay
    static void accumulate_ball(Entity& entity, const SurfaceState& surface,
                                bool half_step);

private:
    // Gravity: F = -m * g * up
    static Vec3Vel gravity_force(const RigidBody& rb);

    // Quadratic drag: F = -0.5 * rho * Cd * A * |v|*v  (evaluated at v_half)
    static Vec3Vel drag_force(const RigidBody& rb,
                              FpVel cd, FpVel cross_section,
                              bool half_step);

    // Magnus: F = S * (omega × v) where S = 0.5 * rho * A * r * CL
    // Linearly scaled by |ω| and |v| — C_L calibrated to Goff & Carré (2010)
    static Vec3Vel magnus_force(const RigidBody& rb,
                                const BallProperties& ball);

    // Wind: F = 0.5 * rho * Cd * A * |v_wind|^2 * wind_dir (ball only)
    static Vec3Vel wind_force(const RigidBody& rb,
                              const BallProperties& ball,
                              const Vec3Vel& wind_vel);

    // Spin decay torque: τ = -k_rot * ω
    static Vec3Vel spin_decay_torque(const BallProperties& ball,
                                     const Vec3Ang& spin);
};

// ---------------------------------------------------------------------------
// Velocity Verlet integrator (one substep)
// ---------------------------------------------------------------------------
class Integrator {
public:
    // Integrate one substep for all active (non-sleeping) rigid bodies
    // force_fn accumulates external forces BEFORE calling this;
    // this function computes the Verlet steps
    void substep(std::array<Entity, MAX_ENTITIES>& entities,
                 const SurfaceState& surface,
                 FpVel dt);

    // Ground constraint: prevent entities falling below y=0 (pitch surface)
    // Applies via constraint solver, but clamp here as safety measure
    static void apply_ground_clamp(Entity& e);

    // Velocity safety clamp (prevent solver explosions)
    static void clamp_velocity(Entity& e);

    // Angular velocity clamp (prevent solver explosions)
    static void clamp_angular_velocity(Entity& e);

private:
    // Single entity Verlet integration (one substep)
    void integrate_entity(Entity& e, const SurfaceState& surface, FpVel dt);
};

// ---------------------------------------------------------------------------
// Input force system — converts player input to force accumulation
// ---------------------------------------------------------------------------
struct PlayerInput {
    Vec3Vel move_dir{};        // Desired movement direction (normalised)
    FpVel   move_speed{};      // Target speed (m/s)
    bool    jump{false};
    bool    kick{false};
    bool    tackle{false};
    FpVel   kick_power{};      // [0, 1]
    Vec3Vel kick_dir{};        // Target kick direction
};

class InputSystem {
public:
    // Apply player input as forces on the skeleton's pelvis rigidbody
    // Acceleration limited by friction circle: a_max = μ * g
    static void apply(Entity& player_entity,
                      const PlayerInput& input,
                      const SurfaceState& surface,
                      EventQueue& events,
                      uint32_t frame);
};

} // namespace dspe
