// ============================================================================
// DSPE Integrator Implementation
// ============================================================================
#include "dspe/systems/integrator.h"
#include "dspe/systems/skeleton.h"
#include <algorithm>
#include <cstdlib>

namespace dspe {

// ============================================================================
// ForceAccumulator
// ============================================================================

Vec3Vel ForceAccumulator::gravity_force(const RigidBody& rb) {
    // F = m * g downward (negative Y)
    FpVel mg = rb.mass * GRAVITY;
    return { FpVel::zero(), -mg, FpVel::zero() };
}

Vec3Vel ForceAccumulator::drag_force(const RigidBody& rb,
                                      FpVel cd, FpVel cross_section,
                                      bool half_step) {
    // F_drag = -0.5 * rho * Cd * A * |v| * v
    // NOTE: half_step=true means we use velocity at v(t+dt/2) — first-order approx
    // C_d is tuned empirically to compensate for this approximation
    (void)half_step; // velocity in rb is already the half-step vel at call time
    Vec3Vel v = rb.velocity;
    FpVel vsq = v.length_sq();
    if (vsq.raw == 0) return Vec3Vel::zero();
    FpVel speed = fp_sqrt(vsq);

    // factor = -0.5 * rho * Cd * A  (all in Q15.16)
    FpVel half      = FpVel::from_float(0.5f);
    FpVel factor    = half * AIR_DENSITY * cd * cross_section;
    factor          = -factor;  // drag opposes motion

    // F = factor * speed * v_normalised = factor * v (since speed * (v/speed) = v)
    return { factor * speed * v.x / speed,
             factor * speed * v.y / speed,
             factor * speed * v.z / speed };
}

Vec3Vel ForceAccumulator::magnus_force(const RigidBody& rb,
                                        const BallProperties& ball) {
    // F_Magnus = S * (ω × v)
    // S = 0.5 * ρ * A * r * C_L  (units: kg/m)
    // ω × v has units rad/s * m/s = m/s² ; S * (ω×v) = kg/m * m/s² = N ✓
    // Note: linear in both |ω| and |v| — C_L calibrated to Goff & Carré (2010)

    // Convert spin from Q8.24 to Q15.16 for cross product with velocity
    Vec3Vel omega = {
        FpVel{ball.spin.x.raw >> 8},  // Q8.24 → Q15.16 shift by 24-16=8
        FpVel{ball.spin.y.raw >> 8},
        FpVel{ball.spin.z.raw >> 8},
    };
    Vec3Vel cross = omega.cross(rb.velocity);

    FpVel half = FpVel::from_float(0.5f);
    FpVel area = BALL_RADIUS * BALL_RADIUS * FpVel::from_float(3.14159f); // π*r^2
    FpVel S    = half * AIR_DENSITY * area * BALL_RADIUS * ball.magnus_cl;

    return cross * S;
}

Vec3Vel ForceAccumulator::wind_force(const RigidBody& rb,
                                      const BallProperties& ball,
                                      const Vec3Vel& wind_vel) {
    // F_wind = 0.5 * ρ * Cd * A * |v_wind|^2 * wind_dir
    // Applied to ball only (mass << player mass so effect is meaningful)
    FpVel speed_sq = wind_vel.length_sq();
    if (speed_sq.raw == 0) return Vec3Vel::zero();
    FpVel speed = fp_sqrt(speed_sq);
    Vec3Vel wind_dir = wind_vel / speed;
    FpVel half = FpVel::from_float(0.5f);
    FpVel area = BALL_RADIUS * BALL_RADIUS * FpVel::from_float(3.14159f);
    FpVel mag  = half * AIR_DENSITY * ball.drag_cd * area * speed_sq;
    return wind_dir * mag;
}

Vec3Vel ForceAccumulator::spin_decay_torque(const BallProperties& ball,
                                             const Vec3Ang& spin) {
    // τ_drag = -k_rot * ω  (opposing spin)
    Vec3Vel omega = {
        FpVel{spin.x.raw >> 8},
        FpVel{spin.y.raw >> 8},
        FpVel{spin.z.raw >> 8},
    };
    return omega * (-ball.spin_decay_k);
}

void ForceAccumulator::accumulate(Entity& entity,
                                   const SurfaceState& surface,
                                   bool half_step) {
    if (!entity.has(COMP_RIGIDBODY)) return;
    RigidBody& rb = entity.rigidbody;
    if (rb.is_static()) return;

    rb.apply_force(gravity_force(rb));
    rb.apply_force(drag_force(rb, rb.drag_cd, rb.cross_section_area, half_step));
}

void ForceAccumulator::accumulate_ball(Entity& entity,
                                        const SurfaceState& surface,
                                        bool half_step) {
    if (!entity.has(COMP_RIGIDBODY | COMP_BALL_PROPERTIES)) return;
    RigidBody& rb = entity.rigidbody;
    BallProperties& ball = entity.ball;

    // Base forces
    accumulate(entity, surface, half_step);

    // Magnus
    rb.apply_force(magnus_force(rb, ball));

    // Wind
    rb.apply_force(wind_force(rb, ball, surface.wind_vel));

    // Spin decay (applied as torque on ball angular velocity)
    Vec3Vel decay = spin_decay_torque(ball, ball.spin);
    // Convert back to Q8.24 and apply
    ball.spin.x.raw -= (decay.x.raw << 8) / 1;  // simplified
    ball.spin.y.raw -= (decay.y.raw << 8) / 1;
    ball.spin.z.raw -= (decay.z.raw << 8) / 1;
}

// ============================================================================
// Integrator
// ============================================================================

void Integrator::substep(std::array<Entity, MAX_ENTITIES>& entities,
                          const SurfaceState& surface,
                          FpVel dt) {
    // Process entities in deterministic order (by ID)
    for (EntityId id = 0; id < MAX_ENTITIES; ++id) {
        Entity& e = entities[id];
        if (!e.has(COMP_RIGIDBODY)) continue;
        if (e.has(COMP_SLEEP))      continue;  // sleeping: skip
        if (e.rigidbody.is_static()) continue; // static: skip
        integrate_entity(e, surface, dt);
    }
}

void Integrator::integrate_entity(Entity& e,
                                   const SurfaceState& surface,
                                   FpVel dt) {
    RigidBody& rb = e.rigidbody;

    // ─── Step 1: Half-step velocity  ────────────────────────────────────────
    // v(t + dt/2) = v(t) + 0.5 * a(t) * dt
    FpVel half_dt = dt >> 1;  // dt/2 in Q15.16
    rb.velocity = vel_add_acc_dt(rb.velocity, rb.acceleration, half_dt);

    // ─── Step 2: Full-step position  ────────────────────────────────────────
    // x(t + dt) = x(t) + v(t + dt/2) * dt
    rb.position = pos_add_vel_dt(rb.position, rb.velocity, dt);

    // ─── Step 3: Accumulate forces at new position (with half-step velocity) ─
    rb.clear_forces();
    if (e.has(COMP_BALL_PROPERTIES)) {
        ForceAccumulator::accumulate_ball(e, surface, /*half_step=*/true);
    } else {
        ForceAccumulator::accumulate(e, surface, /*half_step=*/true);
    }

    // ─── Step 4: New acceleration from forces ───────────────────────────────
    // a(t+dt) = F / m
    if (rb.inv_mass.raw != 0) {
        rb.acceleration = {
            rb.force_accum.x * rb.inv_mass,
            rb.force_accum.y * rb.inv_mass,
            rb.force_accum.z * rb.inv_mass,
        };
    }

    // ─── Step 5: Complete velocity step ─────────────────────────────────────
    // v(t + dt) = v(t + dt/2) + 0.5 * a(t + dt) * dt
    rb.velocity = vel_add_acc_dt(rb.velocity, rb.acceleration, half_dt);

    // ─── Safety clamps ───────────────────────────────────────────────────────
    clamp_velocity(e);
    apply_ground_clamp(e);
}

void Integrator::apply_ground_clamp(Entity& e) {
    RigidBody& rb = e.rigidbody;
    // Ground is at y=0; collider radius offsets the clamp
    FpPos ground_y = FpPos::zero();
    if (e.has(COMP_COLLIDER)) {
        if (e.collider.shape_type == ShapeType::SPHERE) {
            ground_y = e.collider.sphere.radius;
        } else if (e.collider.shape_type == ShapeType::CAPSULE) {
            ground_y = e.collider.capsule.radius;
        }
    }
    if (rb.position.y < ground_y) {
        rb.position.y = ground_y;
        if (rb.velocity.y.raw < 0) rb.velocity.y = FpVel::zero();
    }
}

void Integrator::clamp_velocity(Entity& e) {
    RigidBody& rb = e.rigidbody;
    FpVel speed_sq = rb.velocity.length_sq();
    FpVel max_sq   = MAX_SPEED * MAX_SPEED;
    if (speed_sq > max_sq) {
        FpVel speed = fp_sqrt(speed_sq);
        rb.velocity = rb.velocity * (MAX_SPEED / speed);
    }
}

// ============================================================================
// InputSystem
// ============================================================================

void InputSystem::apply(Entity& player_entity,
                         const PlayerInput& input,
                         const SurfaceState& surface,
                         EventQueue& events,
                         uint32_t frame) {
    if (!player_entity.has(COMP_RIGIDBODY | COMP_SKELETON)) return;
    RigidBody& rb = player_entity.rigidbody;
    Skeleton&  sk = player_entity.skeleton;

    // Friction circle: a_max = μ * g  (player can only push as hard as ground allows)
    FpVel mu      = surface.apply_wetness(
                        get_material(MAT_DRY_GRASS).kinetic_mu);
    FpVel a_max   = mu * GRAVITY;  // m/s²

    // Desired acceleration toward target velocity
    Vec3Vel current_vel_xz = { rb.velocity.x, FpVel::zero(), rb.velocity.z };
    Vec3Vel target_vel     = input.move_dir * input.move_speed;
    Vec3Vel delta_vel      = {
        target_vel.x - current_vel_xz.x,
        FpVel::zero(),
        target_vel.z - current_vel_xz.z,
    };

    // Clamp to friction circle
    FpVel delta_mag = delta_vel.length();
    Vec3Vel accel_dir;
    if (delta_mag.raw > 0) {
        accel_dir = delta_vel / delta_mag;
        FpVel clamped_a = delta_mag < a_max ? delta_mag : a_max;
        Vec3Vel force = accel_dir * (clamped_a * rb.mass);
        rb.apply_force(force);
    }

    // Jump
    if (input.jump && sk.left_foot_grounded || sk.right_foot_grounded) {
        SkeletonController::apply_jump(player_entity,
                                       FpVel::from_float(400.0f)); // 400N impulse
        sk.anim_state = Skeleton::AnimState::JUMP;
    }

    // Tackle: handled by skeleton controller via animation state
    if (input.tackle) {
        sk.anim_state = Skeleton::AnimState::TACKLE;
    }
}

} // namespace dspe