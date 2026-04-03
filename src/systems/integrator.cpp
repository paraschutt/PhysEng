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
    if (speed.raw == 0) return Vec3Vel::zero();  // guard sqrt truncation on tiny vsq

    // F_drag = -0.5 * rho * Cd * A * |v| * v_hat
    //        = -0.5 * rho * Cd * A * v  (since |v| * v_hat = v)
    // So F.x = scale * v.x  where scale = -0.5 * rho * Cd * A * |v| / |v| = -0.5*rho*Cd*A
    // But we need the quadratic part: F = scale * |v| * v_hat = scale * v
    // scale = -0.5 * rho * Cd * A  (units: kg/m)  — F = scale * v gives N
    FpVel half  = FpVel::from_float(0.5f);
    FpVel scale = -(half * AIR_DENSITY * cd * cross_section);

    // F.x = scale * |v| * (v.x / |v|) = scale * v.x  — no division needed
    return { scale * v.x,
             scale * v.y,
             scale * v.z };
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
    if (speed.raw == 0) return Vec3Vel::zero();
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
    // Emergency backstop ONLY — fires when entity falls more than 5m below ground.
    // Normal ground contact (including bouncing) is handled by the constraint solver
    // via sphere_box / capsule_box contact manifolds.
    // DO NOT reset position to ball_radius here — that puts the ball at exactly
    // dist == rs, which the narrow phase misses due to fixed-point rounding.
    FpPos deep_threshold = FpPos::from_float(-5.0f);
    if (rb.position.y < deep_threshold) {
        rb.position.y = FpPos::zero();
        rb.velocity.y = FpVel::zero();
    }
}

void Integrator::clamp_velocity(Entity& e) {
    RigidBody& rb = e.rigidbody;
    // BUG GUARD: Q15.16 cannot represent 200^2 = 40000 (max ~32767).
    // MAX_SPEED.raw = 13,107,200; squaring via operator* overflows int32_t to
    // a negative number, making the clamp trigger on every entity every substep.
    // Fix: compare speed_sq against max_sq using int64 raw arithmetic.
    int64_t vx = rb.velocity.x.raw;
    int64_t vy = rb.velocity.y.raw;
    int64_t vz = rb.velocity.z.raw;
    int64_t speed_sq_raw2 = vx*vx + vy*vy + vz*vz;   // units: raw^2 (not Q15.16)
    int64_t max_raw       = MAX_SPEED.raw;
    int64_t max_sq_raw2   = max_raw * max_raw;         // stays in int64, no overflow
    if (speed_sq_raw2 <= max_sq_raw2) return;          // common case: nothing to do

    // Compute speed in raw units using integer sqrt (Newton-Raphson on int64)
    int64_t speed_raw = max_raw; // initial guess = MAX_SPEED
    for (int i = 0; i < 16; ++i) {
        if (speed_raw == 0) break;
        int64_t next = (speed_raw + speed_sq_raw2 / speed_raw) >> 1;
        if (next >= speed_raw) break;
        speed_raw = next;
    }
    if (speed_raw == 0) return;

    // Scale each component: v_new = v * (MAX_SPEED.raw / speed_raw)
    // Both are in the same raw units so the ratio is dimensionless
    rb.velocity.x.raw = (int32_t)(vx * max_raw / speed_raw);
    rb.velocity.y.raw = (int32_t)(vy * max_raw / speed_raw);
    rb.velocity.z.raw = (int32_t)(vz * max_raw / speed_raw);
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

    // Friction circle: a_max = μ * g
    // Use the ground surface kinetic_mu, then apply wetness.
    // Surface kinetic_mu is taken from the ground material (MAT_DRY_GRASS base)
    // and modulated by wetness. At full wet (wetness=0), factor=0.5:
    //   0.60 * 0.5 = 0.30  (too low vs brief's 0.35 for wet grass)
    // Use MAT_WET_GRASS directly when surface is predominantly wet.
    // Threshold: wetness < 0.5 → use wet grass material directly.
    MaterialId surface_mat = (surface.wetness < FpVel::from_float(0.5f))
                             ? MAT_WET_GRASS : MAT_DRY_GRASS;
    FpVel base_mu = get_material(surface_mat).kinetic_mu;
    FpVel mu      = surface.apply_wetness(base_mu);
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