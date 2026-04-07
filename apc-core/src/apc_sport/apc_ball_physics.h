#pragma once
// =============================================================================
// Ball Physics — Sport ball types, aerodynamics, spin, bounce curves
// =============================================================================
//
// Provides a comprehensive ball physics model supporting both round balls
// (soccer, basketball, tennis, volleyball, golf) and oval/prolate balls
// (rugby, American football, Australian rules football).
//
// Key features:
//   - BallShape: sphere (round) or prolate spheroid (oval)
//   - BallState: position, velocity, angular velocity (spin), deformation
//   - AerodynamicModel: drag + Magnus lift/spin forces
//   - SurfaceBounceTable: per-surface restitution (grass, turf, court, wood, etc.)
//   - BallFactory: preset configurations for each sport
//   - SpinDecay: angular velocity damping over time
//   - KnuckleEffect: unpredictable flutter for low-spin kicks
//
// All physics is deterministic — fixed-order float operations, no FMA, no SIMD.
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation
//   - C++17
//
// =============================================================================

#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include "apc_math/apc_math_common.h"
#include "apc_solver/apc_rigid_body.h"
#include <cstdint>
#include <cmath>

namespace apc {

// =============================================================================
// BallShape — Geometry type for sport balls
// =============================================================================
enum class BallShape : uint8_t {
    SPHERE = 0,         // Round balls: soccer, basketball, tennis, volleyball, golf
    PROLATE = 1         // Oval balls: rugby, American football, Australian rules
};

// =============================================================================
// SurfaceType — Playing surface for bounce curve lookups
// =============================================================================
enum class SurfaceType : uint8_t {
    GRASS       = 0,    // Natural grass
    ARTIFICIAL  = 1,    // Artificial turf
    CLAY        = 2,    // Tennis clay court
    HARD_COURT  = 3,    // Tennis hard court / concrete
    WOOD        = 4,    // Indoor basketball court (hardwood)
    CARPET      = 5,    // Indoor artificial
    SAND        = 6,    // Beach volleyball
    ICE         = 7,    // Ice hockey / bandy
    WATER       = 8,    // Water polo
    TRAMPOLINE  = 9,    // Extreme sports
    MAT         = 10,   // Wrestling / gymnastics mat
    CONCRETE    = 11,   // Skate park / urban
    CUSTOM_0    = 12,
    CUSTOM_1    = 13,
    MAX_SURFACES = 14
};

static constexpr uint32_t NUM_SURFACE_TYPES = 14;

// =============================================================================
// SpinAxis — Semantic spin axis labels for sport context
// =============================================================================
enum class SpinAxis : uint8_t {
    TOPSPIN = 0,     // Forward rotation (ball dips)
    BACKSPIN = 1,    // Backward rotation (ball floats/rides)
    SIDESPIN = 2,    // Lateral rotation (ball curves)
    HELICAL = 3      // Complex spin (kickoff, rugby spiral)
};

// =============================================================================
// BallConfig — Physical properties defining a ball type
// =============================================================================
struct BallConfig {
    BallShape shape = BallShape::SPHERE;
    const char* name = "default";

    // --- Dimensions ---
    float radius = 0.11f;            // Radius for spheres (meters)
    float semi_major = 0.11f;        // Long axis for prolate (meters)
    float semi_minor = 0.11f;        // Short axis for prolate (meters)
    float mass = 0.43f;              // Mass in kg
    float moment_of_inertia = 0.004f; // I = 2/5 * m * r^2 for sphere

    // --- Aerodynamics ---
    float drag_coefficient = 0.47f;   // Cd for sphere in air
    float cross_section_area = 0.038f;// Frontal area for drag (m^2)
    float magnus_coefficient = 0.33f; // Cl for Magnus effect
    float air_density = 1.225f;       // kg/m^3 at sea level

    // --- Surface interaction ---
    float friction_coefficient = 0.4f; // Sliding friction on default surface
    float rolling_friction = 0.01f;   // Rolling resistance
    float spin_friction = 0.05f;      // Spin damping on bounce

    // --- Bounce ---
    float base_restitution = 0.75f;   // Coefficient of restitution
    float restitution_velocity_scale = 0.01f; // Restitution drops at high speed
    float max_bounce_angle = 1.5f;    // Max reflection angle modifier (rad)

    // --- Sport-specific ---
    SpinAxis preferred_spin = SpinAxis::SIDESPIN;
    bool has_panel_seams = true;       // Seam pattern affects aerodynamics
    float seam_drag_modifier = 1.0f;   // Extra drag from seams (1.0 = none)

    // --- Compute derived quantities ---
    float get_effective_radius() const {
        return (shape == BallShape::PROLATE) ?
               (semi_major + semi_minor) * 0.5f : radius;
    }

    float get_drag_force(float speed) const {
        // F_drag = 0.5 * Cd * rho * A * v^2
        return 0.5f * drag_coefficient * air_density * cross_section_area * speed * speed;
    }

    float get_magnus_force(float spin_rate, float speed) const {
        // F_magnus = Cl * (4/3) * pi * r^3 * rho * (omega x v)
        float r3 = radius * radius * radius;
        return magnus_coefficient * 4.18879f * r3 * air_density * spin_rate * speed;
    }

    // --- RigidBody from config ---
    RigidBody to_rigid_body() const {
        RigidBody body;
        body.position = Vec3(0.0f, 0.0f, 0.0f);
        body.linear_velocity = Vec3(0.0f, 0.0f, 0.0f);
        body.orientation = Quat::identity();
        body.angular_velocity = Vec3(0.0f, 0.0f, 0.0f);
        body.inverse_mass = (mass > 0.0f) ? (1.0f / mass) : 0.0f;

        float inv_I = (moment_of_inertia > 0.0f) ? (1.0f / moment_of_inertia) : 0.0f;
        float r = get_effective_radius();
        body.local_inverse_inertia = Mat3{
            {inv_I, 0.0f, 0.0f,
             0.0f, inv_I, 0.0f,
             0.0f, 0.0f, inv_I}
        };
        body.update_world_inertia();
        return body;
    }
};

// =============================================================================
// BallState — Runtime state of a ball (wraps RigidBody with sport extensions)
// =============================================================================
struct BallState {
    RigidBody body;
    BallConfig config;
    uint32_t ball_id = 0;

    // --- Runtime spin info ---
    float spin_rate = 0.0f;          // Scalar angular speed (rad/s)
    Vec3 spin_axis = Vec3(0.0f, 1.0f, 0.0f); // Current spin axis (world)
    SpinAxis spin_type = SpinAxis::SIDESPIN;

    // --- Deformation (visual/gameplay) ---
    float deformation = 0.0f;        // 0.0 = none, 1.0 = max squash
    Vec3 deformation_axis = Vec3(0.0f, 1.0f, 0.0f); // Direction of squash

    // --- State flags ---
    bool on_ground = false;
    bool in_air = true;
    bool in_water = false;
    SurfaceType current_surface = SurfaceType::GRASS;
    float ground_contact_time = 0.0f; // Seconds since last ground contact

    // --- Knuckle effect ---
    float knuckle_intensity = 0.0f;  // 0.0-1.0, how much unpredictable flutter

    void reset() {
        body = config.to_rigid_body();
        spin_rate = 0.0f;
        spin_axis = Vec3(0.0f, 1.0f, 0.0f);
        spin_type = SpinAxis::SIDESPIN;
        deformation = 0.0f;
        on_ground = false;
        in_air = true;
        in_water = false;
        current_surface = SurfaceType::GRASS;
        ground_contact_time = 0.0f;
        knuckle_intensity = 0.0f;
    }

    void update_spin_info() {
        spin_rate = Vec3::length(body.angular_velocity);
        if (spin_rate > APC_EPSILON) {
            spin_axis = Vec3::scale(body.angular_velocity, 1.0f / spin_rate);
        } else {
            spin_axis = Vec3(0.0f, 1.0f, 0.0f);
        }

        // Classify spin type based on alignment with velocity
        Vec3 vel = body.linear_velocity;
        float speed = Vec3::length(vel);
        if (speed < APC_EPSILON || spin_rate < APC_EPSILON) {
            spin_type = SpinAxis::HELICAL;
            return;
        }

        Vec3 vel_dir = Vec3::scale(vel, 1.0f / speed);
        Vec3 up = Vec3(0.0f, 1.0f, 0.0f);

        // Dot with forward direction
        float forward_dot = Vec3::dot(spin_axis, vel_dir);
        // Dot with up direction
        float up_dot = Vec3::dot(spin_axis, up);
        // Lateral component
        Vec3 lateral = Vec3::cross(vel_dir, up);
        float lat_length = Vec3::length(lateral);
        float lateral_dot = lat_length > APC_EPSILON ?
            Vec3::dot(spin_axis, Vec3::scale(lateral, 1.0f / lat_length)) : 0.0f;

        float abs_fwd = std::abs(forward_dot);
        float abs_lat = std::abs(lateral_dot);
        float abs_up = std::abs(up_dot);

        if (abs_fwd > abs_lat && abs_fwd > abs_up) {
            spin_type = (forward_dot > 0.0f) ? SpinAxis::TOPSPIN : SpinAxis::BACKSPIN;
        } else if (abs_lat > abs_up) {
            spin_type = SpinAxis::SIDESPIN;
        } else {
            spin_type = SpinAxis::HELICAL;
        }
    }
};

// =============================================================================
// SurfaceBounceTable — Per-surface bounce properties
// =============================================================================
struct SurfaceBounceEntry {
    float restitution = 0.7f;
    float friction = 0.4f;
    float rolling_resistance = 0.01f;
    float spin_damping = 0.1f;       // Fraction of spin lost on bounce
    float deformation_factor = 0.1f; // How much the ball deforms on this surface
    float bounce_sound_intensity = 0.5f; // For audio hooks
};

struct SurfaceBounceTable {
    SurfaceBounceEntry entries[NUM_SURFACE_TYPES];

    void set(SurfaceType surface, float restitution, float friction,
             float rolling = 0.01f, float spin_damp = 0.1f,
             float deform = 0.1f, float sound = 0.5f)
    {
        uint8_t idx = static_cast<uint8_t>(surface);
        if (idx >= NUM_SURFACE_TYPES) return;
        entries[idx].restitution = restitution;
        entries[idx].friction = friction;
        entries[idx].rolling_resistance = rolling;
        entries[idx].spin_damping = spin_damp;
        entries[idx].deformation_factor = deform;
        entries[idx].bounce_sound_intensity = sound;
    }

    const SurfaceBounceEntry& get(SurfaceType surface) const {
        uint8_t idx = static_cast<uint8_t>(surface);
        if (idx >= NUM_SURFACE_TYPES) idx = 0;
        return entries[idx];
    }

    SurfaceBounceEntry lookup(const BallConfig& config, SurfaceType surface,
                              float impact_speed) const
    {
        SurfaceBounceEntry base = get(surface);

        // Velocity-dependent restitution: drops at high speed
        // e = e0 - k * |v|, clamped to [0.1, e0]
        float e = base.restitution - config.restitution_velocity_scale * impact_speed;
        e = std::max(0.1f, std::min(e, base.restitution));
        base.restitution = e;

        return base;
    }

    // --- Default table with standard sport surfaces ---
    static SurfaceBounceTable make_default() {
        SurfaceBounceTable table;
        for (uint32_t i = 0; i < NUM_SURFACE_TYPES; ++i) {
            table.entries[i] = SurfaceBounceEntry();
        }

        // Natural grass — moderate bounce, decent grip
        table.set(SurfaceType::GRASS, 0.6f, 0.45f, 0.02f, 0.15f, 0.15f, 0.3f);

        // Artificial turf — slightly higher bounce, less friction
        table.set(SurfaceType::ARTIFICIAL, 0.65f, 0.35f, 0.015f, 0.1f, 0.1f, 0.4f);

        // Clay court — low bounce, high friction, heavy spin effect
        table.set(SurfaceType::CLAY, 0.55f, 0.65f, 0.025f, 0.08f, 0.2f, 0.2f);

        // Hard court — high bounce, moderate friction
        table.set(SurfaceType::HARD_COURT, 0.8f, 0.5f, 0.008f, 0.12f, 0.08f, 0.7f);

        // Hardwood — very high bounce, low friction, fast rolling
        table.set(SurfaceType::WOOD, 0.85f, 0.3f, 0.005f, 0.05f, 0.05f, 0.8f);

        // Carpet — low bounce, moderate friction
        table.set(SurfaceType::CARPET, 0.5f, 0.5f, 0.03f, 0.15f, 0.2f, 0.15f);

        // Sand — very low bounce, very high friction
        table.set(SurfaceType::SAND, 0.15f, 0.8f, 0.15f, 0.4f, 0.4f, 0.05f);

        // Ice — moderate bounce, very low friction
        table.set(SurfaceType::ICE, 0.6f, 0.03f, 0.001f, 0.02f, 0.02f, 0.3f);

        // Water — very low bounce, moderate friction
        table.set(SurfaceType::WATER, 0.1f, 0.5f, 0.1f, 0.3f, 0.3f, 0.1f);

        // Trampoline — very high bounce
        table.set(SurfaceType::TRAMPOLINE, 0.95f, 0.6f, 0.001f, 0.02f, 0.15f, 0.4f);

        // Mat — low bounce, high grip
        table.set(SurfaceType::MAT, 0.3f, 0.7f, 0.05f, 0.25f, 0.2f, 0.1f);

        // Concrete — high bounce, moderate friction, harsh sound
        table.set(SurfaceType::CONCRETE, 0.75f, 0.45f, 0.01f, 0.12f, 0.08f, 0.9f);

        return table;
    }
};

// =============================================================================
// AerodynamicModel — Computes drag and Magnus forces on a ball
// =============================================================================
struct AeroForces {
    Vec3 drag;          // Aerodynamic drag force (N)
    Vec3 magnus;        // Magnus lift/curve force (N)
    float drag_magnitude;
    float magnus_magnitude;
    float spin_decay_rate; // rad/s lost per second

    void reset() {
        drag = Vec3(0.0f, 0.0f, 0.0f);
        magnus = Vec3(0.0f, 0.0f, 0.0f);
        drag_magnitude = 0.0f;
        magnus_magnitude = 0.0f;
        spin_decay_rate = 0.0f;
    }
};

struct AerodynamicModel {
    float dt = 1.0f / 240.0f;

    // --- Compute forces on a ball ---
    AeroForces compute(const BallState& ball) const {
        AeroForces forces;
        forces.reset();

        const Vec3& vel = ball.body.linear_velocity;
        const Vec3& ang_vel = ball.body.angular_velocity;
        float speed = Vec3::length(vel);

        if (speed < APC_EPSILON) return forces;

        // --- Drag Force ---
        // F_drag = -0.5 * Cd * rho * A * |v|^2 * v_hat
        float drag_mag = 0.5f * ball.config.drag_coefficient *
                         ball.config.air_density *
                         ball.config.cross_section_area *
                         speed * speed;

        // Seam modifier: seams increase drag slightly
        if (ball.config.has_panel_seams) {
            drag_mag *= ball.config.seam_drag_modifier;
        }

        // Knuckle effect: seams on non-spinning ball cause chaotic drag
        if (ball.knuckle_intensity > 0.0f && ball.spin_rate < 5.0f) {
            drag_mag *= (1.0f + 0.3f * ball.knuckle_intensity);
        }

        Vec3 vel_hat = Vec3::scale(vel, 1.0f / speed);
        forces.drag = Vec3::scale(vel_hat, -drag_mag);
        forces.drag_magnitude = drag_mag;

        // --- Magnus Force ---
        // F_magnus = S * (omega x v) where S is the Magnus coefficient factor
        float spin_speed = Vec3::length(ang_vel);
        if (spin_speed > APC_EPSILON) {
            Vec3 cross = Vec3::cross(ang_vel, vel);
            float cross_len = Vec3::length(cross);

            if (cross_len > APC_EPSILON) {
                float magnus_factor = ball.config.magnus_coefficient *
                    0.5f * ball.config.air_density *
                    4.18879f * ball.config.radius * ball.config.radius *
                    ball.config.radius;

                // Reduce Magnus at very high speeds (Reynolds number effect)
                float reynolds_factor = 1.0f;
                if (speed > 30.0f) {
                    reynolds_factor = 30.0f / speed;
                }

                Vec3 magnus_dir = Vec3::scale(cross, 1.0f / cross_len);
                float magnus_mag = magnus_factor * spin_speed * speed * reynolds_factor;
                forces.magnus = Vec3::scale(magnus_dir, magnus_mag);
                forces.magnus_magnitude = magnus_mag;
            }
        }

        // --- Spin Decay ---
        // Angular velocity decays due to air friction on the spinning surface
        // d(omega)/dt = -k * omega
        // Typical decay: 10-30% over 1 second for sport balls
        forces.spin_decay_rate = ball.spin_rate * 0.15f; // ~15% per second base rate

        // Higher spin = slightly more decay (turbulent boundary layer)
        if (ball.spin_rate > 50.0f) {
            forces.spin_decay_rate *= 1.0f + (ball.spin_rate - 50.0f) * 0.002f;
        }

        return forces;
    }

    // --- Apply aerodynamic forces to ball state ---
    void apply(BallState& ball) const {
        AeroForces forces = compute(ball);

        // Apply drag: F = ma, so a = F/m, dv = a * dt
        if (ball.body.inverse_mass > 0.0f) {
            Vec3 drag_accel = Vec3::scale(forces.drag, ball.body.inverse_mass);
            ball.body.linear_velocity = Vec3::add(ball.body.linear_velocity,
                Vec3::scale(drag_accel, dt));

            // Apply Magnus
            Vec3 magnus_accel = Vec3::scale(forces.magnus, ball.body.inverse_mass);
            ball.body.linear_velocity = Vec3::add(ball.body.linear_velocity,
                Vec3::scale(magnus_accel, dt));

            // Apply spin decay: omega_new = omega * (1 - decay_rate * dt)
            float decay = 1.0f - forces.spin_decay_rate * dt;
            decay = std::max(0.0f, decay); // Clamp to zero
            ball.body.angular_velocity = Vec3::scale(ball.body.angular_velocity, decay);
        }

        ball.update_spin_info();
    }
};

// =============================================================================
// BallBounce — Bounce resolution for ball-surface and ball-body contacts
// =============================================================================
struct BallBounceResult {
    Vec3 new_velocity;            // Post-bounce velocity
    Vec3 new_angular_velocity;    // Post-bounce angular velocity
    float effective_restitution;  // Actual restitution used
    float spin_transfer;          // How much spin was gained/lost
    float deformation_amount;     // Visual deformation
    bool triggered;               // Was a bounce processed?
};

struct BallBounce {
    // --- Resolve ball hitting a flat surface ---
    BallBounceResult resolve_surface(BallState& ball, SurfaceType surface,
                                      const Vec3& surface_normal,
                                      const SurfaceBounceTable& table) const
    {
        BallBounceResult result;
        result.triggered = false;
        result.deformation_amount = 0.0f;
        result.spin_transfer = 0.0f;
        result.effective_restitution = 0.0f;

        // Relative velocity (ball velocity toward surface)
        float vel_along_normal = Vec3::dot(ball.body.linear_velocity, surface_normal);

        // Only resolve if ball is moving toward surface
        if (vel_along_normal >= 0.0f) return result;

        float impact_speed = std::abs(vel_along_normal);

        // Look up surface bounce properties
        SurfaceBounceEntry entry = table.lookup(ball.config, surface, impact_speed);

        // Decompose velocity into normal and tangential
        Vec3 vel_normal = Vec3::scale(surface_normal, vel_along_normal);
        Vec3 vel_tangent = Vec3::sub(ball.body.linear_velocity, vel_normal);

        // Apply restitution to normal component (reverse and scale)
        result.new_velocity = Vec3::add(
            Vec3::scale(vel_normal, -entry.restitution),
            vel_tangent
        );

        // Apply surface friction to tangential component
        float tangent_speed = Vec3::length(vel_tangent);
        if (tangent_speed > APC_EPSILON) {
            float friction_decel = entry.friction * impact_speed;
            float new_tangent_speed = tangent_speed - friction_decel;
            if (new_tangent_speed < 0.0f) new_tangent_speed = 0.0f;
            float tangent_scale = new_tangent_speed / tangent_speed;
            result.new_velocity = Vec3::add(
                Vec3::scale(vel_normal, -entry.restitution),
                Vec3::scale(vel_tangent, tangent_scale)
            );

            // Tangential friction applies torque → spin transfer
            result.spin_transfer = (tangent_speed - new_tangent_speed) /
                                  (ball.config.get_effective_radius() + APC_EPSILON);
        }

        // Spin modification on bounce
        Vec3 spin_change = Vec3::cross(
            Vec3::scale(surface_normal, -1.0f),
            vel_tangent
        );
        float spin_change_mag = Vec3::length(spin_change);
        if (spin_change_mag > APC_EPSILON) {
            // Add surface spin effect scaled by entry
            Vec3 surface_spin = Vec3::scale(
                Vec3::scale(spin_change, 1.0f / spin_change_mag),
                result.spin_transfer * 0.5f
            );
            result.new_angular_velocity = Vec3::add(
                ball.body.angular_velocity,
                surface_spin
            );
            // Apply spin damping
            result.new_angular_velocity = Vec3::scale(result.new_angular_velocity,
                1.0f - entry.spin_damping);
        } else {
            result.new_angular_velocity = Vec3::scale(ball.body.angular_velocity,
                1.0f - entry.spin_damping);
        }

        result.effective_restitution = entry.restitution;
        result.deformation_amount = entry.deformation_factor * impact_speed * 0.1f;
        result.deformation_amount = std::min(result.deformation_amount, 1.0f);
        result.triggered = true;

        // Apply results to ball
        ball.body.linear_velocity = result.new_velocity;
        ball.body.angular_velocity = result.new_angular_velocity;
        ball.deformation = result.deformation_amount;
        ball.deformation_axis = surface_normal;
        ball.on_ground = true;
        ball.in_air = false;
        ball.current_surface = surface;
        ball.ground_contact_time = 0.0f;
        ball.update_spin_info();

        return result;
    }

    // --- Resolve ball hitting another rigid body ---
    BallBounceResult resolve_body(BallState& ball, const RigidBody& other,
                                   const Vec3& contact_normal,
                                   const Vec3& contact_point,
                                   float other_restitution = 0.8f) const
    {
        BallBounceResult result;
        result.triggered = false;
        result.deformation_amount = 0.0f;
        result.spin_transfer = 0.0f;

        Vec3 rel_vel = Vec3::sub(ball.body.linear_velocity, other.linear_velocity);
        float vel_along_normal = Vec3::dot(rel_vel, contact_normal);

        if (vel_along_normal >= 0.0f) return result;

        float impact_speed = std::abs(vel_along_normal);

        // Combined restitution (geometric mean)
        float e = std::sqrt(ball.config.base_restitution * other_restitution);

        // Velocity-dependent restitution reduction
        e = std::max(0.1f, e - ball.config.restitution_velocity_scale * impact_speed);

        // Impulse calculation
        float ball_inv_mass = ball.body.inverse_mass;
        float other_inv_mass = other.inverse_mass;
        float total_inv_mass = ball_inv_mass + other_inv_mass + 1e-10f;

        float j = -(1.0f + e) * vel_along_normal / total_inv_mass;

        // Decompose into normal and tangential
        Vec3 vel_normal = Vec3::scale(contact_normal, vel_along_normal);
        Vec3 vel_tangent = Vec3::sub(rel_vel, vel_normal);

        // Apply impulse to ball
        result.new_velocity = Vec3::add(ball.body.linear_velocity,
            Vec3::scale(contact_normal, j * ball_inv_mass));

        // Friction on tangential component
        float tangent_speed = Vec3::length(vel_tangent);
        if (tangent_speed > APC_EPSILON) {
            float friction_impulse = ball.config.friction_coefficient * std::abs(j);
            float tangent_impulse = std::min(friction_impulse, tangent_speed * ball_inv_mass);
            Vec3 tangent_dir = Vec3::scale(vel_tangent, 1.0f / tangent_speed);
            result.new_velocity = Vec3::sub(result.new_velocity,
                Vec3::scale(tangent_dir, tangent_impulse));

            result.spin_transfer = tangent_impulse /
                (ball.config.get_effective_radius() + APC_EPSILON);
        }

        // Angular velocity change
        Vec3 r = Vec3::sub(contact_point, ball.body.position);
        Vec3 angular_impulse = Vec3::cross(r, Vec3::scale(contact_normal, j));
        result.new_angular_velocity = ball.body.angular_velocity;

        if (ball.body.local_inverse_inertia.m[0] > 0.0f) {
            result.new_angular_velocity = Vec3::add(result.new_angular_velocity,
                Vec3::scale(angular_impulse, ball.config.get_effective_radius() * 0.1f));
        }

        result.effective_restitution = e;
        result.deformation_amount = std::min(0.8f, impact_speed * 0.05f);
        result.triggered = true;

        // Apply to ball
        ball.body.linear_velocity = result.new_velocity;
        ball.body.angular_velocity = result.new_angular_velocity;
        ball.deformation = result.deformation_amount;
        ball.deformation_axis = contact_normal;
        ball.update_spin_info();

        return result;
    }
};

// =============================================================================
// BallFactory — Preset ball configurations for each sport
// =============================================================================
struct BallFactory {
    // --- Soccer / Association Football ---
    static BallConfig make_soccer() {
        BallConfig c;
        c.shape = BallShape::SPHERE;
        c.name = "soccer";
        c.radius = 0.11f;
        c.mass = 0.43f;
        c.moment_of_inertia = 0.004f;
        c.drag_coefficient = 0.25f;      // Low Cd due to panel design
        c.cross_section_area = 0.038f;
        c.magnus_coefficient = 0.33f;
        c.base_restitution = 0.7f;
        c.restitution_velocity_scale = 0.005f;
        c.friction_coefficient = 0.4f;
        c.rolling_friction = 0.015f;
        c.spin_friction = 0.08f;
        c.has_panel_seams = true;
        c.seam_drag_modifier = 1.05f;
        c.preferred_spin = SpinAxis::SIDESPIN;
        return c;
    }

    // --- Basketball ---
    static BallConfig make_basketball() {
        BallConfig c;
        c.shape = BallShape::SPHERE;
        c.name = "basketball";
        c.radius = 0.12f;
        c.mass = 0.62f;
        c.moment_of_inertia = 0.006f;
        c.drag_coefficient = 0.47f;
        c.cross_section_area = 0.045f;
        c.magnus_coefficient = 0.2f;     // Lower Magnus (heavier ball)
        c.base_restitution = 0.8f;       // High bounce on hardwood
        c.restitution_velocity_scale = 0.003f;
        c.friction_coefficient = 0.5f;   // Grippy rubber surface
        c.rolling_friction = 0.008f;
        c.spin_friction = 0.1f;
        c.has_panel_seams = true;
        c.seam_drag_modifier = 1.02f;
        c.preferred_spin = SpinAxis::BACKSPIN;
        return c;
    }

    // --- Rugby Union / League ---
    static BallConfig make_rugby() {
        BallConfig c;
        c.shape = BallShape::PROLATE;
        c.name = "rugby";
        c.semi_major = 0.16f;            // ~32cm long
        c.semi_minor = 0.06f;            // ~12cm diameter
        c.radius = 0.08f;                // Effective average
        c.mass = 0.44f;
        c.moment_of_inertia = 0.003f;
        c.drag_coefficient = 0.35f;
        c.cross_section_area = 0.025f;
        c.magnus_coefficient = 0.4f;     // Strong spiral
        c.base_restitution = 0.55f;      // Lower bounce on grass
        c.restitution_velocity_scale = 0.004f;
        c.friction_coefficient = 0.45f;
        c.rolling_friction = 0.02f;
        c.spin_friction = 0.12f;
        c.has_panel_seams = true;
        c.seam_drag_modifier = 1.08f;
        c.preferred_spin = SpinAxis::HELICAL;
        return c;
    }

    // --- American Football ---
    static BallConfig make_american_football() {
        BallConfig c;
        c.shape = BallShape::PROLATE;
        c.name = "american_football";
        c.semi_major = 0.14f;            // ~28cm long
        c.semi_minor = 0.055f;           // ~11cm diameter
        c.radius = 0.075f;
        c.mass = 0.42f;
        c.moment_of_inertia = 0.0025f;
        c.drag_coefficient = 0.3f;
        c.cross_section_area = 0.02f;
        c.magnus_coefficient = 0.45f;    // Very strong spiral pass
        c.base_restitution = 0.5f;       // Low bounce on turf
        c.restitution_velocity_scale = 0.003f;
        c.friction_coefficient = 0.4f;
        c.rolling_friction = 0.02f;
        c.spin_friction = 0.1f;
        c.has_panel_seams = true;         // Lace seams
        c.seam_drag_modifier = 1.12f;
        c.preferred_spin = SpinAxis::HELICAL;
        return c;
    }

    // --- Tennis ---
    static BallConfig make_tennis() {
        BallConfig c;
        c.shape = BallShape::SPHERE;
        c.name = "tennis";
        c.radius = 0.0335f;              // ~6.7cm diameter
        c.mass = 0.058f;
        c.moment_of_inertia = 0.00007f;
        c.drag_coefficient = 0.55f;      // Fuzzy surface = high drag
        c.cross_section_area = 0.0035f;
        c.magnus_coefficient = 0.25f;
        c.base_restitution = 0.75f;
        c.restitution_velocity_scale = 0.008f;
        c.friction_coefficient = 0.55f;  // Felt surface = high grip
        c.rolling_friction = 0.025f;
        c.spin_friction = 0.05f;         // Spin is preserved well
        c.has_panel_seams = true;         // Felt seam
        c.seam_drag_modifier = 1.15f;
        c.preferred_spin = SpinAxis::TOPSPIN;
        return c;
    }

    // --- Baseball ---
    static BallConfig make_baseball() {
        BallConfig c;
        c.shape = BallShape::SPHERE;
        c.name = "baseball";
        c.radius = 0.037f;               // ~7.4cm diameter
        c.mass = 0.145f;
        c.moment_of_inertia = 0.00013f;
        c.drag_coefficient = 0.3f;       // Smooth leather
        c.cross_section_area = 0.0043f;
        c.magnus_coefficient = 0.35f;    // Strong curveball potential
        c.base_restitution = 0.5f;       // Low bounce off dirt
        c.restitution_velocity_scale = 0.002f;
        c.friction_coefficient = 0.35f;
        c.rolling_friction = 0.015f;
        c.spin_friction = 0.03f;         // Spin preserved very well
        c.has_panel_seams = true;         // Prominent seams
        c.seam_drag_modifier = 1.2f;
        c.preferred_spin = SpinAxis::SIDESPIN;
        return c;
    }

    // --- Golf ---
    static BallConfig make_golf() {
        BallConfig c;
        c.shape = BallShape::SPHERE;
        c.name = "golf";
        c.radius = 0.0213f;              // ~4.27cm diameter
        c.mass = 0.046f;
        c.moment_of_inertia = 0.00004f;
        c.drag_coefficient = 0.25f;      // Dimples reduce drag
        c.cross_section_area = 0.0014f;
        c.magnus_coefficient = 0.28f;
        c.base_restitution = 0.78f;
        c.restitution_velocity_scale = 0.001f;
        c.friction_coefficient = 0.3f;
        c.rolling_friction = 0.005f;
        c.spin_friction = 0.02f;
        c.has_panel_seams = true;         // Dimple pattern
        c.seam_drag_modifier = 0.9f;     // Dimples REDUCE drag
        c.preferred_spin = SpinAxis::BACKSPIN;
        return c;
    }

    // --- Volleyball ---
    static BallConfig make_volleyball() {
        BallConfig c;
        c.shape = BallShape::SPHERE;
        c.name = "volleyball";
        c.radius = 0.107f;               // ~21.4cm diameter
        c.mass = 0.27f;
        c.moment_of_inertia = 0.0025f;
        c.drag_coefficient = 0.4f;
        c.cross_section_area = 0.036f;
        c.magnus_coefficient = 0.22f;
        c.base_restitution = 0.65f;
        c.restitution_velocity_scale = 0.006f;
        c.friction_coefficient = 0.4f;
        c.rolling_friction = 0.01f;
        c.spin_friction = 0.08f;
        c.has_panel_seams = true;
        c.seam_drag_modifier = 1.1f;
        c.preferred_spin = SpinAxis::TOPSPIN;
        return c;
    }

    // --- Australian Rules Football ---
    static BallConfig make_aussie_rules() {
        BallConfig c;
        c.shape = BallShape::PROLATE;
        c.name = "aussie_rules";
        c.semi_major = 0.135f;           // ~27cm long
        c.semi_minor = 0.055f;           // ~11cm diameter
        c.radius = 0.072f;
        c.mass = 0.45f;
        c.moment_of_inertia = 0.003f;
        c.drag_coefficient = 0.35f;
        c.cross_section_area = 0.022f;
        c.magnus_coefficient = 0.38f;
        c.base_restitution = 0.5f;
        c.restitution_velocity_scale = 0.004f;
        c.friction_coefficient = 0.4f;
        c.rolling_friction = 0.02f;
        c.spin_friction = 0.12f;
        c.has_panel_seams = true;
        c.seam_drag_modifier = 1.08f;
        c.preferred_spin = SpinAxis::HELICAL;
        return c;
    }

    // --- Cricket ---
    static BallConfig make_cricket() {
        BallConfig c;
        c.shape = BallShape::SPHERE;
        c.name = "cricket";
        c.radius = 0.036f;               // ~7.2cm diameter
        c.mass = 0.163f;
        c.moment_of_inertia = 0.00015f;
        c.drag_coefficient = 0.4f;
        c.cross_section_area = 0.0041f;
        c.magnus_coefficient = 0.3f;     // Swing bowling
        c.base_restitution = 0.45f;      // Low bounce on pitch
        c.restitution_velocity_scale = 0.003f;
        c.friction_coefficient = 0.3f;
        c.rolling_friction = 0.02f;
        c.spin_friction = 0.04f;
        c.has_panel_seams = true;         // Primary seam for swing
        c.seam_drag_modifier = 1.25f;    // Seam is very pronounced
        c.preferred_spin = SpinAxis::SIDESPIN;
        return c;
    }

    // --- Handball ---
    static BallConfig make_handball() {
        BallConfig c;
        c.shape = BallShape::SPHERE;
        c.name = "handball";
        c.radius = 0.095f;
        c.mass = 0.425f;
        c.moment_of_inertia = 0.003f;
        c.drag_coefficient = 0.47f;
        c.cross_section_area = 0.028f;
        c.magnus_coefficient = 0.2f;
        c.base_restitution = 0.7f;
        c.restitution_velocity_scale = 0.005f;
        c.friction_coefficient = 0.5f;   // Sticky grip
        c.rolling_friction = 0.01f;
        c.spin_friction = 0.08f;
        c.has_panel_seams = true;
        c.seam_drag_modifier = 1.05f;
        c.preferred_spin = SpinAxis::BACKSPIN;
        return c;
    }
};

// =============================================================================
// BallPhysicsWorld — Manages multiple balls with aero + bounce simulation
// =============================================================================
struct BallPhysicsWorld {
    static constexpr uint32_t MAX_BALLS = 32u;

    BallState balls[MAX_BALLS];
    uint32_t ball_count = 0;
    AerodynamicModel aero_model;
    SurfaceBounceTable surface_table;
    BallBounce bounce_resolver;
    float gravity_y = -9.81f;

    uint32_t add_ball(const BallConfig& config) {
        if (ball_count >= MAX_BALLS) return 0xFFFFFFFF;
        BallState& bs = balls[ball_count];
        bs.config = config;
        bs.ball_id = ball_count;
        bs.reset();
        return ball_count++;
    }

    BallState* get_ball(uint32_t id) {
        if (id >= ball_count) return nullptr;
        return &balls[id];
    }

    // --- Step all balls: apply gravity + aerodynamics ---
    void step(float dt) {
        for (uint32_t i = 0; i < ball_count; ++i) {
            BallState& bs = balls[i];

            // Gravity
            bs.body.linear_velocity.y += gravity_y * dt;

            // Aerodynamics
            aero_model.dt = dt;
            aero_model.apply(bs);

            // Ground check (y = 0 plane)
            float r = bs.config.get_effective_radius();
            if (bs.body.position.y <= r && bs.body.linear_velocity.y < 0.0f) {
                Vec3 ground_normal(0.0f, 1.0f, 0.0f);
                bounce_resolver.resolve_surface(bs, bs.current_surface,
                    ground_normal, surface_table);
                bs.body.position.y = r;
            }

            // Integration (semi-implicit Euler)
            bs.body.position = Vec3::add(bs.body.position,
                Vec3::scale(bs.body.linear_velocity, dt));

            // Deformation recovery (spring back to round)
            if (bs.deformation > 0.0f) {
                bs.deformation -= dt * 8.0f; // Recovery rate
                if (bs.deformation < 0.0f) bs.deformation = 0.0f;
            }

            // Update ground contact time
            if (bs.on_ground) {
                bs.ground_contact_time += dt;
                // Check if ball left ground
                if (bs.body.linear_velocity.y > 0.5f) {
                    bs.on_ground = false;
                    bs.in_air = true;
                }
            }

            // Update orientation from angular velocity
            float ang_speed = Vec3::length(bs.body.angular_velocity);
            if (ang_speed > APC_EPSILON) {
                Vec3 axis = Vec3::scale(bs.body.angular_velocity, 1.0f / ang_speed);
                float angle = ang_speed * dt;
                Quat delta = Quat::from_axis_angle(axis, angle);
                bs.body.orientation = Quat::multiply(delta, bs.body.orientation);
                bs.body.orientation = Quat::normalize(bs.body.orientation);
            }
        }
    }
};

} // namespace apc
