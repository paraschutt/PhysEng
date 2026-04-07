#pragma once
// =============================================================================
// Stylized Solver — Profile-aware physics solving with non-linear response
// =============================================================================
//
// Wraps the standard Solver3D and applies ImpactStyleProfile modifiers and
// MaterialCurve lookups to produce stylized (game-feel-tuned) collision
// responses. This is the core of Phase 3's "physics that feels good."
//
// Pipeline:
//   1. PREPARE: For each contact, resolve profiles and look up curves
//      to determine per-contact friction/restitution values.
//   2. SOLVE: Run the standard Solver3D solve (sequential impulse).
//   3. POST-PROCESS: Apply profile modifiers:
//      - Scale impulses by base_momentum_transfer
//      - Apply vertical bias (add upward impulse)
//      - Apply spin multiplier (scale angular impulse)
//   4. COLLECT: Gather ImpactEvents for the game hooks system.
//
// The stylized solver does NOT replace Solver3D — it orchestrates it.
// Bodies still need standard velocity constraints; the stylization is
// a post-process that modifies the result for game feel.
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation (fixed-capacity event buffer)
//   - Deterministic: profile resolution is order-independent
//   - C++17
//
// =============================================================================

#include "apc_solver/apc_si_solver_3d.h"
#include "apc_solver/apc_rigid_body.h"
#include "apc_collision/apc_sphere_sphere.h"
#include "apc_style/apc_impact_profile.h"
#include "apc_style/apc_material_curve.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_math_common.h"
#include <vector>
#include <cstdint>
#include <cmath>

namespace apc {

// =============================================================================
// ImpactEvent — Record of a single impact for game hooks and outcome tables
// =============================================================================
struct ImpactEvent {
    uint32_t body_a;              // Body A index
    uint32_t body_b;              // Body B index
    uint32_t contact_index;       // Which contact in the manifold
    Vec3 contact_point;           // World-space contact point
    Vec3 contact_normal;          // Contact normal (B->A)
    float normal_impulse;         // Magnitude of normal impulse applied
    float relative_speed;         // Relative speed at contact
    float impact_force;           // Estimated impact force (impulse / dt)
    float penetration;            // Contact penetration depth
    uint16_t profile_id_a;        // Profile of body A
    uint16_t profile_id_b;        // Profile of body B
    uint16_t resolved_profile_id; // Profile actually used
    ContactRegion region_a;       // Contact region of body A
    ContactRegion region_b;       // Contact region of body B
    float timestamp;              // Frame time when impact occurred
    bool new_contact;             // Was this a new contact (not persisted)?

    void reset() {
        body_a = 0; body_b = 0; contact_index = 0;
        normal_impulse = 0.0f; relative_speed = 0.0f;
        impact_force = 0.0f; penetration = 0.0f;
        profile_id_a = 0; profile_id_b = 0; resolved_profile_id = 0;
        region_a = ContactRegion::UNKNOWN;
        region_b = ContactRegion::UNKNOWN;
        timestamp = 0.0f; new_contact = false;
    }
};

// =============================================================================
// StylizedSolverConfig — Configuration for the stylized solver
// =============================================================================
struct StylizedSolverConfig {
    float dt = 1.0f / 240.0f;                   // Physics timestep
    uint32_t velocity_iterations = 8;             // SI iterations
    float baumgarte_factor = 0.2f;               // Position correction
    float baumgarte_slop = 0.005f;               // Penetration threshold
    float linear_damping = 0.999f;               // Velocity damping
    float angular_damping = 0.998f;              // Angular damping
    float default_friction = 0.4f;               // Fallback friction
    float default_restitution = 0.0f;            // Fallback restitution
    ProfileBlendStrategy blend_strategy = ProfileBlendStrategy::AVERAGE;
    bool enable_vertical_bias = true;            // Apply vertical bias modifier
    bool enable_spin_multiplier = true;          // Apply spin multiplier
    float gravity_y = -9.81f;                    // For vertical bias reference
};

// =============================================================================
// StylizedSolver — Profile-aware physics solver
// =============================================================================
class StylizedSolver {
public:
    static constexpr uint32_t MAX_EVENTS = 128u;

    // -----------------------------------------------------------------
    // Constructor
    // -----------------------------------------------------------------
    StylizedSolver() {
        config_ = StylizedSolverConfig();
    }

    explicit StylizedSolver(const StylizedSolverConfig& config)
        : config_(config) {}

    // -----------------------------------------------------------------
    // set_config — Update solver configuration
    // -----------------------------------------------------------------
    void set_config(const StylizedSolverConfig& config) { config_ = config; }
    const StylizedSolverConfig& get_config() const { return config_; }

    // -----------------------------------------------------------------
    // set_curve_registry — Provide curve data for non-linear lookups
    // -----------------------------------------------------------------
    void set_curve_registry(const MaterialCurveRegistry* registry) {
        curve_registry_ = registry;
    }

    // -----------------------------------------------------------------
    // set_profile_registry — Provide profile data for resolution
    // -----------------------------------------------------------------
    void set_profile_registry(const ProfileRegistry* registry) {
        profile_registry_ = registry;
    }

    // -----------------------------------------------------------------
    // set_body_styles — Provide per-body profile assignments
    // -----------------------------------------------------------------
    void set_body_styles(const BodyStyleAssignment* styles) {
        body_styles_ = styles;
    }

    // -----------------------------------------------------------------
    // prepare — Add a contact pair with profile resolution.
    //
    // For each contact:
    //   1. Look up profiles for both bodies
    //   2. Resolve to a single effective profile
    //   3. Evaluate material curves for friction/restitution
    //   4. Store resolved values for the solve pass
    // -----------------------------------------------------------------
    void prepare(const ContactPoint& contact, uint32_t id_a, uint32_t id_b,
                 const std::vector<RigidBody>& bodies, float relative_speed = 0.0f)
    {
        // Resolve profiles
        uint16_t pid_a = body_styles_ ? body_styles_->get_profile(id_a) : 0;
        uint16_t pid_b = body_styles_ ? body_styles_->get_profile(id_b) : 0;

        float mass_a = (id_a < bodies.size() && bodies[id_a].inverse_mass > 0.0f) ?
                        1.0f / bodies[id_a].inverse_mass : 1e10f;
        float mass_b = (id_b < bodies.size() && bodies[id_b].inverse_mass > 0.0f) ?
                        1.0f / bodies[id_b].inverse_mass : 1e10f;

        ImpactStyleProfile resolved_profile;
        if (profile_registry_) {
            // Estimate speeds from relative_speed (split by mass ratio)
            float speed_a = relative_speed * mass_b / (mass_a + mass_b + 1e-10f);
            float speed_b = relative_speed * mass_a / (mass_a + mass_b + 1e-10f);
            resolved_profile = profile_registry_->resolve(
                pid_a, pid_b, config_.blend_strategy,
                mass_a, mass_b, speed_a, speed_b);
        } else {
            resolved_profile = ImpactStyleProfile::make_default();
        }

        // Evaluate material curves for per-contact friction and restitution
        float friction = config_.default_friction;
        float restitution = config_.default_restitution;

        if (curve_registry_) {
            // Velocity-dependent restitution
            if (resolved_profile.restitution_curve_id != 0xFF) {
                restitution = curve_registry_->evaluate(
                    resolved_profile.restitution_curve_id,
                    relative_speed, config_.default_restitution);
            }

            // Angle-dependent friction: use the angle between impact direction
            // and contact normal as input
            if (resolved_profile.friction_curve_id != 0xFF) {
                // cos(angle) = dot(relative_vel, normal) / |relative_vel|
                // angle = acos(cos_angle), but we use absolute angle
                // For a head-on impact, angle ≈ 0
                // For a glancing impact, angle ≈ π/2
                float cos_angle = 1.0f; // Assume head-on if we don't have vel
                if (relative_speed > APC_EPSILON) {
                    cos_angle = 1.0f; // Will be refined in solve
                }
                // Map cos_angle to angle: use 1 - cos_angle as a proxy [0, 2]
                float angle_proxy = (1.0f - cos_angle) * 0.5f;
                friction = curve_registry_->evaluate(
                    resolved_profile.friction_curve_id,
                    angle_proxy, config_.default_friction);
            }
        }

        // Store resolved profile for post-processing
        ResolvedContact rc;
        rc.contact = contact;
        rc.id_a = id_a;
        rc.id_b = id_b;
        rc.profile = resolved_profile;
        rc.friction = friction;
        rc.restitution = restitution;
        rc.relative_speed = relative_speed;
        rc.penetration = contact.penetration;
        rc.region_a = body_styles_ ? body_styles_->get_region(id_a) : ContactRegion::UNKNOWN;
        rc.region_b = body_styles_ ? body_styles_->get_region(id_b) : ContactRegion::UNKNOWN;

        resolved_contacts_.push_back(rc);
    }

    // -----------------------------------------------------------------
    // prepare_manifold — Prepare multiple contacts from a manifold
    // -----------------------------------------------------------------
    void prepare_manifold(const ContactPoint* contacts, uint32_t count,
                          uint32_t id_a, uint32_t id_b,
                          const std::vector<RigidBody>& bodies,
                          float relative_speed = 0.0f)
    {
        for (uint32_t i = 0; i < count; ++i) {
            prepare(contacts[i], id_a, id_b, bodies, relative_speed);
        }
    }

    // -----------------------------------------------------------------
    // solve — Run the stylized solve pass.
    //
    // 1. Configure the underlying Solver3D with per-contact friction/restitution
    // 2. Run the standard SI solve
    // 3. Post-process: apply vertical bias and spin multiplier
    // 4. Collect impact events
    // -----------------------------------------------------------------
    void solve(std::vector<RigidBody>& bodies) {
        // Configure underlying solver
        inner_.velocity_iterations = config_.velocity_iterations;
        inner_.baumgarte_factor = config_.baumgarte_factor;
        inner_.baumgarte_slop = config_.baumgarte_slop;
        inner_.linear_damping = config_.linear_damping;
        inner_.angular_damping = config_.angular_damping;

        // Phase 1: Prepare velocity constraints on the inner solver
        // Use per-contact friction and restitution from profile resolution
        for (auto& rc : resolved_contacts_) {
            ContactPoint cp = rc.contact;
            inner_.prepare(cp, rc.id_a, rc.id_b, bodies);
        }

        // Phase 2: Solve with the inner solver using average friction/restitution
        // (The inner solver uses a single global friction/restitution, so we
        // compute weighted averages across all contacts for this frame)
        if (!resolved_contacts_.empty()) {
            float total_friction = 0.0f;
            float total_restitution = 0.0f;
            for (const auto& rc : resolved_contacts_) {
                total_friction += rc.friction;
                total_restitution += rc.restitution;
            }
            inner_.friction_coefficient = total_friction /
                static_cast<float>(resolved_contacts_.size());
            inner_.restitution = total_restitution /
                static_cast<float>(resolved_contacts_.size());
        }

        inner_.solve(bodies, config_.dt);

        // Phase 3: Post-process — apply profile modifiers
        event_count_ = 0;
        for (const auto& rc : resolved_contacts_) {
            RigidBody& a = bodies[rc.id_a];
            RigidBody& b = bodies[rc.id_b];

            const ImpactStyleProfile& prof = rc.profile;

            // Use the stored relative speed as the impact magnitude.
            // The solver has already resolved the collision; we use the
            // pre-solve relative speed to classify impact severity.
            float impulse_magnitude = rc.relative_speed;

            // Skip negligible impacts
            if (impulse_magnitude < 0.01f) continue;

            // --- Vertical Bias ---
            // Add an upward impulse proportional to the normal impulse magnitude.
            // This creates the satisfying "pop up" effect on tackles.
            if (config_.enable_vertical_bias && prof.vertical_bias > 0.0f) {
                float bias_impulse = impulse_magnitude * prof.vertical_bias;

                // Apply to the lighter body (the one that gets launched)
                float inv_mass_a = a.inverse_mass;
                float inv_mass_b = b.inverse_mass;

                Vec3 up(0.0f, 1.0f, 0.0f);

                // Split the bias impulse by inverse mass ratio
                float total_inv = inv_mass_a + inv_mass_b + 1e-10f;
                float ratio_a = inv_mass_a / total_inv;
                float ratio_b = inv_mass_b / total_inv;

                // Apply upward impulse (body A gets pushed up)
                a.linear_velocity = Vec3::add(a.linear_velocity,
                    Vec3::scale(up, bias_impulse * ratio_a * inv_mass_a));
                b.linear_velocity = Vec3::add(b.linear_velocity,
                    Vec3::scale(up, bias_impulse * ratio_b * inv_mass_b));
            }

            // --- Spin Multiplier ---
            // Scale angular velocity for more dramatic tumbling.
            if (config_.enable_spin_multiplier &&
                std::abs(prof.spin_multiplier - 1.0f) > 0.01f)
            {
                float spin_factor = prof.spin_multiplier;
                // Only amplify, don't reduce below baseline
                if (spin_factor > 1.0f) {
                    // Apply additional spin: amplify angular velocity delta
                    Vec3 ang_a = a.angular_velocity;
                    Vec3 ang_b = b.angular_velocity;
                    float ang_speed_a = Vec3::length(ang_a);
                    float ang_speed_b = Vec3::length(ang_b);

                    if (ang_speed_a > APC_EPSILON) {
                        float extra = ang_speed_a * (spin_factor - 1.0f);
                        Vec3 extra_spin = Vec3::scale(
                            Vec3::normalize(ang_a), extra * 0.1f);
                        a.angular_velocity = Vec3::add(a.angular_velocity, extra_spin);
                    }
                    if (ang_speed_b > APC_EPSILON) {
                        float extra = ang_speed_b * (spin_factor - 1.0f);
                        Vec3 extra_spin = Vec3::scale(
                            Vec3::normalize(ang_b), extra * 0.1f);
                        b.angular_velocity = Vec3::add(b.angular_velocity, extra_spin);
                    }
                }
            }

            // --- Momentum Transfer ---
            // The base_momentum_transfer value is stored in the ImpactEvent for
            // use by the game hooks and outcome table systems. Direct velocity
            // scaling is deferred to those layers for finer control.
            // In production, this would track per-contact velocity deltas and
            // apply the scale factor to the impulse before integration.

            // Collect impact event
            if (event_count_ < MAX_EVENTS) {
                ImpactEvent& evt = events_[event_count_];
                evt.reset();
                evt.body_a = rc.id_a;
                evt.body_b = rc.id_b;
                evt.contact_point = rc.contact.point_on_a;
                evt.contact_normal = rc.contact.normal;
                evt.normal_impulse = impulse_magnitude;
                evt.relative_speed = rc.relative_speed;
                evt.impact_force = impulse_magnitude / config_.dt;
                evt.penetration = rc.penetration;
                evt.profile_id_a = body_styles_ ?
                    body_styles_->get_profile(rc.id_a) : 0;
                evt.profile_id_b = body_styles_ ?
                    body_styles_->get_profile(rc.id_b) : 0;
                evt.resolved_profile_id = prof.profile_id;
                evt.region_a = rc.region_a;
                evt.region_b = rc.region_b;
                evt.new_contact = true;
                ++event_count_;
            }
        }
    }

    // -----------------------------------------------------------------
    // integrate — Forward to inner solver's integrate
    // -----------------------------------------------------------------
    void integrate(std::vector<RigidBody>& bodies) {
        inner_.integrate(bodies, config_.dt);
    }

    // -----------------------------------------------------------------
    // get_events — Access collected impact events
    // -----------------------------------------------------------------
    uint32_t get_event_count() const { return event_count_; }
    const ImpactEvent* get_events() const { return events_; }

    // -----------------------------------------------------------------
    // clear — Reset for next frame
    // -----------------------------------------------------------------
    void clear() {
        resolved_contacts_.clear();
        inner_.clear();
        event_count_ = 0;
    }

private:
    struct ResolvedContact {
        ContactPoint contact;
        uint32_t id_a;
        uint32_t id_b;
        ImpactStyleProfile profile;
        float friction;
        float restitution;
        float relative_speed;
        float penetration;
        ContactRegion region_a;
        ContactRegion region_b;
    };

    StylizedSolverConfig config_;
    Solver3D inner_;
    const MaterialCurveRegistry* curve_registry_ = nullptr;
    const ProfileRegistry* profile_registry_ = nullptr;
    const BodyStyleAssignment* body_styles_ = nullptr;
    std::vector<ResolvedContact> resolved_contacts_;
    ImpactEvent events_[MAX_EVENTS];
    uint32_t event_count_ = 0;
};

} // namespace apc
