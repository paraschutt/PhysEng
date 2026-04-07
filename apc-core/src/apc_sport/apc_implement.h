#pragma once
// =============================================================================
// Implement Physics — Bat, racket, club, stick interaction with balls
// =============================================================================
//
// Models the physics of hitting a ball with an implement (bat, racket, club,
// hockey stick, cricket bat, etc.):
//
//   - ImplementType: classification of equipment
//   - ImplementProfile: physical properties (mass, length, stiffness, sweet spot)
//   - HitResult: outcome of an implement-ball collision
//   - SweetSpotModel: how off-center hits affect the result
//   - SpinTransfer: how implement motion creates ball spin
//   - DeflectionModel: angle at which ball leaves the implement face
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation
//   - Deterministic: same inputs → same outputs
//   - C++17
//
// =============================================================================

#include "apc_sport/apc_ball_physics.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include "apc_math/apc_math_common.h"
#include <cstdint>
#include <cmath>

namespace apc {

// =============================================================================
// ImplementType — Classification of striking implements
// =============================================================================
enum class ImplementType : uint8_t {
    BASEBALL_BAT    = 0,
    CRICKET_BAT     = 1,
    TENNIS_RACKET   = 2,
    BADMINTON_RACKET = 3,
    TABLE_TENNIS    = 4,
    SQUASH_RACKET   = 5,
    GOLF_DRIVER     = 6,
    GOLF_IRON       = 7,
    GOLF_WEDGE      = 8,
    GOLF_PUTTER     = 9,
    HOCKEY_STICK    = 10,
    FIELD_HOCKEY    = 11,
    LACROSSE        = 12,
    HURLING_HURLEY  = 13
};

// =============================================================================
// ImplementShape — Geometry of the striking surface
// =============================================================================
enum class ImplementShape : uint8_t {
    CYLINDER    = 0,  // Baseball bat, hockey stick
    FLAT_FACE   = 1,  // Cricket bat, golf iron, hockey stick blade
    STRING_BED  = 2,  // Tennis, badminton, squash rackets
    NET_POCKET  = 3   // Lacrosse stick
};

// =============================================================================
// SweetSpotZone — Hit quality classification
// =============================================================================
enum class SweetSpotZone : uint8_t {
    PERFECT  = 0,  // Dead center of sweet spot
    GOOD     = 1,  // Near sweet spot
    AVERAGE  = 2,  // Middle of implement face
    POOR     = 3,  // Near edge
    MIS_HIT  = 4,  // Off the face entirely (handle, frame, etc.)
    SHANK    = 5   // Hit the hosel/shaft (golf), frame (racket)
};

// =============================================================================
// ImplementProfile — Physical properties of a striking implement
// =============================================================================
struct ImplementProfile {
    ImplementType type = ImplementType::BASEBALL_BAT;
    ImplementShape shape = ImplementShape::CYLINDER;
    const char* name = "baseball_bat";

    // --- Dimensions ---
    float total_length = 0.86f;        // Full length (m)
    float grip_length = 0.18f;         // Handle/grip length (m)
    float hitting_length = 0.50f;      // Effective hitting zone length (m)
    float width = 0.067f;             // Width/diameter of hitting surface (m)
    float thickness = 0.067f;         // Thickness of hitting surface (m)

    // --- Mass distribution ---
    float total_mass = 0.88f;          // Total mass (kg)
    float head_mass = 0.55f;           // Mass in the hitting end (kg)
    float moment_of_inertia = 0.15f;   // Swing MOI (kg·m²)

    // --- Sweet spot ---
    float sweet_spot_center = 0.65f;   // Distance from handle end (m)
    float sweet_spot_radius = 0.05f;   // Radius of perfect hit zone (m)
    float sweet_spot_power_bonus = 1.15f; // Power multiplier at sweet spot

    // --- Face properties ---
    float face_restitution = 0.5f;     // COR of the face material
    float face_friction = 0.4f;        // Friction coefficient
    float face_angle = 0.0f;          // Loft angle of the face (rad)

    // --- String bed (rackets) ---
    float string_tension = 25.0f;      // kg (rackets only)
    float string_restitution = 0.75f;  // COR of strings
    float string_power_retention = 0.9f; // How well strings store energy

    // --- Flex ---
    float flex_stiffness = 0.5f;       // 0 = very flexible, 1 = very stiff
    float flex_power_retention = 0.8f; // Energy returned from flex

    // --- Factory methods for standard implements ---

    static ImplementProfile make_baseball_bat() {
        ImplementProfile p;
        p.type = ImplementType::BASEBALL_BAT;
        p.shape = ImplementShape::CYLINDER;
        p.name = "baseball_bat";
        p.total_length = 0.86f;
        p.grip_length = 0.18f;
        p.hitting_length = 0.45f;
        p.width = 0.067f;
        p.thickness = 0.067f;
        p.total_mass = 0.88f;
        p.head_mass = 0.55f;
        p.moment_of_inertia = 0.15f;
        p.sweet_spot_center = 0.65f;
        p.sweet_spot_radius = 0.04f;
        p.sweet_spot_power_bonus = 1.15f;
        p.face_restitution = 0.50f;
        p.face_friction = 0.3f;
        p.face_angle = 0.0f;
        p.flex_stiffness = 0.6f;
        p.flex_power_retention = 0.85f;
        return p;
    }

    static ImplementProfile make_cricket_bat() {
        ImplementProfile p;
        p.type = ImplementType::CRICKET_BAT;
        p.shape = ImplementShape::FLAT_FACE;
        p.name = "cricket_bat";
        p.total_length = 0.96f;
        p.grip_length = 0.22f;
        p.hitting_length = 0.50f;
        p.width = 0.108f;              // Wide flat face
        p.thickness = 0.045f;
        p.total_mass = 1.2f;
        p.head_mass = 0.75f;
        p.moment_of_inertia = 0.18f;
        p.sweet_spot_center = 0.70f;   // "Middle" of the bat
        p.sweet_spot_radius = 0.05f;
        p.sweet_spot_power_bonus = 1.20f;
        p.face_restitution = 0.45f;    // Lower than baseball (ball is heavier)
        p.face_friction = 0.5f;        // High friction for spin
        p.face_angle = 0.05f;          // Slight loft
        p.flex_stiffness = 0.4f;       // Willow flexes
        p.flex_power_retention = 0.9f;
        return p;
    }

    static ImplementProfile make_tennis_racket() {
        ImplementProfile p;
        p.type = ImplementType::TENNIS_RACKET;
        p.shape = ImplementShape::STRING_BED;
        p.name = "tennis_racket";
        p.total_length = 0.69f;
        p.grip_length = 0.20f;
        p.hitting_length = 0.35f;
        p.width = 0.28f;               // Head width
        p.thickness = 0.03f;
        p.total_mass = 0.32f;
        p.head_mass = 0.15f;
        p.moment_of_inertia = 0.04f;
        p.sweet_spot_center = 0.48f;
        p.sweet_spot_radius = 0.04f;
        p.sweet_spot_power_bonus = 1.10f;
        p.face_restitution = 0.75f;    // Strings return more energy
        p.face_friction = 0.6f;        // Strings grip ball well
        p.face_angle = 0.0f;
        p.string_tension = 25.0f;
        p.string_restitution = 0.75f;
        p.string_power_retention = 0.92f;
        p.flex_stiffness = 0.3f;
        p.flex_power_retention = 0.95f;
        return p;
    }

    static ImplementProfile make_golf_driver() {
        ImplementProfile p;
        p.type = ImplementType::GOLF_DRIVER;
        p.shape = ImplementShape::FLAT_FACE;
        p.name = "golf_driver";
        p.total_length = 1.17f;
        p.grip_length = 0.30f;
        p.hitting_length = 0.20f;
        p.width = 0.11f;
        p.thickness = 0.044f;
        p.total_mass = 0.45f;
        p.head_mass = 0.30f;
        p.moment_of_inertia = 0.25f;
        p.sweet_spot_center = 0.95f;
        p.sweet_spot_radius = 0.02f;
        p.sweet_spot_power_bonus = 1.25f;
        p.face_restitution = 0.83f;    // Very high COR (trampoline effect)
        p.face_friction = 0.25f;
        p.face_angle = 0.35f;          // ~20 degrees loft
        p.flex_stiffness = 0.7f;
        p.flex_power_retention = 0.9f;
        return p;
    }

    static ImplementProfile make_golf_iron() {
        ImplementProfile p;
        p.type = ImplementType::GOLF_IRON;
        p.shape = ImplementShape::FLAT_FACE;
        p.name = "golf_iron";
        p.total_length = 0.97f;
        p.grip_length = 0.25f;
        p.hitting_length = 0.15f;
        p.width = 0.09f;
        p.thickness = 0.02f;
        p.total_mass = 0.43f;
        p.head_mass = 0.25f;
        p.moment_of_inertia = 0.15f;
        p.sweet_spot_center = 0.82f;
        p.sweet_spot_radius = 0.015f;
        p.sweet_spot_power_bonus = 1.15f;
        p.face_restitution = 0.78f;
        p.face_friction = 0.3f;
        p.face_angle = 0.52f;          // ~30 degrees loft (7-iron)
        p.flex_stiffness = 0.65f;
        p.flex_power_retention = 0.88f;
        return p;
    }

    static ImplementProfile make_hockey_stick() {
        ImplementProfile p;
        p.type = ImplementType::HOCKEY_STICK;
        p.shape = ImplementShape::FLAT_FACE;
        p.name = "hockey_stick";
        p.total_length = 1.52f;
        p.grip_length = 0.40f;
        p.hitting_length = 0.20f;
        p.width = 0.10f;
        p.thickness = 0.03f;
        p.total_mass = 0.45f;
        p.head_mass = 0.15f;
        p.moment_of_inertia = 0.2f;
        p.sweet_spot_center = 1.35f;   // Near blade
        p.sweet_spot_radius = 0.03f;
        p.sweet_spot_power_bonus = 1.10f;
        p.face_restitution = 0.5f;
        p.face_friction = 0.2f;        // Ice = low friction
        p.face_angle = 0.26f;          // ~15 degrees lie angle
        p.flex_stiffness = 0.5f;
        p.flex_power_retention = 0.85f;
        return p;
    }

    static ImplementProfile make_badminton_racket() {
        ImplementProfile p;
        p.type = ImplementType::BADMINTON_RACKET;
        p.shape = ImplementShape::STRING_BED;
        p.name = "badminton_racket";
        p.total_length = 0.68f;
        p.grip_length = 0.18f;
        p.hitting_length = 0.30f;
        p.width = 0.25f;
        p.thickness = 0.02f;
        p.total_mass = 0.09f;          // Very light
        p.head_mass = 0.04f;
        p.moment_of_inertia = 0.015f;
        p.sweet_spot_center = 0.46f;
        p.sweet_spot_radius = 0.035f;
        p.sweet_spot_power_bonus = 1.12f;
        p.face_restitution = 0.80f;
        p.face_friction = 0.7f;        // High grip for spin shots
        p.face_angle = 0.0f;
        p.string_tension = 12.0f;
        p.string_restitution = 0.80f;
        p.string_power_retention = 0.95f;
        p.flex_stiffness = 0.25f;      // Very flexible
        p.flex_power_retention = 0.93f;
        return p;
    }
};

// =============================================================================
// SweetSpotModel — Determines hit quality and power modification
// =============================================================================
struct SweetSpotModel {
    // --- Classify hit based on contact point relative to sweet spot ---
    SweetSpotZone classify(float contact_offset, float swing_speed,
                            const ImplementProfile& impl) const
    {
        float dist = std::abs(contact_offset - impl.sweet_spot_center);
        float sweet_radius = impl.sweet_spot_radius;

        // Speed helps: faster swing = slightly more forgiving
        float speed_forgiveness = std::min(0.3f, swing_speed * 0.005f);

        if (dist < sweet_radius * 0.5f) return SweetSpotZone::PERFECT;
        if (dist < sweet_radius) return SweetSpotZone::GOOD;
        if (dist < sweet_radius * 2.0f + speed_forgiveness * 0.1f)
            return SweetSpotZone::AVERAGE;
        if (dist < impl.hitting_length * 0.5f) return SweetSpotZone::POOR;
        if (contact_offset < impl.grip_length) return SweetSpotZone::SHANK;
        return SweetSpotZone::MIS_HIT;
    }

    // --- Get power multiplier based on hit zone ---
    float get_power_multiplier(SweetSpotZone zone) const {
        switch (zone) {
        case SweetSpotZone::PERFECT:  return 1.15f;
        case SweetSpotZone::GOOD:     return 1.05f;
        case SweetSpotZone::AVERAGE:  return 0.90f;
        case SweetSpotZone::POOR:     return 0.70f;
        case SweetSpotZone::MIS_HIT:  return 0.40f;
        case SweetSpotZone::SHANK:    return 0.20f;
        default: return 0.5f;
        }
    }

    // --- Get accuracy modifier based on hit zone ---
    float get_accuracy_modifier(SweetSpotZone zone) const {
        switch (zone) {
        case SweetSpotZone::PERFECT:  return 1.0f;
        case SweetSpotZone::GOOD:     return 0.92f;
        case SweetSpotZone::AVERAGE:  return 0.80f;
        case SweetSpotZone::POOR:     return 0.60f;
        case SweetSpotZone::MIS_HIT:  return 0.30f;
        case SweetSpotZone::SHANK:    return 0.15f;
        default: return 0.5f;
        }
    }

    // --- Get spin modifier based on hit zone ---
    float get_spin_modifier(SweetSpotZone zone) const {
        switch (zone) {
        case SweetSpotZone::PERFECT:  return 1.0f;
        case SweetSpotZone::GOOD:     return 1.1f;    // Slightly more spin on off-center
        case SweetSpotZone::AVERAGE:  return 1.3f;    // More spin (glancing hit)
        case SweetSpotZone::POOR:     return 1.5f;    // Lots of spin, no power
        case SweetSpotZone::MIS_HIT:  return 2.0f;    // Extreme spin
        case SweetSpotZone::SHANK:    return 0.5f;    // Unpredictable
        default: return 1.0f;
        }
    }
};

// =============================================================================
// HitResult — Complete result of an implement striking a ball
// =============================================================================
struct HitResult {
    bool hit = false;
    SweetSpotZone zone = SweetSpotZone::AVERAGE;

    Vec3 ball_velocity;             // Post-hit ball velocity
    Vec3 ball_angular_velocity;     // Post-hit ball spin
    float ball_speed = 0.0f;
    float launch_angle = 0.0f;      // Vertical launch angle
    float deflection_angle = 0.0f;  // Lateral deflection from intended direction

    float power_multiplier = 1.0f;
    float accuracy_multiplier = 1.0f;
    float spin_multiplier = 1.0f;
    float effective_restitution = 0.0f;

    Vec3 contact_point;             // Where on the implement the ball hit
    Vec3 ball_contact_point;        // Where on the ball the implement hit
    float impact_force = 0.0f;
};

// =============================================================================
// ImplementHitResolver — Resolves implement-ball collisions
// =============================================================================
struct ImplementHitResolver {
    SweetSpotModel sweet_spot;

    // --- Resolve a hit ---
    HitResult resolve(BallState& ball,
                       const ImplementProfile& impl,
                       const Vec3& implement_velocity,
                       const Vec3& implement_face_normal,
                       const Vec3& implement_position,
                       float contact_offset_along_implement,
                       float skill_rating = 0.8f)
    {
        HitResult result;
        result.hit = false;

        // Check proximity
        Vec3 to_ball = Vec3::sub(ball.body.position, implement_position);
        float distance = Vec3::length(to_ball);
        float hit_range = impl.width * 0.5f + ball.config.get_effective_radius();
        if (distance > hit_range * 1.5f) return result;

        // Classify sweet spot
        float swing_speed = Vec3::length(implement_velocity);
        result.zone = sweet_spot.classify(contact_offset_along_implement,
            swing_speed, impl);

        result.power_multiplier = sweet_spot.get_power_multiplier(result.zone);
        result.accuracy_multiplier = sweet_spot.get_accuracy_modifier(result.zone);
        result.spin_multiplier = sweet_spot.get_spin_modifier(result.zone);

        // Determine effective restitution
        if (impl.shape == ImplementShape::STRING_BED) {
            result.effective_restitution = impl.string_restitution *
                impl.string_power_retention;
        } else {
            result.effective_restitution = impl.face_restitution *
                impl.flex_power_retention;
        }

        // Adjust for flex: flexible implements store/release energy
        // Stiffer = less energy loss in flex, but less "whip"
        float flex_bonus = 1.0f + (1.0f - impl.flex_stiffness) * 0.15f;
        result.power_multiplier *= flex_bonus;

        // Relative velocity at contact point
        Vec3 rel_vel = Vec3::sub(implement_velocity, ball.body.linear_velocity);
        float approach_speed = Vec3::dot(rel_vel, implement_face_normal);

        if (approach_speed <= 0.0f) return result; // Ball moving away

        // Speed after hit: v_ball = (1 + e) * v_approach * power_mult * mass_ratio
        float ball_inv_mass = ball.body.inverse_mass;
        float impl_eff_mass = impl.head_mass;
        float mass_ratio = impl_eff_mass / (impl_eff_mass +
            (ball_inv_mass > 0.0f ? 1.0f / ball_inv_mass : 1.0f));

        float exit_speed = (1.0f + result.effective_restitution) *
            approach_speed * result.power_multiplier * mass_ratio;

        // Apply skill rating
        exit_speed *= (0.85f + 0.15f * skill_rating);

        // Launch direction: reflect relative velocity around face normal
        Vec3 face_normal = implement_face_normal;
        float fn_len = Vec3::length(face_normal);
        if (fn_len > APC_EPSILON) {
            face_normal = Vec3::scale(face_normal, 1.0f / fn_len);
        }

        // Add loft angle
        Vec3 lofted_normal(
            face_normal.x,
            face_normal.y + std::sin(impl.face_angle),
            face_normal.z
        );
        float lofted_len = Vec3::length(lofted_normal);
        if (lofted_len > APC_EPSILON) {
            lofted_normal = Vec3::scale(lofted_normal, 1.0f / lofted_len);
        }

        // Deflection for off-center hits
        Vec3 deflection(0.0f, 0.0f, 0.0f);
        if (result.zone != SweetSpotZone::PERFECT) {
            float deflection_amount = (1.0f - result.accuracy_multiplier) * 0.5f;
            // Deflect perpendicular to face normal and swing direction
            Vec3 swing_dir = implement_velocity;
            float sw_len = Vec3::length(swing_dir);
            if (sw_len > APC_EPSILON) {
                swing_dir = Vec3::scale(swing_dir, 1.0f / sw_len);
                deflection = Vec3::cross(face_normal, swing_dir);
                float d_len = Vec3::length(deflection);
                if (d_len > APC_EPSILON) {
                    deflection = Vec3::scale(deflection,
                        1.0f / d_len * deflection_amount);
                }
            }
        }

        // Final launch direction
        Vec3 launch_dir = Vec3::add(lofted_normal, deflection);
        float ld_len = Vec3::length(launch_dir);
        if (ld_len > APC_EPSILON) {
            launch_dir = Vec3::scale(launch_dir, 1.0f / ld_len);
        }

        // Apply to ball
        result.ball_velocity = Vec3::scale(launch_dir, exit_speed);
        result.ball_speed = exit_speed;
        result.launch_angle = std::asin(std::abs(launch_dir.y));
        result.deflection_angle = std::abs(Vec3::length(deflection));

        // Spin: from implement motion and face friction
        // Tangential velocity at contact creates spin
        Vec3 vel_normal_component = Vec3::scale(face_normal, approach_speed);
        Vec3 vel_tangential = Vec3::sub(implement_velocity, vel_normal_component);
        float tangential_speed = Vec3::length(vel_tangential);

        // Spin from tangential friction
        float spin_from_friction = tangential_speed * impl.face_friction *
            result.spin_multiplier / (ball.config.get_effective_radius() + APC_EPSILON);

        // Spin from loft angle (backspin/topspin)
        float spin_from_loft = exit_speed * std::sin(impl.face_angle) * 2.0f /
            (ball.config.get_effective_radius() + APC_EPSILON);

        // Combined spin axis
        Vec3 spin_dir(0.0f, 0.0f, 0.0f);
        if (tangential_speed > APC_EPSILON) {
            // Sidespin from tangential motion
            Vec3 tangential_dir = Vec3::scale(vel_tangential, 1.0f / tangential_speed);
            spin_dir = Vec3::cross(face_normal, tangential_dir);
        }

        // Add backspin component
        Vec3 backspin_axis = Vec3::cross(Vec3(0.0f, 1.0f, 0.0f), launch_dir);
        float bs_len = Vec3::length(backspin_axis);
        if (bs_len > APC_EPSILON) {
            backspin_axis = Vec3::scale(backspin_axis, 1.0f / bs_len);
            spin_dir = Vec3::add(spin_dir,
                Vec3::scale(backspin_axis, spin_from_loft));
        }

        float total_spin = spin_from_friction + spin_from_loft;
        float sp_len = Vec3::length(spin_dir);
        if (sp_len > APC_EPSILON) {
            result.ball_angular_velocity = Vec3::scale(
                Vec3::scale(spin_dir, 1.0f / sp_len), total_spin);
        }

        result.impact_force = exit_speed * (ball_inv_mass > 0.0f ?
            1.0f / ball_inv_mass : 1.0f) / (1.0f / 240.0f);
        result.contact_point = implement_position;
        result.ball_contact_point = ball.body.position;

        // Apply to ball state
        ball.body.linear_velocity = result.ball_velocity;
        ball.body.angular_velocity = result.ball_angular_velocity;
        ball.on_ground = false;
        ball.in_air = true;
        ball.deformation = std::min(0.6f, approach_speed * 0.02f);
        ball.deformation_axis = face_normal;
        ball.update_spin_info();

        result.hit = true;
        return result;
    }
};

// =============================================================================
// ImplementSwingModel — Models the swing arc of an implement
// =============================================================================
struct SwingPhase {
    float time = 0.0f;
    float angle = 0.0f;              // Current swing angle (rad)
    float angular_velocity = 0.0f;   // Current angular velocity (rad/s)
    float power = 0.0f;             // Current power (0-1)
};

struct ImplementSwingModel {
    // --- Swing phases ---
    float wind_up_duration = 0.3f;       // Backswing time
    float acceleration_duration = 0.15f; // Forward swing acceleration
    float follow_through_duration = 0.2f; // Follow through
    float total_swing_duration = 0.65f;

    float max_swing_speed = 40.0f;       // Max tip speed (m/s)
    float backswing_angle = 2.5f;        // Backswing angle (rad, ~143 deg)

    // --- Get swing state at a given time ---
    SwingPhase evaluate(float t, float skill_rating = 0.8f) const {
        SwingPhase phase;
        float t_norm = t / total_swing_duration;

        if (t_norm < 0.0f) {
            // Before swing starts
            phase.time = 0.0f;
            phase.angle = -backswing_angle;
            phase.angular_velocity = 0.0f;
            phase.power = 0.0f;
        } else if (t_norm < wind_up_duration / total_swing_duration) {
            // Wind-up phase
            float frac = t_norm * total_swing_duration / wind_up_duration;
            phase.time = t;
            phase.angle = -backswing_angle * frac;
            phase.angular_velocity = -backswing_angle / wind_up_duration;
            phase.power = 0.0f;
        } else if (t_norm < (wind_up_duration + acceleration_duration) /
                   total_swing_duration) {
            // Acceleration phase (where contact happens)
            float frac = (t - wind_up_duration) / acceleration_duration;
            // Sinusoidal acceleration profile
            float sin_frac = std::sin(frac * 1.5707963f); // sin(pi/2 * frac)

            float eff_max = max_swing_speed * (0.85f + 0.15f * skill_rating);
            phase.time = t;
            phase.angle = -backswing_angle * (1.0f - frac) +
                backswing_angle * 0.3f * frac; // Past center slightly
            phase.angular_velocity = eff_max * sin_frac;
            phase.power = sin_frac;
        } else {
            // Follow-through
            float frac = (t - wind_up_duration - acceleration_duration) /
                follow_through_duration;
            frac = std::min(1.0f, frac);
            float decay = 1.0f - frac;

            float eff_max = max_swing_speed * (0.85f + 0.15f * skill_rating);
            phase.time = t;
            phase.angle = backswing_angle * 0.3f + backswing_angle * 0.2f * frac;
            phase.angular_velocity = eff_max * decay * 0.5f;
            phase.power = decay * 0.3f;
        }

        return phase;
    }

    // --- Get tip velocity at a given swing time ---
    Vec3 get_tip_velocity(float t, const Vec3& pivot_pos,
                           const Vec3& swing_axis, float impl_length,
                           float skill_rating = 0.8f) const
    {
        SwingPhase phase = evaluate(t, skill_rating);
        // Tip velocity = angular_velocity × length × tangential_direction
        // Tangential direction = axis × (point - pivot) normalized
        Vec3 axis_norm = swing_axis;
        float ax_len = Vec3::length(axis_norm);
        if (ax_len > APC_EPSILON) {
            axis_norm = Vec3::scale(axis_norm, 1.0f / ax_len);
        } else {
            return Vec3(0.0f, 0.0f, 0.0f);
        }

        float tip_speed = phase.angular_velocity * impl_length;

        // Tangential direction perpendicular to swing axis
        // Use a reference "forward" direction to compute tangent
        Vec3 ref(0.0f, 0.0f, 1.0f);
        if (std::abs(Vec3::dot(axis_norm, ref)) > 0.99f) {
            ref = Vec3(1.0f, 0.0f, 0.0f);
        }
        Vec3 tangent = Vec3::cross(axis_norm, ref);
        float tg_len = Vec3::length(tangent);
        if (tg_len > APC_EPSILON) {
            tangent = Vec3::scale(tangent, 1.0f / tg_len);
        }

        return Vec3::scale(tangent, tip_speed);
    }
};

} // namespace apc
