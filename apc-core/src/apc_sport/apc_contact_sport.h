#pragma once
// =============================================================================
// Contact Sport — Tackle, block, shoulder charge, grapple mechanics
// =============================================================================
//
// Extends Phase 3's impact system with sport-specific contact mechanics:
//
//   - ContactType: classification of athlete-athlete contact
//   - TackleModel: different tackle types (form, shoulder, dive, strip, sack)
//   - BlockModel: blocking and shielding mechanics
//   - ShoulderCharge: standing shoulder-to-shoulder contact
//   - GrappleModel: wrestling/judo-style holds and throws
//   - ContactResult: outcome of athlete-athlete collision
//   - ContactResolver: determines outcome based on impact + sport rules
//
// Integrates with Phase 3's ImpactStyleProfile, OutcomeTable, and GameHooks.
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation
//   - Deterministic: same inputs → same outputs
//   - C++17
//
// =============================================================================

#include "apc_style/apc_impact_profile.h"
#include "apc_style/apc_outcome_table.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include "apc_math/apc_math_common.h"
#include <cstdint>
#include <cmath>

namespace apc {

// =============================================================================
// ContactType — Classification of athlete-athlete contact
// =============================================================================
enum class ContactType : uint8_t {
    TACKLE_FORM     = 0,   // Form tackle (wrap-up, rugby/football)
    TACKLE_SHOULDER = 1,   // Shoulder tackle (impact tackle)
    TACKLE_DIVE     = 2,   // Dive/slide tackle (soccer)
    TACKLE_STRIP    = 3,   // Tackle with strip attempt (football)
    SACK            = 4,   // Quarterback sack (football)
    BLOCK           = 5,   // Block (offensive lineman, basketball screen)
    SHOULDER_CHARGE = 6,   // Standing shoulder charge (rugby, soccer)
    SCREEN          = 7,   // Legal screen/pick (basketball)
    ELBOW           = 8,   // Elbow check (basketball foul)
    HIP_CHECK       = 9,   // Hip check (hockey, rugby)
    CLOTHESLINE     = 10,  // Clothesline tackle (football — foul)
    GRAPPLE_HOLD    = 11,  // Grappling hold (wrestling, rugby maul)
    GRAPPLE_THROW   = 12,  // Grappling throw (judo, wrestling)
    PUSH            = 13,  // Open-hand push (foul in most sports)
    COLLISION       = 14   // Accidental running-into contact
};

// =============================================================================
// TackleType — Specific tackle classification
// =============================================================================
enum class TackleType : uint8_t {
    NONE        = 0,
    LOW         = 1,   // Low tackle (legs)
    HIGH        = 2,   // High tackle (above waist)
    MID         = 3,   // Mid-section tackle
    DIVE_SLIDE  = 4,   // Sliding tackle (soccer)
    DIVE_HEADER = 5,   // Diving header (soccer)
    WRAP        = 6,   // Wrap-up tackle (rugby/football)
    ARM_TACKLE  = 7,   // One-arm tackle
    DOUBLE      = 8,   // Two-man tackle
    BEHIND      = 9,   // Tackle from behind
    SIDE        = 10,  // Side tackle
    FRONT       = 11   // Head-on tackle
};

// =============================================================================
// BlockType — Blocking classification
// =============================================================================
enum class BlockType : uint8_t {
    NONE        = 0,
    PASS_BLOCK  = 1,   // Pass protection block
    RUN_BLOCK   = 2,   // Run blocking
    SCREEN      = 3,   // Basketball screen
    BOX_OUT     = 4,   // Basketball rebounding box-out
    PICK        = 5,   // Setting a pick
    BODY_CHECK  = 6,   // Hockey body check
    LEGAL_SHIELD = 7,  // Rugby shielding
    ILLEGAL     = 8    // Holding / illegal block
};

// =============================================================================
// ContactDirection — Where contact happens relative to ball carrier
// =============================================================================
enum class ContactDirection : uint8_t {
    FRONT       = 0,   // Head-on
    BEHIND      = 1,   // From behind
    LEFT_SIDE   = 2,   // From the left
    RIGHT_SIDE  = 3,   // From the right
    ABOVE       = 4,   // Jumping over (rare)
    BELOW       = 5    // Diving under
};

// =============================================================================
// TackleProfile — Per-tackle-type parameters
// =============================================================================
struct TackleProfile {
    TackleType type = TackleType::NONE;
    const char* name = "none";

    float base_success_rate = 0.7f;      // Probability of bringing down
    float power_multiplier = 1.0f;       // Impact power scaling
    float momentum_transfer = 1.0f;      // How much momentum transfers
    float fumble_chance = 0.05f;         // Ball fumble probability
    float injury_risk = 0.02f;           // Injury risk factor (gameplay)
    float recovery_time = 0.5f;          // Tackler recovery time (seconds)
    float ball_carrier_recovery = 0.3f;  // Carrier recovery time
    float stamina_cost = 0.15f;          // Stamina cost to tackler

    float min_approach_speed = 2.0f;     // Min speed to attempt
    float max_approach_angle = 1.2f;     // Max angle from ideal approach
    float reach = 1.0f;                 // Max tackle reach (meters)
    float execution_time = 0.3f;         // Time to complete tackle animation

    // --- Outcome parameters ---
    float ragdoll_trigger_threshold = 0.7f; // Success rate above this = ragdoll
    float fall_direction_weight = 1.0f;  // How much the tackle direction matters

    // --- Factory methods ---
    static TackleProfile make_form_tackle() {
        TackleProfile p;
        p.type = TackleType::WRAP;
        p.name = "form_tackle";
        p.base_success_rate = 0.85f;
        p.power_multiplier = 1.0f;
        p.momentum_transfer = 0.9f;
        p.fumble_chance = 0.08f;
        p.injury_risk = 0.03f;
        p.recovery_time = 0.6f;
        p.reach = 1.2f;
        p.execution_time = 0.35f;
        p.ragdoll_trigger_threshold = 0.6f;
        return p;
    }

    static TackleProfile make_shoulder_tackle() {
        TackleProfile p;
        p.type = TackleType::MID;
        p.name = "shoulder_tackle";
        p.base_success_rate = 0.75f;
        p.power_multiplier = 1.4f;
        p.momentum_transfer = 1.2f;
        p.fumble_chance = 0.15f;
        p.injury_risk = 0.06f;
        p.recovery_time = 0.4f;
        p.reach = 1.3f;
        p.execution_time = 0.2f;
        p.ragdoll_trigger_threshold = 0.5f;
        return p;
    }

    static TackleProfile make_slide_tackle() {
        TackleProfile p;
        p.type = TackleType::DIVE_SLIDE;
        p.name = "slide_tackle";
        p.base_success_rate = 0.65f;
        p.power_multiplier = 0.8f;
        p.momentum_transfer = 0.6f;
        p.fumble_chance = 0.12f;
        p.injury_risk = 0.05f;
        p.recovery_time = 0.8f;       // Long recovery (on the ground)
        p.reach = 1.8f;              // Extended reach from slide
        p.execution_time = 0.4f;
        p.ragdoll_trigger_threshold = 0.7f;
        p.stamina_cost = 0.25f;
        return p;
    }

    static TackleProfile make_dive_tackle() {
        TackleProfile p;
        p.type = TackleType::DIVE_SLIDE;
        p.name = "dive_tackle";
        p.base_success_rate = 0.6f;
        p.power_multiplier = 0.9f;
        p.momentum_transfer = 0.7f;
        p.fumble_chance = 0.1f;
        p.injury_risk = 0.04f;
        p.recovery_time = 0.9f;
        p.reach = 2.0f;
        p.execution_time = 0.45f;
        p.ragdoll_trigger_threshold = 0.65f;
        p.stamina_cost = 0.3f;
        return p;
    }
};

// =============================================================================
// BlockProfile — Per-block-type parameters
// =============================================================================
struct BlockProfile {
    BlockType type = BlockType::NONE;
    const char* name = "none";

    float base_success_rate = 0.7f;
    float impact_absorption = 0.8f;     // How much force is absorbed
    float deflection_angle = 0.3f;      // How much the attacker is deflected
    float hold_strength = 0.0f;         // Grip strength (0 = no grab)
    float push_back_force = 500.0f;     // Force to push back attacker (N)
    float stamina_cost = 0.1f;
    float hold_duration = 0.0f;         // How long the block is maintained
    float recovery_time = 0.3f;

    static BlockProfile make_pass_block() {
        BlockProfile p;
        p.type = BlockType::PASS_BLOCK;
        p.name = "pass_block";
        p.base_success_rate = 0.65f;
        p.impact_absorption = 0.7f;
        p.deflection_angle = 0.4f;
        p.push_back_force = 600.0f;
        p.stamina_cost = 0.12f;
        p.hold_duration = 2.0f;
        p.recovery_time = 0.3f;
        return p;
    }

    static BlockProfile make_run_block() {
        BlockProfile p;
        p.type = BlockType::RUN_BLOCK;
        p.name = "run_block";
        p.base_success_rate = 0.75f;
        p.impact_absorption = 0.85f;
        p.deflection_angle = 0.5f;
        p.push_back_force = 800.0f;
        p.stamina_cost = 0.15f;
        p.hold_duration = 1.5f;
        p.recovery_time = 0.25f;
        return p;
    }

    static BlockProfile make_basketball_screen() {
        BlockProfile p;
        p.type = BlockType::SCREEN;
        p.name = "basketball_screen";
        p.base_success_rate = 0.8f;
        p.impact_absorption = 0.9f;
        p.deflection_angle = 0.6f;
        p.push_back_force = 200.0f;
        p.stamina_cost = 0.05f;
        p.hold_duration = 0.5f;
        p.recovery_time = 0.2f;
        return p;
    }

    static BlockProfile make_body_check() {
        BlockProfile p;
        p.type = BlockType::BODY_CHECK;
        p.name = "body_check";
        p.base_success_rate = 0.7f;
        p.impact_absorption = 0.5f;     // Less absorption, more impact
        p.deflection_angle = 0.8f;      // Strong deflection
        p.push_back_force = 1200.0f;    // Very strong
        p.stamina_cost = 0.2f;
        p.hold_duration = 0.1f;         // Instant
        p.recovery_time = 0.4f;
        return p;
    }
};

// =============================================================================
// GrappleHold — Grappling state for wrestling/rugby maul mechanics
// =============================================================================
enum class GrappleType : uint8_t {
    NONE         = 0,
    COLLAR_TIE   = 1,   // Wrestling collar tie
    BODY_LOCK    = 2,   // Body lock / bear hug
    SINGLE_LEG   = 3,   // Single leg grab
    DOUBLE_LEG   = 4,   // Double leg (takedown attempt)
    HEAD_LOCK    = 5,   // Head lock
    MAUL_BIND    = 6,   // Rugby maul binding
    RUCK_OVER    = 7,   // Rugby ruck over (driving over)
    JUDO_HIP     = 8,   // Judo hip throw position
    JUDO_SHOULDER = 9   // Judo shoulder throw position
};

struct GrappleState {
    GrappleType type = GrappleType::NONE;
    uint32_t attacker_id = 0xFFFFFFFF;
    uint32_t defender_id = 0xFFFFFFFF;
    float hold_strength = 0.0f;       // 0-1, how secure the grip is
    float hold_duration = 0.0f;       // How long the hold has been maintained
    float max_hold_duration = 3.0f;   // Maximum hold time before break
    float escape_progress = 0.0f;     // 0-1, defender's escape progress
    float throw_progress = 0.0f;      // 0-1, attacker's throw/takedown progress
    bool is_resolved = false;         // Has the grapple been resolved?
    float stalemate_timer = 0.0f;     // Time in stalemate (no progress)

    bool is_active() const {
        return type != GrappleType::NONE && !is_resolved;
    }

    void break_hold() {
        type = GrappleType::NONE;
        hold_strength = 0.0f;
        hold_duration = 0.0f;
        is_resolved = true;
    }

    void reset() {
        type = GrappleType::NONE;
        hold_strength = 0.0f;
        hold_duration = 0.0f;
        escape_progress = 0.0f;
        throw_progress = 0.0f;
        is_resolved = false;
        stalemate_timer = 0.0f;
    }
};

// =============================================================================
// ContactResult — Complete result of athlete-athlete contact
// =============================================================================
struct ContactResult {
    bool contact_made = false;
    ContactType type = ContactType::COLLISION;
    ContactDirection direction = ContactDirection::FRONT;
    TackleType tackle_type = TackleType::NONE;
    BlockType block_type = BlockType::NONE;

    // --- Outcome ---
    bool tackle_successful = false;
    bool ball_dislodged = false;
    bool penalty_flagged = false;

    // --- Physics ---
    Vec3 impact_force;              // Force applied to each athlete
    float impact_magnitude = 0.0f;  // Scalar impact strength
    float impulse_applied = 0.0f;   // Total impulse
    Vec3 post_contact_vel_attacker;
    Vec3 post_contact_vel_defender;

    // --- Timing ---
    float recovery_time_attacker = 0.0f;
    float recovery_time_defender = 0.0f;

    // --- Stylization hooks (for Phase 3 integration) ---
    float hit_stop_ms = 0.0f;
    float camera_shake = 0.0f;
    float time_scale = 1.0f;
};

// =============================================================================
// ContactResolver — Determines outcome of athlete-athlete contact
// =============================================================================
struct ContactResolver {
    // --- Resolve a tackle ---
    ContactResult resolve_tackle(const Vec3& attacker_pos, const Vec3& attacker_vel,
                                  const Vec3& defender_pos, const Vec3& defender_vel,
                                  const TackleProfile& profile,
                                  float attacker_skill = 0.8f,
                                  float defender_skill = 0.8f)
    {
        ContactResult result;
        result.type = ContactType::TACKLE_FORM;
        result.tackle_type = profile.type;

        // Relative approach
        Vec3 to_defender = Vec3::sub(defender_pos, attacker_pos);
        float distance = Vec3::length(to_defender);

        if (distance > profile.reach) {
            return result; // Out of range
        }

        // Contact direction
        Vec3 attacker_dir = attacker_vel;
        float atk_speed = Vec3::length(attacker_dir);
        if (atk_speed > APC_EPSILON) {
            attacker_dir = Vec3::scale(attacker_dir, 1.0f / atk_speed);
        }

        float dot = Vec3::dot(attacker_dir, Vec3::scale(to_defender,
            1.0f / (distance + APC_EPSILON)));
        if (dot > 0.7f) result.direction = ContactDirection::BEHIND;
        else if (dot > 0.3f) result.direction = ContactDirection::FRONT;
        else {
            Vec3 cross = Vec3::cross(attacker_dir,
                Vec3::scale(to_defender, 1.0f / (distance + APC_EPSILON)));
            result.direction = (cross.y > 0.0f) ?
                ContactDirection::RIGHT_SIDE : ContactDirection::LEFT_SIDE;
        }

        // Approach angle penalty
        float approach_angle = std::acos(std::max(-1.0f, std::min(1.0f, dot)));
        float angle_penalty = 1.0f;
        if (approach_angle > profile.max_approach_angle * 0.5f) {
            angle_penalty = 1.0f - (approach_angle /
                profile.max_approach_angle) * 0.5f;
        }

        // Speed check
        if (atk_speed < profile.min_approach_speed) return result;

        result.contact_made = true;

        // Success calculation
        float speed_bonus = std::min(0.2f, atk_speed * 0.01f);
        float skill_diff = (attacker_skill - defender_skill) * 0.3f;
        float success_prob = profile.base_success_rate *
            angle_penalty + speed_bonus + skill_diff;

        // Behind tackle penalty
        if (result.direction == ContactDirection::BEHIND) {
            success_prob -= 0.15f;
            result.penalty_flagged = true; // Potential foul
        }

        result.tackle_successful = success_prob > 0.5f;

        // Impact physics
        float relative_speed = Vec3::length(Vec3::sub(attacker_vel, defender_vel));
        result.impact_magnitude = relative_speed * profile.power_multiplier *
            (attacker_skill * 0.3f + 0.7f); // Skill affects impact quality

        // Impact direction
        if (distance > APC_EPSILON) {
            Vec3 contact_normal = Vec3::scale(to_defender, 1.0f / distance);
            result.impact_force = Vec3::scale(contact_normal,
                result.impact_magnitude);
        }

        // Fumble check
        if (result.tackle_successful) {
            float fumble_prob = profile.fumble_chance *
                (1.0f + result.impact_magnitude * 0.02f) *
                (1.0f - defender_skill * 0.5f);
            result.ball_dislodged = fumble_prob > 0.5f;
        }

        // Post-contact velocities (simplified momentum exchange)
        float m_att = 85.0f; // Typical athlete mass (kg)
        float m_def = 85.0f;
        float total_mass = m_att + m_def;

        Vec3 combined_vel = Vec3::add(
            Vec3::scale(attacker_vel, m_att / total_mass),
            Vec3::scale(defender_vel, m_def / total_mass)
        );

        float transfer = profile.momentum_transfer;
        result.post_contact_vel_attacker = Vec3::add(
            Vec3::scale(combined_vel, transfer),
            Vec3::scale(attacker_vel, 1.0f - transfer)
        );
        result.post_contact_vel_defender = Vec3::add(
            Vec3::scale(combined_vel, transfer),
            Vec3::scale(defender_vel, 1.0f - transfer)
        );

        // Recovery times
        result.recovery_time_attacker = profile.recovery_time;
        if (result.tackle_successful) {
            result.recovery_time_defender = profile.ball_carrier_recovery;
        } else {
            result.recovery_time_defender = 0.1f; // Broke free quickly
        }

        // Stylization
        if (result.impact_magnitude > 8.0f) {
            result.hit_stop_ms = std::min(80.0f,
                result.impact_magnitude * 5.0f);
            result.camera_shake = std::min(0.5f,
                result.impact_magnitude * 0.03f);
        }

        result.impulse_applied = result.impact_magnitude * 0.1f; // Approximate

        return result;
    }

    // --- Resolve a block ---
    ContactResult resolve_block(const Vec3& blocker_pos, const Vec3& blocker_vel,
                                 const Vec3& attacker_pos, const Vec3& attacker_vel,
                                 const BlockProfile& profile,
                                 float blocker_skill = 0.8f,
                                 float attacker_skill = 0.8f)
    {
        ContactResult result;
        result.type = ContactType::BLOCK;
        result.block_type = profile.type;

        Vec3 to_attacker = Vec3::sub(attacker_pos, blocker_pos);
        float distance = Vec3::length(to_attacker);

        if (distance > 1.5f) return result; // Out of block range

        float attacker_speed = Vec3::length(attacker_vel);
        float blocker_speed = Vec3::length(blocker_vel);

        result.contact_made = true;

        // Success: blocker skill + angle + speed
        float dot = 1.0f;
        if (distance > APC_EPSILON && attacker_speed > APC_EPSILON) {
            dot = Vec3::dot(
                Vec3::scale(attacker_vel, 1.0f / attacker_speed),
                Vec3::scale(to_attacker, 1.0f / distance)
            );
        }

        float angle_bonus = std::max(0.0f, dot) * 0.2f;
        float success_prob = profile.base_success_rate +
            angle_bonus + (blocker_skill - attacker_skill) * 0.25f;
        success_prob = std::max(0.2f, std::min(0.95f, success_prob));

        bool block_succeeded = success_prob > 0.5f;

        // Impact
        float relative_speed = Vec3::length(Vec3::sub(attacker_vel, blocker_vel));
        float absorbed = relative_speed * profile.impact_absorption;
        float remaining = relative_speed - absorbed;

        if (distance > APC_EPSILON) {
            Vec3 contact_dir = Vec3::scale(to_attacker, 1.0f / distance);
            result.impact_force = Vec3::scale(contact_dir, absorbed);
        }

        // Deflection
        if (block_succeeded && distance > APC_EPSILON) {
            Vec3 contact_dir = Vec3::scale(to_attacker, 1.0f / distance);
            Vec3 deflect = Vec3::add(
                Vec3::scale(attacker_vel, -profile.deflection_angle * 0.5f),
                Vec3::scale(contact_dir, profile.deflection_angle * remaining)
            );
            result.post_contact_vel_attacker = Vec3::scale(deflect, remaining);
        } else {
            result.post_contact_vel_attacker = attacker_vel;
        }
        result.post_contact_vel_defender = blocker_vel;

        result.impact_magnitude = absorbed;
        result.recovery_time_attacker = profile.recovery_time * (block_succeeded ? 1.0f : 0.3f);
        result.recovery_time_defender = 0.2f;

        return result;
    }

    // --- Resolve shoulder charge ---
    ContactResult resolve_shoulder_charge(const Vec3& charger_pos,
                                          const Vec3& charger_vel,
                                          const Vec3& receiver_pos,
                                          const Vec3& receiver_vel,
                                          float charger_skill = 0.8f,
                                          float receiver_skill = 0.8f)
    {
        ContactResult result;
        result.type = ContactType::SHOULDER_CHARGE;

        Vec3 to_receiver = Vec3::sub(receiver_pos, charger_pos);
        float distance = Vec3::length(to_receiver);

        if (distance > 1.2f) return result;

        float charger_speed = Vec3::length(charger_vel);
        if (charger_speed < 3.0f) return result; // Too slow for charge

        result.contact_made = true;

        // Impact
        float relative_speed = Vec3::length(Vec3::sub(charger_vel, receiver_vel));
        result.impact_magnitude = relative_speed * 1.2f; // Shoulder = strong

        if (distance > APC_EPSILON) {
            Vec3 contact_dir = Vec3::scale(to_receiver, 1.0f / distance);
            result.impact_force = Vec3::scale(contact_dir, result.impact_magnitude);
        }

        // Momentum exchange (simplified)
        float m = 85.0f;
        float total = 2.0f * m;
        result.post_contact_vel_attacker = Vec3::add(
            Vec3::scale(charger_vel, 0.3f),
            Vec3::scale(receiver_vel, 0.7f / total * m)
        );
        result.post_contact_vel_defender = Vec3::add(
            Vec3::scale(receiver_vel, 0.3f),
            Vec3::scale(charger_vel, 0.7f / total * m)
        );

        result.recovery_time_attacker = 0.3f;
        result.recovery_time_defender = 0.4f;

        // Stylization
        if (result.impact_magnitude > 5.0f) {
            result.hit_stop_ms = result.impact_magnitude * 3.0f;
            result.camera_shake = result.impact_magnitude * 0.02f;
        }

        return result;
    }
};

// =============================================================================
// GrappleResolver — Manages grappling state machine
// =============================================================================
struct GrappleResolver {
    static constexpr uint32_t MAX_GRAPPLES = 16u;

    GrappleState grapples[MAX_GRAPPLES];
    uint32_t active_count = 0;

    // --- Initiate a grapple ---
    uint32_t initiate(uint32_t attacker_id, uint32_t defender_id,
                       GrappleType type, float initial_strength)
    {
        if (active_count >= MAX_GRAPPLES) return 0xFFFFFFFF;

        // Check if either athlete is already in a grapple
        for (uint32_t i = 0; i < active_count; ++i) {
            if (grapples[i].attacker_id == attacker_id ||
                grapples[i].defender_id == attacker_id ||
                grapples[i].attacker_id == defender_id ||
                grapples[i].defender_id == defender_id)
            {
                return 0xFFFFFFFF; // Already grappling
            }
        }

        GrappleState& g = grapples[active_count];
        g.attacker_id = attacker_id;
        g.defender_id = defender_id;
        g.type = type;
        g.hold_strength = initial_strength;
        g.hold_duration = 0.0f;
        g.escape_progress = 0.0f;
        g.throw_progress = 0.0f;
        g.is_resolved = false;
        g.stalemate_timer = 0.0f;

        return active_count++;
    }

    // --- Update grapple state ---
    void update(float dt, float attacker_effort, float defender_effort) {
        for (uint32_t i = 0; i < active_count; ++i) {
            GrappleState& g = grapples[i];
            if (!g.is_active()) continue;

            g.hold_duration += dt;

            // Check max duration
            if (g.hold_duration > g.max_hold_duration) {
                g.break_hold();
                continue;
            }

            // Attacker: builds throw progress
            g.throw_progress += attacker_effort * dt * 0.3f * g.hold_strength;

            // Defender: builds escape progress
            g.escape_progress += defender_effort * dt * 0.4f *
                (1.0f - g.hold_strength * 0.5f);

            // Hold strength decays over time (fatigue)
            g.hold_strength -= dt * 0.05f;
            g.hold_strength = std::max(0.0f, g.hold_strength);

            // Stalemate detection
            if (attacker_effort < 0.1f && defender_effort < 0.1f) {
                g.stalemate_timer += dt;
                if (g.stalemate_timer > 1.0f) {
                    g.break_hold();
                }
            } else {
                g.stalemate_timer = 0.0f;
            }

            // Check resolution
            if (g.throw_progress >= 1.0f) {
                // Attacker succeeds — throw/takedown
                g.is_resolved = true;
                g.throw_progress = 1.0f;
            } else if (g.escape_progress >= 1.0f) {
                // Defender escapes
                g.break_hold();
            }
        }
    }

    // --- Get grapple state for an athlete ---
    GrappleState* get_athlete_grapple(uint32_t athlete_id) {
        for (uint32_t i = 0; i < active_count; ++i) {
            if (grapples[i].attacker_id == athlete_id ||
                grapples[i].defender_id == athlete_id)
            {
                return &grapples[i];
            }
        }
        return nullptr;
    }

    void reset() {
        active_count = 0;
    }
};

} // namespace apc
