#pragma once
// =============================================================================
// Impact Outcome Table — Designer-authored conditional physics outcomes
// =============================================================================
//
// Implements a rule-based system where designers define IF/THEN rules that
// override physics simulation toward authored outcomes. This is the core of
// "physics that simulates toward outcomes rather than away from them."
//
// Each rule has:
//   - Conditions: speed thresholds, contact region, airborne state, etc.
//   - Actions: launch angle, spin rate, ragdoll duration, blend mode overrides
//   - Priority: higher priority rules are evaluated first
//
// The table is evaluated after the StylizedSolver produces ImpactEvents.
// Matching rules generate OutcomeActions that modify body velocities and
// blend modes to steer the simulation toward the designer's intent.
//
// Authoring format (text-based, parseable):
//   RULE: "Big Hit Launch"
//     PRIORITY: 100
//     IF speed > 15.0 AND region_b = TORSO AND airborne_b = true
//     THEN LAUNCH body_b angle=45 spin=720 duration=1.2
//     THEN RAGDOLL body_b duration=2.0 stiffness=0.0
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation (fixed-capacity rule/action buffers)
//   - Deterministic: rules evaluated in priority order, first match wins
//   - C++17
//
// =============================================================================

#include "apc_style/apc_stylized_solver.h"
#include "apc_style/apc_impact_profile.h"
#include "apc_style/apc_game_hooks.h"
#include "apc_skeleton/apc_skeleton_types.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include "apc_math/apc_math_common.h"
#include <cstdint>
#include <cstring>
#include <cmath>

namespace apc {

// =============================================================================
// OutcomeActionType — Types of outcome actions
// =============================================================================
enum class OutcomeActionType : uint8_t {
    NONE            = 0,
    LAUNCH          = 1,   // Apply directed launch (angle + speed)
    SPIN            = 2,   // Apply spin (axis + rate)
    RAGDOLL         = 3,   // Switch to full ragdoll
    BLEND_OVERRIDE  = 4,   // Change blend mode for specific bones
    VELOCITY_SCALE  = 5,   // Scale velocity (dampen or amplify)
    ZERO_VELOCITY   = 6    // Kill all velocity (catch/stop)
};

// =============================================================================
// OutcomeAction — A single action to apply when a rule matches
// =============================================================================
struct OutcomeAction {
    OutcomeActionType type = OutcomeActionType::NONE;
    uint32_t target_body = 0;     // Which body to apply to
    float param[4] = {0, 0, 0, 0}; // Type-dependent parameters:
    // LAUNCH:     param[0] = angle (degrees), param[1] = speed (m/s),
    //             param[2] = vertical_bias (0-1), param[3] = randomness
    // SPIN:       param[0] = spin_rate (deg/s), param[1] = axis_x,
    //             param[2] = axis_y, param[3] = axis_z
    // RAGDOLL:    param[0] = duration (seconds), param[1] = stiffness,
    //             param[2] = damping
    // BLEND_OVERRIDE: param[0] = mode (0=ANIM, 1=PHYSICS, 2=BLENDED),
    //               param[1] = stiffness, param[2] = damping,
    //               param[3] = max_deviation
    // VELOCITY_SCALE: param[0] = scale factor
    // ZERO_VELOCITY: (no params)
};

// =============================================================================
// OutcomeCondition — Conditions that must all be true for a rule to match
// =============================================================================
struct OutcomeCondition {
    float min_speed = 0.0f;           // Minimum relative speed (m/s)
    float max_speed = 1e10f;         // Maximum relative speed (m/s)
    ContactRegion region_a = ContactRegion::UNKNOWN; // Required region A (UNKNOWN = any)
    ContactRegion region_b = ContactRegion::UNKNOWN; // Required region B (UNKNOWN = any)
    uint8_t require_airborne_a = 0;   // 1 = body A must be airborne
    uint8_t require_airborne_b = 0;   // 1 = body B must be airborne
    uint8_t require_grounded_a = 0;   // 1 = body A must be grounded
    uint8_t require_grounded_b = 0;   // 1 = body B must be grounded
    float min_impact_force = 0.0f;    // Minimum impact force
    float max_impact_force = 1e10f;   // Maximum impact force
    uint16_t required_profile_a = 0xFFFF; // Specific profile required (0xFFFF = any)
    uint16_t required_profile_b = 0xFFFF; // Specific profile required (0xFFFF = any)
};

// =============================================================================
// OutcomeRule — A complete IF/THEN rule
// =============================================================================
struct OutcomeRule {
    static constexpr uint32_t MAX_ACTIONS = 4u;
    static constexpr uint32_t NAME_LENGTH = 32u;

    char name[NAME_LENGTH];           // Human-readable name
    float priority = 0.0f;           // Higher = evaluated first
    OutcomeCondition condition;      // All must be true
    OutcomeAction actions[MAX_ACTIONS];
    uint8_t action_count = 0;

    void set_name(const char* n) {
        std::memset(name, 0, NAME_LENGTH);
        uint32_t len = 0;
        while (n[len] != '\0' && len < NAME_LENGTH - 1) {
            name[len] = n[len];
            ++len;
        }
    }

    void add_action(const OutcomeAction& action) {
        if (action_count < MAX_ACTIONS) {
            actions[action_count] = action;
            ++action_count;
        }
    }
};

// =============================================================================
// OutcomeTable — Collection of outcome rules
// =============================================================================
class OutcomeTable {
public:
    static constexpr uint32_t MAX_RULES = 32u;
    static constexpr uint32_t MAX_RESULTS = 32u;

    // -----------------------------------------------------------------
    // add_rule — Add a rule to the table.
    // Rules are kept sorted by priority (highest first).
    // -----------------------------------------------------------------
    void add_rule(const OutcomeRule& rule) {
        if (rule_count_ >= MAX_RULES) return;

        // Insert in priority order (highest first)
        uint32_t insert_pos = rule_count_;
        for (uint32_t i = 0; i < rule_count_; ++i) {
            if (rule.priority > rules_[i].priority) {
                insert_pos = i;
                break;
            }
        }

        // Shift rules down to make room
        for (uint32_t i = rule_count_; i > insert_pos; --i) {
            rules_[i] = rules_[i - 1];
        }
        rules_[insert_pos] = rule;
        ++rule_count_;
    }

    // -----------------------------------------------------------------
    // evaluate — Check all rules against an impact event.
    //
    // Returns the first matching rule's actions. Multiple rules can match
    // if stop_on_first_match is false (evaluates all and collects actions).
    //
    // Returns the number of actions generated.
    // -----------------------------------------------------------------
    uint32_t evaluate(const ImpactEvent& evt,
                      OutcomeAction* out_actions,
                      uint32_t max_actions,
                      bool stop_on_first_match = true) const
    {
        uint32_t total_actions = 0;

        for (uint32_t r = 0; r < rule_count_; ++r) {
            const OutcomeRule& rule = rules_[r];

            if (check_conditions(rule.condition, evt)) {
                // Rule matched — collect its actions
                for (uint32_t a = 0;
                     a < rule.action_count && total_actions < max_actions;
                     ++a)
                {
                    out_actions[total_actions++] = rule.actions[a];
                }

                if (stop_on_first_match) break;
            }
        }

        return total_actions;
    }

    // -----------------------------------------------------------------
    // apply_actions — Apply outcome actions to rigid bodies.
    //
    // This modifies body velocities and is called after the solver step.
    // The actions steer the simulation toward the designer's intended outcome.
    // -----------------------------------------------------------------
    static void apply_actions(
        const OutcomeAction* actions,
        uint32_t action_count,
        std::vector<RigidBody>& bodies,
        const Vec3& contact_normal)
    {
        for (uint32_t a = 0; a < action_count; ++a) {
            const OutcomeAction& act = actions[a];

            if (act.target_body >= bodies.size()) continue;
            RigidBody& body = bodies[act.target_body];
            if (body.inverse_mass == 0.0f) continue; // Skip static bodies

            switch (act.type) {
            case OutcomeActionType::LAUNCH: {
                // Apply a directed launch at the specified angle from horizontal
                float angle_deg = act.param[0];
                float speed = act.param[1];
                float vert_bias = act.param[2];

                float angle_rad = angle_deg * 3.14159265358979323846f / 180.0f;

                // Compute launch direction: angle from horizontal in the plane
                // defined by the contact normal and world up
                float cos_a = std::cos(angle_rad);
                float sin_a = std::sin(angle_rad);

                Vec3 launch_dir(
                    cos_a * contact_normal.x,
                    sin_a + vert_bias,
                    cos_a * contact_normal.z
                );

                // Normalize
                float len = Vec3::length(launch_dir);
                if (len > APC_EPSILON) {
                    launch_dir = Vec3::scale(launch_dir, 1.0f / len);
                }

                // Set velocity (not add — launch overrides current motion)
                body.linear_velocity = Vec3::scale(launch_dir, speed);
                break;
            }

            case OutcomeActionType::SPIN: {
                // Apply angular velocity for tumbling
                float spin_rate = act.param[0]; // deg/s
                Vec3 axis(act.param[1], act.param[2], act.param[3]);

                float axis_len = Vec3::length(axis);
                if (axis_len > APC_EPSILON) {
                    axis = Vec3::scale(axis, 1.0f / axis_len);
                } else {
                    // Default: spin around contact normal
                    axis = contact_normal;
                }

                float spin_rad = spin_rate * 3.14159265358979323846f / 180.0f;
                body.angular_velocity = Vec3::scale(axis, spin_rad);
                break;
            }

            case OutcomeActionType::VELOCITY_SCALE: {
                // Scale all velocities by a factor
                float scale = act.param[0];
                body.linear_velocity = Vec3::scale(body.linear_velocity, scale);
                body.angular_velocity = Vec3::scale(body.angular_velocity, scale);
                break;
            }

            case OutcomeActionType::ZERO_VELOCITY: {
                // Kill all velocity (catch/stop)
                body.linear_velocity = Vec3(0, 0, 0);
                body.angular_velocity = Vec3(0, 0, 0);
                break;
            }

            case OutcomeActionType::RAGDOLL:
            case OutcomeActionType::BLEND_OVERRIDE:
            case OutcomeActionType::NONE:
                // These are handled by the game hook system, not directly
                // on rigid bodies. They're collected for the game layer.
                break;
            }
        }
    }

    // -----------------------------------------------------------------
    // get_rule_count — Number of rules in the table
    // -----------------------------------------------------------------
    uint32_t get_rule_count() const { return rule_count_; }

    // -----------------------------------------------------------------
    // setup_default_tackle_rules — Create standard football tackle rules.
    // Populates the table with rules for common tackle scenarios.
    // -----------------------------------------------------------------
    uint32_t setup_default_tackle_rules() {
        rule_count_ = 0;

        // Rule 1: Big hit — high-speed tackle on airborne receiver
        {
            OutcomeRule rule;
            rule.set_name("Big Hit Launch");
            rule.priority = 100.0f;
            rule.condition.min_speed = 15.0f;
            rule.condition.region_b = ContactRegion::TORSO;
            rule.condition.require_airborne_b = 1;

            OutcomeAction launch;
            launch.type = OutcomeActionType::LAUNCH;
            launch.target_body = 1; // Receiver
            launch.param[0] = 45.0f;  // 45 degree launch angle
            launch.param[1] = 8.0f;   // 8 m/s launch speed
            launch.param[2] = 0.3f;   // Vertical bias
            rule.add_action(launch);

            OutcomeAction spin;
            spin.type = OutcomeActionType::SPIN;
            spin.target_body = 1;
            spin.param[0] = 720.0f;  // 720 deg/s spin
            spin.param[1] = 0.0f; spin.param[2] = 1.0f; spin.param[3] = 0.0f; // Y axis
            rule.add_action(spin);

            OutcomeAction ragdoll;
            ragdoll.type = OutcomeActionType::RAGDOLL;
            ragdoll.target_body = 1;
            ragdoll.param[0] = 1.5f;  // 1.5s ragdoll duration
            ragdoll.param[1] = 0.0f;  // No stiffness recovery
            ragdoll.param[2] = 5.0f;  // Damping
            rule.add_action(ragdoll);

            add_rule(rule);
        }

        // Rule 2: Medium tackle — standard block
        {
            OutcomeRule rule;
            rule.set_name("Standard Tackle");
            rule.priority = 50.0f;
            rule.condition.min_speed = 5.0f;
            rule.condition.max_speed = 15.0f;
            rule.condition.region_b = ContactRegion::TORSO;

            OutcomeAction launch;
            launch.type = OutcomeActionType::LAUNCH;
            launch.target_body = 1;
            launch.param[0] = 25.0f;  // Lower launch angle
            launch.param[1] = 3.0f;   // Slower launch
            launch.param[2] = 0.1f;
            rule.add_action(launch);

            OutcomeAction spin;
            spin.type = OutcomeActionType::SPIN;
            spin.target_body = 1;
            spin.param[0] = 180.0f;  // Mild spin
            spin.param[1] = 0.0f; spin.param[2] = 1.0f; spin.param[3] = 0.0f;
            rule.add_action(spin);

            add_rule(rule);
        }

        // Rule 3: Low tackle — ankle/leg tackle
        {
            OutcomeRule rule;
            rule.set_name("Low Tackle");
            rule.priority = 60.0f;
            rule.condition.min_speed = 3.0f;
            rule.condition.region_b = ContactRegion::LOWER_LEG;

            OutcomeAction launch;
            launch.type = OutcomeActionType::LAUNCH;
            launch.target_body = 1;
            launch.param[0] = 15.0f;  // Very low angle (trips)
            launch.param[1] = 2.0f;
            launch.param[2] = 0.0f;   // No vertical bias
            rule.add_action(launch);

            OutcomeAction ragdoll;
            ragdoll.type = OutcomeActionType::RAGDOLL;
            ragdoll.target_body = 1;
            ragdoll.param[0] = 1.0f;
            ragdoll.param[1] = 2.0f;  // Some stiffness (trip, not launch)
            ragdoll.param[2] = 8.0f;
            rule.add_action(ragdoll);

            add_rule(rule);
        }

        // Rule 4: Helmet-to-helmet — head contact, critical hit
        {
            OutcomeRule rule;
            rule.set_name("Helmet Hit");
            rule.priority = 90.0f;
            rule.condition.min_speed = 8.0f;
            rule.condition.region_a = ContactRegion::HEAD;
            rule.condition.region_b = ContactRegion::HEAD;

            OutcomeAction launch;
            launch.type = OutcomeActionType::LAUNCH;
            launch.target_body = 0; // Both go back
            launch.param[0] = 60.0f;
            launch.param[1] = 4.0f;
            launch.param[2] = 0.4f;
            rule.add_action(launch);

            OutcomeAction ragdoll;
            ragdoll.type = OutcomeActionType::RAGDOLL;
            ragdoll.target_body = 0;
            ragdoll.param[0] = 2.0f;
            rule.add_action(ragdoll);

            add_rule(rule);
        }

        // Rule 5: Catch — ball carrier stops
        {
            OutcomeRule rule;
            rule.set_name("Catch Stop");
            rule.priority = 10.0f;
            rule.condition.max_speed = 2.0f;
            rule.condition.require_grounded_b = 1;

            OutcomeAction zero;
            zero.type = OutcomeActionType::ZERO_VELOCITY;
            zero.target_body = 1;
            rule.add_action(zero);

            add_rule(rule);
        }

        return rule_count_;
    }

private:
    OutcomeRule rules_[MAX_RULES];
    uint32_t rule_count_ = 0;

    // -----------------------------------------------------------------
    // check_conditions — Test all conditions against an impact event.
    // Returns true only if ALL conditions are satisfied.
    // -----------------------------------------------------------------
    static bool check_conditions(const OutcomeCondition& cond,
                                  const ImpactEvent& evt) {
        // Speed check
        if (evt.relative_speed < cond.min_speed) return false;
        if (evt.relative_speed > cond.max_speed) return false;

        // Impact force check
        if (evt.impact_force < cond.min_impact_force) return false;
        if (evt.impact_force > cond.max_impact_force) return false;

        // Contact region checks (UNKNOWN = match any)
        if (cond.region_a != ContactRegion::UNKNOWN &&
            evt.region_a != cond.region_a) return false;
        if (cond.region_b != ContactRegion::UNKNOWN &&
            evt.region_b != cond.region_b) return false;

        // Airborne/grounded checks (TODO: add airborne flag to ImpactEvent)
        // For now, use impact force as a proxy:
        //   High force + vertical normal ≈ airborne
        //   Low force + horizontal normal ≈ grounded
        if (cond.require_airborne_a && evt.contact_normal.y < 0.3f) return false;
        if (cond.require_airborne_b && evt.contact_normal.y < 0.3f) return false;
        if (cond.require_grounded_a && evt.contact_normal.y > 0.5f) return false;
        if (cond.require_grounded_b && evt.contact_normal.y > 0.5f) return false;

        // Profile checks
        if (cond.required_profile_a != 0xFFFF &&
            evt.profile_id_a != cond.required_profile_a) return false;
        if (cond.required_profile_b != 0xFFFF &&
            evt.profile_id_b != cond.required_profile_b) return false;

        return true;
    }
};

} // namespace apc
