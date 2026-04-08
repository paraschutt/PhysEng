#pragma once
// =============================================================================
// apc_ai_decision.h — Utility-Based AI Decision System
// =============================================================================
//
// Implements a utility-based AI decision system for selecting the best action
// for an athlete based on game context, positional factors, and response curves.
//
//   - AIActionType: enumeration of all possible AI actions
//   - ResponseCurve: maps input [0,1] to output [0,1] via power curves
//   - UtilityScore: result struct with score, action, confidence
//   - Consideration: single evaluation factor
//   - UtilityAI: evaluates all actions and selects the highest-utility one
//   - ContextFactor: semantic context input enumeration
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation (fixed-size arrays with MAX_* constants)
//   - Deterministic
//   - C++17
//
// =============================================================================

#include "apc_entity/apc_entity_types.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_math_common.h"
#include <cstdint>
#include <cmath>

namespace apc {

// =============================================================================
// Capacity constants
// =============================================================================
static constexpr uint32_t MAX_CONSIDERATIONS  = 16;
static constexpr uint32_t MAX_ACTIONS         = 16;
static constexpr uint32_t MAX_CONTEXT_FACTORS = 8;

// =============================================================================
// AIActionType — Enumeration of all possible AI actions
// =============================================================================
enum class AIActionType : uint8_t {
    IDLE            = 0,
    MOVE_TO_POSITION = 1,
    CHASE_BALL      = 2,
    PASS_BALL       = 3,
    SHOOT_BALL      = 4,
    TACKLE          = 5,
    BLOCK           = 6,
    INTERCEPT       = 7,
    MARK_OPPONENT   = 8,
    SUPPORT_RUN     = 9,
    CROSS           = 10,
    HEADER          = 11,
    DIVE_SAVE       = 12,
    PUNT            = 13,
    FORMATION_HOLD  = 14,
    PRESS           = 15,
    ACTION_COUNT    = 16
};

// =============================================================================
// ResponseCurve — Maps an input value through a power curve
// =============================================================================
enum class ResponseCurve : uint8_t {
    LINEAR     = 0,
    QUADRATIC  = 1,   // exponent ~0.5 (favors extremes)
    EXPONENTIAL = 2    // exponent ~2.0 (favors mid-range)
};

// =============================================================================
// UtilityScore — Result of utility evaluation
// =============================================================================
struct UtilityScore {
    float     score      = 0.0f;
    AIActionType action     = AIActionType::IDLE;
    float     confidence = 0.0f;
};

// =============================================================================
// Consideration — Single evaluation factor for utility scoring
// =============================================================================
struct Consideration {
    const char*   name      = "";
    float         weight    = 1.0f;
    ResponseCurve curve     = ResponseCurve::LINEAR;
    float         min_input = 0.0f;
    float         max_input = 1.0f;

    // --- Evaluate: map input through response curve ---
    APC_FORCEINLINE float evaluate(float input) const {
        // Normalize input within [min_input, max_input]
        float range = max_input - min_input;
        float normalized = 0.0f;
        if (range > APC_EPSILON) {
            normalized = (input - min_input) / range;
        }
        // Clamp to [0, 1]
        if (normalized < 0.0f) normalized = 0.0f;
        if (normalized > 1.0f) normalized = 1.0f;

        // Apply response curve
        float exponent = 1.0f;
        switch (curve) {
        case ResponseCurve::QUADRATIC:
            exponent = 0.5f;
            break;
        case ResponseCurve::EXPONENTIAL:
            exponent = 2.0f;
            break;
        default:
            exponent = 1.0f;
            break;
        }

        return std::pow(normalized, exponent);
    }
};

// =============================================================================
// ContextFactor — Semantic context input enumeration
// =============================================================================
enum class ContextFactor : uint8_t {
    DISTANCE_TO_BALL     = 0,
    DISTANCE_TO_GOAL     = 1,
    DISTANCE_TO_OPPONENT = 2,
    STAMINA_PERCENT      = 3,
    TEAM_POSSESSION      = 4,
    TIME_REMAINING       = 5,
    SCORE_DIFFERENTIAL   = 6,
    POSITION_QUALITY     = 7
};

// =============================================================================
// UtilityAI — Evaluates all actions and selects the highest-utility action
// =============================================================================
struct UtilityAI {
    // --- Considerations: factors that influence action scoring ---
    Consideration considerations[MAX_CONSIDERATIONS];
    uint32_t       consideration_count = 0u;

    // --- Available actions ---
    AIActionType actions[MAX_ACTIONS];
    uint32_t  action_count = 0u;

    // --- Per-action importance weights (indexed by AIActionType value) ---
    float role_weights[MAX_ACTIONS];
    uint32_t role_weight_count = 0u;

    // --- Context factors: current game state inputs ---
    float context_factors[MAX_CONTEXT_FACTORS];
    uint32_t context_factor_count = 0u;

    // =========================================================================
    // evaluate — Score all actions, return the best one
    // =========================================================================
    UtilityScore evaluate(const float* inputs, uint32_t input_count) const
    {
        UtilityScore best;
        best.score = -1.0f;

        for (uint32_t a = 0u; a < action_count; ++a) {
            float action_score = get_action_score(actions[a], inputs, input_count);

            // Apply role weight if available
            uint32_t action_idx = static_cast<uint32_t>(actions[a]);
            if (action_idx < role_weight_count) {
                action_score *= role_weights[action_idx];
            }

            if (action_score > best.score) {
                best.score      = action_score;
                best.action     = actions[a];
                best.confidence = action_score; // Confidence = normalized score
            }
        }

        // Clamp confidence to [0, 1]
        if (best.confidence < 0.0f) best.confidence = 0.0f;
        if (best.confidence > 1.0f) best.confidence = 1.0f;

        return best;
    }

    // =========================================================================
    // get_action_score — Compute utility score for a specific action
    // =========================================================================
    float get_action_score(AIActionType /*action*/, const float* inputs,
                            uint32_t input_count) const
    {
        if (consideration_count == 0u || inputs == nullptr) {
            return 0.0f;
        }

        (void)input_count; // Used via eval_count below
        float result = 1.0f;
        uint32_t eval_count = (input_count < consideration_count)
                              ? input_count : consideration_count;

        for (uint32_t i = 0u; i < eval_count; ++i) {
            float score = considerations[i].evaluate(inputs[i]);
            // Apply weight
            score *= considerations[i].weight;
            result *= score;
        }

        return result;
    }

    // =========================================================================
    // add_consideration — Add a new evaluation factor
    // =========================================================================
    uint8_t add_consideration(const char* name, float weight,
                              ResponseCurve curve, float min_in, float max_in)
    {
        if (consideration_count >= MAX_CONSIDERATIONS) return 0;

        Consideration& c = considerations[consideration_count++];
        c.name      = name;
        c.weight    = weight;
        c.curve     = curve;
        c.min_input = min_in;
        c.max_input = max_in;
        return 1;
    }

    // =========================================================================
    // set_action_weight — Set the role-based weight for an action
    // =========================================================================
    void set_action_weight(AIActionType action, float weight)
    {
        uint32_t idx = static_cast<uint32_t>(action);
        if (idx < MAX_ACTIONS) {
            role_weights[idx] = weight;
            if (idx >= role_weight_count) {
                role_weight_count = idx + 1u;
            }
        }
    }

    // =========================================================================
    // configure_role — Set role_weights preset based on SportRole
    // =========================================================================
    void configure_role(SportRole role)
    {
        role_weight_count = MAX_ACTIONS;
        for (uint32_t i = 0u; i < MAX_ACTIONS; ++i) {
            role_weights[i] = 1.0f; // Default: all equal
        }

        switch (role) {
        case SportRole::SOCCER_GK:
            role_weights[static_cast<uint32_t>(AIActionType::DIVE_SAVE)]      = 5.0f;
            role_weights[static_cast<uint32_t>(AIActionType::FORMATION_HOLD)]  = 4.0f;
            role_weights[static_cast<uint32_t>(AIActionType::INTERCEPT)]       = 3.5f;
            role_weights[static_cast<uint32_t>(AIActionType::PUNT)]            = 3.0f;
            role_weights[static_cast<uint32_t>(AIActionType::CHASE_BALL)]      = 2.0f;
            break;

        case SportRole::SOCCER_CB:
        case SportRole::SOCCER_LB:
        case SportRole::SOCCER_RB:
            role_weights[static_cast<uint32_t>(AIActionType::TACKLE)]          = 3.0f;
            role_weights[static_cast<uint32_t>(AIActionType::BLOCK)]           = 2.5f;
            role_weights[static_cast<uint32_t>(AIActionType::INTERCEPT)]       = 2.5f;
            role_weights[static_cast<uint32_t>(AIActionType::MARK_OPPONENT)]   = 2.5f;
            role_weights[static_cast<uint32_t>(AIActionType::FORMATION_HOLD)]  = 2.0f;
            role_weights[static_cast<uint32_t>(AIActionType::PRESS)]           = 2.0f;
            break;

        case SportRole::SOCCER_CDM:
            role_weights[static_cast<uint32_t>(AIActionType::INTERCEPT)]       = 3.0f;
            role_weights[static_cast<uint32_t>(AIActionType::TACKLE)]          = 2.5f;
            role_weights[static_cast<uint32_t>(AIActionType::SUPPORT_RUN)]     = 2.0f;
            role_weights[static_cast<uint32_t>(AIActionType::FORMATION_HOLD)]  = 2.0f;
            role_weights[static_cast<uint32_t>(AIActionType::PRESS)]           = 2.0f;
            break;

        case SportRole::SOCCER_CM:
            role_weights[static_cast<uint32_t>(AIActionType::SUPPORT_RUN)]     = 2.5f;
            role_weights[static_cast<uint32_t>(AIActionType::PASS_BALL)]       = 2.5f;
            role_weights[static_cast<uint32_t>(AIActionType::INTERCEPT)]       = 2.0f;
            role_weights[static_cast<uint32_t>(AIActionType::CHASE_BALL)]      = 2.0f;
            role_weights[static_cast<uint32_t>(AIActionType::FORMATION_HOLD)]  = 1.5f;
            break;

        case SportRole::SOCCER_CAM:
        case SportRole::SOCCER_LW:
        case SportRole::SOCCER_RW:
            role_weights[static_cast<uint32_t>(AIActionType::PASS_BALL)]       = 3.0f;
            role_weights[static_cast<uint32_t>(AIActionType::SHOOT_BALL)]      = 2.5f;
            role_weights[static_cast<uint32_t>(AIActionType::CROSS)]           = 2.5f;
            role_weights[static_cast<uint32_t>(AIActionType::SUPPORT_RUN)]     = 2.0f;
            role_weights[static_cast<uint32_t>(AIActionType::MOVE_TO_POSITION)] = 2.0f;
            break;

        case SportRole::SOCCER_ST:
            role_weights[static_cast<uint32_t>(AIActionType::SHOOT_BALL)]      = 4.0f;
            role_weights[static_cast<uint32_t>(AIActionType::CHASE_BALL)]      = 3.0f;
            role_weights[static_cast<uint32_t>(AIActionType::HEADER)]          = 2.5f;
            role_weights[static_cast<uint32_t>(AIActionType::MOVE_TO_POSITION)] = 2.0f;
            role_weights[static_cast<uint32_t>(AIActionType::SUPPORT_RUN)]     = 1.5f;
            break;

        default:
            // Balanced defaults for unhandled roles
            break;
        }
    }

    // --- Reset all state ---
    void reset()
    {
        consideration_count = 0u;
        action_count        = 0u;
        role_weight_count   = 0u;
        context_factor_count = 0u;
        for (uint32_t i = 0u; i < MAX_CONSIDERATIONS; ++i) {
            considerations[i] = Consideration();
        }
        for (uint32_t i = 0u; i < MAX_ACTIONS; ++i) {
            actions[i] = AIActionType::IDLE;
            role_weights[i] = 1.0f;
        }
        for (uint32_t i = 0u; i < MAX_CONTEXT_FACTORS; ++i) {
            context_factors[i] = 0.0f;
        }
    }
};

} // namespace apc
