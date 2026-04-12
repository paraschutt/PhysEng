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
    // Context inputs: [0]=dist_ball, [1]=dist_goal, [2]=dist_opponent,
    //                 [3]=stamina, [4]=team_possession, [5]=time_remaining,
    //                 [6]=score_diff, [7]=position_quality
    // =========================================================================
    float get_action_score(AIActionType action, const float* inputs,
                            uint32_t input_count) const
    {
        if (consideration_count == 0u || inputs == nullptr) {
            return 0.0f;
        }

        float dist_ball  = (input_count > 0) ? inputs[0] : 50.0f;
        float dist_goal  = (input_count > 1) ? inputs[1] : 60.0f;
        float stamina    = (input_count > 3) ? inputs[3] : 1.0f;
        float possession = (input_count > 4) ? inputs[4] : 0.0f;

        float result = 1.0f;

        // --- Action-specific scoring ---
        switch (action) {
        case AIActionType::CHASE_BALL:
            // Higher score when closer to ball and has stamina
            result *= 1.0f - std::min(dist_ball / 50.0f, 1.0f); // Closer = better
            result *= (0.5f + 0.5f * stamina);
            result *= (0.4f + 0.6f * possession); // More aggressive with ball
            break;

        case AIActionType::SHOOT_BALL:
            // Only viable within kick range — don't chase from afar
            result *= (dist_ball < 2.0f) ? (1.0f - dist_ball / 2.0f) : 0.0f;
            // Strongly prefer when close to opponent goal
            result *= (dist_goal < 25.0f) ? (1.0f - dist_goal / 25.0f) : 0.05f;
            result *= stamina;
            break;

        case AIActionType::MOVE_TO_POSITION:
            // Decent base score, higher when far from ball
            result *= std::min(dist_ball / 30.0f, 1.0f); // Further = more likely
            result *= 0.7f; // Good base score for positional play
            break;

        case AIActionType::SUPPORT_RUN:
            // Good when team has possession and not too far from ball
            result *= (0.3f + 0.7f * possession);
            result *= (dist_ball < 25.0f) ? 0.8f : 0.4f;
            result *= stamina;
            break;

        case AIActionType::PRESS:
            // High when close to ball and opponent likely has it
            result *= (dist_ball < 15.0f) ? (1.0f - dist_ball / 15.0f) : 0.05f;
            result *= (1.0f - possession); // Press more when NOT in possession
            result *= (0.5f + 0.5f * stamina);
            break;

        case AIActionType::INTERCEPT:
            // Good when ball is moving toward this player and opponent has possession
            result *= (dist_ball < 15.0f) ? (1.0f - dist_ball / 15.0f) : 0.02f;
            result *= (1.0f - 0.5f * possession); // Intercept when opponent has ball
            // Bonus when close to ball trajectory line (between ball and own goal)
            {
                float dist_opp = (input_count > 2) ? inputs[2] : 30.0f;
                // More valuable when opponent is nearby pressing
                if (dist_opp < 8.0f) {
                    result *= 1.3f;
                }
            }
            result *= (0.5f + 0.5f * stamina);
            break;

        case AIActionType::TACKLE:
            // Only when very close to ball and stamina is high
            result *= (dist_ball < 3.0f) ? (1.0f - dist_ball / 3.0f) : 0.0f;
            result *= (stamina > 0.3f) ? stamina : 0.0f;
            break;

        case AIActionType::PASS_BALL:
            // When has ball and teammate nearby (simplified)
            result *= (dist_ball < 2.0f) ? 0.8f : 0.0f;
            result *= possession;
            result *= stamina;
            break;

        case AIActionType::BLOCK:
            // High when close to ball trajectory and not in possession
            // Good for defenders trying to block shots/passes
            result *= (dist_ball < 8.0f) ? (1.0f - dist_ball / 8.0f) : 0.02f;
            result *= (1.0f - possession); // Block when opponent has ball
            result *= (0.6f + 0.4f * stamina);
            // Higher closer to own goal (defensive positioning)
            result *= (dist_goal > 20.0f) ? 0.7f : 1.0f;
            break;

        case AIActionType::MARK_OPPONENT:
            // Stay close to nearest opponent, especially in defensive zone
            {
                float dist_opp = (input_count > 2) ? inputs[2] : 30.0f;
                // Minimum distance threshold: don't mark if no opponents nearby
                if (dist_opp > 10.0f) {
                    result *= 0.02f;
                } else {
                    result *= (dist_opp < 5.0f) ? 0.8f : 0.15f;
                }
                result *= (1.0f - 0.5f * possession); // Mark more when defending
                // More important near own goal
                result *= (dist_goal > 25.0f) ? 0.6f : 1.0f;
            }
            break;

        case AIActionType::CROSS:
            // Cross when wide and has ball (possession), moderate distance to goal
            {
                float dist_opp = (input_count > 2) ? inputs[2] : 30.0f;
                result *= (dist_ball < 3.0f) ? 0.7f : 0.0f; // Need ball nearby
                result *= possession; // Only when team has possession
                result *= (dist_goal > 15.0f && dist_goal < 40.0f) ? 0.8f : 0.1f;
                // Better when wide (opponent far laterally)
                result *= (dist_opp > 5.0f) ? 0.7f : 0.3f;
                result *= stamina;
            }
            break;

        case AIActionType::HEADER:
            // Header when ball is in air nearby and close to goal
            // Simplified: use distance to ball and goal as proxy
            result *= (dist_ball < 4.0f) ? (1.0f - dist_ball / 4.0f) : 0.0f;
            result *= (dist_goal < 20.0f) ? (1.0f - dist_goal / 20.0f) : 0.05f;
            result *= (0.7f + 0.3f * stamina);
            break;

        case AIActionType::DIVE_SAVE:
            // Goalkeeper action: very high when close to own goal and ball nearby
            result *= (dist_ball < 10.0f) ? (1.0f - dist_ball / 10.0f) : 0.0f;
            result *= (dist_goal > 35.0f) ? 1.0f : 0.1f; // Close to own goal
            // Primarily for goalkeepers — this will be boosted by role_weights
            result *= (0.8f + 0.2f * stamina);
            break;

        case AIActionType::PUNT:
            // Goalkeeper action: punt when has ball and far from opponents
            result *= (dist_ball < 3.0f) ? 0.8f : 0.0f;
            result *= (dist_goal > 40.0f) ? 1.0f : 0.3f; // Near own goal
            result *= stamina;
            break;

        case AIActionType::FORMATION_HOLD:
        default:
            // Solid default — holding position is always reasonable
            result = 0.5f;
            // Higher when far from ball (stay in position, don't chase)
            result *= std::min(dist_ball / 25.0f, 1.0f);
            break;
        }

        // Also apply consideration-based modulation
        uint32_t eval_count = (input_count < consideration_count)
                              ? input_count : consideration_count;
        for (uint32_t i = 0u; i < eval_count; ++i) {
            float score = considerations[i].evaluate(inputs[i]);
            score *= considerations[i].weight;
            // Blend consideration into result (don't multiply raw, which kills score)
            result = result * 0.7f + score * 0.3f;
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
            role_weights[static_cast<uint32_t>(AIActionType::FORMATION_HOLD)]  = 3.5f;
            role_weights[static_cast<uint32_t>(AIActionType::CHASE_BALL)]      = 0.3f; // Defenders rarely chase
            role_weights[static_cast<uint32_t>(AIActionType::PRESS)]           = 1.0f;
            break;

        case SportRole::SOCCER_CDM:
            role_weights[static_cast<uint32_t>(AIActionType::INTERCEPT)]       = 3.0f;
            role_weights[static_cast<uint32_t>(AIActionType::TACKLE)]          = 2.5f;
            role_weights[static_cast<uint32_t>(AIActionType::SUPPORT_RUN)]     = 2.0f;
            role_weights[static_cast<uint32_t>(AIActionType::FORMATION_HOLD)]  = 3.0f;
            role_weights[static_cast<uint32_t>(AIActionType::CHASE_BALL)]      = 0.5f; // CDM rarely chases
            role_weights[static_cast<uint32_t>(AIActionType::PRESS)]           = 1.5f;
            break;

        case SportRole::SOCCER_CM:
            role_weights[static_cast<uint32_t>(AIActionType::SUPPORT_RUN)]     = 2.5f;
            role_weights[static_cast<uint32_t>(AIActionType::PASS_BALL)]       = 2.5f;
            role_weights[static_cast<uint32_t>(AIActionType::INTERCEPT)]       = 2.0f;
            role_weights[static_cast<uint32_t>(AIActionType::CHASE_BALL)]      = 0.5f; // CM stays in position
            role_weights[static_cast<uint32_t>(AIActionType::FORMATION_HOLD)]  = 2.5f;
            break;

        case SportRole::SOCCER_CAM:
        case SportRole::SOCCER_LW:
        case SportRole::SOCCER_RW:
            role_weights[static_cast<uint32_t>(AIActionType::PASS_BALL)]       = 3.0f;
            role_weights[static_cast<uint32_t>(AIActionType::SHOOT_BALL)]      = 2.5f;
            role_weights[static_cast<uint32_t>(AIActionType::CROSS)]           = 2.5f;
            role_weights[static_cast<uint32_t>(AIActionType::SUPPORT_RUN)]     = 2.0f;
            role_weights[static_cast<uint32_t>(AIActionType::MOVE_TO_POSITION)] = 2.5f;
            role_weights[static_cast<uint32_t>(AIActionType::CHASE_BALL)]      = 0.8f;
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

    // =========================================================================
    // HYSTERESIS_BONUS — Commitment bonus to prevent decision oscillation
    // =========================================================================
    // Phase 11b Action 5: When the AI evaluated actions on the previous frame
    // and chose action X, action X receives this bonus on the current frame.
    // This prevents the AI from rapidly flipping between two actions whose
    // raw scores are within noise of each other (e.g., CHASE_BALL at 0.42
    // vs SUPPORT_RUN at 0.41). The 15% value means the AI needs to see a
    // clear 15% advantage before switching — a natural "commitment" window.
    // =========================================================================
    static constexpr float HYSTERESIS_BONUS = 0.15f;

    // =========================================================================
    // clear_actions — Wipes the AI's action memory for sport transitions
    // =========================================================================
    // When switching sports (e.g., soccer -> basketball), call this first
    // then use add_action() to inject only the actions relevant to the new
    // sport. This guarantees the AI will never try to execute a CROSS or
    // SLIDE_TACKLE on a basketball court.
    // =========================================================================
    void clear_actions()
    {
        action_count = 0u;
    }

    // =========================================================================
    // add_action — Register a single action for evaluation
    // =========================================================================
    // Called by SceneManager::load_sport_configuration() to inject the
    // sport-specific subset of actions. Memory remains statically allocated;
    // only the count grows. Truncated at MAX_ACTIONS.
    // =========================================================================
    void add_action(AIActionType action)
    {
        if (action_count < MAX_ACTIONS) {
            actions[action_count++] = action;
        }
    }

    // =========================================================================
    // evaluate_with_hysteresis — Score all actions with 15% commitment bonus
    // =========================================================================
    // Phase 11b Action 5: Utility Hysteresis
    //
    // Same logic as evaluate() but applies HYSTERESIS_BONUS to the action
    // that was chosen on the previous frame (tracked via entity.active_action_id).
    // After evaluation, the winning action's ID is written back to
    // entity.active_action_id for the next frame's hysteresis check.
    //
    // This eliminates oscillation without changing the underlying scoring
    // math — it only biases the selection toward the incumbent action.
    //
    // Parameters:
    //   inputs        — Context factor array (same as evaluate())
    //   input_count   — Number of valid context factors
    //   entity        — The athlete being evaluated (non-const: writes active_action_id)
    //
    // Returns:
    //   UtilityScore with the hysteresis-adjusted best action
    // =========================================================================
    UtilityScore evaluate_with_hysteresis(const float* inputs, uint32_t input_count,
                                          AthleteEntity& entity) const
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

            // Apply hysteresis: if this action was chosen last frame, boost it
            if (action_idx == entity.active_action_id) {
                action_score += HYSTERESIS_BONUS;
                // Clamp to prevent float overflow on combined modifiers
                if (action_score > 1.15f) action_score = 1.15f;
            }

            if (action_score > best.score) {
                best.score      = action_score;
                best.action     = actions[a];
                best.confidence = action_score;
            }
        }

        // Clamp confidence to [0, 1]
        if (best.confidence < 0.0f) best.confidence = 0.0f;
        if (best.confidence > 1.0f) best.confidence = 1.0f;

        // Lock in the decision for the next frame's hysteresis check
        entity.active_action_id = static_cast<uint32_t>(best.action);

        return best;
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
