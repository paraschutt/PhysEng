#pragma once
// =============================================================================
// apc_ai_formation.h — Team Shape and Positional Target System
// =============================================================================
//
// Defines team formations, positional targets, and dynamic adjustments based
// on game state (ball position, possession, defend/attack balance).
//
//   - FormationType: enumeration of standard formation layouts
//   - FormationPosition: single position with shift parameters
//   - FormationSet: full formation with 11 positions and tuning
//   - FormationSystem: manages attack/defense formations, dynamic positioning
//
// Coordinate convention (normalized, scaled to field):
//   - X: -1 (own goal) to +1 (opponent goal)
//   - Y: 0 (ground level)
//   - Z: -1 (left sideline) to +1 (right sideline)
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
static constexpr uint32_t MAX_FORMATION_POSITIONS = 11;

// =============================================================================
// FormationType — Enumeration of standard formation layouts
// =============================================================================
enum class FormationType : uint8_t {
    FORMATION_4_4_2    = 0,
    FORMATION_4_3_3    = 1,
    FORMATION_3_5_2    = 2,
    FORMATION_4_2_3_1  = 3,
    FORMATION_3_4_3    = 4,
    FORMATION_4_5_1    = 5,
    FORMATION_5_3_2    = 6,
    FORMATION_4_1_4_1  = 7,
    CUSTOM             = 255
};

// =============================================================================
// FormationPosition — A single formation position with shift parameters
// =============================================================================
struct FormationPosition {
    Vec3      base_position       = { 0.0f, 0.0f, 0.0f }; // Normalized -1 to 1
    float     defensive_shift     = 0.0f;  // How far back when defending (-1 to 1)
    float     attacking_shift     = 0.0f;  // How far forward when attacking (-1 to 1)
    float     width_role          = 0.0f;  // How wide this position plays (-1 to 1)
    SportRole compatible_role     = SportRole::SOCCER_CM;
};

// =============================================================================
// FormationSet — Full formation definition
// =============================================================================
struct FormationSet {
    FormationType  type            = FormationType::FORMATION_4_4_2;
    FormationPosition positions[MAX_FORMATION_POSITIONS];
    uint8_t        position_count  = 0;
    char           name[32]        = {};
    float          defensive_depth = 0.5f;  // Overall depth when defending (0-1)
    float          pressing_intensity = 0.5f; // How high up to press (0-1)
    float          width           = 0.5f;  // Overall formation width (0-1)

    // --- Reset ---
    void reset()
    {
        type               = FormationType::FORMATION_4_4_2;
        position_count     = 0;
        defensive_depth    = 0.5f;
        pressing_intensity = 0.5f;
        width              = 0.5f;
        for (uint32_t i = 0u; i < 32u; ++i) {
            name[i] = '\0';
        }
        for (uint32_t i = 0u; i < MAX_FORMATION_POSITIONS; ++i) {
            positions[i] = FormationPosition();
        }
    }

    // --- Add a position ---
    uint8_t add_position(const FormationPosition& pos)
    {
        if (position_count >= MAX_FORMATION_POSITIONS) return 0;
        positions[position_count++] = pos;
        return 1;
    }
};

// =============================================================================
// FormationSystem — Manages attack/defense formations and dynamic positioning
// =============================================================================
struct FormationSystem {
    FormationSet attack_formation;
    FormationSet defense_formation;

    // --- Transition parameters ---
    float formation_transition_speed = 2.0f; // How fast players shift (0-10)
    float ball_influence_radius       = 15.0f; // Ball influence range (meters)
    float ball_influence_strength     = 0.3f;  // How much ball shifts players (0-1)

    // --- Cached interpolated positions (for smooth transitions) ---
    Vec3   cached_positions[MAX_FORMATION_POSITIONS];
    uint8_t cached_initialized = 0;

    // =========================================================================
    // get_formation_position — Compute interpolated target for a player
    // =========================================================================
    // possession_factor: 0.0 = full defense, 1.0 = full attack
    // Returns world-space position (normalized -1 to 1, scaled to field)
    // =========================================================================
    Vec3 get_formation_position(uint8_t player_index,
                                 const Vec3& ball_position,
                                 const Vec3& team_goal,
                                 const Vec3& opponent_goal,
                                 float possession_factor) const
    {
        if (player_index >= MAX_FORMATION_POSITIONS) {
            return Vec3(0.0f, 0.0f, 0.0f);
        }

        // Clamp possession factor
        float pf = possession_factor;
        if (pf < 0.0f) pf = 0.0f;
        if (pf > 1.0f) pf = 1.0f;

        // Get base positions from both formations
        const FormationPosition& def_pos =
            (player_index < defense_formation.position_count)
                ? defense_formation.positions[player_index]
                : attack_formation.positions[player_index];

        const FormationPosition& atk_pos =
            (player_index < attack_formation.position_count)
                ? attack_formation.positions[player_index]
                : def_pos;

        // Interpolate base positions
        Vec3 base = Vec3::lerp(def_pos.base_position, atk_pos.base_position, pf);

        // Apply defensive/attacking shift
        float shift = def_pos.defensive_shift * (1.0f - pf) +
                      atk_pos.attacking_shift * pf;
        base.x += shift;

        // Apply width
        float w = def_pos.width_role * (1.0f - pf) +
                  atk_pos.width_role * pf;
        base.z += w * 0.3f; // Scale width influence

        // Apply overall formation width
        base.z *= (0.5f + 0.5f * attack_formation.width * pf +
                     0.5f * defense_formation.width * (1.0f - pf));

        // Apply ball influence
        Vec3 influenced = get_ball_influenced_position(base, ball_position, pf);

        // Y is always 0 for ground-level formation positions
        influenced.y = 0.0f;

        return influenced;
    }

    // =========================================================================
    // get_ball_influenced_position — Shift position toward ball
    // =========================================================================
    Vec3 get_ball_influenced_position(const Vec3& base_pos,
                                       const Vec3& ball_pos,
                                       float possession) const
    {
        Vec3 to_ball(
            ball_pos.x - base_pos.x,
            0.0f,
            ball_pos.z - base_pos.z
        );
        float dist = Vec3::length(to_ball);

        if (dist < APC_EPSILON || dist > ball_influence_radius) {
            return base_pos; // No influence if too far or degenerate
        }

        // Influence falls off linearly with distance
        float influence_scale = ball_influence_strength *
            (1.0f - dist / ball_influence_radius);

        // Attacking teams pull toward ball more
        influence_scale *= (0.5f + 0.5f * possession);

        // Higher pressing = more pull
        influence_scale *= (0.5f + 0.5f * defense_formation.pressing_intensity * (1.0f - possession) +
                             0.5f + 0.5f * attack_formation.pressing_intensity * possession);

        Vec3 result = Vec3::add(base_pos, Vec3::scale(to_ball, influence_scale));

        // Keep on ground
        result.y = 0.0f;

        return result;
    }

    // =========================================================================
    // set_formation — Set attack and defense formations
    // =========================================================================
    void set_formation(FormationType attack, FormationType defense)
    {
        attack_formation  = build_formation(attack);
        defense_formation = build_formation(defense);
        cached_initialized = 0;
    }

    // =========================================================================
    // Static preset methods
    // =========================================================================

    // --- 4-4-2: Classic balanced formation ---
    static FormationSet preset_4_4_2()
    {
        FormationSet fs;
        fs.type = FormationType::FORMATION_4_4_2;
        // Copy name
        const char* n = "4-4-2";
        for (uint32_t i = 0u; n[i] != '\0' && i < 31u; ++i) {
            fs.name[i] = n[i];
        }
        fs.name[4] = '\0';

        // GK
        FormationPosition gk;
        gk.base_position = { -0.9f, 0.0f, 0.0f };
        gk.defensive_shift = -0.1f;
        gk.compatible_role = SportRole::SOCCER_GK;
        fs.add_position(gk);

        // LB, CB, CB, RB
        FormationPosition lb;
        lb.base_position = { -0.4f, 0.0f, -0.6f };
        lb.width_role = -0.3f;
        lb.compatible_role = SportRole::SOCCER_LB;
        fs.add_position(lb);

        FormationPosition cb1;
        cb1.base_position = { -0.45f, 0.0f, -0.2f };
        cb1.compatible_role = SportRole::SOCCER_CB;
        fs.add_position(cb1);

        FormationPosition cb2;
        cb2.base_position = { -0.45f, 0.0f, 0.2f };
        cb2.compatible_role = SportRole::SOCCER_CB;
        fs.add_position(cb2);

        FormationPosition rb;
        rb.base_position = { -0.4f, 0.0f, 0.6f };
        rb.width_role = 0.3f;
        rb.compatible_role = SportRole::SOCCER_RB;
        fs.add_position(rb);

        // LM, CM, CM, RM
        FormationPosition lm;
        lm.base_position = { 0.0f, 0.0f, -0.6f };
        lm.width_role = -0.3f;
        lm.attacking_shift = 0.2f;
        lm.compatible_role = SportRole::SOCCER_LW;
        fs.add_position(lm);

        FormationPosition cm1;
        cm1.base_position = { -0.1f, 0.0f, -0.2f };
        cm1.compatible_role = SportRole::SOCCER_CM;
        fs.add_position(cm1);

        FormationPosition cm2;
        cm2.base_position = { -0.1f, 0.0f, 0.2f };
        cm2.compatible_role = SportRole::SOCCER_CM;
        fs.add_position(cm2);

        FormationPosition rm;
        rm.base_position = { 0.0f, 0.0f, 0.6f };
        rm.width_role = 0.3f;
        rm.attacking_shift = 0.2f;
        rm.compatible_role = SportRole::SOCCER_RW;
        fs.add_position(rm);

        // ST, ST
        FormationPosition st1;
        st1.base_position = { 0.5f, 0.0f, -0.15f };
        st1.attacking_shift = 0.2f;
        st1.compatible_role = SportRole::SOCCER_ST;
        fs.add_position(st1);

        FormationPosition st2;
        st2.base_position = { 0.5f, 0.0f, 0.15f };
        st2.attacking_shift = 0.2f;
        st2.compatible_role = SportRole::SOCCER_ST;
        fs.add_position(st2);

        return fs;
    }

    // --- 4-3-3: Attacking formation ---
    static FormationSet preset_4_3_3()
    {
        FormationSet fs;
        fs.type = FormationType::FORMATION_4_3_3;
        const char* n = "4-3-3";
        for (uint32_t i = 0u; n[i] != '\0' && i < 31u; ++i) {
            fs.name[i] = n[i];
        }
        fs.name[4] = '\0';

        // GK
        FormationPosition gk;
        gk.base_position = { -0.9f, 0.0f, 0.0f };
        gk.compatible_role = SportRole::SOCCER_GK;
        fs.add_position(gk);

        // LB, CB, CB, RB
        FormationPosition lb;
        lb.base_position = { -0.4f, 0.0f, -0.6f };
        lb.compatible_role = SportRole::SOCCER_LB;
        fs.add_position(lb);

        FormationPosition cb1;
        cb1.base_position = { -0.45f, 0.0f, -0.2f };
        cb1.compatible_role = SportRole::SOCCER_CB;
        fs.add_position(cb1);

        FormationPosition cb2;
        cb2.base_position = { -0.45f, 0.0f, 0.2f };
        cb2.compatible_role = SportRole::SOCCER_CB;
        fs.add_position(cb2);

        FormationPosition rb;
        rb.base_position = { -0.4f, 0.0f, 0.6f };
        rb.compatible_role = SportRole::SOCCER_RB;
        fs.add_position(rb);

        // CM, CM, CM
        FormationPosition cm1;
        cm1.base_position = { -0.1f, 0.0f, -0.25f };
        cm1.compatible_role = SportRole::SOCCER_CDM;
        fs.add_position(cm1);

        FormationPosition cm2;
        cm2.base_position = { -0.15f, 0.0f, 0.0f };
        cm2.compatible_role = SportRole::SOCCER_CM;
        fs.add_position(cm2);

        FormationPosition cm3;
        cm3.base_position = { -0.1f, 0.0f, 0.25f };
        cm3.compatible_role = SportRole::SOCCER_CM;
        fs.add_position(cm3);

        // LW, ST, RW
        FormationPosition lw;
        lw.base_position = { 0.3f, 0.0f, -0.55f };
        lw.attacking_shift = 0.2f;
        lw.compatible_role = SportRole::SOCCER_LW;
        fs.add_position(lw);

        FormationPosition st;
        st.base_position = { 0.5f, 0.0f, 0.0f };
        st.attacking_shift = 0.2f;
        st.compatible_role = SportRole::SOCCER_ST;
        fs.add_position(st);

        FormationPosition rw;
        rw.base_position = { 0.3f, 0.0f, 0.55f };
        rw.attacking_shift = 0.2f;
        rw.compatible_role = SportRole::SOCCER_RW;
        fs.add_position(rw);

        return fs;
    }

    // --- 3-5-2: Midfield-heavy formation ---
    static FormationSet preset_3_5_2()
    {
        FormationSet fs;
        fs.type = FormationType::FORMATION_3_5_2;
        const char* n = "3-5-2";
        for (uint32_t i = 0u; n[i] != '\0' && i < 31u; ++i) {
            fs.name[i] = n[i];
        }
        fs.name[4] = '\0';

        // GK
        FormationPosition gk;
        gk.base_position = { -0.9f, 0.0f, 0.0f };
        gk.compatible_role = SportRole::SOCCER_GK;
        fs.add_position(gk);

        // CB, CB, CB
        FormationPosition cb1;
        cb1.base_position = { -0.5f, 0.0f, -0.25f };
        cb1.compatible_role = SportRole::SOCCER_CB;
        fs.add_position(cb1);

        FormationPosition cb2;
        cb2.base_position = { -0.55f, 0.0f, 0.0f };
        cb2.compatible_role = SportRole::SOCCER_CB;
        fs.add_position(cb2);

        FormationPosition cb3;
        cb3.base_position = { -0.5f, 0.0f, 0.25f };
        cb3.compatible_role = SportRole::SOCCER_CB;
        fs.add_position(cb3);

        // LWB, CM, CM, CM, RWB
        FormationPosition lwb;
        lwb.base_position = { -0.2f, 0.0f, -0.65f };
        lwb.width_role = -0.4f;
        lwb.compatible_role = SportRole::SOCCER_LB;
        fs.add_position(lwb);

        FormationPosition cm1;
        cm1.base_position = { -0.1f, 0.0f, -0.25f };
        cm1.compatible_role = SportRole::SOCCER_CM;
        fs.add_position(cm1);

        FormationPosition cm2;
        cm2.base_position = { -0.15f, 0.0f, 0.0f };
        cm2.compatible_role = SportRole::SOCCER_CDM;
        fs.add_position(cm2);

        FormationPosition cm3;
        cm3.base_position = { -0.1f, 0.0f, 0.25f };
        cm3.compatible_role = SportRole::SOCCER_CM;
        fs.add_position(cm3);

        FormationPosition rwb;
        rwb.base_position = { -0.2f, 0.0f, 0.65f };
        rwb.width_role = 0.4f;
        rwb.compatible_role = SportRole::SOCCER_RB;
        fs.add_position(rwb);

        // ST, ST
        FormationPosition st1;
        st1.base_position = { 0.4f, 0.0f, -0.15f };
        st1.attacking_shift = 0.2f;
        st1.compatible_role = SportRole::SOCCER_ST;
        fs.add_position(st1);

        FormationPosition st2;
        st2.base_position = { 0.4f, 0.0f, 0.15f };
        st2.attacking_shift = 0.2f;
        st2.compatible_role = SportRole::SOCCER_ST;
        fs.add_position(st2);

        return fs;
    }

    // --- 4-2-3-1: Modern formation ---
    static FormationSet preset_4_2_3_1()
    {
        FormationSet fs;
        fs.type = FormationType::FORMATION_4_2_3_1;
        const char* n = "4-2-3-1";
        for (uint32_t i = 0u; n[i] != '\0' && i < 31u; ++i) {
            fs.name[i] = n[i];
        }
        fs.name[6] = '\0';

        // GK
        FormationPosition gk;
        gk.base_position = { -0.9f, 0.0f, 0.0f };
        gk.compatible_role = SportRole::SOCCER_GK;
        fs.add_position(gk);

        // LB, CB, CB, RB
        FormationPosition lb;
        lb.base_position = { -0.4f, 0.0f, -0.6f };
        lb.compatible_role = SportRole::SOCCER_LB;
        fs.add_position(lb);

        FormationPosition cb1;
        cb1.base_position = { -0.45f, 0.0f, -0.2f };
        cb1.compatible_role = SportRole::SOCCER_CB;
        fs.add_position(cb1);

        FormationPosition cb2;
        cb2.base_position = { -0.45f, 0.0f, 0.2f };
        cb2.compatible_role = SportRole::SOCCER_CB;
        fs.add_position(cb2);

        FormationPosition rb;
        rb.base_position = { -0.4f, 0.0f, 0.6f };
        rb.compatible_role = SportRole::SOCCER_RB;
        fs.add_position(rb);

        // CDM, CDM
        FormationPosition cdm1;
        cdm1.base_position = { -0.25f, 0.0f, -0.2f };
        cdm1.compatible_role = SportRole::SOCCER_CDM;
        fs.add_position(cdm1);

        FormationPosition cdm2;
        cdm2.base_position = { -0.25f, 0.0f, 0.2f };
        cdm2.compatible_role = SportRole::SOCCER_CDM;
        fs.add_position(cdm2);

        // LW, CAM, RW
        FormationPosition lw2;
        lw2.base_position = { 0.0f, 0.0f, -0.6f };
        lw2.attacking_shift = 0.15f;
        lw2.compatible_role = SportRole::SOCCER_LW;
        fs.add_position(lw2);

        FormationPosition cam;
        cam.base_position = { 0.05f, 0.0f, 0.0f };
        cam.attacking_shift = 0.2f;
        cam.compatible_role = SportRole::SOCCER_CAM;
        fs.add_position(cam);

        FormationPosition rw2;
        rw2.base_position = { 0.0f, 0.0f, 0.6f };
        rw2.attacking_shift = 0.15f;
        rw2.compatible_role = SportRole::SOCCER_RW;
        fs.add_position(rw2);

        // ST
        FormationPosition st;
        st.base_position = { 0.45f, 0.0f, 0.0f };
        st.attacking_shift = 0.25f;
        st.compatible_role = SportRole::SOCCER_ST;
        fs.add_position(st);

        return fs;
    }

    // --- Reset ---
    void reset()
    {
        attack_formation.reset();
        defense_formation.reset();
        formation_transition_speed = 2.0f;
        ball_influence_radius       = 15.0f;
        ball_influence_strength     = 0.3f;
        cached_initialized = 0;
    }

private:
    // --- Build formation from type enum ---
    static FormationSet build_formation(FormationType type)
    {
        switch (type) {
        case FormationType::FORMATION_4_3_3:  return preset_4_3_3();
        case FormationType::FORMATION_3_5_2:  return preset_3_5_2();
        case FormationType::FORMATION_4_2_3_1: return preset_4_2_3_1();
        case FormationType::FORMATION_4_4_2:
        default: return preset_4_4_2();
        }
    }
};

} // namespace apc
