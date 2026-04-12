// =============================================================================
// test_sprint30.cpp — Phase 12.5: Multi-Sport Sandbox Tests
// =============================================================================
//
// Validates the sport factory, hot-swap mechanism, and field configuration
// switching between Soccer and Basketball.
//
// Tests:
//   1. soccer_sandbox_factory  — MatchConfig defaults for soccer
//   2. basketball_sandbox_factory — MatchConfig defaults for basketball
//   3. sport_field_dimensions_differ — Soccer and basketball field sizes differ
//   4. load_match_clears_previous — Unload wipes state before rebuild
//   5. sport_config_injects_correct_actions — SOCCER vs BASKETBALL AI actions
//   6. hot_swap_sequence — Soccer → Basketball → Soccer preserves determinism
//   7. sport_module_type_mapping — SportType → SportModuleType correct mapping
//
// =============================================================================

#include "apc_app/apc_application.h"
#include "apc_app/apc_scene_manager.h"
#include "apc_app/apc_sport_config.h"
#include "apc_sport/apc_sport_field.h"
#include "apc_sport/apc_sport_rules.h"

#include <cstdio>
#include <cmath>

using namespace apc;

static int g_tests_passed = 0;
static int g_tests_failed = 0;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::printf("  FAIL: %s (line %d)\n", msg, __LINE__); \
            ++g_tests_failed; \
        } else { \
            ++g_tests_passed; \
        } \
    } while(0)

#define TEST_ASSERT_FLOAT_EQ(a, b, msg, eps) \
    do { \
        float _a = (a), _b = (b); \
        if (std::fabs(_a - _b) > (eps)) { \
            std::printf("  FAIL: %s — expected %.4f, got %.4f (line %d)\n", \
                        msg, _b, _a, __LINE__); \
            ++g_tests_failed; \
        } else { \
            ++g_tests_passed; \
        } \
    } while(0)

// ---------------------------------------------------------------------------
// Test 1: Soccer sandbox factory produces correct defaults
// ---------------------------------------------------------------------------
void test_soccer_sandbox_factory() {
    std::printf("[Sprint 30] Test 1: soccer_sandbox_factory\n");

    MatchConfig cfg = MatchConfig::soccer_sandbox();

    TEST_ASSERT(cfg.sport == SportType::SOCCER, "Sport must be SOCCER");
    TEST_ASSERT_FLOAT_EQ(cfg.field_length, 105.0f, "Field length must be 105m", 0.01f);
    TEST_ASSERT_FLOAT_EQ(cfg.field_width, 68.0f, "Field width must be 68m", 0.01f);
    TEST_ASSERT(cfg.players_per_team == 11, "Players per team must be 11");
    TEST_ASSERT(cfg.halves == 2, "Must have 2 halves");
    TEST_ASSERT_FLOAT_EQ(cfg.half_duration, 2700.0f, "Half duration must be 2700s (45 min)", 0.01f);
    TEST_ASSERT(cfg.offside_enabled == 1, "Offside must be enabled");
    TEST_ASSERT(cfg.home_formation == FormationType::FORMATION_4_4_2, "Home formation must be 4-4-2");
    TEST_ASSERT(cfg.away_formation == FormationType::FORMATION_4_4_2, "Away formation must be 4-4-2");
    TEST_ASSERT_FLOAT_EQ(cfg.position_jitter, 3.0f, "Position jitter must be 3.0m", 0.01f);

    // Team names should be non-empty
    TEST_ASSERT(cfg.home_team_name[0] != '\0', "Home team name must not be empty");
    TEST_ASSERT(cfg.away_team_name[0] != '\0', "Away team name must not be empty");
}

// ---------------------------------------------------------------------------
// Test 2: Basketball sandbox factory produces correct defaults
// ---------------------------------------------------------------------------
void test_basketball_sandbox_factory() {
    std::printf("[Sprint 30] Test 2: basketball_sandbox_factory\n");

    MatchConfig cfg = MatchConfig::basketball_sandbox();

    TEST_ASSERT(cfg.sport == SportType::BASKETBALL, "Sport must be BASKETBALL");
    TEST_ASSERT_FLOAT_EQ(cfg.field_length, 28.0f, "Field length must be 28m", 0.01f);
    TEST_ASSERT_FLOAT_EQ(cfg.field_width, 15.0f, "Field width must be 15m", 0.01f);
    TEST_ASSERT(cfg.players_per_team == 5, "Players per team must be 5");
    TEST_ASSERT(cfg.halves == 4, "Must have 4 quarters");
    TEST_ASSERT_FLOAT_EQ(cfg.half_duration, 600.0f, "Quarter duration must be 600s (10 min)", 0.01f);
    TEST_ASSERT(cfg.offside_enabled == 0, "Offside must be disabled");
    TEST_ASSERT_FLOAT_EQ(cfg.position_jitter, 1.0f, "Position jitter must be 1.0m", 0.01f);

    TEST_ASSERT(cfg.home_team_name[0] != '\0', "Home team name must not be empty");
    TEST_ASSERT(cfg.away_team_name[0] != '\0', "Away team name must not be empty");
}

// ---------------------------------------------------------------------------
// Test 3: Soccer and basketball field dimensions differ significantly
// ---------------------------------------------------------------------------
void test_sport_field_dimensions_differ() {
    std::printf("[Sprint 30] Test 3: sport_field_dimensions_differ\n");

    MatchConfig soccer = MatchConfig::soccer_sandbox();
    MatchConfig basketball = MatchConfig::basketball_sandbox();

    // Soccer field must be much larger than basketball court
    TEST_ASSERT(soccer.field_length > basketball.field_length * 3.0f,
                "Soccer field must be >3x longer than basketball court");
    TEST_ASSERT(soccer.field_width > basketball.field_width * 3.0f,
                "Soccer field must be >3x wider than basketball court");

    // Soccer and basketball have different sport types
    TEST_ASSERT(soccer.sport != basketball.sport,
                "Soccer and basketball must have different SportType values");

    // Different structure rules
    TEST_ASSERT(soccer.halves != basketball.halves,
                "Soccer (2 halves) vs Basketball (4 quarters) must differ");
    TEST_ASSERT(soccer.offside_enabled != basketball.offside_enabled,
                "Offside enabled in soccer, disabled in basketball");
}

// ---------------------------------------------------------------------------
// Test 4: load_match clears previous state before rebuild
// ---------------------------------------------------------------------------
void test_load_match_clears_previous() {
    std::printf("[Sprint 30] Test 4: load_match_clears_previous\n");

    Application app;
    ApplicationConfig app_cfg = Application::soccer_defaults();
    app_cfg.enable_ai_debug = 0;
    app.init(app_cfg);

    // Load soccer first
    MatchConfig soccer = MatchConfig::soccer_sandbox();
    uint8_t result = app.load_match(soccer);
    TEST_ASSERT(result == 1, "load_match soccer must succeed");
    TEST_ASSERT(app.scene.is_loaded == 1, "Scene must be loaded");
    TEST_ASSERT(app.scene.entity_manager.athlete_count > 0, "Athletes must be spawned");
    uint32_t soccer_athletes = app.scene.entity_manager.athlete_count;

    // Score a goal to create state
    app.scene.home_score = 3u;
    app.scene.away_score = 2u;
    app.scene.match_time_seconds = 1500.0f;

    // Now switch to basketball — load_match must clear everything
    MatchConfig basketball = MatchConfig::basketball_sandbox();
    result = app.load_match(basketball);
    TEST_ASSERT(result == 1, "load_match basketball must succeed");

    // Scores must be reset
    TEST_ASSERT(app.scene.home_score == 0u, "Home score must be 0 after switch");
    TEST_ASSERT(app.scene.away_score == 0u, "Away score must be 0 after switch");
    TEST_ASSERT_FLOAT_EQ(app.scene.match_time_seconds, 0.0f,
                         "Match time must be 0 after switch", 0.001f);

    // Ball possession must be cleared
    TEST_ASSERT(app.scene.last_possession_team == TEAM_NONE,
                "Ball possession must be cleared after switch");

    // Field must be reconfigured to basketball dimensions
    TEST_ASSERT(app.scene.config.sport == SportType::BASKETBALL,
                "Config sport must be BASKETBALL after switch");
    TEST_ASSERT_FLOAT_EQ(app.scene.config.field_length, 28.0f,
                         "Field length must be 28m for basketball", 0.01f);
    TEST_ASSERT_FLOAT_EQ(app.scene.config.field_width, 15.0f,
                         "Field width must be 15m for basketball", 0.01f);

    // Influence map must be cleared
    float threat = app.scene.influence_map.get_threat(0, 0);
    float control = app.scene.influence_map.get_control(0, 0);
    TEST_ASSERT_FLOAT_EQ(threat, 0.0f, "Influence map threat must be 0 after switch", 0.001f);
    TEST_ASSERT_FLOAT_EQ(control, 0.0f, "Influence map control must be 0 after switch", 0.001f);

    // Semantic zones must be rebuilt for basketball
    uint32_t defense_zones = 0u;
    for (uint32_t i = 0u; i < app.scene.field.semantic_zone_count; ++i) {
        if (app.scene.field.semantic_zones[i].semantic == ZoneSemantic::RESTRICTED_DEFENSE) {
            ++defense_zones;
        }
    }
    TEST_ASSERT(defense_zones >= 2u,
                "Basketball must have at least 2 RESTRICTED_DEFENSE zones (paints)");

    app.shutdown();
}

// ---------------------------------------------------------------------------
// Test 5: load_sport_configuration injects correct AI actions per sport
// ---------------------------------------------------------------------------
void test_sport_config_injects_correct_actions() {
    std::printf("[Sprint 30] Test 5: sport_config_injects_correct_actions\n");

    // Test SOCCER actions
    {
        Application app;
        ApplicationConfig app_cfg = Application::soccer_defaults();
        app_cfg.enable_ai_debug = 0;
        app.init(app_cfg);

        MatchConfig soccer = MatchConfig::soccer_sandbox();
        app.load_match(soccer);

        // Soccer should have CROSS and HEADER (not in basketball)
        const UtilityAI& home_ai = app.scene.utility_ai[0];
        bool has_cross = false, has_header = false, has_tackle = false;
        bool has_shoot = false, has_pass = false;
        for (uint32_t i = 0u; i < home_ai.action_count; ++i) {
            if (home_ai.actions[i] == AIActionType::CROSS)    has_cross = true;
            if (home_ai.actions[i] == AIActionType::HEADER)   has_header = true;
            if (home_ai.actions[i] == AIActionType::TACKLE)   has_tackle = true;
            if (home_ai.actions[i] == AIActionType::SHOOT_BALL) has_shoot = true;
            if (home_ai.actions[i] == AIActionType::PASS_BALL)  has_pass = true;
        }
        TEST_ASSERT(has_cross, "Soccer AI must have CROSS action");
        TEST_ASSERT(has_header, "Soccer AI must have HEADER action");
        TEST_ASSERT(has_tackle, "Soccer AI must have TACKLE action");
        TEST_ASSERT(has_shoot, "Soccer AI must have SHOOT action");
        TEST_ASSERT(has_pass, "Soccer AI must have PASS action");

        app.shutdown();
    }

    // Test BASKETBALL actions
    {
        Application app;
        ApplicationConfig app_cfg = Application::basketball_defaults();
        app_cfg.enable_ai_debug = 0;
        app.init(app_cfg);

        MatchConfig basketball = MatchConfig::basketball_sandbox();
        app.load_match(basketball);

        const UtilityAI& home_ai = app.scene.utility_ai[0];
        bool has_cross = false, has_header = false, has_tackle = false;
        bool has_shoot = false, has_pass = false;
        for (uint32_t i = 0u; i < home_ai.action_count; ++i) {
            if (home_ai.actions[i] == AIActionType::CROSS)    has_cross = true;
            if (home_ai.actions[i] == AIActionType::HEADER)   has_header = true;
            if (home_ai.actions[i] == AIActionType::TACKLE)   has_tackle = true;
            if (home_ai.actions[i] == AIActionType::SHOOT_BALL) has_shoot = true;
            if (home_ai.actions[i] == AIActionType::PASS_BALL)  has_pass = true;
        }
        TEST_ASSERT(!has_cross, "Basketball AI must NOT have CROSS action");
        TEST_ASSERT(!has_header, "Basketball AI must NOT have HEADER action");
        TEST_ASSERT(!has_tackle, "Basketball AI must NOT have TACKLE action");
        TEST_ASSERT(has_shoot, "Basketball AI must have SHOOT action");
        TEST_ASSERT(has_pass, "Basketball AI must have PASS action");

        app.shutdown();
    }
}

// ---------------------------------------------------------------------------
// Test 6: Hot-swap sequence preserves determinism
// ---------------------------------------------------------------------------
void test_hot_swap_sequence() {
    std::printf("[Sprint 30] Test 6: hot_swap_sequence\n");

    Application app;
    ApplicationConfig app_cfg = Application::soccer_defaults();
    app_cfg.enable_ai_debug = 0;
    app.init(app_cfg);

    // First: load soccer
    MatchConfig soccer = MatchConfig::soccer_sandbox();
    app.load_match(soccer);
    uint32_t soccer_count = app.scene.entity_manager.athlete_count;

    // Hot-swap to basketball
    MatchConfig basketball = MatchConfig::basketball_sandbox();
    app.load_match(basketball);
    TEST_ASSERT(app.scene.config.sport == SportType::BASKETBALL,
                "Must be BASKETBALL after hot-swap");

    // Hot-swap back to soccer
    app.load_match(soccer);
    TEST_ASSERT(app.scene.config.sport == SportType::SOCCER,
                "Must be SOCCER after second hot-swap");
    TEST_ASSERT(app.scene.config.field_length == 105.0f,
                "Field length must be restored to 105m");
    TEST_ASSERT(app.scene.config.field_width == 68.0f,
                "Field width must be restored to 68m");
    TEST_ASSERT(app.scene.home_score == 0u, "Scores must be reset");
    TEST_ASSERT(app.scene.away_score == 0u, "Scores must be reset");

    // Athletes must be re-spawned with same count (deterministic)
    TEST_ASSERT(app.scene.entity_manager.athlete_count == soccer_count,
                "Athlete count must be deterministic after round-trip swap");

    app.shutdown();
}

// ---------------------------------------------------------------------------
// Test 7: SportModuleType mapping is correct
// ---------------------------------------------------------------------------
void test_sport_module_type_mapping() {
    std::printf("[Sprint 30] Test 7: sport_module_type_mapping\n");

    // The mapping is: SportType -> SportModuleType via switch in assign_all_ai
    // Verify the enum values are distinct and ordered correctly
    TEST_ASSERT(static_cast<uint8_t>(SportModuleType::SOCCER) !=
                static_cast<uint8_t>(SportModuleType::BASKETBALL),
                "SOCCER and BASKETBALL module types must be distinct");
    TEST_ASSERT(static_cast<uint8_t>(SportModuleType::SOCCER) == 0u,
                "SOCCER must be value 0");
    TEST_ASSERT(static_cast<uint8_t>(SportModuleType::BASKETBALL) == 1u,
                "BASKETBALL must be value 1");

    // Verify FormationTopology values
    TEST_ASSERT(static_cast<uint8_t>(FormationTopology::FLUID_INVASION) == 0u,
                "FLUID_INVASION must be value 0");
    TEST_ASSERT(static_cast<uint8_t>(FormationTopology::COURT_ZONAL) == 2u,
                "COURT_ZONAL must be value 2");

    // Verify SportModuleConfig default is SOCCER
    SportModuleConfig module_cfg;
    TEST_ASSERT(module_cfg.module_type == SportModuleType::SOCCER,
                "SportModuleConfig default must be SOCCER");
    TEST_ASSERT(module_cfg.topology == FormationTopology::FLUID_INVASION,
                "SportModuleConfig default topology must be FLUID_INVASION");
}

// ===========================================================================
// Main
// ===========================================================================
int main() {
    std::printf("====================================================\n");
    std::printf("  Sprint 30: Phase 12.5 — Multi-Sport Sandbox Tests\n");
    std::printf("====================================================\n\n");

    test_soccer_sandbox_factory();
    test_basketball_sandbox_factory();
    test_sport_field_dimensions_differ();
    test_load_match_clears_previous();
    test_sport_config_injects_correct_actions();
    test_hot_swap_sequence();
    test_sport_module_type_mapping();

    std::printf("\n====================================================\n");
    std::printf("  Results: %d passed, %d failed\n", g_tests_passed, g_tests_failed);
    std::printf("====================================================\n");

    return (g_tests_failed > 0) ? 1 : 0;
}
