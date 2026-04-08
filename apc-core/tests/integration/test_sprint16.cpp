// =============================================================================
// Sprint 16 Tests — Sport Environments & Integration
// =============================================================================
//
// Tests for Phase 4 Sprint 16:
//   1.  FieldGeometry factories (soccer, basketball, american_football, rugby,
//       tennis, volleyball)
//   2.  SportField setup, bounds checking, zones, boundary events
//   3.  GoalPost ball-in-goal / hitting-post detection
//   4.  ScoringSystem (soccer goals, rugby try + conversion)
//   5.  DisciplineSystem (fouls, cards, penalty detection, bonus)
//   6.  ClockSystem (soccer clock, basketball shot clock)
//   7.  SportPhysicsWorld full integration (create soccer world, kick, step,
//       verify ball movement, stop play)
//   8.  SportPhysicsWorld tackle (rugby world, contact resolution)
//   9.  SportPhysicsFactory (basketball, american_football, tennis config)
//  10.  Multiple sport types (SportType enum coverage)
//
// Design:
//   - Header-only C++ library, apc:: namespace
//   - int main() + assert / printf pattern (no test framework)
//   - APC_EPSILON for all float comparisons
//
// =============================================================================

#include "apc_sport/apc_ball_physics.h"
#include "apc_sport/apc_ball_control.h"
#include "apc_sport/apc_pass_kick.h"
#include "apc_sport/apc_implement.h"
#include "apc_sport/apc_contact_sport.h"
#include "apc_sport/apc_sport_field.h"
#include "apc_sport/apc_sport_rules.h"
#include "apc_sport/apc_sport_physics.h"
#include "apc_math/apc_math_common.h"
#include <cassert>
#include <cmath>
#include <cstdio>

using apc::APC_EPSILON;

// =============================================================================
// Helpers
// =============================================================================
static int g_fail = 0;

#define ASSERT_NEAR(a, b, eps) do {                                         \
    float _a = (a), _b = (b);                                              \
    if (std::abs(_a - _b) > (eps)) {                                        \
        printf("  [FAIL] %s:%d  |%.6f - %.6f| > %.6f\n",                  \
               __FILE__, __LINE__, _a, _b, (eps));                          \
        g_fail = 1;                                                         \
    }                                                                       \
} while(0)

#define ASSERT_TRUE(expr) do {                                              \
    if (!(expr)) {                                                          \
        printf("  [FAIL] %s:%d  assertion failed: %s\n",                   \
               __FILE__, __LINE__, #expr);                                  \
        g_fail = 1;                                                         \
    }                                                                       \
} while(0)

#define ASSERT_EQ_U(a, b) do {                                              \
    if ((a) != (b)) {                                                       \
        printf("  [FAIL] %s:%d  %u != %u\n", __FILE__, __LINE__,           \
               (unsigned)(a), (unsigned)(b));                                \
        g_fail = 1;                                                         \
    }                                                                       \
} while(0)

// =============================================================================
// TEST 1: FieldGeometry factories
// =============================================================================
static void test_field_geometry_factories() {
    printf("  [Test 1] FieldGeometry factories...\n");

    // Soccer
    {
        auto f = apc::FieldGeometry::make_soccer();
        ASSERT_NEAR(f.length, 105.0f, APC_EPSILON);
        ASSERT_NEAR(f.width,  68.0f,  APC_EPSILON);
        ASSERT_NEAR(f.goal_width,  7.32f, APC_EPSILON);
        ASSERT_NEAR(f.goal_height, 2.44f, APC_EPSILON);
        ASSERT_TRUE(f.sport == apc::SportType::SOCCER);
        ASSERT_TRUE(f.type  == apc::FieldType::RECTANGLE);
    }

    // Basketball
    {
        auto f = apc::FieldGeometry::make_basketball();
        ASSERT_NEAR(f.length, 28.0f,  APC_EPSILON);
        ASSERT_NEAR(f.width,  15.0f,  APC_EPSILON);
        ASSERT_NEAR(f.goal_height, 3.05f, APC_EPSILON);
        ASSERT_TRUE(f.sport == apc::SportType::BASKETBALL);
    }

    // American Football
    {
        auto f = apc::FieldGeometry::make_american_football();
        ASSERT_NEAR(f.length, 109.7f, 0.01f);
        ASSERT_NEAR(f.width,  48.8f,  0.01f);
        ASSERT_TRUE(f.sport == apc::SportType::AMERICAN_FOOTBALL);
    }

    // Rugby
    {
        auto f = apc::FieldGeometry::make_rugby();
        ASSERT_NEAR(f.length, 100.0f, APC_EPSILON);
        ASSERT_NEAR(f.width,  70.0f,  APC_EPSILON);
        ASSERT_TRUE(f.sport == apc::SportType::RUGBY_UNION);
    }

    // Tennis
    {
        auto f = apc::FieldGeometry::make_tennis();
        ASSERT_NEAR(f.length, 23.77f,  0.01f);
        ASSERT_NEAR(f.width,  10.97f,  0.01f);
        ASSERT_TRUE(f.sport == apc::SportType::TENNIS);
        ASSERT_TRUE(f.default_surface == apc::SurfaceType::HARD_COURT);
    }

    // Volleyball
    {
        auto f = apc::FieldGeometry::make_volleyball();
        ASSERT_NEAR(f.length, 18.0f, APC_EPSILON);
        ASSERT_NEAR(f.width,  9.0f,  APC_EPSILON);
        ASSERT_TRUE(f.sport == apc::SportType::VOLLEYBALL);
    }

    printf("    [PASS]\n");
}

// =============================================================================
// TEST 2: SportField
// =============================================================================
static void test_sport_field() {
    printf("  [Test 2] SportField...\n");

    apc::SportField field;
    auto geo = apc::FieldGeometry::make_soccer();
    field.setup(geo);

    // Origin is in bounds
    ASSERT_TRUE(field.is_in_bounds(apc::Vec3(0.0f, 0.0f, 0.0f)));

    // Out of bounds along x
    ASSERT_TRUE(!field.is_in_bounds(apc::Vec3(100.0f, 0.0f, 0.0f)));

    // Out of bounds along z
    ASSERT_TRUE(!field.is_in_bounds(apc::Vec3(0.0f, 0.0f, 50.0f)));

    // Just inside (corner)
    ASSERT_TRUE(field.is_in_bounds(apc::Vec3(52.0f, 0.0f, 33.0f)));

    // Add a zone with a different surface
    field.add_zone(
        apc::FieldZoneId::PENALTY_AREA, "penalty_area",
        apc::Vec3(-48.0f, 0.0f, 0.0f),
        apc::Vec3(16.5f, 50.0f, 20.16f),
        apc::SurfaceType::ARTIFICIAL
    );

    // get_surface_at inside penalty area → ARTIFICIAL
    apc::SurfaceType s = field.get_surface_at(apc::Vec3(-48.0f, 0.0f, 0.0f));
    ASSERT_TRUE(s == apc::SurfaceType::ARTIFICIAL);

    // get_surface_at outside penalty area → GRASS (default)
    s = field.get_surface_at(apc::Vec3(0.0f, 0.0f, 0.0f));
    ASSERT_TRUE(s == apc::SurfaceType::GRASS);

    // Boundary event: ball far out of bounds
    {
        auto evt = field.check_boundary(
            apc::Vec3(60.0f, 0.0f, 0.0f),    // position
            apc::Vec3(10.0f, 0.0f, 0.0f),    // velocity
            0.11f,                             // ball radius
            0,                                 // ball_id
            1.0f                               // timestamp
        );
        ASSERT_TRUE(evt.type == apc::BoundaryEventType::OUT_OF_BOUNDS);
    }

    printf("    [PASS]\n");
}

// =============================================================================
// TEST 3: GoalPost
// =============================================================================
static void test_goal_post() {
    printf("  [Test 3] GoalPost...\n");

    // Soccer goal at the center of one end.
    // Goal implementation checks: width on rel.x, height on rel.y,
    // depth via dot(rel, facing_direction).
    // We set facing_direction = (1,0,0) so the goal faces +x (into the field).
    // "Inside" the goal means ball.x > opening_center.x (positive dot with facing).
    apc::GoalPost goal;
    goal.position          = apc::Vec3(0.0f, 0.0f, 0.0f);
    goal.opening_center    = apc::Vec3(0.0f, 0.0f, 0.0f);  // At origin for simplicity
    goal.width             = 7.32f;
    goal.height            = 2.44f;
    goal.depth             = 2.5f;
    goal.post_diameter     = 0.12f;
    goal.facing_direction  = apc::Vec3(1.0f, 0.0f, 0.0f);   // Faces +x (into field)
    goal.team_id           = 1;

    // Ball at opening center, on the goal line → in goal (depth=0, within [-radius, depth])
    {
        apc::Vec3 ball_pos(0.0f, 1.22f, 0.0f);   // dead center of opening
        ASSERT_TRUE(goal.is_ball_in_goal(ball_pos, 0.11f));
    }

    // Ball deep in the goal (1m behind goal line, in +x direction)
    {
        apc::Vec3 ball_pos(1.0f, 1.22f, 0.0f);
        ASSERT_TRUE(goal.is_ball_in_goal(ball_pos, 0.11f));
    }

    // Ball in front of goal line (in -x direction from facing)
    {
        apc::Vec3 ball_pos(-0.5f, 1.22f, 0.0f);
        ASSERT_TRUE(!goal.is_ball_in_goal(ball_pos, 0.11f));
    }

    // Ball above crossbar (rel.y > height)
    {
        apc::Vec3 ball_pos(0.0f, 4.0f, 0.0f);  // rel.y = 4.0 > 2.44
        ASSERT_TRUE(!goal.is_ball_in_goal(ball_pos, 0.11f));
    }

    // Ball outside width (rel.x outside [-half_w, half_w])
    {
        apc::Vec3 ball_pos(45.0f, 1.22f, 0.0f);  // rel.x = 45, way outside [-3.66, 3.66]
        ASSERT_TRUE(!goal.is_ball_in_goal(ball_pos, 0.11f));
    }

    // Ball near left post edge → hitting post
    // Left post is at opening_center + (-width/2, 0, 0) — post at ground level
    {
        float half_w = goal.width * 0.5f;
        apc::Vec3 left_post = apc::Vec3(-half_w, 0.0f, 0.0f);
        // Place ball just within hitting distance of the post at ground level
        apc::Vec3 ball_pos = apc::Vec3(
            left_post.x + goal.post_diameter * 0.5f + 0.11f * 0.5f,
            0.0f, 0.0f);
        ASSERT_TRUE(goal.is_ball_hitting_post(ball_pos, 0.11f));
    }

    // Ball clearly away from posts
    {
        apc::Vec3 ball_pos(52.5f, 1.22f, 0.0f);
        ASSERT_TRUE(!goal.is_ball_hitting_post(ball_pos, 0.11f));
    }

    printf("    [PASS]\n");
}

// =============================================================================
// TEST 4: ScoringSystem
// =============================================================================
static void test_scoring_system() {
    printf("  [Test 4] ScoringSystem...\n");

    apc::ScoringSystem scoring;
    scoring.configure_soccer();

    // Team 0 scores a goal
    scoring.add_score(0, 1.0f, "goal", 450.0f, apc::Vec3(52.5f, 1.22f, 0.0f));
    ASSERT_NEAR(scoring.scores[0], 1.0f, APC_EPSILON);
    ASSERT_NEAR(scoring.scores[1], 0.0f, APC_EPSILON);

    // Team 1 scores a goal
    scoring.add_score(1, 1.0f, "goal", 900.0f, apc::Vec3(-52.5f, 1.22f, 0.0f));
    ASSERT_NEAR(scoring.scores[1], 1.0f, APC_EPSILON);

    // Leading team is tied → 0xFFFFFFFF
    ASSERT_EQ_U(scoring.get_leading_team(), 0xFFFFFFFFu);

    // Team 0 scores again
    scoring.add_score(0, 1.0f, "goal", 1200.0f, apc::Vec3(52.5f, 1.22f, 0.0f));
    ASSERT_NEAR(scoring.scores[0], 2.0f, APC_EPSILON);

    // Now team 0 leads
    ASSERT_EQ_U(scoring.get_leading_team(), 0u);

    // Score difference
    ASSERT_NEAR(scoring.get_score_difference(), 1.0f, APC_EPSILON);

    // Rugby scoring: try + conversion = 5 + 2 = 7
    scoring.reset();
    scoring.configure_rugby_union();
    scoring.add_score(0, scoring.try_points, "try", 1800.0f, apc::Vec3(0.0f, 0.0f, 0.0f));
    scoring.add_score(0, scoring.conversion_points, "conversion", 1805.0f,
        apc::Vec3(0.0f, 0.0f, 0.0f));
    ASSERT_NEAR(scoring.scores[0], 7.0f, APC_EPSILON);

    printf("    [PASS]\n");
}

// =============================================================================
// TEST 5: DisciplineSystem
// =============================================================================
static void test_discipline_system() {
    printf("  [Test 5] DisciplineSystem...\n");

    apc::DisciplineSystem discipline;
    discipline.reset();
    discipline.register_athlete(0);
    discipline.register_athlete(1);

    // Record a minor foul (e.g., DELAY_OF_GAME) → no card
    {
        bool ok = discipline.record_foul(
            apc::FoulType::DELAY_OF_GAME,
            0,  // offending team
            0,  // offending athlete
            1,  // fouled team
            1   // fouled athlete
        );
        ASSERT_TRUE(ok);
        ASSERT_TRUE(discipline.fouls[0].card == apc::CardType::NONE);
    }

    // Record multiple fouls for same athlete → eventually yellow card
    // determine_card: for CHARGING (default case), athlete_fouls[id] >= 3 → YELLOW
    {
        discipline.record_foul(apc::FoulType::CHARGING, 0, 1, 1, 0);
        discipline.record_foul(apc::FoulType::CHARGING, 0, 1, 1, 0);
        discipline.record_foul(apc::FoulType::CHARGING, 0, 1, 1, 0);
        // athlete_fouls[1] should now be 4 → yellow
        uint32_t last = discipline.foul_count - 1;
        ASSERT_TRUE(discipline.fouls[last].card == apc::CardType::YELLOW);
    }

    // is_penalty_foul for TACKLE_FROM_BEHIND
    {
        ASSERT_TRUE(discipline.is_penalty_foul(apc::FoulType::TACKLE_FROM_BEHIND));
    }

    // is_penalty_foul for a non-penalty foul
    {
        ASSERT_TRUE(!discipline.is_penalty_foul(apc::FoulType::DELAY_OF_GAME));
    }

    // Team bonus after enough fouls (threshold = 5)
    {
        apc::DisciplineSystem disc2;
        disc2.foul_bonus_threshold = 5;
        for (uint32_t i = 0; i < 5; ++i) {
            disc2.record_foul(apc::FoulType::PERSONAL_FOUL, 0, 2, 1, 3);
        }
        ASSERT_TRUE(disc2.team_in_bonus[0]);
        ASSERT_TRUE(!disc2.team_in_bonus[1]);
    }

    printf("    [PASS]\n");
}

// =============================================================================
// TEST 6: ClockSystem
// =============================================================================
static void test_clock_system() {
    printf("  [Test 6] ClockSystem...\n");

    // --- Soccer: 2x45 min ---
    {
        apc::ClockSystem clock;
        clock.configure_soccer();
        ASSERT_NEAR(clock.period_duration, 2700.0f, APC_EPSILON);
        ASSERT_NEAR(clock.total_duration,  5400.0f, APC_EPSILON);
        ASSERT_EQ_U(clock.total_periods, 2u);

        clock.start();
        ASSERT_TRUE(clock.is_running);

        // Advance 45 minutes (45 * 60 = 2700 seconds)
        for (int i = 0; i < 270; ++i) {
            clock.update(10.0f);   // 10s chunks
        }
        ASSERT_TRUE(clock.game_time >= 2700.0f);
        ASSERT_TRUE(clock.is_period_over());

        // Game not over yet (only half done)
        ASSERT_TRUE(!clock.is_game_over());

        // Finish the second half
        for (int i = 0; i < 270; ++i) {
            clock.update(10.0f);
        }
        ASSERT_TRUE(clock.game_time >= 5400.0f);
        ASSERT_TRUE(clock.is_game_over());
    }

    // --- Basketball with shot clock ---
    {
        apc::ClockSystem clock;
        clock.configure_basketball();
        ASSERT_NEAR(clock.period_duration,     720.0f, APC_EPSILON);
        ASSERT_NEAR(clock.shot_clock_duration,  24.0f, APC_EPSILON);
        ASSERT_EQ_U(clock.total_periods, 4u);

        clock.start();
        clock.reset_shot_clock();
        ASSERT_TRUE(clock.shot_clock_active);

        // Advance 20 seconds → shot clock should have 4s remaining
        clock.update(20.0f);
        ASSERT_NEAR(clock.shot_clock, 4.0f, 0.01f);

        // Advance 4 more seconds → shot clock expires
        clock.update(4.0f);
        ASSERT_NEAR(clock.shot_clock, 0.0f, APC_EPSILON);

        // Reset and count down fully
        clock.reset_shot_clock();
        ASSERT_NEAR(clock.shot_clock, 24.0f, APC_EPSILON);
        clock.update(30.0f);
        ASSERT_NEAR(clock.shot_clock, 0.0f, APC_EPSILON); // clamped at 0
    }

    printf("    [PASS]\n");
}

// =============================================================================
// TEST 7: SportPhysicsWorld — full integration (soccer)
// =============================================================================
static void test_sport_physics_world_soccer() {
    printf("  [Test 7] SportPhysicsWorld soccer integration...\n");

    apc::SportPhysicsWorld world;
    world.initialize(apc::SportConfig::make_soccer());

    // Verify ball was created
    ASSERT_EQ_U(world.ball_world.ball_count, 0u);
    uint32_t ball_id = world.create_ball(apc::BallFactory::make_soccer());
    ASSERT_TRUE(ball_id != 0xFFFFFFFFu);
    ASSERT_EQ_U(world.ball_world.ball_count, 1u);
    ASSERT_EQ_U(world.primary_ball_id, ball_id);

    // Verify field has goals (added by initialize for RECTANGLE fields)
    ASSERT_EQ_U(world.field.goal_count, 2u);

    // Start play
    world.start_play();
    ASSERT_TRUE(world.play_state == apc::PlayState::LIVE);

    // Record ball position before kick
    apc::BallState* ball = world.ball_world.get_ball(ball_id);
    ASSERT_TRUE(ball != nullptr);
    apc::Vec3 pos_before = ball->body.position;

    // Kick ball forward with high power
    apc::PassResult kick_result = world.kick(
        apc::Vec3(1.0f, 0.0f, 0.0f),   // direction
        0.9f,                            // power (high)
        0.8f                             // skill
    );
    ASSERT_TRUE(kick_result.executed);

    // Step simulation for ~0.5 seconds
    for (int i = 0; i < 120; ++i) {
        world.step(1.0f / 240.0f);
    }

    // Verify ball has moved
    apc::Vec3 pos_after = ball->body.position;
    float dist = apc::Vec3::length(apc::Vec3::sub(pos_after, pos_before));
    ASSERT_TRUE(dist > 0.1f);  // Ball should have moved significantly

    // Stop play
    world.stop_play();
    ASSERT_TRUE(world.play_state == apc::PlayState::DEAD_BALL);

    printf("    [PASS]  ball moved %.2fm in 0.5s\n", dist);
}

// =============================================================================
// TEST 8: SportPhysicsWorld — rugby tackle
// =============================================================================
static void test_sport_physics_world_rugby_tackle() {
    printf("  [Test 8] SportPhysicsWorld rugby tackle...\n");

    apc::SportPhysicsWorld world;
    world.initialize(apc::SportConfig::make_rugby());
    world.create_ball(apc::BallFactory::make_rugby());

    // Start play
    world.start_play();

    // Resolve a tackle between two positions
    apc::ContactResult result = world.tackle(
        10,                              // attacker_id
        20,                              // defender_id
        apc::Vec3(0.0f, 0.0f, 0.0f),    // attacker_pos
        apc::Vec3(0.0f, 0.0f, 5.0f),    // attacker_vel
        apc::Vec3(0.0f, 0.0f, 1.0f),    // defender_pos
        apc::Vec3(0.0f, 0.0f, 4.0f),    // defender_vel
        apc::TackleType::WRAP
    );

    ASSERT_TRUE(result.contact_made);
    ASSERT_TRUE(result.tackle_successful);

    printf("    [PASS]  contact=%s tackle=%s impact=%.1f\n",
           result.contact_made ? "yes" : "no",
           result.tackle_successful ? "yes" : "no",
           result.impact_magnitude);
}

// =============================================================================
// TEST 9: SportPhysicsFactory
// =============================================================================
static void test_sport_physics_factory() {
    printf("  [Test 9] SportPhysicsFactory...\n");

    // Basketball
    {
        auto world = apc::SportPhysicsFactory::create_basketball();
        ASSERT_TRUE(world.config.sport == apc::SportType::BASKETBALL);
        ASSERT_EQ_U(world.ball_world.ball_count, 1u);
        ASSERT_EQ_U(world.config.players_per_team, 5u);
    }

    // American Football — ball shape should be PROLATE
    {
        auto world = apc::SportPhysicsFactory::create_american_football();
        ASSERT_TRUE(world.config.sport == apc::SportType::AMERICAN_FOOTBALL);
        ASSERT_EQ_U(world.ball_world.ball_count, 1u);
        apc::BallState* ball = world.ball_world.get_ball(0);
        ASSERT_TRUE(ball != nullptr);
        ASSERT_TRUE(ball->config.shape == apc::BallShape::PROLATE);
        ASSERT_NEAR(ball->config.semi_major, 0.14f, 0.01f);
    }

    // Tennis — ball config has tennis properties
    {
        auto world = apc::SportPhysicsFactory::create_tennis();
        ASSERT_TRUE(world.config.sport == apc::SportType::TENNIS);
        apc::BallState* ball = world.ball_world.get_ball(0);
        ASSERT_TRUE(ball != nullptr);
        ASSERT_NEAR(ball->config.radius, 0.0335f, 0.001f);
        ASSERT_NEAR(ball->config.mass,  0.058f,  0.001f);
        ASSERT_TRUE(ball->config.preferred_spin == apc::SpinAxis::TOPSPIN);
        ASSERT_NEAR(ball->config.friction_coefficient, 0.55f, APC_EPSILON);
    }

    // Soccer factory
    {
        auto world = apc::SportPhysicsFactory::create_soccer();
        ASSERT_TRUE(world.config.sport == apc::SportType::SOCCER);
        ASSERT_EQ_U(world.ball_world.ball_count, 1u);
        ASSERT_EQ_U(world.field.goal_count, 2u);
    }

    // Rugby factory
    {
        auto world = apc::SportPhysicsFactory::create_rugby();
        ASSERT_TRUE(world.config.sport == apc::SportType::RUGBY_UNION);
        ASSERT_EQ_U(world.ball_world.ball_count, 1u);
    }

    printf("    [PASS]\n");
}

// =============================================================================
// TEST 10: Multiple sport types (enum coverage)
// =============================================================================
static void test_sport_type_enum() {
    printf("  [Test 10] SportType enum coverage...\n");

    // Verify all expected values exist
    ASSERT_TRUE(static_cast<uint8_t>(apc::SportType::SOCCER)            == 0);
    ASSERT_TRUE(static_cast<uint8_t>(apc::SportType::BASKETBALL)        == 1);
    ASSERT_TRUE(static_cast<uint8_t>(apc::SportType::AMERICAN_FOOTBALL) == 2);
    ASSERT_TRUE(static_cast<uint8_t>(apc::SportType::RUGBY_UNION)       == 3);
    ASSERT_TRUE(static_cast<uint8_t>(apc::SportType::RUGBY_LEAGUE)      == 4);
    ASSERT_TRUE(static_cast<uint8_t>(apc::SportType::AUSTRALIAN_RULES)  == 5);
    ASSERT_TRUE(static_cast<uint8_t>(apc::SportType::TENNIS)            == 6);
    ASSERT_TRUE(static_cast<uint8_t>(apc::SportType::VOLLEYBALL)        == 7);
    ASSERT_TRUE(static_cast<uint8_t>(apc::SportType::HANDBALL)          == 8);
    ASSERT_TRUE(static_cast<uint8_t>(apc::SportType::ICE_HOCKEY)        == 9);
    ASSERT_TRUE(static_cast<uint8_t>(apc::SportType::FIELD_HOCKEY)      == 10);
    ASSERT_TRUE(static_cast<uint8_t>(apc::SportType::CRICKET)           == 11);
    ASSERT_TRUE(static_cast<uint8_t>(apc::SportType::BASEBALL)          == 12);
    ASSERT_TRUE(static_cast<uint8_t>(apc::SportType::GOLF)              == 13);
    ASSERT_TRUE(static_cast<uint8_t>(apc::SportType::BADMINTON)         == 14);
    ASSERT_TRUE(static_cast<uint8_t>(apc::SportType::TABLE_TENNIS)      == 15);
    ASSERT_TRUE(static_cast<uint8_t>(apc::SportType::LACROSSE)          == 16);
    ASSERT_TRUE(static_cast<uint8_t>(apc::SportType::RUGBY_SEVENS)      == 17);
    ASSERT_TRUE(static_cast<uint8_t>(apc::SportType::BEACH_VOLLEYBALL)  == 18);
    ASSERT_TRUE(static_cast<uint8_t>(apc::SportType::FUTSAL)            == 19);
    ASSERT_TRUE(static_cast<uint8_t>(apc::SportType::HURLING)           == 20);
    ASSERT_TRUE(static_cast<uint8_t>(apc::SportType::GAELIC_FOOTBALL)   == 21);
    ASSERT_TRUE(static_cast<uint8_t>(apc::SportType::WATER_POLO)        == 22);
    ASSERT_TRUE(static_cast<uint8_t>(apc::SportType::BOXING)            == 23);
    ASSERT_TRUE(static_cast<uint8_t>(apc::SportType::MMA)               == 24);
    ASSERT_TRUE(static_cast<uint8_t>(apc::SportType::WRESTLING)         == 25);
    ASSERT_TRUE(static_cast<uint8_t>(apc::SportType::CUSTOM_0)          == 26);
    ASSERT_TRUE(static_cast<uint8_t>(apc::SportType::CUSTOM_1)          == 27);

    // NUM_SPORTS should be last and cover all types
    ASSERT_TRUE(static_cast<uint8_t>(apc::SportType::NUM_SPORTS) == 28);

    // Verify FieldType enum coverage
    ASSERT_TRUE(static_cast<uint8_t>(apc::FieldType::RECTANGLE)  == 0);
    ASSERT_TRUE(static_cast<uint8_t>(apc::FieldType::OVAL)       == 1);
    ASSERT_TRUE(static_cast<uint8_t>(apc::FieldType::DIAMOND)    == 2);
    ASSERT_TRUE(static_cast<uint8_t>(apc::FieldType::CIRCLE)     == 3);
    ASSERT_TRUE(static_cast<uint8_t>(apc::FieldType::RINK)       == 4);
    ASSERT_TRUE(static_cast<uint8_t>(apc::FieldType::RING)       == 5);
    ASSERT_TRUE(static_cast<uint8_t>(apc::FieldType::COURSE)     == 6);
    ASSERT_TRUE(static_cast<uint8_t>(apc::FieldType::MAT)        == 7);

    // Verify PlayState enum coverage
    ASSERT_TRUE(static_cast<uint8_t>(apc::PlayState::NOT_STARTED) == 0);
    ASSERT_TRUE(static_cast<uint8_t>(apc::PlayState::LIVE)        == 2);
    ASSERT_TRUE(static_cast<uint8_t>(apc::PlayState::DEAD_BALL)   == 3);
    ASSERT_TRUE(static_cast<uint8_t>(apc::PlayState::GOAL_SCORED) == 4);
    ASSERT_TRUE(static_cast<uint8_t>(apc::PlayState::GAME_OVER)   == 7);

    printf("    [PASS]\n");
}

// =============================================================================
// Main
// =============================================================================
int main() {
    printf("Sprint 16: Sport Environments & Integration tests\n");
    printf("================================================\n");

    test_field_geometry_factories();      // Test 1
    test_sport_field();                  // Test 2
    test_goal_post();                    // Test 3
    test_scoring_system();               // Test 4
    test_discipline_system();            // Test 5
    test_clock_system();                 // Test 6
    test_sport_physics_world_soccer();   // Test 7
    test_sport_physics_world_rugby_tackle(); // Test 8
    test_sport_physics_factory();        // Test 9
    test_sport_type_enum();              // Test 10

    if (g_fail == 0) {
        printf("\nSprint 16: ALL TESTS PASSED\n");
        return 0;
    } else {
        printf("\nSprint 16: SOME TESTS FAILED\n");
        return 1;
    }
}
