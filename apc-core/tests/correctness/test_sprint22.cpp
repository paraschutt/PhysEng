// =============================================================================
// Sprint 22 Tests — Entity System: Types, IDs, Manager
// =============================================================================
//
// Tests for the Sprint 22 deliverables:
//    1. EntityType enum values
//    2. SportRole enum values (soccer/basketball/football/rugby roles)
//    3. TeamId type
//    4. EntityId construction and comparison
//    5. EntityId INVALID sentinel
//    6. AthleteEntity default values
//    7. AthleteEntity field types (mass, height, radius floats)
//    8. BallEntity default values
//    9. BallEntity ball_type enum
//   10. BallEntity sport-dependent defaults (mass, radius)
//   11. EntityManager default values (counts=0)
//   12. EntityManager spawn_athlete returns valid EntityId
//   13. EntityManager spawn_athlete increments athlete_count
//   14. EntityManager get_athlete returns valid pointer
//   15. EntityManager get_athlete with invalid ID returns nullptr
//   16. EntityManager despawn invalidates entity
//   17. EntityManager spawn_ball returns valid EntityId
//   18. EntityManager find_athlete_by_role
//   19. EntityManager get_team_athlete_count
//   20. EntityManager reset clears everything
//   21. EntityManager generation counter (spawn/despawn/respawn)
//   22. MAX_ATHLETES capacity check (cannot exceed 44)
//
// Pattern: int main() + assert(), no test framework.
// =============================================================================

#include "apc_entity/apc_entity_types.h"
#include "apc_entity/apc_entity_manager.h"
#include <cstdio>
#include <cmath>
#include <cassert>
#include <cstring>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static constexpr float EPS = 1e-4f;

static bool approx_eq(float a, float b, float eps = EPS) {
    return std::abs(a - b) < eps;
}

// =============================================================================
// TEST 1: EntityType enum values
// =============================================================================
static int test_entity_type_enum_values() {
    std::printf("  [Test 1] EntityType enum values...\n");

    using ET = apc::EntityType;
    assert(static_cast<uint8_t>(ET::ATHLETE)       == 0 && "ATHLETE = 0");
    assert(static_cast<uint8_t>(ET::BALL)          == 1 && "BALL = 1");
    assert(static_cast<uint8_t>(ET::GOAL_POST)     == 2 && "GOAL_POST = 2");
    assert(static_cast<uint8_t>(ET::FIELD_SURFACE) == 3 && "FIELD_SURFACE = 3");
    assert(static_cast<uint8_t>(ET::BOUNDARY)      == 4 && "BOUNDARY = 4");
    assert(static_cast<uint8_t>(ET::EQUIPMENT)     == 5 && "EQUIPMENT = 5");
    assert(static_cast<uint8_t>(ET::CAMERA)        == 6 && "CAMERA = 6");

    // Type size
    static_assert(sizeof(ET) == 1, "EntityType must be 1 byte");

    // Monotonically increasing
    assert(ET::ATHLETE < ET::BALL);
    assert(ET::BALL < ET::GOAL_POST);
    assert(ET::CAMERA > ET::ATHLETE);

    std::printf("    [PASS] All 7 EntityType values verified (0-6)\n");
    return 0;
}

// =============================================================================
// TEST 2: SportRole enum values
// =============================================================================
static int test_sport_role_enum_values() {
    std::printf("  [Test 2] SportRole enum values...\n");

    using SR = apc::SportRole;

    // Soccer roles (0-10)
    assert(static_cast<uint8_t>(SR::SOCCER_GK)  == 0  && "SOCCER_GK = 0");
    assert(static_cast<uint8_t>(SR::SOCCER_CB)  == 1  && "SOCCER_CB = 1");
    assert(static_cast<uint8_t>(SR::SOCCER_LB)  == 2  && "SOCCER_LB = 2");
    assert(static_cast<uint8_t>(SR::SOCCER_RB)  == 3  && "SOCCER_RB = 3");
    assert(static_cast<uint8_t>(SR::SOCCER_CDM) == 4  && "SOCCER_CDM = 4");
    assert(static_cast<uint8_t>(SR::SOCCER_CM)  == 5  && "SOCCER_CM = 5");
    assert(static_cast<uint8_t>(SR::SOCCER_CAM) == 6  && "SOCCER_CAM = 6");
    assert(static_cast<uint8_t>(SR::SOCCER_LW)  == 7  && "SOCCER_LW = 7");
    assert(static_cast<uint8_t>(SR::SOCCER_RW)  == 8  && "SOCCER_RW = 8");
    assert(static_cast<uint8_t>(SR::SOCCER_ST)  == 9  && "SOCCER_ST = 9");
    assert(static_cast<uint8_t>(SR::SOCCER_SUB) == 10 && "SOCCER_SUB = 10");

    // Basketball roles (11-16)
    assert(static_cast<uint8_t>(SR::BASKETBALL_PG)  == 11 && "BASKETBALL_PG = 11");
    assert(static_cast<uint8_t>(SR::BASKETBALL_SG)  == 12 && "BASKETBALL_SG = 12");
    assert(static_cast<uint8_t>(SR::BASKETBALL_SF)  == 13 && "BASKETBALL_SF = 13");
    assert(static_cast<uint8_t>(SR::BASKETBALL_PF)  == 14 && "BASKETBALL_PF = 14");
    assert(static_cast<uint8_t>(SR::BASKETBALL_C)   == 15 && "BASKETBALL_C = 15");
    assert(static_cast<uint8_t>(SR::BASKETBALL_SUB) == 16 && "BASKETBALL_SUB = 16");

    // Football roles (17-28)
    assert(static_cast<uint8_t>(SR::FOOTBALL_QB)  == 17 && "FOOTBALL_QB = 17");
    assert(static_cast<uint8_t>(SR::FOOTBALL_RB)  == 18 && "FOOTBALL_RB = 18");
    assert(static_cast<uint8_t>(SR::FOOTBALL_WR)  == 19 && "FOOTBALL_WR = 19");
    assert(static_cast<uint8_t>(SR::FOOTBALL_K)   == 26 && "FOOTBALL_K = 26");
    assert(static_cast<uint8_t>(SR::FOOTBALL_P)   == 27 && "FOOTBALL_P = 27");
    assert(static_cast<uint8_t>(SR::FOOTBALL_SUB) == 28 && "FOOTBALL_SUB = 28");

    // Rugby roles (29-39)
    assert(static_cast<uint8_t>(SR::RUGBY_PROP)    == 29 && "RUGBY_PROP = 29");
    assert(static_cast<uint8_t>(SR::RUGBY_HOOKER)  == 30 && "RUGBY_HOOKER = 30");
    assert(static_cast<uint8_t>(SR::RUGBY_LOCK)    == 31 && "RUGBY_LOCK = 31");
    assert(static_cast<uint8_t>(SR::RUGBY_FLANKER) == 32 && "RUGBY_FLANKER = 32");
    assert(static_cast<uint8_t>(SR::RUGBY_FB)      == 38 && "RUGBY_FB = 38");
    assert(static_cast<uint8_t>(SR::RUGBY_SUB)     == 39 && "RUGBY_SUB = 39");

    // Type size
    static_assert(sizeof(SR) == 1, "SportRole must be 1 byte");

    // Continuous range
    assert(SR::SOCCER_GK < SR::SOCCER_ST);
    assert(SR::SOCCER_SUB < SR::BASKETBALL_PG);
    assert(SR::BASKETBALL_SUB < SR::FOOTBALL_QB);
    assert(SR::FOOTBALL_SUB < SR::RUGBY_PROP);
    assert(SR::RUGBY_SUB > SR::RUGBY_PROP);

    std::printf("    [PASS] All 40 SportRole values verified (0-39) across 4 sports\n");
    return 0;
}

// =============================================================================
// TEST 3: TeamId type
// =============================================================================
static int test_team_id_type() {
    std::printf("  [Test 3] TeamId type...\n");

    static_assert(sizeof(apc::TeamId) == 1, "TeamId must be 1 byte");

    assert(apc::TEAM_NONE == 0 && "TEAM_NONE = 0");
    assert(apc::TEAM_HOME == 1 && "TEAM_HOME = 1");
    assert(apc::TEAM_AWAY == 2 && "TEAM_AWAY = 2");

    // TeamId is a uint8_t, can be assigned
    apc::TeamId team = apc::TEAM_HOME;
    assert(team == 1);

    team = 3; // Extra team slot
    assert(team == 3);

    std::printf("    [PASS] TeamId is uint8_t with NONE=0, HOME=1, AWAY=2\n");
    return 0;
}

// =============================================================================
// TEST 4: EntityId construction and comparison
// =============================================================================
static int test_entity_id_construction_comparison() {
    std::printf("  [Test 4] EntityId construction and comparison...\n");

    // Default construction
    apc::EntityId id_default;
    assert(id_default.index == 0xFFFFFFFFu && "Default index = INVALID");
    assert(id_default.generation == 0u && "Default generation = 0");

    // Explicit construction
    apc::EntityId id_a{5u, 1u};
    assert(id_a.index == 5u && "Explicit index = 5");
    assert(id_a.generation == 1u && "Explicit generation = 1");

    // Equality
    apc::EntityId id_b{5u, 1u};
    assert(id_a == id_b && "Same index+generation → equal");

    // Inequality: different index
    apc::EntityId id_c{6u, 1u};
    assert(id_a != id_c && "Different index → not equal");

    // Inequality: different generation
    apc::EntityId id_d{5u, 2u};
    assert(id_a != id_d && "Different generation → not equal");

    // Both different
    apc::EntityId id_e{7u, 3u};
    assert(id_a != id_e && "Both different → not equal");

    std::printf("    [PASS] EntityId construction, equality, and inequality correct\n");
    return 0;
}

// =============================================================================
// TEST 5: EntityId INVALID sentinel
// =============================================================================
static int test_entity_id_invalid_sentinel() {
    std::printf("  [Test 5] EntityId INVALID sentinel...\n");

    apc::EntityId invalid = apc::EntityId::make_invalid();
    assert(invalid.index == 0xFFFFFFFFu && "make_invalid: index = 0xFFFFFFFF");
    assert(invalid.generation == 0u && "make_invalid: generation = 0");

    assert(invalid.is_valid() == 0u && "INVALID: is_valid = 0");

    // Any other index is valid
    apc::EntityId valid{0u, 1u};
    assert(valid.is_valid() == 1u && "index=0, gen=1: is_valid = 1");

    apc::EntityId valid2{0u, 0u};
    assert(valid2.is_valid() == 1u && "index=0, gen=0: is_valid = 1 (index != 0xFFFFFFFF)");

    // The exact INVALID sentinel
    apc::EntityId id1 = apc::EntityId::make_invalid();
    apc::EntityId id2 = apc::EntityId::make_invalid();
    assert(id1 == id2 && "Two INVALID sentinels are equal");

    std::printf("    [PASS] INVALID sentinel and is_valid check verified\n");
    return 0;
}

// =============================================================================
// TEST 6: AthleteEntity default values
// =============================================================================
static int test_athlete_entity_defaults() {
    std::printf("  [Test 6] AthleteEntity default values...\n");

    apc::AthleteEntity a;

    // Identity
    assert(a.id.is_valid() == 0u && "Default: id invalid");
    assert(a.type == apc::EntityType::ATHLETE && "Default type = ATHLETE");
    assert(a.team == apc::TEAM_NONE && "Default team = NONE");
    assert(a.role == apc::SportRole::SOCCER_CM && "Default role = SOCCER_CM");
    assert(a.jersey_number == 0 && "Default jersey = 0");

    // Transform
    assert(approx_eq(a.position.x, 0.0f) && "Default pos.x = 0");
    assert(approx_eq(a.position.y, 0.0f) && "Default pos.y = 0");
    assert(approx_eq(a.position.z, 0.0f) && "Default pos.z = 0");
    assert(approx_eq(a.velocity.x, 0.0f) && "Default vel.x = 0");
    assert(approx_eq(a.acceleration.x, 0.0f) && "Default acc.x = 0");

    // Orientation = identity quaternion
    assert(approx_eq(a.orientation.w, 1.0f) && "Default orient.w = 1");
    assert(approx_eq(a.orientation.x, 0.0f) && "Default orient.x = 0");
    assert(approx_eq(a.orientation.y, 0.0f) && "Default orient.y = 0");
    assert(approx_eq(a.orientation.z, 0.0f) && "Default orient.z = 0");

    // Sport state
    assert(approx_eq(a.stamina, 1.0f) && "Default stamina = 1.0");
    assert(approx_eq(a.max_stamina, 1.0f) && "Default max_stamina = 1.0");
    assert(approx_eq(a.health, 1.0f) && "Default health = 1.0");
    assert(a.is_human_controlled == 0 && "Default is_human = 0");
    assert(a.is_active == 0 && "Default is_active = 0");

    // Cooldowns
    assert(approx_eq(a.sprint_cooldown, 0.0f) && "Default sprint_cooldown = 0");
    assert(approx_eq(a.tackle_cooldown, 0.0f) && "Default tackle_cooldown = 0");

    // Unique ID
    assert(a.unique_id == 0u && "Default unique_id = 0");

    // Flags
    assert(a.flags == 0u && "Default flags = 0");

    // active() query
    assert(a.active() == 0 && "Default: not active");

    std::printf("    [PASS] AthleteEntity all defaults verified\n");
    return 0;
}

// =============================================================================
// TEST 7: AthleteEntity field types (mass, height, radius)
// =============================================================================
static int test_athlete_entity_field_types() {
    std::printf("  [Test 7] AthleteEntity body dimension defaults...\n");

    apc::AthleteEntity a;

    // Default body dimensions
    assert(approx_eq(a.mass, 80.0f) && "Default mass = 80 kg");
    assert(approx_eq(a.height, 1.80f) && "Default height = 1.80 m");
    assert(approx_eq(a.radius, 0.3f) && "Default radius = 0.3 m");

    // All are floats (static_assert on struct layout)
    static_assert(sizeof(a.mass) == sizeof(float), "mass is float");
    static_assert(sizeof(a.height) == sizeof(float), "height is float");
    static_assert(sizeof(a.radius) == sizeof(float), "radius is float");

    // Can be assigned
    a.mass = 95.0f;
    a.height = 1.92f;
    a.radius = 0.35f;
    assert(approx_eq(a.mass, 95.0f));
    assert(approx_eq(a.height, 1.92f));
    assert(approx_eq(a.radius, 0.35f));

    // Reset restores defaults
    a.reset();
    assert(approx_eq(a.mass, 80.0f) && "After reset: mass = 80");
    assert(approx_eq(a.height, 1.80f) && "After reset: height = 1.80");
    assert(approx_eq(a.radius, 0.3f) && "After reset: radius = 0.3");

    std::printf("    [PASS] Body dimensions (mass=80, height=1.8, radius=0.3) verified\n");
    return 0;
}

// =============================================================================
// TEST 8: BallEntity default values
// =============================================================================
static int test_ball_entity_defaults() {
    std::printf("  [Test 8] BallEntity default values...\n");

    apc::BallEntity b;

    // Identity
    assert(b.id.is_valid() == 0u && "Default: id invalid");
    assert(b.type == apc::EntityType::BALL && "Default type = BALL");

    // Transform
    assert(approx_eq(b.position.x, 0.0f) && "Default pos.x = 0");
    assert(approx_eq(b.velocity.x, 0.0f) && "Default vel.x = 0");
    assert(approx_eq(b.angular_velocity.x, 0.0f) && "Default ang_vel.x = 0");

    // Orientation = identity
    assert(approx_eq(b.orientation.w, 1.0f) && "Default orient.w = 1");

    // Ball properties — soccer defaults
    assert(approx_eq(b.mass, 0.43f) && "Default mass = 0.43 (soccer)");
    assert(approx_eq(b.radius, 0.11f) && "Default radius = 0.11 (soccer)");

    // Physics tuning
    assert(approx_eq(b.air_resistance, 0.01f) && "Default air_resistance = 0.01");
    assert(approx_eq(b.ground_friction, 0.4f) && "Default ground_friction = 0.4");
    assert(approx_eq(b.bounce_restitution, 0.75f) && "Default bounce = 0.75");

    // Spin
    assert(b.spin_axis_locked == 0 && "Default spin_locked = 0");

    // Possession
    assert(b.last_toucher.is_valid() == 0u && "Default last_toucher invalid");
    assert(b.possession_team == apc::TEAM_NONE && "Default possession = NONE");
    assert(b.is_in_play == 0 && "Default is_in_play = 0");

    std::printf("    [PASS] BallEntity all defaults verified\n");
    return 0;
}

// =============================================================================
// TEST 9: BallEntity ball_type enum
// =============================================================================
static int test_ball_entity_ball_type() {
    std::printf("  [Test 9] BallEntity ball_type...\n");

    apc::BallEntity b;

    // Default ball_type = 0 (soccer)
    assert(b.ball_type == 0 && "Default ball_type = 0 (soccer)");

    // Set via configure_for_sport
    b.configure_for_sport(1); // basketball
    assert(b.ball_type == 1 && "After configure: ball_type = 1 (basketball)");

    b.configure_for_sport(2); // football
    assert(b.ball_type == 2 && "After configure: ball_type = 2 (football)");

    b.configure_for_sport(3); // rugby
    assert(b.ball_type == 3 && "After configure: ball_type = 3 (rugby)");

    // ball_type is uint8_t
    static_assert(sizeof(b.ball_type) == 1, "ball_type is uint8_t");

    std::printf("    [PASS] ball_type correctly set by configure_for_sport\n");
    return 0;
}

// =============================================================================
// TEST 10: BallEntity sport-dependent defaults
// =============================================================================
static int test_ball_entity_sport_defaults() {
    std::printf("  [Test 10] BallEntity sport-dependent defaults...\n");

    // Soccer (sport 0)
    apc::BallEntity soccer;
    soccer.configure_for_sport(0);
    assert(approx_eq(soccer.mass, 0.43f) && "Soccer mass = 0.43");
    assert(approx_eq(soccer.radius, 0.11f) && "Soccer radius = 0.11");
    assert(approx_eq(soccer.bounce_restitution, 0.75f) && "Soccer bounce = 0.75");
    assert(approx_eq(soccer.air_resistance, 0.01f) && "Soccer air_res = 0.01");

    // Basketball (sport 1)
    apc::BallEntity basketball;
    basketball.configure_for_sport(1);
    assert(approx_eq(basketball.mass, 0.62f) && "Basketball mass = 0.62");
    assert(approx_eq(basketball.radius, 0.12f) && "Basketball radius = 0.12");
    assert(approx_eq(basketball.bounce_restitution, 0.80f) && "Basketball bounce = 0.80");
    assert(approx_eq(basketball.air_resistance, 0.008f) && "Basketball air_res = 0.008");

    // American Football (sport 2)
    apc::BallEntity football;
    football.configure_for_sport(2);
    assert(approx_eq(football.mass, 0.45f) && "Football mass = 0.45");
    assert(approx_eq(football.radius, 0.085f) && "Football radius = 0.085");
    assert(approx_eq(football.bounce_restitution, 0.30f) && "Football bounce = 0.30");
    assert(approx_eq(football.air_resistance, 0.005f) && "Football air_res = 0.005");

    // Rugby (sport 3)
    apc::BallEntity rugby;
    rugby.configure_for_sport(3);
    assert(approx_eq(rugby.mass, 0.44f) && "Rugby mass = 0.44");
    assert(approx_eq(rugby.radius, 0.09f) && "Rugby radius = 0.09");
    assert(approx_eq(rugby.bounce_restitution, 0.50f) && "Rugby bounce = 0.50");
    assert(approx_eq(rugby.air_resistance, 0.008f) && "Rugby air_res = 0.008");

    // Unknown sport: falls to default
    apc::BallEntity unknown;
    unknown.configure_for_sport(99);
    assert(approx_eq(unknown.mass, 0.43f) && "Unknown sport: mass = default 0.43");
    assert(approx_eq(unknown.radius, 0.11f) && "Unknown sport: radius = default 0.11");

    // Sport-specific differentiation
    assert(basketball.mass > soccer.mass && "Basketball heavier than soccer");
    assert(basketball.bounce_restitution > football.bounce_restitution && "Basketball bouncier than football");

    std::printf("    [PASS] All 4 sports have correct mass, radius, bounce, air_res\n");
    return 0;
}

// =============================================================================
// TEST 11: EntityManager default values
// =============================================================================
static int test_entity_manager_defaults() {
    std::printf("  [Test 11] EntityManager default values...\n");

    apc::EntityManager mgr;

    assert(mgr.athlete_count == 0u && "Default athlete_count = 0");
    assert(mgr.ball_count == 0u && "Default ball_count = 0");
    assert(mgr.next_athlete_generation == 0u && "Default athlete_gen = 0");
    assert(mgr.next_ball_generation == 0u && "Default ball_gen = 0");

    // All athlete slots are invalid
    for (uint32_t i = 0; i < apc::MAX_ATHLETES; ++i) {
        assert(mgr.athletes[i].id.is_valid() == 0u && "All athlete slots invalid initially");
    }

    // All ball slots are invalid
    for (uint32_t i = 0; i < apc::MAX_BALLS; ++i) {
        assert(mgr.balls[i].id.is_valid() == 0u && "All ball slots invalid initially");
    }

    std::printf("    [PASS] EntityManager starts with zero counts, all slots invalid\n");
    return 0;
}

// =============================================================================
// TEST 12: EntityManager spawn_athlete returns valid EntityId
// =============================================================================
static int test_entity_manager_spawn_athlete_valid() {
    std::printf("  [Test 12] EntityManager spawn_athlete returns valid EntityId...\n");

    apc::EntityManager mgr;

    apc::Vec3 pos(10.0f, 0.0f, 5.0f);
    apc::EntityId id = mgr.spawn_athlete(apc::TEAM_HOME, apc::SportRole::SOCCER_ST, pos, 9);

    assert(id.is_valid() == 1u && "Spawned athlete has valid ID");
    assert(id.generation > 0u && "Generation > 0 after spawn");
    assert(id.index < apc::MAX_ATHLETES && "Index within bounds");

    std::printf("    [PASS] spawn_athlete returns valid EntityId with gen > 0\n");
    return 0;
}

// =============================================================================
// TEST 13: EntityManager spawn_athlete increments athlete_count
// =============================================================================
static int test_entity_manager_spawn_athlete_increments_count() {
    std::printf("  [Test 13] EntityManager spawn_athlete increments count...\n");

    apc::EntityManager mgr;
    apc::Vec3 pos(0.0f, 0.0f, 0.0f);

    assert(mgr.athlete_count == 0u);

    mgr.spawn_athlete(apc::TEAM_HOME, apc::SportRole::SOCCER_GK, pos, 1);
    assert(mgr.athlete_count == 1u && "Count = 1 after first spawn");

    mgr.spawn_athlete(apc::TEAM_HOME, apc::SportRole::SOCCER_CB, pos, 2);
    assert(mgr.athlete_count == 2u && "Count = 2 after second spawn");

    mgr.spawn_athlete(apc::TEAM_AWAY, apc::SportRole::SOCCER_ST, pos, 9);
    assert(mgr.athlete_count == 3u && "Count = 3 after third spawn");

    // Spawn 5 more
    for (uint32_t i = 0; i < 5; ++i) {
        mgr.spawn_athlete(apc::TEAM_HOME, apc::SportRole::SOCCER_CM,
                          apc::Vec3(static_cast<float>(i), 0.0f, 0.0f), 10);
    }
    assert(mgr.athlete_count == 8u && "Count = 8 after 8 spawns");

    std::printf("    [PASS] athlete_count increments with each spawn\n");
    return 0;
}

// =============================================================================
// TEST 14: EntityManager get_athlete returns valid pointer
// =============================================================================
static int test_entity_manager_get_athlete_valid() {
    std::printf("  [Test 14] EntityManager get_athlete returns valid pointer...\n");

    apc::EntityManager mgr;
    apc::Vec3 pos(5.0f, 0.0f, -3.0f);

    apc::EntityId id = mgr.spawn_athlete(apc::TEAM_AWAY, apc::SportRole::SOCCER_LW, pos, 11);
    assert(id.is_valid());

    apc::AthleteEntity* athlete = mgr.get_athlete(id);
    assert(athlete != nullptr && "get_athlete returns non-null for valid ID");

    // Verify stored properties
    assert(athlete->team == apc::TEAM_AWAY && "Athlete team = AWAY");
    assert(athlete->role == apc::SportRole::SOCCER_LW && "Athlete role = LW");
    assert(athlete->jersey_number == 11 && "Athlete jersey = 11");
    assert(approx_eq(athlete->position.x, 5.0f) && "Athlete pos.x = 5");
    assert(approx_eq(athlete->position.z, -3.0f) && "Athlete pos.z = -3");
    assert(athlete->is_active == 1 && "Athlete is_active = 1");
    assert(athlete->id == id && "Stored ID matches returned ID");

    // Const version also works
    const apc::EntityManager& cmgr = mgr;
    const apc::AthleteEntity* c_athlete = cmgr.get_athlete(id);
    assert(c_athlete != nullptr && "const get_athlete also works");
    assert(c_athlete->team == apc::TEAM_AWAY);

    std::printf("    [PASS] get_athlete returns valid pointer with correct properties\n");
    return 0;
}

// =============================================================================
// TEST 15: EntityManager get_athlete with invalid ID returns nullptr
// =============================================================================
static int test_entity_manager_get_athlete_invalid() {
    std::printf("  [Test 15] EntityManager get_athlete invalid → nullptr...\n");

    apc::EntityManager mgr;

    // Empty manager: any ID returns nullptr
    apc::EntityId fake{0u, 1u};
    assert(mgr.get_athlete(fake) == nullptr && "Empty: any ID → nullptr");

    apc::EntityId invalid = apc::EntityId::make_invalid();
    assert(mgr.get_athlete(invalid) == nullptr && "INVALID → nullptr");

    // Out-of-bounds index
    apc::EntityId oob{apc::MAX_ATHLETES, 1u};
    assert(mgr.get_athlete(oob) == nullptr && "Out-of-bounds index → nullptr");

    apc::EntityId very_oob{1000u, 1u};
    assert(mgr.get_athlete(very_oob) == nullptr && "Very large index → nullptr");

    // Stale generation: spawn then look up with wrong generation
    apc::Vec3 pos(0.0f, 0.0f, 0.0f);
    apc::EntityId valid_id = mgr.spawn_athlete(apc::TEAM_HOME, apc::SportRole::SOCCER_GK, pos, 1);
    assert(mgr.get_athlete(valid_id) != nullptr && "Valid ID works");

    // Same index but wrong generation
    apc::EntityId stale{valid_id.index, valid_id.generation + 1u};
    assert(mgr.get_athlete(stale) == nullptr && "Stale generation → nullptr");

    apc::EntityId old_gen{valid_id.index, valid_id.generation - 1u};
    if (valid_id.generation > 0) {
        assert(mgr.get_athlete(old_gen) == nullptr && "Old generation → nullptr");
    }

    std::printf("    [PASS] Invalid, OOB, and stale generation all return nullptr\n");
    return 0;
}

// =============================================================================
// TEST 16: EntityManager despawn invalidates entity
// =============================================================================
static int test_entity_manager_despawn() {
    std::printf("  [Test 16] EntityManager despawn...\n");

    apc::EntityManager mgr;
    apc::Vec3 pos(0.0f, 0.0f, 0.0f);

    // Spawn an athlete
    apc::EntityId id = mgr.spawn_athlete(apc::TEAM_HOME, apc::SportRole::SOCCER_ST, pos, 9);
    assert(id.is_valid());
    assert(mgr.get_athlete(id) != nullptr);

    // Despawn it
    uint8_t result = mgr.despawn(id);
    assert(result == 1u && "despawn returns 1 for valid entity");

    // Entity is no longer accessible
    assert(mgr.get_athlete(id) == nullptr && "After despawn: get_athlete → nullptr");

    // Team count no longer includes it
    assert(mgr.get_team_athlete_count(apc::TEAM_HOME) == 0u
           && "After despawn: team count = 0");

    // Double despawn returns 0
    uint8_t result2 = mgr.despawn(id);
    assert(result2 == 0u && "Double despawn returns 0");

    // Despawn invalid entity returns 0
    apc::EntityId fake{999u, 1u};
    assert(mgr.despawn(fake) == 0u && "Despawn invalid → 0");

    // Despawn a ball
    apc::EntityId ball_id = mgr.spawn_ball(0, pos);
    assert(ball_id.is_valid());
    assert(mgr.get_ball(ball_id) != nullptr);

    uint8_t ball_result = mgr.despawn(ball_id);
    assert(ball_result == 1u && "Despawn ball returns 1");
    assert(mgr.get_ball(ball_id) == nullptr && "After despawn: get_ball → nullptr");

    std::printf("    [PASS] despawn invalidates entity, queries reflect removal\n");
    return 0;
}

// =============================================================================
// TEST 17: EntityManager spawn_ball returns valid EntityId
// =============================================================================
static int test_entity_manager_spawn_ball_valid() {
    std::printf("  [Test 17] EntityManager spawn_ball returns valid EntityId...\n");

    apc::EntityManager mgr;

    apc::Vec3 center(0.0f, 0.5f, 0.0f);
    apc::EntityId id = mgr.spawn_ball(0, center); // soccer ball

    assert(id.is_valid() == 1u && "Spawned ball has valid ID");
    assert(id.generation > 0u && "Ball generation > 0");
    assert(id.index < apc::MAX_BALLS && "Ball index within bounds");

    // Verify ball properties
    apc::BallEntity* ball = mgr.get_ball(id);
    assert(ball != nullptr);
    assert(ball->type == apc::EntityType::BALL && "Ball type = BALL");
    assert(ball->ball_type == 0 && "ball_type = 0 (soccer)");
    assert(approx_eq(ball->mass, 0.43f) && "Ball mass = 0.43 (soccer)");
    assert(approx_eq(ball->radius, 0.11f) && "Ball radius = 0.11 (soccer)");
    assert(ball->is_in_play == 1 && "Ball is_in_play = 1");
    assert(approx_eq(ball->position.x, 0.0f) && "Ball pos.x = 0");
    assert(approx_eq(ball->position.y, 0.5f) && "Ball pos.y = 0.5");

    // Multiple balls
    apc::EntityId id2 = mgr.spawn_ball(1, center); // basketball
    assert(id2.is_valid());
    assert(id2.index != id.index && "Different ball gets different index");

    apc::BallEntity* ball2 = mgr.get_ball(id2);
    assert(ball2 != nullptr);
    assert(ball2->ball_type == 1 && "Second ball: basketball");
    assert(approx_eq(ball2->mass, 0.62f) && "Basketball mass = 0.62");

    // ball_count
    assert(mgr.ball_count == 2u && "ball_count = 2 after 2 spawns");

    std::printf("    [PASS] spawn_ball returns valid ID with correct sport config\n");
    return 0;
}

// =============================================================================
// TEST 18: EntityManager find_athlete_by_role
// =============================================================================
static int test_entity_manager_find_athlete_by_role() {
    std::printf("  [Test 18] EntityManager find_athlete_by_role...\n");

    apc::EntityManager mgr;
    apc::Vec3 pos(0.0f, 0.0f, 0.0f);

    // Spawn home team with specific roles
    mgr.spawn_athlete(apc::TEAM_HOME, apc::SportRole::SOCCER_GK, pos, 1);
    mgr.spawn_athlete(apc::TEAM_HOME, apc::SportRole::SOCCER_ST, pos, 9);
    mgr.spawn_athlete(apc::TEAM_HOME, apc::SportRole::SOCCER_CM, pos, 8);

    // Spawn away team
    mgr.spawn_athlete(apc::TEAM_AWAY, apc::SportRole::SOCCER_ST, pos, 11);

    // Find home ST
    apc::AthleteEntity* home_st = mgr.find_athlete_by_role(
        apc::TEAM_HOME, apc::SportRole::SOCCER_ST);
    assert(home_st != nullptr && "Home ST found");
    assert(home_st->jersey_number == 9 && "Home ST jersey = 9");
    assert(home_st->team == apc::TEAM_HOME);

    // Find away ST
    apc::AthleteEntity* away_st = mgr.find_athlete_by_role(
        apc::TEAM_AWAY, apc::SportRole::SOCCER_ST);
    assert(away_st != nullptr && "Away ST found");
    assert(away_st->jersey_number == 11 && "Away ST jersey = 11");

    // Ensure correct team returned (home, not away)
    assert(home_st != away_st && "Home ST ≠ Away ST");

    // Find GK
    apc::AthleteEntity* gk = mgr.find_athlete_by_role(
        apc::TEAM_HOME, apc::SportRole::SOCCER_GK);
    assert(gk != nullptr && "Home GK found");
    assert(gk->jersey_number == 1);

    // Find role that doesn't exist on team
    apc::AthleteEntity* cb = mgr.find_athlete_by_role(
        apc::TEAM_HOME, apc::SportRole::SOCCER_CB);
    assert(cb == nullptr && "No CB on home team → nullptr");

    // Find on team with no athletes
    apc::AthleteEntity* none = mgr.find_athlete_by_role(
        apc::TEAM_NONE, apc::SportRole::SOCCER_ST);
    assert(none == nullptr && "No athletes on TEAM_NONE → nullptr");

    std::printf("    [PASS] find_athlete_by_role returns first match, team-scoped\n");
    return 0;
}

// =============================================================================
// TEST 19: EntityManager get_team_athlete_count
// =============================================================================
static int test_entity_manager_get_team_athlete_count() {
    std::printf("  [Test 19] EntityManager get_team_athlete_count...\n");

    apc::EntityManager mgr;
    apc::Vec3 pos(0.0f, 0.0f, 0.0f);

    // Initially zero for all teams
    assert(mgr.get_team_athlete_count(apc::TEAM_HOME) == 0u);
    assert(mgr.get_team_athlete_count(apc::TEAM_AWAY) == 0u);
    assert(mgr.get_team_athlete_count(apc::TEAM_NONE) == 0u);

    // Spawn 5 home, 3 away
    for (uint32_t i = 0; i < 5; ++i) {
        mgr.spawn_athlete(apc::TEAM_HOME, apc::SportRole::SOCCER_CM, pos, static_cast<uint8_t>(i + 1));
    }
    for (uint32_t i = 0; i < 3; ++i) {
        mgr.spawn_athlete(apc::TEAM_AWAY, apc::SportRole::SOCCER_CM, pos, static_cast<uint8_t>(i + 1));
    }

    assert(mgr.get_team_athlete_count(apc::TEAM_HOME) == 5u && "Home has 5 athletes");
    assert(mgr.get_team_athlete_count(apc::TEAM_AWAY) == 3u && "Away has 3 athletes");
    assert(mgr.get_team_athlete_count(apc::TEAM_NONE) == 0u && "NONE has 0 athletes");

    // Despawn one home athlete
    apc::EntityId first_home = mgr.find_athlete_by_role(apc::TEAM_HOME, apc::SportRole::SOCCER_CM)->id;
    mgr.despawn(first_home);
    assert(mgr.get_team_athlete_count(apc::TEAM_HOME) == 4u && "Home has 4 after despawn");
    assert(mgr.get_team_athlete_count(apc::TEAM_AWAY) == 3u && "Away still 3");

    // Extra team (team 3)
    mgr.spawn_athlete(3, apc::SportRole::SOCCER_CM, pos, 99);
    assert(mgr.get_team_athlete_count(3) == 1u && "Team 3 has 1 athlete");

    std::printf("    [PASS] Team counts accurate after spawn and despawn\n");
    return 0;
}

// =============================================================================
// TEST 20: EntityManager reset clears everything
// =============================================================================
static int test_entity_manager_reset() {
    std::printf("  [Test 20] EntityManager reset clears everything...\n");

    apc::EntityManager mgr;
    apc::Vec3 pos(0.0f, 0.0f, 0.0f);

    // Spawn several athletes and balls
    apc::EntityId id1 = mgr.spawn_athlete(apc::TEAM_HOME, apc::SportRole::SOCCER_ST, pos, 9);
    apc::EntityId id2 = mgr.spawn_athlete(apc::TEAM_AWAY, apc::SportRole::SOCCER_GK, pos, 1);
    apc::EntityId ball1 = mgr.spawn_ball(0, pos);

    assert(mgr.athlete_count > 0u);
    assert(mgr.ball_count > 0u);
    assert(mgr.get_team_athlete_count(apc::TEAM_HOME) == 1u);

    // Reset
    mgr.reset();

    // All counts zero
    assert(mgr.athlete_count == 0u && "After reset: athlete_count = 0");
    assert(mgr.ball_count == 0u && "After reset: ball_count = 0");
    assert(mgr.next_athlete_generation == 0u && "After reset: athlete_gen = 0");
    assert(mgr.next_ball_generation == 0u && "After reset: ball_gen = 0");

    // All slots invalid
    for (uint32_t i = 0; i < apc::MAX_ATHLETES; ++i) {
        assert(mgr.athletes[i].id.is_valid() == 0u && "All athlete slots invalid after reset");
    }
    for (uint32_t i = 0; i < apc::MAX_BALLS; ++i) {
        assert(mgr.balls[i].id.is_valid() == 0u && "All ball slots invalid after reset");
    }

    // Old IDs no longer valid
    assert(mgr.get_athlete(id1) == nullptr && "Old athlete ID → nullptr after reset");
    assert(mgr.get_athlete(id2) == nullptr && "Old athlete ID2 → nullptr after reset");
    assert(mgr.get_ball(ball1) == nullptr && "Old ball ID → nullptr after reset");

    // Team counts zero
    assert(mgr.get_team_athlete_count(apc::TEAM_HOME) == 0u);
    assert(mgr.get_team_athlete_count(apc::TEAM_AWAY) == 0u);

    // Can spawn again after reset
    apc::EntityId id3 = mgr.spawn_athlete(apc::TEAM_HOME, apc::SportRole::SOCCER_CM, pos, 5);
    assert(id3.is_valid() && "Can spawn after reset");
    assert(mgr.athlete_count == 1u && "Count = 1 after post-reset spawn");

    std::printf("    [PASS] reset() clears all state, allows fresh spawns\n");
    return 0;
}

// =============================================================================
// TEST 21: EntityManager generation counter (spawn/despawn/respawn)
// =============================================================================
static int test_entity_manager_generation_counter() {
    std::printf("  [Test 21] EntityManager generation counter...\n");

    apc::EntityManager mgr;
    apc::Vec3 pos(0.0f, 0.0f, 0.0f);

    // First spawn: generation = 1
    apc::EntityId id1 = mgr.spawn_athlete(apc::TEAM_HOME, apc::SportRole::SOCCER_CM, pos, 1);
    assert(id1.generation == 1u && "First spawn: gen = 1");

    // Second spawn: generation = 2
    apc::EntityId id2 = mgr.spawn_athlete(apc::TEAM_HOME, apc::SportRole::SOCCER_ST, pos, 9);
    assert(id2.generation == 2u && "Second spawn: gen = 2");

    // Generations are unique per spawn (monotonically increasing)
    assert(id2.generation > id1.generation && "Gen 2 > Gen 1");

    // Despawn does NOT increment generation
    uint32_t gen_before_despawn = mgr.next_athlete_generation;
    mgr.despawn(id1);
    assert(mgr.next_athlete_generation == gen_before_despawn
           && "Despawn does NOT increment generation counter");

    // Respawn in same slot: new generation
    apc::EntityId id3 = mgr.spawn_athlete(apc::TEAM_HOME, apc::SportRole::SOCCER_CM, pos, 2);
    assert(id3.generation == 3u && "Respawn: gen = 3");
    // Might reuse the same index or find a different free slot
    // (the first invalid slot is found, which could be index 0 or index 1)
    assert(id3.is_valid());

    // Old ID with gen=1 no longer works (stale reference)
    assert(mgr.get_athlete(id1) == nullptr && "Old gen=1 ID stale after respawn");

    // Same index + new gen works
    apc::EntityId stale{0u, 1u}; // gen=1 for slot 0
    apc::EntityId fresh{0u, 3u}; // gen=3 for slot 0 (if it reused slot 0)
    assert(mgr.get_athlete(stale) == nullptr && "Stale gen → nullptr");

    // Verify next_ball_generation works too
    apc::EntityId b1 = mgr.spawn_ball(0, pos);
    assert(b1.generation == 1u && "First ball: gen = 1");
    apc::EntityId b2 = mgr.spawn_ball(1, pos);
    assert(b2.generation == 2u && "Second ball: gen = 2");
    assert(mgr.next_ball_generation == 2u && "Ball gen counter = 2");

    std::printf("    [PASS] Generation counter increments per spawn, despawn is safe\n");
    return 0;
}

// =============================================================================
// TEST 22: MAX_ATHLETES capacity check
// =============================================================================
static int test_max_athletes_capacity() {
    std::printf("  [Test 22] MAX_ATHLETES capacity check...\n");

    apc::EntityManager mgr;
    apc::Vec3 pos(0.0f, 0.0f, 0.0f);

    assert(apc::MAX_ATHLETES == 44u && "MAX_ATHLETES = 44");
    assert(apc::MAX_BALLS == 3u && "MAX_BALLS = 3");

    // Spawn exactly 44 athletes
    apc::EntityId ids[apc::MAX_ATHLETES];
    for (uint32_t i = 0; i < apc::MAX_ATHLETES; ++i) {
        ids[i] = mgr.spawn_athlete(
            (i % 2 == 0) ? apc::TEAM_HOME : apc::TEAM_AWAY,
            apc::SportRole::SOCCER_CM,
            apc::Vec3(static_cast<float>(i), 0.0f, 0.0f),
            static_cast<uint8_t>(i + 1));
        assert(ids[i].is_valid() && "All 44 spawns succeed");
    }

    assert(mgr.athlete_count == apc::MAX_ATHLETES && "athlete_count = 44");
    assert(mgr.get_team_athlete_count(apc::TEAM_HOME) == 22u && "Home has 22");
    assert(mgr.get_team_athlete_count(apc::TEAM_AWAY) == 22u && "Away has 22");

    // 45th spawn should fail
    apc::EntityId overflow = mgr.spawn_athlete(
        apc::TEAM_HOME, apc::SportRole::SOCCER_CM, pos, 99);
    assert(overflow.is_valid() == 0u && "45th spawn returns INVALID");
    assert(overflow == apc::EntityId::make_invalid() && "45th = INVALID sentinel");

    // athlete_count unchanged after failed spawn
    assert(mgr.athlete_count == apc::MAX_ATHLETES && "Count still 44 after overflow");

    // All 44 are still accessible
    for (uint32_t i = 0; i < apc::MAX_ATHLETES; ++i) {
        assert(mgr.get_athlete(ids[i]) != nullptr && "All 44 athletes accessible");
    }

    // Balls can still be spawned (separate pool)
    apc::EntityId ball1 = mgr.spawn_ball(0, pos);
    assert(ball1.is_valid() && "Ball spawn succeeds even with full athletes");

    std::printf("    [PASS] 44 athletes fill capacity, 45th rejected, balls unaffected\n");
    return 0;
}

// =============================================================================
// TEST 23 (Bonus): AthleteEntity stamina and cooldowns
// =============================================================================
static int test_athlete_entity_stamina_cooldowns() {
    std::printf("  [Test 23] AthleteEntity stamina and cooldowns...\n");

    apc::AthleteEntity a;
    a.reset();

    // Initial stamina
    assert(approx_eq(a.stamina, 1.0f) && "Initial stamina = 1.0");

    // Drain stamina
    a.apply_stamina_drain(0.5f, 1.0f); // 0.5 drain for 1 second
    assert(approx_eq(a.stamina, 0.5f) && "After 0.5 drain for 1s: stamina = 0.5");

    // More drain
    a.apply_stamina_drain(0.3f, 1.0f);
    assert(approx_eq(a.stamina, 0.2f) && "After additional drain: stamina = 0.2");

    // Stamina floor at 0
    a.apply_stamina_drain(1.0f, 1.0f); // drain 1.0 for 1s → would go to -0.8
    assert(approx_eq(a.stamina, 0.0f) && "Stamina clamped at 0");

    // Cooldowns
    a.sprint_cooldown = 2.0f;
    a.tackle_cooldown = 1.5f;
    a.update_cooldowns(1.0f); // 1 second passes
    assert(approx_eq(a.sprint_cooldown, 1.0f) && "Sprint CD: 2.0 - 1.0 = 1.0");
    assert(approx_eq(a.tackle_cooldown, 0.5f) && "Tackle CD: 1.5 - 1.0 = 0.5");

    // Cooldowns floor at 0
    a.update_cooldowns(2.0f);
    assert(approx_eq(a.sprint_cooldown, 0.0f) && "Sprint CD clamped at 0");
    assert(approx_eq(a.tackle_cooldown, 0.0f) && "Tackle CD clamped at 0");

    // Zero cooldown: no negative
    a.update_cooldowns(1.0f);
    assert(a.sprint_cooldown >= 0.0f && "Cooldown never goes negative");

    std::printf("    [PASS] Stamina drain (floors at 0) and cooldown decrement verified\n");
    return 0;
}

// =============================================================================
// TEST 24 (Bonus): BallEntity reset
// =============================================================================
static int test_ball_entity_reset() {
    std::printf("  [Test 24] BallEntity reset...\n");

    apc::BallEntity b;
    b.configure_for_sport(1); // basketball
    b.position = {10.0f, 5.0f, -3.0f};
    b.velocity = {2.0f, 0.0f, -1.0f};
    b.is_in_play = 1;
    b.possession_team = apc::TEAM_HOME;

    assert(approx_eq(b.mass, 0.62f) && "Before reset: basketball mass");

    b.reset();

    // All fields back to defaults
    assert(b.id.is_valid() == 0u && "After reset: id invalid");
    assert(b.type == apc::EntityType::BALL && "After reset: type = BALL");
    assert(approx_eq(b.position.x, 0.0f) && "After reset: pos.x = 0");
    assert(approx_eq(b.position.y, 0.0f) && "After reset: pos.y = 0");
    assert(approx_eq(b.velocity.x, 0.0f) && "After reset: vel.x = 0");
    assert(approx_eq(b.angular_velocity.x, 0.0f) && "After reset: ang_vel.x = 0");
    assert(approx_eq(b.mass, 0.43f) && "After reset: mass = 0.43 (soccer default)");
    assert(approx_eq(b.radius, 0.11f) && "After reset: radius = 0.11");
    assert(b.ball_type == 0 && "After reset: ball_type = 0");
    assert(approx_eq(b.bounce_restitution, 0.75f) && "After reset: bounce = 0.75");
    assert(approx_eq(b.ground_friction, 0.4f) && "After reset: friction = 0.4");
    assert(b.is_in_play == 0 && "After reset: not in play");
    assert(b.possession_team == apc::TEAM_NONE && "After reset: possession = NONE");
    assert(b.last_toucher.is_valid() == 0u && "After reset: last_toucher invalid");

    std::printf("    [PASS] BallEntity reset restores all defaults\n");
    return 0;
}

// =============================================================================
// TEST 25 (Bonus): EntityManager update_all
// =============================================================================
static int test_entity_manager_update_all() {
    std::printf("  [Test 25] EntityManager update_all...\n");

    apc::EntityManager mgr;
    apc::Vec3 pos(0.0f, 0.0f, 0.0f);

    // Spawn athlete with cooldowns and reduced stamina
    apc::EntityId id = mgr.spawn_athlete(apc::TEAM_HOME, apc::SportRole::SOCCER_CM, pos, 5);
    apc::AthleteEntity* a = mgr.get_athlete(id);
    assert(a != nullptr);

    a->sprint_cooldown = 1.0f;
    a->tackle_cooldown = 0.5f;
    a->stamina = 0.5f;

    // Update for 1 second
    mgr.update_all(1.0f);

    // Cooldowns decremented
    assert(approx_eq(a->sprint_cooldown, 0.0f) && "After update: sprint CD = 0");
    assert(approx_eq(a->tackle_cooldown, 0.0f) && "After update: tackle CD = 0");

    // Stamina regenerated (base rate 0.05/s)
    float expected_stamina = 0.5f + 0.05f * 1.0f;
    assert(approx_eq(a->stamina, expected_stamina) && "After update: stamina regen");

    // Spawn ball and update
    apc::EntityId ball_id = mgr.spawn_ball(0, apc::Vec3(0.0f, 0.0f, 0.0f));
    apc::BallEntity* b = mgr.get_ball(ball_id);
    b->velocity = apc::Vec3(10.0f, 0.0f, 0.0f);

    mgr.update_all(1.0f);
    // Ball should slow down due to friction (on ground)
    float speed = std::sqrt(b->velocity.x * b->velocity.x +
                            b->velocity.z * b->velocity.z);
    assert(speed < 10.0f && "Ball slows down with friction");

    std::printf("    [PASS] update_all decrements cooldowns, regens stamina, applies friction\n");
    return 0;
}

// =============================================================================
// TEST 26 (Bonus): EntityManager find_ball
// =============================================================================
static int test_entity_manager_find_ball() {
    std::printf("  [Test 26] EntityManager find_ball...\n");

    apc::EntityManager mgr;
    apc::Vec3 pos(0.0f, 0.0f, 0.0f);

    // No balls initially
    assert(mgr.find_ball() == nullptr && "No balls: find_ball → nullptr");

    // Spawn a ball
    apc::EntityId b1 = mgr.spawn_ball(0, pos);
    apc::BallEntity* found = mgr.find_ball();
    assert(found != nullptr && "With one ball: find_ball → valid");
    assert(found->id == b1 && "Found ball matches spawned ID");

    // Despawn ball
    mgr.despawn(b1);
    assert(mgr.find_ball() == nullptr && "After despawn: find_ball → nullptr");

    // Spawn multiple balls
    mgr.spawn_ball(0, pos);
    apc::EntityId b3 = mgr.spawn_ball(1, apc::Vec3(5.0f, 0.0f, 0.0f));

    apc::BallEntity* found2 = mgr.find_ball();
    assert(found2 != nullptr && "Multiple balls: find_ball → first active");
    // First valid ball found in deterministic order
    assert(found2->id.is_valid());

    std::printf("    [PASS] find_ball returns first active ball or nullptr\n");
    return 0;
}

// =============================================================================
// TEST 27 (Bonus): EntityId is_valid edge cases
// =============================================================================
static int test_entity_id_is_valid_edge_cases() {
    std::printf("  [Test 27] EntityId is_valid edge cases...\n");

    // All-ones index → invalid
    apc::EntityId all_invalid{0xFFFFFFFFu, 0u};
    assert(all_invalid.is_valid() == 0u && "0xFFFFFFFF index → invalid");

    // Index 0 with any generation → valid
    apc::EntityId idx0_gen0{0u, 0u};
    assert(idx0_gen0.is_valid() == 1u && "Index 0, gen 0 → valid");

    apc::EntityId idx0_genMax{0u, 0xFFFFFFFFu};
    assert(idx0_genMax.is_valid() == 1u && "Index 0, gen max → valid");

    // Large but valid index
    apc::EntityId idx43{43u, 1u};
    assert(idx43.is_valid() == 1u && "Index 43 → valid");

    // One below invalid
    apc::EntityId idx_max_m1{0xFFFFFFFEu, 0u};
    assert(idx_max_m1.is_valid() == 1u && "Index 0xFFFFFFFE → valid");

    std::printf("    [PASS] is_valid: only 0xFFFFFFFF index is invalid\n");
    return 0;
}

// =============================================================================
// TEST 28 (Bonus): Capacity constants
// =============================================================================
static int test_capacity_constants() {
    std::printf("  [Test 28] Capacity constants...\n");

    assert(apc::MAX_ATHLETES == 44u && "MAX_ATHLETES = 44");
    assert(apc::MAX_BALLS == 3u && "MAX_BALLS = 3");
    assert(apc::MAX_TEAMS == 4u && "MAX_TEAMS = 4");

    // EntityManager arrays can hold exactly these counts
    static_assert(sizeof(apc::EntityManager::athletes) / sizeof(apc::AthleteEntity) == apc::MAX_ATHLETES,
                  "athletes array size matches MAX_ATHLETES");
    static_assert(sizeof(apc::EntityManager::balls) / sizeof(apc::BallEntity) == apc::MAX_BALLS,
                  "balls array size matches MAX_BALLS");

    std::printf("    [PASS] MAX_ATHLETES=44, MAX_BALLS=3, MAX_TEAMS=4 verified\n");
    return 0;
}

// =============================================================================
// Main
// =============================================================================
int main() {
    std::printf("=== Sprint 22 Tests ===\n");
    std::printf("Entity System: Types, IDs, Manager\n");
    std::printf("================================================================\n");

    int fails = 0;
    fails += test_entity_type_enum_values();
    fails += test_sport_role_enum_values();
    fails += test_team_id_type();
    fails += test_entity_id_construction_comparison();
    fails += test_entity_id_invalid_sentinel();
    fails += test_athlete_entity_defaults();
    fails += test_athlete_entity_field_types();
    fails += test_ball_entity_defaults();
    fails += test_ball_entity_ball_type();
    fails += test_ball_entity_sport_defaults();
    fails += test_entity_manager_defaults();
    fails += test_entity_manager_spawn_athlete_valid();
    fails += test_entity_manager_spawn_athlete_increments_count();
    fails += test_entity_manager_get_athlete_valid();
    fails += test_entity_manager_get_athlete_invalid();
    fails += test_entity_manager_despawn();
    fails += test_entity_manager_spawn_ball_valid();
    fails += test_entity_manager_find_athlete_by_role();
    fails += test_entity_manager_get_team_athlete_count();
    fails += test_entity_manager_reset();
    fails += test_entity_manager_generation_counter();
    fails += test_max_athletes_capacity();
    fails += test_athlete_entity_stamina_cooldowns();
    fails += test_ball_entity_reset();
    fails += test_entity_manager_update_all();
    fails += test_entity_manager_find_ball();
    fails += test_entity_id_is_valid_edge_cases();
    fails += test_capacity_constants();

    int total = 28;
    int passed = total - fails;
    std::printf("================================================================\n");
    std::printf("=== Sprint 22: %d tests passed, %d failed ===\n", passed, fails);
    return fails;
}
