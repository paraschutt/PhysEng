#pragma once
// =============================================================================
// Sport Field — Playing field geometry, boundaries, surfaces, obstacles
// =============================================================================
//
// Defines the physical playing environment for various sports:
//
//   - SportType: enumeration of supported sports
//   - FieldGeometry: dimensions, shape, boundary markers
//   - FieldZone: regions with different surfaces/properties
//   - GoalPost: goal structure geometry
//   - NetVolume: net collision volume
//   - BoundaryEvent: out-of-bounds, scoring triggers
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation
//   - Deterministic
//   - C++17
//
// =============================================================================

#include "apc_sport/apc_ball_physics.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_math_common.h"
#include <cstdint>
#include <cmath>

namespace apc {

// =============================================================================
// SportType — Supported sport types
// =============================================================================
enum class SportType : uint8_t {
    SOCCER              = 0,   // Association football
    BASKETBALL          = 1,
    AMERICAN_FOOTBALL   = 2,
    RUGBY_UNION         = 3,
    RUGBY_LEAGUE        = 4,
    AUSTRALIAN_RULES    = 5,
    TENNIS              = 6,
    VOLLEYBALL          = 7,
    HANDBALL            = 8,
    ICE_HOCKEY          = 9,
    FIELD_HOCKEY        = 10,
    CRICKET             = 11,
    BASEBALL            = 12,
    GOLF                = 13,
    BADMINTON           = 14,
    TABLE_TENNIS        = 15,
    LACROSSE            = 16,
    RUGBY_SEVENS        = 17,
    BEACH_VOLLEYBALL    = 18,
    FUTSAL              = 19,
    HURLING             = 20,
    GAELIC_FOOTBALL     = 21,
    WATER_POLO          = 22,
    BOXING              = 23,
    MMA                 = 24,
    WRESTLING           = 25,
    CUSTOM_0            = 26,
    CUSTOM_1            = 27,
    NUM_SPORTS          = 28
};

// =============================================================================
// FieldType — Shape/layout of the playing field
// =============================================================================
enum class FieldType : uint8_t {
    RECTANGLE       = 0,   // Soccer, rugby, basketball, tennis
    OVAL            = 1,   // Cricket, Australian rules
    DIAMOND         = 2,   // Baseball
    CIRCLE          = 3,   // Volleyball, beach volleyball
    RINK            = 4,   // Ice hockey (rounded rectangle)
    RING            = 5,   // Boxing, MMA
    COURSE          = 6,   // Golf (multiple zones)
    MAT             = 7    // Wrestling, gymnastics
};

// =============================================================================
// FieldGeometry — Physical dimensions and layout
// =============================================================================
struct FieldGeometry {
    FieldType type = FieldType::RECTANGLE;
    SportType sport = SportType::SOCCER;
    const char* name = "soccer";

    // --- Rectangular field ---
    float length = 105.0f;          // Length (m) — long axis
    float width = 68.0f;            // Width (m) — short axis
    float center_radius = 9.15f;    // Center circle radius

    // --- Boundary ---
    float boundary_width = 5.0f;    // Width of out-of-bounds area (m)
    float corner_radius = 1.0f;     // Corner arc radius (soccer)
    float corner_post_height = 1.5f; // Corner flag height

    // --- Goal dimensions ---
    float goal_width = 7.32f;       // Soccer goal width
    float goal_height = 2.44f;      // Soccer goal height
    float goal_depth = 2.0f;        // Depth behind goal line (net)
    float goal_post_width = 0.12f;  // Goal post diameter

    // --- Penalty area ---
    float penalty_area_width = 40.32f;  // Width of penalty box
    float penalty_area_depth = 16.5f;   // Depth of penalty box
    float goal_area_width = 18.32f;     // Width of 6-yard box
    float goal_area_depth = 5.5f;       // Depth of 6-yard box
    float penalty_spot = 11.0f;         // Penalty kick distance

    // --- Other markings ---
    float three_point_line = 6.75f;    // Basketball 3-point distance
    float free_throw_line = 4.6f;      // Basketball free throw distance
    float key_width = 4.9f;            // Basketball key/paint width
    float key_length = 5.8f;           // Basketball key/paint length

    // --- Heights ---
    float ground_level = 0.0f;        // Y coordinate of ground
    float wall_height = 0.0f;         // Wall height (0 = no walls)
    float ceiling_height = 0.0f;      // Ceiling (0 = no ceiling)

    // --- Surface ---
    SurfaceType default_surface = SurfaceType::GRASS;
    float surface_hardness = 0.5f;    // 0 = soft, 1 = hard
    float surface_unevenness = 0.0f;  // 0 = flat, 1 = very bumpy

    // --- Factory methods for standard sports ---

    static FieldGeometry make_soccer() {
        FieldGeometry f;
        f.type = FieldType::RECTANGLE;
        f.sport = SportType::SOCCER;
        f.name = "soccer";
        f.length = 105.0f;
        f.width = 68.0f;
        f.center_radius = 9.15f;
        f.boundary_width = 5.0f;
        f.corner_radius = 1.0f;
        f.goal_width = 7.32f;
        f.goal_height = 2.44f;
        f.goal_depth = 2.5f;
        f.goal_post_width = 0.12f;
        f.penalty_area_width = 40.32f;
        f.penalty_area_depth = 16.5f;
        f.goal_area_width = 18.32f;
        f.goal_area_depth = 5.5f;
        f.penalty_spot = 11.0f;
        f.default_surface = SurfaceType::GRASS;
        return f;
    }

    static FieldGeometry make_basketball() {
        FieldGeometry f;
        f.type = FieldType::RECTANGLE;
        f.sport = SportType::BASKETBALL;
        f.name = "basketball";
        f.length = 28.0f;
        f.width = 15.0f;
        f.center_radius = 1.8f;
        f.boundary_width = 2.0f;
        f.goal_width = 1.8f;         // Basket rim diameter
        f.goal_height = 3.05f;       // Rim height
        f.goal_depth = 0.4f;
        f.three_point_line = 6.75f;
        f.free_throw_line = 4.6f;
        f.key_width = 4.9f;
        f.key_length = 5.8f;
        f.wall_height = 0.0f;
        f.ceiling_height = 7.0f;
        f.default_surface = SurfaceType::WOOD;
        return f;
    }

    static FieldGeometry make_american_football() {
        FieldGeometry f;
        f.type = FieldType::RECTANGLE;
        f.sport = SportType::AMERICAN_FOOTBALL;
        f.name = "american_football";
        f.length = 109.7f;           // 100 yards + 2 end zones
        f.width = 48.8f;             // 53.3 yards
        f.center_radius = 0.0f;
        f.boundary_width = 3.0f;
        f.goal_width = 5.64f;        // Goal posts (18.5 feet)
        f.goal_height = 3.05f;       // Crossbar height (10 feet)
        f.goal_depth = 0.0f;
        f.penalty_area_depth = 0.0f;
        f.default_surface = SurfaceType::ARTIFICIAL;
        return f;
    }

    static FieldGeometry make_rugby() {
        FieldGeometry f;
        f.type = FieldType::RECTANGLE;
        f.sport = SportType::RUGBY_UNION;
        f.name = "rugby";
        f.length = 100.0f;
        f.width = 70.0f;
        f.center_radius = 10.0f;
        f.boundary_width = 5.0f;
        f.goal_width = 5.6f;         // Rugby goal width
        f.goal_height = 3.0f;        // Crossbar height
        f.goal_depth = 0.0f;
        f.penalty_spot = 22.0f;      // Conversion kick distance
        f.default_surface = SurfaceType::GRASS;
        return f;
    }

    static FieldGeometry make_tennis() {
        FieldGeometry f;
        f.type = FieldType::RECTANGLE;
        f.sport = SportType::TENNIS;
        f.name = "tennis";
        f.length = 23.77f;
        f.width = 10.97f;
        f.center_radius = 0.0f;
        f.boundary_width = 3.0f;
        f.goal_width = 12.8f;        // Net width (doubles)
        f.goal_height = 0.914f;      // Net height at center
        f.default_surface = SurfaceType::HARD_COURT;
        f.surface_hardness = 0.8f;
        return f;
    }

    static FieldGeometry make_ice_hockey() {
        FieldGeometry f;
        f.type = FieldType::RINK;
        f.sport = SportType::ICE_HOCKEY;
        f.name = "ice_hockey";
        f.length = 60.96f;           // 200 feet
        f.width = 25.91f;            // 85 feet
        f.center_radius = 4.57f;     // Center circle
        f.boundary_width = 0.0f;
        f.wall_height = 1.07f;       // Boards height
        f.goal_width = 1.83f;        // Goal opening
        f.goal_height = 1.22f;       // Goal opening height
        f.goal_depth = 1.0f;
        f.default_surface = SurfaceType::ICE;
        f.wall_height = 1.07f;
        f.ceiling_height = 0.0f;
        return f;
    }

    static FieldGeometry make_baseball_diamond() {
        FieldGeometry f;
        f.type = FieldType::DIAMOND;
        f.sport = SportType::BASEBALL;
        f.name = "baseball";
        f.length = 18.44f;           // Pitcher's mound distance
        f.width = 27.43f;            // Base path (90 feet)
        f.center_radius = 0.0f;
        f.boundary_width = 10.0f;
        f.default_surface = SurfaceType::GRASS;
        return f;
    }

    static FieldGeometry make_aussie_rules() {
        FieldGeometry f;
        f.type = FieldType::OVAL;
        f.sport = SportType::AUSTRALIAN_RULES;
        f.name = "aussie_rules";
        f.length = 165.0f;
        f.width = 135.0f;
        f.center_radius = 15.0f;
        f.boundary_width = 5.0f;
        f.goal_width = 6.4f;
        f.goal_height = 3.0f;
        f.default_surface = SurfaceType::GRASS;
        return f;
    }

    static FieldGeometry make_volleyball() {
        FieldGeometry f;
        f.type = FieldType::RECTANGLE;
        f.sport = SportType::VOLLEYBALL;
        f.name = "volleyball";
        f.length = 18.0f;
        f.width = 9.0f;
        f.center_radius = 0.0f;
        f.boundary_width = 3.0f;
        f.goal_width = 9.0f;         // Net width
        f.goal_height = 2.43f;       // Net height (men's)
        f.default_surface = SurfaceType::WOOD;
        f.wall_height = 0.0f;
        return f;
    }

    static FieldGeometry make_beach_volleyball() {
        FieldGeometry f;
        f.type = FieldType::RECTANGLE;
        f.sport = SportType::BEACH_VOLLEYBALL;
        f.name = "beach_volleyball";
        f.length = 16.0f;
        f.width = 8.0f;
        f.center_radius = 0.0f;
        f.boundary_width = 3.0f;
        f.goal_width = 8.0f;
        f.goal_height = 2.43f;
        f.default_surface = SurfaceType::SAND;
        return f;
    }
};

// =============================================================================
// FieldZone — Named region of the field with specific properties
// =============================================================================
enum class FieldZoneId : uint8_t {
    FULL_FIELD     = 0,
    HALF_HOME      = 1,
    HALF_AWAY      = 2,
    PENALTY_AREA   = 3,
    GOAL_AREA      = 4,
    CENTER_CIRCLE  = 5,
    WING_LEFT      = 6,
    WING_RIGHT     = 7,
    END_ZONE_HOME  = 8,
    END_ZONE_AWAY  = 9,
    THREE_POINT    = 10,
    KEY_PAINT      = 11,
    NO_MAN_LAND    = 12,
    GOAL_MOUTH     = 13,
    OUT_OF_BOUNDS  = 14,
    TECHNICAL_AREA = 15,
    BENCH_AREA     = 16,
    WARMUP_AREA    = 17,
    DUGOUT         = 18,
    MAX_ZONES      = 19
};

struct FieldZone {
    FieldZoneId zone_id;
    const char* name;
    Vec3 center;
    Vec3 extents;            // Half-size (x, y, z)
    SurfaceType surface;     // Override default surface
    bool is_out_of_bounds;   // Ball entering = out of play
    bool is_scoring_zone;    // Ball entering = score check
    uint8_t team_id;         // 0 = neutral, 1 = home, 2 = away

    bool contains_point(const Vec3& point) const {
        return (point.x >= center.x - extents.x && point.x <= center.x + extents.x &&
                point.z >= center.z - extents.z && point.z <= center.z + extents.z);
    }
};

// =============================================================================
// GoalPost — Physical structure of a goal
// =============================================================================
struct GoalPost {
    Vec3 position;               // Center of goal line
    Vec3 opening_center;         // Center of the goal opening
    float width;                 // Opening width
    float height;                // Opening height (crossbar)
    float depth;                 // Net depth behind goal line
    float post_diameter = 0.12f; // Post thickness
    Vec3 facing_direction;       // Direction goal faces (toward field)
    uint8_t team_id = 0;         // Which team's goal

    bool is_ball_in_goal(const Vec3& ball_pos, float ball_radius) const {
        // Check if ball center is within the goal opening
        Vec3 rel = Vec3::sub(ball_pos, opening_center);

        // Width check (along goal line)
        float half_w = width * 0.5f;
        if (rel.x < -half_w || rel.x > half_w) return false;

        // Height check
        if (rel.y < 0.0f || rel.y > height) return false;

        // Depth check (behind goal line)
        float dot = Vec3::dot(rel, facing_direction);
        if (dot < -ball_radius || dot > depth) return false;

        return true;
    }

    bool is_ball_hitting_post(const Vec3& ball_pos, float ball_radius) const {
        // Check left post
        Vec3 left_post = Vec3::add(opening_center, Vec3(-width * 0.5f, 0.0f, 0.0f));
        float dist_left = Vec3::length(Vec3::sub(ball_pos, left_post));
        if (dist_left < ball_radius + post_diameter * 0.5f) return true;

        // Check right post
        Vec3 right_post = Vec3::add(opening_center, Vec3(width * 0.5f, 0.0f, 0.0f));
        float dist_right = Vec3::length(Vec3::sub(ball_pos, right_post));
        if (dist_right < ball_radius + post_diameter * 0.5f) return true;

        // Check crossbar
        Vec3 crossbar_center = Vec3::add(opening_center, Vec3(0.0f, height, 0.0f));
        float dist_cross = std::sqrt(
            (ball_pos.x - crossbar_center.x) * (ball_pos.x - crossbar_center.x) +
            (ball_pos.y - crossbar_center.y) * (ball_pos.y - crossbar_center.y)
        );
        if (dist_cross < ball_radius + post_diameter * 0.5f &&
            ball_pos.z >= crossbar_center.z - post_diameter &&
            ball_pos.z <= crossbar_center.z + depth)
        {
            return true;
        }

        return false;
    }
};

// =============================================================================
// BoundaryEvent — What happens when a ball/athlete crosses a boundary
// =============================================================================
enum class BoundaryEventType : uint8_t {
    NONE           = 0,
    OUT_OF_BOUNDS  = 1,   // Ball leaves field (throw-in, goal kick, etc.)
    GOAL_SCORED    = 2,   // Ball enters goal
    HIT_POST       = 3,   // Ball hits goalpost
    HIT_CROSSBAR   = 4,   // Ball hits crossbar
    HIT_NET        = 5,   // Ball hits net
    TOUCHDOWN      = 6,   // Ball carried into end zone (American football)
    SAFETY         = 7,   // Ball carrier tackled in own end zone
    DEAD_BALL      = 8,   // Ball is dead (whistle)
    GROUNDING      = 9,   // Rugby/Aussie rules grounding in end zone
    BEHIND         = 10,  // Aussie rules behind (1 point)
    HOME_RUN       = 11,  // Baseball home run (over fence)
    FOUL_BALL      = 12   // Baseball foul ball
};

struct BoundaryEvent {
    BoundaryEventType type = BoundaryEventType::NONE;
    uint32_t ball_id = 0;
    Vec3 position;
    Vec3 velocity_at_boundary;
    float timestamp = 0.0f;
    uint8_t team_id = 0;      // Which team benefits
    uint8_t zone_id = 0;      // Which zone triggered
};

// =============================================================================
// SportField — Complete playing field with geometry, goals, zones
// =============================================================================
struct SportField {
    static constexpr uint32_t MAX_ZONES = 20;
    static constexpr uint32_t MAX_GOALS = 4;
    static constexpr uint32_t MAX_EVENTS = 32;

    FieldGeometry geometry;
    FieldZone zones[MAX_ZONES];
    uint32_t zone_count = 0;
    GoalPost goals[MAX_GOALS];
    uint32_t goal_count = 0;
    BoundaryEvent pending_events[MAX_EVENTS];
    uint32_t event_count = 0;
    SurfaceBounceTable surface_table;

    void setup(const FieldGeometry& geo) {
        geometry = geo;
        surface_table = SurfaceBounceTable::make_default();
        zone_count = 0;
        goal_count = 0;
        event_count = 0;

        // Default: full field zone
        add_zone(FieldZoneId::FULL_FIELD, "full_field",
            Vec3(0.0f, 0.0f, 0.0f),
            Vec3(geo.length * 0.5f, 50.0f, geo.width * 0.5f),
            geo.default_surface);
    }

    void add_zone(FieldZoneId id, const char* name,
                   const Vec3& center, const Vec3& extents,
                   SurfaceType surface = SurfaceType::GRASS,
                   bool oob = false, bool scoring = false, uint8_t team = 0)
    {
        if (zone_count >= MAX_ZONES) return;
        FieldZone& z = zones[zone_count++];
        z.zone_id = id;
        z.name = name;
        z.center = center;
        z.extents = extents;
        z.surface = surface;
        z.is_out_of_bounds = oob;
        z.is_scoring_zone = scoring;
        z.team_id = team;
    }

    void add_goal(const GoalPost& goal) {
        if (goal_count >= MAX_GOALS) return;
        goals[goal_count++] = goal;
    }

    // --- Check if a point is in bounds ---
    bool is_in_bounds(const Vec3& point) const {
        float half_l = geometry.length * 0.5f;
        float half_w = geometry.width * 0.5f;
        return (point.x >= -half_l && point.x <= half_l &&
                point.z >= -half_w && point.z <= half_w);
    }

    // --- Get surface type at a point ---
    SurfaceType get_surface_at(const Vec3& point) const {
        // Check zones in reverse order (last added = highest priority)
        for (int32_t i = static_cast<int32_t>(zone_count) - 1; i >= 0; --i) {
            if (zones[i].contains_point(point)) {
                return zones[i].surface;
            }
        }
        return geometry.default_surface;
    }

    // --- Check ball against goals ---
    BoundaryEvent check_goals(const Vec3& ball_pos, float ball_radius,
                               uint32_t ball_id, float timestamp) const
    {
        BoundaryEvent event;
        event.ball_id = ball_id;
        event.timestamp = timestamp;
        event.position = ball_pos;

        for (uint32_t i = 0; i < goal_count; ++i) {
            if (goals[i].is_ball_in_goal(ball_pos, ball_radius)) {
                event.type = BoundaryEventType::GOAL_SCORED;
                event.team_id = goals[i].team_id;
                return event;
            }
            if (goals[i].is_ball_hitting_post(ball_pos, ball_radius)) {
                event.type = BoundaryEventType::HIT_POST;
                return event;
            }
        }

        return event;
    }

    // --- Check ball boundaries ---
    BoundaryEvent check_boundary(const Vec3& ball_pos, const Vec3& ball_vel,
                                  float ball_radius, uint32_t ball_id,
                                  float timestamp)
    {
        BoundaryEvent event;
        event.ball_id = ball_id;
        event.timestamp = timestamp;
        event.velocity_at_boundary = ball_vel;

        // Check goals first
        event = check_goals(ball_pos, ball_radius, ball_id, timestamp);
        if (event.type != BoundaryEventType::NONE) return event;

        // Check field bounds
        float half_l = geometry.length * 0.5f + ball_radius;
        float half_w = geometry.width * 0.5f + ball_radius;

        if (ball_pos.x < -half_l || ball_pos.x > half_l ||
            ball_pos.z < -half_w || ball_pos.z > half_w)
        {
            // Determine which boundary was crossed
            if (ball_pos.z < -half_w || ball_pos.z > half_w) {
                event.type = BoundaryEventType::OUT_OF_BOUNDS;
                event.team_id = (ball_pos.z > half_w) ? 1 : 2;
            } else if (ball_pos.x < -half_l || ball_pos.x > half_l) {
                event.type = BoundaryEventType::OUT_OF_BOUNDS;
                event.team_id = (ball_pos.x > half_l) ? 1 : 2;
            }
            event.position = ball_pos;
        }

        return event;
    }

    void reset_events() {
        event_count = 0;
    }
};

} // namespace apc
