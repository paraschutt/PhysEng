#pragma once
// =============================================================================
// Sport Rules — Scoring, fouls, play state management per sport
// =============================================================================
//
// Defines the rules framework that governs gameplay:
//
//   - SemanticPlayState: generic state semantics for AI/Physics consumption
//   - SemanticFoulConsequence: generic foul consequence for AI/Physics
//   - SportRulesConfig: injected per-sport rule configuration for AI
//   - SportEventContext: sport-specific event payload (HUD/Logging)
//   - PlayState: [LEGACY] sport-specific game state (HUD/Rules engine only)
//   - FoulType: [LEGACY] sport-specific foul classification (Discipline only)
//   - CardType: disciplinary actions
//   - ScoringSystem: point values, scoring conditions
//   - DisciplineSystem: foul detection, card accumulation
//   - ClockSystem: game clock, shot clock, timeouts
//
// Phase 11a Architecture:
//   The AI and Physics engines consume ONLY SemanticPlayState and
//   SemanticFoulConsequence. The underlying SportRulesEngine can still
//   track the specific sport-specific PlayState/FoulType for the UI/HUD,
//   but simulation and AI layers only consume the semantic meaning.
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation
//   - Deterministic
//   - C++17
//
// =============================================================================

#include "apc_sport/apc_sport_field.h"
#include "apc_entity/apc_entity_types.h"
#include <cstdint>
#include <cmath>

namespace apc {

// =============================================================================
// 1. Semantic Play States (Phase 11a — Replaces the God Enum for AI/Physics)
// =============================================================================
// The AI and Physics engines ONLY consume these generic semantics.
// Instead of asking "Is the state CORNER_KICK?", the AI asks
// "Is the state SET_PIECE_OFFENSE?" — a corner kick, an inbound pass,
// and a lineout all resolve to the same AI tactical requirement.
//
// Semantic mapping examples:
//   KICKOFF        -> SET_PIECE_NEUTRAL
//   CORNER_KICK    -> SET_PIECE_OFFENSE
//   FREE_KICK      -> SET_PIECE_OFFENSE
//   THROW_IN       -> SET_PIECE_OFFENSE
//   LINEOUT        -> SET_PIECE_DEFENSE
//   SCRUM          -> SET_PIECE_NEUTRAL
//   FACEOFF        -> SET_PIECE_NEUTRAL
//   JUMP_BALL      -> SET_PIECE_NEUTRAL
//   INBOUND        -> SET_PIECE_OFFENSE
//   LIVE           -> LIVE_PLAY
//   DEAD_BALL      -> DEAD_BALL
//   GOAL_SCORED    -> SCORING_EVENT
//   TIMEOUT        -> INTERMISSION
//   HALFTIME       -> INTERMISSION
//   NOT_STARTED    -> PRE_GAME
//   GAME_OVER      -> INTERMISSION
// =============================================================================
enum class SemanticPlayState : uint8_t {
    PRE_GAME,
    LIVE_PLAY,          // Ball is active, open play
    DEAD_BALL,          // Play is halted, clock may be stopped
    SET_PIECE_NEUTRAL,  // e.g., Faceoff, Jump Ball, Scrum
    SET_PIECE_OFFENSE,  // e.g., Corner Kick, Inbound Pass, Free Kick
    SET_PIECE_DEFENSE,  // Defending the above
    SCORING_EVENT,      // Celebration/reset phase
    INTERMISSION        // Halftime, Timeouts
};

// =============================================================================
// 2. Semantic Foul Consequences (Phase 11a — Replaces the God Enum for AI)
// =============================================================================
// The AI doesn't care if it was a "High Tackle" or "Charging";
// it only evaluates the risk based on the consequence.
//
// Semantic mapping examples:
//   NONE / DELAY_OF_GAME             -> NONE
//   OFFSIDE / TRAVELING               -> TURNOVER
//   ADVANTAGE_PLAY                    -> PENALTY_ADVANTAGE
//   PENALTY_KICK / FREE_THROW         -> PENALTY_SHOT
//   YELLOW_CARD / SIN_BIN             -> TEMPORARY_EJECTION
//   RED_CARD / FLAGRANT_FOUL          -> MATCH_EJECTION
//   MINOR_FOUL (first/second)         -> WARNING
// =============================================================================
enum class SemanticFoulConsequence : uint8_t {
    NONE,
    WARNING,
    TURNOVER,           // Loss of possession (e.g., Offside, Traveling)
    PENALTY_ADVANTAGE,  // Play continues (e.g., Soccer advantage)
    PENALTY_SHOT,       // Direct scoring opportunity given (e.g., Free throw, Penalty kick)
    TEMPORARY_EJECTION, // e.g., Yellow card, Sin bin, Penalty box
    MATCH_EJECTION      // e.g., Red card, Flagrant 2
};

// =============================================================================
// 3. Sport Rule Configuration (Phase 11a)
// =============================================================================
// Injected by the SceneManager so the AI knows what physics actions are legal.
// This decouples the AI from sport-specific rule details.
// =============================================================================
struct SportRulesConfig {
    SemanticPlayState current_state = SemanticPlayState::PRE_GAME;

    // AI Legal/Utility Constraints
    bool offside_rule_active = false;
    float allowed_contact_severity = 0.0f; // 0.0 = non-contact, 1.0 = full tackle
    bool hands_allowed_field = false;      // True for Rugby/Basketball, False for Soccer
    bool feet_allowed_field = true;        // True for Soccer/Rugby, False for Basketball

    // Limits
    uint32_t max_fouls_before_ejection = 0;
    uint32_t possession_clock_max_ms = 0;  // e.g., 24000 for basketball shot clock
};

// =============================================================================
// 4. Specific Sport Event Payload (Phase 11a)
// =============================================================================
// Keeps the specific flavor text/IDs out of the AI layer.
// Used by HUD / Logging / Rules Engine for sport-specific display.
// =============================================================================
struct SportEventContext {
    uint32_t sport_specific_event_id = 0;  // Maps to localized strings (e.g., "Corner Kick")
    uint32_t primary_athlete_id = 0;
    uint32_t secondary_athlete_id = 0;
    SemanticFoulConsequence consequence = SemanticFoulConsequence::NONE;
};

// =============================================================================
// [LEGACY] PlayState — Sport-specific game state
// =============================================================================
// Retained for HUD, DisciplineSystem, and rules engine internal use.
// AI and Physics layers should consume SemanticPlayState instead.
// =============================================================================
enum class PlayState : uint8_t {
    NOT_STARTED     = 0,
    KICKOFF         = 1,    // -> SemanticPlayState::SET_PIECE_NEUTRAL
    LIVE            = 2,    // -> SemanticPlayState::LIVE_PLAY
    DEAD_BALL       = 3,    // -> SemanticPlayState::DEAD_BALL
    GOAL_SCORED     = 4,    // -> SemanticPlayState::SCORING_EVENT
    TIMEOUT         = 5,    // -> SemanticPlayState::INTERMISSION
    PERIOD_END      = 6,    // -> SemanticPlayState::INTERMISSION
    GAME_OVER       = 7,    // -> SemanticPlayState::INTERMISSION
    OVERTIME        = 8,    // -> SemanticPlayState::LIVE_PLAY
    SHOOTOUT        = 9,    // -> SemanticPlayState::SET_PIECE_NEUTRAL
    INJURY_STOP     = 10,   // -> SemanticPlayState::INTERMISSION
    VAR_REVIEW      = 11,   // -> SemanticPlayState::DEAD_BALL
    FREE_KICK       = 12,   // -> SemanticPlayState::SET_PIECE_OFFENSE
    CORNER_KICK     = 13,   // -> SemanticPlayState::SET_PIECE_OFFENSE
    PENALTY_KICK    = 14,   // -> SemanticPlayState::SET_PIECE_OFFENSE
    THROW_IN        = 15,   // -> SemanticPlayState::SET_PIECE_OFFENSE
    LINEOUT         = 16,   // -> SemanticPlayState::SET_PIECE_DEFENSE
    SCRUM           = 17,   // -> SemanticPlayState::SET_PIECE_NEUTRAL
    FACEOFF         = 18,   // -> SemanticPlayState::SET_PIECE_NEUTRAL
    JUMP_BALL       = 19,   // -> SemanticPlayState::SET_PIECE_NEUTRAL
    POSSESSION      = 20,   // -> SemanticPlayState::LIVE_PLAY
    INBOUND         = 21,   // -> SemanticPlayState::SET_PIECE_OFFENSE
    KICKOFF_RECEIVE = 22,   // -> SemanticPlayState::SET_PIECE_NEUTRAL
    EXTRA_POINT     = 23,   // -> SemanticPlayState::SET_PIECE_OFFENSE
    TWO_POINT_CONV  = 24,   // -> SemanticPlayState::SET_PIECE_OFFENSE
    PUNT_PLAY       = 25,   // -> SemanticPlayState::SET_PIECE_OFFENSE
    FIELD_GOAL      = 26    // -> SemanticPlayState::SET_PIECE_OFFENSE
};

// =============================================================================
// [LEGACY] FoulType — Sport-specific foul classification
// =============================================================================
// Retained for DisciplineSystem internal use.
// AI layers should consume SemanticFoulConsequence instead.
// =============================================================================
enum class FoulType : uint8_t {
    NONE                = 0,
    TACKLE_FROM_BEHIND  = 1,
    HIGH_TACKLE         = 2,
    LATE_TACKLE         = 3,
    DANGEROUS_PLAY      = 4,
    HAND_BALL           = 5,   // Soccer
    OFFSIDE             = 6,   // Soccer
    CHARGING            = 7,
    HOLDING             = 8,   // Football, basketball
    BLOCKING_FOUL       = 9,   // Basketball
    PERSONAL_FOUL       = 10,  // Basketball
    TECHNICAL_FOUL      = 11,  // Basketball
    FLAGRANT_FOUL       = 12,  // Basketball
    TRIPPING            = 13,
    ELBOWING            = 14,
    UNSPORTSMANLIKE     = 15,
    DELAY_OF_GAME       = 16,
    TOO_MANY_PLAYERS    = 17,
    WRONG_ZONE          = 18,  // Offside, etc.
    INTENTIONAL_GROUNDING = 19, // American football
    ROUGHING            = 20,  // American football
    PASS_INTERFERENCE   = 21,  // American football
    ICING               = 22,  // Hockey
    OFFSIDES            = 23,  // Hockey
    HIGH_STICK          = 24,  // Hockey
    CROSS_CHECKING      = 25,  // Hockey
    NO_BALL             = 26   // Cricket (LBW, etc.)
};

// =============================================================================
// CardType — Disciplinary actions
// =============================================================================
enum class CardType : uint8_t {
    NONE    = 0,
    YELLOW  = 1,   // Warning / caution
    RED     = 2,   // Sending off
    BLUE    = 3,   // Temporary suspension (e.g., rugby sin bin)
    WHITE   = 4    // Warning (different sport)
};

// =============================================================================
// ScoringEvent — Record of a score
// =============================================================================
struct ScoringEvent {
    uint32_t scorer_team_id = 0;
    uint32_t scorer_athlete_id = 0;
    float points = 0.0f;
    const char* score_type = "goal";  // "goal", "touchdown", "try", etc.
    float game_time = 0.0f;           // When the score happened
    Vec3 position;                     // Where the score happened
    bool valid = true;                 // Was the score upheld?
};

// =============================================================================
// FoulEvent — Record of a foul
// =============================================================================
struct FoulEvent {
    FoulType type = FoulType::NONE;
    CardType card = CardType::NONE;
    uint32_t offending_team_id = 0;
    uint32_t offending_athlete_id = 0;
    uint32_t fouled_team_id = 0;
    uint32_t fouled_athlete_id = 0;
    float game_time = 0.0f;
    Vec3 position;
    PlayState resulting_play_state = PlayState::DEAD_BALL;
    float suspension_time = 0.0f;  // Seconds (for sin bin / ejections)
    bool is_penalty = false;        // Does this result in a penalty?
};

// =============================================================================
// ScoringSystem — Manages scoring rules per sport
// =============================================================================
struct ScoringSystem {
    static constexpr uint32_t MAX_SCORES = 64;

    float scores[2] = {0.0f, 0.0f};  // Home, Away
    ScoringEvent events[MAX_SCORES];
    uint32_t event_count = 0;

    // --- Point values (configurable per sport) ---
    float goal_points = 1.0f;         // Soccer goal
    float try_points = 5.0f;          // Rugby try
    float conversion_points = 2.0f;   // Rugby conversion
    float penalty_goal_points = 3.0f; // Rugby penalty goal
    float drop_goal_points = 3.0f;    // Rugby drop goal
    float touchdown_points = 6.0f;    // American football TD
    float extra_point = 1.0f;         // PAT
    float two_point_conv = 2.0f;
    float field_goal_points = 3.0f;
    float safety_points = 2.0f;
    float free_throw_points = 1.0f;
    float two_point_shot = 2.0f;
    float three_point_shot = 3.0f;
    float behind_points = 1.0f;       // Aussie rules behind
    float goal_minor = 6.0f;          // Aussie rules goal

    void add_score(uint32_t team_id, float points, const char* type,
                    float game_time, const Vec3& pos,
                    uint32_t athlete_id = 0)
    {
        if (team_id < 2) {
            scores[team_id] += points;
        }
        if (event_count < MAX_SCORES) {
            ScoringEvent& ev = events[event_count++];
            ev.scorer_team_id = team_id;
            ev.scorer_athlete_id = athlete_id;
            ev.points = points;
            ev.score_type = type;
            ev.game_time = game_time;
            ev.position = pos;
        }
    }

    // --- Configure for specific sports ---
    void configure_soccer() {
        goal_points = 1.0f;
    }

    void configure_basketball() {
        free_throw_points = 1.0f;
        two_point_shot = 2.0f;
        three_point_shot = 3.0f;
    }

    void configure_rugby_union() {
        try_points = 5.0f;
        conversion_points = 2.0f;
        penalty_goal_points = 3.0f;
        drop_goal_points = 3.0f;
    }

    void configure_american_football() {
        touchdown_points = 6.0f;
        extra_point = 1.0f;
        two_point_conv = 2.0f;
        field_goal_points = 3.0f;
        safety_points = 2.0f;
    }

    void configure_aussie_rules() {
        goal_minor = 6.0f;
        behind_points = 1.0f;
    }

    float get_score_difference() const {
        return scores[0] - scores[1];
    }

    uint32_t get_leading_team() const {
        if (scores[0] > scores[1]) return 0;
        if (scores[1] > scores[0]) return 1;
        return 0xFFFFFFFF; // Tied
    }

    void reset() {
        scores[0] = 0.0f;
        scores[1] = 0.0f;
        event_count = 0;
    }
};

// =============================================================================
// DisciplineSystem — Manages fouls, cards, and suspensions
// =============================================================================
struct DisciplineSystem {
    static constexpr uint32_t MAX_FOULS = 64;

    FoulEvent fouls[MAX_FOULS];
    uint32_t foul_count = 0;

    // Per-team accumulations
    uint32_t team_fouls[2] = {0, 0};
    uint32_t team_yellow_cards[2] = {0, 0};
    uint32_t team_red_cards[2] = {0, 0};

    // Bonus/free throw thresholds
    uint32_t foul_bonus_threshold = 5; // Basketball: 5 team fouls = bonus
    bool team_in_bonus[2] = {false, false};

    // Per-athlete tracking
    static constexpr uint32_t MAX_ATHLETES = MAX_ENTITIES; // 256 (bitmask-scaled)
    uint32_t athlete_fouls[MAX_ATHLETES];
    CardType athlete_cards[MAX_ATHLETES];
    uint32_t athlete_count = 0;

    void reset() {
        foul_count = 0;
        team_fouls[0] = team_fouls[1] = 0;
        team_yellow_cards[0] = team_yellow_cards[1] = 0;
        team_red_cards[0] = team_red_cards[1] = 0;
        team_in_bonus[0] = team_in_bonus[1] = false;
        for (uint32_t i = 0; i < MAX_ATHLETES; ++i) {
            athlete_fouls[i] = 0;
            athlete_cards[i] = CardType::NONE;
        }
        athlete_count = 0;
    }

    void register_athlete(uint32_t athlete_id) {
        if (athlete_id < MAX_ATHLETES) {
            athlete_fouls[athlete_id] = 0;
            athlete_cards[athlete_id] = CardType::NONE;
            if (athlete_id >= athlete_count) {
                athlete_count = athlete_id + 1;
            }
        }
    }

    bool record_foul(FoulType type, uint32_t offending_team,
                      uint32_t offending_athlete,
                      uint32_t fouled_team = 0,
                      uint32_t fouled_athlete = 0,
                      float game_time = 0.0f,
                      const Vec3& position = Vec3(0.0f, 0.0f, 0.0f))
    {
        if (foul_count >= MAX_FOULS) return false;

        FoulEvent& f = fouls[foul_count++];
        f.type = type;
        f.offending_team_id = offending_team;
        f.offending_athlete_id = offending_athlete;
        f.fouled_team_id = fouled_team;
        f.fouled_athlete_id = fouled_athlete;
        f.game_time = game_time;
        f.position = position;

        // Track team fouls
        if (offending_team < 2) {
            team_fouls[offending_team]++;
            if (team_fouls[offending_team] >= foul_bonus_threshold) {
                team_in_bonus[offending_team] = true;
            }
        }

        // Track athlete fouls
        if (offending_athlete < MAX_ATHLETES) {
            athlete_fouls[offending_athlete]++;
        }

        // Determine card
        f.card = determine_card(type, offending_athlete);
        if (f.card == CardType::YELLOW && offending_team < 2) {
            team_yellow_cards[offending_team]++;
        }
        if (f.card == CardType::RED && offending_team < 2) {
            team_red_cards[offending_team]++;
        }

        // Check if this is a penalty
        f.is_penalty = is_penalty_foul(type);
        if (f.is_penalty) {
            f.resulting_play_state = PlayState::PENALTY_KICK;
        } else {
            f.resulting_play_state = PlayState::DEAD_BALL;
        }

        return true;
    }

    CardType determine_card(FoulType type, uint32_t athlete_id) const {
        switch (type) {
        case FoulType::TACKLE_FROM_BEHIND:
        case FoulType::LATE_TACKLE:
        case FoulType::DANGEROUS_PLAY:
        case FoulType::FLAGRANT_FOUL:
            return CardType::RED;
        case FoulType::HIGH_TACKLE:
        case FoulType::UNSPORTSMANLIKE:
        case FoulType::ELBOWING:
            // Second yellow card → red
            if (athlete_id < MAX_ATHLETES && athlete_fouls[athlete_id] >= 4) {
                return CardType::RED;
            }
            return CardType::YELLOW;
        default:
            // First few fouls: no card
            if (athlete_id < MAX_ATHLETES && athlete_fouls[athlete_id] >= 3) {
                return CardType::YELLOW;
            }
            return CardType::NONE;
        }
    }

    bool is_penalty_foul(FoulType type) const {
        return type == FoulType::TACKLE_FROM_BEHIND ||
               type == FoulType::DANGEROUS_PLAY ||
               type == FoulType::HAND_BALL ||
               type == FoulType::FLAGRANT_FOUL;
    }

    bool is_athlete_suspended(uint32_t athlete_id) const {
        if (athlete_id >= MAX_ATHLETES) return false;
        return athlete_cards[athlete_id] == CardType::RED;
    }
};

// =============================================================================
// ClockSystem — Game time management
// =============================================================================
struct ClockSystem {
    float game_time = 0.0f;           // Current game time (seconds)
    float period_duration = 2700.0f;  // Duration of a period (45 min = 2700s)
    float total_duration = 5400.0f;   // Total game duration (2x45 = 5400s)
    uint32_t current_period = 1;      // Current period (1, 2, 3, 4)
    uint32_t total_periods = 2;       // Number of periods

    float shot_clock = 0.0f;          // Basketball shot clock
    float shot_clock_duration = 24.0f; // 24 seconds
    bool shot_clock_active = false;

    float play_clock = 0.0f;          // American football play clock
    float play_clock_duration = 40.0f;

    uint32_t timeouts_remaining[2] = {3, 3}; // Timeouts per team
    float timeout_duration = 60.0f;

    bool is_running = false;
    bool is_stopped = false;
    float accumulated_stoppage = 0.0f; // Injury time, etc.

    void start() {
        is_running = true;
        is_stopped = false;
    }

    void stop() {
        is_running = false;
        is_stopped = true;
    }

    void update(float dt) {
        if (!is_running) return;

        game_time += dt;

        // Period check
        float period_end = period_duration * static_cast<float>(current_period);
        if (game_time >= period_end && current_period < total_periods) {
            // Period ended (handled by rules engine)
        }

        // Shot clock
        if (shot_clock_active) {
            shot_clock -= dt;
            if (shot_clock <= 0.0f) {
                shot_clock = 0.0f;
                // Shot clock violation (handled by rules engine)
            }
        }

        // Play clock
        if (play_clock > 0.0f) {
            play_clock -= dt;
            if (play_clock <= 0.0f) {
                play_clock = 0.0f;
            }
        }
    }

    void reset_shot_clock() {
        shot_clock = shot_clock_duration;
        shot_clock_active = true;
    }

    void reset_play_clock() {
        play_clock = play_clock_duration;
    }

    bool is_period_over() const {
        float period_end = period_duration * static_cast<float>(current_period);
        return game_time >= period_end;
    }

    bool is_game_over() const {
        return game_time >= total_duration;
    }

    float get_time_remaining() const {
        return total_duration - game_time;
    }

    float get_period_time_remaining() const {
        float period_end = period_duration * static_cast<float>(current_period);
        return period_end - game_time;
    }

    void configure_soccer() {
        period_duration = 2700.0f;
        total_duration = 5400.0f;
        total_periods = 2;
        shot_clock_active = false;
        play_clock = 0.0f;
        timeouts_remaining[0] = timeouts_remaining[1] = 3;
    }

    void configure_basketball() {
        period_duration = 720.0f;  // 12 min quarters
        total_duration = 2880.0f; // 4x12 = 48 min
        total_periods = 4;
        shot_clock_duration = 24.0f;
        play_clock = 0.0f;
        timeouts_remaining[0] = timeouts_remaining[1] = 7;
    }

    void configure_american_football() {
        period_duration = 900.0f;  // 15 min quarters
        total_duration = 3600.0f; // 4x15 = 60 min
        total_periods = 4;
        shot_clock_active = false;
        play_clock_duration = 40.0f;
        play_clock = 40.0f;
        timeouts_remaining[0] = timeouts_remaining[1] = 3;
    }
};

} // namespace apc
