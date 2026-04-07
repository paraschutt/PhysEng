#pragma once
// =============================================================================
// Sprint 19: Sport HUD — Heads-up display for sport state visualization
// =============================================================================
//
// Provides:
//   - HUDAnchor: screen position anchors
//   - HUDElement: position, size, text, colors, visibility
//   - SportHUDConfig: toggles and parameters for HUD rendering
//   - SportHUD: processes sport state into HUD draw elements
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation (fixed-capacity arrays)
//   - C++17
//   - All positions in screen-space (x, y in pixels / normalized)
//
// =============================================================================

#include "apc_render_types.h"
#include "apc_debug_draw.h"
#include "apc_sport/apc_sport_field.h"
#include "apc_sport/apc_sport_rules.h"
#include "apc_sport/apc_ball_control.h"
#include "apc_sport/apc_ball_physics.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_math_common.h"
#include <cstdint>
#include <cmath>

namespace apc {

// =============================================================================
// HUDAnchor — Screen position anchor points
// =============================================================================
enum class HUDAnchor : uint8_t {
    TOP_LEFT      = 0,
    TOP_CENTER    = 1,
    TOP_RIGHT     = 2,
    CENTER_LEFT   = 3,
    CENTER        = 4,
    CENTER_RIGHT  = 5,
    BOTTOM_LEFT   = 6,
    BOTTOM_CENTER = 7,
    BOTTOM_RIGHT  = 8
};

// =============================================================================
// HUDElement — Single HUD element (text box)
// =============================================================================
struct HUDElement {
    float x = 0.0f;                    // Screen X position
    float y = 0.0f;                    // Screen Y position
    float width = 100.0f;              // Element width
    float height = 30.0f;              // Element height
    const char* text = "";             // Display text
    RenderColor bg_color = RenderColor(0.0f, 0.0f, 0.0f, 0.7f);
    RenderColor text_color = RenderColor::WHITE();
    bool visible = true;
    HUDAnchor anchor = HUDAnchor::TOP_LEFT;
    uint32_t sort_order = 0u;          // Drawing order (lower = drawn first)

    void set_position(float px, float py, HUDAnchor anch) {
        x = px;
        y = py;
        anchor = anch;
    }

    void set_text(const char* t, const RenderColor& fg = RenderColor::WHITE()) {
        text = t;
        text_color = fg;
    }

    void show() { visible = true; }
    void hide() { visible = false; }
};

// =============================================================================
// SportHUDConfig — Toggles and parameters for HUD
// =============================================================================
struct SportHUDConfig {
    bool show_scoreboard = true;
    bool show_clock = true;
    bool show_period = true;
    bool show_possession = true;
    bool show_fouls = false;
    bool show_play_state = true;
    bool show_shot_clock = false;
    bool show_play_clock = false;
    bool show_timeout_count = false;
    bool show_score_event = true;
    float font_size = 24.0f;
    RenderColor home_team_color = RenderColor::BLUE();
    RenderColor away_team_color = RenderColor::RED();
    HUDAnchor scoreboard_anchor = HUDAnchor::TOP_CENTER;
    HUDAnchor clock_anchor = HUDAnchor::TOP_CENTER;
    float score_event_flash_duration = 3.0f;

    static SportHUDConfig make_default() {
        SportHUDConfig c;
        c.show_scoreboard = true;
        c.show_clock = true;
        c.show_period = true;
        c.show_possession = true;
        c.show_fouls = false;
        c.show_play_state = true;
        c.show_shot_clock = false;
        c.show_play_clock = false;
        c.show_timeout_count = false;
        c.show_score_event = true;
        c.font_size = 24.0f;
        c.home_team_color = RenderColor::BLUE();
        c.away_team_color = RenderColor::RED();
        c.scoreboard_anchor = HUDAnchor::TOP_CENTER;
        c.clock_anchor = HUDAnchor::TOP_CENTER;
        c.score_event_flash_duration = 3.0f;
        return c;
    }

    static SportHUDConfig make_basketball() {
        SportHUDConfig c = make_default();
        c.show_shot_clock = true;
        c.show_fouls = true;
        c.show_timeout_count = true;
        return c;
    }

    static SportHUDConfig make_american_football() {
        SportHUDConfig c = make_default();
        c.show_play_clock = true;
        c.show_timeout_count = true;
        return c;
    }
};

// =============================================================================
// ScoreEventFlash — Score event notification
// =============================================================================
struct ScoreEventFlash {
    bool active = false;
    float trigger_time = 0.0f;
    const char* score_type = "";
    uint32_t team_id = 0;
    float points = 0.0f;
    float duration = 3.0f;
};

// =============================================================================
// SportHUD — Processes sport state into HUD draw elements
// =============================================================================
struct SportHUD {
    static constexpr uint32_t MAX_ELEMENTS = 32u;
    static constexpr uint32_t MAX_FLASHES = 8u;

    SportHUDConfig config;
    HUDElement elements[MAX_ELEMENTS];
    uint32_t element_count = 0u;
    ScoreEventFlash flashes[MAX_FLASHES];
    uint32_t flash_count = 0u;
    float current_time = 0.0f;

    // Cached state
    float scores[2] = {0.0f, 0.0f};
    float game_time = 0.0f;
    uint32_t current_period = 1u;
    PlayState play_state = PlayState::NOT_STARTED;
    uint32_t possession_team = 0u;
    float shot_clock = 0.0f;
    float play_clock = 0.0f;
    uint32_t timeouts_remaining[2] = {3u, 3u};
    uint32_t team_fouls[2] = {0u, 0u};

    /// Set HUD configuration.
    void set_config(const SportHUDConfig& cfg) { config = cfg; }

    /// Process sport state from ScoringSystem, ClockSystem, DisciplineSystem.
    void process_sport_state(const ScoringSystem& scoring,
                              const ClockSystem& clock,
                              uint32_t poss_team = 0u) {
        scores[0] = scoring.scores[0];
        scores[1] = scoring.scores[1];
        game_time = clock.game_time;
        current_period = clock.current_period;
        shot_clock = clock.shot_clock;
        play_clock = clock.play_clock;
        timeouts_remaining[0] = clock.timeouts_remaining[0];
        timeouts_remaining[1] = clock.timeouts_remaining[1];
        possession_team = poss_team;

        // Check for new scoring events
        if (config.show_score_event) {
            for (uint32_t i = 0u; i < scoring.event_count; ++i) {
                const ScoringEvent& ev = scoring.events[i];
                if (ev.valid && flash_count < MAX_FLASHES) {
                    bool already_flashed = false;
                    for (uint32_t j = 0u; j < flash_count; ++j) {
                        if (flashes[j].trigger_time >= ev.game_time - 0.001f &&
                            flashes[j].trigger_time <= ev.game_time + 0.001f) {
                            already_flashed = true;
                            break;
                        }
                    }
                    if (!already_flashed) {
                        ScoreEventFlash& f = flashes[flash_count++];
                        f.active = true;
                        f.trigger_time = ev.game_time;
                        f.score_type = ev.score_type;
                        f.team_id = ev.scorer_team_id;
                        f.points = ev.points;
                        f.duration = config.score_event_flash_duration;
                    }
                }
            }
        }
    }

    /// Update HUD time and expire flashes.
    void update(float dt) {
        current_time += dt;

        for (uint32_t i = 0u; i < flash_count; ++i) {
            if (flashes[i].active) {
                float elapsed = current_time - flashes[i].trigger_time;
                if (elapsed > flashes[i].duration) {
                    flashes[i].active = false;
                }
            }
        }
    }

    // --- Element generation ---

    /// Generate scoreboard element (e.g., "HOME 2 - 1 AWAY").
    void draw_scoreboard(DebugDraw& dd) {
        if (!config.show_scoreboard) return;

        // Scoreboard as text element (visual: two score boxes)
        float sb_x = 0.0f;
        float sb_y = 0.0f;
        float sb_w = 200.0f;
        float sb_h = 40.0f;

        // Home score background
        dd.list.add_line(
            Vec3(sb_x - sb_w * 0.5f, 0.0f, sb_y),
            Vec3(sb_x - 1.0f, 0.0f, sb_y),
            config.home_team_color);

        // Away score background
        dd.list.add_line(
            Vec3(sb_x + 1.0f, 0.0f, sb_y),
            Vec3(sb_x + sb_w * 0.5f, 0.0f, sb_y),
            config.away_team_color);

        // Score indicator lines (height proportional to score)
        float max_display_score = 10.0f;
        float home_h = std::min(scores[0] / max_display_score, 1.0f) * sb_h;
        float away_h = std::min(scores[1] / max_display_score, 1.0f) * sb_h;

        dd.list.add_line(
            Vec3(sb_x - sb_w * 0.25f, 0.0f, sb_y),
            Vec3(sb_x - sb_w * 0.25f, 0.0f, sb_y + home_h),
            config.home_team_color);

        dd.list.add_line(
            Vec3(sb_x + sb_w * 0.25f, 0.0f, sb_y),
            Vec3(sb_x + sb_w * 0.25f, 0.0f, sb_y + away_h),
            config.away_team_color);
    }

    /// Generate match clock display.
    void draw_match_clock(DebugDraw& dd) {
        if (!config.show_clock) return;

        // Clock as a line of proportional length
        float total_duration = 5400.0f; // Default 90 min
        float progress = (game_time < total_duration) ? game_time / total_duration : 1.0f;
        float clock_w = 150.0f * progress;

        RenderColor clock_color = (play_state == PlayState::LIVE) ?
            RenderColor::GREEN() : RenderColor::RED();

        dd.list.add_line(
            Vec3(-75.0f, 0.0f, -25.0f),
            Vec3(-75.0f + clock_w, 0.0f, -25.0f),
            clock_color);
    }

    /// Generate period indicator.
    void draw_period_indicator(DebugDraw& dd) {
        if (!config.show_period) return;

        // Period dots
        float dot_spacing = 10.0f;
        float start_x = -dot_spacing;

        for (uint32_t i = 0u; i < current_period && i < 10u; ++i) {
            dd.list.add_point(
                Vec3(start_x + static_cast<float>(i) * dot_spacing, 0.0f, -20.0f),
                RenderColor::WHITE(), 3.0f);
        }
    }

    /// Generate shot and play clock displays.
    void draw_shot_play_clocks(DebugDraw& dd) {
        if (config.show_shot_clock && shot_clock > 0.0f) {
            float shot_progress = shot_clock / 24.0f;
            dd.list.add_line(
                Vec3(80.0f, 0.0f, -25.0f),
                Vec3(80.0f + 50.0f * shot_progress, 0.0f, -25.0f),
                RenderColor::YELLOW());
        }

        if (config.show_play_clock && play_clock > 0.0f) {
            float play_progress = play_clock / 40.0f;
            dd.list.add_line(
                Vec3(-130.0f, 0.0f, -25.0f),
                Vec3(-130.0f + 50.0f * play_progress, 0.0f, -25.0f),
                RenderColor::ORANGE());
        }
    }

    /// Generate possession arrow.
    void draw_possession_arrow(DebugDraw& dd) {
        if (!config.show_possession) return;

        RenderColor poss_color = (possession_team == 1u) ?
            config.home_team_color : config.away_team_color;

        float arrow_x = (possession_team == 1u) ? -20.0f : 20.0f;
        float arrow_y = -15.0f;

        // Arrow shape (3 lines)
        dd.list.add_line(Vec3(arrow_x, 0.0f, arrow_y),
                        Vec3(arrow_x + 10.0f, 0.0f, arrow_y), poss_color);
        dd.list.add_line(Vec3(arrow_x + 10.0f, 0.0f, arrow_y),
                        Vec3(arrow_x + 7.0f, 0.0f, arrow_y + 3.0f), poss_color);
        dd.list.add_line(Vec3(arrow_x + 10.0f, 0.0f, arrow_y),
                        Vec3(arrow_x + 7.0f, 0.0f, arrow_y - 3.0f), poss_color);
    }

    /// Generate play state indicator.
    void draw_play_state_indicator(DebugDraw& dd) {
        if (!config.show_play_state) return;

        RenderColor state_color = RenderColor::WHITE();
        switch (play_state) {
        case PlayState::LIVE:
            state_color = RenderColor::GREEN();
            break;
        case PlayState::DEAD_BALL:
        case PlayState::FREE_KICK:
        case PlayState::CORNER_KICK:
        case PlayState::PENALTY_KICK:
        case PlayState::THROW_IN:
            state_color = RenderColor::YELLOW();
            break;
        case PlayState::GOAL_SCORED:
            state_color = RenderColor::CYAN();
            break;
        case PlayState::TIMEOUT:
        case PlayState::INJURY_STOP:
            state_color = RenderColor::RED();
            break;
        case PlayState::GAME_OVER:
            state_color = RenderColor::GRAY();
            break;
        default:
            state_color = RenderColor::WHITE();
            break;
        }

        // Play state indicator dot
        dd.list.add_point(Vec3(0.0f, 0.0f, -30.0f), state_color, 5.0f);
    }

    /// Generate foul count display.
    void draw_foul_count(DebugDraw& dd) {
        if (!config.show_fouls) return;

        // Team foul indicators as small dots
        for (uint32_t t = 0u; t < 2u; ++t) {
            float base_x = (t == 0u) ? -60.0f : 40.0f;
            RenderColor fc = (t == 0u) ? config.home_team_color : config.away_team_color;
            uint32_t fouls = team_fouls[t];
            for (uint32_t i = 0u; i < fouls && i < 10u; ++i) {
                dd.list.add_point(
                    Vec3(base_x + static_cast<float>(i) * 5.0f, 0.0f, -35.0f),
                    fc, 2.0f);
            }
        }
    }

    /// Generate timeout count display.
    void draw_timeout_count(DebugDraw& dd) {
        if (!config.show_timeout_count) return;

        for (uint32_t t = 0u; t < 2u; ++t) {
            float base_x = (t == 0u) ? -60.0f : 40.0f;
            RenderColor tc = (t == 0u) ? config.home_team_color : config.away_team_color;
            uint32_t timeouts = timeouts_remaining[t];
            for (uint32_t i = 0u; i < timeouts && i < 7u; ++i) {
                dd.list.add_point(
                    Vec3(base_x + static_cast<float>(i) * 5.0f, 0.0f, -40.0f),
                    tc, 2.0f);
            }
        }
    }

    /// Generate score event flash.
    void draw_score_event(DebugDraw& dd) {
        if (!config.show_score_event) return;

        for (uint32_t i = 0u; i < flash_count; ++i) {
            const ScoreEventFlash& f = flashes[i];
            if (!f.active) continue;

            float elapsed = current_time - f.trigger_time;
            float t = 1.0f - (elapsed / f.duration);
            if (t <= 0.0f) continue;

            RenderColor flash_color = (f.team_id == 0u) ?
                config.home_team_color : config.away_team_color;
            flash_color.a = t;

            // Expanding ring effect
            float ring_r = 30.0f * (1.0f - t);
            uint32_t seg = 12u;
            for (uint32_t s = 0u; s < seg; ++s) {
                float a0 = static_cast<float>(s) / static_cast<float>(seg) * APC_TWO_PI;
                float a1 = static_cast<float>(s + 1) / static_cast<float>(seg) * APC_TWO_PI;
                Vec3 p0(0.0f + ring_r * std::cos(a0), 0.0f, 0.0f + ring_r * std::sin(a0));
                Vec3 p1(0.0f + ring_r * std::cos(a1), 0.0f, 0.0f + ring_r * std::sin(a1));
                dd.list.add_line(p0, p1, flash_color);
            }
        }
    }

    /// Generate all HUD elements.
    void draw_all(DebugDraw& dd) {
        draw_scoreboard(dd);
        draw_match_clock(dd);
        draw_period_indicator(dd);
        draw_shot_play_clocks(dd);
        draw_possession_arrow(dd);
        draw_play_state_indicator(dd);
        draw_foul_count(dd);
        draw_timeout_count(dd);
        draw_score_event(dd);
    }

    /// Reset all state.
    void reset() {
        element_count = 0u;
        flash_count = 0u;
        current_time = 0.0f;
        scores[0] = scores[1] = 0.0f;
        game_time = 0.0f;
        current_period = 1u;
        play_state = PlayState::NOT_STARTED;
        possession_team = 0u;
        shot_clock = 0.0f;
        play_clock = 0.0f;
        timeouts_remaining[0] = timeouts_remaining[1] = 3u;
        team_fouls[0] = team_fouls[1] = 0u;
    }
};

} // namespace apc
