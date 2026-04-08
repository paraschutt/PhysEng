#pragma once
// =============================================================================
// apc_sdl2_renderer.h — SDL2 software rendering backend for APC Physics Engine
// =============================================================================
//
// Provides a concrete 2D top-down rendering backend using SDL2:
//
//   - SDL2Color: RGBA color with pack/unpack for SDL2 pixel format
//   - SDL2Camera: Orthographic top-down camera with follow-ball support
//   - SDL2Renderer: Main renderer — field, athletes, ball, debug, HUD
//
// Projection: World X,Z -> Screen X,Y (top-down, Z maps to screen Y).
//   screen_x = (world_x - cam_x) * scale + window_width / 2
//   screen_y = (world_z - cam_z) * scale + window_height / 2
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation (no new/malloc)
//   - C++17
//   - Uses SDL2 software renderer
//
// =============================================================================

#include <SDL2/SDL.h>
#include "apc_render_types.h"
#include "apc_debug_draw.h"
#include "apc_app/apc_application.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_math_common.h"
#include <cstdint>
#include <cstdio>
#include <cmath>

namespace apc {

// =============================================================================
// SDL2Color — RGBA color
// =============================================================================
struct SDL2Color {
    uint8_t r = 255;
    uint8_t g = 255;
    uint8_t b = 255;
    uint8_t a = 255;

    SDL2Color() = default;
    SDL2Color(uint8_t r_, uint8_t g_, uint8_t b_, uint8_t a_ = 255)
        : r(r_), g(g_), b(b_), a(a_) {}

    static SDL2Color from_render(const RenderColor& c) {
        SDL2Color out;
        out.r = static_cast<uint8_t>(c.r * 255.0f + 0.5f);
        out.g = static_cast<uint8_t>(c.g * 255.0f + 0.5f);
        out.b = static_cast<uint8_t>(c.b * 255.0f + 0.5f);
        out.a = static_cast<uint8_t>(c.a * 255.0f + 0.5f);
        return out;
    }

    // Named color constants
    static SDL2Color GRASS()        { return SDL2Color(31, 127, 31, 255); }
    static SDL2Color GRASS_STRIPE() { return SDL2Color(28, 112, 28, 255); }
    static SDL2Color LINE_WHITE()   { return SDL2Color(240, 240, 240, 255); }
    static SDL2Color HOME_BLUE()    { return SDL2Color(50, 100, 220, 255); }
    static SDL2Color AWAY_RED()     { return SDL2Color(220, 50, 50, 255); }
    static SDL2Color BALL_WHITE()   { return SDL2Color(255, 255, 255, 255); }
    static SDL2Color HUD_BG()       { return SDL2Color(0, 0, 0, 180); }
    static SDL2Color HUD_TEXT()     { return SDL2Color(255, 255, 255, 255); }
    static SDL2Color GOAL_POST()    { return SDL2Color(255, 255, 255, 255); }
    static SDL2Color CORNER_FLAG()  { return SDL2Color(255, 255, 0, 255); }
    static SDL2Color SHADOW()       { return SDL2Color(0, 0, 0, 60); }
};

// =============================================================================
// SDL2Camera — Orthographic top-down camera
// =============================================================================
struct SDL2Camera {
    float cam_x = 0.0f;
    float cam_z = 0.0f;
    float scale = 6.0f;           // Pixels per meter
    uint32_t window_width = 1280;
    uint32_t window_height = 720;
    uint8_t follow_ball = 1;

    float world_to_screen_x(float world_x) const {
        return (world_x - cam_x) * scale + static_cast<float>(window_width) * 0.5f;
    }

    float world_to_screen_y(float world_z) const {
        return (world_z - cam_z) * scale + static_cast<float>(window_height) * 0.5f;
    }

    void auto_fit(float field_length, float field_width) {
        float pad = 1.15f;
        float sx = static_cast<float>(window_width) / (field_length * pad);
        float sy = static_cast<float>(window_height) / (field_width * pad);
        scale = (sx < sy) ? sx : sy;
        if (scale < 1.0f) scale = 1.0f;
    }

    void update_follow(const BallEntity* ball, float dt) {
        if (!follow_ball || !ball) return;
        if (!ball->id.is_valid()) return;
        float lerp_speed = 3.0f * dt;
        if (lerp_speed > 1.0f) lerp_speed = 1.0f;
        cam_x += (ball->position.x - cam_x) * lerp_speed;
        cam_z += (ball->position.z - cam_z) * lerp_speed;
    }

    void zoom(float delta) {
        scale += delta;
        if (scale < 2.0f) scale = 2.0f;
        if (scale > 30.0f) scale = 30.0f;
    }

    void pan(float dx, float dz) {
        cam_x += dx / scale;
        cam_z += dz / scale;
    }
};

// =============================================================================
// SDL2BitmapFont — Simple 3x5 bitmap font for jersey numbers
// =============================================================================
struct SDL2BitmapFont {
    // Each digit is 3 columns x 5 rows, stored as 3 uint8_t (bit-packed)
    // Bit 0 = top pixel, bit 4 = bottom pixel
    static constexpr uint8_t DIGITS[10][3] = {
        { 0b11101u, 0b10101u, 0b11101u }, // 0
        { 0b01001u, 0b11001u, 0b01001u }, // 1
        { 0b11101u, 0b00101u, 0b11101u }, // 2
        { 0b11101u, 0b00101u, 0b11101u }, // 3
        { 0b10101u, 0b11101u, 0b00101u }, // 4
        { 0b11101u, 0b10001u, 0b11101u }, // 5
        { 0b11101u, 0b10001u, 0b11101u }, // 6
        { 0b11101u, 0b00101u, 0b00101u }, // 7
        { 0b11101u, 0b10101u, 0b11101u }, // 8
        { 0b11101u, 0b10101u, 0b11101u }, // 9
    };

    static void draw_digit(uint8_t digit, float sx, float sy,
                            float pixel_size, SDL2Color color,
                            SDL_Renderer* renderer)
    {
        if (digit > 9u) digit = 9u;
        for (int col = 0; col < 3; ++col) {
            uint8_t col_bits = DIGITS[digit][col];
            for (int row = 0; row < 5; ++row) {
                if (col_bits & (1u << row)) {
                    int px = static_cast<int>(sx + static_cast<float>(col) * pixel_size);
                    int py = static_cast<int>(sy + static_cast<float>(row) * pixel_size);
                    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
                    SDL_Rect r = { px, py, static_cast<int>(pixel_size), static_cast<int>(pixel_size) };
                    SDL_RenderFillRect(renderer, &r);
                }
            }
        }
    }

    static void draw_number(uint8_t number, float cx, float cy,
                             float pixel_size, SDL2Color color,
                             SDL_Renderer* renderer)
    {
        if (number > 99u) number = 99u;

        if (number >= 10u) {
            uint8_t tens = number / 10u;
            uint8_t ones = number % 10u;
            float char_w = 3.0f * pixel_size;
            float gap = 1.0f * pixel_size;
            float total_w = 2.0f * char_w + gap;
            float start_x = cx - total_w * 0.5f;
            draw_digit(tens, start_x, cy - 2.5f * pixel_size, pixel_size, color, renderer);
            draw_digit(ones, start_x + char_w + gap, cy - 2.5f * pixel_size, pixel_size, color, renderer);
        } else {
            float char_w = 3.0f * pixel_size;
            draw_digit(number, cx - char_w * 0.5f, cy - 2.5f * pixel_size, pixel_size, color, renderer);
        }
    }
};

// =============================================================================
// SDL2Renderer — Main SDL2 rendering backend
// =============================================================================
struct SDL2Renderer {
    SDL_Window*   window   = nullptr;
    SDL_Renderer* renderer = nullptr;
    SDL2Camera    camera;
    int           window_w = 1280;
    int           window_h = 720;
    uint8_t       initialized = 0;

    // FPS tracking
    float    fps_timer = 0.0f;
    uint32_t fps_frame_count = 0u;
    float    display_fps = 0.0f;

    // =========================================================================
    // Primitive drawing
    // =========================================================================

    void draw_line_2d(int x1, int y1, int x2, int y2, const SDL2Color& color) {
        if (!renderer) return;
        SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
        SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
    }

    void draw_circle_filled(int cx, int cy, int radius, const SDL2Color& color) {
        if (!renderer || radius <= 0) return;
        SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
        // Midpoint circle algorithm — filled via horizontal scan lines
        int x = radius;
        int y = 0;
        int err = 1 - radius;
        while (x >= y) {
            SDL_RenderDrawLine(renderer, cx - x, cy + y, cx + x, cy + y);
            SDL_RenderDrawLine(renderer, cx - x, cy - y, cx + x, cy - y);
            SDL_RenderDrawLine(renderer, cx - y, cy + x, cx + y, cy + x);
            SDL_RenderDrawLine(renderer, cx - y, cy - x, cx + y, cy - x);
            ++y;
            if (err < 0) {
                err += 2 * y + 1;
            } else {
                --x;
                err += 2 * (y - x) + 1;
            }
        }
    }

    void draw_circle_outline(int cx, int cy, int radius, const SDL2Color& color) {
        if (!renderer || radius <= 0) return;
        SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
        int x = radius;
        int y = 0;
        int err = 1 - radius;
        while (x >= y) {
            SDL_RenderDrawPoint(renderer, cx + x, cy + y);
            SDL_RenderDrawPoint(renderer, cx - x, cy + y);
            SDL_RenderDrawPoint(renderer, cx + x, cy - y);
            SDL_RenderDrawPoint(renderer, cx - x, cy - y);
            SDL_RenderDrawPoint(renderer, cx + y, cy + x);
            SDL_RenderDrawPoint(renderer, cx - y, cy + x);
            SDL_RenderDrawPoint(renderer, cx + y, cy - x);
            SDL_RenderDrawPoint(renderer, cx - y, cy - x);
            ++y;
            if (err < 0) {
                err += 2 * y + 1;
            } else {
                --x;
                err += 2 * (y - x) + 1;
            }
        }
    }

    void draw_rect_filled(int x, int y, int w, int h, const SDL2Color& color) {
        if (!renderer) return;
        SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
        SDL_Rect r = { x, y, w, h };
        SDL_RenderFillRect(renderer, &r);
    }

    void draw_rect_outline(int x, int y, int w, int h, const SDL2Color& color) {
        if (!renderer) return;
        SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
        SDL_Rect r = { x, y, w, h };
        SDL_RenderDrawRect(renderer, &r);
    }

    // =========================================================================
    // Field rendering
    // =========================================================================

    void draw_field(const MatchConfig& config) {
        float half_l = config.field_length * 0.5f;
        float half_w = config.field_width * 0.5f;

        // Field grass background
        int x1 = static_cast<int>(camera.world_to_screen_x(-half_l));
        int y1 = static_cast<int>(camera.world_to_screen_y(-half_w));
        int x2 = static_cast<int>(camera.world_to_screen_x(half_l));
        int y2 = static_cast<int>(camera.world_to_screen_y(half_w));

        int fw = x2 - x1;
        int fh = y2 - y1;
        if (fw > 0 && fh > 0) {
            draw_rect_filled(x1, y1, fw, fh, SDL2Color::GRASS());
            // Alternating grass stripes
            float stripe_w = config.field_length / 16.0f;
            for (int i = 0; i < 16; i += 2) {
                float sx = -half_l + static_cast<float>(i) * stripe_w;
                int sx1 = static_cast<int>(camera.world_to_screen_x(sx));
                int sx2 = static_cast<int>(camera.world_to_screen_x(sx + stripe_w));
                int sw = sx2 - sx1;
                if (sw > 0) {
                    draw_rect_filled(sx1, y1, sw, fh, SDL2Color::GRASS_STRIPE());
                }
            }
        }

        // Boundary lines
        SDL2Color lc = SDL2Color::LINE_WHITE();
        draw_line_2d(x1, y1, x2, y1, lc);
        draw_line_2d(x2, y1, x2, y2, lc);
        draw_line_2d(x2, y2, x1, y2, lc);
        draw_line_2d(x1, y2, x1, y1, lc);

        // Half-way line
        int hx = static_cast<int>(camera.world_to_screen_x(0.0f));
        draw_line_2d(hx, y1, hx, y2, lc);

        // Center circle
        float center_r_px = 9.15f * camera.scale;
        int ccx = static_cast<int>(camera.world_to_screen_x(0.0f));
        int ccy = static_cast<int>(camera.world_to_screen_y(0.0f));
        draw_circle_outline(ccx, ccy, static_cast<int>(center_r_px), lc);
        draw_circle_filled(ccx, ccy, 3, lc);

        // Penalty areas
        float pa_w = 40.32f * 0.5f;
        float pa_d = 16.5f;
        float ga_w = 18.32f * 0.5f;
        float ga_d = 5.5f;

        // Left penalty area
        int lx = static_cast<int>(camera.world_to_screen_x(-half_l));
        int lpd = static_cast<int>(camera.world_to_screen_x(-half_l + pa_d));
        int lpaw_n = static_cast<int>(camera.world_to_screen_y(-pa_w));
        int lpaw_p = static_cast<int>(camera.world_to_screen_y(pa_w));
        draw_line_2d(lx, lpaw_n, lpd, lpaw_n, lc);
        draw_line_2d(lpd, lpaw_n, lpd, lpaw_p, lc);
        draw_line_2d(lpd, lpaw_p, lx, lpaw_p, lc);

        // Right penalty area
        int rx = static_cast<int>(camera.world_to_screen_x(half_l));
        int rpd = static_cast<int>(camera.world_to_screen_x(half_l - pa_d));
        draw_line_2d(rpd, lpaw_n, rx, lpaw_n, lc);
        draw_line_2d(rx, lpaw_n, rx, lpaw_p, lc);
        draw_line_2d(rx, lpaw_p, rpd, lpaw_p, lc);

        // Goal areas (6-yard box)
        int lgd = static_cast<int>(camera.world_to_screen_x(-half_l + ga_d));
        int rgd = static_cast<int>(camera.world_to_screen_x(half_l - ga_d));
        int lgaw_n = static_cast<int>(camera.world_to_screen_y(-ga_w));
        int lgaw_p = static_cast<int>(camera.world_to_screen_y(ga_w));
        draw_line_2d(lx, lgaw_n, lgd, lgaw_n, lc);
        draw_line_2d(lgd, lgaw_n, lgd, lgaw_p, lc);
        draw_line_2d(lgd, lgaw_p, lx, lgaw_p, lc);
        draw_line_2d(rgd, lgaw_n, rx, lgaw_n, lc);
        draw_line_2d(rx, lgaw_n, rx, lgaw_p, lc);
        draw_line_2d(rx, lgaw_p, rgd, lgaw_p, lc);

        // Penalty spots
        int lps = static_cast<int>(camera.world_to_screen_x(-half_l + 11.0f));
        int rps = static_cast<int>(camera.world_to_screen_x(half_l - 11.0f));
        draw_circle_filled(lps, ccy, 3, lc);
        draw_circle_filled(rps, ccy, 3, lc);

        // Penalty arcs (simplified circles)
        int arc_r = static_cast<int>(9.15f * camera.scale);
        draw_circle_outline(lps, ccy, arc_r, lc);
        draw_circle_outline(rps, ccy, arc_r, lc);

        // Goal structures
        float goal_w = 7.32f;
        float goal_depth = 2.44f;
        SDL2Color gc = SDL2Color::GOAL_POST();

        // Left goal
        int lg_left = static_cast<int>(camera.world_to_screen_x(-half_l - goal_depth));
        int lg_right = lx;
        int lg_top = static_cast<int>(camera.world_to_screen_y(-goal_w * 0.5f));
        int lg_bot = static_cast<int>(camera.world_to_screen_y(goal_w * 0.5f));
        draw_rect_outline(lg_left, lg_top, lg_right - lg_left, lg_bot - lg_top, gc);

        // Right goal
        int rg_left = rx;
        int rg_right = static_cast<int>(camera.world_to_screen_x(half_l + goal_depth));
        draw_rect_outline(rg_left, lg_top, rg_right - rg_left, lg_bot - lg_top, gc);

        // Corner arcs + flags
        float corner_r = 1.0f * camera.scale;
        int cr = static_cast<int>(corner_r);
        int cx_pts[4] = { lx, rx, rx, lx };
        int cy_pts[4] = { y1, y1, y2, y2 };
        for (int i = 0; i < 4; ++i) {
            draw_circle_outline(cx_pts[i], cy_pts[i], cr, lc);
            draw_line_2d(cx_pts[i], cy_pts[i], cx_pts[i], cy_pts[i] - 8,
                        SDL2Color::CORNER_FLAG());
        }
    }

    // =========================================================================
    // Entity rendering
    // =========================================================================

    void draw_athletes(const EntityManager& em) {
        for (uint32_t i = 0u; i < em.athlete_count; ++i) {
            const AthleteEntity& a = em.athletes[i];
            if (!a.id.is_valid() || !a.is_active) continue;

            int sx = static_cast<int>(camera.world_to_screen_x(a.position.x));
            int sy = static_cast<int>(camera.world_to_screen_y(a.position.z));

            int margin = 30;
            if (sx < -margin || sx > window_w + margin ||
                sy < -margin || sy > window_h + margin) continue;

            int radius = static_cast<int>(a.radius * camera.scale);
            if (radius < 4) radius = 4;
            if (radius > 20) radius = 20;

            // Shadow
            draw_circle_filled(sx + 2, sy + 2, radius + 1, SDL2Color::SHADOW());

            // Body
            SDL2Color team_color = (a.team == TEAM_HOME)
                ? SDL2Color::HOME_BLUE() : SDL2Color::AWAY_RED();
            draw_circle_filled(sx, sy, radius, team_color);
            draw_circle_outline(sx, sy, radius, SDL2Color(0, 0, 0, 200));

            // Jersey number
            if (radius >= 6 && a.jersey_number > 0 && renderer) {
                int pix_size = (radius >= 10) ? 2 : 1;
                SDL2BitmapFont::draw_number(
                    a.jersey_number,
                    static_cast<float>(sx),
                    static_cast<float>(sy),
                    static_cast<float>(pix_size),
                    SDL2Color(255, 255, 255, 255),
                    renderer);
            }

            // Velocity indicator
            float speed = Vec3::length(a.velocity);
            if (speed > 0.5f) {
                int vx = static_cast<int>(a.velocity.x / speed * camera.scale * 1.5f);
                int vy = static_cast<int>(a.velocity.z / speed * camera.scale * 1.5f);
                draw_line_2d(sx, sy, sx + vx, sy + vy, SDL2Color(255, 255, 255, 120));
            }
        }
    }

    void draw_ball(const BallEntity* ball) {
        if (!ball || !ball->id.is_valid()) return;

        int sx = static_cast<int>(camera.world_to_screen_x(ball->position.x));
        int sy = static_cast<int>(camera.world_to_screen_y(ball->position.z));

        int radius = static_cast<int>(ball->radius * camera.scale * 2.5f);
        if (radius < 4) radius = 4;
        if (radius > 15) radius = 15;

        draw_circle_filled(sx + 2, sy + 2, radius + 1, SDL2Color::SHADOW());
        draw_circle_filled(sx, sy, radius, SDL2Color::BALL_WHITE());
        draw_circle_outline(sx, sy, radius, SDL2Color(0, 0, 0, 200));
        draw_circle_filled(sx, sy, std::max(1, radius / 3), SDL2Color(80, 80, 80, 255));

        // Velocity trail
        float speed = Vec3::length(ball->velocity);
        if (speed > 2.0f) {
            float trail = std::min(speed * 0.15f, 3.0f) * camera.scale;
            int tx = static_cast<int>(sx - ball->velocity.x / speed * trail);
            int ty = static_cast<int>(sy - ball->velocity.z / speed * trail);
            draw_line_2d(tx, ty, sx, sy, SDL2Color(200, 200, 200, 100));
        }
    }

    // =========================================================================
    // Debug draw list rendering
    // =========================================================================

    void draw_debug_lines(const DebugDrawList& ddl) {
        for (uint32_t i = 0u; i < ddl.line_count; ++i) {
            const DebugLine& l = ddl.lines[i];
            int lx1 = static_cast<int>(camera.world_to_screen_x(l.a.position.x));
            int ly1 = static_cast<int>(camera.world_to_screen_y(l.a.position.z));
            int lx2 = static_cast<int>(camera.world_to_screen_x(l.b.position.x));
            int ly2 = static_cast<int>(camera.world_to_screen_y(l.b.position.z));
            int m = 500;
            if ((lx1 < -m && lx2 < -m) || (lx1 > window_w + m && lx2 > window_w + m) ||
                (ly1 < -m && ly2 < -m) || (ly1 > window_h + m && ly2 > window_h + m))
                continue;
            draw_line_2d(lx1, ly1, lx2, ly2, SDL2Color::from_render(l.a.color));
        }
    }

    void draw_debug_points(const DebugDrawList& ddl) {
        for (uint32_t i = 0u; i < ddl.point_count; ++i) {
            const DebugPoint& p = ddl.points[i];
            int px = static_cast<int>(camera.world_to_screen_x(p.position.x));
            int py = static_cast<int>(camera.world_to_screen_y(p.position.z));
            if (px < -50 || px > window_w + 50 || py < -50 || py > window_h + 50) continue;
            int size = static_cast<int>(p.size * 0.5f);
            if (size < 2) size = 2;
            if (size > 10) size = 10;
            draw_circle_filled(px, py, size, SDL2Color::from_render(p.color));
        }
    }

    void draw_debug_triangles(const DebugDrawList& ddl) {
        for (uint32_t i = 0u; i < ddl.triangle_count; ++i) {
            const DebugTriangle& t = ddl.triangles[i];
            int tx1 = static_cast<int>(camera.world_to_screen_x(t.v0.position.x));
            int ty1 = static_cast<int>(camera.world_to_screen_y(t.v0.position.z));
            int tx2 = static_cast<int>(camera.world_to_screen_x(t.v1.position.x));
            int ty2 = static_cast<int>(camera.world_to_screen_y(t.v1.position.z));
            int tx3 = static_cast<int>(camera.world_to_screen_x(t.v2.position.x));
            int ty3 = static_cast<int>(camera.world_to_screen_y(t.v2.position.z));
            SDL2Color c = SDL2Color::from_render(t.v0.color);
            draw_line_2d(tx1, ty1, tx2, ty2, c);
            draw_line_2d(tx2, ty2, tx3, ty3, c);
            draw_line_2d(tx3, ty3, tx1, ty1, c);
        }
    }

    void draw_debug(const DebugDrawList& ddl) {
        draw_debug_triangles(ddl);
        draw_debug_lines(ddl);
        draw_debug_points(ddl);
    }

    // =========================================================================
    // HUD rendering
    // =========================================================================

    void draw_hud(const Application& app) {
        const SceneState& scene = app.scene;

        // --- Score bar at top ---
        int bar_h = 36;
        draw_rect_filled(0, 0, window_w, bar_h, SDL2Color::HUD_BG());

        int center_x = window_w / 2;
        int score_y = bar_h / 2;

        // Home team blue square
        draw_rect_filled(center_x - 130, score_y - 6, 12, 12, SDL2Color::HOME_BLUE());
        // "Home" label
        draw_rect_filled(center_x - 112, score_y - 3, 24, 6, SDL2Color(180, 180, 180, 255));

        // Scores (bitmap numbers)
        if (renderer) {
            SDL2BitmapFont::draw_number(
                static_cast<uint8_t>(scene.home_score),
                static_cast<float>(center_x - 80),
                static_cast<float>(score_y),
                3.0f, SDL2Color::HUD_TEXT(), renderer);

            // Dash separators
            draw_rect_filled(center_x - 18, score_y - 2, 6, 4, SDL2Color::HUD_TEXT());
            draw_rect_filled(center_x + 12, score_y - 2, 6, 4, SDL2Color::HUD_TEXT());

            SDL2BitmapFont::draw_number(
                static_cast<uint8_t>(scene.away_score),
                static_cast<float>(center_x + 40),
                static_cast<float>(score_y),
                3.0f, SDL2Color::HUD_TEXT(), renderer);
        }

        // Away team red square
        draw_rect_filled(center_x + 100, score_y - 6, 12, 12, SDL2Color::AWAY_RED());
        // "Away" label
        draw_rect_filled(center_x + 118, score_y - 3, 24, 6, SDL2Color(180, 180, 180, 255));

        // --- Info bar at bottom ---
        int info_h = 24;
        int info_y = window_h - info_h;
        draw_rect_filled(0, info_y, window_w, info_h, SDL2Color::HUD_BG());

        // Sim time MM:SS
        float sim_time = scene.match_time_seconds;
        int minutes = static_cast<int>(sim_time) / 60;
        int seconds = static_cast<int>(sim_time) % 60;

        if (renderer) {
            SDL2BitmapFont::draw_number(
                static_cast<uint8_t>(minutes > 99 ? 99 : minutes),
                15.0f, static_cast<float>(info_y + 4),
                2.0f, SDL2Color::HUD_TEXT(), renderer);
            draw_rect_filled(38, info_y + 7, 2, 10, SDL2Color::HUD_TEXT());
            SDL2BitmapFont::draw_number(
                static_cast<uint8_t>(seconds > 99 ? 99 : seconds),
                44.0f, static_cast<float>(info_y + 4),
                2.0f, SDL2Color::HUD_TEXT(), renderer);

            // FPS (green)
            int fps_x = window_w - 140;
            SDL2BitmapFont::draw_number(
                static_cast<uint8_t>(display_fps > 99 ? 99 : static_cast<int>(display_fps)),
                static_cast<float>(fps_x),
                static_cast<float>(info_y + 4),
                2.0f, SDL2Color(100, 255, 100, 255), renderer);
            draw_rect_filled(fps_x + 20, info_y + 8, 10, 2, SDL2Color(100, 255, 100, 255));
            draw_rect_filled(fps_x + 20, info_y + 14, 10, 2, SDL2Color(100, 255, 100, 255));

            // Ball speed
            const BallEntity* ball = scene.entity_manager.find_ball();
            if (ball) {
                int bs = static_cast<int>(Vec3::length(ball->velocity));
                int bs_x = window_w / 2 - 20;
                SDL2BitmapFont::draw_number(
                    static_cast<uint8_t>(bs > 99 ? 99 : bs),
                    static_cast<float>(bs_x),
                    static_cast<float>(info_y + 4),
                    2.0f, SDL2Color::HUD_TEXT(), renderer);
            }
        }

        // --- Pause overlay ---
        if (app.game_loop.state == GameState::PAUSED) {
            int pw = 100;
            int ph = 30;
            int px = (window_w - pw) / 2;
            int py = (window_h - ph) / 2;
            draw_rect_filled(px, py, pw, ph, SDL2Color(0, 0, 0, 200));
            draw_rect_outline(px, py, pw, ph, SDL2Color(255, 255, 0, 255));
            draw_rect_filled(px + 20, py + 8, 4, 14, SDL2Color(255, 255, 0, 255));
            draw_rect_filled(px + 20, py + 8, 14, 4, SDL2Color(255, 255, 0, 255));
            draw_rect_filled(px + 24, py + 12, 10, 4, SDL2Color(255, 255, 0, 255));
            draw_rect_filled(px + 20, py + 16, 14, 4, SDL2Color(255, 255, 0, 255));
        }
    }

    // =========================================================================
    // Mini-map
    // =========================================================================

    void draw_minimap(const Application& app) {
        int mm_w = 140;
        int mm_h = static_cast<int>(mm_w * 0.647f);
        int mm_x = window_w - mm_w - 8;
        int mm_y = 40;

        draw_rect_filled(mm_x, mm_y, mm_w, mm_h, SDL2Color(20, 80, 20, 200));
        draw_rect_outline(mm_x, mm_y, mm_w, mm_h, SDL2Color(200, 200, 200, 150));

        const MatchConfig& cfg = app.scene.config;
        float half_l = cfg.field_length * 0.5f;
        float half_w = cfg.field_width * 0.5f;
        float mm_sx = static_cast<float>(mm_w) / cfg.field_length;
        float mm_sz = static_cast<float>(mm_h) / cfg.field_width;

        auto to_mm_x = [&](float wx) -> int {
            return static_cast<int>((wx + half_l) * mm_sx) + mm_x;
        };
        auto to_mm_y = [&](float wz) -> int {
            return static_cast<int>((wz + half_w) * mm_sz) + mm_y;
        };

        draw_line_2d(to_mm_x(0), mm_y, to_mm_x(0), mm_y + mm_h, SDL2Color(200, 200, 200, 100));
        int mm_cr = static_cast<int>(9.15f * mm_sx);
        if (mm_cr > 1) {
            draw_circle_outline(to_mm_x(0), to_mm_y(0), mm_cr, SDL2Color(200, 200, 200, 100));
        }

        // Athletes
        const EntityManager& em = app.scene.entity_manager;
        for (uint32_t i = 0u; i < em.athlete_count; ++i) {
            const AthleteEntity& a = em.athletes[i];
            if (!a.id.is_valid() || !a.is_active) continue;
            SDL2Color c = (a.team == TEAM_HOME)
                ? SDL2Color(80, 130, 255, 220) : SDL2Color(255, 80, 80, 220);
            draw_circle_filled(to_mm_x(a.position.x), to_mm_y(a.position.z), 2, c);
        }

        // Ball
        const BallEntity* ball = em.find_ball();
        if (ball && ball->id.is_valid()) {
            draw_circle_filled(to_mm_x(ball->position.x), to_mm_y(ball->position.z), 3,
                              SDL2Color::BALL_WHITE());
        }

        // Camera viewport rect
        float vw = static_cast<float>(window_w) * 0.5f / camera.scale;
        float vh = static_cast<float>(window_h) * 0.5f / camera.scale;
        int vx = to_mm_x(camera.cam_x - vw);
        int vy = to_mm_y(camera.cam_z - vh);
        int vrw = static_cast<int>(vw * 2.0f * mm_sx);
        int vrh = static_cast<int>(vh * 2.0f * mm_sz);
        draw_rect_outline(vx, vy, vrw, vrh, SDL2Color(255, 255, 0, 150));
    }

    // =========================================================================
    // FPS tracking
    // =========================================================================

    void update_fps(float dt) {
        fps_timer += dt;
        ++fps_frame_count;
        if (fps_timer >= 0.5f) {
            display_fps = static_cast<float>(fps_frame_count) / fps_timer;
            fps_frame_count = 0u;
            fps_timer = 0.0f;
        }
    }

    // =========================================================================
    // Full frame render
    // =========================================================================

    void render_frame(Application& app) {
        if (!renderer || !initialized) return;

        SDL_SetRenderDrawColor(renderer, 15, 15, 20, 255);
        SDL_RenderClear(renderer);

        const SceneState& scene = app.scene;
        const BallEntity* ball = scene.entity_manager.find_ball();

        float dt = static_cast<float>(app.game_loop.time.delta_time);
        if (dt < 0.001f) dt = 0.016f;
        camera.update_follow(ball, dt);

        draw_field(scene.config);
        draw_athletes(scene.entity_manager);
        draw_ball(ball);
        draw_debug(app.debug_draw_list);
        draw_hud(app);
        draw_minimap(app);

        SDL_RenderPresent(renderer);
    }
};

} // namespace apc
