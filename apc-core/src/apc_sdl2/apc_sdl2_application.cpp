// =============================================================================
// APC SDL2 Visualizer — Implementation
// =============================================================================
// Phase 12 Action 2: Pure rendering/input shell for apc::Application.
// =============================================================================

#include "apc_sdl2_application.h"

#include <SDL2/SDL.h>
#include <cstdio>
#include <cmath>
#include <algorithm>

namespace apc {

// ---------------------------------------------------------------------------
// Construction / Destruction
// ---------------------------------------------------------------------------

SDL2Application::SDL2Application() = default;

SDL2Application::~SDL2Application() {
    shutdown();
}

// ---------------------------------------------------------------------------
// Initialization
// ---------------------------------------------------------------------------

bool SDL2Application::init(int width, int height) {
    width_  = width;
    height_ = height;

    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::fprintf(stderr, "[APC SDL2] SDL_Init failed: %s\n", SDL_GetError());
        return false;
    }

    window_ = SDL_CreateWindow(
        "APC Physics Engine — Vertical Slice",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        width_, height_,
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE
    );
    if (!window_) {
        std::fprintf(stderr, "[APC SDL2] SDL_CreateWindow failed: %s\n", SDL_GetError());
        shutdown();
        return false;
    }

    renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer_) {
        std::fprintf(stderr, "[APC SDL2] Hardware renderer failed, trying software\n");
        renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_SOFTWARE);
        if (!renderer_) {
            std::fprintf(stderr, "[APC SDL2] SDL_CreateRenderer failed: %s\n", SDL_GetError());
            shutdown();
            return false;
        }
    }

    // Bootstrap the core engine with application defaults
    ApplicationConfig cfg = Application::soccer_defaults();
    cfg.window_width = static_cast<uint32_t>(width_);
    cfg.window_height = static_cast<uint32_t>(height_);
    app_.init(cfg);

    // Center camera on midfield
    camera_.center_x = 0.0f;
    camera_.center_y = 0.0f;
    camera_.zoom = 5.5f; // ~5.5 px/m shows most of the 105m field

    std::printf("[APC SDL2] Initialized: %dx%d window, engine ready\n",
                width_, height_);

    return true;
}

// ---------------------------------------------------------------------------
// Vertical Slice Scenario Setup
// ---------------------------------------------------------------------------

void SDL2Application::setup_vertical_slice() {
    MatchConfig match_cfg = MatchConfig::soccer_sandbox();
    app_.load_match(match_cfg);
    // Mark first home athlete as human-controlled
    const EntityManager& em = app_.scene.entity_manager;
    if (em.athlete_count > 0 && em.athletes[0].id.is_valid()) {
        app_.scene.assign_human(em.athletes[0].id);
    }
}

// ---------------------------------------------------------------------------
// Phase 12.5: Hot-Swap between sports at runtime
// ---------------------------------------------------------------------------
// Tears down the current physics world and entities, wipes AI memory,
// and rebuilds the scene with the specified sport configuration.
//
// The underlying load_match() calls unload() which:
//   - Resets the EntityManager (all athletes + ball despawned)
//   - Clears FormationSystem
//   - Clears SportField + semantic zones
//   - Resets scores, match time, possession state
//   - Wipes AI controllers, UtilityAI actions, PerceptionBuffer, InfluenceMap
//
// Then it rebuilds everything: field geometry, semantic zones, teams, ball,
// AI action injection (sport-specific), and considerations.
// ---------------------------------------------------------------------------

void SDL2Application::switch_sport(SportModuleType type) {
    current_sport_ = type;

    // 1. Pause the simulation during rebuild
    bool was_paused = (app_.state == ApplicationState::PAUSED);
    app_.request_pause();

    // 2. Build the appropriate MatchConfig and load it
    //    load_match() internally calls unload() first, guaranteeing clean teardown
    MatchConfig match_cfg;
    switch (type) {
    case SportModuleType::BASKETBALL:
        match_cfg = MatchConfig::basketball_sandbox();
        break;
    case SportModuleType::SOCCER:
    default:
        match_cfg = MatchConfig::soccer_sandbox();
        break;
    }

    app_.load_match(match_cfg);

    // 2b. Phase 15 Action 2: Reset positions to ensure clean formation
    //     alignment (load_match sets home_position during spawn_team).
    app_.scene.reset_match_positions();

    // 3. Mark first home athlete as human-controlled
    const EntityManager& em = app_.scene.entity_manager;
    if (em.athlete_count > 0 && em.athletes[0].id.is_valid()) {
        app_.scene.assign_human(em.athletes[0].id);
    }

    // 4. Adjust camera zoom for the new field size
    if (type == SportModuleType::BASKETBALL) {
        camera_.zoom = 20.0f;  // Zoom in for 15x28m court
    } else {
        camera_.zoom = 5.5f;   // Zoom out for 68x105m pitch
    }
    camera_.center_x = 0.0f;
    camera_.center_y = 0.0f;

    // 5. Resume if was running
    if (!was_paused) app_.request_resume();

    std::printf("[APC SDL2] Switched to: %s (field %.0fx%.0fm, %u athletes)\n",
                (type == SportModuleType::BASKETBALL) ? "Basketball" : "Soccer",
                match_cfg.field_width, match_cfg.field_length,
                app_.scene.entity_manager.athlete_count);
}

// ---------------------------------------------------------------------------
// Main Loop
// ---------------------------------------------------------------------------

void SDL2Application::run() {
    if (!window_ || !renderer_) return;

    // Phase 12.5: Start with soccer by default (via switch_sport for consistency)
    switch_sport(SportModuleType::SOCCER);

    Uint64 prev_ticks = SDL_GetPerformanceCounter();
    bool running = true;

    while (running) {
        Uint64 now_ticks = SDL_GetPerformanceCounter();
        float frame_dt = (float)(now_ticks - prev_ticks) / (float)SDL_GetPerformanceFrequency();
        prev_ticks = now_ticks;

        // Clamp to prevent spiral of death
        if (frame_dt > 0.1f) frame_dt = 0.1f;

        // Wall clock for engine timing
        double wall_time = (double)SDL_GetTicks() / 1000.0;

        process_events();
        if (!window_) break;

        // Bridge human input into the engine
        handle_input(frame_dt);

        // Core Engine Loop: begin_frame -> tick -> end_frame
        app_.begin_frame(wall_time);
        app_.tick();
        app_.end_frame();

        // Render the engine state
        render();

        // Cap to ~60 FPS display refresh
        SDL_Delay(1);
    }
}

// ---------------------------------------------------------------------------
// Event Processing
// ---------------------------------------------------------------------------

void SDL2Application::process_events() {
    keys_ = SDL_GetKeyboardState(nullptr);

    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        switch (event.type) {
        case SDL_QUIT:
            shutdown();
            return;

        case SDL_WINDOWEVENT:
            if (event.window.event == SDL_WINDOWEVENT_RESIZED) {
                width_  = event.window.data1;
                height_ = event.window.data2;
            }
            break;

        case SDL_KEYDOWN:
            if (event.key.keysym.sym == SDLK_ESCAPE) {
                shutdown();
                return;
            }
            if (event.key.keysym.sym == SDLK_SPACE) {
                if (app_.state == ApplicationState::PAUSED) {
                    app_.request_resume();
                } else {
                    app_.request_pause();
                }
            }
            // Phase 12.5: Hot-swap sport controls
            if (event.key.keysym.sym == SDLK_1) switch_sport(SportModuleType::SOCCER);
            if (event.key.keysym.sym == SDLK_2) switch_sport(SportModuleType::BASKETBALL);
            // Phase 15 Action 1: Match flow controls
            if (event.key.keysym.sym == SDLK_k) {
                app_.scene.rules.current_state = SemanticPlayState::LIVE_PLAY;
            }
            if (event.key.keysym.sym == SDLK_l) {
                app_.scene.rules.current_state = SemanticPlayState::DEAD_BALL;
            }
            // Phase 15 Action 2: Reset formations (teleport + cognitive wipe)
            if (event.key.keysym.sym == SDLK_r) {
                app_.scene.reset_match_positions();
                app_.scene.rules.current_state = SemanticPlayState::PRE_GAME;
                std::printf("[APC SDL2] Positions reset — press K to start play\n");
            }
            break;

        case SDL_MOUSEWHEEL:
            camera_.zoom *= (event.wheel.y > 0) ? 1.1f : 0.9f;
            // Clamp zoom to useful range
            if (camera_.zoom < 1.0f)  camera_.zoom = 1.0f;
            if (camera_.zoom > 80.0f) camera_.zoom = 80.0f;
            break;
        }
    }
}

// ---------------------------------------------------------------------------
// Input Handling — Camera Pan + Player Input
// ---------------------------------------------------------------------------

void SDL2Application::handle_input(float dt) {
    if (!keys_) return;

    // 1. Camera Pan (Arrow keys only — WASD reserved for player control)
    float pan_speed = 5.0f / camera_.zoom * 100.0f;
    if (keys_[SDL_SCANCODE_LEFT])  camera_.center_x -= pan_speed * dt;
    if (keys_[SDL_SCANCODE_RIGHT]) camera_.center_x += pan_speed * dt;
    if (keys_[SDL_SCANCODE_UP])    camera_.center_y += pan_speed * dt;
    if (keys_[SDL_SCANCODE_DOWN])  camera_.center_y -= pan_speed * dt;

    // 2. Map Player 0 Input (WASD) to engine InputState
    InputState p0_input;
    if (keys_[SDL_SCANCODE_W]) p0_input.left_stick.y =  1.0f;
    if (keys_[SDL_SCANCODE_S]) p0_input.left_stick.y = -1.0f;
    if (keys_[SDL_SCANCODE_A]) p0_input.left_stick.x = -1.0f;
    if (keys_[SDL_SCANCODE_D]) p0_input.left_stick.x = 1.0f;

    // Apply dead zone and frame delta for the engine
    p0_input.frame_delta();

    app_.update_input(0, p0_input);
}

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------

void SDL2Application::render() {
    // Clear — dark background
    SDL_SetRenderDrawColor(renderer_, 15, 15, 25, 255);
    SDL_RenderClear(renderer_);

    // Field ground grid (soccer pitch lines)
    draw_ground_grid();

    // --- Semantic Zone Overlay (Phase 14 Action 1) ---
    // Render SportField semantic zones as colored wireframes + faint fills.
    // This allows visual verification that AI spatial queries align with
    // the actual field geometry (critical for RESTRICTED_DEFENSE, SCORING_TARGET).
    {
        const SportField& sf = app_.scene.field;
        for (uint32_t i = 0u; i < sf.semantic_zone_count; ++i) {
            const SemanticFieldZone& zone = sf.semantic_zones[i];

            // Skip OUT_OF_BOUNDS zones — they clutter the view beyond the pitch
            if (zone.semantic == ZoneSemantic::OUT_OF_BOUNDS) continue;

            uint8_t r = 200, g = 200, b = 200;
            switch (zone.semantic) {
            case ZoneSemantic::RESTRICTED_DEFENSE:
                r = 255; g = 140; b = 0;   // Orange — Penalty Box / Paint
                break;
            case ZoneSemantic::RESTRICTED_OFFENSE:
                r = 255; g = 50;  b = 50;  // Red — Offensive restricted
                break;
            case ZoneSemantic::SCORING_TARGET:
                r = 255; g = 255; b = 0;   // Yellow — Goal / Hoop / Endzone
                break;
            case ZoneSemantic::NEUTRAL_PLAYING_FIELD:
                r = 50;  g = 200; b = 50;  // Green — General playing area
                break;
            default: break;
            }

            // Wireframe boundary
            draw_rect_wireframe(zone.min_bounds.x, zone.min_bounds.z,
                                zone.max_bounds.x, zone.max_bounds.z,
                                r, g, b, 100);
            // Faint interior fill for volume indication
            draw_rect_filled(zone.min_bounds.x, zone.min_bounds.z,
                             zone.max_bounds.x, zone.max_bounds.z,
                             r, g, b, 20);
        }
    }

    // --- Influence Map Overlay (Spatial Awareness) ---
    // Red = threat (opponent presence), Blue = control (friendly presence)
    draw_influence_map();

    // --- Draw Athletes from the real engine ---
    const EntityManager& em = app_.scene.entity_manager;

    for (uint32_t i = 0u; i < em.athlete_count; ++i) {
        const AthleteEntity& a = em.athletes[i];
        if (!a.id.is_valid() || !a.is_active) continue;

        // Team coloring: Human = green, Home = red, Away = blue
        uint8_t r, g, b;
        if (a.is_human_controlled) {
            r = 50;  g = 255; b = 100; // Bright green for human
        } else if (a.team == TEAM_HOME) {
            r = 230; g = 60;  b = 60;  // Home: red
        } else {
            r = 60;  g = 80;  b = 230;  // Away: blue
        }

        // Draw athlete circle at world XZ position
        draw_circle(a.position.x, a.position.z, 0.45f, r, g, b, 255);

        // Draw Motor Intent vector (where AI athletes want to move)
        if (!a.is_human_controlled && a.current_intent.has_locomotion()) {
            // Scale intent vector for visibility (3m line at full speed)
            float intent_scale = 3.0f;
            float end_x = a.position.x + a.current_intent.move_direction.x * intent_scale;
            float end_z = a.position.z + a.current_intent.move_direction.z * intent_scale;
            draw_line(a.position.x, a.position.z, end_x, end_z,
                      255, 255, 0, 200); // Yellow intent vector
        }
    }

    // --- Draw Ball from the real engine ---
    const BallEntity* ball = em.find_ball();
    if (ball && ball->id.is_valid()) {
        draw_circle(ball->position.x, ball->position.z, 0.22f,
                     255, 255, 255, 255); // White
    }

    // HUD overlay
    draw_hud();

    SDL_RenderPresent(renderer_);
}

// ---------------------------------------------------------------------------
// Drawing: Circle (scanline fill + outline)
// ---------------------------------------------------------------------------

void SDL2Application::draw_circle(float wx, float wy, float wr,
                                   uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
    float sx = camera_.world_to_screen_x(wx, width_);
    float sy = camera_.world_to_screen_y(wy, height_);
    float sr = camera_.world_to_screen_scale(wr);

    // Cull off-screen
    if (sx + sr < 0 || sx - sr > width_ || sy + sr < 0 || sy - sr > height_) return;
    if (sr < 0.5f) return;

    // Filled circle using scanlines
    SDL_SetRenderDrawColor(renderer_, r, g, b, a);
    int r_int = (int)(sr + 0.5f);
    int cx = (int)sx;
    int cy = (int)sy;

    for (int dy = -r_int; dy <= r_int; ++dy) {
        float dx_sq = (float)(sr * sr - dy * dy);
        if (dx_sq < 0.0f) continue;
        int dx = (int)(std::sqrt(dx_sq) + 0.5f);
        SDL_RenderDrawLine(renderer_, cx - dx, cy + dy, cx + dx, cy + dy);
    }

    // Outline — slightly brighter
    SDL_SetRenderDrawColor(renderer_,
        (uint8_t)std::min(255, r + 60),
        (uint8_t)std::min(255, g + 60),
        (uint8_t)std::min(255, b + 60), 255);

    static constexpr int OUTLINE_SEGMENTS = 24;
    for (int i = 0; i < OUTLINE_SEGMENTS; ++i) {
        float a1 = 2.0f * (float)APC_PI * i / OUTLINE_SEGMENTS;
        float a2 = 2.0f * (float)APC_PI * (i + 1) / OUTLINE_SEGMENTS;
        SDL_RenderDrawLine(renderer_,
            (int)(sx + std::cos(a1) * sr),
            (int)(sy - std::sin(a1) * sr),
            (int)(sx + std::cos(a2) * sr),
            (int)(sy - std::sin(a2) * sr)
        );
    }
}

// ---------------------------------------------------------------------------
// Drawing: Line (world-space, blended)
// ---------------------------------------------------------------------------

void SDL2Application::draw_line(float wx1, float wy1, float wx2, float wy2,
                                 uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
    int sx1 = (int)camera_.world_to_screen_x(wx1, width_);
    int sy1 = (int)camera_.world_to_screen_y(wy1, height_);
    int sx2 = (int)camera_.world_to_screen_x(wx2, width_);
    int sy2 = (int)camera_.world_to_screen_y(wy2, height_);

    SDL_SetRenderDrawBlendMode(renderer_, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(renderer_, r, g, b, a);
    SDL_RenderDrawLine(renderer_, sx1, sy1, sx2, sy2);
    SDL_SetRenderDrawBlendMode(renderer_, SDL_BLENDMODE_NONE);
}

// ---------------------------------------------------------------------------
// Drawing: Filled Rectangle (world-space, blended)
// ---------------------------------------------------------------------------

void SDL2Application::draw_rect_filled(float w_min_x, float w_min_y,
                                        float w_max_x, float w_max_y,
                                        uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
    int sx1 = (int)camera_.world_to_screen_x(w_min_x, width_);
    int sy1 = (int)camera_.world_to_screen_y(w_max_y, height_); // Y-flip: max → top
    int sx2 = (int)camera_.world_to_screen_x(w_max_x, width_);
    int sy2 = (int)camera_.world_to_screen_y(w_min_y, height_); // Y-flip: min → bottom

    SDL_Rect rect = { sx1, sy1, sx2 - sx1, sy2 - sy1 };
    SDL_SetRenderDrawBlendMode(renderer_, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(renderer_, r, g, b, a);
    SDL_RenderFillRect(renderer_, &rect);
    SDL_SetRenderDrawBlendMode(renderer_, SDL_BLENDMODE_NONE);
}

// ---------------------------------------------------------------------------
// Drawing: Wireframe Rectangle (world-space, blended)
// ---------------------------------------------------------------------------

void SDL2Application::draw_rect_wireframe(float w_min_x, float w_min_y,
                                           float w_max_x, float w_max_y,
                                           uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
    int sx1 = (int)camera_.world_to_screen_x(w_min_x, width_);
    int sy1 = (int)camera_.world_to_screen_y(w_max_y, height_);
    int sx2 = (int)camera_.world_to_screen_x(w_max_x, width_);
    int sy2 = (int)camera_.world_to_screen_y(w_min_y, height_);

    SDL_Rect rect = { sx1, sy1, sx2 - sx1, sy2 - sy1 };
    SDL_SetRenderDrawBlendMode(renderer_, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(renderer_, r, g, b, a);
    SDL_RenderDrawRect(renderer_, &rect);
    SDL_SetRenderDrawBlendMode(renderer_, SDL_BLENDMODE_NONE);
}

// ---------------------------------------------------------------------------
// Drawing: Influence Map (Spatial Awareness Heatmap Overlay)
// ---------------------------------------------------------------------------
// Renders the AI's 32x16 dual-grid as a transparent overlay on the field.
//   Red cells   = threat (opponent spatial influence)
//   Blue cells  = control (friendly spatial influence)
//
// Cell intensity maps to alpha opacity (0.1 threshold to suppress noise).
// Grid coordinates are converted back to world space using cached field extents.
// ---------------------------------------------------------------------------

void SDL2Application::draw_influence_map() {
    const InfluenceMap& imap = app_.scene.influence_map;

    float field_ext_x = imap.get_field_ext_x();
    float field_ext_z = imap.get_field_ext_z();

    // Skip if influence map hasn't been initialized (field_ext_x/z == 0)
    if (field_ext_x <= 0.0f || field_ext_z <= 0.0f) return;

    float half_ext_x = field_ext_x * 0.5f;
    float half_ext_z = field_ext_z * 0.5f;

    float cell_w = field_ext_x / (float)InfluenceMap::GRID_WIDTH;
    float cell_h = field_ext_z / (float)InfluenceMap::GRID_HEIGHT;

    for (int x = 0; x < InfluenceMap::GRID_WIDTH; ++x) {
        for (int z = 0; z < InfluenceMap::GRID_HEIGHT; ++z) {
            float threat  = imap.get_threat(x, z);
            float control = imap.get_control(x, z);

            // Skip near-zero cells to avoid filling the entire field with noise
            if (threat < 0.1f && control < 0.1f) continue;

            // Convert grid cell to world-space bounding box
            float world_min_x = (float)x * cell_w - half_ext_x;
            float world_min_z = (float)z * cell_h - half_ext_z;
            float world_max_x = world_min_x + cell_w;
            float world_max_z = world_min_z + cell_h;

            // Render dominant layer: threat (red) takes priority over control (blue)
            if (threat > control) {
                uint8_t alpha = (uint8_t)std::min(threat * 50.0f, 150.0f);
                draw_rect_filled(world_min_x, world_min_z,
                                 world_max_x, world_max_z,
                                 255, 50, 50, alpha);
            } else {
                uint8_t alpha = (uint8_t)std::min(control * 50.0f, 150.0f);
                draw_rect_filled(world_min_x, world_min_z,
                                 world_max_x, world_max_z,
                                 50, 100, 255, alpha);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Drawing: Ground Grid (soccer field lines)
// ---------------------------------------------------------------------------

void SDL2Application::draw_ground_grid() {
    // Phase 12.5: Dispatch to sport-specific grid renderer
    if (current_sport_ == SportModuleType::BASKETBALL) {
        draw_ground_grid_basketball();
    } else {
        draw_ground_grid_soccer();
    }
}

// ---------------------------------------------------------------------------
// Drawing: Soccer field lines
// ---------------------------------------------------------------------------

void SDL2Application::draw_ground_grid_soccer() {
    float half_x = app_.scene.config.field_length * 0.5f;
    float half_z = app_.scene.config.field_width * 0.5f;

    int fx1 = (int)camera_.world_to_screen_x(-half_x, width_);
    int fx2 = (int)camera_.world_to_screen_x( half_x, width_);
    int fz1 = (int)camera_.world_to_screen_y(-half_z, height_);
    int fz2 = (int)camera_.world_to_screen_y( half_z, height_);

    // Field background (dark green)
    SDL_SetRenderDrawColor(renderer_, 20, 40, 18, 255);
    SDL_Rect field_rect;
    field_rect.x = (fx1 < fx2) ? fx1 : fx2;
    field_rect.y = (fz1 < fz2) ? fz1 : fz2;
    field_rect.w = (fx2 > fx1) ? (fx2 - fx1) : (fx1 - fx2);
    field_rect.h = (fz2 > fz1) ? (fz2 - fz1) : (fz1 - fz2);
    SDL_RenderFillRect(renderer_, &field_rect);

    // Field outline (bright green)
    SDL_SetRenderDrawColor(renderer_, 60, 130, 50, 255);
    SDL_RenderDrawRect(renderer_, &field_rect);

    // Center line
    int cx = (int)camera_.world_to_screen_x(0.0f, width_);
    SDL_RenderDrawLine(renderer_, cx, fz1, cx, fz2);

    // Center circle (~9.15m radius)
    float center_r = 9.15f;
    int cr_px = (int)camera_.world_to_screen_scale(center_r);
    int cy_px = (int)((fz1 + fz2) * 0.5f);
    if (cr_px > 2) {
        static constexpr int CIRCLE_SEGMENTS = 32;
        for (int i = 0; i < CIRCLE_SEGMENTS; ++i) {
            float a1 = 2.0f * (float)APC_PI * i / CIRCLE_SEGMENTS;
            float a2 = 2.0f * (float)APC_PI * (i + 1) / CIRCLE_SEGMENTS;
            SDL_RenderDrawLine(renderer_,
                cx + (int)(std::cos(a1) * cr_px), cy_px - (int)(std::sin(a1) * cr_px),
                cx + (int)(std::cos(a2) * cr_px), cy_px - (int)(std::sin(a2) * cr_px)
            );
        }
    }

    // Penalty area rectangles (16.5m deep, 40.32m wide)
    float pa_depth = 16.5f;
    float pa_hw    = 20.16f;

    SDL_SetRenderDrawColor(renderer_, 55, 100, 45, 255);
    // Home penalty area
    SDL_Rect pa_home;
    pa_home.x = (int)camera_.world_to_screen_x(-half_x, width_);
    pa_home.y = (int)camera_.world_to_screen_y(-pa_hw, height_);
    pa_home.w = (int)camera_.world_to_screen_x(-half_x + pa_depth, width_) - pa_home.x;
    pa_home.h = (int)camera_.world_to_screen_y(pa_hw, height_) - pa_home.y;
    SDL_RenderDrawRect(renderer_, &pa_home);

    // Away penalty area
    SDL_Rect pa_away;
    pa_away.x = (int)camera_.world_to_screen_x(half_x - pa_depth, width_);
    pa_away.y = (int)camera_.world_to_screen_y(-pa_hw, height_);
    pa_away.w = (int)camera_.world_to_screen_x(half_x, width_) - pa_away.x;
    pa_away.h = (int)camera_.world_to_screen_y(pa_hw, height_) - pa_away.y;
    SDL_RenderDrawRect(renderer_, &pa_away);
}

// ---------------------------------------------------------------------------
// Drawing: Basketball court lines (Phase 12.5)
// ---------------------------------------------------------------------------

void SDL2Application::draw_ground_grid_basketball() {
    float half_x = app_.scene.config.field_length * 0.5f;  // 14m
    float half_z = app_.scene.config.field_width  * 0.5f;  // 7.5m

    int fx1 = (int)camera_.world_to_screen_x(-half_x, width_);
    int fx2 = (int)camera_.world_to_screen_x( half_x, width_);
    int fz1 = (int)camera_.world_to_screen_y(-half_z, height_);
    int fz2 = (int)camera_.world_to_screen_y( half_z, height_);

    // Court background (warm wood tone — darker)
    SDL_SetRenderDrawColor(renderer_, 40, 28, 18, 255);
    SDL_Rect court_rect;
    court_rect.x = (fx1 < fx2) ? fx1 : fx2;
    court_rect.y = (fz1 < fz2) ? fz1 : fz2;
    court_rect.w = (fx2 > fx1) ? (fx2 - fx1) : (fx1 - fx2);
    court_rect.h = (fz2 > fz1) ? (fz2 - fz1) : (fz1 - fz2);
    SDL_RenderFillRect(renderer_, &court_rect);

    // Court outline (warm wood lines)
    SDL_SetRenderDrawColor(renderer_, 180, 140, 80, 255);
    SDL_RenderDrawRect(renderer_, &court_rect);

    // Center line
    int cx = (int)camera_.world_to_screen_x(0.0f, width_);
    SDL_RenderDrawLine(renderer_, cx, fz1, cx, fz2);

    // Center circle (1.83m radius — FIBA)
    float center_r = 1.83f;
    int cr_px = (int)camera_.world_to_screen_scale(center_r);
    int cy_px = (int)((fz1 + fz2) * 0.5f);
    if (cr_px > 1) {
        static constexpr int CIRCLE_SEGMENTS = 24;
        for (int i = 0; i < CIRCLE_SEGMENTS; ++i) {
            float a1 = 2.0f * (float)APC_PI * i / CIRCLE_SEGMENTS;
            float a2 = 2.0f * (float)APC_PI * (i + 1) / CIRCLE_SEGMENTS;
            SDL_RenderDrawLine(renderer_,
                cx + (int)(std::cos(a1) * cr_px), cy_px - (int)(std::sin(a1) * cr_px),
                cx + (int)(std::cos(a2) * cr_px), cy_px - (int)(std::sin(a2) * cr_px)
            );
        }
    }

    // Paint/Lane rectangles (4.88m wide × 5.79m deep)
    float lane_hw = 2.44f;  // Half of 4.88m
    float lane_d  = 5.79f;   // Free-throw line distance

    SDL_SetRenderDrawColor(renderer_, 160, 120, 60, 255);
    // Home paint
    SDL_Rect paint_home;
    paint_home.x = (int)camera_.world_to_screen_x(-half_x, width_);
    paint_home.y = (int)camera_.world_to_screen_y(-lane_hw, height_);
    paint_home.w = (int)camera_.world_to_screen_x(-half_x + lane_d, width_) - paint_home.x;
    paint_home.h = (int)camera_.world_to_screen_y(lane_hw, height_) - paint_home.y;
    SDL_RenderDrawRect(renderer_, &paint_home);

    // Away paint
    SDL_Rect paint_away;
    paint_away.x = (int)camera_.world_to_screen_x(half_x - lane_d, width_);
    paint_away.y = (int)camera_.world_to_screen_y(-lane_hw, height_);
    paint_away.w = (int)camera_.world_to_screen_x(half_x, width_) - paint_away.x;
    paint_away.h = (int)camera_.world_to_screen_y(lane_hw, height_) - paint_away.y;
    SDL_RenderDrawRect(renderer_, &paint_away);

    // Three-point arcs (6.75m radius from basket center)
    float arc_r = 6.75f;
    int arc_px = (int)camera_.world_to_screen_scale(arc_r);
    SDL_SetRenderDrawColor(renderer_, 140, 100, 50, 255);
    if (arc_px > 2) {
        static constexpr int ARC_SEGMENTS = 32;
        // Home three-point arc (semicircle facing +X)
        int arc_cx = (int)camera_.world_to_screen_x(-half_x, width_);
        int arc_cy = cy_px;
        for (int i = 0; i < ARC_SEGMENTS; ++i) {
            // Arc from -90 to +90 degrees (right-facing semicircle)
            float a1 = (float)APC_PI * (-0.5f + (float)i / ARC_SEGMENTS);
            float a2 = (float)APC_PI * (-0.5f + (float)(i + 1) / ARC_SEGMENTS);
            SDL_RenderDrawLine(renderer_,
                arc_cx + (int)(std::cos(a1) * arc_px), arc_cy - (int)(std::sin(a1) * arc_px),
                arc_cx + (int)(std::cos(a2) * arc_px), arc_cy - (int)(std::sin(a2) * arc_px)
            );
        }
        // Away three-point arc (semicircle facing -X)
        arc_cx = (int)camera_.world_to_screen_x(half_x, width_);
        for (int i = 0; i < ARC_SEGMENTS; ++i) {
            float a1 = (float)APC_PI * (0.5f + (float)i / ARC_SEGMENTS);
            float a2 = (float)APC_PI * (0.5f + (float)(i + 1) / ARC_SEGMENTS);
            SDL_RenderDrawLine(renderer_,
                arc_cx + (int)(std::cos(a1) * arc_px), arc_cy - (int)(std::sin(a1) * arc_px),
                arc_cx + (int)(std::cos(a2) * arc_px), arc_cy - (int)(std::sin(a2) * arc_px)
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Drawing: HUD (Heads-Up Display)
// ---------------------------------------------------------------------------

void SDL2Application::draw_hud() {
    // Semi-transparent background bar
    SDL_SetRenderDrawBlendMode(renderer_, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 160);
    SDL_Rect hud_rect;
    hud_rect.x = 0;
    hud_rect.y = 0;
    hud_rect.w = width_;
    hud_rect.h = 40;
    SDL_RenderFillRect(renderer_, &hud_rect);
    SDL_SetRenderDrawBlendMode(renderer_, SDL_BLENDMODE_NONE);

    // Score display — Home (red) vs Away (blue)
    uint32_t home = app_.scene.home_score;
    uint32_t away = app_.scene.away_score;

    // Home score indicator (red dot + number)
    SDL_SetRenderDrawColor(renderer_, 230, 60, 60, 255);
    SDL_Rect home_bg;
    home_bg.x = 10;
    home_bg.y = 8;
    home_bg.w = 80;
    home_bg.h = 24;
    SDL_RenderFillRect(renderer_, &home_bg);

    // Score bar (visual representation of score)
    for (uint32_t i = 0u; i < home && i < 10u; ++i) {
        SDL_SetRenderDrawColor(renderer_, 255, 255, 255, 255);
        SDL_Rect dot;
        dot.x = 15 + (int)i * 7;
        dot.y = 12;
        dot.w = 5;
        dot.h = 16;
        SDL_RenderFillRect(renderer_, &dot);
    }

    // Away score indicator (blue dot + number)
    SDL_SetRenderDrawColor(renderer_, 60, 80, 230, 255);
    SDL_Rect away_bg;
    away_bg.x = width_ - 90;
    away_bg.y = 8;
    away_bg.w = 80;
    away_bg.h = 24;
    SDL_RenderFillRect(renderer_, &away_bg);

    for (uint32_t i = 0u; i < away && i < 10u; ++i) {
        SDL_SetRenderDrawColor(renderer_, 255, 255, 255, 255);
        SDL_Rect dot;
        dot.x = width_ - 85 + (int)i * 7;
        dot.y = 12;
        dot.w = 5;
        dot.h = 16;
        SDL_RenderFillRect(renderer_, &dot);
    }

    // Match time (center)
    float sim_t = app_.scene.match_time_seconds;
    uint32_t minutes = static_cast<uint32_t>(sim_t / 60.0f);
    uint32_t seconds = static_cast<uint32_t>(sim_t) % 60u;

    // Time bar
    float time_frac = std::fmod(sim_t, 60.0f) / 60.0f;
    SDL_SetRenderDrawColor(renderer_, 150, 200, 150, 255);
    SDL_Rect time_bar;
    time_bar.x = width_ / 2 - 50;
    time_bar.y = 30;
    time_bar.w = (int)(time_frac * 100.0f);
    time_bar.h = 4;
    SDL_RenderFillRect(renderer_, &time_bar);

    SDL_SetRenderDrawColor(renderer_, 100, 200, 100, 255);
    SDL_Rect time_bg;
    time_bg.x = width_ / 2 - 50;
    time_bg.y = 14;
    time_bg.w = 100;
    time_bg.h = 20;
    SDL_RenderDrawRect(renderer_, &time_bg);

    // Phase 12.5: Sport indicator (top-right, colored badge)
    SDL_Rect sport_badge;
    sport_badge.w = 14;
    sport_badge.h = 14;
    sport_badge.y = 13;
    if (current_sport_ == SportModuleType::SOCCER) {
        sport_badge.x = width_ - 110;
        SDL_SetRenderDrawColor(renderer_, 50, 200, 50, 255);  // Green = Soccer
    } else {
        sport_badge.x = width_ - 110;
        SDL_SetRenderDrawColor(renderer_, 255, 140, 0, 255);  // Orange = Basketball
    }
    SDL_RenderFillRect(renderer_, &sport_badge);

    // Phase 15 Action 1: Play state indicator (color bar next to clock)
    {
        SDL_Rect state_bar;
        state_bar.x = width_ / 2 - 55;
        state_bar.y = 8;
        state_bar.w = 8;
        state_bar.h = 20;
        switch (app_.scene.rules.current_state) {
        case SemanticPlayState::LIVE_PLAY:
            SDL_SetRenderDrawColor(renderer_, 50, 220, 50, 255);  // Green = live
            break;
        case SemanticPlayState::DEAD_BALL:
            SDL_SetRenderDrawColor(renderer_, 220, 180, 40, 255);  // Yellow = dead ball
            break;
        case SemanticPlayState::INTERMISSION:
            SDL_SetRenderDrawColor(renderer_, 220, 220, 220, 255); // White = intermission
            break;
        default:
            SDL_SetRenderDrawColor(renderer_, 120, 120, 120, 255); // Grey = pre-game
            break;
        }
        SDL_RenderFillRect(renderer_, &state_bar);
    }

    // Pause indicator
    if (app_.state == ApplicationState::PAUSED) {
        SDL_SetRenderDrawColor(renderer_, 255, 60, 60, 255);
        SDL_Rect pause1;
        pause1.x = width_ / 2 - 15;
        pause1.y = 8;
        pause1.w = 10;
        pause1.h = 20;
        SDL_RenderFillRect(renderer_, &pause1);

        SDL_Rect pause2;
        pause2.x = width_ / 2 + 5;
        pause2.y = 8;
        pause2.w = 10;
        pause2.h = 20;
        SDL_RenderFillRect(renderer_, &pause2);
    }

    // Zoom indicator (bottom-right)
    SDL_SetRenderDrawBlendMode(renderer_, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 120);
    SDL_Rect zoom_bg;
    zoom_bg.x = width_ - 140;
    zoom_bg.y = height_ - 30;
    zoom_bg.w = 130;
    zoom_bg.h = 24;
    SDL_RenderFillRect(renderer_, &zoom_bg);
    SDL_SetRenderDrawBlendMode(renderer_, SDL_BLENDMODE_NONE);

    float zoom_pct = (camera_.zoom - 1.0f) / (80.0f - 1.0f);
    SDL_SetRenderDrawColor(renderer_, 150, 150, 200, 255);
    SDL_Rect zoom_bar;
    zoom_bar.x = width_ - 130;
    zoom_bar.y = height_ - 22;
    zoom_bar.w = (int)(zoom_pct * 100.0f);
    zoom_bar.h = 8;
    SDL_RenderFillRect(renderer_, &zoom_bar);

    // Phase 15 Action 1: Console clock readout (throttled to ~1 Hz)
    {
        static int frame_throttle = 0;
        if (++frame_throttle >= 60) {
            frame_throttle = 0;
            int m = static_cast<int>(app_.scene.period_time_seconds) / 60;
            int s = static_cast<int>(app_.scene.period_time_seconds) % 60;
            const char* state_str = "PRE_GAME";
            switch (app_.scene.rules.current_state) {
            case SemanticPlayState::LIVE_PLAY:      state_str = "LIVE"; break;
            case SemanticPlayState::DEAD_BALL:      state_str = "DEAD_BALL"; break;
            case SemanticPlayState::INTERMISSION:   state_str = "INTERMISSION"; break;
            default: break;
            }
            std::printf("\r[HUD] Period %u | %02d:%02d | %s          ",
                        app_.scene.current_period, m, s, state_str);
            std::fflush(stdout);
        }
    }
}

// ---------------------------------------------------------------------------
// Shutdown
// ---------------------------------------------------------------------------

void SDL2Application::shutdown() {
    if (renderer_) {
        SDL_DestroyRenderer(renderer_);
        renderer_ = nullptr;
    }
    if (window_) {
        SDL_DestroyWindow(window_);
        window_ = nullptr;
    }
    SDL_Quit();
}

} // namespace apc

// =============================================================================
// Entry Point
// =============================================================================

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    std::printf("=== APC Physics Engine — Vertical Slice ===\n");
    std::printf("Controls: Arrows=Pan, Scroll=Zoom, WASD=Player, Space=Pause, 1/2=Sport, K=Play, L=DeadBall, R=Reset, Esc=Quit\n\n");

    apc::SDL2Application app;

    if (!app.init()) {
        std::fprintf(stderr, "Failed to initialize SDL2 visualizer.\n");
        return 1;
    }

    app.run();

    std::printf("\n[APC SDL2] Shut down cleanly.\n");
    return 0;
}
