// =============================================================================
// sdl2_main.cpp — SDL2 windowed entry point for the APC Physics Engine
// =============================================================================
//
// Creates an SDL2 window and runs the soccer simulation with live rendering:
//   - 1280x720 window (configurable)
//   - Orthographic top-down camera view
//   - Real-time rendering of field, athletes, ball
//   - Debug overlay (AI steering, formation, utility)
//   - HUD: score, sim time, FPS, ball speed
//   - Mini-map in corner
//   - ESC to quit, SPACE to pause, +/- to zoom, arrow keys to pan
//
// Build: cmake --build build --target apc_sdl2_application
//
// =============================================================================

#include "apc_render/apc_sdl2_renderer.h"

#include <cstdio>
#include <chrono>
#include <cmath>

// =============================================================================
// Configuration
// =============================================================================
static const int   WINDOW_WIDTH  = 1280;
static const int   WINDOW_HEIGHT = 720;
static const char* WINDOW_TITLE  = "APC Physics Engine — Soccer [SDL2]";

// =============================================================================
// Main
// =============================================================================
int main()
{
    std::printf("============================================\n");
    std::printf(" APC Physics Engine — SDL2 Renderer\n");
    std::printf(" Windowed Soccer Simulation\n");
    std::printf("============================================\n\n");

    // --- Initialize SDL2 ---
    std::printf("[..] Initializing SDL2...\n");

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        std::printf("[ERROR] SDL_Init failed: %s\n", SDL_GetError());
        std::printf("NOTE: This is expected in a headless/server environment.\n");
        std::printf("      The build compilation is the important verification step.\n");
        return 1;
    }
    std::printf("[OK] SDL2 initialized\n");

    // --- Create window ---
    apc::SDL2Renderer rend;
    rend.window = SDL_CreateWindow(
        WINDOW_TITLE,
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        WINDOW_WIDTH,
        WINDOW_HEIGHT,
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE
    );

    if (!rend.window) {
        std::printf("[ERROR] SDL_CreateWindow failed: %s\n", SDL_GetError());
        std::printf("NOTE: This is expected in a headless/server environment.\n");
        std::printf("      The build compilation is the important verification step.\n");
        SDL_Quit();
        return 1;
    }
    std::printf("[OK] Window created (%dx%d)\n", WINDOW_WIDTH, WINDOW_HEIGHT);

    // --- Create renderer ---
    rend.renderer = SDL_CreateRenderer(
        rend.window,
        -1,
        SDL_RENDERER_SOFTWARE | SDL_RENDERER_PRESENTVSYNC
    );

    if (!rend.renderer) {
        std::printf("[ERROR] SDL_CreateRenderer failed: %s\n", SDL_GetError());
        std::printf("NOTE: This is expected in a headless/server environment.\n");
        SDL_DestroyWindow(rend.window);
        SDL_Quit();
        return 1;
    }
    std::printf("[OK] Renderer created\n");

    rend.initialized = 1;
    rend.window_w = WINDOW_WIDTH;
    rend.window_h = WINDOW_HEIGHT;
    rend.camera.window_width = static_cast<uint32_t>(WINDOW_WIDTH);
    rend.camera.window_height = static_cast<uint32_t>(WINDOW_HEIGHT);

    // --- Initialize Application ---
    apc::Application app;
    apc::ApplicationConfig cfg = apc::Application::soccer_defaults();
    cfg.enable_debug_draw = 1;
    cfg.enable_ai_debug   = 1;

    if (!app.init(cfg)) {
        std::printf("[ERROR] Application initialization failed.\n");
        SDL_DestroyRenderer(rend.renderer);
        SDL_DestroyWindow(rend.window);
        SDL_Quit();
        return 1;
    }
    std::printf("[OK] Application initialized\n");

    // --- Load soccer match ---
    apc::MatchConfig match = apc::SceneState::soccer_match();
    if (!app.load_match(match)) {
        std::printf("[ERROR] Match loading failed.\n");
        SDL_DestroyRenderer(rend.renderer);
        SDL_DestroyWindow(rend.window);
        SDL_Quit();
        return 1;
    }
    std::printf("[OK] Match loaded: %s vs %s\n",
                match.home_team_name, match.away_team_name);
    std::printf("     Field: %.0fm x %.0fm\n",
                match.field_length, match.field_width);

    // --- Enable AI debug layers ---
    app.ai_debug.set_layer(apc::AIDebugLayer::STEERING_FORCES, 1);
    app.ai_debug.set_layer(apc::AIDebugLayer::FORMATION_POSITIONS, 1);
    app.ai_debug.set_layer(apc::AIDebugLayer::UTILITY_SCORES, 0); // Off by default (cluttered)
    app.ai_debug.set_layer(apc::AIDebugLayer::MOTOR_INTENT, 1);

    // --- Set camera to fit the field ---
    rend.camera.auto_fit(match.field_length, match.field_width);

    // --- Transition to PLAYING state ---
    app.game_loop.state = apc::GameState::PLAYING;

    // --- Main loop ---
    std::printf("\n--- Simulation running (ESC to quit) ---\n\n");

    using Clock = std::chrono::high_resolution_clock;
    auto wall_start = Clock::now();
    double wall_time = 0.0;
    uint8_t running = 1;
    uint8_t first_frame = 1;
    double last_wall_time = 0.0;

    while (running) {
        auto frame_start = Clock::now();

        // --- Event polling ---
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            switch (event.type) {
            case SDL_QUIT:
                running = 0;
                break;

            case SDL_KEYDOWN:
                switch (event.key.keysym.sym) {
                case SDLK_ESCAPE:
                    running = 0;
                    break;
                case SDLK_SPACE:
                    app.game_loop.toggle_pause();
                    break;
                case SDLK_EQUALS:
                case SDLK_PLUS:
                case SDLK_KP_PLUS:
                    rend.camera.zoom(1.0f);
                    break;
                case SDLK_MINUS:
                case SDLK_KP_MINUS:
                    rend.camera.zoom(-1.0f);
                    break;
                case SDLK_LEFT:
                    rend.camera.pan(-5.0f, 0.0f);
                    rend.camera.follow_ball = 0;
                    break;
                case SDLK_RIGHT:
                    rend.camera.pan(5.0f, 0.0f);
                    rend.camera.follow_ball = 0;
                    break;
                case SDLK_UP:
                    rend.camera.pan(0.0f, -5.0f);
                    rend.camera.follow_ball = 0;
                    break;
                case SDLK_DOWN:
                    rend.camera.pan(0.0f, 5.0f);
                    rend.camera.follow_ball = 0;
                    break;
                case SDLK_f:
                case SDLK_HOME:
                    // Re-center on field and re-follow ball
                    rend.camera.cam_x = 0.0f;
                    rend.camera.cam_z = 0.0f;
                    rend.camera.follow_ball = 1;
                    rend.camera.auto_fit(match.field_length, match.field_width);
                    break;
                case SDLK_d:
                    // Toggle debug overlay
                    cfg.enable_debug_draw = (cfg.enable_debug_draw) ? 0 : 1;
                    app.config.enable_debug_draw = cfg.enable_debug_draw;
                    std::printf("[INFO] Debug draw %s\n",
                                cfg.enable_debug_draw ? "ON" : "OFF");
                    break;
                default:
                    break;
                }
                break;

            case SDL_WINDOWEVENT:
                if (event.window.event == SDL_WINDOWEVENT_RESIZED) {
                    int new_w = event.window.data1;
                    int new_h = event.window.data2;
                    if (new_w > 0 && new_h > 0) {
                        rend.window_w = new_w;
                        rend.window_h = new_h;
                        rend.camera.window_width = static_cast<uint32_t>(new_w);
                        rend.camera.window_height = static_cast<uint32_t>(new_h);
                    }
                }
                break;

            default:
                break;
            }
        }

        // --- Timing ---
        auto now = Clock::now();
        std::chrono::duration<double> elapsed = now - wall_start;
        wall_time = elapsed.count();

        if (first_frame) {
            // On first frame, set current_time to avoid huge delta
            last_wall_time = wall_time;
            first_frame = 0;
        }

        double frame_dt = wall_time - last_wall_time;
        last_wall_time = wall_time;

        // Clamp frame delta to prevent spiral of death
        if (frame_dt > 0.1) frame_dt = 0.1;
        if (frame_dt < 0.0) frame_dt = 0.0;

        // --- Simulation step ---
        app.begin_frame(wall_time);
        app.tick();
        app.end_frame();

        // --- Render ---
        rend.update_fps(static_cast<float>(frame_dt));
        rend.render_frame(app);

        // --- Frame rate limiter (cap at ~60 FPS) ---
        auto frame_end = Clock::now();
        std::chrono::duration<double> frame_elapsed = frame_end - frame_start;
        double sleep_time = (1.0 / 60.0) - frame_elapsed.count();
        if (sleep_time > 0.001) {
            SDL_Delay(static_cast<uint32_t>(sleep_time * 1000.0));
        }
    }

    // --- Shutdown ---
    auto wall_finish = Clock::now();
    std::chrono::duration<double> total_wall = wall_finish - wall_start;

    std::printf("\n============================================\n");
    std::printf(" SIMULATION ENDED\n");
    std::printf("============================================\n");
    std::printf("  Wall time:      %.2f seconds\n", total_wall.count());
    std::printf("  Physics steps:  %u\n", app.game_loop.time.physics_step_count);
    std::printf("  Render frames:  %u\n", app.game_loop.time.frame_count);
    std::printf("  Average FPS:    %.1f\n",
                static_cast<double>(app.game_loop.time.frame_count) / total_wall.count());
    std::printf("  Final score:    Home %u - %u Away\n",
                app.scene.home_score, app.scene.away_score);
    std::printf("============================================\n");

    // --- Cleanup ---
    app.shutdown();
    SDL_DestroyRenderer(rend.renderer);
    SDL_DestroyWindow(rend.window);
    SDL_Quit();

    std::printf("\n[OK] Clean shutdown.\n");
    return 0;
}
