#pragma once
// =============================================================================
// APC SDL2 Visualizer — Rendering & Input Shell for apc::Application
// =============================================================================
//
// Phase 12 Action 2: SDL2 Shell Refactor & Scenario Bridge.
// This module is now a pure rendering and input shell. It owns an
// apc::Application instance and bridges SDL2 input/events into the engine.
//
// All physics, AI, and game logic lives inside apc::Application.
// This file only handles:
//   - SDL2 window creation and event loop
//   - Camera pan/zoom (arrow keys + mouse wheel)
//   - WASD input routing into the engine's InputState buffer
//   - 2D top-down rendering of athletes, ball, and field
//
// Controls:
//   Arrow keys  — pan camera
//   Mouse wheel  — zoom in/out
//   WASD        — control human player (Player 0)
//   SPACE       — pause/resume
//   ESC         — quit
//
// Cross-platform: Windows (MSVC/vcpkg), Linux (apt), macOS (brew)
//
// =============================================================================

#include "../apc_app/apc_application.h"

#include <cstdint>

struct SDL_Window;
struct SDL_Renderer;

namespace apc {

// ---------------------------------------------------------------------------
// Camera — 2D orthographic projection with pan/zoom
// ---------------------------------------------------------------------------
struct Camera {
    float center_x = 0.0f;
    float center_y = 0.0f;
    float zoom     = 5.0f;  // pixels per meter (tuned for 105m field)

    float world_to_screen_x(float wx, int screen_w) const {
        return screen_w * 0.5f + (wx - center_x) * zoom;
    }
    float world_to_screen_y(float wy, int screen_h) const {
        return screen_h * 0.5f - (wy - center_y) * zoom;
    }
    float world_to_screen_scale(float size) const {
        return size * zoom;
    }
};

// ---------------------------------------------------------------------------
// SDL2Application — manages window, input, rendering; delegates to apc::Application
// ---------------------------------------------------------------------------
class SDL2Application {
public:
    static constexpr int DEFAULT_WIDTH  = 1280;
    static constexpr int DEFAULT_HEIGHT = 720;

    SDL2Application();
    ~SDL2Application();

    // Non-copyable
    SDL2Application(const SDL2Application&) = delete;
    SDL2Application& operator=(const SDL2Application&) = delete;

    /// Initialize SDL2, create window + renderer, bootstrap apc::Application.
    bool init(int width = DEFAULT_WIDTH, int height = DEFAULT_HEIGHT);

    /// Load the vertical slice scenario (soccer match with default config).
    void setup_vertical_slice();

    /// Run the main loop. Returns when the window is closed.
    void run();

private:
    // --- SDL handles ---
    SDL_Window*   window_   = nullptr;
    SDL_Renderer* renderer_ = nullptr;
    int           width_    = DEFAULT_WIDTH;
    int           height_   = DEFAULT_HEIGHT;

    // --- Camera ---
    Camera camera_;

    // --- Input state ---
    const uint8_t* keys_ = nullptr;

    // --- THE CORE ENGINE ---
    apc::Application app_;

    // --- Methods ---
    void process_events();
    void handle_input(float real_dt);
    void render();

    // --- Drawing helpers ---
    void draw_circle(float world_x, float world_y, float world_radius,
                     uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255);
    void draw_ground_grid();
    void draw_hud();

    void shutdown();
};

} // namespace apc
