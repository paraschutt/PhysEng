#pragma once
// =============================================================================
// APC SDL2 Visualizer
// =============================================================================
//
// Real-time 2D rendering of the deterministic physics engine using SDL2.
// Projects the 3D simulation onto the XY plane (top-down view) with a
// camera that can be panned and zoomed.
//
// Controls:
//   Arrow keys / WASD — pan camera
//   Mouse wheel       — zoom in/out
//   R                 — reset simulation
//   SPACE             — pause/unpause
//   ESC               — quit
//
// Cross-platform: Windows (MSVC/vcpkg), Linux (apt), macOS (brew)
//
// =============================================================================

#include "apc_solver/apc_rigid_body.h"
#include "apc_collision/apc_collision_dispatch.h"
#include "apc_solver/apc_si_solver_3d.h"

#include <cstdint>
#include <vector>

struct SDL_Window;
struct SDL_Renderer;

namespace apc {

// ---------------------------------------------------------------------------
// Renderable entity — links a RigidBody to a visual CollisionShape + color
// ---------------------------------------------------------------------------
struct RenderEntity {
    RigidBody        body;
    CollisionShape   shape;
    uint8_t          color_r = 255;
    uint8_t          color_g = 100;
    uint8_t          color_b = 50;
    uint8_t          color_a = 255;
};

// ---------------------------------------------------------------------------
// Camera — 2D orthographic projection with pan/zoom
// ---------------------------------------------------------------------------
struct Camera {
    float center_x = 0.0f;
    float center_y = 0.0f;
    float zoom     = 20.0f;  // pixels per meter

    // World → screen
    float world_to_screen_x(float wx, int screen_w) const {
        return screen_w * 0.5f + (wx - center_x) * zoom;
    }
    float world_to_screen_y(float wy, int screen_h) const {
        return screen_h * 0.5f - (wy - center_y) * zoom; // Y-up to screen Y-down
    }
    float world_to_screen_scale(float size) const {
        return size * zoom;
    }
};

// ---------------------------------------------------------------------------
// SDL2Application — manages window, input, physics, and rendering
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

    /// Initialize SDL2, create window + renderer. Returns false on failure.
    bool init(int width = DEFAULT_WIDTH, int height = DEFAULT_HEIGHT);

    /// Set up a demo scene with bouncing spheres and a ground plane.
    void setup_demo_scene();

    /// Run the main loop. Returns when the window is closed.
    void run();

private:
    // --- SDL handles ---
    SDL_Window*   window_   = nullptr;
    SDL_Renderer* renderer_ = nullptr;
    int           width_    = DEFAULT_WIDTH;
    int           height_   = DEFAULT_HEIGHT;

    // --- Simulation state ---
    std::vector<RenderEntity>  entities_;
    Solver3D                   solver_;
    CollisionShape             ground_plane_;
    bool                       paused_    = false;
    float                      sim_time_  = 0.0f;
    static constexpr float     FIXED_DT   = 1.0f / 240.0f; // 240 Hz physics

    // --- Camera ---
    Camera camera_;

    // --- Input state ---
    // SDL_GetKeyboardState() returns scancode-based array (always valid).
    // Scancodes are small integers (0-511), safe to index.
    const uint8_t* keys_ = nullptr;

    // --- Methods ---
    void process_events();
    void handle_input(float real_dt);
    void step_physics();
    void detect_and_solve_collisions();
    void render();

    // --- Drawing helpers ---
    void draw_circle(float world_x, float world_y, float world_radius,
                     uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255);
    void draw_box(const Vec3& pos, const Vec3& half_extents, const Mat3& rot,
                  uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255);
    void draw_ground_grid();
    void draw_hud();

    void shutdown();
};

} // namespace apc
