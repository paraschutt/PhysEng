// =============================================================================
// APC SDL2 Visualizer — Implementation
// =============================================================================

#include "apc_sdl2_application.h"

#include <SDL2/SDL.h>
#include <cstdio>
#include <cmath>

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
        "APC Physics Engine — SDL2 Visualizer",
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
        std::fprintf(stderr, "[APC SDL2] SDL_CreateRenderer failed: %s\n", SDL_GetError());
        // Fall back to software renderer
        renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_SOFTWARE);
        if (!renderer_) {
            std::fprintf(stderr, "[APC SDL2] Software renderer also failed: %s\n", SDL_GetError());
            shutdown();
            return false;
        }
    }

    SDL_RendererInfo renderer_info;
    SDL_GetRendererInfo(renderer_, &renderer_info);
    std::printf("[APC SDL2] Initialized: %dx%d window, %s renderer\n",
                width_, height_,
                (renderer_info.flags & SDL_RENDERER_ACCELERATED)
                    ? "hardware" : "software");

    return true;
}

// ---------------------------------------------------------------------------
// Demo Scene Setup
// ---------------------------------------------------------------------------

void SDL2Application::setup_demo_scene() {
    entities_.clear();
    solver_.clear();
    sim_time_ = 0.0f;

    // Solver configuration
    solver_.friction_coefficient = 0.4f;
    solver_.baumgarte_factor     = 0.2f;
    solver_.baumgarte_slop       = 0.005f;
    solver_.restitution          = 0.5f;  // Bouncy for visual interest
    solver_.velocity_iterations  = 8;
    solver_.linear_damping       = 0.999f;
    solver_.angular_damping      = 0.998f;

    // Ground plane at y = 0, normal pointing up
    ground_plane_ = CollisionShape::make_plane(
        Vec3(0.0f, 0.0f, 0.0f),
        Vec3(0.0f, 1.0f, 0.0f)
    );

    // --- Create bouncing spheres in a grid pattern ---
    struct SphereDef {
        float x, y, z, r, mass;
        uint8_t cr, cg, cb;
    };

    SphereDef defs[] = {
        // Row 1 — large spheres
        { -4.0f,  8.0f, 0.0f,  0.5f, 1.0f,  230,  70,  70 },   // Red
        { -2.0f, 10.0f, 0.0f,  0.5f, 1.0f,  70,  180,  70 },   // Green
        {  0.0f, 12.0f, 0.0f,  0.5f, 1.0f,  70,  70, 230 },   // Blue
        {  2.0f, 10.0f, 0.0f,  0.5f, 1.0f, 230, 180,  70 },   // Orange
        {  4.0f,  8.0f, 0.0f,  0.5f, 1.0f, 180,  70, 230 },   // Purple

        // Row 2 — medium spheres
        { -3.0f, 15.0f, 0.0f,  0.35f, 0.5f, 230, 130, 130 },  // Light red
        { -1.0f, 18.0f, 0.0f,  0.35f, 0.5f, 130, 230, 130 },  // Light green
        {  1.0f, 20.0f, 0.0f,  0.35f, 0.5f, 130, 130, 230 },  // Light blue
        {  3.0f, 15.0f, 0.0f,  0.35f, 0.5f, 230, 230, 100 },  // Yellow

        // Row 3 — small spheres
        { -5.0f, 22.0f, 0.0f,  0.2f, 0.2f, 200, 200, 200 },  // White
        { -3.5f, 25.0f, 0.0f,  0.2f, 0.2f, 255, 150, 200 },  // Pink
        { -2.0f, 28.0f, 0.0f,  0.2f, 0.2f, 100, 255, 255 },  // Cyan
        {  2.0f, 28.0f, 0.0f,  0.2f, 0.2f, 255, 255, 150 },  // Light yellow
        {  3.5f, 25.0f, 0.0f,  0.2f, 0.2f, 150, 200, 255 },  // Sky
        {  5.0f, 22.0f, 0.0f,  0.2f, 0.2f, 200, 255, 200 },  // Mint

        // Some off-center for asymmetry
        { -6.0f, 14.0f, 0.0f,  0.4f, 0.8f, 180, 100,  50 },  // Brown
        {  6.0f, 14.0f, 0.0f,  0.4f, 0.8f,  50, 100, 180 },  // Navy

        // A couple of spheres with initial lateral velocity
        { -7.0f, 20.0f, 0.0f,  0.3f, 0.4f, 255, 100, 100 },  // Moving right
        {  7.0f, 20.0f, 0.0f,  0.3f, 0.4f, 100, 100, 255 },  // Moving left
    };

    for (const auto& d : defs) {
        RenderEntity e;
        e.body.position = Vec3(d.x, d.y, d.z);
        e.body.linear_velocity = Vec3(
            (d.x > 5.0f) ? -3.0f : (d.x < -5.0f) ? 3.0f : 0.0f,
            0.0f,
            0.0f
        );
        e.body.orientation = Quat::identity();
        e.body.angular_velocity = Vec3(0.0f, 0.0f, 0.0f);
        e.body.inverse_mass = 1.0f / d.mass;

        // Approximate sphere inertia: I = 2/5 * m * r^2 → inv_I = 5/(2*m*r^2)
        float inv_inertia = 5.0f / (2.0f * d.mass * d.r * d.r);
        e.body.local_inverse_inertia = Mat3{{
            inv_inertia, 0.0f, 0.0f,
            0.0f, inv_inertia, 0.0f,
            0.0f, 0.0f, inv_inertia
        }};
        e.body.update_world_inertia();

        e.shape = CollisionShape::make_sphere(d.r, e.body.position);
        e.shape.update_cache();

        e.color_r = d.cr;
        e.color_g = d.cg;
        e.color_b = d.cb;
        e.color_a = 255;

        entities_.push_back(e);
    }

    std::printf("[APC SDL2] Demo scene: %u entities, ground plane at y=0\n",
                (uint32_t)entities_.size());
}

// ---------------------------------------------------------------------------
// Main Loop
// ---------------------------------------------------------------------------

void SDL2Application::run() {
    if (!window_ || !renderer_) return;

    setup_demo_scene();

    Uint64 prev_ticks = SDL_GetPerformanceCounter();
    float accumulator = 0.0f;

    bool running = true;
    while (running) {
        Uint64 now_ticks = SDL_GetPerformanceCounter();
        float frame_dt = (float)(now_ticks - prev_ticks) / (float)SDL_GetPerformanceFrequency();
        prev_ticks = now_ticks;

        // Clamp frame_dt to prevent spiral of death
        if (frame_dt > 0.1f) frame_dt = 0.1f;

        // --- Events ---
        process_events();
        if (!window_) break;

        // --- Input (camera) ---
        handle_input(frame_dt);

        // --- Physics (fixed timestep) ---
        if (!paused_) {
            accumulator += frame_dt;
            while (accumulator >= FIXED_DT) {
                step_physics();
                accumulator -= FIXED_DT;
            }
        }

        // --- Render ---
        render();

        // Cap to ~60 FPS display
        SDL_Delay(1);
    }
}

// ---------------------------------------------------------------------------
// Event Processing
// ---------------------------------------------------------------------------

void SDL2Application::process_events() {
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
            if (event.key.keysym.sym < 256) {
                keys_down_[event.key.keysym.sym] = true;
            }
            if (event.key.keysym.sym == SDLK_ESCAPE) {
                shutdown();
                return;
            }
            if (event.key.keysym.sym == SDLK_r) {
                setup_demo_scene();
            }
            if (event.key.keysym.sym == SDLK_SPACE) {
                paused_ = !paused_;
            }
            break;

        case SDL_KEYUP:
            if (event.key.keysym.sym < 256) {
                keys_down_[event.key.keysym.sym] = false;
            }
            break;

        case SDL_MOUSEWHEEL:
            camera_.zoom *= (event.wheel.y > 0) ? 1.1f : 0.9f;
            // Clamp zoom
            if (camera_.zoom < 2.0f)  camera_.zoom = 2.0f;
            if (camera_.zoom > 200.0f) camera_.zoom = 200.0f;
            break;
        }
    }
}

// ---------------------------------------------------------------------------
// Input Handling (Camera pan)
// ---------------------------------------------------------------------------

void SDL2Application::handle_input(float dt) {
    float pan_speed = 5.0f / camera_.zoom * 100.0f;

    if (keys_down_[SDLK_LEFT]  || keys_down_[SDLK_a]) camera_.center_x -= pan_speed * dt;
    if (keys_down_[SDLK_RIGHT] || keys_down_[SDLK_d]) camera_.center_x += pan_speed * dt;
    if (keys_down_[SDLK_UP]    || keys_down_[SDLK_w]) camera_.center_y += pan_speed * dt;
    if (keys_down_[SDLK_DOWN]  || keys_down_[SDLK_s]) camera_.center_y -= pan_speed * dt;
}

// ---------------------------------------------------------------------------
// Physics Step
// ---------------------------------------------------------------------------

void SDL2Application::step_physics() {
    // --- Gravity ---
    for (auto& e : entities_) {
        if (e.body.inverse_mass > 0.0f) {
            e.body.linear_velocity = Vec3::add(
                e.body.linear_velocity,
                Vec3(0.0f, -9.81f * FIXED_DT, 0.0f)
            );
        }
    }

    // --- Collision detection + solving ---
    detect_and_solve_collisions();

    // --- Integration ---

    // Build a temporary body vector for integration
    std::vector<RigidBody> bodies;
    bodies.reserve(entities_.size());
    for (auto& e : entities_) {
        bodies.push_back(e.body);
    }

    // Integrate all dynamic bodies
    for (auto& body : bodies) {
        if (body.inverse_mass == 0.0f) continue;
        body.position = Vec3::add(body.position, Vec3::scale(body.linear_velocity, FIXED_DT));

        // Simple orientation update (angular velocity mostly cosmetic for spheres)
        Vec3 half_ang = Vec3::scale(body.angular_velocity, FIXED_DT * 0.5f);
        Quat delta_q(half_ang.x, half_ang.y, half_ang.z, 0.0f);
        body.orientation = Quat::normalize(Quat::multiply(delta_q, body.orientation));
        body.update_world_inertia();

        body.linear_velocity = Vec3::scale(body.linear_velocity, solver_.linear_damping);
        body.angular_velocity = Vec3::scale(body.angular_velocity, solver_.angular_damping);
    }

    // Write back
    for (size_t i = 0; i < entities_.size(); ++i) {
        entities_[i].body = bodies[i];
        entities_[i].shape.position = bodies[i].position;
    }

    sim_time_ += FIXED_DT;
}

// ---------------------------------------------------------------------------
// Collision Detection & Solving
// ---------------------------------------------------------------------------

void SDL2Application::detect_and_solve_collisions() {
    uint32_t n = (uint32_t)entities_.size();

    // --- Ground collisions with direct impulse ---
    for (auto& e : entities_) {
        if (e.body.inverse_mass == 0.0f) continue;

        SphereCollider col;
        col.radius = e.shape.sphere_radius;
        PlaneCollider plane;
        plane.point  = ground_plane_.position;
        plane.normal = ground_plane_.plane_normal;

        ContactPoint contact;
        if (detect_sphere_plane(e.body.position, col, plane, contact)) {
            // Relative velocity at contact
            float vel_along_normal = Vec3::dot(e.body.linear_velocity, contact.normal);

            // Position correction
            float positional_error = std::max(contact.penetration - solver_.baumgarte_slop, 0.0f)
                                     * solver_.baumgarte_factor / FIXED_DT;

            // Restitution
            float restitution_bias = 0.0f;
            if (vel_along_normal < -APC_EPSILON && contact.penetration < solver_.baumgarte_slop * 2.0f) {
                restitution_bias = -solver_.restitution * vel_along_normal;
            }

            float inv_mass = e.body.inverse_mass; // Ground has 0
            float normal_mass = (inv_mass > 0.0f) ? (1.0f / inv_mass) : 0.0f;
            float delta_n = normal_mass * (-vel_along_normal + positional_error + restitution_bias);
            if (delta_n > 0.0f) {
                Vec3 impulse = Vec3::scale(contact.normal, delta_n);
                e.body.linear_velocity = Vec3::add(e.body.linear_velocity, Vec3::scale(impulse, inv_mass));
            }
        }
    }

    // --- Sphere-sphere with direct impulse ---
    for (uint32_t i = 0; i < n; ++i) {
        for (uint32_t j = i + 1; j < n; ++j) {
            SphereCollider col_a;
            col_a.radius = entities_[i].shape.sphere_radius;
            SphereCollider col_b;
            col_b.radius = entities_[j].shape.sphere_radius;

            ContactPoint contact;
            if (detect_sphere_sphere(
                    entities_[i].body.position, col_a,
                    entities_[j].body.position, col_b, contact)) {

                RigidBody& a = entities_[i].body;
                RigidBody& b = entities_[j].body;

                float inv_mass_sum = a.inverse_mass + b.inverse_mass;
                if (inv_mass_sum <= 0.0f) continue;

                float normal_mass = 1.0f / inv_mass_sum;

                float vel_along_normal = Vec3::dot(
                    Vec3::sub(a.linear_velocity, b.linear_velocity),
                    contact.normal
                );

                float positional_error = std::max(contact.penetration - solver_.baumgarte_slop, 0.0f)
                                         * solver_.baumgarte_factor / FIXED_DT;

                float restitution_bias = 0.0f;
                if (vel_along_normal < -APC_EPSILON && contact.penetration < solver_.baumgarte_slop * 2.0f) {
                    restitution_bias = -solver_.restitution * vel_along_normal;
                }

                float delta_n = normal_mass * (-vel_along_normal + positional_error + restitution_bias);
                if (delta_n > 0.0f) {
                    Vec3 impulse = Vec3::scale(contact.normal, delta_n);
                    a.linear_velocity = Vec3::add(a.linear_velocity, Vec3::scale(impulse, a.inverse_mass));
                    b.linear_velocity = Vec3::sub(b.linear_velocity, Vec3::scale(impulse, b.inverse_mass));
                }

                // Friction (simplified)
                Vec3 rel_vel = Vec3::sub(a.linear_velocity, b.linear_velocity);
                float vel_n = Vec3::dot(rel_vel, contact.normal);
                Vec3 vel_tangent = Vec3::sub(rel_vel, Vec3::scale(contact.normal, vel_n));
                float tangent_len_sq = Vec3::length_sq(vel_tangent);

                if (tangent_len_sq > APC_EPSILON_SQ) {
                    float tangent_len = std::sqrt(tangent_len_sq);
                    Vec3 tangent = Vec3::scale(vel_tangent, 1.0f / tangent_len);

                    float friction_delta = normal_mass * tangent_len;
                    float max_friction = solver_.friction_coefficient * delta_n;
                    float applied = std::min(friction_delta, max_friction);

                    Vec3 friction_impulse = Vec3::scale(tangent, -applied);
                    a.linear_velocity = Vec3::add(a.linear_velocity, Vec3::scale(friction_impulse, a.inverse_mass));
                    b.linear_velocity = Vec3::sub(b.linear_velocity, Vec3::scale(friction_impulse, b.inverse_mass));
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------

void SDL2Application::render() {
    // Clear — dark background
    SDL_SetRenderDrawColor(renderer_, 15, 15, 25, 255);
    SDL_RenderClear(renderer_);

    // Ground grid
    draw_ground_grid();

    // Draw entities
    for (const auto& e : entities_) {
        if (e.shape.type == ShapeType::Sphere) {
            draw_circle(
                e.body.position.x, e.body.position.y,
                e.shape.sphere_radius,
                e.color_r, e.color_g, e.color_b, e.color_a
            );
        }
        // Future: boxes, capsules, etc.
    }

    // HUD overlay
    draw_hud();

    // Present
    SDL_RenderPresent(renderer_);
}

// ---------------------------------------------------------------------------
// Drawing: Circle (approximated with line segments)
// ---------------------------------------------------------------------------

void SDL2Application::draw_circle(float wx, float wy, float wr,
                                   uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
    float sx = camera_.world_to_screen_x(wx, width_);
    float sy = camera_.world_to_screen_y(wy, height_);
    float sr = camera_.world_to_screen_scale(wr);

    // Cull off-screen
    if (sx + sr < 0 || sx - sr > width_ || sy + sr < 0 || sy - sr > height_) return;
    if (sr < 0.5f) return;

    // Filled circle with triangles (using SDL_RenderDrawLine for speed)
    int segments = (int)(sr * 1.5f);
    if (segments < 8) segments = 8;
    if (segments > 64) segments = 64;

    // Fill — draw filled triangle fan using lines from center to perimeter
    SDL_SetRenderDrawColor(renderer_, r, g, b, a);

    // Simple fill: draw horizontal scan lines (brute force but simple)
    int r_int = (int)(sr + 0.5f);
    int cx = (int)sx;
    int cy = (int)sy;

    for (int dy = -r_int; dy <= r_int; ++dy) {
        float dx_sq = (float)(sr * sr - dy * dy);
        if (dx_sq < 0) continue;
        int dx = (int)(std::sqrt(dx_sq) + 0.5f);
        SDL_RenderDrawLine(renderer_, cx - dx, cy + dy, cx + dx, cy + dy);
    }

    // Outline — slightly brighter
    SDL_SetRenderDrawColor(renderer_,
        (uint8_t)std::min(255, r + 60),
        (uint8_t)std::min(255, g + 60),
        (uint8_t)std::min(255, b + 60), 255);

    // Draw outline using line segments
    static constexpr int OUTLINE_SEGMENTS = 32;
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
// Drawing: Box (wireframe)
// ---------------------------------------------------------------------------

void SDL2Application::draw_box(const Vec3& pos, const Vec3& he, const Mat3& rot,
                                uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
    // Project 8 corners to screen and draw wireframe
    float corners[8][3] = {
        {-he.x, -he.y, -he.z}, { he.x, -he.y, -he.z},
        {-he.x,  he.y, -he.z}, { he.x,  he.y, -he.z},
        {-he.x, -he.y,  he.z}, { he.x, -he.y,  he.z},
        {-he.x,  he.y,  he.z}, { he.x,  he.y,  he.z}
    };

    float screen_pts[8][2];
    for (int i = 0; i < 8; ++i) {
        Vec3 local(corners[i][0], corners[i][1], corners[i][2]);
        Vec3 world = Vec3::add(pos, rot.transform_vec(local));
        screen_pts[i][0] = camera_.world_to_screen_x(world.x, width_);
        screen_pts[i][1] = camera_.world_to_screen_y(world.y, height_);
    }

    SDL_SetRenderDrawColor(renderer_, r, g, b, a);

    // 12 edges of a cube
    static const int edges[12][2] = {
        {0,1},{0,2},{0,4},{1,3},{1,5},{2,3},
        {2,6},{3,7},{4,5},{4,6},{5,7},{6,7}
    };
    for (const auto& e : edges) {
        SDL_RenderDrawLine(renderer_,
            (int)screen_pts[e[0]][0], (int)screen_pts[e[0]][1],
            (int)screen_pts[e[1]][0], (int)screen_pts[e[1]][1]);
    }
}

// ---------------------------------------------------------------------------
// Drawing: Ground Grid
// ---------------------------------------------------------------------------

void SDL2Application::draw_ground_grid() {
    // Determine visible world range
    float left   = camera_.center_x - (float)width_  / (2.0f * camera_.zoom);
    float right  = camera_.center_x + (float)width_  / (2.0f * camera_.zoom);
    float bottom = camera_.center_y - (float)height_ / (2.0f * camera_.zoom);
    float top    = camera_.center_y + (float)height_ / (2.0f * camera_.zoom);

    // Grid spacing adapts to zoom level
    float grid_spacing = 1.0f;
    if (camera_.zoom < 10.0f)  grid_spacing = 5.0f;
    if (camera_.zoom < 4.0f)   grid_spacing = 10.0f;
    if (camera_.zoom > 50.0f)  grid_spacing = 0.5f;

    // Ground line at y=0
    float ground_sy = camera_.world_to_screen_y(0.0f, height_);
    if (ground_sy >= 0 && ground_sy <= height_) {
        // Ground fill (dark green-brown below ground)
        SDL_SetRenderDrawColor(renderer_, 25, 35, 20, 255);
        SDL_Rect ground_rect;
        ground_rect.x = 0;
        ground_rect.y = (int)ground_sy;
        ground_rect.w = width_;
        ground_rect.h = height_ - (int)ground_sy;
        SDL_RenderFillRect(renderer_, &ground_rect);

        // Ground line
        SDL_SetRenderDrawColor(renderer_, 80, 120, 60, 255);
        SDL_RenderDrawLine(renderer_, 0, (int)ground_sy, width_, (int)ground_sy);
    }

    // Vertical grid lines
    float x_start = std::floor(left / grid_spacing) * grid_spacing;
    for (float x = x_start; x <= right; x += grid_spacing) {
        int sx = (int)camera_.world_to_screen_x(x, width_);
        // Major/minor distinction
        bool is_major = std::fmod(std::abs(x), grid_spacing * 5.0f) < APC_EPSILON;
        SDL_SetRenderDrawColor(renderer_,
            is_major ? 50 : 30, is_major ? 50 : 30, is_major ? 55 : 35, 255);
        SDL_RenderDrawLine(renderer_, sx, 0, sx, height_);
    }

    // Horizontal grid lines (in world Y direction, shown above ground)
    float y_start = std::floor(std::max(0.0f, bottom) / grid_spacing) * grid_spacing;
    for (float y = y_start; y <= top; y += grid_spacing) {
        int sy = (int)camera_.world_to_screen_y(y, height_);
        bool is_major = std::fmod(std::abs(y), grid_spacing * 5.0f) < APC_EPSILON;
        SDL_SetRenderDrawColor(renderer_,
            is_major ? 50 : 30, is_major ? 50 : 30, is_major ? 55 : 35, 255);
        SDL_RenderDrawLine(renderer_, 0, sy, width_, sy);
    }
}

// ---------------------------------------------------------------------------
// Drawing: HUD (Heads-Up Display)
// ---------------------------------------------------------------------------

void SDL2Application::draw_hud() {
    // Simple text overlay using basic SDL (no SDL_ttf dependency)
    // We draw a semi-transparent background bar at the top

    SDL_SetRenderDrawBlendMode(renderer_, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 140);
    SDL_Rect hud_rect;
    hud_rect.x = 0;
    hud_rect.y = 0;
    hud_rect.w = width_;
    hud_rect.h = 36;
    SDL_RenderFillRect(renderer_, &hud_rect);
    SDL_SetRenderDrawBlendMode(renderer_, SDL_BLENDMODE_NONE);

    // FPS and info text — using basic rectangles as "LED indicators" for state
    // (We avoid SDL_ttf to keep dependencies minimal)

    // Simulation time indicator (bar at top-left)
    float time_bar_width = std::fmod(sim_time_, 10.0f) / 10.0f * 100.0f;
    SDL_SetRenderDrawColor(renderer_, 100, 200, 100, 255);
    SDL_Rect time_bar;
    time_bar.x = 10;
    time_bar.y = 12;
    time_bar.w = (int)time_bar_width;
    time_bar.h = 12;
    SDL_RenderFillRect(renderer_, &time_bar);

    // Entity count indicator (dots)
    uint32_t count = (uint32_t)entities_.size();
    for (uint32_t i = 0; i < count && i < 20; ++i) {
        SDL_SetRenderDrawColor(renderer_,
            entities_[i].color_r, entities_[i].color_g, entities_[i].color_b, 255);
        SDL_Rect dot;
        dot.x = 130 + (int)i * 14;
        dot.y = 14;
        dot.w = 8;
        dot.h = 8;
        SDL_RenderFillRect(renderer_, &dot);
    }

    // Pause indicator (red square if paused)
    if (paused_) {
        SDL_SetRenderDrawColor(renderer_, 255, 60, 60, 255);
        SDL_Rect pause_box;
        pause_box.x = width_ - 50;
        pause_box.y = 8;
        pause_box.w = 20;
        pause_box.h = 20;
        SDL_RenderFillRect(renderer_, &pause_box);

        pause_box.x = width_ - 25;
        pause_box.y = 8;
        pause_box.w = 20;
        pause_box.h = 20;
        SDL_RenderFillRect(renderer_, &pause_box);
    }

    // Zoom indicator (bottom-right)
    SDL_SetRenderDrawBlendMode(renderer_, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 100);
    SDL_Rect zoom_bg;
    zoom_bg.x = width_ - 140;
    zoom_bg.y = height_ - 30;
    zoom_bg.w = 130;
    zoom_bg.h = 24;
    SDL_RenderFillRect(renderer_, &zoom_bg);
    SDL_SetRenderDrawBlendMode(renderer_, SDL_BLENDMODE_NONE);

    // Zoom bar
    float zoom_pct = (camera_.zoom - 2.0f) / (200.0f - 2.0f);
    SDL_SetRenderDrawColor(renderer_, 150, 150, 200, 255);
    SDL_Rect zoom_bar;
    zoom_bar.x = width_ - 130;
    zoom_bar.y = height_ - 22;
    zoom_bar.w = (int)(zoom_pct * 100.0f);
    zoom_bar.h = 8;
    SDL_RenderFillRect(renderer_, &zoom_bar);
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

    std::printf("=== APC Physics Engine — SDL2 Visualizer ===\n");
    std::printf("Controls: WASD/Arrows=Pan, Scroll=Zoom, R=Reset, Space=Pause, Esc=Quit\n\n");

    apc::SDL2Application app;

    if (!app.init()) {
        std::fprintf(stderr, "Failed to initialize SDL2 visualizer.\n");
        return 1;
    }

    app.run();

    std::printf("\n[APC SDL2] Shut down cleanly.\n");
    return 0;
}
