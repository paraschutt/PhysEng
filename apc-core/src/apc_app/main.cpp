// =============================================================================
// main.cpp — Entry point for the APC Impact League application
// =============================================================================
//
// Runs a headless simulation of a soccer match:
//   - Loads 4-4-2 vs 4-3-3 formation
//   - Steps physics at 240Hz with fixed timestep
//   - AI controls all 22 athletes via steering + motor intent pipeline
//   - Prints match status every simulated second
//   - Terminates after a configurable number of simulated seconds
//
// Build: cmake --build build --target apc_application
// =============================================================================

#include "apc_app/apc_application.h"
#include <cstdio>
#include <chrono>
#include <thread>

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
static const double SIMULATION_DURATION_SECONDS = 10.0;   // Simulate 10 seconds
static const double STATUS_PRINT_INTERVAL      = 1.0;     // Print every 1 sim-second
static const double TARGET_FRAME_TIME           = 1.0 / 60.0; // 60 FPS cap

// ---------------------------------------------------------------------------
// print_match_status — Console output of current match state
// ---------------------------------------------------------------------------
static void print_match_status(const apc::Application& app, double sim_time,
                                double wall_elapsed)
{
    const apc::SceneState& scene = app.scene;

    // Count athletes per team
    uint32_t home_count = scene.entity_manager.get_team_athlete_count(apc::TEAM_HOME);
    uint32_t away_count = scene.entity_manager.get_team_athlete_count(apc::TEAM_AWAY);

    // Count active AI vs human
    uint32_t ai_active = 0;
    uint32_t human_count = 0;
    for (uint32_t i = 0u; i < scene.entity_manager.athlete_count; ++i) {
        const apc::AthleteEntity& a = scene.entity_manager.athletes[i];
        if (!a.id.is_valid()) continue;
        if (a.is_human_controlled) {
            ++human_count;
        } else {
            ++ai_active;
        }
    }

    // Get ball info
    const apc::BallEntity* ball = scene.entity_manager.find_ball();

    // Physics stats
    uint32_t physics_steps = app.game_loop.time.physics_step_count;
    uint32_t frame_count   = app.game_loop.time.frame_count;

    std::printf("\n");
    std::printf("=== APC Impact League — Sim Time: %.1fs (wall: %.2fs) ===\n",
                sim_time, wall_elapsed);
    std::printf("  Score: Home %u - %u Away\n", scene.home_score, scene.away_score);
    std::printf("  Athletes: Home=%u  Away=%u  (AI=%u  Human=%u)\n",
                home_count, away_count, ai_active, human_count);

    if (ball) {
        float ball_speed = apc::Vec3::length(ball->velocity);
        std::printf("  Ball: pos=(%.2f, %.2f, %.2f)  speed=%.2f m/s  type=%u\n",
                    ball->position.x, ball->position.y, ball->position.z,
                    ball_speed, ball->ball_type);
    }

    // Show a few sample athlete positions and actions
    std::printf("  Sample positions:\n");
    uint32_t shown = 0;
    for (uint32_t i = 0u; i < scene.entity_manager.athlete_count && shown < 6u; ++i) {
        const apc::AthleteEntity& a = scene.entity_manager.athletes[i];
        if (!a.id.is_valid()) continue;
        float speed = apc::Vec3::length(a.velocity);
        const char* team_str = (a.team == apc::TEAM_HOME) ? "H" : "A";

        // Decode AI action from flags
        const char* action_str = "IDLE";
        apc::AIActionType action = static_cast<apc::AIActionType>(a.flags & 0xFFu);
        switch (action) {
        case apc::AIActionType::CHASE_BALL:     action_str = "CHASE"; break;
        case apc::AIActionType::SHOOT_BALL:     action_str = "SHOOT"; break;
        case apc::AIActionType::PASS_BALL:      action_str = "PASS";  break;
        case apc::AIActionType::TACKLE:         action_str = "TACKL"; break;
        case apc::AIActionType::MOVE_TO_POSITION: action_str = "MOVE"; break;
        case apc::AIActionType::SUPPORT_RUN:    action_str = "SUPP";  break;
        case apc::AIActionType::PRESS:          action_str = "PRESS"; break;
        case apc::AIActionType::INTERCEPT:      action_str = "INTRC"; break;
        case apc::AIActionType::FORMATION_HOLD: action_str = "FORM";  break;
        default: break;
        }

        std::printf("    [%s#%02u] pos=(%6.1f, %4.1f, %6.1f)  spd=%5.1f  AI=%s\n",
                    team_str, a.jersey_number,
                    a.position.x, a.position.y, a.position.z,
                    speed, action_str);
        ++shown;
    }

    std::printf("  Physics: %u steps @ 240Hz  |  Frames: %u\n",
                physics_steps, frame_count);

    // Formation info
    std::printf("  Formation: Home=%s  Away=%s\n",
                scene.formation_system.attack_formation.name,
                scene.formation_system.defense_formation.name);

    // Debug draw stats
    std::printf("  Debug Draw: %u lines, %u points\n",
                app.debug_draw_list.get_line_count(),
                app.debug_draw_list.get_point_count());
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main()
{
    using Clock = std::chrono::high_resolution_clock;

    std::printf("============================================\n");
    std::printf(" APC Physics Engine — Impact League\n");
    std::printf(" Headless Soccer Simulation\n");
    std::printf("============================================\n\n");

    // --- Initialize application ---
    apc::Application app;
    apc::ApplicationConfig cfg = apc::Application::soccer_defaults();
    cfg.enable_debug_draw = 1;
    cfg.enable_ai_debug   = 1;

    if (!app.init(cfg)) {
        std::printf("ERROR: Application initialization failed.\n");
        return 1;
    }
    std::printf("[OK] Application initialized\n");

    // --- Load soccer match ---
    apc::MatchConfig match = apc::SceneState::soccer_match();
    if (!app.load_match(match)) {
        std::printf("ERROR: Match loading failed.\n");
        return 1;
    }
    std::printf("[OK] Match loaded: %s vs %s\n",
                match.home_team_name, match.away_team_name);
    std::printf("     Field: %.0fm x %.0fm  |  Duration: %.0fs (%u halves)\n",
                match.field_length, match.field_width,
                match.match_duration_seconds, match.halves);

    // --- Enable AI debug layers ---
    app.ai_debug.set_layer(apc::AIDebugLayer::STEERING_FORCES, 1);
    app.ai_debug.set_layer(apc::AIDebugLayer::FORMATION_POSITIONS, 1);
    app.ai_debug.set_layer(apc::AIDebugLayer::UTILITY_SCORES, 1);
    app.ai_debug.set_layer(apc::AIDebugLayer::MOTOR_INTENT, 1);

    // game_loop.start_playing() is called automatically by load_match().
    // No manual state override needed.

    // --- Run simulation loop ---
    // We feed monotonically increasing simulated time to the game loop.
    // The accumulator pattern then runs the correct number of physics sub-steps.
    double sim_time = 0.0;
    double next_print_time = STATUS_PRINT_INTERVAL;

    auto wall_start = Clock::now();

    std::printf("\n--- Simulation running (%.0f simulated seconds) ---\n",
                SIMULATION_DURATION_SECONDS);

    while (sim_time < SIMULATION_DURATION_SECONDS) {
        auto frame_start = Clock::now();

        // Advance simulated time by one render frame (1/60s)
        double frame_dt = TARGET_FRAME_TIME;
        sim_time += frame_dt;

        // Feed simulated time to the game loop
        app.begin_frame(sim_time);
        app.tick();
        app.end_frame();

        // Print status at intervals
        if (sim_time >= next_print_time) {
            auto wall_now = Clock::now();
            std::chrono::duration<double> wall_elapsed = wall_now - wall_start;
            print_match_status(app, sim_time, wall_elapsed.count());
            next_print_time += STATUS_PRINT_INTERVAL;
        }

        // Frame rate limiter: sleep to maintain ~60 FPS wall-clock
        auto frame_end = Clock::now();
        std::chrono::duration<double> frame_elapsed = frame_end - frame_start;
        double sleep_time = TARGET_FRAME_TIME - frame_elapsed.count();
        if (sleep_time > 0.001) {
            std::this_thread::sleep_for(
                std::chrono::duration<double>(sleep_time));
        }
    }

    auto wall_finish = Clock::now();
    std::chrono::duration<double> total_wall = wall_finish - wall_start;

    // --- Final status ---
    print_match_status(app, sim_time, total_wall.count());

    // --- Summary ---
    std::printf("\n============================================\n");
    std::printf(" SIMULATION COMPLETE\n");
    std::printf("============================================\n");
    std::printf("  Simulated time: %.2f seconds\n", sim_time);
    std::printf("  Wall time:      %.2f seconds\n", total_wall.count());
    std::printf("  Physics steps:  %u\n", app.game_loop.time.physics_step_count);
    std::printf("  Render frames:  %u\n", app.game_loop.time.frame_count);
    std::printf("  Final score:    Home %u - %u Away\n",
                app.scene.home_score, app.scene.away_score);
    std::printf("  Athletes:       %u spawned\n",
                app.scene.entity_manager.athlete_count);
    std::printf("  Debug lines:    %u drawn\n",
                app.debug_draw_list.get_line_count());
    std::printf("============================================\n");

    // --- Shutdown ---
    app.shutdown();

    std::printf("\n[OK] Clean shutdown.\n");
    return 0;
}
