// =============================================================================
// det_game_loop.cpp — Determinism verification for the full game loop pipeline
// =============================================================================
//
// Runs the Application (game loop + AI + physics) for a fixed number of steps,
// computes a state hash, then resets and re-runs. If the two hashes differ,
// the simulation is NOT deterministic.
//
// This verifies the entire pipeline:
//   GameLoop → SceneState.update → AI decision → steering → motor → physics
//
// Build: linked via tests/determinism/CMakeLists.txt
//
// =============================================================================

#include "apc_platform/apc_fp_mode.h"
#include "apc_app/apc_application.h"
#include "apc_app/apc_scene_manager.h"
#include "apc_entity/apc_entity_types.h"

#include <cstdio>
#include <cstdint>
#include <cstring>

namespace {

// =============================================================================
// FNV-1a hash for raw bytes
// =============================================================================
inline uint64_t fnv1a_bytes(const void* data, size_t length) {
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    uint64_t hash = 0xcbf29ce484222325ULL;
    for (size_t i = 0; i < length; ++i) {
        hash ^= bytes[i];
        hash *= 0x100000001b3ULL;
    }
    return hash;
}

// =============================================================================
// Hash a single float (bit-exact, no tolerance)
// =============================================================================
inline uint64_t hash_float(float f) {
    uint32_t bits;
    std::memcpy(&bits, &f, sizeof(bits));
    uint64_t h = 0x9e3779b97f4a7c15ULL;
    h ^= bits + 0x9e3779b97f4a7c15ULL + (h << 12) + (h >> 4);
    return h;
}

// =============================================================================
// Hash the full simulation state (deterministic across runs)
// =============================================================================
uint64_t hash_simulation_state(const apc::Application& app) {
    uint64_t h = 0xcbf29ce484222325ULL; // FNV-1a offset basis

    const apc::SceneState& scene = app.scene;
    const apc::EntityManager& em = scene.entity_manager;

    // --- Hash all athlete states (fixed iteration order) ---
    for (uint32_t i = 0u; i < em.athlete_count; ++i) {
        const apc::AthleteEntity& a = em.athletes[i];
        if (!a.id.is_valid()) continue;

        // Position (3 floats)
        h ^= hash_float(a.position.x);
        h *= 0x100000001b3ULL;
        h ^= hash_float(a.position.y);
        h *= 0x100000001b3ULL;
        h ^= hash_float(a.position.z);
        h *= 0x100000001b3ULL;

        // Velocity (3 floats)
        h ^= hash_float(a.velocity.x);
        h *= 0x100000001b3ULL;
        h ^= hash_float(a.velocity.y);
        h *= 0x100000001b3ULL;
        h ^= hash_float(a.velocity.z);
        h *= 0x100000001b3ULL;

        // Stamina, health
        h ^= hash_float(a.stamina);
        h *= 0x100000001b3ULL;
        h ^= hash_float(a.health);
        h *= 0x100000001b3ULL;

        // Cooldowns
        h ^= hash_float(a.sprint_cooldown);
        h *= 0x100000001b3ULL;
        h ^= hash_float(a.tackle_cooldown);
        h *= 0x100000001b3ULL;
    }

    // --- Hash all ball states ---
    for (uint32_t i = 0u; i < em.ball_count; ++i) {
        const apc::BallEntity& b = em.balls[i];
        if (!b.id.is_valid()) continue;

        // Position
        h ^= hash_float(b.position.x);
        h *= 0x100000001b3ULL;
        h ^= hash_float(b.position.y);
        h *= 0x100000001b3ULL;
        h ^= hash_float(b.position.z);
        h *= 0x100000001b3ULL;

        // Velocity
        h ^= hash_float(b.velocity.x);
        h *= 0x100000001b3ULL;
        h ^= hash_float(b.velocity.y);
        h *= 0x100000001b3ULL;
        h ^= hash_float(b.velocity.z);
        h *= 0x100000001b3ULL;

        // Angular velocity
        h ^= hash_float(b.angular_velocity.x);
        h *= 0x100000001b3ULL;
        h ^= hash_float(b.angular_velocity.y);
        h *= 0x100000001b3ULL;
        h ^= hash_float(b.angular_velocity.z);
        h *= 0x100000001b3ULL;

        // Possession
        h ^= static_cast<uint64_t>(b.possession_team);
        h *= 0x100000001b3ULL;
        h ^= static_cast<uint64_t>(b.is_in_play);
        h *= 0x100000001b3ULL;
    }

    // --- Hash scores ---
    h ^= static_cast<uint64_t>(scene.home_score);
    h *= 0x100000001b3ULL;
    h ^= static_cast<uint64_t>(scene.away_score);
    h *= 0x100000001b3ULL;

    // --- Hash match time ---
    h ^= hash_float(scene.match_time_seconds);
    h *= 0x100000001b3ULL;

    // --- Hash physics step count ---
    h ^= static_cast<uint64_t>(app.game_loop.time.physics_step_count);
    h *= 0x100000001b3ULL;

    return h;
}

// =============================================================================
// Run simulation for N physics steps, return final state hash
// =============================================================================
uint64_t run_simulation_and_hash(uint32_t physics_steps) {
    apc::Application app;
    apc::ApplicationConfig cfg = apc::Application::soccer_defaults();
    cfg.enable_debug_draw = 0; // Off for determinism test
    cfg.enable_ai_debug   = 0;

    if (!app.init(cfg)) {
        std::fprintf(stderr, "[DET] Application init failed\n");
        return 0;
    }

    apc::MatchConfig match = apc::SceneState::soccer_match();
    if (!app.load_match(match)) {
        std::fprintf(stderr, "[DET] Match loading failed\n");
        return 0;
    }

    // Run exactly N physics steps
    double sim_time = 0.0;
    float dt = app.game_loop.time.fixed_delta;
    uint32_t steps_done = 0;

    while (steps_done < physics_steps) {
        app.begin_frame(sim_time);
        app.tick();

        // Count how many physics steps this tick() performed
        uint32_t new_steps = app.game_loop.time.physics_step_count;
        steps_done = new_steps;

        app.end_frame();
        sim_time += dt;
    }

    uint64_t hash = hash_simulation_state(app);

    app.shutdown();

    return hash;
}

} // anonymous namespace

// =============================================================================
// Public entry point — called from det_main.cpp
// =============================================================================
uint64_t run_det_game_loop() {
    static constexpr uint32_t STEPS = 480; // 2 seconds at 240Hz

    std::fprintf(stdout, "  Running game loop determinism test (%u steps x2)...\n", STEPS);

    // Run 1
    uint64_t hash1 = run_simulation_and_hash(STEPS);
    std::fprintf(stdout, "    Run 1 hash: %016llx\n", (unsigned long long)hash1);

    // Run 2 (must produce identical hash)
    uint64_t hash2 = run_simulation_and_hash(STEPS);
    std::fprintf(stdout, "    Run 2 hash: %016llx\n", (unsigned long long)hash2);

    if (hash1 == hash2) {
        std::fprintf(stdout, "    PASS: Deterministic (hashes match)\n");
    } else {
        std::fprintf(stdout, "    FAIL: NON-DETERMINISTIC (hashes differ!)\n");
    }

    // Also run a shorter test for additional confidence
    uint64_t hash3 = run_simulation_and_hash(240); // 1 second
    std::fprintf(stdout, "    Run 3 hash (240 steps): %016llx\n", (unsigned long long)hash3);

    // hash3 should be different from hash1 (different number of steps),
    // but a second run of 240 steps should match hash3
    uint64_t hash4 = run_simulation_and_hash(240);
    std::fprintf(stdout, "    Run 4 hash (240 steps): %016llx\n", (unsigned long long)hash4);

    if (hash3 == hash4) {
        std::fprintf(stdout, "    PASS: Short run also deterministic\n");
    } else {
        std::fprintf(stdout, "    FAIL: Short run NON-DETERMINISTIC\n");
    }

    return hash1 ^ hash2; // 0 if deterministic
}
