// =============================================================================
// Sprint 27 Tests — Application Lifecycle & Configuration
// =============================================================================
//
// Tests for the APC Sprint 27 header (apc_application.h):
//   1.  ApplicationConfig defaults (window 1920x1080, debug enabled)
//   2.  ApplicationState enum values (SHUTDOWN=0, INITIALIZING=1, RUNNING=2, etc.)
//   3.  Application defaults (state=SHUTDOWN)
//   4.  Application.init() changes state to RUNNING
//   5.  Application.init() initializes subsystems
//   6.  Application.shutdown() changes state to SHUTDOWN
//   7.  Application soccer_defaults() preset
//   8.  Application basketball_defaults() preset
//   9.  ApplicationConfig field sizes (title, asset_path lengths within bounds)
//
// Pattern: int main() + assert(), no test framework.
// =============================================================================

#include "apc_app/apc_application.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_math_common.h"
#include <cassert>
#include <cstdio>
#include <cmath>
#include <cstring>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static constexpr float EPS = 1e-4f;

static bool approx_eq(float a, float b, float eps = EPS) {
    return std::abs(a - b) < eps;
}

// =============================================================================
// TEST 1: ApplicationConfig defaults
// =============================================================================
static int test_application_config_defaults() {
    std::printf("  [Test 1] ApplicationConfig defaults (window 1920x1080, debug enabled)...\n");

    apc::ApplicationConfig cfg;

    assert(cfg.window_width == 1920u && "window_width = 1920");
    assert(cfg.window_height == 1080u && "window_height = 1080");
    assert(approx_eq(cfg.render_scale, 1.0f) && "render_scale = 1.0");
    assert(cfg.fullscreen == 0 && "fullscreen = 0");
    assert(cfg.vsync == 1 && "vsync = 1");
    assert(cfg.anti_aliasing_samples == 4 && "anti_aliasing_samples = 4");
    assert(approx_eq(cfg.target_fps, 60.0f) && "target_fps = 60");
    assert(cfg.enable_debug_draw == 1 && "enable_debug_draw = 1");
    assert(cfg.enable_ai_debug == 1 && "enable_ai_debug = 1");
    assert(cfg.enable_physics_debug == 1 && "enable_physics_debug = 1");
    assert(cfg.log_level == 3 && "log_level = 3 (info)");

    std::printf("    [PASS] ApplicationConfig defaults verified\n");
    return 0;
}

// =============================================================================
// TEST 2: ApplicationState enum values
// =============================================================================
static int test_application_state_enum() {
    std::printf("  [Test 2] ApplicationState enum values...\n");

    using AS = apc::ApplicationState;

    assert(static_cast<uint8_t>(AS::SHUTDOWN)       == 0);
    assert(static_cast<uint8_t>(AS::INITIALIZING)   == 1);
    assert(static_cast<uint8_t>(AS::RUNNING)        == 2);
    assert(static_cast<uint8_t>(AS::PAUSED)          == 3);
    assert(static_cast<uint8_t>(AS::MATCH_LOADING)   == 4);
    assert(static_cast<uint8_t>(AS::MATCH_PLAYING)   == 5);
    assert(static_cast<uint8_t>(AS::MATCH_ENDED)     == 6);

    std::printf("    [PASS] ApplicationState enum values verified\n");
    return 0;
}

// =============================================================================
// TEST 3: Application defaults
// =============================================================================
static int test_application_defaults() {
    std::printf("  [Test 3] Application defaults (state=SHUTDOWN)...\n");

    apc::Application app;

    assert(app.state == apc::ApplicationState::SHUTDOWN && "state = SHUTDOWN");

    // Game loop should be at defaults
    assert(app.game_loop.state == apc::GameState::UNINITIALIZED && "game_loop.state = UNINITIALIZED");

    // Scene should not be loaded
    assert(app.scene.is_loaded == 0 && "scene.is_loaded = 0");

    // AI debug should have zero counts
    assert(app.ai_debug.utility_count == 0u && "ai_debug.utility_count = 0");
    assert(app.ai_debug.steering_count == 0u && "ai_debug.steering_count = 0");
    assert(app.ai_debug.formation_count == 0u && "ai_debug.formation_count = 0");

    // Config should be at defaults
    assert(app.config.window_width == 1920u && "config.window_width = 1920");
    assert(app.config.window_height == 1080u && "config.window_height = 1080");

    std::printf("    [PASS] Application defaults verified\n");
    return 0;
}

// =============================================================================
// TEST 4: Application.init() changes state to RUNNING
// =============================================================================
static int test_application_init_state() {
    std::printf("  [Test 4] Application.init() changes state to RUNNING...\n");

    apc::Application app;
    assert(app.state == apc::ApplicationState::SHUTDOWN);

    apc::ApplicationConfig cfg;
    uint8_t result = app.init(cfg);

    assert(result == 1 && "init returns 1 (success)");
    assert(app.state == apc::ApplicationState::RUNNING && "state = RUNNING after init");

    // Calling init again should work fine (re-initialize)
    result = app.init(cfg);
    assert(result == 1 && "re-init returns 1");
    assert(app.state == apc::ApplicationState::RUNNING && "state = RUNNING after re-init");

    std::printf("    [PASS] Application.init() state transition verified\n");
    return 0;
}

// =============================================================================
// TEST 5: Application.init() initializes subsystems
// =============================================================================
static int test_application_init_subsystems() {
    std::printf("  [Test 5] Application.init() initializes subsystems...\n");

    apc::Application app;

    apc::ApplicationConfig cfg;
    cfg.enable_ai_debug = 1;
    cfg.enable_debug_draw = 1;

    app.init(cfg);

    // Game loop should be initialized (WARMUP state)
    assert(app.game_loop.state == apc::GameState::WARMUP && "game_loop initialized");
    assert(app.game_loop.time.current_time == 0.0 && "game_loop time reset");
    assert(app.game_loop.time.frame_count == 0u && "game_loop frame_count = 0");

    // Config should be stored
    assert(app.config.enable_ai_debug == 1 && "config stored: enable_ai_debug = 1");
    assert(app.config.enable_debug_draw == 1 && "config stored: enable_debug_draw = 1");

    // AI debug should be fresh
    assert(app.ai_debug.utility_count == 0u && "ai_debug cleared");
    assert(app.ai_debug.config.enabled_layers == 0u && "ai_debug layers cleared");

    // Debug draw list should be cleared
    // (Cannot test internal state directly, but we know clear() was called)

    // begin_frame should work without crash
    app.begin_frame(0.016);
    app.begin_frame(0.032);

    std::printf("    [PASS] Application.init() subsystems verified\n");
    return 0;
}

// =============================================================================
// TEST 6: Application.shutdown() changes state to SHUTDOWN
// =============================================================================
static int test_application_shutdown() {
    std::printf("  [Test 6] Application.shutdown() changes state to SHUTDOWN...\n");

    apc::Application app;
    apc::ApplicationConfig cfg;
    app.init(cfg);

    assert(app.state == apc::ApplicationState::RUNNING && "running before shutdown");

    app.shutdown();

    assert(app.state == apc::ApplicationState::SHUTDOWN && "state = SHUTDOWN after shutdown");

    // Game loop should be reset
    assert(app.game_loop.state == apc::GameState::UNINITIALIZED && "game_loop reset after shutdown");

    // Scene should be unloaded
    assert(app.scene.is_loaded == 0 && "scene unloaded after shutdown");

    // AI debug should be reset
    assert(app.ai_debug.utility_count == 0u && "ai_debug reset");
    assert(app.ai_debug.steering_count == 0u && "ai_debug steering reset");
    assert(app.ai_debug.formation_count == 0u && "ai_debug formation reset");

    // Shutdown from any state should work
    app.state = apc::ApplicationState::MATCH_PLAYING;
    app.shutdown();
    assert(app.state == apc::ApplicationState::SHUTDOWN && "shutdown from MATCH_PLAYING works");

    std::printf("    [PASS] Application.shutdown() verified\n");
    return 0;
}

// =============================================================================
// TEST 7: Application soccer_defaults() preset
// =============================================================================
static int test_soccer_defaults() {
    std::printf("  [Test 7] Application soccer_defaults() preset...\n");

    apc::ApplicationConfig cfg = apc::Application::soccer_defaults();

    // Window settings
    assert(cfg.window_width == 1920u && "soccer: window_width = 1920");
    assert(cfg.window_height == 1080u && "soccer: window_height = 1080");
    assert(approx_eq(cfg.render_scale, 1.0f) && "soccer: render_scale = 1.0");
    assert(cfg.fullscreen == 0 && "soccer: fullscreen = 0");
    assert(cfg.vsync == 1 && "soccer: vsync = 1");

    // Debug settings
    assert(cfg.enable_debug_draw == 1 && "soccer: debug_draw = 1");
    assert(cfg.enable_ai_debug == 1 && "soccer: ai_debug = 1");
    assert(cfg.enable_physics_debug == 1 && "soccer: physics_debug = 1");
    assert(cfg.log_level == 3 && "soccer: log_level = 3");

    // Window title should contain "Soccer"
    bool has_soccer = false;
    for (uint32_t i = 0u; i < apc::MAX_NAME_LENGTH; ++i) {
        if (cfg.window_title[i] == 'S' || cfg.window_title[i] == 's') {
            // Check if followed by 'occer' or 'occer'
            if (i + 5 < apc::MAX_NAME_LENGTH &&
                cfg.window_title[i+1] == 'o' &&
                cfg.window_title[i+2] == 'c' &&
                cfg.window_title[i+3] == 'c' &&
                cfg.window_title[i+4] == 'e' &&
                cfg.window_title[i+5] == 'r') {
                has_soccer = true;
                break;
            }
        }
    }
    assert(has_soccer && "soccer: title contains 'Soccer'");

    std::printf("    [PASS] soccer_defaults() preset verified\n");
    return 0;
}

// =============================================================================
// TEST 8: Application basketball_defaults() preset
// =============================================================================
static int test_basketball_defaults() {
    std::printf("  [Test 8] Application basketball_defaults() preset...\n");

    apc::ApplicationConfig cfg = apc::Application::basketball_defaults();

    // Window settings
    assert(cfg.window_width == 1920u && "basketball: window_width = 1920");
    assert(cfg.window_height == 1080u && "basketball: window_height = 1080");
    assert(approx_eq(cfg.render_scale, 1.0f) && "basketball: render_scale = 1.0");
    assert(cfg.fullscreen == 0 && "basketball: fullscreen = 0");
    assert(cfg.vsync == 1 && "basketball: vsync = 1");
    assert(cfg.anti_aliasing_samples == 4 && "basketball: aa = 4");
    assert(approx_eq(cfg.target_fps, 60.0f) && "basketball: fps = 60");

    // Debug settings
    assert(cfg.enable_debug_draw == 1 && "basketball: debug_draw = 1");
    assert(cfg.enable_ai_debug == 1 && "basketball: ai_debug = 1");
    assert(cfg.enable_physics_debug == 1 && "basketball: physics_debug = 1");

    // Window title should contain "Basketball"
    bool has_basketball = false;
    for (uint32_t i = 0u; i < apc::MAX_NAME_LENGTH; ++i) {
        if (cfg.window_title[i] == 'B' &&
            i + 9 < apc::MAX_NAME_LENGTH &&
            cfg.window_title[i+1] == 'a' &&
            cfg.window_title[i+2] == 's' &&
            cfg.window_title[i+3] == 'k' &&
            cfg.window_title[i+4] == 'e' &&
            cfg.window_title[i+5] == 't' &&
            cfg.window_title[i+6] == 'b' &&
            cfg.window_title[i+7] == 'a' &&
            cfg.window_title[i+8] == 'l' &&
            cfg.window_title[i+9] == 'l') {
            has_basketball = true;
            break;
        }
    }
    assert(has_basketball && "basketball: title contains 'Basketball'");

    std::printf("    [PASS] basketball_defaults() preset verified\n");
    return 0;
}

// =============================================================================
// TEST 9: ApplicationConfig field sizes
// =============================================================================
static int test_application_config_field_sizes() {
    std::printf("  [Test 9] ApplicationConfig field sizes (title, asset_path lengths within bounds)...\n");

    apc::ApplicationConfig cfg;

    // window_title is char[MAX_NAME_LENGTH] = char[64]
    // Fill it up to MAX_NAME_LENGTH - 1 characters + null terminator
    const char* long_title = "This is a very long window title that is exactly the right length test";
    uint32_t copy_len = 0;
    for (uint32_t i = 0u; long_title[i] && i < apc::MAX_NAME_LENGTH - 1u; ++i) {
        cfg.window_title[i] = long_title[i];
        ++copy_len;
    }
    cfg.window_title[copy_len] = '\0';

    assert(std::strlen(cfg.window_title) < apc::MAX_NAME_LENGTH && "title fits in MAX_NAME_LENGTH");
    assert(copy_len <= apc::MAX_NAME_LENGTH - 1u && "copy_len <= MAX_NAME_LENGTH - 1");

    // Verify soccer_defaults title fits within bounds
    apc::ApplicationConfig soccer = apc::Application::soccer_defaults();
    assert(std::strlen(soccer.window_title) < apc::MAX_NAME_LENGTH && "soccer title fits");

    // Verify basketball_defaults title fits within bounds
    apc::ApplicationConfig basketball = apc::Application::basketball_defaults();
    assert(std::strlen(basketball.window_title) < apc::MAX_NAME_LENGTH && "basketball title fits");

    // asset_path is char[MAX_PATH_LENGTH] = char[256]
    const char* long_path = "/this/is/a/very/long/asset/path/that/should/still/fit/within/the/buffer";
    uint32_t path_len = 0;
    for (uint32_t i = 0u; long_path[i] && i < apc::MAX_PATH_LENGTH - 1u; ++i) {
        cfg.asset_path[i] = long_path[i];
        ++path_len;
    }
    cfg.asset_path[path_len] = '\0';

    assert(std::strlen(cfg.asset_path) < apc::MAX_PATH_LENGTH && "asset_path fits in MAX_PATH_LENGTH");
    assert(path_len <= apc::MAX_PATH_LENGTH - 1u && "path_len <= MAX_PATH_LENGTH - 1");

    // Verify constants
    assert(apc::MAX_NAME_LENGTH == 64u && "MAX_NAME_LENGTH = 64");
    assert(apc::MAX_PATH_LENGTH == 256u && "MAX_PATH_LENGTH = 256");

    std::printf("    [PASS] ApplicationConfig field sizes verified\n");
    return 0;
}

// =============================================================================
// main
// =============================================================================
int main() {
    std::printf("=== Sprint 27: Application Lifecycle & Configuration ===\n\n");

    int result = 0;
    result |= test_application_config_defaults();
    result |= test_application_state_enum();
    result |= test_application_defaults();
    result |= test_application_init_state();
    result |= test_application_init_subsystems();
    result |= test_application_shutdown();
    result |= test_soccer_defaults();
    result |= test_basketball_defaults();
    result |= test_application_config_field_sizes();

    int total = 9;
    int passed = total - result;
    std::printf("\n=== Sprint 27: %d tests passed, %d failed ===\n", passed, result);
    return result;
}
