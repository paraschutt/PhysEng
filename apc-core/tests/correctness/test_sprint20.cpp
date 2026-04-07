// =============================================================================
// Sprint 20 Tests — Camera Systems, VFX Integration & Multi-Viewport
// =============================================================================
//
// Tests for the APC Sprint 20 headers:
//   1.  CameraShotType enum completeness
//   2.  BroadcastCameraConfig defaults
//   3.  BroadcastCameraSystem register & active camera
//   4.  BroadcastCameraSystem sport event → shot switch
//   5.  BroadcastCameraSystem trigger_cut by type
//   6.  BroadcastCameraSystem auto-switch after inactivity
//   7.  BroadcastCameraSystem cut delay enforcement
//   8.  ReplayCameraConfig defaults
//   9.  ReplayCamera seek, play, pause
//   10. ReplayCamera loop behavior
//   11. ReplayCamera orbit camera transform
//   12. CameraShakeConfig defaults
//   13. CameraShake trigger and decay
//   14. CameraShake offset and rotation
//   15. PhysicsToRenderSync double-buffer swap
//   16. ScreenFlashVFX trigger and decay
//   17. SoundTriggerEvent defaults
//   18. VibrationEvent patterns (PULSE, RAMP, CONSTANT)
//   19. HitStopIntegration consume and tick
//   20. TimeScaleIntegration consume and recovery
//   21. VFXBridge dispatch and update
//   22. VFXBridge effective time scale (hit-stop + time-scale)
//   23. ViewportConfig defaults and layer presets
//   24. MultiViewport FULL layout
//   25. MultiViewport HORIZONTAL layout
//   26. MultiViewport VERTICAL layout
//   27. MultiViewport QUAD layout
//   28. MultiViewport per-viewport camera and layer assignment
//   29. MultiViewport viewport border drawing
//   30. RenderPerformanceOverlay FPS tracking
//   31. RenderPerformanceOverlay physics stats
//   32. RenderPerformanceOverlay render stats
//   33. Integration: broadcast camera + shake + VFX + viewport + overlay
//
// Pattern: int main() + assert(), no test framework.
// =============================================================================

#include "apc_render/apc_camera_system.h"
#include "apc_render/apc_vfx_bridge.h"
#include "apc_render/apc_multi_viewport.h"
#include "apc_render/apc_debug_draw.h"
#include "apc_style/apc_game_hooks.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include "apc_math/apc_math_common.h"
#include <cassert>
#include <cstdio>
#include <cmath>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static constexpr float EPS = 1e-4f;

static bool approx_eq(float a, float b, float eps = EPS) {
    return std::abs(a - b) < eps;
}

// =============================================================================
// TEST 1: CameraShotType enum completeness
// =============================================================================
static int test_camera_shot_type_enum() {
    std::printf("  [Test 1] CameraShotType enum completeness...\n");

    // Verify all expected values exist
    assert(static_cast<uint8_t>(apc::CameraShotType::FREE) == 0);
    assert(static_cast<uint8_t>(apc::CameraShotType::FOLLOW_TARGET) == 1);
    assert(static_cast<uint8_t>(apc::CameraShotType::BROADCAST_MAIN) == 2);
    assert(static_cast<uint8_t>(apc::CameraShotType::BROADCAST_WIDE) == 3);
    assert(static_cast<uint8_t>(apc::CameraShotType::BROADCAST_TACTICAL) == 4);
    assert(static_cast<uint8_t>(apc::CameraShotType::REPLAY_ORBIT) == 5);
    assert(static_cast<uint8_t>(apc::CameraShotType::REPLAY_FIXED) == 6);
    assert(static_cast<uint8_t>(apc::CameraShotType::SIDELINE) == 7);
    assert(static_cast<uint8_t>(apc::CameraShotType::ENDZONE) == 8);
    assert(static_cast<uint8_t>(apc::CameraShotType::SKYCAM) == 9);
    assert(static_cast<uint8_t>(apc::CameraShotType::FIRST_PERSON) == 10);

    // SportEventType
    assert(static_cast<uint8_t>(apc::SportEventType::NONE) == 0);
    assert(static_cast<uint8_t>(apc::SportEventType::GOAL_SCORED) == 1);
    assert(static_cast<uint8_t>(apc::SportEventType::TACKLE) == 2);
    assert(static_cast<uint8_t>(apc::SportEventType::FREE_KICK) == 3);
    assert(static_cast<uint8_t>(apc::SportEventType::TIMEOUT) == 4);
    assert(static_cast<uint8_t>(apc::SportEventType::KICKOFF) == 5);
    assert(static_cast<uint8_t>(apc::SportEventType::CORNER_KICK) == 6);
    assert(static_cast<uint8_t>(apc::SportEventType::PENALTY_KICK) == 7);
    assert(static_cast<uint8_t>(apc::SportEventType::SCORE_ANY) == 8);
    assert(static_cast<uint8_t>(apc::SportEventType::BALL_CHANGE) == 9);
    assert(static_cast<uint8_t>(apc::SportEventType::WHISTLE) == 10);

    std::printf("    [PASS] CameraShotType and SportEventType enums verified\n");
    return 0;
}

// =============================================================================
// TEST 2: BroadcastCameraConfig defaults
// =============================================================================
static int test_broadcast_camera_config() {
    std::printf("  [Test 2] BroadcastCameraConfig defaults...\n");

    apc::BroadcastCameraConfig cfg;

    assert(approx_eq(cfg.cut_delay, 2.0f) && "cut_delay = 2.0");
    assert(approx_eq(cfg.smooth_transition_speed, 3.0f) && "transition_speed = 3.0");
    assert(approx_eq(cfg.auto_switch_threshold, 5.0f) && "auto_switch_threshold = 5.0");

    // Check some priority weights
    assert(approx_eq(cfg.shot_priority_weights[0], 1.0f) && "FREE priority = 1.0");
    assert(approx_eq(cfg.shot_priority_weights[2], 3.0f) && "BROADCAST_MAIN priority = 3.0");
    assert(approx_eq(cfg.shot_priority_weights[3], 2.5f) && "BROADCAST_WIDE priority = 2.5");

    // Check some zoom preferences
    assert(approx_eq(cfg.zoom_preferences[0], 0.5f) && "FREE zoom = 0.5");
    assert(approx_eq(cfg.zoom_preferences[9], 0.2f) && "SKYCAM zoom = 0.2");
    assert(approx_eq(cfg.zoom_preferences[10], 0.8f) && "FIRST_PERSON zoom = 0.8");

    std::printf("    [PASS] BroadcastCameraConfig defaults verified\n");
    return 0;
}

// =============================================================================
// TEST 3: BroadcastCameraSystem register & active camera
// =============================================================================
static int test_broadcast_camera_register() {
    std::printf("  [Test 3] BroadcastCameraSystem register & active camera...\n");

    apc::BroadcastCameraSystem bcs;

    assert(bcs.camera_count == 0u && "Initial camera_count = 0");

    apc::CameraTransform main_cam;
    main_cam.eye = apc::Vec3(0.0f, 30.0f, -50.0f);
    main_cam.target = apc::Vec3(0.0f, 0.0f, 0.0f);

    uint32_t idx = bcs.register_camera(0u, apc::CameraShotType::BROADCAST_MAIN, main_cam, 3.0f);
    assert(idx == 0u && "First camera index = 0");
    assert(bcs.camera_count == 1u && "camera_count = 1");
    assert(bcs.active_camera_id == 0u && "First camera auto-active");
    assert(bcs.get_current_shot_type() == apc::CameraShotType::BROADCAST_MAIN);

    // Register second camera
    apc::CameraTransform wide_cam;
    wide_cam.eye = apc::Vec3(0.0f, 60.0f, -80.0f);
    wide_cam.target = apc::Vec3(0.0f, 0.0f, 0.0f);
    uint32_t idx2 = bcs.register_camera(1u, apc::CameraShotType::BROADCAST_WIDE, wide_cam, 2.5f);
    assert(idx2 == 1u && "Second camera index = 1");
    assert(bcs.camera_count == 2u);

    // Active camera should still be the first
    const apc::CameraTransform& active = bcs.get_active_camera();
    assert(approx_eq(active.eye.y, 30.0f) && "Active camera eye y = 30");

    std::printf("    [PASS] BroadcastCameraSystem register verified\n");
    return 0;
}

// =============================================================================
// TEST 4: BroadcastCameraSystem sport event → shot switch
// =============================================================================
static int test_broadcast_camera_sport_event() {
    std::printf("  [Test 4] BroadcastCameraSystem sport event → shot switch...\n");

    apc::BroadcastCameraSystem bcs;

    apc::CameraTransform main_cam, wide_cam, follow_cam, tactical_cam, skycam_cam;
    main_cam.eye = apc::Vec3(0.0f, 30.0f, -50.0f);
    wide_cam.eye = apc::Vec3(0.0f, 60.0f, -80.0f);
    follow_cam.eye = apc::Vec3(5.0f, 5.0f, -10.0f);
    tactical_cam.eye = apc::Vec3(20.0f, 15.0f, -30.0f);
    skycam_cam.eye = apc::Vec3(0.0f, 80.0f, 0.0f);

    bcs.register_camera(0u, apc::CameraShotType::BROADCAST_MAIN, main_cam, 3.0f);
    bcs.register_camera(1u, apc::CameraShotType::BROADCAST_WIDE, wide_cam, 2.5f);
    bcs.register_camera(2u, apc::CameraShotType::FOLLOW_TARGET, follow_cam, 2.0f);
    bcs.register_camera(3u, apc::CameraShotType::BROADCAST_TACTICAL, tactical_cam, 2.0f);
    bcs.register_camera(4u, apc::CameraShotType::SKYCAM, skycam_cam, 1.5f);

    // Simulate time passing to allow cut
    bcs.time_since_cut = 10.0f; // Force enough time for cut

    // Goal scored → WIDE
    bcs.update(apc::SportEventType::GOAL_SCORED, 0.0f);
    assert(bcs.get_current_shot_type() == apc::CameraShotType::BROADCAST_WIDE && "Goal → WIDE");

    // Force time again
    bcs.time_since_cut = 10.0f;

    // Tackle → FOLLOW_TARGET
    bcs.update(apc::SportEventType::TACKLE, 0.0f);
    assert(bcs.get_current_shot_type() == apc::CameraShotType::FOLLOW_TARGET && "Tackle → FOLLOW");

    bcs.time_since_cut = 10.0f;

    // Free kick → TACTICAL
    bcs.update(apc::SportEventType::FREE_KICK, 0.0f);
    assert(bcs.get_current_shot_type() == apc::CameraShotType::BROADCAST_TACTICAL && "FreeKick → TACTICAL");

    bcs.time_since_cut = 10.0f;

    // Timeout → WIDE
    bcs.update(apc::SportEventType::TIMEOUT, 0.0f);
    assert(bcs.get_current_shot_type() == apc::CameraShotType::BROADCAST_WIDE && "Timeout → WIDE");

    bcs.time_since_cut = 10.0f;

    // Kickoff → SKYCAM
    bcs.update(apc::SportEventType::KICKOFF, 0.0f);
    assert(bcs.get_current_shot_type() == apc::CameraShotType::SKYCAM && "Kickoff → SKYCAM");

    std::printf("    [PASS] Sport event camera switching verified\n");
    return 0;
}

// =============================================================================
// TEST 5: BroadcastCameraSystem trigger_cut by type
// =============================================================================
static int test_broadcast_camera_trigger_cut() {
    std::printf("  [Test 5] BroadcastCameraSystem trigger_cut by type...\n");

    apc::BroadcastCameraSystem bcs;

    apc::CameraTransform main_cam, wide_cam;
    main_cam.eye = apc::Vec3(0.0f, 30.0f, -50.0f);
    wide_cam.eye = apc::Vec3(0.0f, 60.0f, -80.0f);

    bcs.register_camera(0u, apc::CameraShotType::BROADCAST_MAIN, main_cam, 3.0f);
    bcs.register_camera(1u, apc::CameraShotType::BROADCAST_WIDE, wide_cam, 2.5f);

    // Force time for cut
    bcs.time_since_cut = 10.0f;

    // Manual cut to WIDE
    bool ok = bcs.trigger_cut(apc::CameraShotType::BROADCAST_WIDE);
    assert(ok && "trigger_cut to WIDE succeeds");
    assert(bcs.get_current_shot_type() == apc::CameraShotType::BROADCAST_WIDE);
    assert(bcs.is_transitioning() && "Transition starts");
    assert(bcs.time_since_cut < 0.01f && "Cut timer reset");

    // Cut too soon should fail
    bool fail = bcs.trigger_cut(apc::CameraShotType::BROADCAST_MAIN);
    assert(!fail && "Cut during delay fails");

    // Trigger cut by camera ID
    bcs.time_since_cut = 10.0f;
    bool ok2 = bcs.trigger_cut(0u);
    assert(ok2 && "trigger_cut by ID succeeds");
    assert(bcs.get_current_shot_type() == apc::CameraShotType::BROADCAST_MAIN);

    // Non-existent camera ID fails
    bcs.time_since_cut = 10.0f;
    bool fail2 = bcs.trigger_cut(99u);
    assert(!fail2 && "trigger_cut invalid ID fails");

    std::printf("    [PASS] trigger_cut verified\n");
    return 0;
}

// =============================================================================
// TEST 6: BroadcastCameraSystem auto-switch after inactivity
// =============================================================================
static int test_broadcast_camera_auto_switch() {
    std::printf("  [Test 6] BroadcastCameraSystem auto-switch after inactivity...\n");

    apc::BroadcastCameraSystem bcs;
    bcs.config.auto_switch_threshold = 3.0f;
    bcs.config.cut_delay = 0.5f;

    apc::CameraTransform cam_a, cam_b;
    cam_a.eye = apc::Vec3(0.0f, 30.0f, -50.0f);
    cam_b.eye = apc::Vec3(0.0f, 60.0f, -80.0f);

    bcs.register_camera(0u, apc::CameraShotType::BROADCAST_MAIN, cam_a, 3.0f);
    bcs.register_camera(1u, apc::CameraShotType::BROADCAST_WIDE, cam_b, 2.5f);

    // Start with camera 0 active
    assert(bcs.active_camera_id == 0u && "Camera 0 active");

    // Update for longer than auto_switch_threshold
    bcs.update(apc::SportEventType::NONE, 4.0f);
    assert(bcs.active_camera_id == 1u && "Auto-switched to camera 1");
    assert(bcs.get_current_shot_type() == apc::CameraShotType::BROADCAST_WIDE);

    // Another auto-switch should go back
    bcs.update(apc::SportEventType::NONE, 4.0f);
    assert(bcs.active_camera_id == 0u && "Auto-switched back to camera 0");

    std::printf("    [PASS] Auto-switch after inactivity verified\n");
    return 0;
}

// =============================================================================
// TEST 7: BroadcastCameraSystem cut delay enforcement
// =============================================================================
static int test_broadcast_camera_cut_delay() {
    std::printf("  [Test 7] BroadcastCameraSystem cut delay enforcement...\n");

    apc::BroadcastCameraSystem bcs;
    bcs.config.cut_delay = 2.0f;

    apc::CameraTransform cam_a, cam_b;
    bcs.register_camera(0u, apc::CameraShotType::BROADCAST_MAIN, cam_a, 3.0f);
    bcs.register_camera(1u, apc::CameraShotType::BROADCAST_WIDE, cam_b, 2.5f);

    // Update 0.5s — should not auto-switch yet
    bcs.time_since_event = 100.0f;
    bcs.update(apc::SportEventType::NONE, 0.5f);
    assert(bcs.active_camera_id == 0u && "No switch before cut_delay (0.5s < 2.0s)");

    // Update past cut_delay
    bcs.time_since_event = 100.0f;
    bcs.update(apc::SportEventType::NONE, 3.0f);
    assert(bcs.active_camera_id == 1u && "Switch after cut_delay (3.0s > 2.0s)");

    std::printf("    [PASS] Cut delay enforcement verified\n");
    return 0;
}

// =============================================================================
// TEST 8: ReplayCameraConfig defaults
// =============================================================================
static int test_replay_camera_config() {
    std::printf("  [Test 8] ReplayCameraConfig defaults...\n");

    apc::ReplayCameraConfig cfg;

    assert(approx_eq(cfg.playback_speed, 1.0f) && "playback_speed = 1.0");
    assert(approx_eq(cfg.loop_start, 0.0f) && "loop_start = 0.0");
    assert(approx_eq(cfg.loop_end, 10.0f) && "loop_end = 10.0");
    assert(approx_eq(cfg.orbit_distance, 15.0f) && "orbit_distance = 15.0");
    assert(approx_eq(cfg.orbit_yaw, 0.0f) && "orbit_yaw = 0.0");
    assert(approx_eq(cfg.orbit_pitch, 0.4f) && "orbit_pitch = 0.4");
    assert(approx_eq(cfg.seek_speed, 0.1f) && "seek_speed = 0.1");

    std::printf("    [PASS] ReplayCameraConfig defaults verified\n");
    return 0;
}

// =============================================================================
// TEST 9: ReplayCamera seek, play, pause
// =============================================================================
static int test_replay_camera_playback() {
    std::printf("  [Test 9] ReplayCamera seek, play, pause...\n");

    apc::ReplayCamera rc;
    rc.config.loop_start = 0.0f;
    rc.config.loop_end = 10.0f;

    // Initial state
    assert(!rc.is_playing && "Not playing initially");
    assert(approx_eq(rc.get_current_time(), 0.0f) && "Starts at 0");

    // Seek
    rc.seek_to(5.0f);
    assert(approx_eq(rc.get_current_time(), 5.0f) && "Seeked to 5.0");

    // Play
    rc.play();
    assert(rc.is_playing && "Playing after play()");

    // Update 2 seconds (normal speed)
    rc.update(2.0f);
    assert(approx_eq(rc.get_current_time(), 7.0f) && "At 7.0 after 2s");

    // Pause
    rc.pause();
    assert(!rc.is_playing && "Paused");

    // Update should not advance when paused
    rc.update(2.0f);
    assert(approx_eq(rc.get_current_time(), 7.0f) && "Still at 7.0 (paused)");

    // Half speed
    rc.config.playback_speed = 0.5f;
    rc.play();
    rc.update(2.0f);
    assert(approx_eq(rc.get_current_time(), 8.0f) && "Half speed: 7.0 + 1.0 = 8.0");

    // Double speed
    rc.config.playback_speed = 2.0f;
    rc.update(1.0f);
    assert(approx_eq(rc.get_current_time(), 10.0f) && "Double speed: 8.0 + 2.0 = 10.0");
    assert(!rc.is_playing && "Stopped at end");
    assert(rc.is_at_end() && "At end");

    std::printf("    [PASS] ReplayCamera playback verified\n");
    return 0;
}

// =============================================================================
// TEST 10: ReplayCamera loop behavior
// =============================================================================
static int test_replay_camera_loop() {
    std::printf("  [Test 10] ReplayCamera loop behavior...\n");

    apc::ReplayCamera rc;
    rc.config.loop_start = 2.0f;
    rc.config.loop_end = 6.0f;
    rc.set_loop(true);

    rc.seek_to(2.0f);
    rc.play();

    // Play past end — should loop back
    rc.update(6.0f); // 2.0 + 6.0 = 8.0, but loop_end = 6.0
    // After looping: should be 2.0 + (8.0 - 2.0) - (6.0 - 2.0) = 2.0 + 4.0 = 6.0
    // But loop clamp: 2.0 + (6.0 - 4.0) = 4.0
    assert(rc.is_playing && "Still playing (looping)");
    assert(rc.get_current_time() >= rc.config.loop_start && "Time >= loop_start");
    assert(rc.get_current_time() <= rc.config.loop_end && "Time <= loop_end");

    std::printf("    [PASS] ReplayCamera loop verified\n");
    return 0;
}

// =============================================================================
// TEST 11: ReplayCamera orbit camera transform
// =============================================================================
static int test_replay_camera_orbit() {
    std::printf("  [Test 11] ReplayCamera orbit camera transform...\n");

    apc::ReplayCamera rc;
    rc.set_orbit(apc::Vec3(0.0f, 0.0f, 0.0f), 10.0f, 0.0f, 0.0f);

    apc::CameraTransform ct = rc.get_camera_at_time();
    // pitch=0, yaw=0: eye = center + (0, 0, distance) = (0, 0, 10)
    assert(approx_eq(ct.eye.x, 0.0f) && "Orbit eye x = 0");
    assert(approx_eq(ct.eye.y, 0.0f) && "Orbit eye y = 0");
    assert(approx_eq(ct.eye.z, 10.0f) && "Orbit eye z = 10 (distance)");
    assert(approx_eq(ct.target.x, 0.0f) && "Orbit target = center");

    // pitch = PI/2: eye directly above
    rc.config.orbit_pitch = 1.5707963f; // ~PI/2
    apc::CameraTransform ct2 = rc.get_camera_at_time();
    assert(ct2.eye.y > 9.0f && "Pitch=PI/2: eye above center");
    assert(approx_eq(ct2.eye.z, 0.0f, 0.01f) && "Pitch=PI/2: eye z ≈ 0");

    // yaw = PI/2: eye on +X axis
    rc.config.orbit_pitch = 0.0f;
    rc.config.orbit_yaw = 1.5707963f; // ~PI/2
    apc::CameraTransform ct3 = rc.get_camera_at_time();
    assert(ct3.eye.x > 9.0f && "Yaw=PI/2: eye on +X");
    assert(approx_eq(ct3.eye.z, 0.0f, 0.01f) && "Yaw=PI/2: eye z ≈ 0");

    std::printf("    [PASS] ReplayCamera orbit transform verified\n");
    return 0;
}

// =============================================================================
// TEST 12: CameraShakeConfig defaults
// =============================================================================
static int test_camera_shake_config() {
    std::printf("  [Test 12] CameraShakeConfig defaults...\n");

    apc::CameraShakeConfig cfg;

    assert(approx_eq(cfg.intensity_decay_rate, 3.0f) && "decay_rate = 3.0");
    assert(approx_eq(cfg.frequency, 15.0f) && "frequency = 15.0");
    assert(cfg.noise_octaves == 2u && "noise_octaves = 2");
    assert(approx_eq(cfg.directional_bias_x, 0.0f) && "bias_x = 0");
    assert(approx_eq(cfg.max_offset, 2.0f) && "max_offset = 2.0");
    assert(approx_eq(cfg.max_rotation_offset, 0.05f) && "max_rotation = 0.05");

    std::printf("    [PASS] CameraShakeConfig defaults verified\n");
    return 0;
}

// =============================================================================
// TEST 13: CameraShake trigger and decay
// =============================================================================
static int test_camera_shake_decay() {
    std::printf("  [Test 13] CameraShake trigger and decay...\n");

    apc::CameraShake shake;

    // Not active initially
    assert(!shake.is_active() && "Not active initially");
    assert(approx_eq(shake.get_current_intensity(), 0.0f) && "Intensity = 0");

    // Trigger
    shake.trigger_shake(apc::Vec3(0, 0, 0), 0.8f, 1.0f);
    assert(shake.is_active() && "Active after trigger");
    assert(shake.get_current_intensity() > 0.5f && "Intensity > 0.5");

    // Update — intensity should decrease
    float intensity_before = shake.get_current_intensity();
    shake.update(0.5f);
    float intensity_after = shake.get_current_intensity();
    assert(intensity_after < intensity_before && "Intensity decays over time");

    // Update a lot — should reach near zero
    for (int i = 0; i < 50; ++i) {
        shake.update(0.1f);
    }
    assert(!shake.is_active() && "Deactivates after decay");
    assert(approx_eq(shake.get_current_intensity(), 0.0f) && "Intensity = 0 after full decay");

    // Reset
    shake.trigger_shake(apc::Vec3(0, 0, 0), 0.5f, 1.0f);
    shake.reset();
    assert(!shake.is_active() && "Reset clears shake");

    std::printf("    [PASS] CameraShake decay verified\n");
    return 0;
}

// =============================================================================
// TEST 14: CameraShake offset and rotation
// =============================================================================
static int test_camera_shake_offset() {
    std::printf("  [Test 14] CameraShake offset and rotation...\n");

    apc::CameraShake shake;

    // No shake → zero offset
    apc::Vec3 offset = shake.get_offset();
    assert(approx_eq(offset.x, 0.0f) && "No shake: offset x = 0");
    assert(approx_eq(offset.y, 0.0f) && "No shake: offset y = 0");
    assert(approx_eq(offset.z, 0.0f) && "No shake: offset z = 0");

    apc::Vec3 rot = shake.get_rotation_offset();
    assert(approx_eq(rot.x, 0.0f) && "No shake: rot x = 0");
    assert(approx_eq(rot.y, 0.0f) && "No shake: rot y = 0");
    assert(approx_eq(rot.z, 0.0f) && "No shake: rot z = 0");

    // With shake → non-zero offset (deterministic)
    shake.trigger_shake(apc::Vec3(0, 0, 0), 1.0f, 2.0f);
    apc::Vec3 offset_active = shake.get_offset();
    // With max_offset=2.0 and intensity=1.0, the offset should be non-trivial
    float offset_mag = apc::Vec3::length(offset_active);
    assert(offset_mag > 0.01f && "Active shake: non-zero offset");

    apc::Vec3 rot_active = shake.get_rotation_offset();
    float rot_mag = apc::Vec3::length(rot_active);
    assert(rot_mag > 0.0001f && "Active shake: non-zero rotation");

    // Directional bias
    apc::CameraShake biased;
    biased.config.directional_bias_x = 1.0f;
    biased.trigger_shake(apc::Vec3(0, 0, 0), 1.0f, 2.0f);
    apc::Vec3 biased_offset = biased.get_offset();
    // With full X bias, X component should be large
    assert(std::abs(biased_offset.x) > 0.5f && "Biased shake: X offset is large");

    std::printf("    [PASS] CameraShake offset and rotation verified\n");
    return 0;
}

// =============================================================================
// TEST 15: PhysicsToRenderSync double-buffer swap
// =============================================================================
static int test_physics_to_render_sync() {
    std::printf("  [Test 15] PhysicsToRenderSync double-buffer swap...\n");

    apc::PhysicsToRenderSync sync;

    // Initial state
    assert(sync.get_body_count() == 0u && "Initial count = 0");
    assert(sync.get_frame_number() == 0u && "Frame = 0");
    assert(sync.get_last_sync() == apc::PhysicsToRenderSync::SyncPoint::POST_PHYSICS);

    // Sync from physics
    apc::Vec3 positions[3] = {
        apc::Vec3(1.0f, 2.0f, 3.0f),
        apc::Vec3(4.0f, 5.0f, 6.0f),
        apc::Vec3(7.0f, 8.0f, 9.0f)
    };
    apc::Quat orientations[3] = {
        apc::Quat::identity(),
        apc::Quat::identity(),
        apc::Quat::identity()
    };

    uint32_t copied = sync.sync_from_physics(positions, orientations, 3,
        apc::PhysicsToRenderSync::SyncPoint::POST_PHYSICS);
    assert(copied == 3u && "Copied 3 bodies");
    assert(sync.get_body_count() == 3u);
    assert(sync.get_last_sync() == apc::PhysicsToRenderSync::SyncPoint::POST_PHYSICS);

    // Before swap, front buffer should be default (identity)
    const auto* front_before = sync.get_transforms();
    assert(approx_eq(front_before[0].position.x, 0.0f) && "Front buffer: default before swap");

    // Swap
    sync.swap_buffers();
    assert(sync.get_frame_number() == 1u && "Frame = 1 after swap");

    // After swap, front buffer should have physics data
    const auto* front_after = sync.get_transforms();
    assert(approx_eq(front_after[0].position.x, 1.0f) && "Front buffer: pos x = 1 after swap");
    assert(approx_eq(front_after[1].position.y, 5.0f) && "Front buffer: pos y = 5 after swap");
    assert(approx_eq(front_after[2].position.z, 9.0f) && "Front buffer: pos z = 9 after swap");

    // Reset
    sync.reset();
    assert(sync.get_body_count() == 0u && "Reset: count = 0");
    assert(sync.get_frame_number() == 0u && "Reset: frame = 0");

    std::printf("    [PASS] PhysicsToRenderSync double-buffer verified\n");
    return 0;
}

// =============================================================================
// TEST 16: ScreenFlashVFX trigger and decay
// =============================================================================
static int test_screen_flash() {
    std::printf("  [Test 16] ScreenFlashVFX trigger and decay...\n");

    apc::ScreenFlashVFX flash;

    assert(!flash.active() && "Not active initially");
    assert(approx_eq(flash.get_current_intensity(), 0.0f) && "Intensity = 0");

    // Trigger white flash
    flash.trigger(apc::RenderColor::WHITE(), 0.8f);
    assert(flash.active() && "Active after trigger");
    assert(approx_eq(flash.get_current_intensity(), 0.8f) && "Intensity = 0.8");

    apc::RenderColor col = flash.get_current_color();
    assert(approx_eq(col.r, 0.8f) && "Color r = intensity * 1.0 = 0.8");
    assert(approx_eq(col.g, 0.8f) && "Color g = 0.8");

    // Update — decay
    flash.update(0.2f);
    float after = flash.get_current_intensity();
    assert(after < 0.8f && "Intensity decays");

    // Decay to zero
    for (int i = 0; i < 50; ++i) {
        flash.update(0.1f);
    }
    assert(!flash.active() && "Deactivates after decay");
    assert(approx_eq(flash.get_current_intensity(), 0.0f) && "Intensity = 0");

    // Colored flash
    flash.trigger(apc::RenderColor::RED(), 1.0f);
    apc::RenderColor red_col = flash.get_current_color();
    assert(approx_eq(red_col.r, 1.0f) && "Red flash r = 1.0");
    assert(approx_eq(red_col.g, 0.0f) && "Red flash g = 0.0");
    assert(approx_eq(red_col.b, 0.0f) && "Red flash b = 0.0");

    std::printf("    [PASS] ScreenFlashVFX verified\n");
    return 0;
}

// =============================================================================
// TEST 17: SoundTriggerEvent defaults
// =============================================================================
static int test_sound_trigger_event() {
    std::printf("  [Test 17] SoundTriggerEvent defaults...\n");

    apc::SoundTriggerEvent snd;

    assert(snd.sound_id == 0u && "sound_id = 0");
    assert(approx_eq(snd.position.x, 0.0f) && "position x = 0");
    assert(approx_eq(snd.volume, 1.0f) && "volume = 1.0");
    assert(snd.priority == 0u && "priority = 0");

    // Modify
    snd.sound_id = 42u;
    snd.position = apc::Vec3(5.0f, 2.0f, -3.0f);
    snd.volume = 0.75f;
    snd.priority = 10u;

    assert(snd.sound_id == 42u && "Modified sound_id = 42");
    assert(approx_eq(snd.position.x, 5.0f) && "Modified position x = 5");
    assert(approx_eq(snd.volume, 0.75f) && "Modified volume = 0.75");

    std::printf("    [PASS] SoundTriggerEvent verified\n");
    return 0;
}

// =============================================================================
// TEST 18: VibrationEvent patterns
// =============================================================================
static int test_vibration_event_patterns() {
    std::printf("  [Test 18] VibrationEvent patterns (PULSE, RAMP, CONSTANT)...\n");

    apc::VibrationEvent pulse;
    pulse.intensity = 1.0f;
    pulse.duration_ms = 100u;
    pulse.pattern = apc::VibrationPattern::PULSE;

    // PULSE: ramp up to 0.2, then decay
    assert(pulse.is_active_at_time(0.0f) && "PULSE active at t=0");
    float p0 = pulse.get_intensity_at_time(0.0f);
    assert(approx_eq(p0, 0.0f) && "PULSE t=0: starts at 0");
    float p1 = pulse.get_intensity_at_time(10.0f); // 10/100 = 0.1 → ramp up (0.1/0.2 = 0.5)
    assert(p1 > 0.0f && "PULSE t=10: ramping up");
    float p50 = pulse.get_intensity_at_time(50.0f); // 50/100 = 0.5 → decaying
    assert(p50 > 0.0f && "PULSE t=50: still active");
    assert(!pulse.is_active_at_time(100.0f) && "PULSE expired at t=100");
    assert(approx_eq(pulse.get_intensity_at_time(200.0f), 0.0f) && "PULSE t=200: 0");

    // RAMP: triangle wave
    apc::VibrationEvent ramp;
    ramp.intensity = 1.0f;
    ramp.duration_ms = 100u;
    ramp.pattern = apc::VibrationPattern::RAMP;

    float r25 = ramp.get_intensity_at_time(25.0f); // 0.25 * 2 = 0.5
    assert(approx_eq(r25, 0.5f) && "RAMP t=25: 0.5");
    float r50 = ramp.get_intensity_at_time(50.0f); // peak at 0.5
    assert(approx_eq(r50, 1.0f) && "RAMP t=50: peak = 1.0");
    float r75 = ramp.get_intensity_at_time(75.0f); // (2 - 1.5) = 0.5
    assert(approx_eq(r75, 0.5f) && "RAMP t=75: 0.5");

    // CONSTANT
    apc::VibrationEvent constant;
    constant.intensity = 0.7f;
    constant.duration_ms = 200u;
    constant.pattern = apc::VibrationPattern::CONSTANT;

    assert(approx_eq(constant.get_intensity_at_time(0.0f), 0.7f) && "CONSTANT = 0.7");
    assert(approx_eq(constant.get_intensity_at_time(100.0f), 0.7f) && "CONSTANT t=100 = 0.7");
    assert(approx_eq(constant.get_intensity_at_time(200.0f), 0.0f) && "CONSTANT t=200 = 0");

    // NONE pattern
    apc::VibrationEvent none;
    none.intensity = 1.0f;
    none.duration_ms = 100u;
    none.pattern = apc::VibrationPattern::NONE;
    assert(!none.is_active_at_time(50.0f) && "NONE: not active");

    std::printf("    [PASS] VibrationEvent patterns verified\n");
    return 0;
}

// =============================================================================
// TEST 19: HitStopIntegration consume and tick
// =============================================================================
static int test_hit_stop_integration() {
    std::printf("  [Test 19] HitStopIntegration consume and tick...\n");

    apc::HitStopIntegration hs;

    assert(!hs.should_freeze_render() && "Not frozen initially");
    assert(approx_eq(hs.get_remaining_ms(), 0.0f) && "Remaining = 0");
    assert(approx_eq(hs.get_time_scale(), 1.0f) && "Time scale = 1");

    // Consume hit-stop
    hs.consume(80.0f);
    assert(hs.should_freeze_render() && "Frozen after consume");
    assert(approx_eq(hs.get_remaining_ms(), 80.0f) && "Remaining = 80");
    assert(approx_eq(hs.get_time_scale(), 0.0f) && "Time scale = 0");

    // Update 30ms
    hs.update(30.0f);
    assert(hs.should_freeze_render() && "Still frozen");
    assert(approx_eq(hs.get_remaining_ms(), 50.0f) && "Remaining = 50");

    // Consume more (should keep max)
    hs.consume(20.0f);
    assert(approx_eq(hs.get_remaining_ms(), 50.0f) && "Keeps max 50");

    // Consume larger value
    hs.consume(100.0f);
    assert(approx_eq(hs.get_remaining_ms(), 100.0f) && "Takes max 100");

    // Tick down to zero
    hs.update(100.0f);
    assert(!hs.should_freeze_render() && "Unfrozen after tick down");
    assert(approx_eq(hs.get_remaining_ms(), 0.0f) && "Remaining = 0");
    assert(approx_eq(hs.get_time_scale(), 1.0f) && "Time scale = 1");

    // Reset
    hs.consume(50.0f);
    hs.reset();
    assert(!hs.should_freeze_render() && "Reset clears freeze");

    std::printf("    [PASS] HitStopIntegration verified\n");
    return 0;
}

// =============================================================================
// TEST 20: TimeScaleIntegration consume and recovery
// =============================================================================
static int test_time_scale_integration() {
    std::printf("  [Test 20] TimeScaleIntegration consume and recovery...\n");

    apc::TimeScaleIntegration ts;

    assert(!ts.is_slow_motion() && "Not slow-mo initially");
    assert(approx_eq(ts.get_time_scale(), 1.0f) && "Scale = 1.0");

    // Consume slow-mo
    ts.consume(0.25f, 500.0f); // 25% speed for 500ms
    assert(ts.get_time_scale() < 1.0f && "Scale < 1 after consume");

    // Tick forward — should start blending toward target
    ts.update(100.0f); // 100ms
    assert(ts.get_time_scale() < 1.0f && "Still slow");

    // Tick past duration
    ts.update(500.0f); // 500ms more (total 600ms > 500ms)
    // After duration expires, target goes back to 1.0, blend starts
    // The scale should be recovering toward 1.0
    assert(ts.get_time_scale() > 0.25f && "Recovering toward 1.0");

    // Full recovery
    for (int i = 0; i < 100; ++i) {
        ts.update(10.0f);
    }
    assert(approx_eq(ts.get_time_scale(), 1.0f) && "Fully recovered");

    // Reset
    ts.consume(0.1f, 200.0f);
    ts.reset();
    assert(approx_eq(ts.get_time_scale(), 1.0f) && "Reset: scale = 1.0");

    std::printf("    [PASS] TimeScaleIntegration verified\n");
    return 0;
}

// =============================================================================
// TEST 21: VFXBridge dispatch and update
// =============================================================================
static int test_vfx_bridge_dispatch() {
    std::printf("  [Test 21] VFXBridge dispatch and update...\n");

    apc::VFXBridge bridge;

    // Create game hook outputs
    apc::GameHookOutput outputs[3];
    outputs[0].type = apc::HookEffectType::CAMERA_SHAKE;
    outputs[0].impact_intensity = 0.7f;
    outputs[0].position = apc::Vec3(1.0f, 2.0f, 3.0f);
    outputs[0].float_param = 0.5f; // duration

    outputs[1].type = apc::HookEffectType::HIT_STOP;
    outputs[1].float_param = 50.0f; // 50ms

    outputs[2].type = apc::HookEffectType::TIME_SCALE;
    outputs[2].float_param = 0.3f; // 30% speed
    outputs[2].float_param2 = 200.0f; // 200ms

    bridge.dispatch(outputs, 3);

    // Camera shake should be active
    assert(bridge.camera_shake.is_active() && "Camera shake active after dispatch");

    // Hit-stop should be active
    assert(bridge.hit_stop.should_freeze_render() && "Hit-stop active after dispatch");

    // Time scale should be slow
    assert(bridge.time_scale.get_time_scale() < 1.0f && "Time scale < 1 after dispatch");

    // Update
    bridge.update(0.016f); // One frame

    // Sound queue should be consumed (cleared) after update
    assert(bridge.sound_count == 0u && "Sounds consumed after update");
    assert(bridge.vibration_count == 0u && "Vibrations consumed after update");

    std::printf("    [PASS] VFXBridge dispatch verified\n");
    return 0;
}

// =============================================================================
// TEST 22: VFXBridge effective time scale
// =============================================================================
static int test_vfx_bridge_effective_time_scale() {
    std::printf("  [Test 22] VFXBridge effective time scale...\n");

    apc::VFXBridge bridge;

    // Normal state: effective scale = 1.0
    assert(approx_eq(bridge.get_effective_time_scale(), 1.0f) && "Normal: scale = 1.0");
    assert(!bridge.is_render_frozen() && "Normal: not frozen");

    // With hit-stop: effective scale = 0
    bridge.hit_stop.consume(100.0f);
    assert(approx_eq(bridge.get_effective_time_scale(), 0.0f) && "Hit-stop: scale = 0");
    assert(bridge.is_render_frozen() && "Hit-stop: frozen");

    // Tick hit-stop away
    bridge.update(100.0f);
    assert(approx_eq(bridge.get_effective_time_scale(), 1.0f) && "After hit-stop: scale = 1");

    // With time-scale only
    bridge.time_scale.consume(0.5f, 500.0f);
    float eff = bridge.get_effective_time_scale();
    assert(eff < 1.0f && eff > 0.0f && "Time-scale: 0 < scale < 1");

    // Combined: hit-stop overrides time-scale
    bridge.hit_stop.consume(50.0f);
    assert(approx_eq(bridge.get_effective_time_scale(), 0.0f) && "Hit-stop + time-scale: 0");

    // Reset
    bridge.reset();
    assert(approx_eq(bridge.get_effective_time_scale(), 1.0f) && "Reset: scale = 1");
    assert(!bridge.camera_shake.is_active() && "Reset: no shake");
    assert(!bridge.screen_flash.active() && "Reset: no flash");
    assert(!bridge.hit_stop.should_freeze_render() && "Reset: no hit-stop");

    std::printf("    [PASS] VFXBridge effective time scale verified\n");
    return 0;
}

// =============================================================================
// TEST 23: ViewportConfig defaults and layer presets
// =============================================================================
static int test_viewport_config() {
    std::printf("  [Test 23] ViewportConfig defaults and layer presets...\n");

    apc::ViewportConfig vc;

    assert(approx_eq(vc.x, 0.0f) && "x = 0");
    assert(approx_eq(vc.y, 0.0f) && "y = 0");
    assert(approx_eq(vc.width, 1.0f) && "width = 1");
    assert(approx_eq(vc.height, 1.0f) && "height = 1");
    assert(vc.camera_id == 0u && "camera_id = 0");
    assert(vc.render_layer_mask == 0xFFu && "layer_mask = 0xFF");

    // Contains point
    assert(vc.contains_point(0.5f, 0.5f) && "Contains (0.5, 0.5)");
    assert(!vc.contains_point(1.5f, 0.5f) && "Doesn't contain (1.5, 0.5)");
    assert(!vc.contains_point(0.5f, 1.5f) && "Doesn't contain (0.5, 1.5)");

    // Aspect ratio
    assert(approx_eq(vc.get_aspect_ratio(), 1.0f) && "1:1 aspect = 1.0");

    // Layer presets
    uint32_t main = apc::ViewportConfig::layer_main_view();
    assert((main & apc::ViewportConfig::LAYER_OPAQUE) != 0u && "Main: opaque");
    assert((main & apc::ViewportConfig::LAYER_TRANSPARENT) != 0u && "Main: transparent");
    assert((main & apc::ViewportConfig::LAYER_DEBUG) == 0u && "Main: no debug");

    uint32_t dbg = apc::ViewportConfig::layer_debug_view();
    assert((dbg & apc::ViewportConfig::LAYER_OVERLAY) != 0u && "Debug: overlay");
    assert((dbg & apc::ViewportConfig::LAYER_DEBUG) != 0u && "Debug: debug");
    assert((dbg & apc::ViewportConfig::LAYER_OPAQUE) == 0u && "Debug: no opaque");

    uint32_t mini = apc::ViewportConfig::layer_minimap();
    assert((mini & apc::ViewportConfig::LAYER_OPAQUE) != 0u && "Minimap: opaque");

    uint32_t all = apc::ViewportConfig::layer_all();
    assert((all & apc::ViewportConfig::LAYER_OPAQUE) != 0u && "All: opaque");
    assert((all & apc::ViewportConfig::LAYER_HUD) != 0u && "All: HUD");

    std::printf("    [PASS] ViewportConfig defaults and presets verified\n");
    return 0;
}

// =============================================================================
// TEST 24: MultiViewport FULL layout
// =============================================================================
static int test_multi_viewport_full() {
    std::printf("  [Test 24] MultiViewport FULL layout...\n");

    apc::MultiViewport mv;
    mv.set_split_layout(apc::SplitLayout::FULL);

    assert(mv.get_viewport_count() == 1u && "FULL: 1 viewport");
    assert(mv.get_layout() == apc::SplitLayout::FULL);

    const apc::ViewportConfig* vp = mv.get_viewport(0);
    assert(vp != nullptr && "Viewport 0 exists");
    assert(approx_eq(vp->x, 0.0f) && "FULL: x = 0");
    assert(approx_eq(vp->y, 0.0f) && "FULL: y = 0");
    assert(approx_eq(vp->width, 1.0f) && "FULL: width = 1");
    assert(approx_eq(vp->height, 1.0f) && "FULL: height = 1");

    assert(mv.get_viewport(1) == nullptr && "Viewport 1 doesn't exist in FULL");

    std::printf("    [PASS] FULL layout verified\n");
    return 0;
}

// =============================================================================
// TEST 25: MultiViewport HORIZONTAL layout
// =============================================================================
static int test_multi_viewport_horizontal() {
    std::printf("  [Test 25] MultiViewport HORIZONTAL layout...\n");

    apc::MultiViewport mv;
    mv.set_split_layout(apc::SplitLayout::HORIZONTAL);

    assert(mv.get_viewport_count() == 2u && "HORIZONTAL: 2 viewports");

    const apc::ViewportConfig* vp0 = mv.get_viewport(0);
    const apc::ViewportConfig* vp1 = mv.get_viewport(1);

    assert(vp0 != nullptr && vp1 != nullptr && "Both viewports exist");

    assert(approx_eq(vp0->x, 0.0f) && "Left: x = 0");
    assert(approx_eq(vp0->width, 0.5f) && "Left: width = 0.5");
    assert(approx_eq(vp0->height, 1.0f) && "Left: height = 1.0");
    assert(vp0->camera_id == 0u && "Left: camera_id = 0");

    assert(approx_eq(vp1->x, 0.5f) && "Right: x = 0.5");
    assert(approx_eq(vp1->width, 0.5f) && "Right: width = 0.5");
    assert(approx_eq(vp1->height, 1.0f) && "Right: height = 1.0");
    assert(vp1->camera_id == 1u && "Right: camera_id = 1");

    std::printf("    [PASS] HORIZONTAL layout verified\n");
    return 0;
}

// =============================================================================
// TEST 26: MultiViewport VERTICAL layout
// =============================================================================
static int test_multi_viewport_vertical() {
    std::printf("  [Test 26] MultiViewport VERTICAL layout...\n");

    apc::MultiViewport mv;
    mv.set_split_layout(apc::SplitLayout::VERTICAL);

    assert(mv.get_viewport_count() == 2u && "VERTICAL: 2 viewports");

    const apc::ViewportConfig* vp0 = mv.get_viewport(0);
    const apc::ViewportConfig* vp1 = mv.get_viewport(1);

    assert(approx_eq(vp0->x, 0.0f) && "Top: x = 0");
    assert(approx_eq(vp0->y, 0.0f) && "Top: y = 0");
    assert(approx_eq(vp0->width, 1.0f) && "Top: width = 1");
    assert(approx_eq(vp0->height, 0.5f) && "Top: height = 0.5");

    assert(approx_eq(vp1->x, 0.0f) && "Bottom: x = 0");
    assert(approx_eq(vp1->y, 0.5f) && "Bottom: y = 0.5");
    assert(approx_eq(vp1->width, 1.0f) && "Bottom: width = 1");
    assert(approx_eq(vp1->height, 0.5f) && "Bottom: height = 0.5");

    std::printf("    [PASS] VERTICAL layout verified\n");
    return 0;
}

// =============================================================================
// TEST 27: MultiViewport QUAD layout
// =============================================================================
static int test_multi_viewport_quad() {
    std::printf("  [Test 27] MultiViewport QUAD layout...\n");

    apc::MultiViewport mv;
    mv.set_split_layout(apc::SplitLayout::QUAD);

    assert(mv.get_viewport_count() == 4u && "QUAD: 4 viewports");

    // Top-left
    const apc::ViewportConfig* tl = mv.get_viewport(0);
    assert(approx_eq(tl->x, 0.0f) && "TL: x = 0");
    assert(approx_eq(tl->y, 0.0f) && "TL: y = 0");
    assert(approx_eq(tl->width, 0.5f) && "TL: width = 0.5");
    assert(approx_eq(tl->height, 0.5f) && "TL: height = 0.5");
    assert(tl->camera_id == 0u && "TL: camera 0");

    // Top-right
    const apc::ViewportConfig* tr = mv.get_viewport(1);
    assert(approx_eq(tr->x, 0.5f) && "TR: x = 0.5");
    assert(approx_eq(tr->y, 0.0f) && "TR: y = 0");
    assert(tr->camera_id == 1u && "TR: camera 1");

    // Bottom-left
    const apc::ViewportConfig* bl = mv.get_viewport(2);
    assert(approx_eq(bl->x, 0.0f) && "BL: x = 0");
    assert(approx_eq(bl->y, 0.5f) && "BL: y = 0.5");
    assert(bl->camera_id == 2u && "BL: camera 2");

    // Bottom-right
    const apc::ViewportConfig* br = mv.get_viewport(3);
    assert(approx_eq(br->x, 0.5f) && "BR: x = 0.5");
    assert(approx_eq(br->y, 0.5f) && "BR: y = 0.5");
    assert(br->camera_id == 3u && "BR: camera 3");

    // Out of bounds
    assert(mv.get_viewport(4) == nullptr && "Viewport 4 doesn't exist");

    std::printf("    [PASS] QUAD layout verified\n");
    return 0;
}

// =============================================================================
// TEST 28: MultiViewport per-viewport camera and layer assignment
// =============================================================================
static int test_multi_viewport_assignment() {
    std::printf("  [Test 28] MultiViewport per-viewport camera and layer assignment...\n");

    apc::MultiViewport mv;
    mv.set_split_layout(apc::SplitLayout::HORIZONTAL);

    // Change camera assignment
    bool ok1 = mv.set_viewport_camera(0u, 10u);
    assert(ok1 && "Set camera 10 on viewport 0");
    assert(mv.get_viewport(0)->camera_id == 10u && "Viewport 0: camera = 10");

    // Change layer mask
    bool ok2 = mv.set_viewport_layers(1u, apc::ViewportConfig::layer_debug_view());
    assert(ok2 && "Set debug layers on viewport 1");
    assert(mv.get_viewport(1)->render_layer_mask == apc::ViewportConfig::layer_debug_view());

    // Invalid index
    bool fail1 = mv.set_viewport_camera(5u, 0u);
    assert(!fail1 && "Invalid viewport index fails");

    // Add custom viewport
    apc::ViewportConfig custom;
    custom.x = 0.75f;
    custom.y = 0.75f;
    custom.width = 0.25f;
    custom.height = 0.25f;
    custom.camera_id = 99u;
    uint32_t id = mv.add_viewport(custom);
    assert(id != 0xFFFFFFFF && "Custom viewport added");
    assert(mv.get_viewport_count() == 3u && "Total 3 viewports");
    assert(mv.get_viewport(2)->camera_id == 99u && "Custom: camera = 99");

    std::printf("    [PASS] Per-viewport assignment verified\n");
    return 0;
}

// =============================================================================
// TEST 29: MultiViewport viewport border drawing
// =============================================================================
static int test_multi_viewport_borders() {
    std::printf("  [Test 29] MultiViewport viewport border drawing...\n");

    // FULL layout: no borders
    {
        apc::MultiViewport mv;
        mv.set_split_layout(apc::SplitLayout::FULL);
        apc::DebugDraw dd;
        mv.draw_viewport_borders(dd, 1920.0f, 1080.0f);
        assert(dd.list.get_line_count() == 0u && "FULL: no borders");
    }

    // HORIZONTAL layout: 1 vertical divider
    {
        apc::MultiViewport mv;
        mv.set_split_layout(apc::SplitLayout::HORIZONTAL);
        apc::DebugDraw dd;
        mv.draw_viewport_borders(dd, 1920.0f, 1080.0f);
        assert(dd.list.get_line_count() == 1u && "HORIZONTAL: 1 border line");
    }

    // VERTICAL layout: 1 horizontal divider
    {
        apc::MultiViewport mv;
        mv.set_split_layout(apc::SplitLayout::VERTICAL);
        apc::DebugDraw dd;
        mv.draw_viewport_borders(dd, 1920.0f, 1080.0f);
        assert(dd.list.get_line_count() == 1u && "VERTICAL: 1 border line");
    }

    // QUAD layout: 2 dividers (1 vertical + 1 horizontal)
    {
        apc::MultiViewport mv;
        mv.set_split_layout(apc::SplitLayout::QUAD);
        apc::DebugDraw dd;
        mv.draw_viewport_borders(dd, 1920.0f, 1080.0f);
        assert(dd.list.get_line_count() == 2u && "QUAD: 2 border lines");
    }

    std::printf("    [PASS] Viewport border drawing verified\n");
    return 0;
}

// =============================================================================
// TEST 30: RenderPerformanceOverlay FPS tracking
// =============================================================================
static int test_perf_overlay_fps() {
    std::printf("  [Test 30] RenderPerformanceOverlay FPS tracking...\n");

    apc::RenderPerformanceOverlay overlay;

    assert(approx_eq(overlay.get_fps(), 60.0f) && "Default FPS = 60");
    assert(overlay.show_fps && "show_fps = true");

    // Simulate 60 frames at 60fps (16.67ms each)
    apc::RenderStats stats;
    stats.frame_time_ms = 16.67f;
    for (uint32_t i = 0; i < 60; ++i) {
        overlay.update_frame(1.0f / 60.0f, stats);
    }

    float fps = overlay.get_fps();
    assert(fps > 50.0f && fps < 70.0f && "FPS ~60 after 60 frames");

    // Draw FPS
    apc::DebugDraw dd;
    overlay.draw_fps(dd);
    assert(dd.list.get_line_count() == 1u && "FPS bar: 1 line");

    std::printf("    [PASS] FPS tracking verified\n");
    return 0;
}

// =============================================================================
// TEST 31: RenderPerformanceOverlay physics stats
// =============================================================================
static int test_perf_overlay_physics() {
    std::printf("  [Test 31] RenderPerformanceOverlay physics stats...\n");

    apc::RenderPerformanceOverlay overlay;
    overlay.show_physics_stats = true;

    overlay.update_physics_stats(0.005f, 42u, 128u);
    assert(approx_eq(overlay.physics_frame_time_ms, 5.0f) && "Physics time = 5ms");
    assert(overlay.contact_count == 42u && "Contacts = 42");
    assert(overlay.broadphase_pairs == 128u && "Broadphase = 128");

    apc::DebugDraw dd;
    overlay.draw_physics_stats(dd);
    // Physics: 3 bars (frame time, contacts, broadphase)
    assert(dd.list.get_line_count() == 3u && "Physics: 3 bars");

    std::printf("    [PASS] Physics stats verified\n");
    return 0;
}

// =============================================================================
// TEST 32: RenderPerformanceOverlay render stats
// =============================================================================
static int test_perf_overlay_render() {
    std::printf("  [Test 32] RenderPerformanceOverlay render stats...\n");

    apc::RenderPerformanceOverlay overlay;
    overlay.show_render_stats = true;

    apc::RenderStats stats;
    stats.draw_calls = 150u;
    stats.triangle_count = 5000u;
    stats.frame_time_ms = 8.0f;
    overlay.update_frame(0.008f, stats);

    assert(overlay.draw_calls == 150u && "Draw calls = 150");
    assert(overlay.triangle_count == 5000u && "Triangles = 5000");
    assert(approx_eq(overlay.get_frame_time_ms(), 8.0f) && "Frame time = 8ms");

    apc::DebugDraw dd;
    overlay.draw_render_stats(dd);
    // Render: 2 bars (draw calls, triangles)
    assert(dd.list.get_line_count() == 2u && "Render: 2 bars");

    // draw_all
    overlay.show_fps = true;
    overlay.show_memory_stats = true;
    apc::DebugDraw dd_all;
    overlay.draw_all(dd_all);
    // FPS + physics(3) + render(2) + memory(1) = 7 lines (physics not enabled)
    // FPS(1) + render(2) + memory(1) = 4
    assert(dd_all.list.get_line_count() >= 4u && "draw_all: at least 4 bars");

    std::printf("    [PASS] Render stats verified\n");
    return 0;
}

// =============================================================================
// TEST 33: Integration — broadcast camera + shake + VFX + viewport + overlay
// =============================================================================
static int test_integration() {
    std::printf("  [Test 33] Integration: broadcast + shake + VFX + viewport + overlay...\n");

    // Set up broadcast camera system
    apc::BroadcastCameraSystem bcs;
    apc::CameraTransform main_cam, wide_cam, follow_cam;
    main_cam.eye = apc::Vec3(0.0f, 30.0f, -50.0f);
    main_cam.target = apc::Vec3(0.0f, 0.0f, 0.0f);
    wide_cam.eye = apc::Vec3(0.0f, 60.0f, -80.0f);
    wide_cam.target = apc::Vec3(0.0f, 0.0f, 0.0f);
    follow_cam.eye = apc::Vec3(5.0f, 3.0f, -10.0f);
    follow_cam.target = apc::Vec3(0.0f, 1.0f, 0.0f);

    bcs.register_camera(0u, apc::CameraShotType::BROADCAST_MAIN, main_cam, 3.0f);
    bcs.register_camera(1u, apc::CameraShotType::BROADCAST_WIDE, wide_cam, 2.5f);
    bcs.register_camera(2u, apc::CameraShotType::FOLLOW_TARGET, follow_cam, 2.0f);

    // Set up multi-viewport (horizontal split)
    apc::MultiViewport mv;
    mv.set_split_layout(apc::SplitLayout::HORIZONTAL);
    mv.set_viewport_camera(0u, 0u); // Left = main cam
    mv.set_viewport_camera(1u, 1u); // Right = wide cam

    // Set up VFX bridge
    apc::VFXBridge vfx;

    // Set up performance overlay
    apc::RenderPerformanceOverlay perf;
    perf.show_fps = true;
    perf.show_physics_stats = true;
    perf.show_render_stats = true;

    // Simulate: goal scored event
    bcs.time_since_cut = 10.0f;
    bcs.update(apc::SportEventType::GOAL_SCORED, 0.016f);
    assert(bcs.get_current_shot_type() == apc::CameraShotType::BROADCAST_WIDE);

    // Simulate: impact → VFX
    apc::GameHookOutput hook_outs[2];
    hook_outs[0].type = apc::HookEffectType::CAMERA_SHAKE;
    hook_outs[0].impact_intensity = 0.9f;
    hook_outs[0].position = apc::Vec3(45.0f, 0.0f, 0.0f);
    hook_outs[0].float_param = 0.8f;

    hook_outs[1].type = apc::HookEffectType::SCREEN_FLASH;
    hook_outs[1].impact_intensity = 0.5f;

    vfx.dispatch(hook_outs, 2);
    assert(vfx.camera_shake.is_active() && "Integration: shake active");
    assert(vfx.screen_flash.active() && "Integration: flash active");

    // Update VFX
    vfx.update(0.016f);

    // Get camera shake offset and apply to camera transform
    apc::Vec3 shake_offset = vfx.camera_shake.get_offset();
    apc::Vec3 shake_rot = vfx.camera_shake.get_rotation_offset();
    assert(apc::Vec3::length(shake_offset) > 0.0f && "Integration: non-zero shake offset");

    // Update performance overlay
    apc::RenderStats rstats;
    rstats.draw_calls = 42u;
    rstats.triangle_count = 3000u;
    rstats.frame_time_ms = 16.67f;
    perf.update_frame(0.016f, rstats);
    perf.update_physics_stats(0.004f, 15u, 80u);

    // Draw everything
    apc::DebugDraw dd;
    mv.draw_viewport_borders(dd, 1920.0f, 1080.0f);
    assert(dd.list.get_line_count() == 1u && "Integration: 1 border line");

    perf.draw_all(dd);
    uint32_t perf_lines = dd.list.get_line_count() - 1u;
    assert(perf_lines >= 4u && "Integration: performance bars drawn");

    // Physics-to-render sync
    apc::PhysicsToRenderSync sync;
    apc::Vec3 positions[2] = {
        apc::Vec3(10.0f, 1.0f, 20.0f),
        apc::Vec3(-10.0f, 1.0f, -20.0f)
    };
    apc::Quat orientations[2] = { apc::Quat::identity(), apc::Quat::identity() };
    sync.sync_from_physics(positions, orientations, 2,
        apc::PhysicsToRenderSync::SyncPoint::POST_PHYSICS);
    sync.swap_buffers();

    const auto* transforms = sync.get_transforms();
    assert(approx_eq(transforms[0].position.x, 10.0f) && "Integration: synced pos x = 10");
    assert(approx_eq(transforms[1].position.z, -20.0f) && "Integration: synced pos z = -20");

    std::printf("    [PASS] Full integration verified\n");
    return 0;
}

// =============================================================================
// MAIN
// =============================================================================
int main() {
    std::printf("=== Sprint 20: Camera Systems, VFX Integration & Multi-Viewport ===\n\n");

    int result = 0;
    result |= test_camera_shot_type_enum();
    result |= test_broadcast_camera_config();
    result |= test_broadcast_camera_register();
    result |= test_broadcast_camera_sport_event();
    result |= test_broadcast_camera_trigger_cut();
    result |= test_broadcast_camera_auto_switch();
    result |= test_broadcast_camera_cut_delay();
    result |= test_replay_camera_config();
    result |= test_replay_camera_playback();
    result |= test_replay_camera_loop();
    result |= test_replay_camera_orbit();
    result |= test_camera_shake_config();
    result |= test_camera_shake_decay();
    result |= test_camera_shake_offset();
    result |= test_physics_to_render_sync();
    result |= test_screen_flash();
    result |= test_sound_trigger_event();
    result |= test_vibration_event_patterns();
    result |= test_hit_stop_integration();
    result |= test_time_scale_integration();
    result |= test_vfx_bridge_dispatch();
    result |= test_vfx_bridge_effective_time_scale();
    result |= test_viewport_config();
    result |= test_multi_viewport_full();
    result |= test_multi_viewport_horizontal();
    result |= test_multi_viewport_vertical();
    result |= test_multi_viewport_quad();
    result |= test_multi_viewport_assignment();
    result |= test_multi_viewport_borders();
    result |= test_perf_overlay_fps();
    result |= test_perf_overlay_physics();
    result |= test_perf_overlay_render();
    result |= test_integration();

    std::printf("\n");
    if (result == 0) {
        std::printf("Sprint 20: ALL TESTS PASSED\n");
    } else {
        std::printf("Sprint 20: SOME TESTS FAILED\n");
    }

    return result;
}
