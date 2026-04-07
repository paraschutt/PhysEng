#pragma once
// =============================================================================
// Camera System — Broadcast, Replay, Shake, and Physics-to-Render Sync
// =============================================================================
//
// Provides:
//   - CameraShotType: enumeration of broadcast camera shot types
//   - BroadcastCameraConfig: shot switching configuration
//   - BroadcastCameraSystem: sport-aware broadcast camera director
//   - ReplayCameraConfig: replay playback configuration
//   - ReplayCamera: replay seek/play/orbit camera
//   - CameraShakeConfig: shake parameters
//   - CameraShake: intensity-based camera shake with decay
//   - PhysicsToRenderSync: double-buffered transform sync
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation (fixed-capacity arrays)
//   - Deterministic: same inputs always produce same outputs
//   - C++17
//
// =============================================================================

#include "apc_render_types.h"
#include "apc_render_camera.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include "apc_math/apc_math_common.h"
#include <cstdint>
#include <cmath>

namespace apc {

// =============================================================================
// CameraShotType — Types of broadcast camera shots
// =============================================================================
enum class CameraShotType : uint8_t {
    FREE              = 0,
    FOLLOW_TARGET     = 1,
    BROADCAST_MAIN    = 2,
    BROADCAST_WIDE    = 3,
    BROADCAST_TACTICAL = 4,
    REPLAY_ORBIT      = 5,
    REPLAY_FIXED      = 6,
    SIDELINE          = 7,
    ENDZONE           = 8,
    SKYCAM            = 9,
    FIRST_PERSON       = 10
};

// =============================================================================
// SportEventType — Events that trigger camera switches
// =============================================================================
enum class SportEventType : uint8_t {
    NONE            = 0,
    GOAL_SCORED     = 1,
    TACKLE          = 2,
    FREE_KICK       = 3,
    TIMEOUT         = 4,
    KICKOFF         = 5,
    CORNER_KICK     = 6,
    PENALTY_KICK    = 7,
    SCORE_ANY       = 8,    // Generic score (touchdown, try, etc.)
    BALL_CHANGE     = 9,    // Ball possession change
    WHISTLE         = 10    // Referee whistle / stoppage
};

// =============================================================================
// BroadcastCameraConfig — Shot switching configuration
// =============================================================================
struct BroadcastCameraConfig {
    float cut_delay                 = 2.0f;   // Min seconds between auto-cuts
    float shot_priority_weights[11] = {
        1.0f,  // FREE
        2.0f,  // FOLLOW_TARGET
        3.0f,  // BROADCAST_MAIN
        2.5f,  // BROADCAST_WIDE
        2.0f,  // BROADCAST_TACTICAL
        1.5f,  // REPLAY_ORBIT
        1.0f,  // REPLAY_FIXED
        2.0f,  // SIDELINE
        2.0f,  // ENDZONE
        1.5f,  // SKYCAM
        1.0f   // FIRST_PERSON
    };
    float zoom_preferences[11] = {
        0.5f,  // FREE
        0.6f,  // FOLLOW_TARGET
        0.7f,  // BROADCAST_MAIN
        0.3f,  // BROADCAST_WIDE
        0.4f,  // BROADCAST_TACTICAL
        0.5f,  // REPLAY_ORBIT
        0.5f,  // REPLAY_FIXED
        0.6f,  // SIDELINE
        0.5f,  // ENDZONE
        0.2f,  // SKYCAM
        0.8f   // FIRST_PERSON
    };
    float smooth_transition_speed   = 3.0f;   // Blend speed for transitions
    float auto_switch_threshold     = 5.0f;   // Seconds of inactivity before auto-switch
};

// =============================================================================
// CameraShot — A registered camera shot
// =============================================================================
struct CameraShot {
    uint32_t        camera_id   = 0;
    CameraShotType  type        = CameraShotType::FREE;
    CameraTransform transform;
    float           priority    = 1.0f;
    float           zoom_level  = 0.5f;
    bool            enabled     = true;
};

// =============================================================================
// BroadcastCameraSystem — Sport-aware broadcast camera director
// =============================================================================
struct BroadcastCameraSystem {
    static constexpr uint32_t MAX_CAMERAS = 8u;

    BroadcastCameraConfig config;
    CameraShot cameras[MAX_CAMERAS];
    uint32_t  camera_count = 0u;

    uint32_t        active_camera_id  = 0u;
    CameraShotType  current_shot_type = CameraShotType::BROADCAST_MAIN;
    float           time_since_cut    = 0.0f;
    float           time_since_event  = 100.0f;  // Start high to allow first cut
    bool            transition_active = false;
    float           transition_progress = 0.0f;

    // --- register_camera ---
    uint32_t register_camera(uint32_t id, CameraShotType type,
                              const CameraTransform& transform,
                              float priority = 1.0f)
    {
        if (camera_count >= MAX_CAMERAS) return 0xFFFFFFFF;

        CameraShot& shot = cameras[camera_count];
        shot.camera_id    = id;
        shot.type         = type;
        shot.transform    = transform;
        shot.priority     = priority;
        shot.zoom_level   = config.zoom_preferences[static_cast<uint8_t>(type)];
        shot.enabled      = true;

        ++camera_count;

        // First camera becomes active
        if (camera_count == 1u) {
            active_camera_id = id;
            current_shot_type = type;
        }

        return camera_count - 1u;
    }

    // --- update: advance time and handle auto-switch ---
    void update(SportEventType sport_event, float dt) {
        time_since_cut   += dt;
        time_since_event += dt;

        // Handle sport event → trigger shot change
        if (sport_event != SportEventType::NONE) {
            time_since_event = 0.0f;
            handle_sport_event(sport_event);
        }

        // Auto-switch after inactivity threshold
        if (time_since_event > config.auto_switch_threshold &&
            time_since_cut > config.cut_delay)
        {
            auto_switch();
        }

        // Update transition blend
        if (transition_active) {
            transition_progress += dt * config.smooth_transition_speed;
            if (transition_progress >= 1.0f) {
                transition_progress = 1.0f;
                transition_active = false;
            }
        }
    }

    // --- get_active_camera: get current camera transform ---
    const CameraTransform& get_active_camera() const {
        for (uint32_t i = 0; i < camera_count; ++i) {
            if (cameras[i].camera_id == active_camera_id) {
                return cameras[i].transform;
            }
        }
        // Fallback: return first camera
        return cameras[0].transform;
    }

    // --- get_current_shot_type ---
    CameraShotType get_current_shot_type() const {
        return current_shot_type;
    }

    // --- trigger_cut: manually switch to a specific camera ---
    bool trigger_cut(uint32_t camera_id) {
        if (time_since_cut < config.cut_delay) return false;

        for (uint32_t i = 0; i < camera_count; ++i) {
            if (cameras[i].camera_id == camera_id && cameras[i].enabled) {
                active_camera_id  = camera_id;
                current_shot_type = cameras[i].type;
                time_since_cut    = 0.0f;
                transition_active = true;
                transition_progress = 0.0f;
                return true;
            }
        }
        return false;
    }

    // --- trigger_cut: switch to a specific shot type (picks highest priority) ---
    bool trigger_cut(CameraShotType type) {
        if (time_since_cut < config.cut_delay) return false;

        int best = -1;
        float best_priority = -1.0f;
        for (uint32_t i = 0; i < camera_count; ++i) {
            if (cameras[i].type == type && cameras[i].enabled &&
                cameras[i].priority > best_priority)
            {
                best = static_cast<int>(i);
                best_priority = cameras[i].priority;
            }
        }

        if (best >= 0) {
            active_camera_id  = cameras[best].camera_id;
            current_shot_type = type;
            time_since_cut    = 0.0f;
            transition_active = true;
            transition_progress = 0.0f;
            return true;
        }
        return false;
    }

    // --- set_transform: update a camera's transform ---
    bool set_transform(uint32_t camera_id, const CameraTransform& transform) {
        for (uint32_t i = 0; i < camera_count; ++i) {
            if (cameras[i].camera_id == camera_id) {
                cameras[i].transform = transform;
                return true;
            }
        }
        return false;
    }

    // --- get_zoom_level ---
    float get_zoom_level() const {
        for (uint32_t i = 0; i < camera_count; ++i) {
            if (cameras[i].camera_id == active_camera_id) {
                return cameras[i].zoom_level;
            }
        }
        return 0.5f;
    }

    // --- is_transitioning ---
    bool is_transitioning() const { return transition_active; }

    // --- get_transition_progress ---
    float get_transition_progress() const { return transition_progress; }

private:
    // --- handle_sport_event: pick best camera for the event ---
    void handle_sport_event(SportEventType event) {
        CameraShotType desired = CameraShotType::BROADCAST_MAIN;

        switch (event) {
        case SportEventType::GOAL_SCORED:
            desired = CameraShotType::BROADCAST_WIDE;
            break;
        case SportEventType::TACKLE:
            desired = CameraShotType::FOLLOW_TARGET;
            break;
        case SportEventType::FREE_KICK:
        case SportEventType::PENALTY_KICK:
        case SportEventType::CORNER_KICK:
            desired = CameraShotType::BROADCAST_TACTICAL;
            break;
        case SportEventType::TIMEOUT:
        case SportEventType::WHISTLE:
            desired = CameraShotType::BROADCAST_WIDE;
            break;
        case SportEventType::KICKOFF:
            desired = CameraShotType::SKYCAM;
            break;
        case SportEventType::SCORE_ANY:
            desired = CameraShotType::REPLAY_ORBIT;
            break;
        default:
            desired = CameraShotType::BROADCAST_MAIN;
            break;
        }

        trigger_cut(desired);
    }

    // --- auto_switch: pick next best camera for variety ---
    void auto_switch() {
        // Find the camera with highest priority that is not currently active
        int best = -1;
        float best_priority = -1.0f;
        for (uint32_t i = 0; i < camera_count; ++i) {
            if (cameras[i].camera_id != active_camera_id &&
                cameras[i].enabled &&
                cameras[i].priority > best_priority)
            {
                best = static_cast<int>(i);
                best_priority = cameras[i].priority;
            }
        }

        if (best >= 0) {
            active_camera_id  = cameras[best].camera_id;
            current_shot_type = cameras[best].type;
            time_since_cut    = 0.0f;
            transition_active = true;
            transition_progress = 0.0f;
        }
    }
};

// =============================================================================
// ReplayCameraConfig — Replay playback parameters
// =============================================================================
struct ReplayCameraConfig {
    float playback_speed       = 1.0f;     // 1.0 = normal, 0.5 = half, 2.0 = double
    float loop_start           = 0.0f;     // Start time of replay loop
    float loop_end             = 10.0f;    // End time of replay loop
    float orbit_center_x       = 0.0f;
    float orbit_center_y       = 0.0f;
    float orbit_center_z       = 0.0f;
    float orbit_distance       = 15.0f;
    float orbit_yaw            = 0.0f;
    float orbit_pitch          = 0.4f;
    float seek_speed           = 0.1f;     // Normal speed seek rate
};

// =============================================================================
// ReplayCamera — Replay seek/play/orbit camera
// =============================================================================
struct ReplayCamera {
    ReplayCameraConfig config;
    float current_time  = 0.0f;
    bool  is_playing    = false;
    bool  is_looping    = false;

    // --- seek_to ---
    void seek_to(float time) {
        current_time = time;
        if (current_time < config.loop_start) current_time = config.loop_start;
        if (current_time > config.loop_end) current_time = config.loop_end;
    }

    // --- play ---
    void play() { is_playing = true; }

    // --- pause ---
    void pause() { is_playing = false; }

    // --- toggle_loop ---
    void set_loop(bool loop) { is_looping = loop; }

    // --- set_orbit ---
    void set_orbit(const Vec3& center, float distance, float yaw, float pitch) {
        config.orbit_center_x = center.x;
        config.orbit_center_y = center.y;
        config.orbit_center_z = center.z;
        config.orbit_distance = distance;
        config.orbit_yaw      = yaw;
        config.orbit_pitch    = pitch;
    }

    // --- update ---
    void update(float dt) {
        if (!is_playing) return;

        current_time += dt * config.playback_speed;

        if (current_time >= config.loop_end) {
            if (is_looping) {
                current_time = config.loop_start +
                    (current_time - config.loop_start) -
                    (config.loop_end - config.loop_start);
                if (current_time < config.loop_start) current_time = config.loop_start;
            } else {
                current_time = config.loop_end;
                is_playing = false;
            }
        }
    }

    // --- get_camera_at_time: compute CameraTransform for orbit replay view ---
    CameraTransform get_camera_at_time() const {
        CameraTransform result;

        Vec3 center(config.orbit_center_x, config.orbit_center_y, config.orbit_center_z);

        // Compute orbit position from yaw, pitch, distance
        float cos_pitch = std::cos(config.orbit_pitch);
        float sin_pitch = std::sin(config.orbit_pitch);
        float cos_yaw   = std::cos(config.orbit_yaw);
        float sin_yaw   = std::sin(config.orbit_yaw);

        float dx = config.orbit_distance * cos_pitch * sin_yaw;
        float dy = config.orbit_distance * sin_pitch;
        float dz = config.orbit_distance * cos_pitch * cos_yaw;

        result.eye    = Vec3::add(center, Vec3(dx, dy, dz));
        result.target = center;
        result.up     = Vec3(0.0f, 1.0f, 0.0f);

        return result;
    }

    // --- get_current_time ---
    float get_current_time() const { return current_time; }

    // --- is_at_end ---
    bool is_at_end() const {
        return current_time >= config.loop_end - APC_EPSILON;
    }
};

// =============================================================================
// CameraShakeConfig — Camera shake parameters
// =============================================================================
struct CameraShakeConfig {
    float intensity_decay_rate  = 3.0f;    // Exponential decay rate
    float frequency             = 15.0f;   // Shake frequency (Hz)
    uint32_t noise_octaves      = 2u;      // Number of noise octaves
    float directional_bias_x    = 0.0f;    // Bias shake direction (-1 to 1)
    float directional_bias_y    = 0.0f;
    float directional_bias_z    = 0.0f;
    float max_offset            = 2.0f;    // Maximum shake offset
    float max_rotation_offset   = 0.05f;   // Maximum rotation shake (radians)
};

// =============================================================================
// CameraShake — Intensity-based camera shake with exponential decay
// =============================================================================
struct CameraShake {
    CameraShakeConfig config;
    float current_intensity = 0.0f;  // Current shake intensity [0, 1]
    float elapsed_time     = 0.0f;  // Time since shake was triggered

    // --- trigger_shake ---
    void trigger_shake(const Vec3& position, float intensity, float duration) {
        (void)position; // Position could be used for spatial attenuation
        current_intensity = intensity;
        if (current_intensity > 1.0f) current_intensity = 1.0f;
        if (current_intensity < 0.0f) current_intensity = 0.0f;
        elapsed_time = 0.0f;
        // Store duration for decay calculation
        decay_duration_ = duration;
    }

    // --- update ---
    void update(float dt) {
        if (current_intensity <= 0.0f) return;

        elapsed_time += dt;

        // Exponential decay
        float decay = std::exp(-config.intensity_decay_rate * elapsed_time);
        // Also apply duration-based fade
        float duration_fade = 1.0f;
        if (decay_duration_ > 0.0f && elapsed_time > decay_duration_) {
            duration_fade = 0.0f;
        } else if (decay_duration_ > 0.0f) {
            float remaining = decay_duration_ - elapsed_time;
            float fade_zone = decay_duration_ * 0.3f; // Last 30% fades out
            if (remaining < fade_zone) {
                duration_fade = remaining / fade_zone;
            }
        }

        current_intensity *= decay;
        current_intensity *= duration_fade;

        if (current_intensity < 0.001f) {
            current_intensity = 0.0f;
        }
    }

    // --- get_offset: compute shake offset using deterministic sine-based noise ---
    Vec3 get_offset() const {
        if (current_intensity <= 0.0f) return Vec3(0.0f, 0.0f, 0.0f);

        float t = elapsed_time * config.frequency;

        // Multi-octave sine-based noise (deterministic, no random)
        float nx = 0.0f, ny = 0.0f, nz = 0.0f;
        float amp = 1.0f;

        for (uint32_t oct = 0; oct < config.noise_octaves; ++oct) {
            float freq = t * static_cast<float>(1u << oct);
            nx += amp * std::sin(freq * 1.7f + static_cast<float>(oct) * 2.3f);
            ny += amp * std::sin(freq * 2.3f + static_cast<float>(oct) * 3.7f);
            nz += amp * std::sin(freq * 1.1f + static_cast<float>(oct) * 5.1f);
            amp *= 0.5f;
        }

        // Normalize and apply intensity
        float scale = current_intensity * config.max_offset;
        nx *= scale;
        ny *= scale;
        nz *= scale;

        // Apply directional bias
        nx += config.directional_bias_x * current_intensity * config.max_offset;
        ny += config.directional_bias_y * current_intensity * config.max_offset;
        nz += config.directional_bias_z * current_intensity * config.max_offset;

        return Vec3(nx, ny, nz);
    }

    // --- get_rotation_offset: compute shake rotation ---
    Vec3 get_rotation_offset() const {
        if (current_intensity <= 0.0f) return Vec3(0.0f, 0.0f, 0.0f);

        float t = elapsed_time * config.frequency * 0.7f; // Slightly different frequency

        float rx = std::sin(t * 1.3f) * current_intensity * config.max_rotation_offset;
        float ry = std::sin(t * 1.7f + 1.0f) * current_intensity * config.max_rotation_offset;
        float rz = std::sin(t * 2.1f + 2.0f) * current_intensity * config.max_rotation_offset;

        return Vec3(rx, ry, rz);
    }

    // --- get_current_intensity ---
    float get_current_intensity() const { return current_intensity; }

    // --- is_active ---
    bool is_active() const { return current_intensity > 0.001f; }

    // --- reset ---
    void reset() {
        current_intensity = 0.0f;
        elapsed_time = 0.0f;
    }

private:
    float decay_duration_ = 1.0f;
};

// =============================================================================
// PhysicsToRenderSync — Frame-locked double-buffered transform sync
// =============================================================================
struct PhysicsToRenderSync {
    static constexpr uint32_t MAX_BODIES = 256u;

    enum class SyncPoint : uint8_t {
        PRE_PHYSICS   = 0,
        POST_PHYSICS  = 1,
        PRE_RENDER    = 2
    };

    // Double-buffered transforms
    struct BodyTransform {
        Vec3 position    = Vec3(0.0f, 0.0f, 0.0f);
        Quat orientation = Quat::identity();
        Vec3 scale       = Vec3(1.0f, 1.0f, 1.0f);
    };

    BodyTransform front_buffer[MAX_BODIES];  // Being read by renderer
    BodyTransform back_buffer[MAX_BODIES];   // Being written by physics

    uint32_t body_count      = 0u;
    SyncPoint last_sync      = SyncPoint::POST_PHYSICS;
    uint32_t frame_number    = 0u;

    // --- sync_from_physics: copy physics transforms to back buffer ---
    uint32_t sync_from_physics(const Vec3* positions, const Quat* orientations,
                                uint32_t count, SyncPoint sync)
    {
        uint32_t copied = 0u;
        for (uint32_t i = 0u; i < count && i < MAX_BODIES; ++i) {
            back_buffer[i].position    = positions[i];
            back_buffer[i].orientation = orientations[i];
            back_buffer[i].scale       = Vec3(1.0f, 1.0f, 1.0f);
            ++copied;
        }
        body_count = copied;
        last_sync  = sync;
        return copied;
    }

    // --- swap_buffers: flip front/back for render consumption ---
    void swap_buffers() {
        for (uint32_t i = 0u; i < body_count && i < MAX_BODIES; ++i) {
            front_buffer[i] = back_buffer[i];
        }
        ++frame_number;
    }

    // --- get_transforms: read-only access to front buffer ---
    const BodyTransform* get_transforms() const { return front_buffer; }

    // --- get_body_count ---
    uint32_t get_body_count() const { return body_count; }

    // --- get_frame_number ---
    uint32_t get_frame_number() const { return frame_number; }

    // --- get_last_sync ---
    SyncPoint get_last_sync() const { return last_sync; }

    // --- reset ---
    void reset() {
        body_count   = 0u;
        frame_number = 0u;
        last_sync    = SyncPoint::POST_PHYSICS;
        for (uint32_t i = 0u; i < MAX_BODIES; ++i) {
            front_buffer[i] = BodyTransform();
            back_buffer[i]  = BodyTransform();
        }
    }
};

} // namespace apc
