#pragma once
// =============================================================================
// Game Hooks — Impact event processing for game-feel effects
// =============================================================================
//
// Processes ImpactEvents from the StylizedSolver and produces game-side
// effect requests (hit-stop, time-scale, camera shake, blend mode overrides).
//
// The game integration layer reads GameHookOutputs each frame and applies
// the requested effects:
//   - Hit-stop: pause gameplay for N ms (impact "freeze frame")
//   - Time-scale: run physics at reduced speed for N ms (slow-motion)
//   - Camera shake: request camera shake of given intensity
//   - Blend mode override: transition skeleton bones to PHYSICS_DRIVEN
//     on high impacts (auto-ragdoll)
//   - Sound trigger: request impact sound with given intensity
//
// Architecture:
//   1. StylizedSolver produces ImpactEvents during solve
//   2. GameHookSystem::process_impacts() evaluates events against
//      profile thresholds and produces GameHookOutputs
//   3. Game code reads outputs and applies effects via its own systems
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation (fixed-capacity buffers)
//   - Deterministic: same events always produce same outputs
//   - C++17
//
// =============================================================================

#include "apc_style/apc_stylized_solver.h"
#include "apc_style/apc_impact_profile.h"
#include "apc_skeleton/apc_skeleton_types.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_math_common.h"
#include <cstdint>
#include <cmath>

namespace apc {

// =============================================================================
// HookEffectType — Types of game feel effects
// =============================================================================
enum class HookEffectType : uint8_t {
    NONE            = 0,
    HIT_STOP        = 1,   // Freeze frame on impact
    TIME_SCALE      = 2,   // Slow-motion window
    CAMERA_SHAKE    = 3,   // Camera shake request
    BLEND_OVERRIDE  = 4,   // Change bone blend mode
    SOUND_TRIGGER   = 5,   // Impact sound
    SCREEN_FLASH    = 6,   // Brief screen flash (cinematic)
    VIBRATION       =7    // Controller vibration
};

// =============================================================================
// BlendOverrideRequest — Request to change a bone's blend mode
// =============================================================================
struct BlendOverrideRequest {
    uint32_t skeleton_id;        // Target skeleton
    uint32_t bone_index;         // Target bone (0xFFFFFFFF = all bones)
    PhysicsBlendMode new_mode;   // New blend mode
    float stiffness;             // Spring stiffness (for BLENDED mode)
    float damping;               // Damping (for BLENDED mode)
    float max_deviation;         // Max deviation (for BLENDED mode)
    float duration_ms;           // How long to hold this override (-1 = permanent)
};

// =============================================================================
// GameHookOutput — Single game-feel effect request
// =============================================================================
struct GameHookOutput {
    HookEffectType type = HookEffectType::NONE;
    float float_param = 0.0f;          // Duration or intensity
    float float_param2 = 0.0f;         // Secondary parameter
    uint32_t target_body = 0;          // Target body index
    Vec3 position = Vec3(0, 0, 0);    // World position for effects
    float impact_intensity = 0.0f;     // Normalized intensity [0, 1]
    uint16_t profile_id = 0;           // Source profile
    BlendOverrideRequest blend_override; // Blend override data (if type == BLEND_OVERRIDE)

    void reset() {
        type = HookEffectType::NONE;
        float_param = 0.0f;
        float_param2 = 0.0f;
        target_body = 0;
        position = Vec3(0, 0, 0);
        impact_intensity = 0.0f;
        profile_id = 0;
        blend_override = BlendOverrideRequest();
    }
};

// =============================================================================
// GameHookConfig — Configuration for the hook system
// =============================================================================
struct GameHookConfig {
    bool enable_hit_stop = true;
    bool enable_time_scale = true;
    bool enable_camera_shake = true;
    bool enable_blend_overrides = true;
    bool enable_sound_triggers = true;
    bool enable_screen_flash = false;     // Usually cinematic only

    float max_hit_stop_ms = 120.0f;       // Cap hit-stop duration
    float max_time_scale_duration_ms = 500.0f;
    float max_camera_shake = 1.0f;

    float auto_ragdoll_threshold = 0.8f;  // Impact intensity above which
                                          // bones auto-switch to ragdoll
    float auto_ragdoll_stiffness = 0.0f;  // Spring stiffness for recovery
    float auto_ragdoll_damping = 5.0f;    // Damping for recovery
    float auto_ragdoll_max_dev = 0.0f;    // Max deviation (0 = pure physics)
    float auto_ragdoll_duration_ms = 2000.0f; // How long to stay in ragdoll

    uint32_t max_outputs_per_frame = 32u;
    float cooldown_between_hooks_ms = 50.0f; // Minimum time between effects
};

// =============================================================================
// GameHookSystem — Processes impacts into game-feel effect requests
// =============================================================================
class GameHookSystem {
public:
    static constexpr uint32_t MAX_OUTPUTS = 64u;

    // -----------------------------------------------------------------
    // Constructor
    // -----------------------------------------------------------------
    GameHookSystem() = default;
    explicit GameHookSystem(const GameHookConfig& config) : config_(config) {}

    // -----------------------------------------------------------------
    // set_config — Update configuration
    // -----------------------------------------------------------------
    void set_config(const GameHookConfig& config) { config_ = config; }

    // -----------------------------------------------------------------
    // process_impacts — Evaluate impact events and generate hook outputs.
    //
    // For each ImpactEvent:
    //   1. Classify impact severity (light / heavy / critical)
    //   2. Look up profile thresholds
    //   3. Generate appropriate game hook outputs
    //
    // Returns the number of outputs generated.
    // -----------------------------------------------------------------
    uint32_t process_impacts(
        const ImpactEvent* events,
        uint32_t event_count,
        const ProfileRegistry* profiles,
        GameHookOutput* outputs,
        uint32_t max_outputs) const
    {
        uint32_t out_count = 0;

        for (uint32_t e = 0; e < event_count; ++e) {
            const ImpactEvent& evt = events[e];

            // Get the resolved profile
            const ImpactStyleProfile* prof = nullptr;
            if (profiles) {
                prof = profiles->get(evt.resolved_profile_id);
            }
            if (!prof) continue;

            // Classify impact severity
            float intensity = classify_impact(evt, *prof);
            if (intensity < 0.01f) continue; // Too weak to generate effects

            // Check cooldown
            if (evt.timestamp < last_hook_time_ + config_.cooldown_between_hooks_ms * 0.001f) {
                continue;
            }

            // Generate effects based on severity and profile
            if (out_count < max_outputs) {
                // --- Hit Stop ---
                if (config_.enable_hit_stop &&
                    prof->hit_stop_duration_ms > 0.0f &&
                    intensity > 0.3f)
                {
                    GameHookOutput& out = outputs[out_count++];
                    out.reset();
                    out.type = HookEffectType::HIT_STOP;
                    out.float_param = prof->hit_stop_duration_ms * intensity;
                    // Cap to max
                    if (out.float_param > config_.max_hit_stop_ms) {
                        out.float_param = config_.max_hit_stop_ms;
                    }
                    out.position = evt.contact_point;
                    out.impact_intensity = intensity;
                    out.profile_id = prof->profile_id;
                    out.target_body = evt.body_b; // Receiver gets hit-stop
                }

                // --- Camera Shake ---
                if (config_.enable_camera_shake &&
                    prof->camera_shake_intensity > 0.0f &&
                    intensity > 0.2f)
                {
                    GameHookOutput& out = outputs[out_count++];
                    out.reset();
                    out.type = HookEffectType::CAMERA_SHAKE;
                    out.float_param = prof->camera_shake_intensity * intensity;
                    if (out.float_param > config_.max_camera_shake) {
                        out.float_param = config_.max_camera_shake;
                    }
                    out.position = evt.contact_point;
                    out.impact_intensity = intensity;
                    out.profile_id = prof->profile_id;
                }

                // --- Time Scale ---
                if (config_.enable_time_scale &&
                    prof->time_scale_on_impact < 0.99f &&
                    prof->time_scale_duration_ms > 0.0f &&
                    intensity > 0.5f)
                {
                    GameHookOutput& out = outputs[out_count++];
                    out.reset();
                    out.type = HookEffectType::TIME_SCALE;
                    out.float_param = prof->time_scale_on_impact; // Scale factor
                    out.float_param2 = prof->time_scale_duration_ms * intensity;
                    if (out.float_param2 > config_.max_time_scale_duration_ms) {
                        out.float_param2 = config_.max_time_scale_duration_ms;
                    }
                    out.impact_intensity = intensity;
                    out.profile_id = prof->profile_id;
                }

                // --- Sound Trigger ---
                if (config_.enable_sound_triggers && intensity > 0.1f) {
                    GameHookOutput& out = outputs[out_count++];
                    out.reset();
                    out.type = HookEffectType::SOUND_TRIGGER;
                    out.float_param = intensity; // Volume/intensity
                    out.position = evt.contact_point;
                    out.impact_intensity = intensity;
                    out.profile_id = prof->profile_id;
                }

                // --- Blend Override (Auto-Ragdoll) ---
                if (config_.enable_blend_overrides &&
                    intensity > config_.auto_ragdoll_threshold)
                {
                    GameHookOutput& out = outputs[out_count++];
                    out.reset();
                    out.type = HookEffectType::BLEND_OVERRIDE;
                    out.float_param = config_.auto_ragdoll_duration_ms;
                    out.target_body = evt.body_b; // Receiver goes ragdoll
                    out.impact_intensity = intensity;
                    out.profile_id = prof->profile_id;
                    out.position = evt.contact_point;

                    out.blend_override.skeleton_id = 0; // Default
                    out.blend_override.bone_index = 0xFFFFFFFF; // All bones
                    out.blend_override.new_mode = PhysicsBlendMode::PHYSICS_DRIVEN;
                    out.blend_override.stiffness = config_.auto_ragdoll_stiffness;
                    out.blend_override.damping = config_.auto_ragdoll_damping;
                    out.blend_override.max_deviation = config_.auto_ragdoll_max_dev;
                    out.blend_override.duration_ms = config_.auto_ragdoll_duration_ms;
                }
            }
        }

        // Update cooldown timer
        // (In production, this would use actual frame time from the game)
        if (out_count > 0) {
            last_hook_time_ = events[0].timestamp;
        }

        return out_count;
    }

    // -----------------------------------------------------------------
    // classify_impact — Classify impact severity on [0, 1] scale.
    //
    // Maps the impact force through the profile's threshold bands:
    //   light_impact_threshold ... heavy_impact_threshold ... critical
    //   0.0  ................... 0.5  ........................ 1.0
    // -----------------------------------------------------------------
    static float classify_impact(const ImpactEvent& evt,
                                  const ImpactStyleProfile& prof)
    {
        float force = evt.impact_force;

        // Use relative speed as a secondary metric if force is zero
        if (force < APC_EPSILON) {
            force = evt.relative_speed * 10.0f; // Approximate force from speed
        }

        // Below light threshold: no effect
        if (force < prof.light_impact_threshold) return 0.0f;

        // Between light and heavy: ramp from 0 to 0.5
        if (force < prof.heavy_impact_threshold) {
            float range = prof.heavy_impact_threshold - prof.light_impact_threshold;
            if (range < APC_EPSILON) return 0.5f;
            return 0.5f * (force - prof.light_impact_threshold) / range;
        }

        // Between heavy and critical: ramp from 0.5 to 1.0
        if (force < prof.critical_impact_threshold) {
            float range = prof.critical_impact_threshold - prof.heavy_impact_threshold;
            if (range < APC_EPSILON) return 1.0f;
            return 0.5f + 0.5f * (force - prof.heavy_impact_threshold) / range;
        }

        // Above critical: full intensity (cap at 1.0)
        return 1.0f;
    }

    // -----------------------------------------------------------------
    // blend_severity_to_duration — Map impact severity to effect duration.
    // Light impacts get short durations, critical impacts get max.
    // -----------------------------------------------------------------
    static float blend_severity_to_duration(float severity, float base_duration,
                                             float max_multiplier = 2.0f)
    {
        float multiplier = 1.0f + severity * (max_multiplier - 1.0f);
        return base_duration * multiplier;
    }

private:
    GameHookConfig config_;
    mutable float last_hook_time_ = -1.0f; // No cooldown on first frame
};

// =============================================================================
// GameHookAccumulator — Accumulates hook outputs over multiple frames
// =============================================================================
//
// Manages active effects that span multiple frames (time-scale, blend overrides).
// The game code queries this each frame to determine current active effects.
//
class GameHookAccumulator {
public:
    static constexpr uint32_t MAX_ACTIVE = 16u;

    // -----------------------------------------------------------------
    // update — Process new outputs and tick active effects.
    //
    // new_outputs: freshly generated hook outputs from process_impacts()
    // new_count: number of new outputs
    // frame_dt: time since last frame (seconds)
    // -----------------------------------------------------------------
    void update(const GameHookOutput* new_outputs, uint32_t new_count,
                float frame_dt)
    {
        // Add new effects (instant effects like hit-stop and sound are
        // one-shot and don't need to be tracked)
        for (uint32_t i = 0; i < new_count; ++i) {
            const GameHookOutput& out = new_outputs[i];

            switch (out.type) {
            case HookEffectType::HIT_STOP:
                active_hit_stop_ms_ = out.float_param;
                break;

            case HookEffectType::TIME_SCALE:
                if (active_count_ < MAX_ACTIVE) {
                    active_[active_count_] = out;
                    active_[active_count_].float_param2 = out.float_param2;
                    ++active_count_;
                }
                break;

            case HookEffectType::CAMERA_SHAKE:
                // Camera shake decays, store the initial intensity
                camera_shake_intensity_ = std::max(camera_shake_intensity_,
                    out.float_param);
                camera_shake_decay_ = 1.0f; // Start at full
                break;

            case HookEffectType::BLEND_OVERRIDE:
                if (active_count_ < MAX_ACTIVE) {
                    active_[active_count_] = out;
                    ++active_count_;
                }
                break;

            default:
                // One-shot effects (SOUND, SCREEN_FLASH) — consumed immediately
                one_shot_count_ = 0;
                if (one_shot_count_ < MAX_ACTIVE) {
                    one_shots_[one_shot_count_++] = out;
                }
                break;
            }
        }

        // Tick active effects
        // Hit-stop: consume by frame time
        if (active_hit_stop_ms_ > 0.0f) {
            active_hit_stop_ms_ -= frame_dt * 1000.0f;
            if (active_hit_stop_ms_ < 0.0f) active_hit_stop_ms_ = 0.0f;
        }

        // Camera shake: decay
        if (camera_shake_intensity_ > 0.0f) {
            camera_shake_decay_ -= frame_dt * 3.0f; // ~0.33s decay
            if (camera_shake_decay_ < 0.0f) {
                camera_shake_decay_ = 0.0f;
                camera_shake_intensity_ = 0.0f;
            }
        }

        // Time-scale and blend overrides: tick durations
        for (uint32_t i = 0; i < active_count_; ) {
            active_[i].float_param2 -= frame_dt * 1000.0f;
            if (active_[i].float_param2 <= 0.0f) {
                // Expired — remove by swapping with last
                active_[i] = active_[active_count_ - 1];
                --active_count_;
            } else {
                ++i;
            }
        }
    }

    // -----------------------------------------------------------------
    // get_hit_stop_ms — Current hit-stop duration remaining (0 = none)
    // -----------------------------------------------------------------
    float get_hit_stop_ms() const { return active_hit_stop_ms_; }
    bool has_hit_stop() const { return active_hit_stop_ms_ > 0.0f; }

    // -----------------------------------------------------------------
    // get_time_scale — Current time scale factor (1.0 = normal)
    // -----------------------------------------------------------------
    float get_time_scale() const {
        float scale = 1.0f;
        for (uint32_t i = 0; i < active_count_; ++i) {
            if (active_[i].type == HookEffectType::TIME_SCALE) {
                scale = std::min(scale, active_[i].float_param);
            }
        }
        return scale;
    }

    // -----------------------------------------------------------------
    // get_camera_shake — Current camera shake intensity
    // -----------------------------------------------------------------
    float get_camera_shake() const {
        return camera_shake_intensity_ * camera_shake_decay_;
    }

    // -----------------------------------------------------------------
    // get_active_blend_overrides — Get all active blend mode overrides
    // -----------------------------------------------------------------
    uint32_t get_active_blend_overrides(BlendOverrideRequest* out,
                                         uint32_t max_count) const
    {
        uint32_t count = 0;
        for (uint32_t i = 0; i < active_count_ && count < max_count; ++i) {
            if (active_[i].type == HookEffectType::BLEND_OVERRIDE) {
                out[count++] = active_[i].blend_override;
            }
        }
        return count;
    }

    // -----------------------------------------------------------------
    // get_one_shot_effects — Get effects that were fired this frame
    // -----------------------------------------------------------------
    uint32_t get_one_shot_effects(GameHookOutput* out,
                                   uint32_t max_count) const
    {
        uint32_t count = 0;
        for (uint32_t i = 0; i < one_shot_count_ && count < max_count; ++i) {
            out[count++] = one_shots_[i];
        }
        return count;
    }

    // -----------------------------------------------------------------
    // reset — Clear all active effects
    // -----------------------------------------------------------------
    void reset() {
        active_hit_stop_ms_ = 0.0f;
        camera_shake_intensity_ = 0.0f;
        camera_shake_decay_ = 0.0f;
        active_count_ = 0;
        one_shot_count_ = 0;
    }

private:
    float active_hit_stop_ms_ = 0.0f;
    float camera_shake_intensity_ = 0.0f;
    float camera_shake_decay_ = 0.0f;
    GameHookOutput active_[MAX_ACTIVE];
    uint32_t active_count_ = 0;
    GameHookOutput one_shots_[MAX_ACTIVE];
    uint32_t one_shot_count_ = 0;
};

} // namespace apc
