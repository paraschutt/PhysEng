#pragma once
// =============================================================================
// VFX Bridge — Visual effects integration layer
// =============================================================================
//
// Provides:
//   - ScreenFlashConfig: screen flash parameters
//   - ScreenFlashVFX: screen flash effect with decay
//   - SoundTriggerEvent: sound trigger request
//   - VibrationEvent: controller vibration request
//   - VibrationPattern: vibration pattern enumeration
//   - HitStopIntegration: freeze-frame effect consuming hit-stop from game hooks
//   - TimeScaleIntegration: slow-motion effect consuming time-scale from game hooks
//   - VFXBridge: central VFX dispatch consuming GameHookOutput[]
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation (fixed-capacity arrays)
//   - Deterministic: same inputs produce same outputs
//   - C++17
//
// =============================================================================

#include "apc_render_types.h"
#include "apc_style/apc_game_hooks.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_math_common.h"
#include <cstdint>
#include <cmath>

namespace apc {

// =============================================================================
// ScreenFlashConfig — Screen flash parameters
// =============================================================================
struct ScreenFlashConfig {
    RenderColor color      = RenderColor::WHITE();
    float       intensity  = 1.0f;     // Peak intensity [0, 1]
    float       decay_rate = 4.0f;     // Exponential decay rate
};

// =============================================================================
// ScreenFlashVFX — Screen flash effect with decay
// =============================================================================
struct ScreenFlashVFX {
    ScreenFlashConfig config;
    float current_intensity = 0.0f;
    float elapsed_time     = 0.0f;
    bool  is_active        = false;

    // --- trigger ---
    void trigger(const RenderColor& color, float intensity) {
        config.color = color;
        current_intensity = intensity;
        if (current_intensity > 1.0f) current_intensity = 1.0f;
        if (current_intensity < 0.0f) current_intensity = 0.0f;
        elapsed_time = 0.0f;
        is_active = true;
    }

    // --- update ---
    void update(float dt) {
        if (!is_active) return;

        elapsed_time += dt;
        current_intensity *= std::exp(-config.decay_rate * dt);

        if (current_intensity < 0.001f) {
            current_intensity = 0.0f;
            is_active = false;
        }
    }

    // --- get_current_intensity ---
    float get_current_intensity() const { return current_intensity; }

    // --- get_current_color: return color modulated by intensity ---
    RenderColor get_current_color() const {
        return RenderColor(
            config.color.r * current_intensity,
            config.color.g * current_intensity,
            config.color.b * current_intensity,
            config.color.a * current_intensity
        );
    }

    // --- active check ---
    bool active() const { return is_active; }

    // --- reset ---
    void reset() {
        current_intensity = 0.0f;
        elapsed_time = 0.0f;
        is_active = false;
    }
};

// =============================================================================
// SoundTriggerEvent — Sound trigger request
// =============================================================================
struct SoundTriggerEvent {
    uint32_t sound_id  = 0u;
    Vec3     position  = Vec3(0.0f, 0.0f, 0.0f);
    float    volume    = 1.0f;     // [0, 1]
    uint32_t priority  = 0u;       // Higher = more important
};

// =============================================================================
// VibrationPattern — Controller vibration patterns
// =============================================================================
enum class VibrationPattern : uint8_t {
    NONE     = 0,
    PULSE    = 1,   // Single pulse
    RAMP     = 2,   // Ramp up then down
    CONSTANT = 3    // Constant vibration
};

// =============================================================================
// VibrationEvent — Controller vibration request
// =============================================================================
struct VibrationEvent {
    float           intensity   = 0.0f;  // [0, 1]
    uint32_t        duration_ms = 0u;
    VibrationPattern pattern     = VibrationPattern::NONE;

    // --- get_intensity_at_time: compute vibration intensity at a given time ---
    float get_intensity_at_time(float elapsed_ms) const {
        if (elapsed_ms >= static_cast<float>(duration_ms)) return 0.0f;

        switch (pattern) {
        case VibrationPattern::PULSE: {
            // Single pulse: ramp up to peak then drop
            float t = elapsed_ms / static_cast<float>(duration_ms);
            if (t < 0.2f) {
                return intensity * (t / 0.2f);
            }
            return intensity * (1.0f - (t - 0.2f) / 0.8f);
        }
        case VibrationPattern::RAMP: {
            // Ramp up then down (triangle)
            float t = elapsed_ms / static_cast<float>(duration_ms);
            if (t < 0.5f) {
                return intensity * (t * 2.0f);
            }
            return intensity * (2.0f - t * 2.0f);
        }
        case VibrationPattern::CONSTANT: {
            return intensity;
        }
        case VibrationPattern::NONE:
        default:
            return 0.0f;
        }
    }

    // --- is_active_at_time ---
    bool is_active_at_time(float elapsed_ms) const {
        return elapsed_ms < static_cast<float>(duration_ms) &&
               pattern != VibrationPattern::NONE;
    }
};

// =============================================================================
// HitStopIntegration — Freeze-frame effect
// =============================================================================
struct HitStopIntegration {
    float remaining_ms   = 0.0f;  // Remaining freeze duration
    bool  is_frozen      = false;

    // --- consume: consume hit-stop duration from accumulator ---
    void consume(float hit_stop_ms) {
        if (hit_stop_ms > remaining_ms) {
            remaining_ms = hit_stop_ms;
        }
        is_frozen = remaining_ms > 0.0f;
    }

    // --- update: tick down the freeze timer ---
    void update(float dt_ms) {
        if (remaining_ms > 0.0f) {
            remaining_ms -= dt_ms;
            if (remaining_ms < 0.0f) remaining_ms = 0.0f;
            is_frozen = remaining_ms > 0.0f;
        } else {
            is_frozen = false;
        }
    }

    // --- get_remaining_ms ---
    float get_remaining_ms() const { return remaining_ms; }

    // --- should_freeze_render: render should render the last frame ---
    bool should_freeze_render() const { return is_frozen; }

    // --- get_time_scale: return 0 during freeze, 1 otherwise ---
    float get_time_scale() const {
        return is_frozen ? 0.0f : 1.0f;
    }

    // --- reset ---
    void reset() {
        remaining_ms = 0.0f;
        is_frozen = false;
    }
};

// =============================================================================
// TimeScaleIntegration — Slow-motion effect
// =============================================================================
struct TimeScaleIntegration {
    float current_scale       = 1.0f;  // Current time scale factor
    float target_scale        = 1.0f;  // Target scale
    float remaining_duration  = 0.0f;  // Remaining slow-mo duration (ms)
    float blend_speed         = 5.0f;  // How fast to blend to target scale

    // --- consume: consume time-scale from accumulator ---
    void consume(float time_scale, float duration_ms) {
        if (time_scale < target_scale) {
            target_scale = time_scale;
            remaining_duration = duration_ms;
            // Immediately apply a step toward the target so the scale
            // changes without waiting for the first update() call.
            current_scale += (target_scale - current_scale) * 0.5f;
        }
    }

    // --- update: tick timer and blend scale ---
    void update(float dt_ms) {
        if (remaining_duration > 0.0f) {
            remaining_duration -= dt_ms;

            // Blend current_scale toward target_scale
            float blend = blend_speed * (dt_ms / 1000.0f);
            if (blend > 1.0f) blend = 1.0f;
            current_scale += (target_scale - current_scale) * blend;

            if (remaining_duration <= 0.0f) {
                remaining_duration = 0.0f;
                // Start recovering to normal speed
                target_scale = 1.0f;
            }
        } else {
            // Recover to normal speed
            float blend = blend_speed * (dt_ms / 1000.0f);
            if (blend > 1.0f) blend = 1.0f;
            current_scale += (1.0f - current_scale) * blend;

            if (std::abs(current_scale - 1.0f) < 0.001f) {
                current_scale = 1.0f;
                target_scale = 1.0f;
            }
        }
    }

    // --- get_time_scale ---
    float get_time_scale() const { return current_scale; }

    // --- is_slow_motion ---
    bool is_slow_motion() const { return current_scale < 0.99f; }

    // --- reset ---
    void reset() {
        current_scale = 1.0f;
        target_scale = 1.0f;
        remaining_duration = 0.0f;
    }
};

// =============================================================================
// VFXBridge — Central VFX dispatch layer
// =============================================================================
//
// Consumes GameHookOutput[] from the GameHookSystem/Accumulator and dispatches
// to the appropriate VFX subsystems: camera shake, screen flash, time scale,
// hit-stop, sound triggers, vibration events.
//
struct VFXBridge {
    static constexpr uint32_t MAX_SOUNDS    = 32u;
    static constexpr uint32_t MAX_VIBRATIONS = 16u;

    CameraShake         camera_shake;
    ScreenFlashVFX      screen_flash;
    HitStopIntegration  hit_stop;
    TimeScaleIntegration time_scale;

    SoundTriggerEvent   sound_queue[MAX_SOUNDS];
    uint32_t            sound_count = 0u;

    VibrationEvent      vibration_queue[MAX_VIBRATIONS];
    uint32_t            vibration_count = 0u;

    // --- dispatch: process all game hook outputs ---
    void dispatch(const GameHookOutput* outputs, uint32_t count) {
        for (uint32_t i = 0u; i < count; ++i) {
            const GameHookOutput& out = outputs[i];

            switch (out.type) {
            case HookEffectType::CAMERA_SHAKE:
                camera_shake.trigger_shake(out.position, out.impact_intensity,
                                           out.float_param);
                break;

            case HookEffectType::SCREEN_FLASH:
                // Use impact color: red for heavy, white for light
                if (out.impact_intensity > 0.7f) {
                    screen_flash.trigger(RenderColor::RED(), out.impact_intensity * 0.5f);
                } else if (out.impact_intensity > 0.3f) {
                    screen_flash.trigger(RenderColor::YELLOW(), out.impact_intensity * 0.3f);
                } else {
                    screen_flash.trigger(RenderColor::WHITE(), out.impact_intensity * 0.2f);
                }
                break;

            case HookEffectType::HIT_STOP:
                hit_stop.consume(out.float_param);
                break;

            case HookEffectType::TIME_SCALE:
                time_scale.consume(out.float_param, out.float_param2);
                break;

            case HookEffectType::SOUND_TRIGGER:
                if (sound_count < MAX_SOUNDS) {
                    SoundTriggerEvent& snd = sound_queue[sound_count++];
                    snd.sound_id = out.profile_id;
                    snd.position = out.position;
                    snd.volume   = out.impact_intensity;
                    snd.priority = static_cast<uint32_t>(out.impact_intensity * 100.0f);
                }
                break;

            case HookEffectType::VIBRATION:
                if (vibration_count < MAX_VIBRATIONS) {
                    VibrationEvent& vib = vibration_queue[vibration_count++];
                    vib.intensity   = out.impact_intensity;
                    vib.duration_ms = static_cast<uint32_t>(out.float_param);
                    vib.pattern     = VibrationPattern::PULSE;
                }
                break;

            default:
                break;
            }
        }
    }

    // --- update: tick all VFX subsystems ---
    void update(float dt) {
        float dt_ms = dt * 1000.0f;

        camera_shake.update(dt);
        screen_flash.update(dt);
        hit_stop.update(dt_ms);
        time_scale.update(dt_ms);

        // Consume one-shot queues (sounds and vibrations are one-shot)
        sound_count = 0u;
        vibration_count = 0u;
    }

    // --- get_effective_time_scale: combined time scale from hit-stop + time-scale ---
    float get_effective_time_scale() const {
        float hs = hit_stop.get_time_scale();
        float ts = time_scale.get_time_scale();
        return hs * ts; // If either is 0 (frozen), result is 0
    }

    // --- is_render_frozen: true if hit-stop is active ---
    bool is_render_frozen() const {
        return hit_stop.should_freeze_render();
    }

    // --- reset ---
    void reset() {
        camera_shake.reset();
        screen_flash.reset();
        hit_stop.reset();
        time_scale.reset();
        sound_count = 0u;
        vibration_count = 0u;
    }
};

} // namespace apc
