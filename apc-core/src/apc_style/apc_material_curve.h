#pragma once
// =============================================================================
// Material Curve — Parameterized lookup tables for non-linear physics response
// =============================================================================
//
// Provides piecewise-linear lookup tables that map an input scalar (e.g. impact
// speed, contact angle) to an output scalar (e.g. restitution coefficient,
// friction coefficient). These enable designers to author non-linear material
// responses without touching code:
//
//   - Velocity-dependent restitution: low-speed impacts absorb energy,
//     high-speed impacts bounce more (or less, depending on the curve).
//   - Angle-dependent friction: sliding friction varies with the angle of
//     contact relative to the surface tangent.
//   - Custom curves for any tunable parameter.
//
// Design:
//   - Header-only, apc:: namespace
//   - Fixed-size sample arrays (no dynamic allocation)
//   - Deterministic: linear interpolation with clamped input, no branching
//     on lookup within the valid range
//   - C++17
//
// =============================================================================

#include "apc_math/apc_math_common.h"
#include <cstdint>
#include <cmath>

namespace apc {

// =============================================================================
// Curve constants
// =============================================================================
static constexpr uint32_t APC_CURVE_SAMPLES = 8u;   // Samples per curve
static constexpr uint32_t APC_MAX_CURVES    = 16u;   // Max curves in registry

// =============================================================================
// MaterialCurve — Piecewise-linear scalar mapping
// =============================================================================
//
// Maps input values in [input_min, input_max] to output values via linear
// interpolation over APC_CURVE_SAMPLES equally-spaced control points.
// Values outside the range are clamped to the nearest endpoint sample.
//
// Usage:
//   MaterialCurve curve = MaterialCurve::make_velocity_restitution();
//   float e = curve.evaluate(impact_speed);  // returns restitution [0,1]
//
struct MaterialCurve {
    float samples[APC_CURVE_SAMPLES]; // Output values at each sample point
    float input_min;                  // Minimum input value (maps to samples[0])
    float input_max;                  // Maximum input value (maps to samples[7])
    uint8_t curve_id;                 // Registry index

    // -----------------------------------------------------------------
    // evaluate — Sample the curve at a given input value.
    // Linearly interpolates between adjacent samples. Clamps to range.
    // -----------------------------------------------------------------
    float evaluate(float x) const {
        float range = input_max - input_min;
        if (range <= 0.0f) return samples[0];

        float t = (x - input_min) / range;
        // Clamp to [0, 1]
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;

        // Map to sample index
        float idx = t * static_cast<float>(APC_CURVE_SAMPLES - 1);
        uint32_t i = static_cast<uint32_t>(idx);

        if (i >= APC_CURVE_SAMPLES - 1) {
            return samples[APC_CURVE_SAMPLES - 1];
        }

        float frac = idx - static_cast<float>(i);
        return samples[i] + (samples[i + 1] - samples[i]) * frac;
    }

    // -----------------------------------------------------------------
    // Factory: constant value across entire range
    // -----------------------------------------------------------------
    static MaterialCurve make_constant(float value, float x_min = 0.0f,
                                        float x_max = 1.0f) {
        MaterialCurve c;
        c.input_min = x_min;
        c.input_max = x_max;
        c.curve_id = 0;
        for (uint32_t i = 0; i < APC_CURVE_SAMPLES; ++i) {
            c.samples[i] = value;
        }
        return c;
    }

    // -----------------------------------------------------------------
    // Factory: linear ramp from y_start to y_end
    // -----------------------------------------------------------------
    static MaterialCurve make_linear(float y_start, float y_end,
                                      float x_min = 0.0f, float x_max = 1.0f) {
        MaterialCurve c;
        c.input_min = x_min;
        c.input_max = x_max;
        c.curve_id = 0;
        for (uint32_t i = 0; i < APC_CURVE_SAMPLES; ++i) {
            float t = static_cast<float>(i) / static_cast<float>(APC_CURVE_SAMPLES - 1);
            c.samples[i] = y_start + (y_end - y_start) * t;
        }
        return c;
    }

    // -----------------------------------------------------------------
    // Factory: velocity-dependent restitution curve
    //
    // Models the common game-feel pattern where:
    //   - Very low speed impacts (< low_speed) have zero restitution (dead feel)
    //   - Medium speeds ramp up to peak restitution
    //   - Very high speeds (> high_speed) reduce restitution (energy absorption)
    //
    // Parameters:
    //   low_speed    — Speed below which restitution is zero
    //   peak_speed   — Speed at which restitution peaks
    //   high_speed   — Speed above which restitution falls off
    //   peak_value   — Maximum restitution (typically 0.3–0.8)
    //   high_value   — Restitution at high_speed (typically 0.1–0.3)
    // -----------------------------------------------------------------
    static MaterialCurve make_velocity_restitution(
        float low_speed = 1.0f,
        float peak_speed = 5.0f,
        float high_speed = 20.0f,
        float peak_value = 0.6f,
        float high_value = 0.2f)
    {
        MaterialCurve c;
        c.input_min = 0.0f;
        c.input_max = high_speed;
        c.curve_id = 0;

        for (uint32_t i = 0; i < APC_CURVE_SAMPLES; ++i) {
            float t = static_cast<float>(i) / static_cast<float>(APC_CURVE_SAMPLES - 1);
            float speed = t * high_speed;

            if (speed < low_speed) {
                c.samples[i] = 0.0f;
            } else if (speed < peak_speed) {
                float local_t = (speed - low_speed) / (peak_speed - low_speed);
                c.samples[i] = peak_value * local_t;
            } else {
                float local_t = (speed - peak_speed) / (high_speed - peak_speed);
                c.samples[i] = peak_value + (high_value - peak_value) * local_t;
            }
        }
        return c;
    }

    // -----------------------------------------------------------------
    // Factory: angle-dependent friction curve
    //
    // Maps the absolute angle between the impact direction and surface
    // normal to a friction coefficient. This allows steep-angle impacts
    // to have less friction (glancing blows slide off) while head-on
    // impacts have maximum grip.
    //
    // Parameters:
    //   x_min / x_max — Angle range in radians (0 = head-on, π/2 = grazing)
    //   head_on_friction — Friction at 0 degrees (direct impact)
    //   grazing_friction — Friction at 90 degrees (glancing impact)
    // -----------------------------------------------------------------
    static MaterialCurve make_angle_friction(
        float x_min = 0.0f,
        float x_max = 1.5708f,  // π/2
        float head_on_friction = 0.8f,
        float grazing_friction = 0.2f)
    {
        return make_linear(head_on_friction, grazing_friction, x_min, x_max);
    }

    // -----------------------------------------------------------------
    // Factory: step function (useful for threshold-based responses)
    // -----------------------------------------------------------------
    static MaterialCurve make_step(float threshold, float low_val, float high_val,
                                    float x_min = 0.0f, float x_max = 1.0f) {
        MaterialCurve c;
        c.input_min = x_min;
        c.input_max = x_max;
        c.curve_id = 0;
        for (uint32_t i = 0; i < APC_CURVE_SAMPLES; ++i) {
            float t = static_cast<float>(i) / static_cast<float>(APC_CURVE_SAMPLES - 1);
            float x = x_min + (x_max - x_min) * t;
            c.samples[i] = (x < threshold) ? low_val : high_val;
        }
        return c;
    }
};

// =============================================================================
// MaterialCurveRegistry — Collection of named curves for profile resolution
// =============================================================================
struct MaterialCurveRegistry {
    MaterialCurve curves[APC_MAX_CURVES];
    uint32_t count = 0;

    // -----------------------------------------------------------------
    // add — Register a curve. Returns its ID (index).
    // Sets curve_id on the curve object.
    // -----------------------------------------------------------------
    uint8_t add(MaterialCurve curve) {
        if (count >= APC_MAX_CURVES) return 0xFF;
        curve.curve_id = static_cast<uint8_t>(count);
        curves[count] = curve;
        ++count;
        return curve.curve_id;
    }

    // -----------------------------------------------------------------
    // get — Retrieve a curve by ID. Returns nullptr if invalid.
    // -----------------------------------------------------------------
    const MaterialCurve* get(uint8_t id) const {
        if (id >= count) return nullptr;
        return &curves[id];
    }

    // -----------------------------------------------------------------
    // evaluate — Sample a curve by ID. Returns fallback if invalid.
    // -----------------------------------------------------------------
    float evaluate(uint8_t id, float x, float fallback = 0.0f) const {
        const MaterialCurve* c = get(id);
        if (!c) return fallback;
        return c->evaluate(x);
    }

    // -----------------------------------------------------------------
    // setup_defaults — Populate with the standard set of game-feel curves.
    // Returns the number of curves added.
    //
    // Default curves:
    //   0: Velocity-dependent restitution (standard)
    //   1: Angle-dependent friction (standard)
    //   2: Velocity-dependent restitution (heavy — for big bodies)
    //   3: Angle-dependent friction (sticky — for ground contact)
    //   4: Momentum transfer multiplier (linear ramp)
    // -----------------------------------------------------------------
    uint32_t setup_defaults() {
        count = 0;

        // 0: Standard velocity restitution
        add(MaterialCurve::make_velocity_restitution(1.0f, 5.0f, 20.0f, 0.6f, 0.2f));

        // 1: Standard angle friction
        add(MaterialCurve::make_angle_friction(0.0f, 1.5708f, 0.8f, 0.2f));

        // 2: Heavy body restitution (lower peak, earlier falloff)
        add(MaterialCurve::make_velocity_restitution(0.5f, 3.0f, 15.0f, 0.3f, 0.1f));

        // 3: Sticky ground friction (high head-on, maintains grip at angle)
        add(MaterialCurve::make_angle_friction(0.0f, 1.5708f, 1.0f, 0.5f));

        // 4: Linear momentum transfer ramp (0.5 at min, 1.5 at max)
        add(MaterialCurve::make_linear(0.5f, 1.5f, 0.0f, 30.0f));

        return count;
    }
};

} // namespace apc
