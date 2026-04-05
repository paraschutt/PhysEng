#pragma once
// ============================================================================
// DSPE Material System
// 7 surface types: dry grass, wet grass, artificial turf, hard surface,
//                  player boot (vs ball), player boot (vs player), goalpost
// Wet modifier: multiplies static_mu and kinetic_mu by wetness_factor [0.5, 1.0]
// ============================================================================
#include "fixed_point.h"
#include <cstdint>
#include <array>

namespace dspe {

// ---------------------------------------------------------------------------
// Material ID — 8-bit, ID 0 = default (dry grass)
// ---------------------------------------------------------------------------
using MaterialId = uint8_t;
static constexpr MaterialId MAT_DRY_GRASS     = 0;
static constexpr MaterialId MAT_WET_GRASS     = 1;
static constexpr MaterialId MAT_ARTIFICIAL    = 2;
static constexpr MaterialId MAT_HARD_SURFACE  = 3;
static constexpr MaterialId MAT_BOOT_VS_BALL  = 4;
static constexpr MaterialId MAT_BOOT_VS_PLAYER= 5;
static constexpr MaterialId MAT_GOALPOST      = 6;
static constexpr MaterialId MAT_COUNT         = 7;

// ---------------------------------------------------------------------------
// Material properties
// ---------------------------------------------------------------------------
struct Material {
    FpVel restitution;   // Coefficient of restitution [0, 1]
    FpVel static_mu;     // Static friction coefficient
    FpVel kinetic_mu;    // Kinetic friction coefficient
    FpVel rolling_mu;    // Rolling friction coefficient (0 = N/A)
    const char* name;
};

// ---------------------------------------------------------------------------
// Material database (values from revised brief Table in Section 9)
// ---------------------------------------------------------------------------
static constexpr std::array<Material, MAT_COUNT> k_materials = {{
    // MAT_DRY_GRASS
    { FpVel::from_float(0.80f), FpVel::from_float(0.70f),
      FpVel::from_float(0.60f), FpVel::from_float(0.018f), "Dry Grass" },
    // MAT_WET_GRASS
    { FpVel::from_float(0.75f), FpVel::from_float(0.45f),
      FpVel::from_float(0.35f), FpVel::from_float(0.012f), "Wet Grass" },
    // MAT_ARTIFICIAL (3G turf)
    { FpVel::from_float(0.82f), FpVel::from_float(0.65f),
      FpVel::from_float(0.55f), FpVel::from_float(0.020f), "Artificial Turf" },
    // MAT_HARD_SURFACE (indoor)
    { FpVel::from_float(0.95f), FpVel::from_float(0.50f),
      FpVel::from_float(0.40f), FpVel::from_float(0.010f), "Hard Surface" },
    // MAT_BOOT_VS_BALL
    { FpVel::from_float(0.50f), FpVel::from_float(0.80f),
      FpVel::from_float(0.70f), FpVel::from_float(0.000f), "Boot vs Ball" },
    // MAT_BOOT_VS_PLAYER
    { FpVel::from_float(0.30f), FpVel::from_float(0.55f),
      FpVel::from_float(0.45f), FpVel::from_float(0.000f), "Boot vs Player" },
    // MAT_GOALPOST
    { FpVel::from_float(0.60f), FpVel::from_float(0.30f),
      FpVel::from_float(0.25f), FpVel::from_float(0.000f), "Goalpost (steel)" },
}};

inline const Material& get_material(MaterialId id) {
    if (id >= MAT_COUNT) id = MAT_DRY_GRASS;
    return k_materials[id];
}

// Combined restitution for two materials: geometric mean
inline FpVel combined_restitution(MaterialId a, MaterialId b) {
    FpVel ea = get_material(a).restitution;
    FpVel eb = get_material(b).restitution;
    // sqrt(ea * eb), both in Q15.16
    FpVel product = ea * eb;
    return fp_sqrt(product);
}

// Combined friction for two materials: min (conservative)
inline FpVel combined_kinetic_mu(MaterialId a, MaterialId b) {
    FpVel fa = get_material(a).kinetic_mu;
    FpVel fb = get_material(b).kinetic_mu;
    return fa < fb ? fa : fb;
}

// ---------------------------------------------------------------------------
// Weather/surface state — set per stadium, interpolated over match
// ---------------------------------------------------------------------------
struct SurfaceState {
    FpVel wetness;        // [0.0, 1.0] where 1.0 = fully dry
    Vec3Vel wind_vel;     // Stadium wind vector (m/s)

    // Effective kinetic friction after wetness modifier
    // kinetic_mu_eff = kinetic_mu * lerp(0.5, 1.0, wetness)
    FpVel apply_wetness(FpVel kinetic_mu) const {
        // wetness_factor = 0.5 + 0.5 * wetness
        FpVel half     = FpVel::from_float(0.5f);
        FpVel factor   = half + (half * wetness);
        return kinetic_mu * factor;
    }
};

// needs math_types.h for Vec3Vel
#include "math_types.h"

} // namespace dspe
