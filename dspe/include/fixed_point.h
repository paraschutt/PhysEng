#pragma once
// ============================================================================
// DSPE Fixed-Point Arithmetic Library
// Formats: Q24.8 (positions), Q15.16 (velocities/forces), Q8.24 (angular)
// Rule: ALL multiplications widen to int64_t before computing.
//       Compile with: -mfpmath=sse -msse2 -ffloat-store -ffp-contract=off
// ============================================================================
#include <cstdint>
#include <cstdlib>
#include <cassert>
#include <algorithm>
#include <type_traits>

namespace dspe {

// ---------------------------------------------------------------------------
// FixedPoint<FRAC> — signed Q(31-FRAC).FRAC in int32_t storage
// ---------------------------------------------------------------------------
template<int FRAC>
struct FixedPoint {
    static_assert(FRAC >= 0 && FRAC <= 30, "FRAC must be in [0,30]");

    static constexpr int   FRAC_BITS = FRAC;
    static constexpr int32_t ONE     = int32_t(1) << FRAC;
    static constexpr int32_t HALF    = ONE >> 1;

    int32_t raw{0};

    // --- Construction -------------------------------------------------------
    constexpr FixedPoint() = default;
    constexpr explicit FixedPoint(int32_t raw_val) : raw(raw_val) {}

    // From float — only for initialisation/test code, not hot path
    static constexpr FixedPoint from_float(float v) {
        return FixedPoint{static_cast<int32_t>(v * ONE)};
    }
    static constexpr FixedPoint from_int(int32_t v) {
        return FixedPoint{v << FRAC};
    }
    float to_float() const { return static_cast<float>(raw) / ONE; }
    int32_t to_int() const { return raw >> FRAC; }

    // --- Arithmetic (all via int64_t) ---------------------------------------
    constexpr FixedPoint operator+(FixedPoint o) const {
        return FixedPoint{raw + o.raw};
    }
    constexpr FixedPoint operator-(FixedPoint o) const {
        return FixedPoint{raw - o.raw};
    }
    constexpr FixedPoint operator-() const {
        return FixedPoint{-raw};
    }
    // Multiply: widen to int64, shift back
    constexpr FixedPoint operator*(FixedPoint o) const {
        int64_t prod = static_cast<int64_t>(raw) * static_cast<int64_t>(o.raw);
        return FixedPoint{static_cast<int32_t>(prod >> FRAC)};
    }
    // Divide: widen, shift numerator, divide
    constexpr FixedPoint operator/(FixedPoint o) const {
        int64_t num = (static_cast<int64_t>(raw) << FRAC);
        return FixedPoint{static_cast<int32_t>(num / static_cast<int64_t>(o.raw))};
    }
    // Scalar multiply (integer)
    constexpr FixedPoint operator*(int32_t s) const {
        return FixedPoint{static_cast<int32_t>(
            static_cast<int64_t>(raw) * static_cast<int64_t>(s))};
    }
    constexpr FixedPoint operator>>(int s) const { return FixedPoint{raw >> s}; }
    constexpr FixedPoint operator<<(int s) const { return FixedPoint{raw << s}; }

    // --- Comparison ---------------------------------------------------------
    constexpr bool operator==(FixedPoint o) const { return raw == o.raw; }
    constexpr bool operator!=(FixedPoint o) const { return raw != o.raw; }
    constexpr bool operator< (FixedPoint o) const { return raw <  o.raw; }
    constexpr bool operator<=(FixedPoint o) const { return raw <= o.raw; }
    constexpr bool operator> (FixedPoint o) const { return raw >  o.raw; }
    constexpr bool operator>=(FixedPoint o) const { return raw >= o.raw; }

    // --- Compound assignment ------------------------------------------------
    FixedPoint& operator+=(FixedPoint o) { raw += o.raw; return *this; }
    FixedPoint& operator-=(FixedPoint o) { raw -= o.raw; return *this; }
    FixedPoint& operator*=(FixedPoint o) { *this = *this * o; return *this; }
    FixedPoint& operator/=(FixedPoint o) { *this = *this / o; return *this; }

    // --- Utility ------------------------------------------------------------
    constexpr FixedPoint abs() const {
        return FixedPoint{raw < 0 ? -raw : raw};
    }

    static constexpr FixedPoint zero() { return FixedPoint{0}; }
    static constexpr FixedPoint one()  { return FixedPoint{ONE}; }
    static constexpr FixedPoint max_val() { return FixedPoint{INT32_MAX}; }
    static constexpr FixedPoint min_val() { return FixedPoint{INT32_MIN}; }
};

// ---------------------------------------------------------------------------
// Cross-format multiply: Fp<A> * Fp<B> -> Fp<C>
// Used when mixing formats (e.g. position * velocity)
// ---------------------------------------------------------------------------
template<int FRAC_OUT, int FRAC_A, int FRAC_B>
constexpr FixedPoint<FRAC_OUT> fp_mul(FixedPoint<FRAC_A> a, FixedPoint<FRAC_B> b) {
    int64_t prod = static_cast<int64_t>(a.raw) * static_cast<int64_t>(b.raw);
    constexpr int shift = FRAC_A + FRAC_B - FRAC_OUT;
    return FixedPoint<FRAC_OUT>{static_cast<int32_t>(prod >> shift)};
}

// ---------------------------------------------------------------------------
// Integer sqrt using Newton-Raphson (deterministic, no libm)
// Input/output in same fixed-point format
// ---------------------------------------------------------------------------
template<int FRAC>
FixedPoint<FRAC> fp_sqrt(FixedPoint<FRAC> x) {
    if (x.raw <= 0) return FixedPoint<FRAC>{0};
    // Work in int64 for precision
    int64_t val = static_cast<int64_t>(x.raw) << FRAC; // shift up by FRAC
    int64_t r = val;
    // Initial guess: highest set bit
    int64_t g = 1;
    while (g * g < r) g <<= 1;
    // Newton-Raphson iterations (5 is enough for 32-bit)
    for (int i = 0; i < 8; ++i) {
        if (g == 0) break;
        int64_t next = (g + r / g) >> 1;
        if (next >= g) break;
        g = next;
    }
    return FixedPoint<FRAC>{static_cast<int32_t>(g)};
}

// Clamp
template<int FRAC>
constexpr FixedPoint<FRAC> fp_clamp(FixedPoint<FRAC> v,
                                     FixedPoint<FRAC> lo,
                                     FixedPoint<FRAC> hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// ---------------------------------------------------------------------------
// Type aliases (per domain as specified in brief)
// ---------------------------------------------------------------------------
using FpPos = FixedPoint<8>;   // Q24.8  — world positions (range ±16.7M m)
using FpVel = FixedPoint<16>;  // Q15.16 — velocities, forces, accelerations
using FpAng = FixedPoint<24>;  // Q8.24  — angular velocity (range ±127 rad/s)

// Convenience constants
inline FpPos fp_pos(float f) { return FpPos::from_float(f); }
inline FpVel fp_vel(float f) { return FpVel::from_float(f); }
inline FpAng fp_ang(float f) { return FpAng::from_float(f); }

// ---------------------------------------------------------------------------
// LUT-based deterministic sin/cos (256-entry, Q1.30 precision)
// MUST be used instead of std::sin/std::cos in the simulation loop
// ---------------------------------------------------------------------------
namespace lut {

// 256-entry quarter-wave table for sin, range [0, pi/2]
// Stored as Q1.30 (multiply result by desired scale)
// Generated offline: sin_table[i] = round(sin(i * PI/2 / 255) * (1<<30))
// Full 256 values inline for zero runtime dependency
extern const int32_t sin_q30[256];

// sin(angle_q16) where angle_q16 is angle in [0, 2*pi) as Q16 (0=0, 65536=2pi)
int32_t sin_fp(int32_t angle_q16);
int32_t cos_fp(int32_t angle_q16);

} // namespace lut

} // namespace dspe
