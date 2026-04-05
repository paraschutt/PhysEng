#pragma once
// ============================================================================
// DSPE Math Types — Vec3 and Quaternion over fixed-point formats
// ============================================================================
#include "fixed_point.h"
#include <cstdint>

namespace dspe {

// ---------------------------------------------------------------------------
// Vec3<Fp> — generic 3-component vector
// ---------------------------------------------------------------------------
template<typename Fp>
struct Vec3 {
    Fp x{}, y{}, z{};

    constexpr Vec3() = default;
    constexpr Vec3(Fp x_, Fp y_, Fp z_) : x(x_), y(y_), z(z_) {}

    static Vec3 zero()   { return {}; }
    static Vec3 up()     { return {Fp::zero(), Fp::one(), Fp::zero()}; }
    static Vec3 forward(){ return {Fp::zero(), Fp::zero(), Fp::one()}; }

    // Arithmetic
    constexpr Vec3 operator+(Vec3 o) const { return {x+o.x, y+o.y, z+o.z}; }
    constexpr Vec3 operator-(Vec3 o) const { return {x-o.x, y-o.y, z-o.z}; }
    constexpr Vec3 operator-()       const { return {-x, -y, -z}; }
    constexpr Vec3 operator*(Fp s)   const { return {x*s, y*s, z*s}; }
    constexpr Vec3 operator/(Fp s)   const { return {x/s, y/s, z/s}; }
    constexpr Vec3 operator*(int32_t s) const { return {x*s, y*s, z*s}; }
    Vec3& operator+=(Vec3 o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
    Vec3& operator-=(Vec3 o) { x-=o.x; y-=o.y; z-=o.z; return *this; }
    Vec3& operator*=(Fp s)   { x*=s; y*=s; z*=s; return *this; }
    bool operator==(Vec3 o) const { return x==o.x && y==o.y && z==o.z; }

    // Dot product — uses int64_t intermediates via Fp::operator*
    constexpr Fp dot(Vec3 o) const {
        return (x*o.x) + (y*o.y) + (z*o.z);
    }

    // Cross product
    constexpr Vec3 cross(Vec3 o) const {
        return {
            (y*o.z) - (z*o.y),
            (z*o.x) - (x*o.z),
            (x*o.y) - (y*o.x)
        };
    }

    // Squared length — NOTE: overflows Q15.16 for values > ~181 (e.g. 200^2 = 40000 > 32767)
    // For safety-critical code (velocity clamping) use length_sq_i64() instead.
    constexpr Fp length_sq() const { return dot(*this); }

    // Squared length in raw^2 units (int64) — overflow-safe for any velocity
    int64_t length_sq_i64() const {
        int64_t rx = x.raw, ry = y.raw, rz = z.raw;
        return rx*rx + ry*ry + rz*rz;
    }

    // Length via deterministic sqrt
    Fp length() const { return fp_sqrt(length_sq()); }

    // Normalize (returns zero vector if near-zero length)
    Vec3 normalized() const {
        Fp len = length();
        if (len.raw == 0) return zero();
        return *this / len;
    }

    // Component-wise abs
    Vec3 abs() const { return {x.abs(), y.abs(), z.abs()}; }
};

// ---------------------------------------------------------------------------
// Domain-specific Vec3 aliases
// ---------------------------------------------------------------------------
using Vec3Pos = Vec3<FpPos>;  // World positions (Q24.8)
using Vec3Vel = Vec3<FpVel>;  // Velocities, forces, accelerations (Q15.16)
using Vec3Ang = Vec3<FpAng>;  // Angular velocities (Q8.24)

// Cross-format helpers
// Position += Velocity * dt (Q24.8 += Q15.16 * Q15.16)
// dt stored as Q15.16; velocity * dt -> Q15.16 * Q15.16 -> Q31.32, narrow to Q24.8
inline Vec3Pos pos_add_vel_dt(Vec3Pos pos, Vec3Vel vel, FpVel dt) {
    // result_q8 = vel_q16 * dt_q16 >> 24 (shift 16+16-8=24)
    auto conv = [&](FpVel v) -> FpPos {
        int64_t prod = static_cast<int64_t>(v.raw) * static_cast<int64_t>(dt.raw);
        return FpPos{static_cast<int32_t>(prod >> 24)};
    };
    return {pos.x + conv(vel.x),
            pos.y + conv(vel.y),
            pos.z + conv(vel.z)};
}

// Velocity += Acceleration * dt (Q15.16 += Q15.16 * Q15.16)
inline Vec3Vel vel_add_acc_dt(Vec3Vel vel, Vec3Vel acc, FpVel dt) {
    return vel + (acc * dt);
}

// ---------------------------------------------------------------------------
// Quaternion<Fp> — unit quaternion for orientation
// Stored as (w, x, y, z) where w = cos(θ/2)
// ---------------------------------------------------------------------------
template<typename Fp>
struct Quat {
    Fp w{Fp::one()}, x{}, y{}, z{};

    constexpr Quat() = default;
    constexpr Quat(Fp w_, Fp x_, Fp y_, Fp z_) : w(w_), x(x_), y(y_), z(z_) {}

    static Quat identity() { return {}; }

    // Hamilton product
    Quat operator*(Quat q) const {
        return {
            (w*q.w) - (x*q.x) - (y*q.y) - (z*q.z),
            (w*q.x) + (x*q.w) + (y*q.z) - (z*q.y),
            (w*q.y) - (x*q.z) + (y*q.w) + (z*q.x),
            (w*q.z) + (x*q.y) - (y*q.x) + (z*q.w)
        };
    }

    Quat conjugate() const { return {w, -x, -y, -z}; }

    // Rotate a vector by this quaternion: q * v * q^{-1}
    Vec3<Fp> rotate(Vec3<Fp> v) const {
        // Optimised form: 2 cross products
        Vec3<Fp> qv{x, y, z};
        Vec3<Fp> uv  = qv.cross(v);
        Vec3<Fp> uuv = qv.cross(uv);
        // v + 2*w*uv + 2*uuv
        auto two = Fp::from_int(2);
        return v + (uv * (w * two)) + (uuv * two);
    }

    // Normalise (fast first-order approximation to avoid sqrt)
    // q ~= q / |q|; for small integration steps quaternion drift is minimal
    Quat normalised_fast() const {
        Fp sq = (w*w)+(x*x)+(y*y)+(z*z);
        if (sq.raw == 0) return identity();
        Fp inv_len = Fp::one() / fp_sqrt(sq);
        if (inv_len.raw == 0) return *this;  // already unit length within fp precision
        return {w*inv_len, x*inv_len, y*inv_len, z*inv_len};
    }

    // Integrate angular velocity ω for timestep dt
    // q' = q + 0.5 * dt * [0, ω] * q
    Quat integrate(Vec3<Fp> omega_half_dt) const {
        // Δq = 0.5 * Quat(0, ω*dt) * q (first-order)
        Quat dq{Fp::zero(), omega_half_dt.x, omega_half_dt.y, omega_half_dt.z};
        Quat product = dq * (*this);
        return Quat{
            w + product.w,
            x + product.x,
            y + product.y,
            z + product.z
        }.normalised_fast();
    }

    // Euler angle extraction (for joint limit checks), returns pitch/yaw/roll in Q15.16
    // Simplified: returns twist angle around Z axis (used for hinge joint limits)
    FpVel twist_z() const;
};

using QuatVel = Quat<FpVel>;

// ---------------------------------------------------------------------------
// AABB — axis-aligned bounding box using positions
// ---------------------------------------------------------------------------
struct AABB {
    Vec3Pos min_pt{}, max_pt{};

    AABB() = default;
    AABB(Vec3Pos mn, Vec3Pos mx) : min_pt(mn), max_pt(mx) {}

    // Expand by a point
    void expand(Vec3Pos p) {
        if (p.x < min_pt.x) min_pt.x = p.x;
        if (p.y < min_pt.y) min_pt.y = p.y;
        if (p.z < min_pt.z) min_pt.z = p.z;
        if (p.x > max_pt.x) max_pt.x = p.x;
        if (p.y > max_pt.y) max_pt.y = p.y;
        if (p.z > max_pt.z) max_pt.z = p.z;
    }

    // Union of two AABBs
    AABB merged(AABB other) const {
        AABB r;
        r.min_pt.x = min_pt.x < other.min_pt.x ? min_pt.x : other.min_pt.x;
        r.min_pt.y = min_pt.y < other.min_pt.y ? min_pt.y : other.min_pt.y;
        r.min_pt.z = min_pt.z < other.min_pt.z ? min_pt.z : other.min_pt.z;
        r.max_pt.x = max_pt.x > other.max_pt.x ? max_pt.x : other.max_pt.x;
        r.max_pt.y = max_pt.y > other.max_pt.y ? max_pt.y : other.max_pt.y;
        r.max_pt.z = max_pt.z > other.max_pt.z ? max_pt.z : other.max_pt.z;
        return r;
    }

    // Surface area (for SAH heuristic in tree)
    FpPos surface_area() const {
        FpPos dx = max_pt.x - min_pt.x;
        FpPos dy = max_pt.y - min_pt.y;
        FpPos dz = max_pt.z - min_pt.z;
        return fp_mul<8,8,8>(dx,dy) + fp_mul<8,8,8>(dy,dz) + fp_mul<8,8,8>(dx,dz);
    }

    bool overlaps(const AABB& o) const {
        return (min_pt.x <= o.max_pt.x && max_pt.x >= o.min_pt.x) &&
               (min_pt.y <= o.max_pt.y && max_pt.y >= o.min_pt.y) &&
               (min_pt.z <= o.max_pt.z && max_pt.z >= o.min_pt.z);
    }

    bool contains(Vec3Pos p) const {
        return (p.x >= min_pt.x && p.x <= max_pt.x) &&
               (p.y >= min_pt.y && p.y <= max_pt.y) &&
               (p.z >= min_pt.z && p.z <= max_pt.z);
    }

    // Inflate by margin
    AABB inflated(FpPos margin) const {
        return {
            {min_pt.x - margin, min_pt.y - margin, min_pt.z - margin},
            {max_pt.x + margin, max_pt.y + margin, max_pt.z + margin}
        };
    }
};

// ---------------------------------------------------------------------------
// Quaternion twist_z implementation
// ---------------------------------------------------------------------------
template<typename Fp>
FpVel Quat<Fp>::twist_z() const {
    // Projection of quaternion onto Z axis, extract angle
    // angle = 2 * atan2(z, w) — approximate with ratio for small angles
    // For joint limits, a fast approximation is acceptable
    // Using: angle ≈ 2 * z/w for |angle| < pi/4
    if (w.raw == 0) return FpVel::from_float(3.14159f); // 180 degrees
    int64_t numer = static_cast<int64_t>(z.raw) << 16;
    int32_t ratio = static_cast<int32_t>(numer / w.raw); // Q15.16 ratio z/w
    // angle ≈ 2 * atan(z/w) ≈ 2 * (z/w) for small angles
    return FpVel{ratio * 2};
}

} // namespace dspe