#pragma once
#include "apc_math_common.h"
#include <cmath>
#include <functional> 

namespace apc {

struct Vec3 {
    float x, y, z;
    
    // --- Constructors ---
    APC_FORCEINLINE Vec3() : x(0.0f), y(0.0f), z(0.0f) {}
    APC_FORCEINLINE Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    APC_FORCEINLINE explicit Vec3(float s) : x(s), y(s), z(s) {}
    
    // --- Basic Ops (NO operator overloading for physics hot paths) ---
    // We use named functions to make dependency on FP order explicit
    
    APC_FORCEINLINE static Vec3 add(const Vec3& a, const Vec3& b) {
        return Vec3(a.x + b.x, a.y + b.y, a.z + b.z);
    }
    
    APC_FORCEINLINE static Vec3 sub(const Vec3& a, const Vec3& b) {
        return Vec3(a.x - b.x, a.y - b.y, a.z - b.z);
    }
    
    APC_FORCEINLINE static Vec3 scale(const Vec3& v, float s) {
        return Vec3(v.x * s, v.y * s, v.z * s);
    }
    
    // Scaled add: a + b*s (common in solver, avoids intermediate)
    APC_FORCEINLINE static Vec3 scaled_add(const Vec3& a, const Vec3& b, float s) {
        return Vec3(a.x + b.x * s, a.y + b.y * s, a.z + b.z * s);
    }
    
    // Component-wise multiply (for inertia tensor transforms)
    APC_FORCEINLINE static Vec3 mul_comp(const Vec3& a, const Vec3& b) {
        return Vec3(a.x * b.x, a.y * b.y, a.z * b.z);
    }
    
    // --- Dot Product ---
    // CRITICAL: Order of operations must be consistent
    // We use (x*x) + (y*y) + (z*z) explicitly
    APC_FORCEINLINE static float dot(const Vec3& a, const Vec3& b) {
        float xx = a.x * b.x;
        float yy = a.y * b.y;
        float zz = a.z * b.z;
        return xx + yy + zz;
    }
    
    // Self-dot (squared length)
    APC_FORCEINLINE static float length_sq(const Vec3& v) {
        float xx = v.x * v.x;
        float yy = v.y * v.y;
        float zz = v.z * v.z;
        return xx + yy + zz;
    }
    
    // --- Cross Product ---
    // Standard formula, explicit ordering
    APC_FORCEINLINE static Vec3 cross(const Vec3& a, const Vec3& b) {
        return Vec3(
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        );
    }
    
    // --- Length (uses sqrt, not sqrtf - both map to same on IEEE platforms) ---
    APC_FORCEINLINE static float length(const Vec3& v) {
        return std::sqrt(length_sq(v));
    }
    
    // --- Normalize ---
    // Returns zero vector if length is too small (no division by zero, no NaN)
    APC_FORCEINLINE static Vec3 normalize(const Vec3& v) {
        float len_sq = length_sq(v);
        if (len_sq < APC_EPSILON_SQ) {
            return Vec3(0.0f, 0.0f, 0.0f);
        }
        float inv_len = 1.0f / std::sqrt(len_sq);
        return scale(v, inv_len);
    }
    
    // --- Safe Normalize (returns fallback if degenerate) ---
    APC_FORCEINLINE static Vec3 safe_normalize(const Vec3& v, const Vec3& fallback) {
        float len_sq = length_sq(v);
        if (len_sq < APC_EPSILON_SQ) {
            return fallback;
        }
        float inv_len = 1.0f / std::sqrt(len_sq);
        return scale(v, inv_len);
    }
    
    // --- Lerp ---
    APC_FORCEINLINE static Vec3 lerp(const Vec3& a, const Vec3& b, float t) {
        return Vec3(
            a.x + (b.x - a.x) * t,
            a.y + (b.y - a.y) * t,
            a.z + (b.z - a.z) * t
        );
    }
    
    // --- Comparison (for sorting, NOT for physics) ---
    APC_FORCEINLINE bool equals_exact(const Vec3& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
    
    APC_FORCEINLINE bool equals_approx(const Vec3& other, float eps = APC_EPSILON) const {
        return std::abs(x - other.x) <= eps &&
               std::abs(y - other.y) <= eps &&
               std::abs(z - other.z) <= eps;
    }
    
    // --- Hash for determinism testing ---
    uint32_t hash() const;
};

// Operator overloads provided ONLY for non-hot-path convenience (tools, tests, init)
// These are NOT used in the simulation loop
#ifndef APC_SIM_HOT_PATH
APC_FORCEINLINE bool operator==(const Vec3& a, const Vec3& b) { return a.equals_exact(b); }
APC_FORCEINLINE bool operator!=(const Vec3& a, const Vec3& b) { return !a.equals_exact(b); }
APC_FORCEINLINE Vec3 operator+(const Vec3& a, const Vec3& b) { return Vec3::add(a, b); }
APC_FORCEINLINE Vec3 operator-(const Vec3& a, const Vec3& b) { return Vec3::sub(a, b); }
APC_FORCEINLINE Vec3 operator*(const Vec3& v, float s) { return Vec3::scale(v, s); }
APC_FORCEINLINE Vec3 operator*(float s, const Vec3& v) { return Vec3::scale(v, s); }
#endif

} // namespace apc

// Specialize std::hash for unordered containers (tests, tools only)
namespace std {
template<>
struct hash<apc::Vec3> {
    size_t operator()(const apc::Vec3& v) const noexcept {
        size_t h = 0;
        h ^= std::hash<float>{}(v.x) + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= std::hash<float>{}(v.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= std::hash<float>{}(v.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    }
};
}