#pragma once
#include <cstdint>

namespace apc {

// 16.16 Fixed-Point implementation for guaranteed cross-platform determinism.
// The integer representation stores values as (whole << 16) | fraction,
// giving ~5 decimal digits of precision (range: -32768 to 32767.99998).
//
// Use this as a drop-in replacement for float in any simulation path that
// must produce bit-identical results across x86-64, ARM64, and consoles
// regardless of compiler FP settings.
//
// Construction from float is a bridge point — once inside the fixed-point
// domain, all arithmetic is integer-only and fully deterministic.
class FixedPoint {
private:
    int32_t value;
    static constexpr int FRACT_BITS = 16;
    static constexpr int32_t FRACT_SCALE = (1 << FRACT_BITS);   // 65536
    static constexpr int32_t FRACT_MASK  = FRACT_SCALE - 1;     // 0xFFFF

    explicit FixedPoint(int32_t raw_val, bool /* is_raw */) : value(raw_val) {}

public:
    FixedPoint() : value(0) {}
    explicit FixedPoint(int32_t i) : value(i << FRACT_BITS) {}
    explicit FixedPoint(float f) : value(static_cast<int32_t>(f * FRACT_SCALE)) {}

    // -----------------------------------------------------------------------
    // Type Conversions
    // -----------------------------------------------------------------------
    int32_t to_int() const { return value >> FRACT_BITS; }
    float   to_float() const { return static_cast<float>(value) / static_cast<float>(FRACT_SCALE); }

    // -----------------------------------------------------------------------
    // Arithmetic Operators
    // -----------------------------------------------------------------------
    FixedPoint operator+(const FixedPoint& other) const {
        return FixedPoint(value + other.value, true);
    }

    FixedPoint operator-(const FixedPoint& other) const {
        return FixedPoint(value - other.value, true);
    }

    FixedPoint operator*(const FixedPoint& other) const {
        // Cast to 64-bit to prevent overflow during intermediate calculation
        int64_t temp = static_cast<int64_t>(value) * static_cast<int64_t>(other.value);
        return FixedPoint(static_cast<int32_t>(temp >> FRACT_BITS), true);
    }

    FixedPoint operator/(const FixedPoint& other) const {
        int64_t temp = (static_cast<int64_t>(value) << FRACT_BITS) / other.value;
        return FixedPoint(static_cast<int32_t>(temp), true);
    }

    // Compound assignment
    FixedPoint& operator+=(const FixedPoint& other) { value += other.value; return *this; }
    FixedPoint& operator-=(const FixedPoint& other) { value -= other.value; return *this; }
    FixedPoint& operator*=(const FixedPoint& other) {
        int64_t temp = static_cast<int64_t>(value) * static_cast<int64_t>(other.value);
        value = static_cast<int32_t>(temp >> FRACT_BITS);
        return *this;
    }
    FixedPoint& operator/=(const FixedPoint& other) {
        int64_t temp = (static_cast<int64_t>(value) << FRACT_BITS) / other.value;
        value = static_cast<int32_t>(temp);
        return *this;
    }

    // Unary minus
    FixedPoint operator-() const { return FixedPoint(-value, true); }

    // -----------------------------------------------------------------------
    // Comparison Operators
    // -----------------------------------------------------------------------
    bool operator==(const FixedPoint& other) const { return value == other.value; }
    bool operator!=(const FixedPoint& other) const { return value != other.value; }
    bool operator<(const FixedPoint& other) const  { return value < other.value; }
    bool operator>(const FixedPoint& other) const  { return value > other.value; }
    bool operator<=(const FixedPoint& other) const { return value <= other.value; }
    bool operator>=(const FixedPoint& other) const { return value >= other.value; }

    // -----------------------------------------------------------------------
    // Raw access (for serialization / hashing)
    // -----------------------------------------------------------------------
    int32_t raw_value() const { return value; }

    // -----------------------------------------------------------------------
    // Factory: construct from raw integer representation
    // -----------------------------------------------------------------------
    static FixedPoint from_raw(int32_t raw) { return FixedPoint(raw, true); }
};

} // namespace apc
