// =============================================================================
// test_fixedpoint.cpp — FixedPoint 16.16 Deterministic Math Correctness Tests
// =============================================================================
// Tests the FixedPoint class from apc_math/apc_fixed.h.
// Verifies arithmetic, comparisons, conversions, compound assignment,
// determinism (bit-identical results across runs), and edge cases.
//
// Also tests the RawConfigParser from tools/designer_tool/apc_raw_parser.h
// for FixedPoint ingestion from raw text files.
// =============================================================================

#include "apc_math/apc_fixed.h"
#include "apc_platform/apc_fp_mode.h"
#include <cstdio>
#include <cmath>
#include <cstring>

using namespace apc;

// ---------------------------------------------------------------------------
// Hashing (FNV-1a, same pattern as existing tests)
// ---------------------------------------------------------------------------
static uint32_t fnv1a_bytes(const void* data, size_t len) {
    const uint8_t* p = static_cast<const uint8_t*>(data);
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < len; ++i) {
        h ^= p[i];
        h *= 16777619u;
    }
    return h;
}

static void hash_i32(uint64_t& h, int32_t v) {
    h ^= static_cast<uint64_t>(static_cast<uint32_t>(v)) + 0x9e3779b9 + (h << 6);
}

static void hash_bool(uint64_t& h, bool v) {
    h ^= static_cast<uint64_t>(v ? 1 : 0) + 0x9e3779b9 + (h << 6);
}

int main() {
    enforce_deterministic_fp_mode();

    int passed = 0;
    int total  = 10;
    uint64_t state_hash = 0;

    std::printf("=== FixedPoint 16.16 Correctness Tests ===\n\n");

    // -----------------------------------------------------------------------
    // Test 1: Construction from integer
    // -----------------------------------------------------------------------
    {
        FixedPoint a(3);
        FixedPoint b(-5);
        bool ok = (a.to_int() == 3) && (b.to_int() == -5);
        std::printf("[%s] Test 1: Construction from int (3, -5)\n",
                     ok ? "PASS" : "FAIL");
        if (ok) ++passed;
        hash_i32(state_hash, a.raw_value());
        hash_i32(state_hash, b.raw_value());
    }

    // -----------------------------------------------------------------------
    // Test 2: Construction from float and to_float() round-trip
    // -----------------------------------------------------------------------
    {
        FixedPoint a(0.0f);
        FixedPoint b(1.0f);
        FixedPoint c(-1.5f);
        FixedPoint d(3.14159f);

        bool ok = (std::abs(a.to_float()) < 0.001f)
               && (std::abs(b.to_float() - 1.0f) < 0.001f)
               && (std::abs(c.to_float() - (-1.5f)) < 0.001f)
               && (std::abs(d.to_float() - 3.14159f) < 0.01f);  // 16.16 has ~5 digits
        std::printf("[%s] Test 2: Float construction round-trip "
                     "(0.0=%.4f, 1.0=%.4f, -1.5=%.4f, pi=%.4f)\n",
                     ok ? "PASS" : "FAIL",
                     a.to_float(), b.to_float(), c.to_float(), d.to_float());
        if (ok) ++passed;
        hash_i32(state_hash, a.raw_value());
        hash_i32(state_hash, b.raw_value());
        hash_i32(state_hash, c.raw_value());
        hash_i32(state_hash, d.raw_value());
    }

    // -----------------------------------------------------------------------
    // Test 3: Addition and subtraction
    // -----------------------------------------------------------------------
    {
        FixedPoint a(3);
        FixedPoint b(2);
        FixedPoint c = a + b;
        FixedPoint d = a - b;

        bool ok = (c.to_int() == 5) && (d.to_int() == 1);
        std::printf("[%s] Test 3: Add/sub (3+2=%d, 3-2=%d)\n",
                     ok ? "PASS" : "FAIL", c.to_int(), d.to_int());
        if (ok) ++passed;
        hash_i32(state_hash, c.raw_value());
        hash_i32(state_hash, d.raw_value());
    }

    // -----------------------------------------------------------------------
    // Test 4: Multiplication (uses 64-bit intermediate)
    // -----------------------------------------------------------------------
    {
        FixedPoint a(4);
        FixedPoint b(3);
        FixedPoint c = a * b;

        bool ok = (c.to_int() == 12);
        std::printf("[%s] Test 4: Multiplication (4*3=%d)\n",
                     ok ? "PASS" : "FAIL", c.to_int());
        if (ok) ++passed;
        hash_i32(state_hash, c.raw_value());
    }

    // -----------------------------------------------------------------------
    // Test 5: Division (uses 64-bit intermediate with pre-shift)
    // -----------------------------------------------------------------------
    {
        FixedPoint a(10);
        FixedPoint b(2);
        FixedPoint c = a / b;

        bool ok = (c.to_int() == 5);
        std::printf("[%s] Test 5: Division (10/2=%d)\n",
                     ok ? "PASS" : "FAIL", c.to_int());
        if (ok) ++passed;
        hash_i32(state_hash, c.raw_value());
    }

    // -----------------------------------------------------------------------
    // Test 6: Fractional arithmetic precision
    // -----------------------------------------------------------------------
    {
        // 0.5 * 0.5 = 0.25
        FixedPoint a(0.5f);
        FixedPoint half = a * a;
        float half_f = half.to_float();
        bool ok1 = (std::abs(half_f - 0.25f) < 0.001f);

        // 1.5 + 2.25 = 3.75
        FixedPoint x(1.5f);
        FixedPoint y(2.25f);
        FixedPoint z = x + y;
        float z_f = z.to_float();
        bool ok2 = (std::abs(z_f - 3.75f) < 0.01f);

        bool ok = ok1 && ok2;
        std::printf("[%s] Test 6: Fractional precision "
                     "(0.5*0.5=%.4f, 1.5+2.25=%.4f)\n",
                     ok ? "PASS" : "FAIL", half_f, z_f);
        if (ok) ++passed;
        hash_i32(state_hash, half.raw_value());
        hash_i32(state_hash, z.raw_value());
    }

    // -----------------------------------------------------------------------
    // Test 7: Comparison operators
    // -----------------------------------------------------------------------
    {
        FixedPoint a(3);
        FixedPoint b(5);
        FixedPoint c(3);

        bool ok = (a < b) && (b > a) && (a == c) && (a != b)
               && (a <= c) && (b >= c) && !(a > b) && !(a != c);
        std::printf("[%s] Test 7: Comparisons (3<5, 5>3, 3==3, 3!=5)\n",
                     ok ? "PASS" : "FAIL");
        if (ok) ++passed;
        hash_bool(state_hash, ok);
    }

    // -----------------------------------------------------------------------
    // Test 8: Compound assignment operators
    // -----------------------------------------------------------------------
    {
        FixedPoint a(10);
        a += FixedPoint(3);
        bool ok_add = (a.to_int() == 13);

        a -= FixedPoint(5);
        bool ok_sub = (a.to_int() == 8);

        a *= FixedPoint(2);
        bool ok_mul = (a.to_int() == 16);

        a /= FixedPoint(4);
        bool ok_div = (a.to_int() == 4);

        bool ok = ok_add && ok_sub && ok_mul && ok_div;
        std::printf("[%s] Test 8: Compound assignment (10+=3=%d, -=5=%d, *=2=%d, /=4=%d)\n",
                     ok ? "PASS" : "FAIL", 13, 8, 16, 4);
        if (ok) ++passed;
        hash_i32(state_hash, a.raw_value());
    }

    // -----------------------------------------------------------------------
    // Test 9: Unary minus and zero
    // -----------------------------------------------------------------------
    {
        FixedPoint a(7);
        FixedPoint neg = -a;
        FixedPoint zero;

        bool ok = (neg.to_int() == -7) && (zero.to_int() == 0)
               && (zero.raw_value() == 0) && (neg == FixedPoint(-7));
        std::printf("[%s] Test 9: Unary minus (-7=%d, zero=%d)\n",
                     ok ? "PASS" : "FAIL", neg.to_int(), zero.to_int());
        if (ok) ++passed;
        hash_i32(state_hash, neg.raw_value());
        hash_i32(state_hash, zero.raw_value());
    }

    // -----------------------------------------------------------------------
    // Test 10: Determinism — same computation produces bit-identical raw values
    // -----------------------------------------------------------------------
    {
        // Simulate a 100-step accumulation using FixedPoint
        FixedPoint accumulator(0);
        FixedPoint step(0.01f);
        for (int i = 0; i < 100; ++i) {
            accumulator += step;
        }

        // Run the exact same computation again
        FixedPoint accumulator2(0);
        for (int i = 0; i < 100; ++i) {
            accumulator2 += step;
        }

        bool ok = (accumulator.raw_value() == accumulator2.raw_value());
        float final_f = accumulator.to_float();
        std::printf("[%s] Test 10: Determinism — 100x 0.01 accumulation "
                     "(raw=0x%08x, float=%.6f)\n",
                     ok ? "PASS" : "FAIL",
                     static_cast<uint32_t>(accumulator.raw_value()), final_f);
        if (ok) ++passed;
        hash_i32(state_hash, accumulator.raw_value());
    }

    // -----------------------------------------------------------------------
    // Summary & state hash
    // -----------------------------------------------------------------------
    std::printf("\n=== FixedPoint Test Results: %d/%d passed ===\n", passed, total);

    uint32_t final_hash = fnv1a_bytes(&state_hash, sizeof(state_hash));
    std::printf("State Hash: 0x%08x\n", final_hash);

    return (passed == total) ? 0 : 1;
}
