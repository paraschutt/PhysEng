#include "apc_math/apc_mat3.h"
#include "apc_math/apc_quat.h"
#include "apc_math/apc_vec3.h"
#include <cstdint>
#include <cstring>

struct Xorshift64 {
    uint64_t state;
    Xorshift64(uint64_t seed) : state(seed) { if(state == 0) state = 1; }
    uint64_t next() { uint64_t x = state; x ^= x << 13; x ^= x >> 7; x ^= x << 17; state = x; return x; }
    float next_float() { uint64_t u = next(); uint32_t bits = (uint32_t)(u & 0x007FFFFF) | 0x3F800000; float f; std::memcpy(&f, &bits, sizeof(f)); return (f - 1.0f) * 2.0f - 1.0f; }
};

inline uint64_t hash_float_bits(float f) {
    uint32_t bits;
    std::memcpy(&bits, &f, sizeof(bits));
    uint64_t h = bits;
    h ^= h >> 16;
    h *= 0x45d9f3b;
    return h;
}

uint64_t run_det_mat3_ops() {
    Xorshift64 rng(11223344);
    uint64_t hash = 0;

    for (int i = 0; i < 10000; ++i) {
        apc::Mat3 m1 = apc::Mat3::identity();
        apc::Mat3 m2 = apc::Mat3::identity();

        for(int j=0; j<9; ++j) m1.m[j] = rng.next_float();
        for(int j=0; j<9; ++j) m2.m[j] = rng.next_float();

        if (i % 100 == 0) {
            for(int j=0; j<9; ++j) m1.m[j] = 0.0f; 
        }

        apc::Mat3 mult = apc::Mat3::multiply(m1, m2);
        apc::Mat3 trans = m1.transpose();
        apc::Mat3 inv = m1.inverse();
        apc::Vec3 diag = inv.diagonal();

        for(int j=0; j<9; ++j) hash ^= hash_float_bits(mult.m[j]);
        for(int j=0; j<9; ++j) hash ^= hash_float_bits(trans.m[j]);
        for(int j=0; j<9; ++j) hash ^= hash_float_bits(inv.m[j]);
        hash ^= hash_float_bits(diag.x) ^ hash_float_bits(diag.y) ^ hash_float_bits(diag.z);
    }
    return hash;
}