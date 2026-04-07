#include "apc_math/apc_vec3.h"
#include <cstdint>
#include <cstring>

struct Xorshift64 {
    uint64_t state;
    Xorshift64(uint64_t seed) : state(seed) { if(state == 0) state = 1; }

    uint64_t next() {
        uint64_t x = state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        state = x;
        return x;
    }

    float next_float() {
        uint64_t u = next();
        uint32_t bits = (uint32_t)(u & 0x007FFFFF) | 0x3F800000; 
        float f;
        std::memcpy(&f, &bits, sizeof(f));
        return (f - 1.0f) * 2000.0f - 1000.0f;
    }
};

inline uint64_t hash_float_bits(float f) {
    uint32_t bits;
    std::memcpy(&bits, &f, sizeof(bits));
    uint64_t h = bits;
    h ^= h >> 16;
    h *= 0x45d9f3b;
    return h;
}

uint64_t run_det_vec3_ops() {
    Xorshift64 rng(12345678);
    uint64_t hash = 0;

    for (int i = 0; i < 10000; ++i) {
        apc::Vec3 a(rng.next_float(), rng.next_float(), rng.next_float());
        apc::Vec3 b(rng.next_float(), rng.next_float(), rng.next_float());
        float s = rng.next_float();

        if (i % 100 == 0) a = apc::Vec3(0.0f, 0.0f, 0.0f);
        if (i % 100 == 1) b = apc::Vec3(-0.0f, -0.0f, -0.0f);
        if (i % 100 == 2) s = 0.0f;

        apc::Vec3 c = apc::Vec3::add(a, b);
        apc::Vec3 d = apc::Vec3::sub(a, b);
        apc::Vec3 e = apc::Vec3::scale(a, s);
        apc::Vec3 f = apc::Vec3::cross(a, b);
        float dot = apc::Vec3::dot(a, b);
        apc::Vec3 n = apc::Vec3::normalize(a);

        hash ^= hash_float_bits(c.x) ^ hash_float_bits(c.y) ^ hash_float_bits(c.z);
        hash ^= hash_float_bits(d.x) ^ hash_float_bits(d.y) ^ hash_float_bits(d.z);
        hash ^= hash_float_bits(e.x) ^ hash_float_bits(e.y) ^ hash_float_bits(e.z);
        hash ^= hash_float_bits(f.x) ^ hash_float_bits(f.y) ^ hash_float_bits(f.z);
        hash ^= hash_float_bits(dot);
        hash ^= hash_float_bits(n.x) ^ hash_float_bits(n.y) ^ hash_float_bits(n.z);
    }
    return hash;
}