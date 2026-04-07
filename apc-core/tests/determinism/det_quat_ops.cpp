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

uint64_t run_det_quat_ops() {
    Xorshift64 rng(87654321);
    uint64_t hash = 0;

    for (int i = 0; i < 10000; ++i) {
        apc::Vec3 axis(rng.next_float(), rng.next_float(), rng.next_float());
        float angle = rng.next_float() * 6.283185307f;
        
        if (i % 100 == 0) axis = apc::Vec3(0,0,0);

        apc::Quat q = apc::Quat::from_axis_angle(axis, angle);
        apc::Quat qn = apc::Quat::normalize(q);

        apc::Vec3 vec(rng.next_float(), rng.next_float(), rng.next_float());
        apc::Vec3 rotated = qn.rotate(vec);

        hash ^= hash_float_bits(qn.x) ^ hash_float_bits(qn.y) ^ hash_float_bits(qn.z) ^ hash_float_bits(qn.w);
        hash ^= hash_float_bits(rotated.x) ^ hash_float_bits(rotated.y) ^ hash_float_bits(rotated.z);
    }
    return hash;
}