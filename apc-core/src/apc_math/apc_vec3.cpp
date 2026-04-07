#include "apc_vec3.h"
#include <cstdint>

namespace apc {

// FNV-1a hash over the raw bitwise representation of the 3 floats.
// We hash bits, not logical values, so -0.0f and +0.0f hash differently, 
// preventing silent state divergence.
uint32_t Vec3::hash() const {
    const uint8_t* data = reinterpret_cast<const uint8_t*>(this);
    uint32_t h = 0x811c9dc5u;
    
    for (size_t i = 0; i < sizeof(float) * 3; ++i) {
        h ^= data[i];
        h *= 0x01000193u; // FNV prime
    }
    
    return h;
}

} // namespace apc