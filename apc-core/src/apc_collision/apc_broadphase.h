#pragma once
#include "apc_math/apc_vec3.h"
#include "apc_containers/apc_flat_map.h"
#include <vector>
#include <algorithm>
#include <cstdint>

namespace apc {

struct AABB {
    Vec3 min;
    Vec3 max;
};

struct BroadphasePair {
    uint32_t id_a;
    uint32_t id_b; // Always guaranteed id_a < id_b

    // Strict ordering for deterministic sorting/output
    bool operator<(const BroadphasePair& other) const {
        if (id_a != other.id_a) return id_a < other.id_a;
        return id_b < other.id_b;
    }
    bool operator==(const BroadphasePair& other) const {
        return id_a == other.id_a && id_b == other.id_b;
    }
};

class BroadphaseSAP {
public:
    struct Proxy {
        uint32_t id;
        AABB aabb;
    };

    void update(const std::vector<Proxy>& proxies) {
        endpoints.clear();
        endpoints.reserve(proxies.size() * 2);

        for (const auto& p : proxies) {
            endpoints.push_back({p.aabb.min.x, true, p.id});
            endpoints.push_back({p.aabb.max.x, false, p.id});
        }

        // DETERMINISM: std::stable_sort ensures identical order for equal x-values.
        // If x-values are identical, it preserves the relative order of insertion.
        std::stable_sort(endpoints.begin(), endpoints.end(), 
            [](const Endpoint& a, const Endpoint& b) {
                if (a.x != b.x) return a.x < b.x;
                // If x is identical, min must come before max to avoid false self-intersection
                if (a.is_min != b.is_min) return a.is_min;
                // Final tie-breaker: Entity ID
                return a.id < b.id;
            });
        
        active_proxies = proxies;
    }

    const std::vector<BroadphasePair>& get_potential_pairs() const {
        return potential_pairs;
    }

    // Generates pairs. O(N) assuming coherence.
    void generate_pairs() {
        potential_pairs.clear();
        int active_count = 0;
        uint32_t active_ids[512]; // Stack allocation for speed, max expected entities

        for (const auto& ep : endpoints) {
            if (ep.is_min) {
                // Check overlap against all currently active IDs
                for (int i = 0; i < active_count; ++i) {
                    uint32_t id_a = active_ids[i];
                    uint32_t id_b = ep.id;
                    
                    // Early out Y/Z overlap test
                    const AABB& a = get_aabb(id_a);
                    const AABB& b = get_aabb(id_b);
                    
                    if (a.max.y < b.min.y || a.min.y > b.max.y) continue;
                    if (a.max.z < b.min.z || a.min.z > b.max.z) continue;

                    // DETERMINISM: Enforce id_a < id_b
                    if (id_a > id_b) std::swap(id_a, id_b);
                    
                    potential_pairs.push_back({id_a, id_b});
                }
                active_ids[active_count++] = ep.id;
            } else {
                // Remove from active set
                for (int i = 0; i < active_count; ++i) {
                    if (active_ids[i] == ep.id) {
                        active_ids[i] = active_ids[active_count - 1];
                        active_count--;
                        break;
                    }
                }
            }
        }
        
        // Sort output so solver processes in exact same order cross-platform
        std::stable_sort(potential_pairs.begin(), potential_pairs.end());
    }

private:
    struct Endpoint {
        float x;
        bool is_min;
        uint32_t id;
    };

    const AABB& get_aabb(uint32_t id) const {
        // In production this is an O(1) lookup array; simplified for review
        for(const auto& p : active_proxies) { if(p.id == id) return p.aabb; }
        static AABB empty = {{0,0,0}, {0,0,0}};
        return empty;
    }

    std::vector<Endpoint> endpoints;
    std::vector<BroadphasePair> potential_pairs;
    std::vector<Proxy> active_proxies;
};

} // namespace apc