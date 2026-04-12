#pragma once
#include "apc_math/apc_vec3.h"
#include <cstdint>

namespace apc {

struct AABB {
    Vec3 min;
    Vec3 max;
};

struct BroadphasePair {
    uint32_t body_a_id;
    uint32_t body_b_id; // Always guaranteed body_a_id < body_b_id

    // Strict ordering for deterministic sorting/output
    bool operator<(const BroadphasePair& other) const {
        if (body_a_id != other.body_a_id) return body_a_id < other.body_a_id;
        return body_b_id < other.body_b_id;
    }
    bool operator==(const BroadphasePair& other) const {
        return body_a_id == other.body_a_id && body_b_id == other.body_b_id;
    }
};

class Broadphase {
public:
    // Fixed allocation budgets (Zero dynamic allocation)
    static constexpr uint32_t MAX_BODIES = 256;
    static constexpr uint32_t MAX_PAIRS = 4000;

    struct SAPProxy {
        uint32_t body_id;
        float min_x;
        float max_x;
        AABB aabb;
    };

private:
    SAPProxy proxies[MAX_BODIES];
    uint32_t proxy_count = 0;

    BroadphasePair pairs[MAX_PAIRS];
    uint32_t pair_count = 0;

public:
    void clear() {
        proxy_count = 0;
        pair_count = 0;
    }

    void add_aabb(uint32_t body_id, const AABB& bounds) {
        if (proxy_count < MAX_BODIES) {
            SAPProxy& p = proxies[proxy_count++];
            p.body_id = body_id;
            p.min_x   = bounds.min.x;
            p.max_x   = bounds.max.x;
            p.aabb    = bounds;
        }
    }

    void compute_pairs() {
        pair_count = 0;
        if (proxy_count < 2) return;

        // 1. Sort the proxies along the X-axis using Insertion Sort.
        // This is O(N) for coherent frames where objects haven't moved much.
        for (uint32_t i = 1; i < proxy_count; ++i) {
            SAPProxy key = proxies[i];
            int32_t j = static_cast<int32_t>(i) - 1;

            while (j >= 0 && proxies[j].min_x > key.min_x) {
                proxies[j + 1] = proxies[j];
                j--;
            }
            proxies[j + 1] = key;
        }

        // 2. Sweep the sorted array to find overlapping pairs
        for (uint32_t i = 0; i < proxy_count; ++i) {
            const SAPProxy& left = proxies[i];

            for (uint32_t j = i + 1; j < proxy_count; ++j) {
                const SAPProxy& right = proxies[j];

                // PRUNE: If the right object's min_x is greater than the left
                // object's max_x, they cannot overlap. Since the array is sorted,
                // no further objects in the inner loop can overlap with left either.
                if (right.min_x > left.max_x) {
                    break;
                }

                // If X overlaps, check Y and Z for full AABB intersection
                if (test_aabb_overlap(left.aabb, right.aabb)) {
                    if (pair_count < MAX_PAIRS) {
                        // DETERMINISM: Enforce body_a_id < body_b_id ordering
                        uint32_t a = left.body_id;
                        uint32_t b = right.body_id;
                        if (a > b) {
                            uint32_t tmp = a;
                            a = b;
                            b = tmp;
                        }
                        pairs[pair_count++] = {a, b};
                    }
                }
            }
        }

        // 3. Sort output pairs lexicographically so the solver processes in
        // exact same order cross-platform (determinism guarantee).
        insertion_sort_pairs();
    }

    const BroadphasePair* get_pairs() const { return pairs; }
    uint32_t get_pair_count() const { return pair_count; }

private:
    inline bool test_aabb_overlap(const AABB& a, const AABB& b) const {
        return (a.min.x <= b.max.x && a.max.x >= b.min.x) &&
               (a.min.y <= b.max.y && a.max.y >= b.min.y) &&
               (a.min.z <= b.max.z && a.max.z >= b.min.z);
    }

    // Simple insertion sort for output pairs — already nearly sorted from the
    // sweep, so this is effectively O(N) for typical frame-to-frame coherence.
    void insertion_sort_pairs() {
        for (uint32_t i = 1; i < pair_count; ++i) {
            BroadphasePair key = pairs[i];
            int32_t j = static_cast<int32_t>(i) - 1;
            while (j >= 0 && pairs[j].body_a_id > key.body_a_id) {
                pairs[j + 1] = pairs[j];
                j--;
            }
            pairs[j + 1] = key;
        }
    }
};

} // namespace apc
