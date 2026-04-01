#pragma once
// ============================================================================
// DSPE Broad Phase — Dynamic AABB Tree
// Insert/remove/update entities; query overlapping pairs
// Tree nodes stored in flat array for cache efficiency
// ============================================================================
#include "../math_types.h"
#include "../entity.h"
#include <array>
#include <vector>
#include <cstdint>

namespace dspe {

// ---------------------------------------------------------------------------
// Tree node
// ---------------------------------------------------------------------------
static constexpr int32_t NULL_NODE = -1;

struct AABBNode {
    AABB     aabb{};
    int32_t  parent{NULL_NODE};
    int32_t  child[2]{NULL_NODE, NULL_NODE};
    EntityId entity{INVALID_ENTITY};  // INVALID_ENTITY for internal nodes
    int32_t  height{0};

    bool is_leaf() const { return entity != INVALID_ENTITY; }
};

// ---------------------------------------------------------------------------
// Dynamic AABB Tree
// Capacity: 256 nodes (2 * MAX_ENTITIES * 4 — generous for environment)
// ---------------------------------------------------------------------------
static constexpr int32_t AABB_TREE_CAPACITY = 256;

// Fat AABB inflation margin (avoid rebuild every frame for slow movers)
static const FpPos AABB_FAT_MARGIN = FpPos::from_float(0.1f); // 10cm

class BroadPhase {
public:
    BroadPhase();

    // Insert entity; returns node index
    int32_t insert(EntityId id, const AABB& tight_aabb);

    // Remove entity
    void remove(EntityId id);

    // Update entity's AABB (only rebuilds tree node if moved outside fat AABB)
    void update(EntityId id, const AABB& tight_aabb);

    // Query all overlapping pairs; results appended to out_pairs
    // out_pairs MUST be sorted by caller for determinism
    void query_pairs(std::vector<EntityPair>& out_pairs) const;

    // Query single AABB against tree (for CCD, trigger checks)
    void query_aabb(const AABB& query, std::vector<EntityId>& out) const;

    // Wipe tree
    void clear();

    // Validation (debug)
    bool validate() const;

private:
    std::array<AABBNode, AABB_TREE_CAPACITY> nodes_{};
    int32_t root_{NULL_NODE};
    int32_t free_list_{NULL_NODE};
    int32_t node_count_{0};

    // Node management
    int32_t alloc_node();
    void    free_node(int32_t idx);

    // Tree operations
    int32_t insert_leaf(int32_t leaf);
    void    remove_leaf(int32_t leaf);
    int32_t balance(int32_t idx);
    void    fix_heights(int32_t idx);
    void    fix_aabbs(int32_t idx);

    // Entity -> node index lookup
    std::array<int32_t, MAX_ENTITIES> entity_to_node_;

    // Recursive pair collection
    void collect_pairs(int32_t nodeA, int32_t nodeB,
                       std::vector<EntityPair>& out) const;
};

} // namespace dspe