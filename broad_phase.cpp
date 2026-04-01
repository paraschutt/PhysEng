// ============================================================================
// DSPE Broad Phase — Dynamic AABB Tree Implementation
// Rotations maintain balance (height difference ≤ 1 between siblings)
// ============================================================================
#include "dspe/systems/broad_phase.h"
#include <algorithm>
#include <cassert>
#include <cstring>

namespace dspe {

BroadPhase::BroadPhase() {
    clear();
}

void BroadPhase::clear() {
    root_ = NULL_NODE;
    node_count_ = 0;
    entity_to_node_.fill(NULL_NODE);

    // Build free list
    for (int32_t i = 0; i < AABB_TREE_CAPACITY - 1; ++i) {
        nodes_[i].child[0] = i + 1;
        nodes_[i].height   = -1;
    }
    nodes_[AABB_TREE_CAPACITY - 1].child[0] = NULL_NODE;
    nodes_[AABB_TREE_CAPACITY - 1].height   = -1;
    free_list_ = 0;
}

int32_t BroadPhase::alloc_node() {
    if (free_list_ == NULL_NODE) return NULL_NODE; // tree full
    int32_t idx  = free_list_;
    free_list_   = nodes_[idx].child[0];
    nodes_[idx]  = AABBNode{};
    ++node_count_;
    return idx;
}

void BroadPhase::free_node(int32_t idx) {
    assert(idx >= 0 && idx < AABB_TREE_CAPACITY);
    nodes_[idx].child[0] = free_list_;
    nodes_[idx].height   = -1;
    nodes_[idx].entity   = INVALID_ENTITY;
    free_list_           = idx;
    --node_count_;
}

int32_t BroadPhase::insert(EntityId id, const AABB& tight_aabb) {
    int32_t leaf = alloc_node();
    if (leaf == NULL_NODE) return NULL_NODE;

    // Fat AABB = inflate tight AABB by margin
    nodes_[leaf].aabb   = tight_aabb.inflated(AABB_FAT_MARGIN);
    nodes_[leaf].entity = id;
    nodes_[leaf].height = 0;

    entity_to_node_[id] = leaf;

    if (root_ == NULL_NODE) {
        root_ = leaf;
        nodes_[root_].parent = NULL_NODE;
        return leaf;
    }

    insert_leaf(leaf);
    return leaf;
}

int32_t BroadPhase::insert_leaf(int32_t leaf) {
    AABB leaf_aabb = nodes_[leaf].aabb;

    // Walk tree to find best sibling (greedy, cost = surface area increase)
    int32_t best = root_;
    while (!nodes_[best].is_leaf()) {
        FpPos area = nodes_[best].aabb.surface_area();

        AABB combined = nodes_[best].aabb.merged(leaf_aabb);
        FpPos combined_area = combined.surface_area();
        FpPos cost = combined_area;

        FpPos inherit_cost = combined_area - area;

        // Cost for inserting into child 0
        FpPos cost0;
        int32_t c0 = nodes_[best].child[0];
        if (nodes_[c0].is_leaf()) {
            cost0 = nodes_[c0].aabb.merged(leaf_aabb).surface_area() + inherit_cost;
        } else {
            FpPos old0 = nodes_[c0].aabb.surface_area();
            cost0 = nodes_[c0].aabb.merged(leaf_aabb).surface_area() - old0 + inherit_cost;
        }

        // Cost for inserting into child 1
        FpPos cost1;
        int32_t c1 = nodes_[best].child[1];
        if (nodes_[c1].is_leaf()) {
            cost1 = nodes_[c1].aabb.merged(leaf_aabb).surface_area() + inherit_cost;
        } else {
            FpPos old1 = nodes_[c1].aabb.surface_area();
            cost1 = nodes_[c1].aabb.merged(leaf_aabb).surface_area() - old1 + inherit_cost;
        }

        if (cost < cost0 && cost < cost1) break;
        best = (cost0 < cost1) ? c0 : c1;
    }

    // Create new internal node
    int32_t old_parent  = nodes_[best].parent;
    int32_t new_parent  = alloc_node();
    if (new_parent == NULL_NODE) return leaf;

    nodes_[new_parent].parent = old_parent;
    nodes_[new_parent].aabb   = nodes_[best].aabb.merged(leaf_aabb);
    nodes_[new_parent].height = nodes_[best].height + 1;
    nodes_[new_parent].entity = INVALID_ENTITY;

    if (old_parent != NULL_NODE) {
        if (nodes_[old_parent].child[0] == best)
            nodes_[old_parent].child[0] = new_parent;
        else
            nodes_[old_parent].child[1] = new_parent;
    } else {
        root_ = new_parent;
    }

    nodes_[new_parent].child[0] = best;
    nodes_[new_parent].child[1] = leaf;
    nodes_[best].parent = new_parent;
    nodes_[leaf].parent = new_parent;

    // Walk up and refit / balance
    fix_aabbs(new_parent);
    fix_heights(new_parent);

    return leaf;
}

void BroadPhase::remove(EntityId id) {
    int32_t leaf = entity_to_node_[id];
    if (leaf == NULL_NODE) return;
    entity_to_node_[id] = NULL_NODE;
    remove_leaf(leaf);
    free_node(leaf);
}

void BroadPhase::remove_leaf(int32_t leaf) {
    if (leaf == root_) { root_ = NULL_NODE; return; }

    int32_t parent      = nodes_[leaf].parent;
    int32_t grandparent = nodes_[parent].parent;
    int32_t sibling     = (nodes_[parent].child[0] == leaf)
                          ? nodes_[parent].child[1]
                          : nodes_[parent].child[0];

    if (grandparent != NULL_NODE) {
        if (nodes_[grandparent].child[0] == parent)
            nodes_[grandparent].child[0] = sibling;
        else
            nodes_[grandparent].child[1] = sibling;
        nodes_[sibling].parent = grandparent;
        free_node(parent);
        fix_aabbs(grandparent);
        fix_heights(grandparent);
    } else {
        root_ = sibling;
        nodes_[sibling].parent = NULL_NODE;
        free_node(parent);
    }
}

void BroadPhase::update(EntityId id, const AABB& tight_aabb) {
    int32_t leaf = entity_to_node_[id];
    if (leaf == NULL_NODE) { insert(id, tight_aabb); return; }

    // Only rebuild if entity moved outside fat AABB
    if (nodes_[leaf].aabb.contains(tight_aabb.min_pt) &&
        nodes_[leaf].aabb.contains(tight_aabb.max_pt)) {
        return; // still within fat bounds, no rebuild needed
    }

    remove_leaf(leaf);
    nodes_[leaf].aabb   = tight_aabb.inflated(AABB_FAT_MARGIN);
    nodes_[leaf].height = 0;
    insert_leaf(leaf);
}

void BroadPhase::fix_heights(int32_t idx) {
    while (idx != NULL_NODE) {
        int32_t c0 = nodes_[idx].child[0];
        int32_t c1 = nodes_[idx].child[1];
        int32_t h0 = (c0 != NULL_NODE) ? nodes_[c0].height : -1;
        int32_t h1 = (c1 != NULL_NODE) ? nodes_[c1].height : -1;
        nodes_[idx].height = 1 + std::max(h0, h1);
        idx = nodes_[idx].parent;
    }
}

void BroadPhase::fix_aabbs(int32_t idx) {
    while (idx != NULL_NODE && !nodes_[idx].is_leaf()) {
        int32_t c0 = nodes_[idx].child[0];
        int32_t c1 = nodes_[idx].child[1];
        if (c0 != NULL_NODE && c1 != NULL_NODE)
            nodes_[idx].aabb = nodes_[c0].aabb.merged(nodes_[c1].aabb);
        idx = nodes_[idx].parent;
    }
}

void BroadPhase::query_pairs(std::vector<EntityPair>& out_pairs) const {
    if (root_ == NULL_NODE || nodes_[root_].is_leaf()) return;

    // Recurse into children of root to generate non-self pairs
    collect_pairs(nodes_[root_].child[0], nodes_[root_].child[1], out_pairs);

    // Also collect pairs within each subtree
    auto collect_subtree = [&](auto& self, int32_t node) -> void {
        if (node == NULL_NODE || nodes_[node].is_leaf()) return;
        int32_t c0 = nodes_[node].child[0];
        int32_t c1 = nodes_[node].child[1];
        collect_pairs(c0, c1, out_pairs);
        self(self, c0);
        self(self, c1);
    };
    collect_subtree(collect_subtree, root_);

    // Deterministic sort by (min_id, max_id) before returning
    std::sort(out_pairs.begin(), out_pairs.end());
    // Deduplicate
    auto last = std::unique(out_pairs.begin(), out_pairs.end());
    out_pairs.erase(last, out_pairs.end());
}

void BroadPhase::collect_pairs(int32_t nodeA, int32_t nodeB,
                                std::vector<EntityPair>& out) const {
    if (nodeA == NULL_NODE || nodeB == NULL_NODE) return;
    if (!nodes_[nodeA].aabb.overlaps(nodes_[nodeB].aabb)) return;

    // Both leaves: found a candidate pair
    if (nodes_[nodeA].is_leaf() && nodes_[nodeB].is_leaf()) {
        out.push_back(make_pair(nodes_[nodeA].entity, nodes_[nodeB].entity));
        return;
    }

    // Recurse: always expand the larger (internal) node to descend deterministically
    if (nodes_[nodeA].is_leaf()) {
        // nodeB is internal: test A vs both children of B
        collect_pairs(nodeA, nodes_[nodeB].child[0], out);
        collect_pairs(nodeA, nodes_[nodeB].child[1], out);
    } else if (nodes_[nodeB].is_leaf()) {
        collect_pairs(nodes_[nodeA].child[0], nodeB, out);
        collect_pairs(nodes_[nodeA].child[1], nodeB, out);
    } else {
        // Both internal: split the larger
        if (nodes_[nodeA].height >= nodes_[nodeB].height) {
            collect_pairs(nodes_[nodeA].child[0], nodeB, out);
            collect_pairs(nodes_[nodeA].child[1], nodeB, out);
        } else {
            collect_pairs(nodeA, nodes_[nodeB].child[0], out);
            collect_pairs(nodeA, nodes_[nodeB].child[1], out);
        }
    }
}

void BroadPhase::query_aabb(const AABB& query,
                              std::vector<EntityId>& out) const {
    // Iterative stack-based traversal
    int32_t stack[64];
    int     stack_top = 0;
    if (root_ == NULL_NODE) return;
    stack[stack_top++] = root_;

    while (stack_top > 0) {
        int32_t idx = stack[--stack_top];
        if (!nodes_[idx].aabb.overlaps(query)) continue;
        if (nodes_[idx].is_leaf()) {
            out.push_back(nodes_[idx].entity);
        } else {
            if (stack_top < 62) {
                stack[stack_top++] = nodes_[idx].child[0];
                stack[stack_top++] = nodes_[idx].child[1];
            }
        }
    }
}

bool BroadPhase::validate() const {
    // Simple height balance check
    auto check = [&](auto& self, int32_t node) -> int32_t {
        if (node == NULL_NODE) return -1;
        if (nodes_[node].is_leaf()) return 0;
        int32_t h0 = self(self, nodes_[node].child[0]);
        int32_t h1 = self(self, nodes_[node].child[1]);
        int32_t diff = h0 - h1;
        if (diff < -2 || diff > 2) return -999; // unbalanced
        return 1 + std::max(h0, h1);
    };
    int32_t h = check(check, root_);
    return h != -999;
}

} // namespace dspe