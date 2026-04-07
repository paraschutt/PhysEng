#pragma once
#include "apc_obb.h"
#include "apc_convex_asset.h"
#include <vector>
#include <algorithm>

namespace apc {

struct OBBTreeNode {
    OBB box;
    int left_child;
    int right_child;
    int hull_index;   // -1 if internal node, >= 0 if leaf
};

class OBBTree {
public:
    void build(const ConvexAsset& asset) {
        nodes.clear();
        root_index = -1;
        int piece_count = asset.get_piece_count();
        if (piece_count == 0) return;

        // Phase 1: Create all leaf nodes
        std::vector<int> leaf_indices(piece_count);
        for (int i = 0; i < piece_count; ++i) {
            OBBTreeNode leaf;
            leaf.hull_index = i;
            leaf.left_child = -1;
            leaf.right_child = -1;

            // Calculate tight AABB from piece vertices, store as axis-aligned OBB
            const auto& piece = asset.get_piece(i);
            if (piece.vertices.empty()) continue;

            Vec3 mn = piece.vertices[0], mx = mn;
            for (const auto& v : piece.vertices) {
                mn.x = std::min(mn.x, v.x); mn.y = std::min(mn.y, v.y); mn.z = std::min(mn.z, v.z);
                mx.x = std::max(mx.x, v.x); mx.y = std::max(mx.y, v.y); mx.z = std::max(mx.z, v.z);
            }
            leaf.box.center = Vec3::scale(Vec3::add(mn, mx), 0.5f);
            leaf.box.extents = Vec3::scale(Vec3::sub(mx, mn), 0.5f);
            leaf.box.orientation = Quat::identity();
            leaf.box.update_cache();

            leaf_indices[i] = static_cast<int>(nodes.size());
            nodes.push_back(leaf);
        }

        // Phase 2: Build binary tree top-down (deterministic split: longest axis)
        if (nodes.empty()) return;
        root_index = build_recursive(leaf_indices, 0, static_cast<int>(leaf_indices.size()) - 1);
    }

    // Updates world-space OBBs based on a root transform.
    // For skeletal meshes, this would be extended with per-bone transforms.
    void refit(const Vec3& position, const Quat& rotation) {
        if (nodes.empty()) return;
        Mat3 rot_mat = Mat3::from_quat(rotation);
        for (auto& node : nodes) {
            node.box.center = Vec3::add(position, rot_mat.transform_vec(node.box.center));
            node.box.orientation = Quat::multiply(rotation, node.box.orientation);
            node.box.update_cache();
        }
    }

    int get_root() const { return root_index; }
    const OBBTreeNode& get_node(int index) const { return nodes[index]; }
    int get_node_count() const { return static_cast<int>(nodes.size()); }

    // Query: returns true if any leaf in the tree overlaps the given OBB.
    // Uses deterministic traversal order (left before right).
    bool query_overlap(const OBB& query) const {
        if (root_index < 0) return false;
        return query_recursive(root_index, query);
    }

private:
    std::vector<OBBTreeNode> nodes;
    int root_index = -1;

    // Recursively builds the tree. Returns the index of the (new or existing) node
    // that represents the subtree covering leaf_indices[start..end].
    int build_recursive(const std::vector<int>& leaf_indices, int start, int end) {
        if (start == end) {
            // Single leaf — node already exists at leaf_indices[start]
            return leaf_indices[start];
        }

        // Find combined AABB of all leaves in this range to determine split axis
        Vec3 mn = nodes[leaf_indices[start]].box.center;
        Vec3 mx = mn;
        for (int i = start + 1; i <= end; ++i) {
            Vec3 c = nodes[leaf_indices[i]].box.center;
            mn.x = std::min(mn.x, c.x); mn.y = std::min(mn.y, c.y); mn.z = std::min(mn.z, c.z);
            mx.x = std::max(mx.x, c.x); mx.y = std::max(mx.y, c.y); mx.z = std::max(mx.z, c.z);
        }
        Vec3 size = Vec3::sub(mx, mn);

        // Split on longest axis (deterministic tie-break: X > Y > Z)
        int split_axis = 0;
        if (size.y > size.x) split_axis = 1;
        if (size.z > size.x && size.z > size.y) split_axis = 2;

        // Sort a copy of the leaf index range by center on the split axis
        int range_count = end - start + 1;
        std::vector<int> sorted(leaf_indices.begin() + start, leaf_indices.begin() + end + 1);
        std::stable_sort(sorted.begin(), sorted.end(), [&](int a, int b) {
            float av = (split_axis == 0) ? nodes[a].box.center.x
                      : (split_axis == 1) ? nodes[a].box.center.y
                      : nodes[a].box.center.z;
            float bv = (split_axis == 0) ? nodes[b].box.center.x
                      : (split_axis == 1) ? nodes[b].box.center.y
                      : nodes[b].box.center.z;
            return av < bv;
        });

        // Split at midpoint
        int mid = range_count / 2;

        // Recursively build children
        int left_child = build_recursive(sorted, 0, mid - 1);
        int right_child = build_recursive(sorted, mid, range_count - 1);

        // Create internal node that wraps both children
        OBBTreeNode internal;
        internal.hull_index = -1;
        internal.left_child = left_child;
        internal.right_child = right_child;
        int internal_idx = static_cast<int>(nodes.size());
        nodes.push_back(internal);

        // Calculate parent OBB from children (AABB enclosing both child OBBs)
        combine_obb(nodes[left_child].box, nodes[right_child].box, nodes[internal_idx].box);

        return internal_idx;
    }

    void combine_obb(const OBB& a, const OBB& b, OBB& out) {
        // Use AABB enclosing both child OBBs for the parent node.
        // A true merged OBB via covariance matrix is more expensive and rarely worth it
        // for game physics trees. AABB parent is standard practice.
        Vec3 a_max = Vec3::add(a.center, a.extents);
        Vec3 a_min = Vec3::sub(a.center, a.extents);
        Vec3 b_max = Vec3::add(b.center, b.extents);
        Vec3 b_min = Vec3::sub(b.center, b.extents);

        Vec3 combined_min(
            std::min(a_min.x, b_min.x),
            std::min(a_min.y, b_min.y),
            std::min(a_min.z, b_min.z)
        );
        Vec3 combined_max(
            std::max(a_max.x, b_max.x),
            std::max(a_max.y, b_max.y),
            std::max(a_max.z, b_max.z)
        );

        out.center = Vec3::scale(Vec3::add(combined_min, combined_max), 0.5f);
        out.extents = Vec3::scale(Vec3::sub(combined_max, combined_min), 0.5f);
        out.orientation = Quat::identity();
        out.update_cache();
    }

    bool query_recursive(int node_idx, const OBB& query) const {
        const OBBTreeNode& node = nodes[node_idx];

        // Broad check: does this node's OBB overlap the query?
        if (!obb_intersect(node.box, query)) return false;

        // Leaf node — overlap confirmed
        if (node.hull_index >= 0) return true;

        // Internal node — check children in deterministic order (left first)
        if (node.left_child >= 0 && query_recursive(node.left_child, query)) return true;
        if (node.right_child >= 0 && query_recursive(node.right_child, query)) return true;
        return false;
    }
};

} // namespace apc
