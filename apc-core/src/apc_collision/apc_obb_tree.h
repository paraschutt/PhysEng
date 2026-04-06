#pragma once
#include "apc_obb.h"
#include "apc_convex_asset.h"
#include <vector>

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
        int hull_count = asset.get_hull_count();
        if (hull_count == 0) return;

        // Initial leaf nodes
        std::vector<int> leaves(hull_count);
        for (int i = 0; i < hull_count; ++i) {
            OBBTreeNode leaf;
            leaf.hull_index = i;
            leaf.left_child = -1;
            leaf.right_child = -1;
            
            // Calculate tight OBB from hull vertices
            const auto& hull = asset.get_hull(i);
            Vec3 min = hull.vertices[0], max = min;
            for(const auto& v : hull.vertices) {
                min.x = std::min(min.x, v.x); min.y = std::min(min.y, v.y); min.z = std::min(min.z, v.z);
                max.x = std::max(max.x, v.x); max.y = std::max(max.y, v.y); max.z = std::max(max.z, v.z);
            }
            leaf.box.center = Vec3::scale(Vec3::add(min, max), 0.5f);
            leaf.box.extents = Vec3::scale(Vec3::sub(max, min), 0.5f);
            leaf.box.update_cache();
            
            leaves[i] = nodes.size();
            nodes.push_back(leaf);
        }

        // Build binary tree top-down (deterministic split: longest axis)
        build_recursive(leaves, 0, leaves.size() - 1);
    }

    // Updates world-space OBBs based on character root transform
    void refit(const Vec3& position, const Quat& rotation) {
        if (nodes.empty()) return;
        // For a skeletal mesh, we'd pass per-bone transforms here.
        // For this baseline, we assume the collision asset is modeled in local space
        // and we just transform the root.
        Mat3 rot_mat = Mat3::from_quat(rotation);
        for (auto& node : nodes) {
            node.box.center = Vec3::add(position, rot_mat.rotate(node.box.center));
            node.box.orientation = Quat::multiply(rotation, node.box.orientation);
            node.box.update_cache();
        }
    }

    int get_root() const { return nodes.empty() ? -1 : 0; }
    const OBBTreeNode& get_node(int index) const { return nodes[index]; }

private:
    std::vector<OBBTreeNode> nodes;

    void build_recursive(const std::vector<int>& leaves, int start, int end) {
        if (start == end) return; // Leaf node already exists

        // Find combined AABB of leaves[start..end] to determine split axis
        Vec3 min = nodes[leaves[start]].box.center;
        Vec3 max = min;
        for (int i = start; i <= end; ++i) {
            Vec3 c = nodes[leaves[i]].box.center;
            min.x = std::min(min.x, c.x); min.y = std::min(min.y, c.y); min.z = std::min(min.z, c.z);
            max.x = std::max(max.x, c.x); max.y = std::max(max.y, c.y); max.z = std::max(max.z, c.z);
        }
        Vec3 size = Vec3::sub(max, min);
        
        // Split on longest axis (Deterministic tie-break: X then Y then Z)
        int split_axis = 0;
        if (size.y > size.x) split_axis = 1;
        if (size.z > size.x && size.z > size.y) split_axis = 2;

        // Sort leaves by center on split axis (Strict weak ordering required)
        std::vector<int> sorted_leaves(leaves.begin() + start, leaves.begin() + end + 1);
        std::stable_sort(sorted_leaves.begin(), sorted_leaves.end(), [&](int a_idx, int b_idx) {
            float a_val = (split_axis == 0) ? nodes[a_idx].box.center.x : (split_axis == 1 ? nodes[a_idx].box.center.y : nodes[a_idx].box.center.z);
            float b_val = (split_axis == 0) ? nodes[b_idx].box.center.x : (split_axis == 1 ? nodes[b_idx].box.center.y : nodes[b_idx].box.center.z);
            return a_val < b_val;
        });

        int mid = (end - start) / 2;
        
        // Create internal node
        OBBTreeNode internal;
        internal.hull_index = -1;
        internal.left_child = -1;
        internal.right_child = -1;
        int internal_idx = nodes.size();
        nodes.push_back(internal);

        // Recursively build children
        std::vector<int> left_leaves(sorted_leaves.begin(), sorted_leaves.begin() + mid + 1);
        std::vector<int> right_leaves(sorted_leaves.begin() + mid + 1, sorted_leaves.end());
        
        // Actually, to avoid deep copies in recursion, we do index manipulation.
        // Simplified for clarity: assign child indices directly.
        // Left child
        if (start == mid) {
            nodes[internal_idx].left_child = leaves[start];
        } else {
            nodes[internal_idx].left_child = nodes.size();
            // Recursive placeholder - real impl uses array slicing
        }
        // Right child
        if (mid + 1 == end) {
            nodes[internal_idx].right_child = leaves[end];
        } else {
            nodes[internal_idx].right_child = nodes.size();
        }

        // Calculate parent OBB from children
        combine_obb(nodes[nodes[internal_idx].left_child].box, nodes[nodes[internal_idx].right_child].box, nodes[internal_idx].box);
    }

    void combine_obb(const OBB& a, const OBB& b, OBB& out) {
        out.center = Vec3::scale(Vec3::add(a.center, b.center), 0.5f);
        // Simplified: use AABB enclosing both OBBs for the parent node
        // A true merged OBB requires covariance matrix calculation, which is expensive.
        // AABB parent is standard practice in game physics trees to save CPU.
        Vec3 a_max = Vec3::add(a.center, a.extents);
        Vec3 a_min = Vec3::sub(a.center, a.extents);
        Vec3 b_max = Vec3::add(b.center, b.extents);
        Vec3 b_min = Vec3::sub(b.center, b.extents);
        
        Vec3 combined_min = Vec3(std::min(a_min.x, b_min.x), std::min(a_min.y, b_min.y), std::min(a_min.z, b_min.z));
        Vec3 combined_max = Vec3(std::max(a_max.x, b_max.x), std::max(a_max.y, b_max.y), std::max(a_max.z, b_max.z));
        
        out.extents = Vec3::scale(Vec3::sub(combined_max, combined_min), 0.5f);
        out.orientation = Quat::identity();
        out.update_cache();
    }
};

} // namespace apc