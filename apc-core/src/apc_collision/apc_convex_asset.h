#pragma once
#include "apc_math/apc_vec3.h"
#include <vector>

namespace apc {

// Single convex hull from a decomposition pipeline (e.g., V-HACD output).
// Stores the vertices of one convex piece of a collision mesh.
struct ConvexHull {
    std::vector<Vec3> vertices;
};

// A collision asset composed of multiple convex hulls.
// Represents the full convex decomposition of a mesh (e.g., a character limb).
// The OBBTree is built over these hulls for midphase acceleration.
struct ConvexAsset {
    std::vector<ConvexHull> hulls;

    int get_hull_count() const { return static_cast<int>(hulls.size()); }

    const ConvexHull& get_hull(int index) const {
        return hulls[index];
    }
};

} // namespace apc
