#pragma once
#include "apc_math/apc_vec3.h"
#include <vector>

namespace apc {

// Single convex hull piece from a decomposition pipeline (e.g., V-HACD output).
// Stores the vertices of one convex piece of a collision mesh.
// Named ConvexPiece to avoid clashing with GJK's ConvexHull (support-function wrapper).
struct ConvexPiece {
    std::vector<Vec3> vertices;
};

// A collision asset composed of multiple convex hull pieces.
// Represents the full convex decomposition of a mesh (e.g., a character limb).
// The OBBTree is built over these pieces for midphase acceleration.
struct ConvexAsset {
    std::vector<ConvexPiece> pieces;

    int get_piece_count() const { return static_cast<int>(pieces.size()); }

    const ConvexPiece& get_piece(int index) const {
        return pieces[index];
    }
};

} // namespace apc
