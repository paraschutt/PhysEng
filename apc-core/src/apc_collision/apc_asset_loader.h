#pragma once
// =============================================================================
// Asset Loader — Binary .apccol Convex Asset Format
// =============================================================================
//
// Loads convex collision assets produced by the V-HACD CLI tool.
//
// Binary format (little-endian, packed):
//   Header:    4 bytes magic "APCC" | 2 bytes version (uint16) | 2 bytes hull_count (uint16)
//   Per hull:  4 bytes vert_count (uint32) | 4 bytes tri_count (uint32)
//              vert_count * 12 bytes (3 floats per vertex: x, y, z)
//              tri_count * 12 bytes (3 uint32 per triangle: a, b, c)
//
// Only vertex data is loaded into ConvexPiece (GJK/EPA operate on vertex clouds;
// triangle connectivity is skipped).
//
// =============================================================================

#include "apc_collision/apc_convex_asset.h"
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

namespace apc {

struct AssetLoader {
    static constexpr uint32_t MAGIC = 0x43435041u; // "APCC" in little-endian

    // -----------------------------------------------------------------------
    // load_apccol — Load from a file on disk.
    // Returns true on success. Populates out_asset with ConvexPiece entries.
    // -----------------------------------------------------------------------
    static bool load_apccol(const char* filepath, ConvexAsset& out_asset) {
        out_asset.pieces.clear();

        std::FILE* f = std::fopen(filepath, "rb");
        if (!f) return false;

        // Read header
        uint32_t magic = 0u;
        uint16_t version = 0u;
        uint16_t hull_count = 0u;

        if (std::fread(&magic, 4u, 1u, f) != 1u) { std::fclose(f); return false; }
        if (std::fread(&version, 2u, 1u, f) != 1u) { std::fclose(f); return false; }
        if (std::fread(&hull_count, 2u, 1u, f) != 1u) { std::fclose(f); return false; }

        if (magic != MAGIC) { std::fclose(f); return false; }
        if (version != 1u) { std::fclose(f); return false; }

        // Read each hull
        for (uint16_t h = 0u; h < hull_count; ++h) {
            uint32_t vert_count = 0u;
            uint32_t tri_count = 0u;

            if (std::fread(&vert_count, 4u, 1u, f) != 1u) { std::fclose(f); return false; }
            if (std::fread(&tri_count, 4u, 1u, f) != 1u) { std::fclose(f); return false; }

            ConvexPiece piece;
            piece.vertices.resize(vert_count);

            // Read vertices (3 floats each)
            for (uint32_t v = 0u; v < vert_count; ++v) {
                float xyz[3] = {0.0f, 0.0f, 0.0f};
                if (std::fread(xyz, sizeof(float), 3u, f) != 3u) {
                    std::fclose(f);
                    return false;
                }
                piece.vertices[v] = Vec3(xyz[0], xyz[1], xyz[2]);
            }

            // Skip triangle indices (not needed for GJK/EPA vertex-based collision)
            if (tri_count > 0u) {
                long skip = static_cast<long>(tri_count) * 3L * static_cast<long>(sizeof(uint32_t));
                if (std::fseek(f, skip, SEEK_CUR) != 0) {
                    std::fclose(f);
                    return false;
                }
            }

            out_asset.pieces.push_back(piece);
        }

        std::fclose(f);
        return true;
    }

    // -----------------------------------------------------------------------
    // load_apccol_memory — Load from an in-memory byte buffer.
    // data: pointer to the raw binary blob.
    // size: size of the blob in bytes.
    // Returns true on success.
    // -----------------------------------------------------------------------
    static bool load_apccol_memory(const uint8_t* data, uint32_t size, ConvexAsset& out_asset) {
        out_asset.pieces.clear();

        if (size < 8u) return false;

        // Parse header using memcpy (avoids alignment issues)
        uint32_t magic = 0u;
        uint16_t version = 0u;
        uint16_t hull_count = 0u;

        std::memcpy(&magic, data, 4u);
        std::memcpy(&version, data + 4u, 2u);
        std::memcpy(&hull_count, data + 6u, 2u);

        if (magic != MAGIC) return false;
        if (version != 1u) return false;

        uint32_t offset = 8u;

        for (uint16_t h = 0u; h < hull_count; ++h) {
            if (offset + 8u > size) return false;

            uint32_t vert_count = 0u;
            uint32_t tri_count = 0u;

            std::memcpy(&vert_count, data + offset, 4u);
            std::memcpy(&tri_count, data + offset + 4u, 4u);
            offset += 8u;

            uint32_t vert_bytes = vert_count * 3u * static_cast<uint32_t>(sizeof(float));
            uint32_t tri_bytes  = tri_count * 3u * static_cast<uint32_t>(sizeof(uint32_t));

            if (offset + vert_bytes + tri_bytes > size) return false;

            ConvexPiece piece;
            piece.vertices.resize(vert_count);

            for (uint32_t v = 0u; v < vert_count; ++v) {
                float xyz[3];
                std::memcpy(xyz, data + offset + v * 12u, 12u);
                piece.vertices[v] = Vec3(xyz[0], xyz[1], xyz[2]);
            }

            offset += vert_bytes + tri_bytes;
            out_asset.pieces.push_back(piece);
        }

        return true;
    }
};

} // namespace apc
