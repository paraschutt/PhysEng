#pragma once
// =============================================================================
// apc_ai_influence_map.h — Semantic Influence Map for AI Spatial Awareness
// =============================================================================
//
// Phase 11b Action 7: Influence Maps
//
// A low-resolution (32x16) grid that accumulates spatial influence from all
// athletes on the field. The AI queries this map to find the best open space,
// naturally avoiding opponents AND respecting semantic field rules from
// SportField::get_zone_at() (Phase 11a Action 2).
//
// The true power of the Influence Map isn't just knowing where defenders are —
// it's understanding the RULES of the space. By feeding SportField semantics
// into find_best_open_space(), the AI won't just avoid opponents; it will
// natively avoid running OUT_OF_BOUNDS or lingering in RESTRICTED_DEFENSE.
//
// Key types:
//   - InfluenceMap: threat/control dual-grid with semantic-aware pathfinding
//
// Integration with Perception (Action 6):
//   The map is populated from DELAYED perception snapshots, ensuring the AI's
//   spatial awareness matches its delayed visual perception. This prevents the
//   AI from "knowing" where opponents are before it could realistically see them.
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation (fixed-size arrays, embedded in class)
//   - Deterministic (fixed iteration order, no heap)
//   - C++17
//
// =============================================================================

#include "../apc_math/apc_vec3.h"
#include "../apc_sport/apc_sport_field.h"
#include <cstdint>

namespace apc {

class InfluenceMap {
public:
    // Low-res grid keeps the CPU footprint minuscule
    // 32x16 = 512 cells, each traversed in <1us total
    static constexpr int32_t GRID_WIDTH  = 32;
    static constexpr int32_t GRID_HEIGHT = 16;

private:
    float threat_grid[GRID_WIDTH][GRID_HEIGHT] = {};
    float control_grid[GRID_WIDTH][GRID_HEIGHT] = {};

    // Field extents cached from SportField for coordinate mapping
    // field_ext_x = total X extent (geometry.length, e.g. 105m for soccer)
    // field_ext_z = total Z extent (geometry.width, e.g. 68m for soccer)
    float field_ext_x = 0.0f;
    float field_ext_z = 0.0f;

public:
    // =========================================================================
    // initialize — Cache field dimensions for world <-> grid coordinate mapping
    // =========================================================================
    // Must be called once before inject_influence() or find_best_open_space().
    // Stores the field's X and Z extents so the grid can map world positions
    // to cell indices and back without recomputing halved extents each call.
    //
    // Note: field_ext_x maps to X world axis (geometry.length, long axis),
    //       field_ext_z maps to Z world axis (geometry.width, short axis).
    // =========================================================================
    void initialize(const SportField& field) {
        field_ext_x = field.geometry.length;
        field_ext_z = field.geometry.width;
    }

    // =========================================================================
    // clear — Zero all grid cells
    // =========================================================================
    // Called at the start of each AI tick before injecting new influence.
    // Uses flat iteration order for cache-friendly access and deterministic
    // clearing regardless of compiler optimization level.
    // =========================================================================
    void clear() {
        for (int x = 0; x < GRID_WIDTH; ++x) {
            for (int z = 0; z < GRID_HEIGHT; ++z) {
                threat_grid[x][z]  = 0.0f;
                control_grid[x][z] = 0.0f;
            }
        }
    }

    // =========================================================================
    // inject_influence — Paint a radial influence footprint onto the grid
    // =========================================================================
    // Converts a world position to grid coordinates, then stamps a radial
    // falloff pattern onto the target grid (threat or control).
    //
    // Parameters:
    //   world_pos   — Center of influence in world space
    //   intensity   — Strength of influence (1.0 = standard athlete)
    //   radius_m    — Radius of influence in meters (e.g., 8.0m for spatial)
    //   is_threat   — true = opponent influence, false = friendly control
    //
    // Falloff: linear from 1.0 at center to 0.0 at radius_m.
    // Only cells within radius_m are touched — O(cells_touched) not O(GRID^2).
    // =========================================================================
    void inject_influence(const Vec3& world_pos, float intensity, float radius_m,
                          bool is_threat) {
        if (field_ext_x <= 0.0f || field_ext_z <= 0.0f) return;

        float half_ext_x = field_ext_x * 0.5f;
        float half_ext_z = field_ext_z * 0.5f;

        // World -> normalized [0, 1] -> grid cell
        float nx = (world_pos.x + half_ext_x) / field_ext_x;
        float nz = (world_pos.z + half_ext_z) / field_ext_z;

        int center_x = static_cast<int>(nx * GRID_WIDTH);
        int center_z = static_cast<int>(nz * GRID_HEIGHT);

        // Convert radius from meters to grid cells
        int cell_radius_x = static_cast<int>((radius_m / field_ext_x) * GRID_WIDTH) + 1;
        int cell_radius_z = static_cast<int>((radius_m / field_ext_z) * GRID_HEIGHT) + 1;

        auto& target_grid = is_threat ? threat_grid : control_grid;

        for (int x = center_x - cell_radius_x; x <= center_x + cell_radius_x; ++x) {
            for (int z = center_z - cell_radius_z; z <= center_z + cell_radius_z; ++z) {
                if (x >= 0 && x < GRID_WIDTH && z >= 0 && z < GRID_HEIGHT) {
                    // Convert cell offset back to world-space distance
                    float dx = static_cast<float>(x - center_x) * (field_ext_x / GRID_WIDTH);
                    float dz = static_cast<float>(z - center_z) * (field_ext_z / GRID_HEIGHT);
                    float dist_sq = (dx * dx) + (dz * dz);
                    float radius_sq = radius_m * radius_m;

                    if (dist_sq < radius_sq) {
                        float falloff = 1.0f - (dist_sq / radius_sq);
                        target_grid[x][z] += intensity * falloff;
                    }
                }
            }
        }
    }

    // =========================================================================
    // find_best_open_space — Find the safest cell near a position
    // =========================================================================
    // Searches within search_radius_m of current_pos for the cell with the
    // highest combined score: (Our Control - 1.5x Enemy Threat - Rule Penalty).
    //
    // Semantic field rules (Phase 11a Action 2) are applied as penalties:
    //   - OUT_OF_BOUNDS:       cell is skipped entirely (invalid space)
    //   - RESTRICTED_DEFENSE:  -5.0 penalty (avoid clustering in the box/crease)
    //   - RESTRICTED_OFFENSE:  no penalty (attacker might legitimately be here)
    //   - SCORING_TARGET:      no penalty (high-value destination)
    //
    // This means the AI natively understands field boundaries and restricted
    // zones through the same semantic system used by physics and rules.
    //
    // Parameters:
    //   current_pos     — Athlete's current world position
    //   search_radius_m — Search window in meters (e.g., 15.0m)
    //   field           — SportField reference for semantic zone queries
    //
    // Returns:
    //   World position of the best open-space cell (or current_pos if no better)
    // =========================================================================
    Vec3 find_best_open_space(const Vec3& current_pos, float search_radius_m,
                              const SportField& field) const {
        if (field_ext_x <= 0.0f || field_ext_z <= 0.0f) {
            return current_pos;
        }

        float half_ext_x = field_ext_x * 0.5f;
        float half_ext_z = field_ext_z * 0.5f;

        float nx = (current_pos.x + half_ext_x) / field_ext_x;
        float nz = (current_pos.z + half_ext_z) / field_ext_z;

        int center_x = static_cast<int>(nx * GRID_WIDTH);
        int center_z = static_cast<int>(nz * GRID_HEIGHT);
        int search_cells = static_cast<int>((search_radius_m / field_ext_x) * GRID_WIDTH) + 1;

        float best_score = -9999.0f;
        int best_x = center_x;
        int best_z = center_z;

        for (int x = center_x - search_cells; x <= center_x + search_cells; ++x) {
            for (int z = center_z - search_cells; z <= center_z + search_cells; ++z) {
                if (x >= 0 && x < GRID_WIDTH && z >= 0 && z < GRID_HEIGHT) {

                    // Convert grid cell back to world position for semantic query
                    Vec3 cell_world_pos;
                    cell_world_pos.x = (static_cast<float>(x) / GRID_WIDTH) * field_ext_x - half_ext_x;
                    cell_world_pos.y = current_pos.y;
                    cell_world_pos.z = (static_cast<float>(z) / GRID_HEIGHT) * field_ext_z - half_ext_z;

                    // Apply semantic penalties from SportField
                    ZoneSemantic semantic = field.get_zone_at(cell_world_pos);
                    if (semantic == ZoneSemantic::OUT_OF_BOUNDS) continue; // Invalid space

                    float semantic_penalty = 0.0f;
                    if (semantic == ZoneSemantic::RESTRICTED_DEFENSE) {
                        semantic_penalty = 5.0f; // High penalty to avoid clustering in the crease/box
                    }

                    // Score = Our Control - Enemy Threat (weighted 1.5x) - Rule Penalties
                    float score = control_grid[x][z] - (threat_grid[x][z] * 1.5f) - semantic_penalty;

                    if (score > best_score) {
                        best_score = score;
                        best_x = x;
                        best_z = z;
                    }
                }
            }
        }

        // Convert best cell back to world coordinates
        Vec3 target_world;
        target_world.x = (static_cast<float>(best_x) / GRID_WIDTH) * field_ext_x - half_ext_x;
        target_world.y = current_pos.y;
        target_world.z = (static_cast<float>(best_z) / GRID_HEIGHT) * field_ext_z - half_ext_z;
        return target_world;
    }

    // =========================================================================
    // query_raw — Read raw threat/control values at a world position
    // =========================================================================
    // Direct access to grid values for advanced AI reasoning (e.g., "am I
    // standing in a high-threat zone right now?").
    //
    // Returns false if the position maps outside the grid bounds.
    // =========================================================================
    bool query_raw(const Vec3& world_pos, float& out_threat, float& out_control) const {
        if (field_ext_x <= 0.0f || field_ext_z <= 0.0f) return false;

        float half_ext_x = field_ext_x * 0.5f;
        float half_ext_z = field_ext_z * 0.5f;

        float nx = (world_pos.x + half_ext_x) / field_ext_x;
        float nz = (world_pos.z + half_ext_z) / field_ext_z;

        int gx = static_cast<int>(nx * GRID_WIDTH);
        int gz = static_cast<int>(nz * GRID_HEIGHT);

        if (gx < 0 || gx >= GRID_WIDTH || gz < 0 || gz >= GRID_HEIGHT) {
            return false;
        }

        out_threat  = threat_grid[gx][gz];
        out_control = control_grid[gx][gz];
        return true;
    }
};

} // namespace apc
