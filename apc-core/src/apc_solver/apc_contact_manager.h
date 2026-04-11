#pragma once
// =============================================================================
// Contact Manager — Persistent contact tracking for solver warmstarting
// =============================================================================
//
// Tracks contact manifolds across simulation frames to enable warmstarting in
// the sequential impulse solver. When body pairs remain in contact, the
// accumulated impulses from the previous frame provide initial guess values
// that dramatically improve solver convergence (typically 2-3x fewer iterations).
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation (fixed-size arrays, stack only)
//   - Deterministic: sorted iteration, index-order tie-breaking, no FMA
//   - C++17, GCC + MSVC cross-platform
//
// Integration with Solver3D:
//   After calling ContactManager::update() with the current frame's collision
//   manifolds, query get_persistent(id_a, id_b) to retrieve the persistent
//   manifold. Use its accumulated impulses as warmstart values in
//   VelocityConstraint::accumulated_normal_impulse and
//   VelocityConstraint::accumulated_friction_impulse.
//
// =============================================================================

#include "apc_math/apc_math_common.h"
#include "apc_math/apc_vec3.h"
#include "apc_collision/apc_collision_dispatch.h"
#include "apc_collision/apc_sphere_sphere.h"

#include <cstdint>
#include <cstring>

namespace apc {

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Scale factor applied to inherited accumulated impulses on warmstart.
/// 0.0 = cold start (ignore previous), 1.0 = full inheritance.
static constexpr float APC_WARMSTART_FACTOR = 0.8f;

/// Match threshold: contacts within this distance are considered the same
/// contact point across frames (world units, compared as squared distance).
static constexpr float APC_CONTACT_MATCH_DISTANCE = 0.1f;
static constexpr float APC_CONTACT_MATCH_DISTANCE_SQ =
    APC_CONTACT_MATCH_DISTANCE * APC_CONTACT_MATCH_DISTANCE;

/// Sentinel value for "manifold not found" in find_manifold.
static constexpr uint32_t APC_INVALID_INDEX = 0xFFFFFFFFu;

// ---------------------------------------------------------------------------
// Internal detail helpers
// ---------------------------------------------------------------------------
namespace detail {

/// Squared distance between two points. No FMA — explicit multiply-add.
APC_FORCEINLINE float contact_dist_sq(const Vec3& a, const Vec3& b) {
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    const float dz = a.z - b.z;
    const float xx = dx * dx;
    const float yy = dy * dy;
    const float zz = dz * dz;
    return xx + yy + zz;
}

/// Normalize pair so that lo <= hi. Deterministic swap via temp variable.
APC_FORCEINLINE void normalize_pair(uint32_t& a, uint32_t& b) {
    if (a > b) {
        const uint32_t tmp = a;
        a = b;
        b = tmp;
    }
}

/// Pack (lo, hi) into a uint64 sort key. lo occupies the high 32 bits so
/// that sorting by key first orders by lo, then by hi.
APC_FORCEINLINE uint64_t pair_sort_key(uint32_t lo, uint32_t hi) {
    return (static_cast<uint64_t>(lo) << 32) | static_cast<uint64_t>(hi);
}

/// Zero-initialize a struct. Uses assignment rather than memset to
/// handle non-trivial types (e.g., ContactPoint containing Vec3 with
/// constructors). Each field must be set explicitly or via default init.
template<typename T>
APC_FORCEINLINE void zero_struct(T& out) {
    out = T{};
}

} // namespace detail

// =============================================================================
// PersistentContact
// =============================================================================
//
// A contact point that persists across frames, carrying accumulated solver
// impulses for warmstarting. Each contact is assigned a unique ID for
// debugging and inspection (does not affect physics determinism).
//
// =============================================================================
struct PersistentContact {
    uint32_t  contact_id;                    ///< Unique ID for this contact
    ContactPoint contact;                    ///< Contact geometry data
    float     accumulated_normal_impulse;    ///< Normal impulse from prev frame
    Vec3      accumulated_friction_impulse;  ///< Friction impulse from prev frame
    uint32_t  age;                           ///< Frames this contact has been active

    /// Reset to default zero state.
    APC_FORCEINLINE void reset() {
        detail::zero_struct(*this);
    }
};

// =============================================================================
// PersistentManifold
// =============================================================================
//
// A collision manifold that persists across frames. Tracks up to
// MAX_CONTACTS_PER_PAIR contact points with their accumulated impulses.
// Pair IDs are normalized: id_a <= id_b.
//
// =============================================================================
struct PersistentManifold {
    uint32_t id_a;            ///< Lower body ID (normalized)
    uint32_t id_b;            ///< Higher body ID (normalized)
    PersistentContact contacts[MAX_CONTACTS_PER_PAIR];
    uint32_t contact_count;

    /// Internal counter for generating contact IDs via add_or_update.
    uint32_t next_contact_id;

    /// Reset to empty state.
    APC_FORCEINLINE void reset() {
        id_a = 0u;
        id_b = 0u;
        contact_count = 0u;
        next_contact_id = 0u;
        for (uint32_t i = 0u; i < MAX_CONTACTS_PER_PAIR; ++i) {
            contacts[i].reset();
        }
    }

    // -----------------------------------------------------------------------
    // add_or_update
    // -----------------------------------------------------------------------
    /// Insert a new contact or merge with an existing one by proximity.
    ///
    /// Scans existing contacts in index order. The first existing contact
    /// within distance_threshold of new_contact (by point_on_a) is updated:
    ///   - Position: averaged (lerp at t=0.5)
    ///   - Normal: replaced with new value (latest collision result)
    ///   - Penetration: max of old and new
    ///   - Age: incremented
    ///   - Accumulated impulses: preserved (NOT scaled — caller must scale
    ///     if desired, e.g. via ContactManager::update)
    ///
    /// If no match is found and room exists, a new contact is appended with
    /// zero impulses and age 0.
    ///
    /// Deterministic: index-order scan, first match wins.
    // -----------------------------------------------------------------------
    APC_FORCEINLINE void add_or_update(const ContactPoint& new_contact,
                                       float distance_threshold) {
        // Search for closest existing contact by point_on_a distance
        float best_dist_sq = 1e30f;  // Effectively FLT_MAX, no <limits> needed
        uint32_t best_idx = MAX_CONTACTS_PER_PAIR;

        for (uint32_t i = 0u; i < contact_count; ++i) {
            const float d = detail::contact_dist_sq(
                new_contact.point_on_a,
                contacts[i].contact.point_on_a
            );
            if (d < best_dist_sq) {
                best_dist_sq = d;
                best_idx = i;
            }
        }

        // Compute squared threshold. Explicit multiply — no FMA.
        const float threshold_sq = distance_threshold * distance_threshold;

        if (best_idx < contact_count && best_dist_sq < threshold_sq) {
            // ---- Match found: merge with existing ----
            PersistentContact& existing = contacts[best_idx];

            // Average positions via lerp(0.5). Vec3::lerp does:
            //   a + (b - a) * t  — two operations, no FMA.
            existing.contact.point_on_a = Vec3::lerp(
                existing.contact.point_on_a,
                new_contact.point_on_a,
                0.5f
            );
            existing.contact.point_on_b = Vec3::lerp(
                existing.contact.point_on_b,
                new_contact.point_on_b,
                0.5f
            );

            // Use the latest normal from collision detection
            existing.contact.normal = new_contact.normal;

            // Keep the deeper penetration for stability
            if (new_contact.penetration > existing.contact.penetration) {
                existing.contact.penetration = new_contact.penetration;
            }

            // Advance age
            existing.age = existing.age + 1u;

        } else if (contact_count < MAX_CONTACTS_PER_PAIR) {
            // ---- No match: insert new contact ----
            PersistentContact& slot = contacts[contact_count];
            slot.contact_id = next_contact_id;
            next_contact_id = next_contact_id + 1u;
            slot.contact = new_contact;
            slot.accumulated_normal_impulse = 0.0f;
            slot.accumulated_friction_impulse = Vec3(0.0f, 0.0f, 0.0f);
            slot.age = 0u;
            contact_count = contact_count + 1u;
        }
        // else: manifold full, new contact silently dropped
    }

    // -----------------------------------------------------------------------
    // prune
    // -----------------------------------------------------------------------
    /// Remove outlier contacts whose point_on_a is too far from the centroid
    /// of all contacts in this manifold.
    ///
    /// This is useful for cleaning up stale contacts when the collision
    /// geometry has shifted significantly (e.g., sliding, rotation).
    ///
    /// With 0 or 1 contacts, nothing is pruned (no meaningful centroid).
    /// Deterministic: contacts are compacted in index order.
    // -----------------------------------------------------------------------
    APC_FORCEINLINE void prune(float max_distance) {
        if (contact_count <= 1u) {
            return;
        }

        // Compute centroid of all point_on_a positions.
        // Explicit accumulation — no FMA.
        Vec3 centroid(0.0f, 0.0f, 0.0f);
        for (uint32_t i = 0u; i < contact_count; ++i) {
            centroid = Vec3::add(centroid, contacts[i].contact.point_on_a);
        }
        const float inv_count = 1.0f / static_cast<float>(contact_count);
        centroid = Vec3::scale(centroid, inv_count);

        // Compact in-place: keep contacts within max_distance of centroid.
        const float max_dist_sq = max_distance * max_distance;
        uint32_t write = 0u;
        for (uint32_t read = 0u; read < contact_count; ++read) {
            const float d = detail::contact_dist_sq(
                contacts[read].contact.point_on_a,
                centroid
            );
            if (d <= max_dist_sq) {
                if (write != read) {
                    // Copy via memcpy — safe for POD-like structs
                    std::memcpy(&contacts[write], &contacts[read],
                                sizeof(PersistentContact));
                }
                write = write + 1u;
            }
        }

        // Zero remaining slots (optional, but prevents stale data leakage)
        for (uint32_t i = write; i < contact_count; ++i) {
            contacts[i].reset();
        }

        contact_count = write;
    }
};

// =============================================================================
// ContactManager
// =============================================================================
//
// Manages persistent contact manifolds for all active body pairs in the
// simulation. Each frame, call update() with the new set of collision
// manifolds. The manager matches new contacts against previous-frame contacts
// by proximity and carries forward accumulated impulses for warmstarting.
//
// Memory: fixed-size array of MAX_PAIRS PersistentManifolds. No allocation.
// Determinism: manifolds are sorted by (id_a, id_b) after each update.
//
// =============================================================================
class ContactManager {
public:
    static constexpr uint32_t MAX_PAIRS = 256u;

    // -----------------------------------------------------------------------
    // update
    // -----------------------------------------------------------------------
    /// Process the current frame's collision manifolds, matching against
    /// persisted state from the previous frame.
    ///
    /// Algorithm:
    ///   1. For each new manifold, normalize pair order (lo, hi).
    ///   2. Search for a matching persistent manifold.
    ///   3. If found: match each new contact against existing contacts by
    ///      point_on_a proximity. Matched contacts inherit impulses scaled
    ///      by APC_WARMSTART_FACTOR. Unmatched new contacts start at zero.
    ///   4. If not found: create a new persistent manifold with zero impulses.
    ///   5. Remove persistent manifolds not matched by any new manifold.
    ///   6. Sort by pair key for deterministic iteration.
    ///
    /// Complexity: O(N * M * C^2) where N = new manifolds, M = MAX_PAIRS,
    /// C = MAX_CONTACTS_PER_PAIR. For typical values this is negligible.
    // -----------------------------------------------------------------------
    void update(const ContactManifold* new_manifolds, uint32_t new_count) {
        // Track which existing manifolds were matched by a new manifold
        bool matched[MAX_PAIRS];
        for (uint32_t i = 0u; i < MAX_PAIRS; ++i) {
            matched[i] = false;
        }

        // ---- Phase 1: Process each new manifold ----
        for (uint32_t nm = 0u; nm < new_count; ++nm) {
            const ContactManifold& nm_ref = new_manifolds[nm];

            // Skip empty manifolds (no contacts = separated pair)
            if (nm_ref.contact_count == 0u) {
                continue;
            }

            // Normalize pair order for deterministic lookup
            uint32_t lo = nm_ref.id_a;
            uint32_t hi = nm_ref.id_b;
            detail::normalize_pair(lo, hi);

            // Search for existing persistent manifold
            const uint32_t existing_idx = find_manifold(lo, hi);

            if (existing_idx < manifold_count_) {
                // ---- Existing manifold found: match and merge ----
                matched[existing_idx] = true;
                PersistentManifold& pm = manifolds_[existing_idx];

                // Build merged contact list.
                // We must NOT modify pm.contacts in-place during the loop
                // because subsequent matches would see modified data.
                // Use a temp buffer.
                PersistentContact merged[MAX_CONTACTS_PER_PAIR];
                uint32_t merged_count = 0u;
                bool old_used[MAX_CONTACTS_PER_PAIR];
                for (uint32_t i = 0u; i < MAX_CONTACTS_PER_PAIR; ++i) {
                    old_used[i] = false;
                }

                // Match each new contact against existing contacts
                for (uint32_t nc = 0u; nc < nm_ref.contact_count; ++nc) {
                    const ContactPoint& new_cp = nm_ref.contacts[nc];

                    // Scan existing contacts in index order, pick first match
                    float best_dist_sq = 1e30f;
                    uint32_t best_ec = MAX_CONTACTS_PER_PAIR;

                    for (uint32_t ec = 0u; ec < pm.contact_count; ++ec) {
                        if (old_used[ec]) {
                            continue;
                        }
                        const float d = detail::contact_dist_sq(
                            new_cp.point_on_a,
                            pm.contacts[ec].contact.point_on_a
                        );
                        if (d < best_dist_sq) {
                            best_dist_sq = d;
                            best_ec = ec;
                        }
                    }

                    if (best_ec < pm.contact_count &&
                        best_dist_sq < APC_CONTACT_MATCH_DISTANCE_SQ &&
                        merged_count < MAX_CONTACTS_PER_PAIR)
                    {
                        // ---- Match found: merge and inherit impulses ----
                        old_used[best_ec] = true;
                        const PersistentContact& old = pm.contacts[best_ec];

                        PersistentContact& out = merged[merged_count];
                        out.contact_id = old.contact_id;

                        // Average positions: lerp at t=0.5
                        out.contact.point_on_a = Vec3::lerp(
                            old.contact.point_on_a,
                            new_cp.point_on_a,
                            0.5f
                        );
                        out.contact.point_on_b = Vec3::lerp(
                            old.contact.point_on_b,
                            new_cp.point_on_b,
                            0.5f
                        );

                        // Use the latest normal from collision detection
                        out.contact.normal = new_cp.normal;

                        // Keep the deeper penetration for stability
                        out.contact.penetration = old.contact.penetration;
                        if (new_cp.penetration > old.contact.penetration) {
                            out.contact.penetration = new_cp.penetration;
                        }

                        // Inherit accumulated impulses, scaled by warmstart
                        // factor. Explicit multiply — no FMA.
                        out.accumulated_normal_impulse =
                            old.accumulated_normal_impulse * APC_WARMSTART_FACTOR;
                        out.accumulated_friction_impulse =
                            Vec3::scale(old.accumulated_friction_impulse,
                                        APC_WARMSTART_FACTOR);

                        // Advance age
                        out.age = old.age + 1u;

                        merged_count = merged_count + 1u;

                    } else if (merged_count < MAX_CONTACTS_PER_PAIR) {
                        // ---- No match: new contact, zero impulses ----
                        PersistentContact& out = merged[merged_count];
                        out.contact_id = next_contact_id_;
                        next_contact_id_ = next_contact_id_ + 1u;
                        out.contact = new_cp;
                        out.accumulated_normal_impulse = 0.0f;
                        out.accumulated_friction_impulse =
                            Vec3(0.0f, 0.0f, 0.0f);
                        out.age = 0u;
                        merged_count = merged_count + 1u;
                    }
                    // else: merged array full, contact dropped
                }

                // Write merged contacts back into the persistent manifold
                pm.contact_count = merged_count;
                for (uint32_t i = 0u; i < merged_count; ++i) {
                    pm.contacts[i] = merged[i];
                }
                // Zero remaining slots
                for (uint32_t i = merged_count; i < MAX_CONTACTS_PER_PAIR; ++i) {
                    pm.contacts[i].reset();
                }

            } else {
                // ---- No existing manifold: create new ----
                if (manifold_count_ < MAX_PAIRS) {
                    PersistentManifold& pm =
                        manifolds_[manifold_count_];
                    pm.reset();
                    pm.id_a = lo;
                    pm.id_b = hi;
                    pm.next_contact_id = next_contact_id_;

                    for (uint32_t nc = 0u; nc < nm_ref.contact_count; ++nc) {
                        if (pm.contact_count < MAX_CONTACTS_PER_PAIR) {
                            PersistentContact& slot =
                                pm.contacts[pm.contact_count];
                            slot.contact_id = next_contact_id_;
                            next_contact_id_ = next_contact_id_ + 1u;
                            slot.contact = nm_ref.contacts[nc];
                            slot.accumulated_normal_impulse = 0.0f;
                            slot.accumulated_friction_impulse =
                                Vec3(0.0f, 0.0f, 0.0f);
                            slot.age = 0u;
                            pm.contact_count = pm.contact_count + 1u;
                        }
                    }

                    matched[manifold_count_] = true;
                    manifold_count_ = manifold_count_ + 1u;
                }
                // else: MAX_PAIRS reached, manifold silently dropped
            }
        }

        // ---- Phase 2: Remove unmatched persistent manifolds ----
        // Compact the array in-place.
        uint32_t write = 0u;
        for (uint32_t read = 0u; read < manifold_count_; ++read) {
            if (matched[read]) {
                if (write != read) {
                    std::memcpy(&manifolds_[write], &manifolds_[read],
                                sizeof(PersistentManifold));
                }
                write = write + 1u;
            }
        }
        // Zero removed slots
        for (uint32_t i = write; i < manifold_count_; ++i) {
            manifolds_[i].reset();
        }
        manifold_count_ = write;

        // ---- Phase 3: Sort by pair key for deterministic iteration ----
        sort_manifolds();
    }

    // -----------------------------------------------------------------------
    // get_persistent
    // -----------------------------------------------------------------------
    /// Retrieve the persistent manifold for a body pair.
    /// Returns nullptr if the pair is not currently tracked.
    /// The pair (id_a, id_b) is internally normalized to lo <= hi.
    // -----------------------------------------------------------------------
    APC_FORCEINLINE const PersistentManifold* get_persistent(
        uint32_t id_a, uint32_t id_b) const
    {
        uint32_t lo = id_a;
        uint32_t hi = id_b;
        detail::normalize_pair(lo, hi);

        const uint32_t idx = find_manifold(lo, hi);
        if (idx < manifold_count_) {
            return &manifolds_[idx];
        }
        return nullptr;
    }

    // -----------------------------------------------------------------------
    // get_persistent_mut
    // -----------------------------------------------------------------------
    /// Mutable access to persistent manifold. Used by the solver to write
    /// back accumulated impulses after solving (for next-frame warmstart).
    /// Returns nullptr if the pair is not currently tracked.
    // -----------------------------------------------------------------------
    APC_FORCEINLINE PersistentManifold* get_persistent_mut(
        uint32_t id_a, uint32_t id_b)
    {
        uint32_t lo = id_a;
        uint32_t hi = id_b;
        detail::normalize_pair(lo, hi);

        const uint32_t idx = find_manifold(lo, hi);
        if (idx < manifold_count_) {
            return &manifolds_[idx];
        }
        return nullptr;
    }

    // -----------------------------------------------------------------------
    // apply_warmstart_to_constraint
    // -----------------------------------------------------------------------
    /// Bridge method: extract warmstart impulses from a persistent manifold
    /// and apply them to solver constraint accumulators.
    ///
    /// Call this after ContactManager::update() and before Solver3D::solve()
    /// to initialize each VelocityConstraint's accumulated impulses from the
    /// previous frame's solver output.
    ///
    /// Returns true if a warmstart match was found and impulses were applied.
    // -----------------------------------------------------------------------
    APC_FORCEINLINE bool apply_warmstart_to_constraint(
        uint32_t id_a, uint32_t id_b,
        const Vec3& constraint_point_on_a,
        float& out_normal_impulse,
        Vec3& out_friction_impulse) const
    {
        const PersistentManifold* pm = get_persistent(id_a, id_b);
        if (pm == nullptr) {
            return false;
        }

        float best_dist_sq = 1e30f;
        uint32_t best_idx = MAX_CONTACTS_PER_PAIR;

        for (uint32_t i = 0u; i < pm->contact_count; ++i) {
            const float d = detail::contact_dist_sq(
                constraint_point_on_a,
                pm->contacts[i].contact.point_on_a
            );
            if (d < best_dist_sq) {
                best_dist_sq = d;
                best_idx = i;
            }
        }

        if (best_idx < pm->contact_count &&
            best_dist_sq < APC_CONTACT_MATCH_DISTANCE_SQ)
        {
            out_normal_impulse =
                pm->contacts[best_idx].accumulated_normal_impulse * APC_WARMSTART_FACTOR;
            out_friction_impulse =
                Vec3::scale(pm->contacts[best_idx].accumulated_friction_impulse,
                            APC_WARMSTART_FACTOR);
            return true;
        }

        return false;
    }

    // -----------------------------------------------------------------------
    // clear
    // -----------------------------------------------------------------------
    /// Clear all persistent state. Call on scene reset.
    // The contact ID counter is also reset.
    // -----------------------------------------------------------------------
    void clear() {
        for (uint32_t i = 0u; i < manifold_count_; ++i) {
            manifolds_[i].reset();
        }
        manifold_count_ = 0u;
        next_contact_id_ = 0u;
    }

    // -----------------------------------------------------------------------
    // pair_count
    // -----------------------------------------------------------------------
    /// Number of currently active (tracked) body pairs.
    // -----------------------------------------------------------------------
    APC_FORCEINLINE uint32_t pair_count() const {
        return manifold_count_;
    }

    // -----------------------------------------------------------------------
    // Accessors for iteration
    // -----------------------------------------------------------------------
    /// Direct read-only access to the manifold array.
    /// Valid indices: [0, pair_count()).
    APC_FORCEINLINE const PersistentManifold* manifolds() const {
        return manifolds_;
    }

    /// Direct read-write access to the manifold array.
    /// Valid indices: [0, pair_count()).
    APC_FORCEINLINE PersistentManifold* manifolds_mut() {
        return manifolds_;
    }

    // -----------------------------------------------------------------------
    // store_impulse
    // -----------------------------------------------------------------------
    /// Write back solver accumulated impulses for a specific contact in a
    /// specific manifold. Call this after the solver finishes to prepare
    /// warmstart data for the next frame.
    ///
    /// Returns false if the manifold or contact index is invalid.
    // -----------------------------------------------------------------------
    APC_FORCEINLINE bool store_impulse(
        uint32_t id_a, uint32_t id_b,
        uint32_t contact_index,
        float normal_impulse,
        const Vec3& friction_impulse)
    {
        PersistentManifold* pm = get_persistent_mut(id_a, id_b);
        if (pm == nullptr) {
            return false;
        }
        if (contact_index >= pm->contact_count) {
            return false;
        }

        pm->contacts[contact_index].accumulated_normal_impulse = normal_impulse;
        pm->contacts[contact_index].accumulated_friction_impulse = friction_impulse;
        return true;
    }

private:
    PersistentManifold manifolds_[MAX_PAIRS];
    uint32_t manifold_count_ = 0u;
    uint32_t next_contact_id_ = 0u;

    // -----------------------------------------------------------------------
    // find_manifold
    // -----------------------------------------------------------------------
    /// Linear search for a persistent manifold matching (lo, hi).
    /// Returns the index, or APC_INVALID_INDEX if not found.
    ///
    /// Since manifolds are kept sorted by pair key after update, we could
    /// use binary search. However, with MAX_PAIRS=256 and typical pair
    /// counts well under 100, linear scan is competitive and simpler.
    /// The sort still guarantees deterministic iteration order.
    // -----------------------------------------------------------------------
    APC_FORCEINLINE uint32_t find_manifold(uint32_t lo, uint32_t hi) const {
        for (uint32_t i = 0u; i < manifold_count_; ++i) {
            if (manifolds_[i].id_a == lo && manifolds_[i].id_b == hi) {
                return i;
            }
        }
        return APC_INVALID_INDEX;
    }

    // -----------------------------------------------------------------------
    // sort_manifolds
    // -----------------------------------------------------------------------
    /// Insertion sort the active manifold range by pair key.
    /// Insertion sort is chosen because:
    ///   1. Stable and deterministic across platforms
    ///   2. No dynamic allocation (unlike introsort's recursion guard)
    ///   3. Optimal for small N (N <= 256)
    ///   4. O(N) for nearly-sorted data (common case between frames)
    // -----------------------------------------------------------------------
    void sort_manifolds() {
        for (uint32_t i = 1u; i < manifold_count_; ++i) {
            // Copy current element to temp via memcpy (safe for POD-like)
            PersistentManifold key;
            std::memcpy(&key, &manifolds_[i], sizeof(PersistentManifold));

            const uint64_t key_k =
                detail::pair_sort_key(key.id_a, key.id_b);

            // Shift elements greater than key to the right
            int32_t j = static_cast<int32_t>(i) - 1;
            while (j >= 0) {
                const uint64_t jk = detail::pair_sort_key(
                    manifolds_[j].id_a, manifolds_[j].id_b);
                if (jk <= key_k) {
                    break;
                }
                std::memcpy(&manifolds_[j + 1], &manifolds_[j],
                            sizeof(PersistentManifold));
                j = j - 1;
            }

            // Insert key at correct position
            std::memcpy(&manifolds_[j + 1], &key,
                        sizeof(PersistentManifold));
        }
    }
};

} // namespace apc
