#pragma once
// ============================================================================
// DSPE World — Main Simulation Loop
// Deterministic update order per tick:
//   1. InputSystem → ForceAccumulator
//   2. SkeletonController (animation → motor torques)
//   3. PhysicsStep: 4x substeps of Velocity Verlet
//      a. Flag CCD entities
//      b. CCD sweep (ball + fast movers)
//      c. Integrate (half-vel, full-pos, forces, new-acc, full-vel)
//   4. BroadPhase AABB tree update
//   5. NarrowPhase contact generation (sorted by EntityPair)
//   6. TriggerSystem (goal, OOB, penalty box)
//   7. ConstraintSolver (10 iterations, Baumgarte, friction cone)
//   8. SleepSystem (sleep/wake thresholds)
//   9. EventSystem dispatch
// ============================================================================
#include "dspe/components.h"
#include "dspe/systems/integrator.h"
#include "dspe/systems/broad_phase.h"
#include "dspe/systems/narrow_phase.h"
#include "dspe/systems/constraint_solver.h"
#include "dspe/systems/ccd.h"
#include "dspe/systems/skeleton.h"
#include "dspe/systems/sleep_system.h"
#include "dspe/systems/trigger_system.h"
#include "dspe/systems/event_system.h"
#include <array>
#include <vector>
#include <functional>
#include <cstdint>

namespace dspe {

// ---------------------------------------------------------------------------
// State checksum (FNV-1a 64-bit) for desync detection
// ---------------------------------------------------------------------------
inline uint64_t compute_state_hash(const std::array<Entity, MAX_ENTITIES>& entities) {
    static constexpr uint64_t FNV_OFFSET = 14695981039346656037ULL;
    static constexpr uint64_t FNV_PRIME  = 1099511628211ULL;
    uint64_t hash = FNV_OFFSET;

    // Hash all rigidbody position + velocity raw values in entity ID order
    for (EntityId id = 0; id < MAX_ENTITIES; ++id) {
        const Entity& e = entities[id];
        if (!e.has(COMP_RIGIDBODY)) continue;
        const RigidBody& rb = e.rigidbody;
        auto mix = [&](int32_t v) {
            const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&v);
            for (int i = 0; i < 4; ++i) {
                hash ^= bytes[i];
                hash *= FNV_PRIME;
            }
        };
        mix(rb.position.x.raw);    mix(rb.position.y.raw);    mix(rb.position.z.raw);
        mix(rb.velocity.x.raw);    mix(rb.velocity.y.raw);    mix(rb.velocity.z.raw);
        mix(rb.angular_velocity.x.raw);
        mix(rb.angular_velocity.y.raw);
        mix(rb.angular_velocity.z.raw);
        // Include ball spin
        if (e.has(COMP_BALL_PROPERTIES)) {
            mix(e.ball.spin.x.raw);
            mix(e.ball.spin.y.raw);
            mix(e.ball.spin.z.raw);
        }
    }
    return hash;
}

// ---------------------------------------------------------------------------
// Match input — one per tick per player
// ---------------------------------------------------------------------------
struct TickInput {
    PlayerInput inputs[30]{};   // Index = player entity ID
};

// ---------------------------------------------------------------------------
// World configuration
// ---------------------------------------------------------------------------
struct WorldConfig {
    PitchLayout   pitch{};
    SurfaceState  surface{};   // Initial surface state
    SolverParams  solver{};
    uint32_t      rng_seed{12345};
};

// ---------------------------------------------------------------------------
// World — owns all state, runs one tick at a time
// ---------------------------------------------------------------------------
class World {
public:
    explicit World(const WorldConfig& config = {});

    // ── Entity factory helpers ───────────────────────────────────────────────
    EntityId create_player(uint8_t player_index, Vec3Pos start_pos, bool team_b = false);
    EntityId create_ball  (Vec3Pos start_pos);
    EntityId create_static_box(Vec3Pos centre, Vec3Pos half_extents,
                               MaterialId mat = MAT_GOALPOST);

    // ── Simulation ──────────────────────────────────────────────────────────
    // Step one full outer tick (60Hz = 4 substeps of 240Hz)
    // Returns state hash for desync detection
    uint64_t tick(const TickInput& inputs);

    // ── State access ────────────────────────────────────────────────────────
    const std::array<Entity, MAX_ENTITIES>& entities() const { return entities_; }
    std::array<Entity, MAX_ENTITIES>&       entities()       { return entities_; }
    const EventQueue& events()  const { return events_; }
    EventQueue&       events()        { return events_; }
    uint32_t          frame()   const { return frame_; }
    uint64_t          last_hash() const { return last_hash_; }

    // ── External callbacks ──────────────────────────────────────────────────
    void set_event_callback(EventCallback cb) { event_callback_ = cb; }

    // ── State override (for rollback / resync) ──────────────────────────────
    void restore_state(const std::array<Entity, MAX_ENTITIES>& snapshot,
                       uint32_t frame);

    // ── Surface state update ─────────────────────────────────────────────────
    void set_surface(const SurfaceState& s) { surface_ = s; }

    // ── Debug ───────────────────────────────────────────────────────────────
    void debug_print_entity(EntityId id) const;

private:
    std::array<Entity, MAX_ENTITIES> entities_{};
    EventQueue      events_{};
    BroadPhase      broad_phase_{};
    NarrowPhase     narrow_phase_{};
    ConstraintSolver solver_;
    Integrator      integrator_{};
    SkeletonController skeleton_ctrl_{};
    SleepSystem     sleep_system_{};
    TriggerSystem   trigger_system_{};
    ContinuousCollisionDetection ccd_{};

    SurfaceState    surface_{};
    uint32_t        frame_{0};
    uint64_t        last_hash_{0};
    EventCallback   event_callback_{};

    // Persistent contact manifold cache (indexed by EntityPair)
    // Using vector sorted by EntityPair for determinism
    std::vector<ContactManifold> manifolds_;

    void run_substep(FpVel dt);
    void update_aabb_tree();
    void generate_contacts();
    void dispatch_events();
};

} // namespace dspe