#pragma once
// =============================================================================
// apc_ai_perception.h — Perception Ring Buffer for AI Reaction Delay
// =============================================================================
//
// Phase 11b Action 6: Eliminates AI "superhuman omniscience" by introducing
// a deterministic, configurable reaction delay.
//
// Every 60Hz cognitive tick, the true physics world state is snapshot into
// this circular buffer. When an AI athlete evaluates its utility curves, it
// queries a historical frame based on its reaction_frames stat (added to
// AthleteEntity in Action 5). This creates authentic human-like reaction
// gaps (~200ms) while remaining 100% deterministic and zero-allocation.
//
// Key types:
//   - AthletePercept:  compact snapshot of one athlete's external-visible state
//   - PerceptionSnapshot: one frame of world state (ball + all athletes)
//   - PerceptionRingBuffer: fixed-size circular buffer with bitwise wrapping
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation (fixed-size arrays, power-of-2 buffer)
//   - Deterministic (fixed iteration order, no heap)
//   - C++17
//
// =============================================================================

#include "apc_entity/apc_entity_types.h"
#include "apc_math/apc_vec3.h"
#include <cstdint>

namespace apc {

// =============================================================================
// AthletePercept — Compact snapshot of one athlete's external-visible state
// =============================================================================
// Captures only what OTHER athletes can perceive: position and team.
// The entity's own internal state (stamina, intent, cooldowns) is always
// known to itself and does not need perception delay.
//
// Layout: 20 bytes (4 index + 12 position + 1 team + 3 padding)
// =============================================================================
struct AthletePercept {
    uint32_t entity_index  = 0xFFFFFFFFu;
    Vec3     position      = {0.0f, 0.0f, 0.0f};
    uint8_t  team          = 0u;
    uint8_t  is_active     = 0u;
    uint8_t  padding[2]    = {};
};

// =============================================================================
// PerceptionSnapshot — One frame of the world state for AI perception
// =============================================================================
// Captures the ball state (position, velocity, possession) and all athlete
// positions. When the AI queries a delayed frame, it sees the ball and other
// athletes where they were N frames ago — not where they are now.
//
// Memory: 16 (ball) + 20 * 22 (athletes) + 4 (count) = 460 bytes per frame
//         64 frames * 460 = 29,440 bytes total (fits comfortably in static)
// =============================================================================
struct PerceptionSnapshot {
    // --- Ball state (what the AI "sees" about the ball) ---
    Vec3     ball_position        = {0.0f, 0.0f, 0.0f};
    Vec3     ball_velocity        = {0.0f, 0.0f, 0.0f};
    uint8_t  ball_possession_team = 0u;
    uint8_t  ball_in_play         = 1u;
    uint8_t  padding[2]           = {};

    // --- Athlete positions (what the AI "sees" about others) ---
    static constexpr uint32_t MAX_PERCEPTS = 22u; // 11v11 maximum
    AthletePercept athletes[MAX_PERCEPTS];
    uint32_t      athlete_count = 0u;

    // --- Reset to default empty state ---
    void reset() {
        ball_position        = {0.0f, 0.0f, 0.0f};
        ball_velocity        = {0.0f, 0.0f, 0.0f};
        ball_possession_team = 0u;
        ball_in_play         = 1u;
        padding[0]           = 0u;
        padding[1]           = 0u;
        athlete_count        = 0u;
        for (uint32_t i = 0u; i < MAX_PERCEPTS; ++i) {
            athletes[i] = AthletePercept();
        }
    }
};

// =============================================================================
// PerceptionRingBuffer — Fixed-size circular buffer for reaction delay
// =============================================================================
// Phase 11b Action 6: Perception Ring Buffer
//
// How it works:
//   1. Each 60Hz AI tick, push_state() snapshots the current world into the
//      buffer at the head position, then advances head (bitwise wrap).
//   2. When AI evaluates an athlete with reaction_frames=12, it calls
//      get_delayed_state(12) which reads the snapshot from 12 frames ago.
//   3. The AI then computes distances/opponent positions from the DELAYED
//      snapshot instead of the live state — creating a ~200ms reaction gap.
//
// Power-of-2 size (64) enables ultra-fast bitwise index wrapping:
//   next_head = (head + 1) & 63
//   delayed   = (head + 64 - 1 - delay) & 63
//
// Maximum history: 64 frames at 60Hz = ~1.067 seconds
// Memory: 64 * sizeof(PerceptionSnapshot) = 64 * 460 = 29,440 bytes
//
// Zero-allocation: all memory is statically embedded in the class.
// =============================================================================
class PerceptionRingBuffer {
public:
    // 64 frames at 60Hz = ~1.067 seconds of maximum history
    // Power of 2 enables bitwise wrapping (no modulo/division)
    static constexpr uint32_t BUFFER_SIZE = 64u;
    static constexpr uint32_t INDEX_MASK  = BUFFER_SIZE - 1u;

private:
    PerceptionSnapshot history[BUFFER_SIZE];
    uint32_t head  = 0u;
    uint32_t count = 0u;

public:
    // =========================================================================
    // clear — Reset buffer to empty state
    // =========================================================================
    void clear() {
        head  = 0u;
        count = 0u;
        for (uint32_t i = 0u; i < BUFFER_SIZE; ++i) {
            history[i].reset();
        }
    }

    // =========================================================================
    // push_state — Snapshot the current world state into the ring buffer
    // =========================================================================
    // Called once per 60Hz AI cognitive tick (from Application::tick).
    // The snapshot captures ball state and all athlete positions from the
    // EntityManager at the time of the call.
    //
    // Parameters:
    //   ball_pos     — Current ball position (world space)
    //   ball_vel     — Current ball velocity (world space)
    //   poss_team    — Current ball possession team (TeamId)
    //   in_play      — Whether the ball is currently in play
    //   athletes     — Pointer to athlete array
    //   athlete_count — Number of valid athletes
    // =========================================================================
    void push_state(const Vec3& ball_pos, const Vec3& ball_vel,
                    uint8_t poss_team, uint8_t in_play,
                    const AthleteEntity* athletes, uint32_t athlete_count)
    {
        PerceptionSnapshot& snap = history[head];

        // Capture ball state
        snap.ball_position        = ball_pos;
        snap.ball_velocity        = ball_vel;
        snap.ball_possession_team = poss_team;
        snap.ball_in_play         = in_play;

        // Capture athlete positions (compact: index + position + team + active)
        uint32_t percept_count = (athlete_count < PerceptionSnapshot::MAX_PERCEPTS)
                                 ? athlete_count : PerceptionSnapshot::MAX_PERCEPTS;
        snap.athlete_count = percept_count;

        for (uint32_t i = 0u; i < percept_count; ++i) {
            snap.athletes[i].entity_index = athletes[i].id.index;
            snap.athletes[i].position     = athletes[i].position;
            snap.athletes[i].team         = athletes[i].team;
            snap.athletes[i].is_active    = athletes[i].is_active;
        }

        // Advance head with bitwise wrap
        head = (head + 1u) & INDEX_MASK;
        if (count < BUFFER_SIZE) {
            ++count;
        }
    }

    // =========================================================================
    // get_delayed_state — Fetch the world state from N frames ago
    // =========================================================================
    // Returns a reference to the snapshot that was pushed `frames_delay` ticks
    // before the most recent push. The reference is valid until the next
    // push_state() call.
    //
    // Safety:
    //   - If the buffer is empty ( queried before first push ), returns a
    //     static empty snapshot.
    //   - If frames_delay exceeds available history, clamps to the oldest
    //     available frame (prevents reading uninitialized data).
    //
    // Typical values:
    //   reaction_frames=12 → 200ms delay at 60Hz
    //   reaction_frames=18 → 300ms delay at 60Hz
    // =========================================================================
    const PerceptionSnapshot& get_delayed_state(uint32_t frames_delay) const {
        if (count == 0u) {
            // Fallback: empty snapshot if queried before first push
            static PerceptionSnapshot empty_snap{};
            return empty_snap;
        }

        // Clamp delay to available history
        uint32_t clamped_delay = (frames_delay >= count)
                                 ? (count - 1u)
                                 : frames_delay;

        // Calculate the delayed index using bitwise wrap-around
        // head always points to the NEXT write position, so the most recent
        // frame is at (head - 1). We subtract clamped_delay from that.
        // Adding BUFFER_SIZE before masking prevents unsigned underflow.
        uint32_t delayed_index = (head + BUFFER_SIZE - 1u - clamped_delay) & INDEX_MASK;

        return history[delayed_index];
    }

    // =========================================================================
    // get_latest_state — Fetch the most recently pushed snapshot
    // =========================================================================
    const PerceptionSnapshot& get_latest_state() const {
        return get_delayed_state(0u);
    }

    // --- Accessors ---
    uint32_t get_head() const  { return head; }
    uint32_t get_count() const { return count; }

    // --- Reset ---
    void reset() { clear(); }
};

} // namespace apc
