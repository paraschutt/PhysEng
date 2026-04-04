#pragma once
// ============================================================================
// DSPE Event System — collision callbacks, trigger events, game events
// ============================================================================
#include "entity.h"
#include "math_types.h"
#include <cstdint>
#include <array>
#include <functional>

namespace dspe {

// ---------------------------------------------------------------------------
// Event types
// ---------------------------------------------------------------------------
enum class EventType : uint8_t {
    // Physics events
    COLLISION,           // Two bodies made contact
    SEPARATION,          // Two bodies separated
    // Game events
    KICK,                // Player kicked ball
    HEADER,              // Player headed ball
    TACKLE_CONTACT,      // Sliding tackle contact
    // Trigger events
    GOAL_LEFT,           // Ball entered left goal
    GOAL_RIGHT,          // Ball entered right goal
    OUT_OF_BOUNDS,       // Ball exited pitch
    BALL_IN_BOX_LEFT,    // Ball entered left penalty box
    BALL_IN_BOX_RIGHT,   // Ball entered right penalty box
    KICKOFF_INFRINGEMENT,// Player inside kickoff circle
    // System events
    DESYNC,              // State hash mismatch detected
    ENTITY_SLEPT,        // Entity entered sleep
    ENTITY_WOKE,         // Entity exited sleep
};

// ---------------------------------------------------------------------------
// Event record (fixed-size, no heap)
// ---------------------------------------------------------------------------
struct PhysicsEvent {
    EventType   type{};
    uint32_t    frame{};          // Simulation frame number
    EntityId    entity_a{INVALID_ENTITY};
    EntityId    entity_b{INVALID_ENTITY};
    Vec3Vel     contact_point{};  // World-space contact (if applicable)
    Vec3Vel     contact_normal{}; // Outward normal from entity_a
    FpVel       impulse_magnitude{};
    FpVel       extra{};          // Type-dependent: speed, angle, etc.
};

// ---------------------------------------------------------------------------
// Event queue — ring buffer, deterministic ordering
// Capacity: 512 events per frame (more than enough for 15v15)
// ---------------------------------------------------------------------------
static constexpr size_t EVENT_QUEUE_CAPACITY = 512;

struct EventQueue {
    std::array<PhysicsEvent, EVENT_QUEUE_CAPACITY> events{};
    uint32_t head{0};
    uint32_t tail{0};
    uint32_t count{0};
    bool     overflow{false};

    void push(PhysicsEvent e) {
        if (count >= EVENT_QUEUE_CAPACITY) {
            overflow = true;
            return;
        }
        events[tail] = e;
        tail = (tail + 1) % EVENT_QUEUE_CAPACITY;
        ++count;
    }

    bool pop(PhysicsEvent& out) {
        if (count == 0) return false;
        out = events[head];
        head = (head + 1) % EVENT_QUEUE_CAPACITY;
        --count;
        return true;
    }

    void clear() { head = tail = count = 0; overflow = false; }
    bool empty()  const { return count == 0; }
    uint32_t size() const { return count; }
};

// ---------------------------------------------------------------------------
// Callback type for external game code
// ---------------------------------------------------------------------------
using EventCallback = std::function<void(const PhysicsEvent&)>;

} // namespace dspe
