// ============================================================================
// DSPE Diagnostic v4 — Stability stack: what contacts are generated?
// ============================================================================
#include "world.h"
#include <cstdio>
#include <cmath>

using namespace dspe;

static void diag_stability_contacts() {
    printf("=== DIAG: stability_stack contact analysis ===\n");
    World w;
    
    // Just 3 players stacked (simpler than 30)
    w.create_player(0, { FpPos::zero(), FpPos::from_float(0.2f), FpPos::zero() });
    w.create_player(1, { FpPos::zero(), FpPos::from_float(0.4f), FpPos::zero() });
    w.create_player(2, { FpPos::zero(), FpPos::from_float(0.6f), FpPos::zero() });
    
    int contact_count = 0;
    w.set_event_callback([&](const PhysicsEvent& ev) {
        if (ev.type == EventType::COLLISION) {
            contact_count++;
            if (contact_count <= 20) {
                printf("  Contact: e%d-e%d normal=(%.3f,%.3f,%.3f) impulse=%.3f\n",
                       ev.entity_a, ev.entity_b,
                       ev.contact_normal.x.to_float(),
                       ev.contact_normal.y.to_float(),
                       ev.contact_normal.z.to_float(),
                       ev.impulse_magnitude.to_float());
            }
        }
    });
    
    TickInput no_input{};
    for (int tick = 0; tick < 10; ++tick) {
        w.tick(no_input);
        if (tick < 3 || tick == 9) {
            for (int p = 0; p < 3; ++p) {
                const Entity& e = w.entities()[PLAYER_BEGIN + p];
                printf("  tick %d player %d: y=%.3f vy=%.3f\n", tick, p,
                       e.rigidbody.position.y.to_float(),
                       e.rigidbody.velocity.y.to_float());
            }
        }
    }
    printf("  Total contacts in first 10 ticks: %d\n", contact_count);
}

static void diag_single_player_settle() {
    printf("\n=== DIAG: single player settling over 600 ticks ===\n");
    World w;
    w.create_player(0, { FpPos::zero(), FpPos::from_float(0.2f), FpPos::zero() });
    
    TickInput no_input{};
    float min_y = 999, max_y = -999;
    float y_at_100 = 0, y_at_600 = 0;
    
    for (int tick = 0; tick < 600; ++tick) {
        w.tick(no_input);
        float y = w.entities()[PLAYER_BEGIN].rigidbody.position.y.to_float();
        if (y < min_y) min_y = y;
        if (y > max_y) max_y = y;
        if (tick == 99) y_at_100 = y;
        if (tick == 599) y_at_600 = y;
    }
    
    printf("  y range: [%.3f, %.3f]\n", min_y, max_y);
    printf("  y at tick 100: %.3f\n", y_at_100);
    printf("  y at tick 600: %.3f\n", y_at_600);
    printf("  Drift over 500 ticks: %.3f\n", y_at_600 - y_at_100);
    printf("  Expected: stable near 0.18 (capsule radius)\n");
}

static void diag_30_stack_simple() {
    printf("\n=== DIAG: 30-player stack — position tracking ===\n");
    World w;
    w.create_ball({ FpPos::from_float(52.5f), FpPos::from_float(1.0f), FpPos::zero() });
    
    for (int p = 0; p < 30; ++p) {
        w.create_player((uint8_t)p,
            { FpPos::zero(), FpPos::from_float((float)p * 0.2f), FpPos::zero() },
            p >= 15);
    }
    
    TickInput no_input{};
    for (int tick = 0; tick < 600; ++tick) {
        w.tick(no_input);
        if (tick == 0 || tick == 1 || tick == 10 || tick == 50 || 
            tick == 100 || tick == 300 || tick == 599) {
            float max_abs = 0;
            int worst_id = -1;
            for (int p = 0; p < 30; ++p) {
                const Entity& e = w.entities()[PLAYER_BEGIN + p];
                float px = std::abs(e.rigidbody.position.x.to_float());
                float py = std::abs(e.rigidbody.position.y.to_float());
                float pz = std::abs(e.rigidbody.position.z.to_float());
                float mp = std::max({px, py, pz});
                if (mp > max_abs) { max_abs = mp; worst_id = p; }
            }
            printf("  tick %3d: max_extent=%.2f (player %d)\n", tick, max_abs, worst_id);
            if (max_abs > 100) { printf("  *** EXPLODING — stopping early\n"); break; }
        }
    }
}

int main() {
    diag_stability_contacts();
    diag_single_player_settle();
    diag_30_stack_simple();
    return 0;
}
