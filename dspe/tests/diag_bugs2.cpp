// ============================================================================
// DSPE Diagnostic v2 — Targeted analysis of the 3 failing tests
// ============================================================================
#include "world.h"
#include <cstdio>
#include <cmath>

using namespace dspe;

// Instrument the energy_conservation test to see what happens at bounce
static void diag_energy_bounce() {
    printf("=== DIAG: energy_conservation bounce analysis ===\n");
    World w;
    w.create_ball({ FpPos::zero(), FpPos::from_float(10.0f), FpPos::zero() });
    
    TickInput no_input{};
    float prev_vy = -999;
    float min_y = 10.0f;
    int bounce_tick = -1;
    
    for (int tick = 0; tick < 180; ++tick) {
        w.tick(no_input);
        const Entity& ball = w.entities()[BALL_ENTITY];
        float vy = ball.rigidbody.velocity.y.to_float();
        float y  = ball.rigidbody.position.y.to_float();
        
        if (y < min_y) min_y = y;
        
        // Detect bounce: was falling (vy<0), now rising (vy>0)
        if (prev_vy < -1.0f && vy > 0.0f && bounce_tick < 0) {
            bounce_tick = tick;
            printf("  Bounce at tick %d: y=%.4f vy=%.4f prev_vy=%.4f\n", 
                   tick, y, vy, prev_vy);
            printf("  Ball fell from 10m. Impact speed expected: sqrt(2*9.81*10)=%.2f m/s\n",
                   sqrtf(2*9.81f*10.0f));
            printf("  Expected post-bounce speed: 0.8 * %.2f = %.2f m/s\n",
                   sqrtf(2*9.81f*10.0f), 0.8f*sqrtf(2*9.81f*10.0f));
            printf("  Expected peak height: 10 * 0.8^2 = 6.4m\n");
        }
        
        // Track peak after bounce
        if (bounce_tick > 0 && tick > bounce_tick && vy > 0) {
            // Still going up
        }
        if (bounce_tick > 0 && tick > bounce_tick) {
            static float peak = 0;
            if (y > peak) peak = y;
            if (vy <= 0) {
                printf("  Peak after bounce at tick %d: y=%.4f\n", tick, peak);
                printf("  Expected: 6.4m ± 0.5m\n");
                printf("  Deficit: %.4fm (%.1f%%)\n", 6.4f - peak, (6.4f - peak)/6.4f*100.0f);
                break;
            }
        }
        
        prev_vy = vy;
    }
    printf("  Min y reached: %.4f\n\n", min_y);
}

// Check if capsule_box normal is actually +Y when player stands on flat ground
// We'll manually inspect by creating a narrow-phase test
static void diag_normal_direction() {
    printf("=== DIAG: capsule_box contact normal inspection ===\n");
    
    // The ground box is at y=-0.5 with half-extent 0.5, so top face at y=0
    // Player capsule base at y=0, tip at y=1.62, radius=0.18
    // For a player at COM y=0.20, the capsule base is at y=0.20 (local_base=0)
    // and tip at y=0.20+1.62=1.82
    // Closest point to ground is capsule base at y=0.20
    // Distance from ground surface (y=0) = 0.20 - 0.18 (radius) = 0.02m
    // Normal should be +Y (pointing from ground toward player)
    
    World w;
    w.create_player(0, { FpPos::zero(), FpPos::from_float(0.20f), FpPos::zero() });
    
    // Hook into event system to inspect contacts
    bool got_contact = false;
    w.set_event_callback([&](const PhysicsEvent& ev) {
        if (ev.type == EventType::COLLISION) {
            if (!got_contact) {
                printf("  Contact event: entity_a=%d entity_b=%d\n", ev.entity_a, ev.entity_b);
                printf("  Normal: (%.3f, %.3f, %.3f)\n",
                       ev.contact_normal.x.to_float(),
                       ev.contact_normal.y.to_float(),
                       ev.contact_normal.z.to_float());
                printf("  Impulse: %.3f\n", ev.impulse_magnitude.to_float());
                got_contact = true;
            }
        }
    });
    
    TickInput no_input{};
    for (int tick = 0; tick < 5 && !got_contact; ++tick) {
        w.tick(no_input);
    }
    
    if (!got_contact) {
        printf("  NO CONTACT EVENT GENERATED in 5 ticks!\n");
        printf("  This means capsule_box is returning false or contact is being filtered.\n");
    }
    
    printf("  Expected: normal.y > 0 (pointing up from ground)\n\n");
}

// Check friction more carefully — what mu is being used? what impulse?
static void diag_friction_detail() {
    printf("=== DIAG: friction detail ===\n");
    
    // Wet surface: wetness=0 → fully wet → MAT_WET_GRASS (kinetic mu = 0.35)
    // After wetness modifier: 0.35 * (0.5 + 0.5*0) = 0.35 * 0.5 = 0.175
    // Expected stopping decel: 0.175 * 9.81 = 1.72 m/s²
    // Expected stopping dist: 8²/(2*1.72) = 18.6m (!) much longer than 9.32m
    
    printf("  Surface wetness = 0 (fully wet)\n");
    printf("  MAT_WET_GRASS kinetic_mu = 0.35\n");
    printf("  Wetness modifier: 0.5 + 0.5*wetness = 0.5 + 0.5*0 = 0.5\n");
    printf("  Effective mu = 0.35 * 0.5 = 0.175\n");
    printf("  Expected decel = 0.175 * 9.81 = 1.717 m/s²\n");
    printf("  Expected stopping dist = 8²/(2*1.717) = 18.63m\n");
    printf("  Test expects 9.32m (which assumes mu=0.35, NOT 0.175)\n\n");
    
    // The wetness factor is halving the friction! The test expects mu=0.35
    // but the wetness modifier reduces it to 0.175.
    // Either the test's wetness value is wrong, or the wetness application is wrong.
    
    // Let's check what wetness=1.0 (fully dry) gives:
    printf("  If wetness=1.0 (fully dry):\n");
    printf("  Wetness modifier: 0.5 + 0.5*1.0 = 1.0\n");
    printf("  Effective mu = 0.35 * 1.0 = 0.35\n");
    printf("  Expected decel = 0.35 * 9.81 = 3.434 m/s²\n");
    printf("  Expected stopping dist = 8²/(2*3.434) = 9.32m ✓\n\n");
    
    // But wait — the test creates a World with default SurfaceState, then sets
    // wetness=0 and calls set_surface. Let's verify what material the ground uses
    // and how combined_kinetic_mu works.
    // 
    // Ground material is MAT_DRY_GRASS (set in World constructor).
    // Player material is MAT_DRY_GRASS (set in create_player).
    // combined_kinetic_mu(DRY, DRY) = min(0.60, 0.60) = 0.60
    // 
    // Then apply_wetness(0.60) with wetness=0:
    // factor = 0.5 + 0.5*0 = 0.5
    // result = 0.60 * 0.5 = 0.30
    //
    // So effective mu = 0.30, not 0.35!
    // Expected decel = 0.30 * 9.81 = 2.943 m/s²
    // Expected stopping dist = 8²/(2*2.943) = 13.59m — still not 9.32m!
    
    printf("  REFINED ANALYSIS:\n");
    printf("  Ground material = MAT_DRY_GRASS (kinetic_mu=0.60)\n");
    printf("  Player material = MAT_DRY_GRASS (kinetic_mu=0.60)\n");
    printf("  combined_kinetic_mu = min(0.60, 0.60) = 0.60\n");
    printf("  With wetness=0: effective_mu = 0.60 * 0.5 = 0.30\n");
    printf("  Expected decel = 0.30 * 9.81 = 2.943 m/s²\n");
    printf("  Expected stopping dist = 64/5.886 = 10.87m\n\n");
    
    // The test expects 9.32m which assumes mu_effective = 0.35*1.0 = 0.35
    // i.e. the wetness factor should NOT be applied to ground friction in this test.
    // The test comment says "wet grass (mu=0.35)" — this implies the surface
    // material should be MAT_WET_GRASS (mu=0.35) not MAT_DRY_GRASS with wetness mod.
    
    printf("  CONCLUSION: The test expects mu_eff=0.35, but the actual mu_eff depends on:\n");
    printf("  1. Material pair (ground+player both MAT_DRY_GRASS, kinetic_mu=0.60)\n");
    printf("  2. Wetness modifier (wetness=0 halves it to 0.30)\n");
    printf("  To get mu_eff=0.35: either use MAT_WET_GRASS ground, or set wetness=0.583\n\n");
}

int main() {
    diag_energy_bounce();
    diag_normal_direction();
    diag_friction_detail();
    return 0;
}
