// ============================================================================
// DSPE Diagnostic — Verify specific bug hypotheses
// ============================================================================
#include "world.h"
#include <cstdio>
#include <cmath>

using namespace dspe;

// Check capsule_box normal direction for a player on ground
static void diag_capsule_box_normal() {
    printf("=== DIAG: capsule_box normal direction ===\n");
    World w;
    // Place a single player at COM height 0.20m (above capsule radius 0.18m)
    w.create_player(0, { FpPos::zero(), FpPos::from_float(0.20f), FpPos::zero() });
    
    // Run 2 ticks — first tick establishes contact, second tick resolves
    TickInput no_input{};
    for (int tick = 0; tick < 5; ++tick) {
        w.tick(no_input);
        const Entity& player = w.entities()[PLAYER_BEGIN];
        float px = player.rigidbody.position.x.to_float();
        float py = player.rigidbody.position.y.to_float();
        float vy = player.rigidbody.velocity.y.to_float();
        printf("  tick %d: pos=(%.3f, %.3f) vel.y=%.3f sleep=%d\n",
               tick, px, py, vy, player.has(COMP_SLEEP) ? 1 : 0);
    }
    printf("  Expected: player rests near y=0.18 (capsule radius)\n");
    printf("  If player falls to y<0 or explodes upward → normal direction bug\n\n");
}

// Check player inv_inertia value
static void diag_player_inertia() {
    printf("=== DIAG: player inv_inertia ===\n");
    World w;
    w.create_player(0, { FpPos::zero(), FpPos::from_float(1.0f), FpPos::zero() });
    const Entity& p = w.entities()[PLAYER_BEGIN];
    printf("  inv_inertia_x = %f  (raw=%d)\n", 
           p.rigidbody.inv_inertia_x.to_float(), p.rigidbody.inv_inertia_x.raw);
    printf("  inv_inertia_y = %f  (raw=%d)\n", 
           p.rigidbody.inv_inertia_y.to_float(), p.rigidbody.inv_inertia_y.raw);
    printf("  inv_inertia_z = %f  (raw=%d)\n", 
           p.rigidbody.inv_inertia_z.to_float(), p.rigidbody.inv_inertia_z.raw);
    printf("  Expected: non-zero (approximately 0.823 = 1/(75*0.5*0.0324))\n");
    printf("  If 0.0 → TASKS.md Bug #3 Cause C confirmed\n\n");
}

// Check combined_restitution fp_sqrt
static void diag_restitution_sqrt() {
    printf("=== DIAG: combined_restitution fp_sqrt ===\n");
    FpVel e_dry = FpVel::from_float(0.80f);
    FpVel product = e_dry * e_dry;
    printf("  0.80 * 0.80 in Q15.16: product=%.6f (expected 0.640000)\n", product.to_float());
    FpVel result = combined_restitution(MAT_DRY_GRASS, MAT_DRY_GRASS);
    printf("  combined_restitution(DRY, DRY) = %.6f (expected 0.800000)\n", result.to_float());
    printf("  If result != ~0.80 → fp_sqrt bug confirmed\n\n");
}

// Check friction_cone: does player on wet grass experience friction?
static void diag_friction_wet() {
    printf("=== DIAG: friction on wet grass ===\n");
    World w;
    SurfaceState wet;
    wet.wetness = FpVel::from_float(0.0f); // fully wet
    w.set_surface(wet);
    
    w.create_player(0, { FpPos::zero(), FpPos::from_float(0.20f), FpPos::zero() });
    w.entities()[PLAYER_BEGIN].rigidbody.velocity = {
        FpVel::from_float(8.0f), FpVel::zero(), FpVel::zero()
    };

    TickInput no_input{};
    for (int tick = 0; tick < 60; ++tick) {
        w.tick(no_input);
        const Entity& p = w.entities()[PLAYER_BEGIN];
        float vx = p.rigidbody.velocity.x.to_float();
        float py = p.rigidbody.position.y.to_float();
        if (tick < 10 || tick % 10 == 0 || vx < 0.1f) {
            printf("  tick %d: vx=%.3f px=%.3f py=%.3f\n", tick, vx, 
                   p.rigidbody.position.x.to_float(), py);
        }
        if (vx < 0.05f) { printf("  Player stopped at tick %d\n", tick); break; }
    }
    printf("  Expected: vx decreases steadily to ~0\n\n");
}

int main() {
    diag_capsule_box_normal();
    diag_player_inertia();
    diag_restitution_sqrt();
    diag_friction_wet();
    return 0;
}
