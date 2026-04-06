// ============================================================================
// DSPE Diagnostic v5 — Friction cone detailed trace
// ============================================================================
#include "world.h"
#include <cstdio>
#include <cmath>

using namespace dspe;

static void diag_friction_wet_detailed() {
    printf("=== DIAG: friction on wet grass (detailed) ===\n");
    World w;
    SurfaceState wet;
    wet.wetness = FpVel::from_float(0.0f); // fully wet
    w.set_surface(wet);
    
    // Check ground material after set_surface
    const Entity& ground = w.entities()[31]; // first static entity
    printf("  Ground entity 31 material_id: %d (%s)\n", 
           ground.collider.material_id,
           ground.collider.material_id == MAT_WET_GRASS ? "WET_GRASS" :
           ground.collider.material_id == MAT_DRY_GRASS ? "DRY_GRASS" : "OTHER");
    printf("  MAT_WET_GRASS kinetic_mu = 0.35\n");
    printf("  MAT_DRY_GRASS kinetic_mu = 0.60\n\n");
    
    w.create_player(0, { FpPos::zero(), FpPos::from_float(0.20f), FpPos::zero() });
    w.entities()[PLAYER_BEGIN].rigidbody.velocity = {
        FpVel::from_float(8.0f), FpVel::zero(), FpVel::zero()
    };
    
    // Check player material
    const Entity& player = w.entities()[PLAYER_BEGIN];
    printf("  Player material_id: %d (%s)\n", 
           player.collider.material_id,
           player.collider.material_id == MAT_DRY_GRASS ? "DRY_GRASS" : "OTHER");
    
    TickInput no_input{};
    for (int tick = 0; tick < 400; ++tick) {
        w.tick(no_input);
        const Entity& p = w.entities()[PLAYER_BEGIN];
        float vx = p.rigidbody.velocity.x.to_float();
        float px = p.rigidbody.position.x.to_float();
        if (tick <= 5 || tick % 20 == 0 || vx < 0.05f) {
            printf("  tick %3d: vx=%.4f px=%.4f py=%.4f\n", tick, vx, px,
                   p.rigidbody.position.y.to_float());
        }
        if (vx < 0.05f) { 
            printf("  STOPPED at tick %d, x=%.4f\n", tick, px);
            printf("  Slide distance: %.4fm\n", px);
            printf("  Expected: ~9.32m\n");
            break; 
        }
    }
}

int main() {
    diag_friction_wet_detailed();
    return 0;
}
