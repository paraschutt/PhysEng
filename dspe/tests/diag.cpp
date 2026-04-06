// Quick diagnostic: trace capsule_box contacts and solver behavior
#include "world.h"
#include <cstdio>
#include <cmath>

using namespace dspe;

int main() {
    printf("=== DSPE Diagnostic ===\n\n");

    // Test 1: inv_inertia check
    {
        printf("--- Player inertia check ---\n");
        World w;
        w.create_ball({FpPos::from_float(50.0f), FpPos::from_float(1.0f), FpPos::zero()});
        w.create_player(0, {FpPos::zero(), FpPos::from_float(0.20f), FpPos::zero()}, false);
        const Entity& player = w.entities()[PLAYER_BEGIN];
        const RigidBody& rb = player.rigidbody;
        printf("inv_mass:    %.6f (raw=%d)\n", rb.inv_mass.to_float(), rb.inv_mass.raw);
        printf("inv_inertia_x: %.6f (raw=%d)\n", rb.inv_inertia_x.to_float(), rb.inv_inertia_x.raw);
        printf("inv_inertia_y: %.6f (raw=%d)\n", rb.inv_inertia_y.to_float(), rb.inv_inertia_y.raw);
        printf("inv_inertia_z: %.6f (raw=%d)\n", rb.inv_inertia_z.to_float(), rb.inv_inertia_z.raw);
    }

    // Test 2: friction velocity trace
    {
        printf("\n--- Friction velocity trace ---\n");
        World w;
        SurfaceState wet;
        wet.wetness = FpVel::from_float(0.0f);
        w.set_surface(wet);
        w.create_player(0, {FpPos::zero(), FpPos::from_float(0.20f), FpPos::zero()}, false);
        w.entities()[PLAYER_BEGIN].rigidbody.velocity = {FpVel::from_float(8.0f), FpVel::zero(), FpVel::zero()};
        TickInput no_input{};
        for (int tick = 0; tick < 120; ++tick) {
            w.tick(no_input);
            if (tick < 15 || tick % 10 == 0) {
                const Entity& p = w.entities()[PLAYER_BEGIN];
                printf("t=%3d vx=%.4f vy=%.4f y=%.4f\n", tick,
                    p.rigidbody.velocity.x.to_float(), p.rigidbody.velocity.y.to_float(),
                    p.rigidbody.position.y.to_float());
            }
        }
    }

    // Test 3: energy conservation detail
    {
        printf("\n--- Energy conservation trace ---\n");
        World w;
        w.create_ball({FpPos::zero(), FpPos::from_float(10.0f), FpPos::zero()});
        TickInput no_input{};
        float peak = 0;
        bool bounced = false;
        for (int tick = 0; tick < 180; ++tick) {
            w.tick(no_input);
            const Entity& ball = w.entities()[BALL_ENTITY];
            float vy = ball.rigidbody.velocity.y.to_float();
            float y = ball.rigidbody.position.y.to_float();
            if (!bounced && vy > 0.0f && y < 1.0f) bounced = true;
            if (bounced && y > peak) peak = y;
            if (tick < 20 || (tick > 40 && tick < 70) || tick % 20 == 0) {
                printf("t=%3d y=%.4f vy=%.4f%s\n", tick, y, vy, bounced ? " [post-bounce]" : "");
            }
        }
        printf("Peak after bounce: %.4f (expected 6.4)\n", peak);
    }

    return 0;
}
