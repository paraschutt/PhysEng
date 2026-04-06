#include "world.h"
#include <cstdio>
using namespace dspe;
int main() {
    World w;
    w.create_player(0, {FpPos::zero(), FpPos::from_float(0.2f), FpPos::zero()}, false);
    auto& rb = w.entities()[0].rigidbody;
    printf("inv_mass=%.6f raw=%d\n", rb.inv_mass.to_float(), rb.inv_mass.raw);
    printf("inv_ix=%.6f raw=%d\n", rb.inv_inertia_x.to_float(), rb.inv_inertia_x.raw);
    printf("inv_iy=%.6f raw=%d\n", rb.inv_inertia_y.to_float(), rb.inv_inertia_y.raw);
    printf("inv_iz=%.6f raw=%d\n", rb.inv_inertia_z.to_float(), rb.inv_inertia_z.raw);
    SurfaceState wet; wet.wetness = FpVel::from_float(0.0f);
    World w2; w2.set_surface(wet);
    w2.create_player(0, {FpPos::zero(), FpPos::from_float(0.2f), FpPos::zero()}, false);
    w2.entities()[0].rigidbody.velocity = {FpVel::from_float(8.0f), FpVel::zero(), FpVel::zero()};
    TickInput no{};
    for(int t=0;t<120;++t){w2.tick(no);if(t<15||t%10==0){auto& p=w2.entities()[0];printf("t=%3d vx=%.4f vy=%.4f y=%.4f\n",t,p.rigidbody.velocity.x.to_float(),p.rigidbody.velocity.y.to_float(),p.rigidbody.position.y.to_float());}}
    return 0;
}