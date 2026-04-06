# DSPE Bug Fix Task List

## Test Results Summary
**Failed Tests:** 3/8
**Passed Tests:** 5/8 (fixed_point_no_overflow, determinism, ccd_no_tunnel, spin_magnus, trigger_goal)

---

## Active Bugs

### 1. [HIGH] energy_conservation — Bounce height 5.41m, expected 6.40m (tol 0.50m)
**Test:** `energy_conservation`
**File:** `tests/test_phase1.cpp:137`
**Symptom:** Ball dropped from 10m bounces to 5.41m instead of 6.40m. Deficit is ~0.99m (~15% low).

**Analysis:**
Combined restitution is computed as `sqrt(e_a * e_b)` via `combined_restitution()` in `materials.h`.
Ball material is `MAT_DRY_GRASS` (e=0.80), ground material is also `MAT_DRY_GRASS` (e=0.80).
`combined_restitution = sqrt(0.80 * 0.80) = 0.80` — correct on paper.

The deficit of ~15% points to one of three causes:

**Cause A — Restitution applied at wrong velocity (most likely).**
In `build_contact_constraints()` (constraint_solver.cpp), the restitution bias is:
```cpp
if ((-vn) > params_.restitution_slop) {
    c.bias += e_combined * (-vn);
}
```
`vn` is sampled from `velocity_at()` at the start of `build_contact_constraints`, which runs
AFTER the 4 substeps of Verlet integration for that tick. By the time this runs, the ball has
already partially penetrated the ground box and the velocity has been partially modified by
the ground clamp and previous solver iterations. The relative velocity `vn` used for restitution
is therefore smaller than the true pre-collision velocity, so the bounce impulse is undersized.

**Cause B — Energy loss from Baumgarte position correction.**
Even at the corrected 60 Hz `dt_inv`, every tick where the ball rests on the ground the
Baumgarte bias injects a small upward velocity. On bounce frames the opposite is true:
penetration depth > slop triggers a corrective bias that adds to the bounce, but penetration
is also consuming some of the kinetic energy budget before the restitution term fires.

**Cause C — `restitution_slop` threshold too high.**
`params_.restitution_slop = 0.5f` m/s. A ball dropped from 10m hits the ground at
`sqrt(2 * 9.81 * 10) ≈ 14.0 m/s`. `-vn` will be well above 0.5, so this threshold is not
the problem for the first bounce, but may affect subsequent bounces.

**Most likely fix:**
Cause A is dominant. The restitution velocity `vn` must be captured from the relative velocity
at the moment of first contact (i.e. the incoming velocity before any impulse is applied that
tick), not after warm-starting has already partially modified the velocities. The warm_start()
call precedes the contact constraint build in the current code path — check ordering.

Additionally check: `combined_restitution()` uses `fp_sqrt(ea * eb)` where both are Q15.16.
`0.80 * 0.80 = 0.64` in Q15.16: `raw = 0.64 * 65536 = 41943`. `fp_sqrt(41943)` uses the
Newton-Raphson integer sqrt — verify it returns the correct Q15.16 value for 0.80 (raw=52429).

**Files to check:**
- `src/systems/constraint_solver.cpp` — ordering of warm_start vs build_contact_constraints,
  restitution velocity capture
- `include/materials.h` — `combined_restitution()` fp_sqrt correctness for Q15.16 inputs

**Fix Priority:** P1

---

### 2. [CRITICAL] stability_stack — Solver explosion 141.48m (limit 20m)
**Test:** `stability_stack`
**File:** `tests/test_phase1.cpp:227`
**Symptom:** 30 stacked players reach 141.48m after 10s. Reduced from 2974m (previous Baumgarte
fix helped significantly) but still 7× over the 20m limit.

**Analysis:**
The remaining explosion after the Baumgarte and joint-limit fixes is almost certainly coming
from the capsule_box contact normal direction. When a player capsule sits on top of the ground
box, `capsule_box()` in `narrow_phase.cpp` walks segment candidates and finds the lowest point.
The contact normal returned is `qb.rotate(best_normal_local)` where `best_normal_local` is
computed in box-local space.

**Suspected cause A — Wrong normal direction convention for capsule_box.**
`generate_contact()` convention (from all other shape pairs) is: normal points FROM body B
TOWARD body A (i.e. outward from B's surface into A). For sphere_box, body A is the sphere,
body B is the box; the normal points upward (+Y) when the sphere sits on top.
In the new `capsule_box`, body A is the capsule, body B is the box. The normal in the
"sphere sits above box" case should also be +Y (pointing from box surface toward capsule).
If `best_normal_local` has the wrong sign, the constraint solver applies the impulse in the
wrong direction, accelerating entities into the ground rather than pushing them out.

Verify: in the dispatch, `BOX+CAPSULE` flips the normal with `out.points[0].normal = -out.points[0].normal`.
The `CAPSULE+BOX` path does not flip. Confirm that the un-flipped normal actually points from
box surface toward capsule (i.e. +Y for a player standing on flat ground).

**Suspected cause B — Effective mass calculation for static bodies.**
`effective_mass()` computes:
```cpp
FpVel lin_mass = a.inv_mass + b.inv_mass;
```
For a static body, `inv_mass = 0` and `inv_inertia_x/y/z = 0`. This is correct — the static
body contributes nothing to the denominator. But if the entity pair order (a, b) is wrong
and the capsule entity's inv_mass ends up in the static slot, the effective mass calculation
collapses to near-zero, producing enormous impulses.

Check: in `build_contact_constraints`, `ea = entities[ida]` where `ida = manifold.pair.a`
(the smaller entity ID). Players are IDs 0–29; ground box is ID 31+. So for player-ground
pairs, ea = player (capsule), eb = ground (static). The constraint is built with `ra` as the
offset from the player COM and `rb` from the ground COM. Player has non-zero inv_mass and
inv_inertia — this should be correct. Verify `effective_mass` actually receives the player
as `a` and the ground as `b`.

**Suspected cause C — capsule_box generates contact with incorrect depth, driving Baumgarte hard.**
If `best_depth` is systematically over-estimated (e.g. returning `cap_r + dy` when the capsule
is barely touching the surface), the Baumgarte bias `beta * dt_inv * excess` fires aggressively
even for resting contact. At β=0.2, dt_inv=60, excess=0.1m: bias = 1.2 m/s per iteration,
×10 iterations = 12 m/s upward velocity injected per tick, which compounds across 30 entities.

The `dist.raw == 0` branch in `capsule_box` computes:
```cpp
depth = cap_r + dx;  // or dy, dz
```
This is the depth when the segment point is INSIDE the box. For a resting capsule whose base
point is exactly at y=0 (box top surface), the segment base in box-local space will be at
y = -(box_half_y) + 0 = at the top face, so `dist.raw` may be 0 or near-zero, triggering the
interior branch and computing a depth of `cap_r + dy` which can be large.

The correct resting-contact depth is just `cap_r - dist` from the exterior case. The interior
branch should only fire when the capsule has genuinely tunnelled into the box. Add a check:
if the closest box point is on the face (not interior), use the exterior formula.

**Fix Priority:** P0 — Blocks stability_stack

---

### 3. [HIGH] friction_cone — Player never stops sliding
**Test:** `friction_cone`
**File:** `tests/test_phase1.cpp:322`
**Symptom:** Player at 8 m/s on wet grass (μ=0.35) never decelerates to stop.

**Analysis:**
The `capsule_box` contact is now generating contacts (players no longer fall through the
ground — confirmed by stability_stack showing 141m rather than infinite fall). However the
player is still not stopping, which means friction impulse is either not being generated or
is being cancelled.

**Cause A — Contact normal pointing down instead of up cancels friction.**
If the `capsule_box` normal is inverted (pointing -Y into the ground rather than +Y away from
it), then `lambda_n` (the normal impulse clamped to >= 0) will be driven to zero because the
velocity in the normal direction `vn = rel_vel.dot(normal)` will be positive (moving away from
surface in the inverted-normal frame), producing a negative lambda_delta that clamps to 0.
With `lambda_n = 0`, `max_friction = mu * lambda_n = 0`, and the friction cone clamp zeros
both tangential impulses. Net result: no friction, player slides forever.

This is the same normal-direction bug identified in Bug #2 and is likely the root cause of
both failures.

**Cause B — Player capsule not making persistent contact.**
The sleep system or the broad-phase fat AABB may be causing the contact to drop in and out.
If the manifold is rebuilt as "new" every tick (no warm-starting), `lambda_n` starts at 0
each tick and must build up from scratch over 10 iterations — it may not converge to a large
enough value to generate meaningful friction in a single solve pass.

Check: after the first tick with ground contact, does `manifold.count > 0` persist on the
second tick? If `update_manifold` is not matching the contact point (the 2cm proximity
threshold may be too tight for a sliding contact point that has moved), the manifold resets.

**Cause C — Player entity has inv_inertia = 0.**
In `world.cpp` `create_player()`, the inertia is computed as:
```cpp
FpVel I_inv = FpVel::one() / (PLAYER_MASS * FpVel::from_float(0.5f)
                             * FpVel::from_float(0.18f * 0.18f));
```
`0.18 * 0.18 = 0.0324`. In Q15.16: `0.0324 * 65536 = 2123 raw`.
`PLAYER_MASS * 0.5 = 37.5` → raw = `37.5 * 65536 = 2,457,600`.
`2,457,600 * 2123 = 5,215,257,600` — overflows int32 (max 2,147,483,647).
The multiplication `PLAYER_MASS * FpVel::from_float(0.5f) * FpVel::from_float(0.18f * 0.18f)`
uses Q15.16 operator* which widens to int64 for each pairwise multiply, but the intermediate
`(PLAYER_MASS * 0.5f)` = 37.5 in Q15.16 is fine (raw=2,457,600). Then:
`2,457,600 * 2123 >> 16 = 5,215,257,600 >> 16 = 79,574` — this fits int32 after the shift.
So `I = 79,574` in Q15.16 = 1.215 N·m — plausible. `I_inv = ONE / 79574 = 65536 / 79574 = 0`
because integer division truncates. This means **inv_inertia = 0 for all players**.

With `inv_inertia = 0`, the angular term in `effective_mass` is zero, and more critically
`apply_impulse` does not update `angular_velocity` at all (multiplied by 0). The friction
impulse still applies to linear velocity, so this alone shouldn't stop friction — but verify.

**Fix Priority:** P1 — depends on resolving Bug #2 normal direction first; may auto-fix.

---

## Fix Implementation Order

1. **P0 — Fix `capsule_box` contact normal direction** (narrow_phase.cpp)
   - Add debug assertion: for a player capsule sitting on flat ground, normal.y must be > 0
   - Trace the normal sign through the `dist.raw == 0` and `dist.raw > 0` branches
   - Verify the `BOX+CAPSULE` flip in generate_contact is correct

2. **P0 — Fix `capsule_box` depth computation in interior branch** (narrow_phase.cpp)
   - The `dist.raw == 0` branch fires when segment point is clamped to box interior
   - For resting contact (segment endpoint exactly on box surface), dist will be near-zero
     due to fixed-point rounding, incorrectly triggering the interior-depth formula
   - Gate: only use `cap_r + face_dist` if the point is genuinely interior (all three axes
     show positive penetration). If the point is on a face, use `cap_r - dist` instead.

3. **P1 — Fix player inv_inertia underflow** (world.cpp)
   - `FpVel::one() / I` where I ≈ 1.215 truncates to 0 in Q15.16
   - Compute inertia in float, take reciprocal in float, then convert:
     `FpVel::from_float(1.0f / (75.0f * 0.5f * 0.18f * 0.18f))`

4. **P1 — Verify restitution velocity capture ordering** (constraint_solver.cpp)
   - Confirm warm_start() runs AFTER build_contact_constraints() captures vn
   - If not, restructure so vn is captured from pre-warm-start velocities

5. **P2 — Verify combined_restitution fp_sqrt correctness** (materials.h)
   - Unit test: `combined_restitution(MAT_DRY_GRASS, MAT_DRY_GRASS)` should return 0.80 ± 0.01

---

## Verification Checklist

After fixes, re-run:
```bash
cd build && cmake --build . && ./dspe_tests
```

Expected output:
```
Results: 8/8 passed
```

Target values:
- `energy_conservation`: bounce height 6.4m ± 0.5m
- `stability_stack`: max position extent < 20m after 600 ticks
- `friction_cone`: slide distance 9.32m ± 2.0m on wet grass (μ=0.35)