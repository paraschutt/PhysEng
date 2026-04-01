# DSPE — Deterministic Sports Physics Engine
## Phase 1 Implementation

C++20. No Python in the simulation pipeline.

---

## Directory Structure

```
dspe/
├── CMakeLists.txt
├── include/dspe/
│   ├── fixed_point.h        — Q24.8 / Q15.16 / Q8.24 fixed-point with 64-bit intermediates
│   ├── math_types.h         — Vec3<Fp>, Quat<Fp>, AABB
│   ├── entity.h             — EntityId, ComponentFlag, ShapeType, EntityPair
│   ├── components.h         — RigidBody, Collider, Skeleton, BallProperties,
│   │                          TriggerVolume, SleepState, Entity
│   ├── materials.h          — 7 surface types, wetness modifier
│   ├── world.h              — World simulation loop, state hash
│   └── systems/
│       ├── integrator.h     — Velocity Verlet + ForceAccumulator + InputSystem
│       ├── broad_phase.h    — Dynamic AABB Tree
│       ├── narrow_phase.h   — GJK/EPA, capsule/sphere/box contacts
│       ├── constraint_solver.h — Sequential Impulse, friction cone, warm-starting
│       ├── ccd.h            — Swept sphere CCD
│       ├── skeleton.h       — 15-bone FK, motor drives, foot velocity sensing
│       ├── sleep_system.h   — Sleep/wake with proximity radius
│       ├── trigger_system.h — Goal, OOB, penalty box, kickoff circle
│       └── event_system.h   — EventQueue ring buffer
├── src/
│   ├── world.cpp
│   └── systems/
│       ├── integrator.cpp
│       ├── broad_phase.cpp
│       ├── narrow_phase.cpp
│       ├── constraint_solver.cpp
│       ├── ccd.cpp
│       ├── skeleton.cpp
│       ├── sleep_system.cpp
│       └── trigger_system.cpp
└── tests/
    └── test_phase1.cpp      — Full Phase 1 test suite (8 tests)
```

---

## Build

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
./dspe_tests
```

**Required compiler:** GCC 13+, Clang 16+, or MSVC 2022.
**Required platform:** x86-64 (SSE2). ARM requires `-mfpu=vfp` substitution in CMakeLists.txt.

---

## Determinism Flags

The CMakeLists.txt enforces all flags from Brief Section 11:

| Flag | Purpose |
|---|---|
| `-ffloat-store` | Flush x87 extended-precision registers on every store |
| `-mfpmath=sse -msse2` | Use 32/64-bit SSE FP, not 80-bit x87 |
| `-ffp-contract=off` | Disable FMA fusion (different rounding from separate mul+add) |

> **Critical:** `-ffloat-store` alone is insufficient on x86. You must also set `-mfpmath=sse` to prevent the x87 FPU from being used at all.

---

## Fixed-Point Format Assignment

| Quantity | Format | Range | Notes |
|---|---|---|---|
| World positions | `FpPos` (Q24.8) | ±16.7 Mm | Pitch ≤ 105m, ample headroom |
| Velocities / forces | `FpVel` (Q15.16) | ±32,767 m/s | Max ball speed ~50 m/s |
| Angular velocity | `FpAng` (Q8.24) | ±127 rad/s | Max spin ~100 rad/s |
| All intermediates | `int64_t` | ±9.2×10¹⁸ | Mandatory for all multiplications |

---

## System Execution Order (per tick)

```
1. InputSystem         — player commands → force accumulation
2. SkeletonController  — animation targets → motor torques → FK → foot sensors
3. Physics (×4 substeps)
   a. Flag CCD entities (ball always; others by velocity threshold)
   b. CCD sweep (swept sphere vs capsule/box, finds earliest TOI)
   c. Velocity Verlet:  v(t+dt/2), x(t+dt), F(x,v_half), a(t+dt), v(t+dt)
4. BroadPhase update   — AABB tree refit (only if entity moved outside fat AABB)
5. NarrowPhase         — GJK/EPA contacts, sorted by EntityPair for determinism
6. TriggerSystem       — goal / OOB / penalty box / kickoff circle (once per outer tick)
7. ConstraintSolver    — 10 iterations, β=0.2, slop=5mm, 2D friction cone, warm-start
8. SleepSystem         — sleep after 30 ticks below threshold; proximity wake at 1.0m
9. EventDispatch       — callbacks to game layer
```

---

## Phase 1 Test Suite

| # | Test | Validates |
|---|---|---|
| 1 | `fixed_point_no_overflow` | Q24.8, Q15.16, Q8.24 range; 64-bit intermediate |
| 2 | `determinism` | Two identical runs produce bit-identical state hashes |
| 3 | `energy_conservation` | Ball from 10m bounces to 6.4m (e=0.8 ±5%) |
| 4 | `ccd_no_tunnel` | Ball at 40 m/s registers collision with 0.12m goalpost |
| 5 | `stability_stack` | 30 stacked players: max speed < 5 m/s after 10s |
| 6 | `spin_magnus` | Topspin ball drops ~2.95m extra vs no-spin (±5%) |
| 7 | `friction_cone` | Player slides 9.32m on wet grass (μ=0.35 ±15%) |
| 8 | `trigger_goal` | EVENT_GOAL_LEFT fires within 1 tick of goal line crossing |

---

## Key Design Decisions

**Velocity Verlet drag approximation:** Drag is evaluated at `v(t+dt/2)` (half-step velocity), introducing a first-order error. `C_d` is tuned empirically against reference trajectories rather than using the ISA theoretical value. This is documented in Brief Section 2.1.1.

**Magnus scaling:** Force scales linearly with `|ω|` and `|v|`. `C_L` is calibrated to Goff & Carré (2010) empirical football data, not theoretical aerodynamic tables.

**Foot handling:** Feet are active constraint participants, not passive sensors. Foot velocity at contact feeds kick impulse and spin generation (Brief Section 5.2).

**Sleep system:** Ball never sleeps. All other entities sleep after 30 ticks below 0.01 m/s linear and 0.01 rad/s angular threshold, and wake on proximity (1.0m) to any active entity.

**No Python:** The simulation pipeline is pure C++20. Python may be used for offline tooling (data export, test harnesses) but never in the real-time loop.

---

## Rollback vs Replay (Brief Section 10)

These are **separate systems**:

- **Rollback buffer:** 8 frames × ~160 bytes × 64 entities = **82 KB**. Used for netcode re-simulation on late input arrival.
- **Replay buffer:** 600 frames × ~56 bytes × 64 entities = **2.15 MB**. Used for broadcast replay and VAR review. Written every tick, never used for re-simulation.

Both fit within the 64 MB physics state budget.

---

## Phase 2 (Render Pipeline) — Not in this deliverable

- Interpolation/extrapolation between 60Hz ticks for 144Hz+ rendering
- GPU mesh skinning (15-bone skeletons)
- Foot placement IK (two-bone analytical, post-simulation)
- Ball trail, debug overlays
- Stadium BVH, weather shaders, crowd wind field → physics wind force connection