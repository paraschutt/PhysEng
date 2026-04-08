# Phase 9: Engine / Simulation / AI Consolidation

**Goal**: Harden the existing system into a production-quality soccer simulation.
The core physics engine (40 tests passing) is solid. The application layer (Phases 4-8)
is wired but has gaps. This phase closes those gaps systematically.

---

## 9a — Simulation Integrity (Determinism Verification)

**Problem**: We claim determinism but have no proof the app-layer sim is deterministic.
The core engine has `det_*.cpp` tests, but the game loop → AI → physics pipeline
has never been verified.

**Tasks**:
- Add state hash function: hash all entity positions, velocities, ball state, scores
- Record a snapshot hash every N physics steps
- Implement a replay mode: run sim for T steps, reset, run again, compare hashes
- Fix the duplicate `GameLoop` (Application + SceneState each own one — dead code)
- Fix `Application::run()` infinite loop (no path to SHUTDOWN state)
- Verify: two runs of identical 10-second sims produce identical hashes

**Files**: `apc_app/apc_application.h`, `apc_app/apc_scene_manager.h`,
         `apc_app/apc_game_loop.h`, new `tests/determinism/det_game_loop.cpp`

---

## 9b — AI Plays Soccer (Coherent Team Play)

**Problem**: The Utility AI has 16 action types but 5 have no scoring logic
(BLOCK, MARK_OPPONENT, CROSS, HEADER, DIVE_SAVE). Steering has 13 behaviors
but 3 are stubs (LEADER_FOLLOW, PATH_FOLLOWING, PREDICTION). The AI will
produce *something* but it won't look like football.

**Tasks**:
- Implement missing AI action scoring: BLOCK, MARK_OPPONENT, CROSS, HEADER, DIVE_SAVE
- Fix `pursue()` and `evade()` — they accept velocity but ignore it (cast to void)
- Fix `alignment()` — uses positions instead of velocities
- Wire TACKLE and PASS_BALL actions through the tick() switch (currently fall through)
- Add `position_quality` context factor (currently hardcoded 0.5f)
- Fix formation preset name matching in `AIMotorController::set_preset` (first-char collision)
- Tune weights so 4-4-2 vs 4-3-3 produces recognizable team shapes

**Files**: `apc_ai/apc_ai_decision.h`, `apc_ai/apc_ai_steering.h`,
         `apc_ai/apc_ai_motor.h`, `apc_app/apc_application.h`

---

## 9c — Contact Quality (Penetration / Friction Stability)

**Problem**: The solver works for demo scenarios but hasn't been stress-tested
with 22 athletes + 1 ball + ground plane simultaneously. Known risks:
ground penetration under stacking, friction jitter at low velocities, ball
tunneling at high speed.

**Tasks**:
- Add sequential impulse warmstarting (cache previous frame's impulses)
- Increase solver iterations for the ball (lighter, faster — needs more iterations)
- Add velocity threshold below which friction impulse is zeroed (prevent jitter)
- Implement ball-specific CCD (continuous collision detection) for high-speed shots
- Add sleep system: entities at rest for N frames skip collision detection
- Test: stack 10 spheres, verify no penetration after 5 seconds of settling

**Files**: `apc_solver/apc_si_solver_3d.h`, `apc_solver/apc_contact_manager.h`

---

## 9d — Test Coverage (App-Layer Tests)

**Problem**: 40 tests exist but they only cover the core engine (math, collision,
solver, skeleton). Zero tests for the application layer — game loop timing,
AI decision scoring, formation positions, motor intent, input mapping.

**Tasks**:
- `test_game_loop.cpp`: fixed timestep accumulator, pause/resume, time scale
- `test_ai_decision.cpp`: each action produces valid scores, role configs differ
- `test_ai_steering.cpp`: seek/flee/arrive/pursue produce correct directions
- `test_ai_formation.cpp`: all preset positions are valid, ball influence shifts
- `test_ai_motor.cpp`: preset names map to correct intent parameters
- `test_input_mapping.cpp`: button presses produce correct motor intent
- Target: +30 new assertions, all green

**Files**: new tests in `tests/correctness/` and `tests/integration/`

---

## 9e — Performance Profiling

**Problem**: Unknown performance characteristics. 22 entities at 240Hz = 5,280
physics steps per second. Broadphase is O(n²) brute force. No profiling data.

**Tasks**:
- Add high-resolution timer around: broadphase, narrowphase, solver, integrate, AI
- Print per-system timing every 60 frames to console
- Measure: can the sim sustain 240Hz with 22 entities on target hardware?
- If not: implement sweep-and-prune broadphase (sort by axis, O(n log n))
- Document performance budget per subsystem

**Files**: `apc_app/apc_application.h`, `apc_app/apc_game_loop.h`
