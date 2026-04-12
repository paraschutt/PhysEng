---
Task ID: 9
Agent: main
Task: Phase 9 — Engine/Sim/AI Consolidation (all sub-phases)

Work Log:
- Read and analyzed full codebase state: 29/40 tests passing, Phase 9a (determinism) already done
- Fixed GCC build issues: added -Wno-unused-variable/-Wno-unused-parameter, fixed test_vec3 unused var, fixed ball_physics.h unused return, fixed test_sprint25 wrong type
- Phase 9b: Implemented all 6 missing AI action scoring (BLOCK, MARK_OPPONENT, CROSS, HEADER, DIVE_SAVE, PUNT) in get_action_score(). Wired all into tick() steering switch. Registered all 15 actions in scene manager UtilityAI config.
- Phase 9d: Used subagent to fix all 11 failing tests (now 40/40 green)
- Phase 9c: Added friction_velocity_threshold to Solver3D to prevent friction jitter on resting contacts
- Phase 9e: Created apc_perf_timer.h with PerfTimer/PerfSection/PerfReport/ScopedTimer. Instrumented Application::tick() with scoped timers. Reports printed every 240 physics steps.

Stage Summary:
- All 40 tests passing (was 29)
- All 16 AIActionTypes have scoring logic + steering targets
- Performance profiling integrated into game loop
- 4 commits pushed to Opencode branch
---
Task ID: 1
Agent: main
Task: Step 1 (9b) — Fix AI Steering Math & Preset String Matching

Work Log:
- Read and analyzed apc_ai_steering.h (pursue, evade, alignment), apc_ai_motor.h (set_preset), test_sprint23.cpp
- Verified pursue() and evade() already use target_vel correctly for dynamic look-ahead prediction
- Replaced alignment() no-op stub with proper velocity-based implementation (new signature accepts velocities array)
- Fixed set_preset() first-character collision bug → replaced with std::strcmp
- Added formation flocking weights (separation_weight, cohesion_weight, alignment_weight) to AIMotorController
- Updated test_sprint23.cpp test 15 for new alignment() signature with velocity-based assertions
- Compiled all test files (sprints 23-27) to verify no regressions
- All 23 sprint23 tests pass; sprint24 failure is pre-existing (not caused by our changes)

Stage Summary:
- Commit: 7b92980 on Opencode branch
- Key insight: pursue/evade were already mathematically correct; alignment was a no-op stub; set_preset had a latent collision bug
- alignment() signature changed from (Vec3* neighbors, count, heading, max_force) to (Vec3* positions, Vec3* velocities, count, max_force)
- 5 presets now configure role-specific flocking weights
---
Task ID: 2
Agent: main
Task: Step 2 — Solver Stability & Sleep System

Work Log:
- Explored solver architecture: two parallel physics systems (EntityManager vs Solver3D pipeline)
- Read apc_rigid_body.h (53 lines), apc_contact_manager.h (674 lines), apc_si_solver_3d.h (207 lines)
- Identified that ContactManager already has warmstart infrastructure (APC_WARMSTART_FACTOR=0.8)
- Added BodyStateFlags enum and sleep system to RigidBody (state_flags, wake_up, try_sleep)
- Added apply_warmstart_to_constraint() bridge method to ContactManager
- Rewrote Solver3D with: prepare_warmstarted(), two-phase solve (base + ball extras), CCD ground check, sleep-aware integration
- Compiled and verified all solver-related tests: sprint4(96), solver_v2(6), sphere_bounce, friction(5), sprint10, sprint13, sprint17, sprint18, sprint23(23)

Stage Summary:
- Commit: 867f236 on Opencode branch
- 3 files changed, +524/-111 lines
- Key architectural decision: CCD uses swept-sphere vs ground plane (not OBB tree) for simplicity
- Sleep system uses bitwise flags for cache-friendly filtering in hot loops
- Dynamic ball iterations: 8 base + 4 extra for ball contacts (configurable)
---
Task ID: 3.1
Agent: Main Agent
Task: Implement Athlete-Ball Block Solver (Step 3.1)

Work Log:
- Read apc_si_solver_3d.h (407 lines), apc_collision_dispatch.h, apc_rigid_body.h
- Identified architecture: solver uses VelocityConstraint internally, not ContactManifold directly
- Confirmed STATE_IS_BALL exists as uint32_t bitmask flag (1u << 3u) on RigidBody
- Added `bool is_block_solved = false;` to ContactManifold struct in apc_collision_dispatch.h
- Added `bool is_block_solved = false;` to VelocityConstraint struct in apc_si_solver_3d.h
- Replaced `uint32_t ball_extra_iterations = 4;` with `bool enable_block_solve = true;`
- Implemented `solve_block_constraint()`: direct 1-shot normal impulse j = -(1+e)*v_n*normal_mass
- Rewrote `solve()`: Phase 1 block-solves ball contacts (XOR STATE_IS_BALL check), Phase 2 runs SI
- Updated `solve_single_constraint()`: skips normal impulse when is_block_solved, still computes friction
- Built successfully (zero compilation errors)
- All 43 tests passed via CTest
- Determinism verified: dual-run hashes match (52ba976540b02ef1)
- Committed as 9da2b3e, pushed to origin/Opencode

Stage Summary:
- Block solver eliminates 250:1 mass ratio convergence problem in single step
- ball_extra_iterations band-aid entirely removed (saved 4 SI passes per frame for ball contacts)
- Friction still iterated via SI using block-solved normal impulse for Coulomb cone limit
- Zero test regressions, determinism preserved
