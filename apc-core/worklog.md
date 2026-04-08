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
