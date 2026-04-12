cmake_minimum_required(VERSION 3.20)
project(APCTests)

# ---------------------------------------------------------
# Determinism Tests
# ---------------------------------------------------------
# Test 1: Math ops combined
add_executable(det_main 
    determinism/det_main.cpp
    determinism/det_vec3_ops.cpp
    determinism/det_quat_ops.cpp
    determinism/det_mat3_ops.cpp
    determinism/det_game_loop.cpp
)
target_link_libraries(det_main PRIVATE apc_core)
add_test(NAME MathOpsDeterminism COMMAND det_main)

# Test 2: 10k frame trajectory simulation (Missing previously)
add_executable(det_trajectory_sim determinism/det_trajectory_sim.cpp)
target_link_libraries(det_trajectory_sim PRIVATE apc_core)
add_test(NAME TrajectorySimDeterminism COMMAND det_trajectory_sim)

# ---------------------------------------------------------
# Correctness Tests (Branch Coverage)
# ---------------------------------------------------------
add_executable(test_vec3 correctness/test_vec3.cpp)
target_link_libraries(test_vec3 PRIVATE apc_core)
add_test(NAME Vec3Correctness COMMAND test_vec3)

add_executable(test_quat correctness/test_quat.cpp)
target_link_libraries(test_quat PRIVATE apc_core)
add_test(NAME QuatCorrectness COMMAND test_quat)

add_executable(test_mat3 correctness/test_mat3.cpp)
target_link_libraries(test_mat3 PRIVATE apc_core)
add_test(NAME Mat3Correctness COMMAND test_mat3)

add_executable(test_skeleton_fk correctness/test_skeleton_fk.cpp)
target_link_libraries(test_skeleton_fk PRIVATE apc_core)
add_test(NAME SkeletonFKCorrectness COMMAND test_skeleton_fk)

add_executable(test_obb correctness/test_obb.cpp)
target_link_libraries(test_obb PRIVATE apc_core)
add_test(NAME OBBCorrectness COMMAND test_obb)

add_executable(test_gjk_epa correctness/test_gjk_epa.cpp)
target_link_libraries(test_gjk_epa PRIVATE apc_core)
add_test(NAME GJKEPACorrectness COMMAND test_gjk_epa)

add_executable(test_broadphase correctness/test_broadphase.cpp)
target_link_libraries(test_broadphase PRIVATE apc_core)
add_test(NAME BroadphaseCorrectness COMMAND test_broadphase)

add_executable(test_dispatch correctness/test_dispatch.cpp)
target_link_libraries(test_dispatch PRIVATE apc_core)
add_test(NAME DispatchCorrectness COMMAND test_dispatch)

# ---------------------------------------------------------
# FixedPoint 16.16 Deterministic Math
# ---------------------------------------------------------
add_executable(test_fixedpoint correctness/test_fixedpoint.cpp)
target_link_libraries(test_fixedpoint PRIVATE apc_core)
add_test(NAME FixedPointCorrectness COMMAND test_fixedpoint)

# ---------------------------------------------------------
# Integration Tests
# ---------------------------------------------------------
# Pipeline test: Broadphase + GJK narrowphase
add_executable(test_pipeline_sphere_box integration/test_pipeline_sphere_box.cpp)
target_link_libraries(test_pipeline_sphere_box PRIVATE apc_core)
add_test(NAME PipelineSphereBox COMMAND test_pipeline_sphere_box)

add_executable(test_3d_sphere_bounce integration/test_3d_sphere_bounce.cpp)
target_link_libraries(test_3d_sphere_bounce PRIVATE apc_core)
add_test(NAME 3DSolverSphereBounce COMMAND test_3d_sphere_bounce)

add_executable(test_apc_pendulum integration/test_apc_pendulum.cpp)
target_link_libraries(test_apc_pendulum PRIVATE apc_core)
add_test(NAME ABAIntegration COMMAND test_apc_pendulum)

add_executable(test_blend_modes integration/test_blend_modes.cpp)
target_link_libraries(test_blend_modes PRIVATE apc_core)
add_test(NAME BlendModes COMMAND test_blend_modes)

add_executable(test_friction integration/test_friction.cpp)
target_link_libraries(test_friction PRIVATE apc_core)
add_test(NAME FrictionIntegration COMMAND test_friction)

add_executable(test_solver_v2 integration/test_solver_v2.cpp)
target_link_libraries(test_solver_v2 PRIVATE apc_core)
add_test(NAME SolverV2Integration COMMAND test_solver_v2)

# ---------------------------------------------------------
# Sprint 4 Tests — Capsule, Cylinder, Plane-Box, Contact Manager
# ---------------------------------------------------------
add_executable(test_sprint4 correctness/test_sprint4.cpp)
target_link_libraries(test_sprint4 PRIVATE apc_core)
add_test(NAME Sprint4Correctness COMMAND test_sprint4)

# ---------------------------------------------------------
# Sprint 5 Tests — Joint Types, Limits, Extended API
# ---------------------------------------------------------
add_executable(test_sprint5 correctness/test_sprint5.cpp)
target_link_libraries(test_sprint5 PRIVATE apc_core)
add_test(NAME Sprint5Correctness COMMAND test_sprint5)

# ---------------------------------------------------------
# Sprint 6 Tests — Blend Modes, Spring-Damper Pipeline
# ---------------------------------------------------------
add_executable(test_sprint6_blend integration/test_sprint6_blend.cpp)
target_link_libraries(test_sprint6_blend PRIVATE apc_core)
add_test(NAME Sprint6BlendIntegration COMMAND test_sprint6_blend)

# ---------------------------------------------------------
# Sprint 7 Tests — Skeleton-Collision Integration
# ---------------------------------------------------------
add_executable(test_sprint7_collision integration/test_sprint7_collision.cpp)
target_link_libraries(test_sprint7_collision PRIVATE apc_core)
add_test(NAME Sprint7CollisionIntegration COMMAND test_sprint7_collision)

# ---------------------------------------------------------
# Sprint 8 Tests — Humanoid Demo, Loop Closures, Standing & Knock-Over
# ---------------------------------------------------------
add_executable(test_sprint8_humanoid integration/test_sprint8_humanoid.cpp)
target_link_libraries(test_sprint8_humanoid PRIVATE apc_core)
add_test(NAME Sprint8HumanoidIntegration COMMAND test_sprint8_humanoid)

# ---------------------------------------------------------
# Sprint 9 Tests — ImpactStyleProfile, MaterialCurve LUTs
# ---------------------------------------------------------
add_executable(test_sprint9 correctness/test_sprint9.cpp)
target_link_libraries(test_sprint9 PRIVATE apc_core)
add_test(NAME Sprint9Correctness COMMAND test_sprint9)

# ---------------------------------------------------------
# Sprint 10 Tests — StylizedSolver: Profile-Aware Non-Linear Solving
# ---------------------------------------------------------
add_executable(test_sprint10_stylized integration/test_sprint10_stylized.cpp)
target_link_libraries(test_sprint10_stylized PRIVATE apc_core)
add_test(NAME Sprint10StylizedIntegration COMMAND test_sprint10_stylized)

# ---------------------------------------------------------
# Sprint 11 Tests — Game Hooks: Hit-Stop, Time-Scale, Camera Shake, Blend Overrides
# ---------------------------------------------------------
add_executable(test_sprint11_hooks integration/test_sprint11_hooks.cpp)
target_link_libraries(test_sprint11_hooks PRIVATE apc_core)
add_test(NAME Sprint11HooksIntegration COMMAND test_sprint11_hooks)

# ---------------------------------------------------------
# Sprint 12 Tests — ImpactOutcomeTable + Tackle Vertical Slice
# ---------------------------------------------------------
add_executable(test_sprint12_tackle integration/test_sprint12_tackle.cpp)
target_link_libraries(test_sprint12_tackle PRIVATE apc_core)
add_test(NAME Sprint12TackleIntegration COMMAND test_sprint12_tackle)

# ---------------------------------------------------------
# Sprint 13 Tests — Ball Physics Core
# ---------------------------------------------------------
add_executable(test_sprint13 correctness/test_sprint13.cpp)
target_link_libraries(test_sprint13 PRIVATE apc_core)
add_test(NAME Sprint13Correctness COMMAND test_sprint13)

# ---------------------------------------------------------
# Sprint 14 Tests — Ball Control & Passing
# ---------------------------------------------------------
add_executable(test_sprint14_control integration/test_sprint14.cpp)
target_link_libraries(test_sprint14_control PRIVATE apc_core)
add_test(NAME Sprint14ControlIntegration COMMAND test_sprint14_control)

# ---------------------------------------------------------
# Sprint 15 Tests — Bat/Racket & Contact Sports
# ---------------------------------------------------------
add_executable(test_sprint15 correctness/test_sprint15.cpp)
target_link_libraries(test_sprint15 PRIVATE apc_core)
add_test(NAME Sprint15Correctness COMMAND test_sprint15)

# ---------------------------------------------------------
# Sprint 16 Tests — Sport Environments & Integration
# ---------------------------------------------------------
add_executable(test_sprint16 integration/test_sprint16.cpp)
target_link_libraries(test_sprint16 PRIVATE apc_core)
add_test(NAME Sprint16Integration COMMAND test_sprint16)

# ---------------------------------------------------------
# Sprint 17 Tests — Render Foundation & Debug Draw
# ---------------------------------------------------------
add_executable(test_sprint17 correctness/test_sprint17.cpp)
target_link_libraries(test_sprint17 PRIVATE apc_core)
add_test(NAME Sprint17Correctness COMMAND test_sprint17)

# ---------------------------------------------------------
# Sprint 18 Tests — Skeletal & Contact Visualization
# ---------------------------------------------------------
add_executable(test_sprint18 correctness/test_sprint18.cpp)
target_link_libraries(test_sprint18 PRIVATE apc_core)
add_test(NAME Sprint18Correctness COMMAND test_sprint18)

# ---------------------------------------------------------
# Sprint 19 Tests — Sport Field & Ball Visualization
# ---------------------------------------------------------
add_executable(test_sprint19 correctness/test_sprint19.cpp)
target_link_libraries(test_sprint19 PRIVATE apc_core)
add_test(NAME Sprint19Correctness COMMAND test_sprint19)

# ---------------------------------------------------------
# Sprint 20 Tests — Camera Systems, VFX Integration & Multi-Viewport
# ---------------------------------------------------------
add_executable(test_sprint20 correctness/test_sprint20.cpp)
target_link_libraries(test_sprint20 PRIVATE apc_core)
add_test(NAME Sprint20Correctness COMMAND test_sprint20)

# ---------------------------------------------------------
# Sprint 21 Tests — Input System: Types, State, Mapping
# ---------------------------------------------------------
add_executable(test_sprint21 correctness/test_sprint21.cpp)
target_link_libraries(test_sprint21 PRIVATE apc_core)
add_test(NAME Sprint21Correctness COMMAND test_sprint21)

# ---------------------------------------------------------
# Sprint 22 Tests — Entity System: Types, IDs, Manager
# ---------------------------------------------------------
add_executable(test_sprint22 correctness/test_sprint22.cpp)
target_link_libraries(test_sprint22 PRIVATE apc_core)
add_test(NAME Sprint22Correctness COMMAND test_sprint22)

# ---------------------------------------------------------
# Sprint 23 Tests — AI Steering Behaviors & Motor Controller
# ---------------------------------------------------------
add_executable(test_sprint23 correctness/test_sprint23.cpp)
target_link_libraries(test_sprint23 PRIVATE apc_core)
add_test(NAME Sprint23Correctness COMMAND test_sprint23)

# ---------------------------------------------------------
# Sprint 24 Tests — AI Decision System & Formation System
# ---------------------------------------------------------
add_executable(test_sprint24 correctness/test_sprint24.cpp)
target_link_libraries(test_sprint24 PRIVATE apc_core)
add_test(NAME Sprint24Correctness COMMAND test_sprint24)

# ---------------------------------------------------------
# Sprint 25 Tests — AI Debug Visualization
# ---------------------------------------------------------
add_executable(test_sprint25 correctness/test_sprint25.cpp)
target_link_libraries(test_sprint25 PRIVATE apc_core)
add_test(NAME Sprint25Correctness COMMAND test_sprint25)

# ---------------------------------------------------------
# Sprint 26 Tests — Game Loop & Scene Manager
# ---------------------------------------------------------
add_executable(test_sprint26 correctness/test_sprint26.cpp)
target_link_libraries(test_sprint26 PRIVATE apc_core)
add_test(NAME Sprint26Correctness COMMAND test_sprint26)

# ---------------------------------------------------------
# Sprint 27 Tests — Application Lifecycle
# ---------------------------------------------------------
add_executable(test_sprint27 correctness/test_sprint27.cpp)
target_link_libraries(test_sprint27 PRIVATE apc_core)
add_test(NAME Sprint27Correctness COMMAND test_sprint27)

# ---------------------------------------------------------
# Step 3 Integration Tests — App-Layer Coverage
# ---------------------------------------------------------
# AI Steering: pursue/evade/alignment/separation/cohesion math validation
add_executable(test_ai_steering integration/test_ai_steering.cpp)
target_link_libraries(test_ai_steering PRIVATE apc_core)
add_test(NAME AISteeringIntegration COMMAND test_ai_steering)

# Game Loop: fixed-timestep accumulator, pause/resume, time scaling, pipeline order
add_executable(test_game_loop integration/test_game_loop.cpp)
target_link_libraries(test_game_loop PRIVATE apc_core)
add_test(NAME GameLoopIntegration COMMAND test_game_loop)