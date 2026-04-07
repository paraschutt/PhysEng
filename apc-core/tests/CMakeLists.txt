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