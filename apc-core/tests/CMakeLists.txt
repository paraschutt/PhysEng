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
# ---------------------------------------------------------
# Integration Tests
# ---------------------------------------------------------
# COMMENTED OUT: Requires GJK/EPA/Broadphase which aren't in the core build target yet.
# add_executable(test_pipeline_sphere_box integration/test_pipeline_sphere_box.cpp)
# target_link_libraries(test_pipeline_sphere_box PRIVATE apc_core)
# add_test(NAME PipelineSphereBox COMMAND test_pipeline_sphere_box)

add_executable(test_3d_sphere_bounce integration/test_3d_sphere_bounce.cpp)
target_link_libraries(test_3d_sphere_bounce PRIVATE apc_core)
add_test(NAME 3DSolverSphereBounce COMMAND test_3d_sphere_bounce)

add_executable(test_apc_pendulum integration/test_apc_pendulum.cpp)
target_link_libraries(test_apc_pendulum PRIVATE apc_core)
add_test(NAME ABAIntegration COMMAND test_apc_pendulum)

add_executable(test_blend_modes integration/test_blend_modes.cpp)
target_link_libraries(test_blend_modes PRIVATE apc_core)
add_test(NAME BlendModes COMMAND test_blend_modes)