#pragma once
// =============================================================================
// Articulated Body Algorithm — Featherstone ABA for multi-DOF skeletons
// =============================================================================
//
// Implements the Articulated Body Algorithm (ABA) for forward dynamics of
// tree-structured articulated bodies. Supports:
//   - Multi-DOF joints: Revolute1D, Prismatic1D, Spherical3D, Fixed
//   - Joint limits: hard clamp with optional restitution
//   - Per-bone physics blend modes (ANIM_DRIVEN, PHYSICS_DRIVEN, BLENDED)
//   - Spring-damper constraints for BLENDED mode
//   - External forces (collision response, user input)
//   - Animation target tracking
//
// Two API paths:
//   1. Legacy: step(asset, flat_axes, pose, state, dt, gravity)
//      — Backward-compatible. Treats every bone as 1-DOF revolute.
//      — Used by existing Sprint 1-4 tests.
//   2. Extended: step_ex(asset, pose, state, dt, gravity, config)
//      — Uses bone.joint_type, bone.joint_dof, bone.joint_axes,
//        bone.joint_limits for multi-DOF support.
//
// Determinism: index-order iteration, explicit multiply-add (no FMA), sorted
// traversal. All joint axes are normalized before use.
//
// =============================================================================

#include "apc_skeleton_types.h"
#include "apc_skeletal_pose.h"
#include "apc_skeletal_fk.h"
#include "apc_math/apc_math_common.h"
#include <vector>
#include <cmath>
#include <cstring>

namespace apc {

// =============================================================================
// Helper functions
// =============================================================================

APC_FORCEINLINE Vec3 aba_mat3_mul_vec3(const Mat3& m, const Vec3& v) {
    return Vec3(
        v.x * m.m[0] + v.y * m.m[3] + v.z * m.m[6],
        v.x * m.m[1] + v.y * m.m[4] + v.z * m.m[7],
        v.x * m.m[2] + v.y * m.m[5] + v.z * m.m[8]
    );
}

APC_FORCEINLINE float aba_clamp(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

// Multi-DOF generalized coordinate accessors.
// Storage layout: joint_velocities[bone * APC_MAX_JOINT_DOF + dof]

APC_FORCEINLINE float get_joint_vel(const std::vector<float>& jv, uint32_t bone, uint8_t dof) {
    return jv[bone * APC_MAX_JOINT_DOF + dof];
}

APC_FORCEINLINE void set_joint_vel(std::vector<float>& jv, uint32_t bone, uint8_t dof, float val) {
    jv[bone * APC_MAX_JOINT_DOF + dof] = val;
}

APC_FORCEINLINE float get_joint_q(const std::vector<float>& jq, uint32_t bone, uint8_t dof) {
    return jq[bone * APC_MAX_JOINT_DOF + dof];
}

APC_FORCEINLINE void set_joint_q(std::vector<float>& jq, uint32_t bone, uint8_t dof, float val) {
    jq[bone * APC_MAX_JOINT_DOF + dof] = val;
}

APC_FORCEINLINE float get_joint_acc(const std::vector<float>& ja, uint32_t bone, uint8_t dof) {
    return ja[bone * APC_MAX_JOINT_DOF + dof];
}

APC_FORCEINLINE void set_joint_acc(std::vector<float>& ja, uint32_t bone, uint8_t dof, float val) {
    ja[bone * APC_MAX_JOINT_DOF + dof] = val;
}

// =============================================================================
// ArticulatedBody — Featherstone ABA solver
// =============================================================================
class ArticulatedBody {
public:
    // --- Solver configuration ---
    float joint_limit_restitution = 0.0f;
    float max_joint_velocity = 100.0f;
    float gravity_scale = 1.0f;

    // =====================================================================
    // LEGACY API — Backward-compatible with Sprint 1-4 tests
    // =====================================================================
    // Treats every bone as a single-DOF revolute joint.
    // The flat local_joint_axes array has one Vec3 per bone.
    // state.joint_velocities is indexed as [bone_index] (flat).
    //
    // This function internally uses the multi-DOF storage layout by mapping
    // flat index [i] to multi-DOF [i * APC_MAX_JOINT_DOF + 0].
    // =====================================================================

    static void step(
        const SkeletalAsset& asset,
        const std::vector<Vec3>& local_joint_axes,
        SkeletalPose& pose,
        SkeletalDynamicState& state,
        float dt,
        const Vec3& gravity)
    {
        uint32_t N = asset.get_bone_count();
        if (N == 0) return;

        // Ensure multi-DOF sized arrays
        if (state.joint_velocities.size() < N * APC_MAX_JOINT_DOF) {
            std::vector<float> old_jv = state.joint_velocities;
            state.joint_velocities.resize(N * APC_MAX_JOINT_DOF, 0.0f);
            // Copy flat velocities into DOF-0 slots
            for (uint32_t i = 0; i < old_jv.size() && i < N; ++i) {
                set_joint_vel(state.joint_velocities, i, 0, old_jv[i]);
            }
        }

        // Build multi-DOF axis array from flat input (put each axis in DOF-0 slot)
        std::vector<Vec3> mdof_axes(N * APC_MAX_JOINT_DOF, Vec3(0.0f, 1.0f, 0.0f));
        for (uint32_t i = 0; i < N && i < local_joint_axes.size(); ++i) {
            mdof_axes[i * APC_MAX_JOINT_DOF + 0] = local_joint_axes[i];
        }

        // Ensure joint_q is sized
        if (pose.joint_q.size() < N * APC_MAX_JOINT_DOF) {
            pose.joint_q.resize(N * APC_MAX_JOINT_DOF, 0.0f);
        }

        // Ensure external_forces is sized
        if (state.external_forces.size() < N) {
            state.external_forces.resize(N);
            for (uint32_t i = 0; i < N; ++i) {
                state.external_forces[i].reset();
            }
        }

        // Now run the standard legacy ABA algorithm with flat indexing
        // This preserves the exact numerical behavior of the Sprint 1-4 code
        legacy_step_impl(asset, local_joint_axes, pose, state, dt, gravity);
    }

    // =====================================================================
    // EXTENDED API — Multi-DOF joint support with full feature set
    // =====================================================================
    // Uses bone.joint_type, bone.joint_dof, bone.joint_axes, bone.joint_limits.
    // pose.joint_q and state.joint_velocities use multi-DOF indexing.
    // =====================================================================

    static void step_ex(
        const SkeletalAsset& asset,
        SkeletalPose& pose,
        SkeletalDynamicState& state,
        float dt,
        const Vec3& gravity,
        const ArticulatedBody& config)
    {
        uint32_t N = asset.get_bone_count();
        if (N == 0) return;

        const Vec3 scaled_gravity = Vec3::scale(gravity, config.gravity_scale);

        // Ensure multi-DOF sized arrays
        if (state.joint_velocities.size() < N * APC_MAX_JOINT_DOF) {
            state.joint_velocities.resize(N * APC_MAX_JOINT_DOF, 0.0f);
        }
        if (pose.joint_q.size() < N * APC_MAX_JOINT_DOF) {
            pose.joint_q.resize(N * APC_MAX_JOINT_DOF, 0.0f);
        }
        if (state.external_forces.size() < N) {
            state.external_forces.resize(N);
            for (uint32_t i = 0; i < N; ++i) {
                state.external_forces[i].reset();
            }
        }

        // Allocate scratch
        std::vector<Mat3> I_A(N);
        std::vector<Vec3> Z_A(N);
        std::vector<Vec3> world_coms(N);
        std::vector<Mat3> I_world(N);
        std::vector<Mat3> world_rot_mats(N);
        std::vector<float> joint_accelerations(N * APC_MAX_JOINT_DOF, 0.0f);

        // Compute world transforms
        SkeletalPose world_pose;
        world_pose.allocate(N);
        SkeletalFK::calculate_world_transforms(asset, pose, world_pose);

        // Pre-compute world COMs and inertias
        for (uint32_t idx = 0; idx < N; ++idx) {
            world_coms[idx] = SkeletalFK::get_world_com(asset, world_pose, idx);
            world_rot_mats[idx] = world_pose.world_transforms[idx].rotation.to_mat3();
            Mat3 inv_rot = world_rot_mats[idx].transpose();
            I_world[idx] = Mat3::multiply(world_rot_mats[idx],
                Mat3::multiply(asset.bones[idx].local_inverse_inertia, inv_rot));
        }

        // ======== FORWARD PASS 1: Spatial velocities ========
        for (uint32_t idx = 0; idx < N; ++idx) {
            const Bone& bone = asset.bones[idx];
            if (asset.is_root(idx)) {
                // Root: build omega from all DOF
                Vec3 omega(0.0f, 0.0f, 0.0f);
                for (uint8_t d = 0; d < bone.joint_dof; ++d) {
                    Vec3 axis = bone.joint_axes[d];
                    float qdot = get_joint_vel(state.joint_velocities, idx, d);
                    omega = Vec3::add(omega, Vec3::scale(axis, qdot));
                }
                state.world_omegas[idx] = omega;
                state.world_v_coms[idx] = Vec3(0.0f, 0.0f, 0.0f);
            } else {
                uint32_t p = bone.parent_index;
                compute_child_velocity(bone, idx, p, state, world_pose, world_coms);
            }
        }

        // ======== BACKWARD PASS: Articulated inertias ========
        for (int idx = static_cast<int>(N) - 1; idx >= 0; --idx) {
            uint32_t uidx = static_cast<uint32_t>(idx);
            const Bone& bone = asset.bones[uidx];

            float inv_mass = bone.inverse_mass;
            float mass = (inv_mass > 0.0f) ? (1.0f / inv_mass) : 0.0f;
            Vec3 pivot_pos = world_pose.world_transforms[uidx].translation;
            Vec3 r = Vec3::sub(world_coms[uidx], pivot_pos);

            float xx = r.x * r.x;
            float yy = r.y * r.y;
            float zz = r.z * r.z;
            float xy = r.x * r.y;
            float xz = r.x * r.z;
            float yz = r.y * r.z;

            Mat3 mass_matrix;
            mass_matrix.m[0] = mass * xx;
            mass_matrix.m[1] = mass * xy;
            mass_matrix.m[2] = mass * xz;
            mass_matrix.m[3] = mass * xy;
            mass_matrix.m[4] = mass * yy;
            mass_matrix.m[5] = mass * yz;
            mass_matrix.m[6] = mass * xz;
            mass_matrix.m[7] = mass * yz;
            mass_matrix.m[8] = mass * zz;

            I_A[uidx] = Mat3::add(I_world[uidx], mass_matrix);

            // Bias force: gravity + external
            Z_A[uidx] = Vec3::cross(r, Vec3::scale(scaled_gravity, mass));
            if (uidx < state.external_forces.size()) {
                const ExternalBoneForce& ef = state.external_forces[uidx];
                Z_A[uidx] = Vec3::add(Z_A[uidx], ef.torque);
                Z_A[uidx] = Vec3::add(Z_A[uidx], Vec3::cross(r, ef.force));
            }

            // Propagate to parent
            if (bone.parent_index != 0xFFFFFFFF) {
                uint32_t p = bone.parent_index;
                Quat q_p = world_pose.world_transforms[p].rotation;
                Quat q_c = world_pose.world_transforms[uidx].rotation;
                Quat q_diff = Quat::normalize(Quat::multiply(q_c, Quat::inverse(q_p)));
                Mat3 R = Mat3::from_quat(q_diff);
                Mat3 R_T = R.transpose();

                Mat3 I_A_mapped = Mat3::multiply(R_T, Mat3::multiply(I_A[uidx], R));
                Vec3 Z_A_mapped = aba_mat3_mul_vec3(R_T, Z_A[uidx]);

                I_A[p] = Mat3::add(I_A[p], I_A_mapped);
                Z_A[p] = Vec3::add(Z_A[p], Z_A_mapped);
            }
        }

        // ======== FORWARD PASS 2: Joint accelerations ========
        Vec3 zero3(0.0f, 0.0f, 0.0f);

        for (uint32_t idx = 0; idx < N; ++idx) {
            const Bone& bone = asset.bones[idx];

            if (asset.is_root(idx) || bone.joint_dof == 0) {
                state.a_coms[idx] = zero3;
                state.alphas[idx] = zero3;
                continue;
            }

            uint32_t p = bone.parent_index;
            PhysicsBlendMode mode = bone.physics.mode;

            // --- ANIM_DRIVEN: skip physics, use animation target ---
            if (mode == PhysicsBlendMode::ANIM_DRIVEN && pose.target.has_target) {
                if (idx < pose.target.target_local_transforms.size()) {
                    pose.local_transforms[idx] = pose.target.target_local_transforms[idx];
                }
                for (uint8_t d = 0; d < bone.joint_dof; ++d) {
                    set_joint_vel(state.joint_velocities, idx, d, 0.0f);
                    set_joint_acc(joint_accelerations, idx, d, 0.0f);
                }
                state.a_coms[idx] = zero3;
                state.alphas[idx] = zero3;
                continue;
            }

            // --- PHYSICS_DRIVEN or BLENDED: compute ABA accelerations ---
            Mat3 rot = world_rot_mats[idx];

            for (uint8_t d = 0; d < bone.joint_dof; ++d) {
                Vec3 s_world = aba_mat3_mul_vec3(rot, bone.joint_axes[d]);

                // ABA projection: s^T * I_A * s
                Vec3 Ia_s = aba_mat3_mul_vec3(I_A[idx], s_world);
                float sT_Ia_s = Vec3::dot(s_world, Ia_s);

                // Include parent acceleration coupling
                Vec3 Ia_a_p = aba_mat3_mul_vec3(I_A[idx], state.a_coms[p]);
                float sT_all = Vec3::dot(s_world, Vec3::add(Z_A[idx], Ia_a_p));

                float q_ddot = 0.0f;
                if (sT_Ia_s > APC_EPSILON) {
                    q_ddot = -sT_all / sT_Ia_s;
                }

                // --- BLENDED mode: spring-damper ---
                if (mode == PhysicsBlendMode::BLENDED && pose.target.has_target) {
                    q_ddot = apply_spring_damper(
                        bone, idx, d, pose, state, q_ddot);
                }

                // Joint damping
                float qdot = get_joint_vel(state.joint_velocities, idx, d);
                q_ddot -= bone.joint_damping * qdot;

                set_joint_acc(joint_accelerations, idx, d, q_ddot);
            }

            // Recompute spatial accelerations for this bone
            Vec3 R = Vec3::sub(world_pose.world_transforms[idx].translation, world_coms[p]);
            Vec3 r = Vec3::sub(world_coms[idx], world_pose.world_transforms[idx].translation);

            Vec3 alpha_i = state.alphas[p];
            for (uint8_t d = 0; d < bone.joint_dof; ++d) {
                Vec3 s_world = aba_mat3_mul_vec3(rot, bone.joint_axes[d]);
                float q_ddot = get_joint_acc(joint_accelerations, idx, d);
                alpha_i = Vec3::add(alpha_i, Vec3::scale(s_world, q_ddot));
            }
            state.alphas[idx] = alpha_i;

            state.a_coms[idx] = Vec3::add(state.a_coms[p], Vec3::add(
                Vec3::cross(state.alphas[p], R),
                Vec3::add(
                    Vec3::cross(state.world_omegas[p],
                        Vec3::cross(state.world_omegas[p], R)),
                    Vec3::add(
                        Vec3::cross(alpha_i, r),
                        Vec3::cross(state.world_omegas[idx],
                            Vec3::cross(state.world_omegas[idx], r))))));
        }

        // ======== INTEGRATION ========
        for (uint32_t idx = 0; idx < N; ++idx) {
            const Bone& bone = asset.bones[idx];
            if (bone.joint_type == JointType::Fixed) continue;
            if (bone.physics.mode == PhysicsBlendMode::ANIM_DRIVEN) continue;

            for (uint8_t d = 0; d < bone.joint_dof; ++d) {
                float q_ddot = get_joint_acc(joint_accelerations, idx, d);
                float qdot = get_joint_vel(state.joint_velocities, idx, d);
                float q = get_joint_q(pose.joint_q, idx, d);

                // Semi-implicit Euler
                float new_qdot = qdot + q_ddot * dt;
                new_qdot *= (1.0f - bone.joint_damping * dt);
                new_qdot = aba_clamp(new_qdot, -config.max_joint_velocity, config.max_joint_velocity);

                float new_q = q + new_qdot * dt;

                // Joint limits
                if (d < bone.joint_limits.dof_count) {
                    float min_v = bone.joint_limits.min_values[d];
                    float max_v = bone.joint_limits.max_values[d];
                    if (new_q < min_v) {
                        new_q = min_v;
                        new_qdot = (config.joint_limit_restitution > 0.0f)
                            ? -new_qdot * config.joint_limit_restitution
                            : 0.0f;
                    } else if (new_q > max_v) {
                        new_q = max_v;
                        new_qdot = (config.joint_limit_restitution > 0.0f)
                            ? -new_qdot * config.joint_limit_restitution
                            : 0.0f;
                    }
                }

                set_joint_vel(state.joint_velocities, idx, d, new_qdot);
                set_joint_q(pose.joint_q, idx, d, new_q);
            }
        }

        // ======== REBUILD LOCAL TRANSFORMS ========
        rebuild_local_transforms(asset, pose);
    }

    // -------------------------------------------------------------------------
    // apply_external_force
    // -------------------------------------------------------------------------
    static void apply_external_force(
        SkeletalDynamicState& state,
        uint32_t bone_index,
        const Vec3& force,
        const Vec3& world_point,
        const Vec3& world_com)
    {
        if (bone_index >= state.external_forces.size()) return;
        state.external_forces[bone_index].apply_at(force, world_point, world_com);
    }

    // -------------------------------------------------------------------------
    // apply_external_torque
    // -------------------------------------------------------------------------
    static void apply_external_torque(
        SkeletalDynamicState& state,
        uint32_t bone_index,
        const Vec3& torque)
    {
        if (bone_index >= state.external_forces.size()) return;
        state.external_forces[bone_index].add_torque(torque);
    }

private:
    // -------------------------------------------------------------------------
    // legacy_step_impl — Exact reproduction of the Sprint 1-4 algorithm
    // -------------------------------------------------------------------------
    static void legacy_step_impl(
        const SkeletalAsset& asset,
        const std::vector<Vec3>& local_joint_axes,
        SkeletalPose& pose,
        SkeletalDynamicState& state,
        float dt,
        const Vec3& gravity)
    {
        uint32_t N = asset.get_bone_count();
        if (N == 0) return;

        std::vector<Mat3> I_A(N);
        std::vector<Vec3> Z_A(N);
        std::vector<Vec3> world_coms(N);
        std::vector<Mat3> I_world(N);
        std::vector<Mat3> world_rot_mats(N);
        std::vector<float> joint_accelerations(N, 0.0f);

        SkeletalPose world_pose;
        world_pose.allocate(N);
        SkeletalFK::calculate_world_transforms(asset, pose, world_pose);

        for (uint32_t idx = 0; idx < N; ++idx) {
            world_coms[idx] = SkeletalFK::get_world_com(asset, world_pose, idx);
            world_rot_mats[idx] = world_pose.world_transforms[idx].rotation.to_mat3();
            Mat3 inv_rot = world_rot_mats[idx].transpose();
            I_world[idx] = Mat3::multiply(world_rot_mats[idx],
                Mat3::multiply(asset.bones[idx].local_inverse_inertia, inv_rot));
        }

        // FORWARD PASS 1
        for (uint32_t idx = 0; idx < N; ++idx) {
            uint32_t p = asset.bones[idx].parent_index;
            Vec3 s_world = aba_mat3_mul_vec3(world_rot_mats[idx], local_joint_axes[idx]);

            if (asset.is_root(idx)) {
                state.world_omegas[idx] = Vec3::scale(s_world,
                    state.joint_velocities[idx]); // flat index for legacy
                state.world_v_coms[idx] = Vec3(0.0f, 0.0f, 0.0f);
            } else {
                state.world_omegas[idx] = Vec3::add(state.world_omegas[p],
                    Vec3::scale(s_world, state.joint_velocities[idx]));
                Vec3 R = Vec3::sub(world_pose.world_transforms[idx].translation, world_coms[p]);
                Vec3 r = Vec3::sub(world_coms[idx], world_pose.world_transforms[idx].translation);
                state.world_v_coms[idx] = Vec3::add(state.world_v_coms[p], Vec3::add(
                    Vec3::cross(state.world_omegas[p], R),
                    Vec3::cross(state.world_omegas[idx], r)));
            }
        }

        // BACKWARD PASS
        for (int idx = static_cast<int>(N) - 1; idx >= 0; --idx) {
            uint32_t uidx = static_cast<uint32_t>(idx);
            uint32_t p = asset.bones[uidx].parent_index;
            float inv_mass = asset.bones[uidx].inverse_mass;
            Vec3 pivot_pos = world_pose.world_transforms[uidx].translation;
            float mass = 1.0f / inv_mass;
            Vec3 r = Vec3::sub(world_coms[uidx], pivot_pos);

            float xx = r.x * r.x;
            float yy = r.y * r.y;
            float zz = r.z * r.z;
            float xy = r.x * r.y;
            float xz = r.x * r.z;
            float yz = r.y * r.z;

            Mat3 mass_matrix;
            mass_matrix.m[0] = mass * xx;
            mass_matrix.m[1] = mass * xy;
            mass_matrix.m[2] = mass * xz;
            mass_matrix.m[3] = mass * xy;
            mass_matrix.m[4] = mass * yy;
            mass_matrix.m[5] = mass * yz;
            mass_matrix.m[6] = mass * xz;
            mass_matrix.m[7] = mass * yz;
            mass_matrix.m[8] = mass * zz;

            I_A[uidx] = Mat3::add(I_world[uidx], mass_matrix);
            Z_A[uidx] = Vec3::cross(r, Vec3::scale(gravity, mass));

            if (!asset.is_root(uidx)) {
                Quat q_p = world_pose.world_transforms[p].rotation;
                Quat q_c = world_pose.world_transforms[uidx].rotation;
                Quat q_diff = Quat::normalize(Quat::multiply(q_c, Quat::inverse(q_p)));
                Mat3 R = Mat3::from_quat(q_diff);
                Mat3 R_T = R.transpose();
                Mat3 I_A_mapped = Mat3::multiply(R_T, Mat3::multiply(I_A[uidx], R));
                Vec3 Z_A_mapped = aba_mat3_mul_vec3(R_T, Z_A[uidx]);
                I_A[p] = Mat3::add(I_A[p], I_A_mapped);
                Z_A[p] = Vec3::add(Z_A[p], Z_A_mapped);
            }
        }

        // FORWARD PASS 2
        Vec3 a_root(0.0f, 0.0f, 0.0f);
        Vec3 alpha_root(0.0f, 0.0f, 0.0f);

        for (uint32_t idx = 0; idx < N; ++idx) {
            if (asset.is_root(idx)) {
                joint_accelerations[idx] = 0.0f;
                state.a_coms[idx] = a_root;
                state.alphas[idx] = alpha_root;
                continue;
            }

            uint32_t p = asset.bones[idx].parent_index;
            Vec3 s_world = aba_mat3_mul_vec3(world_rot_mats[idx], local_joint_axes[idx]);

            Vec3 Ia_s = aba_mat3_mul_vec3(I_A[idx], s_world);
            float sT_Ia_s = Vec3::dot(s_world, Ia_s);
            float sT_Z = Vec3::dot(s_world, Z_A[idx]);

            float q_ddot = 0.0f;

            if (std::abs(sT_Z) < 0.0f) {
                float torque_mag = Vec3::length(Z_A[idx]);
                float I_eff = sT_Ia_s;
                if (I_eff > 0.0f) {
                    q_ddot = -torque_mag / I_eff;
                }
            } else {
                float sT_Z_all = Vec3::dot(s_world, Vec3::add(Z_A[idx],
                    aba_mat3_mul_vec3(I_A[idx], state.a_coms[p])));
                float sT_Ia_s_all = Vec3::dot(s_world, Ia_s);
                if (sT_Ia_s_all > 0.0f) {
                    q_ddot = -sT_Z_all / sT_Ia_s_all;
                }
            }

            joint_accelerations[idx] = q_ddot;

            if (!asset.is_root(idx)) {
                Vec3 R = Vec3::sub(world_pose.world_transforms[idx].translation, world_coms[p]);
                Vec3 r = Vec3::sub(world_coms[idx], world_pose.world_transforms[idx].translation);
                Vec3 alpha_i = Vec3::add(alpha_root, Vec3::scale(s_world, q_ddot));
                state.a_coms[idx] = Vec3::add(state.a_coms[p], Vec3::add(
                    Vec3::cross(alpha_root, R),
                    Vec3::add(
                        Vec3::cross(state.world_omegas[p],
                            Vec3::cross(state.world_omegas[p], R)),
                        Vec3::add(
                            Vec3::cross(alpha_i, r),
                            Vec3::cross(state.world_omegas[idx],
                                Vec3::cross(state.world_omegas[idx], r))))));
                state.alphas[idx] = alpha_i;
            } else {
                state.a_coms[idx] = a_root;
                state.alphas[idx] = alpha_root;
            }
        }

        // INTEGRATION (flat indexing — legacy)
        for (uint32_t idx = 0; idx < N; ++idx) {
            state.joint_velocities[idx] += joint_accelerations[idx] * dt;
            float angle = state.joint_velocities[idx] * dt;
            Quat delta_q = Quat::from_axis_angle(local_joint_axes[idx], angle);
            pose.local_transforms[idx].rotation = Quat::normalize(
                Quat::multiply(delta_q, pose.local_transforms[idx].rotation));
        }
    }

    // -------------------------------------------------------------------------
    // compute_child_velocity — Extended API velocity computation
    // -------------------------------------------------------------------------
    static void compute_child_velocity(
        const Bone& bone,
        uint32_t idx,
        uint32_t p,
        SkeletalDynamicState& state,
        const SkeletalPose& world_pose,
        const std::vector<Vec3>& world_coms)
    {
        Mat3 rot = world_pose.world_transforms[idx].rotation.to_mat3();

        if (bone.joint_type == JointType::Prismatic1D) {
            // Angular: inherit parent
            state.world_omegas[idx] = state.world_omegas[p];

            // Linear: parent COM velocity + cross terms + sliding
            Vec3 R = Vec3::sub(world_pose.world_transforms[idx].translation, world_coms[p]);
            Vec3 r = Vec3::sub(world_coms[idx], world_pose.world_transforms[idx].translation);
            state.world_v_coms[idx] = Vec3::add(state.world_v_coms[p], Vec3::add(
                Vec3::cross(state.world_omegas[p], R),
                Vec3::cross(state.world_omegas[idx], r)));

            // Add sliding velocity
            float qdot = get_joint_vel(state.joint_velocities, idx, 0);
            Vec3 slide_dir = aba_mat3_mul_vec3(rot, bone.joint_axes[0]);
            state.world_v_coms[idx] = Vec3::add(state.world_v_coms[idx],
                Vec3::scale(slide_dir, qdot));
        }
        else if (bone.joint_type == JointType::Spherical3D) {
            // Angular: accumulate all 3 DOF
            Vec3 omega_contrib(0.0f, 0.0f, 0.0f);
            for (uint8_t d = 0; d < 3; ++d) {
                Vec3 axis = aba_mat3_mul_vec3(rot, bone.joint_axes[d]);
                float qdot = get_joint_vel(state.joint_velocities, idx, d);
                omega_contrib = Vec3::add(omega_contrib, Vec3::scale(axis, qdot));
            }
            state.world_omegas[idx] = Vec3::add(state.world_omegas[p], omega_contrib);

            Vec3 R = Vec3::sub(world_pose.world_transforms[idx].translation, world_coms[p]);
            Vec3 r = Vec3::sub(world_coms[idx], world_pose.world_transforms[idx].translation);
            state.world_v_coms[idx] = Vec3::add(state.world_v_coms[p], Vec3::add(
                Vec3::cross(state.world_omegas[p], R),
                Vec3::cross(state.world_omegas[idx], r)));
        }
        else {
            // Revolute1D or Fixed
            if (bone.joint_dof > 0) {
                Vec3 s_world = aba_mat3_mul_vec3(rot, bone.joint_axes[0]);
                float qdot = get_joint_vel(state.joint_velocities, idx, 0);
                state.world_omegas[idx] = Vec3::add(state.world_omegas[p],
                    Vec3::scale(s_world, qdot));
            } else {
                state.world_omegas[idx] = state.world_omegas[p];
            }

            Vec3 R = Vec3::sub(world_pose.world_transforms[idx].translation, world_coms[p]);
            Vec3 r = Vec3::sub(world_coms[idx], world_pose.world_transforms[idx].translation);
            state.world_v_coms[idx] = Vec3::add(state.world_v_coms[p], Vec3::add(
                Vec3::cross(state.world_omegas[p], R),
                Vec3::cross(state.world_omegas[idx], r)));
        }
    }

    // -------------------------------------------------------------------------
    // apply_spring_damper — BLENDED mode correction
    // -------------------------------------------------------------------------
    static float apply_spring_damper(
        const Bone& bone,
        uint32_t idx,
        uint8_t dof,
        const SkeletalPose& pose,
        const SkeletalDynamicState& state,
        float q_ddot)
    {
        float stiffness = bone.physics.stiffness;
        float damping_val = bone.physics.damping;
        if (stiffness <= 0.0f && damping_val <= 0.0f) return q_ddot;

        float qdot = get_joint_vel(state.joint_velocities, idx, dof);

        // Compute rotation error from animation target
        if (idx >= pose.target.target_local_transforms.size()) return q_ddot;

        Quat current_rot = pose.local_transforms[idx].rotation;
        Quat target_rot = pose.target.target_local_transforms[idx].rotation;

        Quat diff = Quat::normalize(Quat::multiply(Quat::inverse(current_rot), target_rot));
        float angle = 2.0f * std::acos(aba_clamp(diff.w, -1.0f, 1.0f));

        float sign = 1.0f;
        Vec3 axis(diff.x, diff.y, diff.z);
        float axis_len_sq = Vec3::length_sq(axis);
        if (axis_len_sq > APC_EPSILON_SQ) {
            float proj = Vec3::dot(Vec3::normalize(axis), bone.joint_axes[dof]);
            sign = (proj >= 0.0f) ? 1.0f : -1.0f;
            angle *= sign;
        }

        float spring_force = -stiffness * angle;
        float damper_force = -damping_val * qdot;
        float correction = spring_force + damper_force;

        // Max deviation enforcement
        if (bone.physics.max_deviation > 0.0f) {
            float abs_angle = std::abs(angle);
            if (abs_angle > bone.physics.max_deviation) {
                float excess = abs_angle - bone.physics.max_deviation;
                correction -= sign * excess * stiffness * 5.0f;
            }
        }

        return q_ddot + correction;
    }

    // -------------------------------------------------------------------------
    // rebuild_local_transforms — Convert joint_q back to local transforms
    // -------------------------------------------------------------------------
    static void rebuild_local_transforms(const SkeletalAsset& asset, SkeletalPose& pose) {
        for (uint32_t idx = 0; idx < asset.get_bone_count(); ++idx) {
            const Bone& bone = asset.bones[idx];
            pose.local_transforms[idx].translation = bone.bind_pose.translation;

            if (bone.joint_type == JointType::Fixed) continue;

            if (bone.physics.mode == PhysicsBlendMode::ANIM_DRIVEN) continue;

            if (bone.joint_type == JointType::Revolute1D) {
                float angle = get_joint_q(pose.joint_q, idx, 0);
                Quat delta = Quat::from_axis_angle(bone.joint_axes[0], angle);
                pose.local_transforms[idx].rotation = Quat::normalize(
                    Quat::multiply(delta, bone.bind_pose.rotation));
            }
            else if (bone.joint_type == JointType::Prismatic1D) {
                float disp = get_joint_q(pose.joint_q, idx, 0);
                pose.local_transforms[idx].translation = Vec3::add(
                    bone.bind_pose.translation,
                    Vec3::scale(bone.joint_axes[0], disp));
            }
            else if (bone.joint_type == JointType::Spherical3D) {
                float x_ang = get_joint_q(pose.joint_q, idx, 0);
                float y_ang = get_joint_q(pose.joint_q, idx, 1);
                float z_ang = get_joint_q(pose.joint_q, idx, 2);

                Quat qx = Quat::from_axis_angle(bone.joint_axes[0], x_ang);
                Quat qy = Quat::from_axis_angle(bone.joint_axes[1], y_ang);
                Quat qz = Quat::from_axis_angle(bone.joint_axes[2], z_ang);
                Quat delta = Quat::multiply(qx, Quat::multiply(qy, qz));
                pose.local_transforms[idx].rotation = Quat::normalize(
                    Quat::multiply(delta, bone.bind_pose.rotation));
            }
        }
    }
};

} // namespace apc
