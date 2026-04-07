#pragma once
#include "apc_skeleton_types.h"
#include "apc_skeletal_pose.h"
#include "apc_skeletal_fk.h"
#include <vector>

namespace apc {

inline Vec3 mat3_mul_vec3(const Mat3& m, const Vec3& v) {
    return Vec3(
        v.x * m.m[0] + v.y * m.m[3] + v.z * m.m[6],
        v.x * m.m[1] + v.y * m.m[4] + v.z * m.m[7],
        v.x * m.m[2] + v.y * m.m[5] + v.z * m.m[8]
    );
}

struct SkeletalDynamicState {
    std::vector<float> joint_velocities;     
    std::vector<Vec3> world_omegas;          
    std::vector<Vec3> world_v_coms;          
    std::vector<Vec3> a_coms;               
    std::vector<Vec3> alphas;               

    void allocate(uint32_t bone_count) {
        joint_velocities.resize(bone_count, 0.0f);
        world_omegas.resize(bone_count, Vec3(0.0f, 0.0f, 0.0f));
        world_v_coms.resize(bone_count, Vec3(0.0f, 0.0f, 0.0f));
        a_coms.resize(bone_count, Vec3(0.0f, 0.0f, 0.0f));
        alphas.resize(bone_count, Vec3(0.0f, 0.0f, 0.0f));
    }
};

class ArticulatedBody {
public:
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

        std::vector<Mat3> I_A(N);       
        std::vector<Vec3> Z_A(N);       
        std::vector<Vec3> world_coms(N); 
        std::vector<Mat3> I_world(N);
        std::vector<Mat3> world_rot_mats(N);

        SkeletalPose world_pose;
        world_pose.allocate(N);
        SkeletalFK::calculate_world_transforms(asset, pose, world_pose);

        for (uint32_t idx = 0; idx < N; ++idx) {
            world_coms[idx] = SkeletalFK::get_world_com(asset, world_pose, idx);
            world_rot_mats[idx] = world_pose.world_transforms[idx].rotation.to_mat3();
            
            Mat3 inv_rot = world_rot_mats[idx].transpose();
            I_world[idx] = Mat3::multiply(world_rot_mats[idx], Mat3::multiply(asset.bones[idx].local_inverse_inertia, inv_rot));
        }

        // FORWARD PASS 1
        for (uint32_t idx = 0; idx < N; ++idx) {
            uint32_t p = asset.bones[idx].parent_index;
            Vec3 s_world = mat3_mul_vec3(world_rot_mats[idx], local_joint_axes[idx]);
            
            if (asset.is_root(idx)) {
                state.world_omegas[idx] = Vec3::scale(s_world, state.joint_velocities[idx]);
                state.world_v_coms[idx] = Vec3(0.0f, 0.0f, 0.0f);
            } else {
                state.world_omegas[idx] = Vec3::add(state.world_omegas[p], Vec3::scale(s_world, state.joint_velocities[idx]));
                
                Vec3 R = Vec3::sub(world_pose.world_transforms[idx].translation, world_coms[p]);
                Vec3 r = Vec3::sub(world_coms[idx], world_pose.world_transforms[idx].translation);
                
                state.world_v_coms[idx] = Vec3::add(state.world_v_coms[p], Vec3::add(
                    Vec3::cross(state.world_omegas[p], R),
                    Vec3::cross(state.world_omegas[idx], r)
                ));
            }
        }

        // BACKWARD PASS
        for (int idx = N - 1; idx >= 0; --idx) {
            uint32_t p = asset.bones[idx].parent_index;
            float inv_mass = asset.bones[idx].inverse_mass;
            Vec3 pivot_pos = world_pose.world_transforms[idx].translation;
            
            float mass = 1.0f / inv_mass;
            Vec3 r = Vec3::sub(world_coms[idx], pivot_pos); 
            
            // CORRECT 3D [r]x[r]x calculation
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
            mass_matrix.m[5] = mass_matrix.m[1]; // xy
            mass_matrix.m[6] = mass * xz;
            mass_matrix.m[7] = mass_matrix.m[2]; // xz
            mass_matrix.m[8] = mass * zz;
            
            I_A[idx] = Mat3::add(I_world[idx], mass_matrix);
            Z_A[idx] = Vec3::cross(r, Vec3::scale(gravity, mass));

            if (!asset.is_root(idx)) {
                Quat q_p = world_pose.world_transforms[p].rotation;
                Quat q_c = world_pose.world_transforms[idx].rotation;
                
                Quat q_diff = Quat::normalize(Quat::multiply(q_c, Quat::inverse(q_p)));
                Mat3 R = Mat3::from_quat(q_diff);
                Mat3 R_T = R.transpose();
                
                Mat3 I_A_mapped = Mat3::multiply(R_T, Mat3::multiply(I_A[idx], R));
                Vec3 Z_A_mapped = mat3_mul_vec3(R_T, Z_A[idx]);
                
                I_A[p] = Mat3::add(I_A[p], I_A_mapped);
                Z_A[p] = Vec3::add(Z_A[p], Z_A_mapped);
            }
        }

        // FORWARD PASS 2: Compute Accelerations
        std::vector<float> joint_accelerations(N, 0.0f);
        Vec3 a_root = Vec3(0.0f, 0.0f, 0.0f); 
        Vec3 alpha_root = Vec3(0.0f, 0.0f, 0.0f);
        
        for (uint32_t idx = 0; idx < N; ++idx) {
            if (asset.is_root(idx)) {
                joint_accelerations[idx] = 0.0f;
                state.a_coms[idx] = a_root;
                state.alphas[idx] = alpha_root;
                continue;
            }

            uint32_t p = asset.bones[idx].parent_index;
            Vec3 s_world = mat3_mul_vec3(world_rot_mats[idx], local_joint_axes[idx]);
            
            Vec3 a_p = state.a_coms[p];
            Vec3 alpha_p = state.alphas[p];
            
            // Calculate the standard ABA projection: s^T * I_A * s
            Vec3 Ia_s = mat3_mul_vec3(I_A[idx], s_world);
            float sT_Ia_s = Vec3::dot(s_world, Ia_s);
            
            // Project bias forces (Z_A) onto joint axis
            float sT_Z = Vec3::dot(s_world, Z_A[idx]);
            
            float q_ddot = 0.0f;
            
            if (std::abs(sT_Z) < 0.0f) {
                // 1-DOF Edge Case Fallback:
                // The constraint axis is perpendicular to the force/torque vector (e.g., 2D systems on perpendicular hinges).
                // Standard s^T_Z_A projection is zero. Standard ABA fails here.
                // Fallback to Lagrangian magnitude mapping.
                float torque_mag = Vec3::length(Z_A[idx]);
                float I_eff = sT_Ia_s; // Effective inertia along joint axis
                
                if (I_eff > 0.0f) {
                    q_ddot = -torque_mag / I_eff;
                } else {
                    q_ddot = 0.0f;
                }
            } else {
                // Standard ABA mapping for 3D multi-body systems
                float sT_Z_all = Vec3::dot(s_world, Vec3::add(Z_A[idx], mat3_mul_vec3(I_A[idx], a_p)));
                float sT_Ia_s_all = Vec3::dot(s_world, Ia_s);
                
                if (sT_Ia_s_all > 0.0f) {
                    q_ddot = -sT_Z_all / sT_Ia_s_all; 
                }
            }

            joint_accelerations[idx] = q_ddot;

            if (!asset.is_root(idx)) {
                Vec3 R = Vec3::sub(world_pose.world_transforms[idx].translation, world_coms[p]);
                Vec3 r = Vec3::sub(world_coms[idx], world_pose.world_transforms[idx].translation);
                
                Vec3 alpha_i = Vec3::add(alpha_p, Vec3::scale(s_world, q_ddot));
                
                state.a_coms[idx] = Vec3::add(a_p, Vec3::add(
                    Vec3::cross(alpha_p, R),
                    Vec3::add(Vec3::cross(state.world_omegas[p], Vec3::cross(state.world_omegas[p], R)),
                               Vec3::add(Vec3::cross(alpha_i, r),
                                          Vec3::cross(state.world_omegas[idx], Vec3::cross(state.world_omegas[idx], r))))
                ));
                
                state.alphas[idx] = alpha_i; 
            } else {
                state.a_coms[idx] = a_root;
                state.alphas[idx] = alpha_root;
            }
        }

        // INTEGRATION
        for (uint32_t idx = 0; idx < N; ++idx) {
            state.joint_velocities[idx] += joint_accelerations[idx] * dt;
            
            float angle = state.joint_velocities[idx] * dt;
            Quat delta_q = Quat::from_axis_angle(local_joint_axes[idx], angle);
            pose.local_transforms[idx].rotation = Quat::normalize(Quat::multiply(delta_q, pose.local_transforms[idx].rotation));
        }
    }
};

} // namespace apc