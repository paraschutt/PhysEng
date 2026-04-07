#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include "apc_math/apc_mat3.h"
#include "apc_platform/apc_fp_mode.h"
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace apc::test {

// Simple hash combiner (FNV-1a variant)
struct HashAccumulator {
    uint64_t state = 0xcbf29ce484222325ull;
    
    void add_bytes(const void* data, size_t len) {
        const uint8_t* bytes = static_cast<const uint8_t*>(data);
        for (size_t i = 0; i < len; ++i) {
            state ^= bytes[i];
            state *= 0x100000001b3ull;
        }
    }
    
    void add_float(float f) {
        uint32_t bits;
        std::memcpy(&bits, &f, sizeof(bits));
        add_bytes(&bits, sizeof(bits));
    }
    
    void add_uint32(uint32_t val) {
        add_bytes(&val, sizeof(val));
    }
    
    void add_vec3(const Vec3& v) {
        add_float(v.x);
        add_float(v.y);
        add_float(v.z);
    }
    
    void add_quat(const Quat& q) {
        add_float(q.x);
        add_float(q.y);
        add_float(q.z);
        add_float(q.w);
    }
    
    uint64_t finalize() const { return state; }
};

// Simulate a simple projectile with spin, air resistance, and ground bounce
struct TrajectorySim {
    Vec3 position;
    Vec3 velocity;
    Quat orientation;
    Vec3 angular_velocity;
    
    float mass;
    float drag_coeff;
    float restitution;
    float angular_damping;
    
    static constexpr float DT = 1.0f / 240.0f;
    static constexpr float GROUND_Y = 0.0f;
    
    void step(HashAccumulator& hash) {
        velocity.y -= 9.81f * DT;
        
        float speed_sq = Vec3::length_sq(velocity);
        if (speed_sq > 0.0001f) {
            float speed = std::sqrt(speed_sq);
            float drag_force = drag_coeff * speed_sq;
            float drag_decel = drag_force / mass * DT;
            if (drag_decel < speed) {
                Vec3 drag_dir = Vec3::scale(velocity, 1.0f / speed);
                velocity = Vec3::sub(velocity, Vec3::scale(drag_dir, drag_decel));
            } else {
                velocity = Vec3(0.0f, 0.0f, 0.0f);
            }
        }
        
        position = Vec3::add(position, Vec3::scale(velocity, DT));
        
        float ang_speed = Vec3::length(angular_velocity);
        if (ang_speed > 0.0001f) {
            Vec3 axis = Vec3::scale(angular_velocity, 1.0f / ang_speed);
            float half_angle = ang_speed * DT * 0.5f;
            Quat delta = Quat::from_axis_angle(axis, half_angle);
            orientation = Quat::multiply(delta, orientation);
            orientation = Quat::normalize(orientation);
        }
        
        angular_velocity = Vec3::scale(angular_velocity, 1.0f - angular_damping * DT);
        
        if (position.y < GROUND_Y) {
            position.y = GROUND_Y;
            
            if (velocity.y < 0.0f) {
                velocity.y = -velocity.y * restitution;
                velocity.x *= 0.95f;
                velocity.z *= 0.95f;
                
                Vec3 spin_axis = Vec3::cross(angular_velocity, Vec3(0.0f, 1.0f, 0.0f));
                velocity = Vec3::scaled_add(velocity, spin_axis, 0.1f * DT);
            }
            
            angular_velocity = Vec3::scale(angular_velocity, 0.98f);
        }
        
        hash.add_vec3(position);
        hash.add_vec3(velocity);
        hash.add_quat(orientation);
        hash.add_vec3(angular_velocity);
    }
};

} // namespace apc::test

int main(int argc, char* argv[]) {
    using namespace apc;
    using namespace apc::test;
    
    FPUState fp_state = enforce_deterministic_fp_mode();
    if (!fp_state.is_valid) {
        std::fprintf(stderr, "FATAL: Could not enforce deterministic FP mode\n");
        return 1;
    }
    std::fprintf(stdout, "FP mode enforced successfully\n");
    
    const int NUM_STEPS = 100000;
    const int HASH_INTERVAL = 100;

    TrajectorySim sim;
    sim.position = Vec3(0.0f, 10.0f, 0.0f);
    sim.velocity = Vec3(15.0f, 20.0f, 5.0f);
    sim.orientation = Quat::identity();
    sim.angular_velocity = Vec3(50.0f, 10.0f, -30.0f);
    sim.mass = 0.45f;
    sim.drag_coeff = 0.001f;
    sim.restitution = 0.6f;
    sim.angular_damping = 0.5f;
    
    HashAccumulator final_hash;
    
    final_hash.add_vec3(sim.position);
    final_hash.add_vec3(sim.velocity);
    final_hash.add_quat(sim.orientation);
    final_hash.add_vec3(sim.angular_velocity);
    final_hash.add_float(sim.mass);
    final_hash.add_float(sim.drag_coeff);
    final_hash.add_float(sim.restitution);
    final_hash.add_float(sim.angular_damping);
    
    std::fprintf(stdout, "Running %d simulation steps...\n", NUM_STEPS);
    
    for (int i = 0; i < NUM_STEPS; ++i) {
#ifdef APC_VERIFY_FP_MODE_EVERY_FRAME
        if ((i % 1000) == 0) {
            if (!verify_fp_mode(fp_state)) {
                std::fprintf(stderr, "FATAL: FP mode corrupted at step %d\n", i);
                return 2;
            }
        }
#endif
        
        sim.step(final_hash);
        
        if ((i % HASH_INTERVAL) == 0) {
            HashAccumulator checkpoint;
            checkpoint.add_uint32(i);
            checkpoint.add_vec3(sim.position);
            std::fprintf(stdout, "  [%6d] pos=(%.6f, %.6f, %.6f) hash=%016llx\n",
                        i, sim.position.x, sim.position.y, sim.position.z,
                        (unsigned long long)checkpoint.finalize());
        }
    }
    
    uint64_t result = final_hash.finalize();
    std::fprintf(stdout, "\n=== DETERMINISM RESULT ===\n");
    std::fprintf(stdout, "Final hash: %016llx\n", (unsigned long long)result);
    
#if defined(_MSC_VER)
    std::fprintf(stdout, "Platform: x64 (MSVC)\n");
#elif defined(__clang__)
    std::fprintf(stdout, "Platform: Clang\n");
#else
    std::fprintf(stdout, "Platform: GCC\n");
#endif
    
    std::fprintf(stdout, "==========================\n");
    
    return 0;
}