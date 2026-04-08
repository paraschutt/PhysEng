#pragma once
// =============================================================================
// Render Camera — Camera system for the APC render pipeline
// =============================================================================
//
// Provides:
//   - CameraMode: perspective / orthographic
//   - CameraProjection: projection parameters
//   - CameraTransform: eye/target/up with look-at matrix computation
//   - CameraController: unified camera with perspective/ortho support
//   - FollowCamera: smooth follow camera with look-ahead
//   - OrbitCamera: orbit/zoom/pan camera
//
// All 4x4 matrices use column-major float[16] convention:
//   m[col * 4 + row]
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation
//   - C++17
//
// =============================================================================

#include "apc_render_types.h"
#include "apc_math/apc_vec3.h"
#include "apc_math/apc_quat.h"
#include "apc_math/apc_mat3.h"
#include "apc_math/apc_math_common.h"
#include <cstdint>
#include <cmath>

namespace apc {

// ---------------------------------------------------------------------------
// CameraMode
// ---------------------------------------------------------------------------
enum class CameraMode : uint8_t {
    PERSPECTIVE   = 0,
    ORTHOGRAPHIC  = 1
};

// ---------------------------------------------------------------------------
// CameraProjection — Projection parameters
// ---------------------------------------------------------------------------
struct CameraProjection {
    float fov_y        = 1.0471975512f;  // ~60 degrees in radians
    float aspect_ratio = 16.0f / 9.0f;
    float near_plane   = 0.1f;
    float far_plane    = 1000.0f;
    float ortho_size   = 10.0f;           // Half-height in world units
};

// ---------------------------------------------------------------------------
// CameraTransform — Eye/target/up with look-at matrix
// ---------------------------------------------------------------------------
struct CameraTransform {
    Vec3 eye   = Vec3(0.0f, 0.0f, 0.0f);
    Vec3 target = Vec3(0.0f, 0.0f, -1.0f);
    Vec3 up    = Vec3(0.0f, 1.0f, 0.0f);

    // Build a 4x4 look-at matrix (column-major).
    // Convention: forward = normalize(target - eye), places eye at origin
    // looking along +Z (OpenGL style, but with -Z forward in view space).
    void compute_view_matrix(float out[16]) const {
        Vec3 forward = Vec3::normalize(Vec3::sub(target, eye));
        Vec3 right   = Vec3::normalize(Vec3::cross(forward, up));
        Vec3 corrected_up = Vec3::cross(right, forward);

        // Column 0: right
        out[0]  = right.x;
        out[1]  = corrected_up.x;
        out[2]  = -forward.x;
        out[3]  = 0.0f;

        // Column 1: up
        out[4]  = right.y;
        out[5]  = corrected_up.y;
        out[6]  = -forward.y;
        out[7]  = 0.0f;

        // Column 2: -forward (looking down -Z in view space)
        out[8]  = right.z;
        out[9]  = corrected_up.z;
        out[10] = -forward.z;
        out[11] = 0.0f;

        // Column 3: translation
        out[12] = -Vec3::dot(right, eye);
        out[13] = -Vec3::dot(corrected_up, eye);
        out[14] = Vec3::dot(forward, eye);
        out[15] = 1.0f;
    }
};

// ---------------------------------------------------------------------------
// mat4_multiply — Column-major 4x4 matrix multiply: out = a * b
// ---------------------------------------------------------------------------
namespace detail {

inline void mat4_multiply(const float a[16], const float b[16], float out[16]) {
    for (int col = 0; col < 4; ++col) {
        for (int row = 0; row < 4; ++row) {
            float sum = 0.0f;
            for (int k = 0; k < 4; ++k) {
                sum += a[k * 4 + row] * b[col * 4 + k];
            }
            out[col * 4 + row] = sum;
        }
    }
}

inline void mat4_identity(float out[16]) {
    for (int i = 0; i < 16; ++i) out[i] = 0.0f;
    out[0] = 1.0f; out[5] = 1.0f; out[10] = 1.0f; out[15] = 1.0f;
}

} // namespace detail

// ---------------------------------------------------------------------------
// CameraController — Unified camera with perspective/ortho support
// ---------------------------------------------------------------------------
struct CameraController {
    CameraMode mode = CameraMode::PERSPECTIVE;
    CameraProjection projection;
    CameraTransform transform;

    // Cached matrices
    float cached_view[16]       = {};
    float cached_projection[16] = {};
    float cached_vp[16]         = {};
    bool matrices_dirty         = true;

    void set_perspective(float fov_y, float aspect, float near_p, float far_p) {
        mode = CameraMode::PERSPECTIVE;
        projection.fov_y        = fov_y;
        projection.aspect_ratio = aspect;
        projection.near_plane   = near_p;
        projection.far_plane    = far_p;
        matrices_dirty = true;
    }

    void set_orthographic(float size, float aspect, float near_p, float far_p) {
        mode = CameraMode::ORTHOGRAPHIC;
        projection.ortho_size   = size;
        projection.aspect_ratio = aspect;
        projection.near_plane   = near_p;
        projection.far_plane    = far_p;
        matrices_dirty = true;
    }

    void look_at(const Vec3& eye_, const Vec3& target_, const Vec3& up_) {
        transform.eye    = eye_;
        transform.target = target_;
        transform.up     = up_;
        matrices_dirty = true;
    }

    const float* get_view_matrix() {
        if (matrices_dirty) update_matrices();
        return cached_view;
    }

    const float* get_projection_matrix() {
        if (matrices_dirty) update_matrices();
        return cached_projection;
    }

    const float* get_view_projection() {
        if (matrices_dirty) update_matrices();
        return cached_vp;
    }

private:
    void update_matrices() {
        // View matrix
        transform.compute_view_matrix(cached_view);

        // Projection matrix
        if (mode == CameraMode::PERSPECTIVE) {
            float f = 1.0f / std::tan(projection.fov_y * 0.5f);
            float nf = 1.0f / (projection.near_plane - projection.far_plane);

            cached_projection[0]  = f / projection.aspect_ratio;
            cached_projection[1]  = 0.0f;
            cached_projection[2]  = 0.0f;
            cached_projection[3]  = 0.0f;

            cached_projection[4]  = 0.0f;
            cached_projection[5]  = f;
            cached_projection[6]  = 0.0f;
            cached_projection[7]  = 0.0f;

            cached_projection[8]  = 0.0f;
            cached_projection[9]  = 0.0f;
            cached_projection[10] = (projection.far_plane + projection.near_plane) * nf;
            cached_projection[11] = -1.0f;

            cached_projection[12] = 0.0f;
            cached_projection[13] = 0.0f;
            cached_projection[14] = 2.0f * projection.far_plane * projection.near_plane * nf;
            cached_projection[15] = 0.0f;
        } else {
            // Orthographic
            float half_w = projection.ortho_size * projection.aspect_ratio;
            float half_h = projection.ortho_size;
            float rl = 1.0f / (half_w + half_w); // 1 / (right - left) = 1 / (2*half_w)
            float tb = 1.0f / (half_h + half_h); // 1 / (top - bottom) = 1 / (2*half_h)
            float fn = 1.0f / (projection.near_plane - projection.far_plane);

            cached_projection[0]  = 2.0f * rl;
            cached_projection[1]  = 0.0f;
            cached_projection[2]  = 0.0f;
            cached_projection[3]  = 0.0f;

            cached_projection[4]  = 0.0f;
            cached_projection[5]  = 2.0f * tb;
            cached_projection[6]  = 0.0f;
            cached_projection[7]  = 0.0f;

            cached_projection[8]  = 0.0f;
            cached_projection[9]  = 0.0f;
            cached_projection[10] = (projection.far_plane + projection.near_plane) * fn;
            cached_projection[11] = 0.0f;

            cached_projection[12] = 0.0f;
            cached_projection[13] = 0.0f;
            cached_projection[14] = 2.0f * projection.far_plane * projection.near_plane * fn;
            cached_projection[15] = 1.0f;
        }

        // VP = View * Projection
        detail::mat4_multiply(cached_view, cached_projection, cached_vp);

        matrices_dirty = false;
    }
};

// ---------------------------------------------------------------------------
// FollowCameraConfig — Configuration for follow camera
// ---------------------------------------------------------------------------
struct FollowCameraConfig {
    Vec3  offset             = Vec3(0.0f, 5.0f, -10.0f);
    float look_ahead_distance = 2.0f;
    float smooth_factor       = 0.1f;    // 0 = no smoothing, 1 = instant
    float min_distance        = 2.0f;
    float max_distance        = 50.0f;
    float min_height          = 1.0f;
    float max_height          = 100.0f;
};

// ---------------------------------------------------------------------------
// FollowCamera — Smooth follow camera with look-ahead
// ---------------------------------------------------------------------------
struct FollowCamera {
    FollowCameraConfig config;
    Vec3 current_position = Vec3(0.0f, 5.0f, -10.0f);
    Vec3 current_target   = Vec3(0.0f, 0.0f, 0.0f);

    void update(const Vec3& target_pos, const Vec3& target_vel, float dt) {
        // Look-ahead: predict where target will be
        Vec3 predicted = Vec3::add(target_pos,
            Vec3::scale(target_vel, config.look_ahead_distance));

        // Desired camera position: target + offset
        Vec3 desired_pos = Vec3::add(predicted, config.offset);

        // Smooth interpolation
        float t = 1.0f - std::exp(-config.smooth_factor * 10.0f * dt);
        current_position = Vec3::lerp(current_position, desired_pos, t);

        // Clamp distance
        Vec3 to_target = Vec3::sub(target_pos, current_position);
        float dist = Vec3::length(to_target);
        if (dist < config.min_distance && dist > APC_EPSILON) {
            current_position = Vec3::sub(target_pos,
                Vec3::scale(Vec3::normalize(to_target), config.min_distance));
        } else if (dist > config.max_distance && dist > APC_EPSILON) {
            current_position = Vec3::sub(target_pos,
                Vec3::scale(Vec3::normalize(to_target), config.max_distance));
        }

        // Clamp height
        if (current_position.y < config.min_height) {
            current_position.y = config.min_height;
        }
        if (current_position.y > config.max_height) {
            current_position.y = config.max_height;
        }

        // Smooth target tracking
        current_target = Vec3::lerp(current_target, target_pos, t);
    }

    Vec3 get_position() const { return current_position; }
    Vec3 get_look_target() const { return current_target; }
};

// ---------------------------------------------------------------------------
// OrbitCameraConfig — Configuration for orbit camera
// ---------------------------------------------------------------------------
struct OrbitCameraConfig {
    Vec3  orbit_center   = Vec3(0.0f, 0.0f, 0.0f);
    float distance       = 10.0f;
    float yaw            = 0.0f;
    float pitch          = 0.5f;
    float min_pitch      = -APC_HALF_PI + 0.01f;
    float max_pitch      = APC_HALF_PI - 0.01f;
    float min_distance   = 1.0f;
    float max_distance   = 100.0f;
    float zoom_speed     = 2.0f;
    float rotate_speed   = 0.005f;
};

// ---------------------------------------------------------------------------
// OrbitCamera — Orbit/zoom/pan camera
// ---------------------------------------------------------------------------
struct OrbitCamera {
    OrbitCameraConfig config;

    void rotate(float delta_yaw, float delta_pitch) {
        config.yaw   += delta_yaw * config.rotate_speed;
        config.pitch += delta_pitch * config.rotate_speed;

        // Clamp pitch
        if (config.pitch < config.min_pitch) config.pitch = config.min_pitch;
        if (config.pitch > config.max_pitch) config.pitch = config.max_pitch;
    }

    void zoom(float delta) {
        config.distance += delta * config.zoom_speed;
        if (config.distance < config.min_distance) config.distance = config.min_distance;
        if (config.distance > config.max_distance) config.distance = config.max_distance;
    }

    void pan(float dx, float dy) {
        // Pan in the camera's local XZ plane
        float cos_y = std::cos(config.yaw);
        float sin_y = std::sin(config.yaw);
        config.orbit_center.x += (-sin_y * dx + cos_y * dy) * config.distance * 0.01f;
        config.orbit_center.z += ( cos_y * dx + sin_y * dy) * config.distance * 0.01f;
    }

    CameraTransform update() const {
        CameraTransform result;

        // Compute eye position from spherical coordinates
        float cos_pitch = std::cos(config.pitch);
        float sin_pitch = std::sin(config.pitch);
        float cos_yaw   = std::cos(config.yaw);
        float sin_yaw   = std::sin(config.yaw);

        // Direction from center to eye
        float dx = config.distance * cos_pitch * sin_yaw;
        float dy = config.distance * sin_pitch;
        float dz = config.distance * cos_pitch * cos_yaw;

        result.eye    = Vec3::add(config.orbit_center, Vec3(dx, dy, dz));
        result.target = config.orbit_center;
        result.up     = Vec3(0.0f, 1.0f, 0.0f);

        return result;
    }

    Vec3 get_position() const {
        float cos_pitch = std::cos(config.pitch);
        float sin_pitch = std::sin(config.pitch);
        float cos_yaw   = std::cos(config.yaw);
        float sin_yaw   = std::sin(config.yaw);

        float dx = config.distance * cos_pitch * sin_yaw;
        float dy = config.distance * sin_pitch;
        float dz = config.distance * cos_pitch * cos_yaw;

        return Vec3::add(config.orbit_center, Vec3(dx, dy, dz));
    }

    Vec3 get_target() const { return config.orbit_center; }
};

} // namespace apc
