# 3D Render Pipeline — Adaptive Physics Core (APC)

> **Complementary rendering system** for the APC physics engine. Designed for tight integration with skeletal dynamics, deterministic replay, and "Impact League" visualization requirements.
> 
> **Key Principle:** The render pipeline is *driven by* physics state but is not required to be bit-exact deterministic. Visual interpolation, LOD, and effects can vary per-platform as long as they accurately represent the underlying physics simulation.

---

## Table of Contents

1. [Architecture Overview](#1-architecture-overview)
2. [Physics-to-Render State Transfer](#2-physics-to-render-state-transfer)
3. [Skeletal Animation Blending](#3-skeletal-animation-blending)
4. [Character Rendering Pipeline](#4-character-rendering-pipeline)
5. [Contact Visualization](#5-contact-visualization)
6. [Impact Effect System](#6-impact-effect-system)
7. [Camera & Time Manipulation](#7-camera--time-manipulation)
8. [Performance Budget](#8-performance-budget)
9. [Implementation Phases](#9-implementation-phases)

---

## 1. Architecture Overview

### 1.1 Design Philosophy

The APC render pipeline serves three primary goals:

1. **Faithful Physics Visualization** — Render exactly what the physics simulation produces, with smooth interpolation between 240Hz physics ticks at 60/120Hz render rates.
2. **Game Feel Amplification** — Enhance physical impacts with visual feedback (camera shake, hit-stop, motion blur, particles) without altering simulation state.
3. **Debug & Authoring Support** — Provide wireframe, contact manifold, and constraint visualization for tuning physics behavior.

### 1.2 System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    GAME / VISUAL SCRIPTING                       │
│  (Animation graphs, VFX triggers, camera directors, UI overlay) │
└─────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                    RENDER PIPELINE CORE                          │
│  Interpolation, skinning, LOD, material binding, draw dispatch  │
└─────────────────────────────────────────────────────────────────┘
                                │
                ┌───────────────┼───────────────┐
                ▼               ▼               ▼
    ┌────────────────┐ ┌────────────────┐ ┌────────────────┐
    │  CHARACTER     │ │  ENVIRONMENT   │ │  VFX / IMPACT  │
    │  RENDERER      │ │  RENDERER      │ │  RENDERER      │
    │  Skinning      │ │  Static/Dynamic│ │  Particles     │
    │  Blend poses   │ │  meshes        │ │  Decals        │
    │  LOD selection │ │  Terrain       │ │  Hit-stop      │
    └────────────────┘ └────────────────┘ └────────────────┘
                │               │               │
                └───────────────┼───────────────┘
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                    PHYSICS STATE INTERFACE                      │
│  Snapshot access, interpolation, contact query, bone transforms │
└─────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                    APC SIMULATION CORE                           │
│  240Hz fixed timestep, skeletal dynamics, collision, solver     │
└─────────────────────────────────────────────────────────────────┘
```

### 1.3 Threading Model

The render pipeline runs on a separate thread from the physics simulation, with double-buffered state transfer:

```
Frame N Timeline (16.67ms for 60Hz render):

Physics Thread (240Hz, 4.17ms per substep):
├─ Substep 1 (0.00 - 4.17ms)
├─ Substep 2 (4.17 - 8.33ms)
├─ Substep 3 (8.33 - 12.50ms)
└─ Substep 4 (12.50 - 16.67ms) → Write to Back Buffer

Render Thread (60Hz or 120Hz):
├─ Read Front Buffer (physics state at frame N-1)
├─ Interpolate to target time (frame N + render_lag)
├─ Skinning, culling, draw calls
└─ Present

Swap Buffers at VSync
```

**Key Concept:** The render thread always reads a *consistent snapshot* of physics state. The physics thread writes to the back buffer only after completing all 4 substeps for the frame. This prevents tearing artifacts where some characters are rendered at different simulation times.

---

## 2. Physics-to-Render State Transfer

### 2.1 State Snapshot Structure

```cpp
struct PhysicsSnapshot {
    uint64_t frame_number;          // Monotonic frame counter
    float    sim_time;              // Simulation time in seconds
    
    // Rigid body states (for props, ball, environment pieces)
    struct BodyState {
        uint32_t entity_id;
        Vec3     position;
        Quat     orientation;
        Vec3     linear_velocity;   // For interpolation
        Vec3     angular_velocity;
    };
    FlatMap<uint32_t, BodyState> bodies;  // Sorted by entity_id
    
    // Skeletal states (for characters)
    struct SkeletalState {
        uint32_t character_id;
        uint32_t skeleton_asset_id;
        Vec3     root_position;
        Quat     root_orientation;
        Vec3     joint_rotations[MAX_JOINTS];  // Quaternion as Vec4 (xyz, w)
        Vec3     joint_positions[MAX_JOINTS];  // Local-space translations
        uint8_t  bone_blend_modes[MAX_JOINTS]; // ANIM/PHYSICS/BLENDED
        float    bone_physics_weights[MAX_JOINTS];
    };
    FlatMap<uint32_t, SkeletalState> skeletons;
    
    // Contact manifolds (for debug visualization, impact VFX)
    struct ContactState {
        uint32_t body_a, body_b;
        Vec3     contact_point;
        Vec3     contact_normal;
        float    penetration_depth;
        float    impulse_magnitude;  // From solver
    };
    std::vector<ContactState> contacts;  // Sorted by (body_a, body_b)
};
```

### 2.2 Double-Buffered Swap

```cpp
class PhysicsRenderInterface {
    PhysicsSnapshot buffers[2];
    uint32_t write_buffer = 0;
    uint32_t read_buffer = 0;
    std::atomic<bool> swap_pending = false;
    
public:
    // Called by physics thread after completing all substeps
    void commit_snapshot() {
        buffers[write_buffer].frame_number++;
        swap_pending.store(true, std::memory_order_release);
    }
    
    // Called by render thread at start of frame
    bool acquire_snapshot() {
        if (swap_pending.load(std::memory_order_acquire)) {
            write_buffer = 1 - write_buffer;
            read_buffer = 1 - read_buffer;
            swap_pending.store(false, std::memory_order_release);
            return true;  // New snapshot available
        }
        return false;  // Still rendering with previous frame's state
    }
    
    const PhysicsSnapshot& get_snapshot() const {
        return buffers[read_buffer];
    }
};
```

### 2.3 Temporal Interpolation

Since physics runs at 240Hz and rendering typically runs at 60Hz or 120Hz, the render pipeline interpolates between snapshots:

```cpp
struct InterpolatedBodyState {
    Vec3 position;
    Quat orientation;
};

InterpolatedBodyState interpolate(
    const PhysicsSnapshot& prev,
    const PhysicsSnapshot& curr,
    float alpha  // 0.0 = prev, 1.0 = curr
) {
    const auto& prev_body = prev.bodies.get(entity_id);
    const auto& curr_body = curr.bodies.get(entity_id);
    
    InterpolatedBodyState result;
    result.position = lerp(prev_body.position, curr_body.position, alpha);
    result.orientation = slerp(prev_body.orientation, curr_body.orientation, alpha);
    
    return result;
}
```

**Typical alpha calculation for 60Hz render with 240Hz physics:**

```cpp
// If render thread is 1 frame behind physics:
float alpha = render_lag / physics_dt;  // Usually 0.0 to 1.0
```

---

## 3. Skeletal Animation Blending

### 3.1 Pose Composition Pipeline

APC's skeletal dynamics system produces per-bone blend weights (`bone_physics_weights`). The render pipeline composes the final pose:

```
┌─────────────────────┐
│  Animation Pose     │ ← From animation graph (game-driven)
│  (target pose)      │
└─────────┬───────────┘
          │
          ▼
┌─────────────────────┐
│  Physics Pose       │ ← From skeletal dynamics (FK from physics joints)
│  (simulated pose)   │
└─────────┬───────────┘
          │
          ▼
┌─────────────────────┐
│  Per-Bone Lerp      │ ← Using bone_physics_weights
│  Final_Pose[i] =    │
│    lerp(anim[i],    │
│         phys[i],    │
│         weight[i])  │
└─────────┬───────────┘
          │
          ▼
┌─────────────────────┐
│  Skin Matrices      │ ← Compute skinning matrices for GPU
│  (bind pose⁻¹ ×     │
│   final_pose)       │
└─────────┬───────────┘
          │
          ▼
┌─────────────────────┐
│  GPU Skinning       │ ← Vertex shader transformation
└─────────────────────┘
```

### 3.2 Bone Blend Modes

Each bone has a blend mode (from `apc_skeleton_types.h`):

```cpp
enum class BoneBlendMode : uint8_t {
    ANIM_DRIVEN = 0,      // 100% animation, 0% physics (e.g., head, spine)
    PHYSICS_DRIVEN = 1,   // 100% physics, 0% animation (e.g., knocked-down limb)
    BLENDED = 2           // Smooth blend based on physics_weight
};

struct BoneFinalPose {
    Vec3 position;   // Local to parent
    Quat rotation;
};

BoneFinalPose compute_bone_pose(
    const BonePhysicsState& phys_state,
    const BoneFinalPose& anim_pose,
    const BoneFinalPose& phys_pose
) {
    switch (phys_state.blend_mode) {
        case BoneBlendMode::ANIM_DRIVEN:
            return anim_pose;
        
        case BoneBlendMode::PHYSICS_DRIVEN:
            return phys_pose;
        
        case BoneBlendMode::BLENDED: {
            BoneFinalPose result;
            result.position = lerp(anim_pose.position, phys_pose.position, 
                                   phys_state.physics_weight);
            result.rotation = slerp(anim_pose.rotation, phys_pose.rotation,
                                    phys_state.physics_weight);
            return result;
        }
    }
}
```

### 3.3 Spring-Back Visualization

When a bone is physics-driven due to an impact, it springs back toward the animation pose over time. The render pipeline can add secondary motion cues:

```cpp
// In bone update loop:
float spring_visualization_factor = 1.0f - phys_state.physics_weight;

// Exaggerate spring-back slightly for visual clarity (optional)
if (spring_visualization_factor < 0.3f && phys_state.velocity_magnitude > threshold) {
    // Add subtle motion blur or stretch along velocity direction
    bone_stretch_scale = 1.0f + phys_state.velocity_magnitude * 0.05f;
}
```

---

## 4. Character Rendering Pipeline

### 4.1 LOD Strategy

APC characters have 60-80 convex collision pieces. The visual mesh LOD strategy mirrors the collision LOD:

| Distance | Visual LOD | Collision LOD | Bone Count | Target Triangles |
|----------|-----------|---------------|------------|------------------|
| 0-5m     | LOD0      | Full (60-80)  | 80 bones   | 50,000+          |
| 5-15m    | LOD1      | Simplified (30-40) | 50 bones   | 20,000           |
| 15-30m   | LOD2      | Capsule-only  | 20 bones   | 5,000            |
| 30m+     | LOD3      | Billboard     | 0 (no sim) | 2 tris           |

LOD selection uses both distance and velocity heuristics:

```cpp
int select_character_lod(float distance, float velocity_magnitude) {
    int lod = 0;
    
    if (distance > 30.0f) lod = 3;
    else if (distance > 15.0f) lod = 2;
    else if (distance > 5.0f) lod = 1;
    
    // High-velocity characters stay at higher LOD longer
    if (velocity_magnitude > 10.0f && lod > 0) {
        lod--;  // One LOD level higher than distance alone would dictate
    }
    
    return lod;
}
```

### 4.2 Skinning Implementation

**GPU Skinning (Preferred):** Upload bone matrices as uniform buffer or texture buffer.

```glsl
// Vertex shader (GLSL)
layout(std430, binding = 0) readonly buffer BoneMatrices {
    mat4 bone_matrices[MAX_BONES];
};

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
layout(location = 2) in uvec4 bone_indices;
layout(location = 3) in vec4 bone_weights;

void main() {
    mat4 skin_matrix = 
        bone_matrices[bone_indices.x] * bone_weights.x +
        bone_matrices[bone_indices.y] * bone_weights.y +
        bone_matrices[bone_indices.z] * bone_weights.z +
        bone_matrices[bone_indices.w] * bone_weights.w;
    
    vec4 skinned_pos = skin_matrix * vec4(position, 1.0);
    gl_Position = projection_matrix * view_matrix * skinned_pos;
}
```

**CPU Skinning (Fallback):** For platforms without compute shaders or when bone count exceeds GPU limits.

### 4.3 Uniform Material System

All characters use a unified material system that supports:

- **PBR Base:** Albedo, metallic, roughness, normal maps
- **Damage Overlays:** Dirt, grass stains, sweat (projected decals or vertex colors)
- **Impact Highlights:** Flash white on high-impulse contacts (see Section 6)

```cpp
struct CharacterMaterial {
    Texture2D albedo_map;
    Texture2D normal_map;
    Texture2D roughness_map;
    Texture2D metallic_map;
    
    // Dynamic overlays
    Texture2D damage_overlay;    // RGBA: dirt RGB + intensity A
    float sweat_factor;          // 0.0 - 1.0, increases with exertion
    float impact_flash;          // 0.0 - 1.0, decays over ~200ms
    Vec3 impact_flash_color;     // Typically white or team color
    
    // Uniform parameters
    float roughness_override = -1.0f;  // < 0 = use texture
    float metallic_override = -1.0f;
};
```

---

## 5. Contact Visualization

### 5.1 Debug Rendering Modes

For physics tuning and bug investigation, APC provides several contact visualization modes:

```cpp
enum class ContactDebugMode {
    NONE,           // No contact visualization
    POINTS_ONLY,    // Render contact points as spheres
    NORMALS,        // Points + normal vectors
    MANIFOLDS,      // Full contact manifolds (convex hulls)
    IMPULSE_HEATMAP // Color-code by impulse magnitude
};

void render_contact_debug(const PhysicsSnapshot& snapshot, ContactDebugMode mode) {
    for (const auto& contact : snapshot.contacts) {
        Vec3 color;
        
        if (mode == ContactDebugMode::IMPULSE_HEATMAP) {
            // Green = low impulse, Red = high impulse
            float t = clamp(contact.impulse_magnitude / max_impulse, 0.0f, 1.0f);
            color = lerp(Vec3(0, 1, 0), Vec3(1, 0, 0), t);
        } else {
            color = Vec3(1, 1, 0);  // Yellow for points
        }
        
        // Draw contact point sphere
        debug_draw_sphere(contact.contact_point, 0.02f, color);
        
        if (mode == ContactDebugMode::NORMALS || 
            mode == ContactDebugMode::MANIFOLDS) {
            // Draw normal vector
            Vec3 normal_end = contact.contact_point + contact.contact_normal * 0.1f;
            debug_draw_line(contact.contact_point, normal_end, Vec3(0, 1, 0));
        }
        
        if (mode == ContactDebugMode::MANIFOLDS) {
            // Draw penetration depth indicator
            Vec3 penetration_vec = -contact.contact_normal * contact.penetration_depth;
            debug_draw_arrow(contact.contact_point, penetration_vec, Vec3(1, 0, 0));
        }
    }
}
```

### 5.2 Persistent Contact Trails

To visualize contact persistence over time (useful for tuning warmstarting):

```cpp
struct ContactTrail {
    Vec3 positions[8];
    float intensities[8];
    uint8_t head_index;
    uint8_t count;
};

void update_contact_trail(uint32_t pair_key, const ContactPoint& new_contact) {
    auto& trail = contact_trails[pair_key];
    trail.positions[trail.head_index] = new_contact.point_on_a;
    trail.intensities[trail.head_index] = new_contact.penetration;
    trail.head_index = (trail.head_index + 1) % 8;
    trail.count = min(trail.count + 1, 8);
}

void render_contact_trails() {
    for (const auto& trail : contact_trails) {
        for (uint8_t i = 0; i < trail.count; ++i) {
            float alpha = trail.intensities[i] * (i / 8.0f);  // Fade older points
            debug_draw_sphere(trail.positions[i], 0.015f, 
                              Vec3(1, alpha, 0));
        }
    }
}
```

---

## 6. Impact Effect System

### 6.1 ImpactStyleProfile Integration

APC's `ImpactStyleProfile` (from ARCHITECTURE.md) defines game-feel parameters. The render pipeline consumes these for visual feedback:

```cpp
struct ImpactVFXTrigger {
    uint32_t body_a, body_b;
    Vec3     contact_point;
    Vec3     contact_normal;
    float    impulse_magnitude;
    float    relative_velocity;
    
    // From ImpactStyleProfile
    float    hit_stop_duration_ms;
    float    camera_shake_intensity;
    float    time_scale_on_impact;
    
    // VFX parameters
    uint32_t spark_particle_id;
    uint32_t dust_particle_id;
    float    decal_size;
};

void trigger_impact_vfx(const ImpactVFXTrigger& trigger) {
    // 1. Spawn particle systems
    if (trigger.impulse_magnitude > 5.0f) {
        spawn_particles(trigger.spark_particle_id, 
                        trigger.contact_point,
                        trigger.contact_normal);
    }
    
    // 2. Project impact decal
    if (trigger.impulse_magnitude > 3.0f) {
        spawn_decal("impact_scuff", 
                    trigger.contact_point,
                    trigger.contact_normal,
                    trigger.decal_size);
    }
    
    // 3. Request hit-stop (pauses render, not physics)
    if (trigger.hit_stop_duration_ms > 0.0f) {
        request_hit_stop(trigger.hit_stop_duration_ms);
    }
    
    // 4. Request camera shake
    if (trigger.camera_shake_intensity > 0.0f) {
        request_camera_shake(trigger.camera_shake_intensity,
                             decay_rate = 0.95f,
                             duration = 300ms);
    }
    
    // 5. Set character material flash
    set_character_impact_flash(trigger.body_a, 
                               trigger.impulse_magnitude);
}
```

### 6.2 Hit-Stop Implementation

Hit-stop briefly pauses the render loop while physics continues simulating. This creates the illusion of a more impactful collision:

```cpp
class HitStopManager {
    float accumulated_hit_stop_time = 0.0f;
    float remaining_hit_stop = 0.0f;
    
public:
    void request_hit_stop(float duration_ms) {
        accumulated_hit_stop_time += duration_ms;
    }
    
    bool should_apply_hit_stop() const {
        return remaining_hit_stop > 0.0f;
    }
    
    float consume_hit_stop(float frame_time_ms) {
        if (remaining_hit_stop <= 0.0f && accumulated_hit_stop_time > 0.0f) {
            remaining_hit_stop = accumulated_hit_stop_time;
            accumulated_hit_stop_time = 0.0f;
        }
        
        if (remaining_hit_stop > 0.0f) {
            float consumed = min(remaining_hit_stop, frame_time_ms);
            remaining_hit_stop -= consumed;
            return consumed;  // Don't advance render time by this amount
        }
        return 0.0f;
    }
};
```

**Render loop integration:**

```cpp
void render_frame(float delta_time_ms) {
    float hit_stop_consumed = hit_stop_manager.consume_hit_stop(delta_time_ms);
    float render_advance_time = delta_time_ms - hit_stop_consumed;
    
    // Only interpolate physics forward by render_advance_time
    // Physics simulation itself continues at full 240Hz
    interpolate_and_render(render_advance_time);
}
```

### 6.3 Camera Shake

```cpp
struct CameraShakeState {
    Vec3 position_offset;
    Vec3 rotation_offset;  // Euler angles
    float intensity;
    float decay_rate;
    float duration_remaining;
    uint32_t noise_seed;
};

CameraShakeState update_camera_shake(float dt) {
    if (state.duration_remaining <= 0.0f) {
        return CameraShakeState{};  // No shake
    }
    
    state.duration_remaining -= dt;
    state.intensity *= pow(state.decay_rate, dt * 60.0f);  // Per-frame decay
    
    // Pseudo-random offset using prime multipliers (deterministic per seed)
    float t = state.duration_remaining;
    state.position_offset.x = sin(t * 37.0f + state.noise_seed) * state.intensity * 0.02f;
    state.position_offset.y = cos(t * 41.0f + state.noise_seed) * state.intensity * 0.02f;
    state.position_offset.z = sin(t * 43.0f + state.noise_seed) * state.intensity * 0.01f;
    
    state.rotation_offset.x = cos(t * 29.0f) * state.intensity * 1.5f;  // Degrees
    state.rotation_offset.y = sin(t * 31.0f) * state.intensity * 1.5f;
    state.rotation_offset.z = sin(t * 33.0f) * state.intensity * 0.5f;
    
    return state;
}
```

---

## 7. Camera & Time Manipulation

### 7.1 Time Scale on Impact

From `ImpactStyleProfile`, `time_scale_on_impact` allows brief slow-motion windows. Unlike hit-stop, this affects both render and physics perception:

```cpp
class TimeScaleManager {
    float current_time_scale = 1.0f;
    float target_time_scale = 1.0f;
    float ramp_duration = 0.0f;
    float ramp_elapsed = 0.0f;
    
public:
    void request_time_scale(float scale, float duration, float ramp = 0.1f) {
        target_time_scale = scale;
        ramp_duration = ramp;
        ramp_elapsed = 0.0f;
    }
    
    float get_time_scale() const {
        return current_time_scale;
    }
    
    void update(float dt) {
        if (ramp_elapsed < ramp_duration) {
            ramp_elapsed += dt;
            float t = ramp_elapsed / ramp_duration;
            current_time_scale = lerp(current_time_scale, target_time_scale, t);
        } else if (target_time_scale != 1.0f) {
            // Hold phase - maintain target until explicit reset
            current_time_scale = target_time_scale;
        } else {
            // Return to normal
            current_time_scale = 1.0f;
        }
    }
};
```

**Note:** Time scale is a *render-side effect only*. Physics continues at 240Hz. The time scale affects:
- Animation playback speed
- Particle simulation rate
- Camera movement speed
- Interpolation alpha calculation

### 7.2 Replay Camera System

For broadcast-style replays in "Impact League":

```cpp
struct ReplayCamera {
    enum class Mode {
        FOLLOW_PLAYER,
        FREE_LOOK,
        IMPACT_FOCUS,    // Auto-zoom on high-impulse collisions
        END_ZONE_OVERVIEW,
        SKY_CAM
    };
    
    Mode mode;
    uint32_t follow_target_id;
    Vec3 offset;
    float fov;
    float focus_distance;
    
    // Auto-director parameters
    float anticipation_time = 0.5f;  // Look ahead by this much
    float impact_focus_threshold = 15.0f;  // Impulse magnitude to trigger
};

ReplayCamera update_replay_camera(
    const ReplayCamera& cam,
    const PhysicsSnapshot& snapshot,
    float dt
) {
    ReplayCamera result = cam;
    
    if (cam.mode == Mode::IMPACT_FOCUS) {
        // Find highest-impulse contact in recent frames
        uint32_t best_contact_pair = 0;
        float best_impulse = 0.0f;
        
        for (const auto& contact : snapshot.contacts) {
            if (contact.impulse_magnitude > best_impulse) {
                best_impulse = contact.impulse_magnitude;
                best_contact_pair = contact.body_a;
            }
        }
        
        if (best_impulse > cam.impact_focus_threshold) {
            result.follow_target_id = best_contact_pair;
            result.focus_distance = lerp(result.focus_distance, 3.0f, dt * 2.0f);
        }
    }
    
    if (cam.mode == Mode::FOLLOW_PLAYER || cam.mode == Mode::IMPACT_FOCUS) {
        // Get target position (with anticipation)
        const auto& target = snapshot.bodies.get(cam.follow_target_id);
        Vec3 anticipated_pos = target.position + target.linear_velocity * cam.anticipation_time;
        
        // Smooth camera follow
        Vec3 desired_cam_pos = anticipated_pos + result.offset;
        // ... apply damping, collision avoidance, etc.
    }
    
    return result;
}
```

---

## 8. Performance Budget

### 8.1 Frame Budget Breakdown (60Hz Target)

| Stage | Budget | Notes |
|-------|--------|-------|
| Physics simulation (240Hz × 4 substeps) | 5.0ms | Separate thread |
| State snapshot + interpolation | 0.3ms | Memory bandwidth bound |
| Skeletal pose composition (22 chars × 80 bones) | 0.5ms | CPU skinning matrix computation |
| Culling (frustum + occlusion) | 0.5ms | Hierarchical Z-buffer |
| GPU skinning (22 chars × 20k avg tris) | 3.0ms | Vertex shader bound |
| Environment rendering | 3.0ms | Field, stadium, crowd |
| VFX (particles, decals) | 1.5ms | Fragment shader bound |
| Post-processing (tone map, bloom) | 1.0ms | Full-screen passes |
| **Total Render** | **~9.8ms** | Leaves 6.8ms headroom for 16.67ms frame |

### 8.2 Memory Budget (Per Frame)

| Allocation | Size | Lifetime |
|------------|------|----------|
| Physics snapshot (double-buffered) | 2MB × 2 | Persistent |
| Interpolated state cache | 512KB | Per-frame temp |
| Bone matrices (22 chars × 80 bones × 2 LODs) | 256KB | Per-frame temp |
| Particle vertex buffers | 128KB | Per-frame temp |
| Decal projections | 64KB | Per-frame temp |
| **Total Render RAM** | **~3MB** | Excluding GPU textures |

### 8.3 Draw Call Budget

| Category | Draw Calls | Instances |
|----------|------------|-----------|
| Characters (LOD0-2) | 44 | 22 chars × 2 materials avg |
| Character shadows | 44 | Cascade shadow maps |
| Environment static | 50-100 | Batched by material |
| Environment dynamic | 20 | Props, goalposts, etc. |
| Particles | 10-20 | Instanced quads |
| Decals | 20-30 | Projected meshes |
| Debug visualization | 10-20 | Wireframes, contacts |
| **Total** | **~200-250** | Well within D3D12/Vulkan limits |

---

## 9. Implementation Phases

### Phase R0: Foundation (Weeks 1-2)
**Deliverable:** Basic physics state snapshot + single character rendering

```
MILESTONES:
□ PhysicsRenderInterface with double-buffered snapshots
□ Basic interpolation (lerp, slerp) between physics states
□ Single character GPU skinning pipeline
□ Simple follow camera
□ Profile baseline performance
```

### Phase R1: Full Character Pipeline (Weeks 3-6)
**Deliverable:** 22 characters with LOD, proper blending, materials

```
MILESTONES:
□ Multi-character rendering (22 players)
□ LOD system (4 levels per character)
□ Per-bone animation/physics blending
□ Character material system (PBR + overlays)
□ Shadow cascade implementation
□ Performance optimization pass (meet 10ms budget)
```

### Phase R2: Contact & Impact VFX (Weeks 7-10)
**Deliverable:** Impact visualization, hit-stop, camera shake

```
MILESTONES:
□ Contact manifold debug visualization
□ ImpactVFXTrigger system
□ Hit-stop manager
□ Camera shake system
□ Particle system integration (sparks, dust, turf)
□ Decal projection system (scuffs, grass stains)
□ ImpactStyleProfile → VFX parameter mapping
```

### Phase R3: Time & Camera Tools (Weeks 11-14)
**Deliverable:** Replay system, time manipulation, broadcast cameras

```
MILESTONES:
□ TimeScaleManager for slow-motion impacts
□ ReplayCamera with multiple modes
□ Auto-director logic (impact focus, end zone overview)
□ Input recording + playback for replays
□ Camera transition smoothing
□ Broadcast-style FOV automation
```

### Phase R4: Polish & Optimization (Weeks 15-18)
**Deliverable:** Shippable render pipeline for "Impact League"

```
MILESTONES:
□ Motion blur (per-object + camera)
□ Depth of field (cinematic replays)
□ Crowd rendering (instanced, simplified)
□ Stadium environment (field, stands, lighting)
□ HDR + tone mapping
□ Console certification (memory, performance, stability)
□ Final optimization pass (hit 60fps on all platforms)
```

### Phase R5: Middleware Extraction (Weeks 19-24)
**Deliverable:** APC Render as optional middleware component

```
MILESTONES:
□ Clean API boundary (engine-agnostic)
□ Unity/Unreal integration plugins
□ Standalone renderer example
□ Documentation (integration guides, shader specs)
□ Sample projects (basic, skeletal, sport-specific)
□ Versioning and packaging
```

---

## Appendix A: Shader Specifications

### A.1 Character Vertex Shader (GLSL)

```glsl
#version 450

layout(std430, binding = 0) readonly buffer BoneMatrices {
    mat4 bone_matrices[MAX_BONES];
};

layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_normal;
layout(location = 2) in vec2 in_texcoord;
layout(location = 3) in uvec4 in_bone_indices;
layout(location = 4) in vec4 in_bone_weights;

layout(location = 0) out vec3 out_position;
layout(location = 1) out vec3 out_normal;
layout(location = 2) out vec2 out_texcoord;
layout(location = 3) out vec3 out_world_position;

uniform mat4 u_model_matrix;
uniform mat4 u_view_projection;

void main() {
    // Skinning
    mat4 skin_matrix = 
        bone_matrices[in_bone_indices.x] * in_bone_weights.x +
        bone_matrices[in_bone_indices.y] * in_bone_weights.y +
        bone_matrices[in_bone_indices.z] * in_bone_weights.z +
        bone_matrices[in_bone_indices.w] * in_bone_weights.w;
    
    vec4 skinned_pos = skin_matrix * vec4(in_position, 1.0);
    vec4 skinned_normal = skin_matrix * vec4(in_normal, 0.0);
    
    out_position = skinned_pos.xyz;
    out_normal = normalize(mat3(u_model_matrix) * skinned_normal.xyz);
    out_texcoord = in_texcoord;
    out_world_position = (u_model_matrix * skinned_pos).xyz;
    
    gl_Position = u_view_projection * u_model_matrix * skinned_pos;
}
```

### A.2 Character Fragment Shader (GLSL)

```glsl
#version 450

layout(binding = 0) uniform sampler2D u_albedo_map;
layout(binding = 1) uniform sampler2D u_normal_map;
layout(binding = 2) uniform sampler2D u_roughness_map;
layout(binding = 3) uniform sampler2D u_metallic_map;
layout(binding = 4) uniform sampler2D u_damage_overlay;

layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_normal;
layout(location = 2) in vec2 in_texcoord;
layout(location = 3) in vec3 in_world_position;

layout(location = 0) out vec4 out_color;

uniform vec3 u_view_position;
uniform vec3 u_light_direction;
uniform vec3 u_light_color;
uniform float u_impact_flash;
uniform vec3 u_impact_flash_color;

void main() {
    vec4 albedo = texture(u_albedo_map, in_texcoord);
    vec3 normal_ts = texture(u_normal_map, in_texcoord).rgb * 2.0 - 1.0;
    float roughness = texture(u_roughness_map, in_texcoord).r;
    float metallic = texture(u_metallic_map, in_texcoord).r;
    
    // Damage overlay blend
    vec4 damage = texture(u_damage_overlay, in_texcoord);
    albedo.rgb = mix(albedo.rgb, damage.rgb, damage.a * 0.5);
    
    // Impact flash
    albedo.rgb = mix(albedo.rgb, u_impact_flash_color, u_impact_flash);
    
    // Simple Lambertian + specular (replace with full PBR in production)
    vec3 world_normal = normalize(in_normal);
    vec3 view_dir = normalize(u_view_position - in_world_position);
    vec3 light_dir = normalize(-u_light_direction);
    
    vec3 diffuse = albedo.rgb * max(dot(world_normal, light_dir), 0.0);
    vec3 half_vec = normalize(light_dir + view_dir);
    float spec = pow(max(dot(world_normal, half_vec), 0.0), 32.0);
    vec3 specular = u_light_color * spec * (1.0 - metallic);
    
    vec3 ambient = albedo.rgb * 0.1;
    vec3 color = ambient + (diffuse + specular) * u_light_color;
    
    out_color = vec4(color, albedo.a);
}
```

---

## Appendix B: Integration Checklist

### B.1 Engine Integration Requirements

```
□ Graphics API abstraction (D3D12, Vulkan, Metal, GL)
□ Resource management (textures, buffers, pipelines)
□ Command buffer submission
□ Sync primitives (fences, semaphores) for physics/render thread coordination
□ Profiler hooks (CPU/GPU timing)
□ Hot-reload support for shaders and materials
```

### B.2 APC-Specific Hooks

```
□ PhysicsRenderInterface initialization before first physics step
□ Snapshot acquisition at render thread start
□ Bone matrix upload after pose composition
□ Contact query for VFX triggers
□ ImpactStyleProfile lookup for VFX parameters
□ Hit-stop integration in render loop
□ Time scale application to interpolation
```

---

## Appendix C: Risk Register

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| GPU skinning too slow for 22 chars | Low | High | Fallback to CPU skinning; aggressive LOD |
| Physics/render thread contention | Medium | Medium | Increase buffer lag; lock-free data structures |
| Hit-stop causes stuttering | Medium | Medium | Cap at 100ms; smooth ramp in/out |
| VFX particle count explodes | Medium | Low | Hard limits per impact; distance culling |
| Memory bandwidth bottleneck | Low | Medium | Compress bone matrices; instanced draws |
| Cross-platform shader compilation issues | High | Medium | Early console testing; SPIR-V intermediate |

---

**Document Status:** Initial Specification  
**Last Updated:** [Current Date]  
**Author:** Lead Developer, APC Project  

This document will be updated weekly during Phase R0 and bi-weekly thereafter. All changes tracked in project wiki.
