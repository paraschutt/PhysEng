// ============================================================================
// DSPE CCD — Swept Sphere Implementation
// Detects tunnelling: ball at 30 m/s, substep 4.166ms → 0.125m/substep
// ============================================================================
#include "dspe/systems/ccd.h"
#include <algorithm>
#include <limits>
#include <cmath>

namespace dspe {

// Convert Vec3Pos to Vec3Vel
static Vec3Vel pv(Vec3Pos p) {
    return { FpVel{p.x.raw << 8}, FpVel{p.y.raw << 8}, FpVel{p.z.raw << 8} };
}

// ============================================================================
// Quadratic solver: find smallest positive t for at² + bt + c = 0
// Returns FpVel{-1} if no positive root
// ============================================================================
FpVel ContinuousCollisionDetection::solve_quadratic_min_positive(
    FpVel a_fp, FpVel b_fp, FpVel c_fp) {

    // Work in float for numerical stability of discriminant
    float a = a_fp.to_float();
    float b = b_fp.to_float();
    float c = c_fp.to_float();

    if (fabsf(a) < 1e-8f) {
        // Linear: t = -c/b
        if (fabsf(b) < 1e-8f) return FpVel{-1};
        float t = -c / b;
        return t >= 0.0f ? FpVel::from_float(t) : FpVel{-1};
    }

    float disc = b*b - 4.0f*a*c;
    if (disc < 0.0f) return FpVel{-1};

    float sq  = sqrtf(disc);
    float t1  = (-b - sq) / (2.0f * a);
    float t2  = (-b + sq) / (2.0f * a);

    if (t1 >= 0.0f) return FpVel::from_float(t1);
    if (t2 >= 0.0f) return FpVel::from_float(t2);
    return FpVel{-1};
}

// ============================================================================
// Swept sphere vs capsule
// Returns TOI ∈ [0,1] or FpVel{-1} if no hit within interval
// ============================================================================
FpVel ContinuousCollisionDetection::swept_sphere_capsule(
    Vec3Vel sphere_start, Vec3Vel sphere_vel, FpVel sphere_r,
    Vec3Vel cap_base,     Vec3Vel cap_tip,   FpVel cap_r,
    Vec3Vel& out_normal) {

    FpVel sum_r = sphere_r + cap_r;
    FpVel sum_r2= sum_r * sum_r;

    Vec3Vel d   = cap_tip - cap_base;   // Capsule axis
    FpVel   d2  = d.dot(d);
    FpVel   min_toi = FpVel{-1};

    // Test sphere vs capsule as: min distance from swept-sphere line to segment
    // Reduces to: test swept sphere vs infinite cylinder, then vs end caps

    // Check if the initial position already overlaps (handled by discrete detection)
    Vec3Vel s0_to_base = sphere_start - cap_base;

    // ── vs infinite cylinder ──────────────────────────────────────────────
    if (d2.raw > 0) {
        // Project sphere motion onto cylinder
        // e = v - (v·d/|d|^2)*d  (component of motion perpendicular to axis)
        // Using the parametric form: |c0 + e*t|^2 = sum_r^2
        Vec3Vel v    = sphere_vel;
        FpVel   v_d  = v.dot(d);
        FpVel   s_d  = s0_to_base.dot(d);

        // Perpendicular components
        // e = v - (v_d/d2)*d   m = s0_to_base - (s_d/d2)*d
        // Int64 division for cross products
        auto proj = [&](Vec3Vel vec, FpVel scalar, FpVel len2) -> Vec3Vel {
            if (len2.raw == 0) return vec;
            int64_t s64 = (static_cast<int64_t>(scalar.raw) << 16) / len2.raw;
            FpVel t{static_cast<int32_t>(s64)};
            return vec - d * t;
        };

        Vec3Vel e = proj(v, v_d, d2);
        Vec3Vel m = proj(s0_to_base, s_d, d2);

        FpVel a_c = e.dot(e);
        FpVel b_c = m.dot(e) * FpVel::from_int(2);
        FpVel c_c = m.dot(m) - sum_r2;

        FpVel t_cyl = solve_quadratic_min_positive(a_c, b_c, c_c);
        if (t_cyl.raw >= 0 && t_cyl <= FpVel::one()) {
            // Check that hit point lies within capsule segment (0 ≤ s ≤ 1)
            Vec3Vel hit_pt = sphere_start + sphere_vel * t_cyl;
            Vec3Vel hit_to_base = hit_pt - cap_base;
            FpVel   seg_t_num = hit_to_base.dot(d);
            if (seg_t_num.raw >= 0 && seg_t_num <= d2) {
                min_toi = t_cyl;
                // Compute normal at hit
                int64_t s64 = (static_cast<int64_t>(seg_t_num.raw) << 16) / d2.raw;
                FpVel seg_t{static_cast<int32_t>(s64)};
                Vec3Vel hit_on_cap = cap_base + d * seg_t;
                out_normal = (hit_pt - hit_on_cap).normalized();
            }
        }
    }

    // ── vs end caps (sphere-sphere test) ─────────────────────────────────
    auto test_end_cap = [&](Vec3Vel cap_end) {
        Vec3Vel m2 = sphere_start - cap_end;
        FpVel a2 = sphere_vel.dot(sphere_vel);
        FpVel b2 = m2.dot(sphere_vel) * FpVel::from_int(2);
        FpVel c2 = m2.dot(m2) - sum_r2;
        FpVel t_cap = solve_quadratic_min_positive(a2, b2, c2);
        if (t_cap.raw >= 0 && t_cap <= FpVel::one()) {
            if (min_toi.raw < 0 || t_cap < min_toi) {
                min_toi = t_cap;
                Vec3Vel hit_pt = sphere_start + sphere_vel * t_cap;
                out_normal = (hit_pt - cap_end).normalized();
            }
        }
    };

    test_end_cap(cap_base);
    test_end_cap(cap_tip);

    return min_toi;
}

// ============================================================================
// Swept sphere vs AABB box
// ============================================================================
FpVel ContinuousCollisionDetection::swept_sphere_box(
    Vec3Vel sphere_start, Vec3Vel sphere_vel, FpVel sphere_r,
    const AABB& box, Vec3Vel& out_normal) {

    // Expand box by sphere radius on all sides, then ray test against expanded box
    // (Minkowski sum of sphere and box = rounded box; approximate with expanded AABB)
    Vec3Vel box_min = pv(box.min_pt) - Vec3Vel{sphere_r, sphere_r, sphere_r};
    Vec3Vel box_max = pv(box.max_pt) + Vec3Vel{sphere_r, sphere_r, sphere_r};

    // Slab intersection test (ray vs AABB)
    FpVel t_min{0};
    FpVel t_max = FpVel::one();
    Vec3Vel normal{};

    auto test_axis = [&](FpVel origin, FpVel vel, FpVel lo, FpVel hi, int axis) -> bool {
        FpVel eps = FpVel::from_float(1e-6f);
        if (vel.abs() < eps) {
            return (origin >= lo && origin <= hi);
        }
        FpVel inv = FpVel::one() / vel;
        FpVel t1  = (lo - origin) * inv;
        FpVel t2  = (hi - origin) * inv;
        if (t1 > t2) { FpVel tmp = t1; t1 = t2; t2 = tmp; }
        if (t1 > t_min) {
            t_min = t1;
            normal = Vec3Vel::zero();
            (&normal.x)[axis] = vel.raw < 0 ? FpVel::one() : -FpVel::one();
        }
        t_max = t_max < t2 ? t_max : t2;
        return t_min <= t_max;
    };

    if (!test_axis(sphere_start.x, sphere_vel.x, box_min.x, box_max.x, 0)) return FpVel{-1};
    if (!test_axis(sphere_start.y, sphere_vel.y, box_min.y, box_max.y, 1)) return FpVel{-1};
    if (!test_axis(sphere_start.z, sphere_vel.z, box_min.z, box_max.z, 2)) return FpVel{-1};

    if (t_min.raw < 0 || t_min > FpVel::one()) return FpVel{-1};

    out_normal = normal;
    return t_min;
}

// ============================================================================
// CCD threshold check
// ============================================================================
bool ContinuousCollisionDetection::needs_ccd(const Entity& e, FpVel dt) {
    if (!e.has(COMP_RIGIDBODY | COMP_COLLIDER)) return false;
    if (e.has(COMP_STATIC)) return false;
    if (is_ball(e.skeleton.entity_id)) return true; // ball always CCD

    FpVel speed = e.rigidbody.velocity.length();
    FpVel displacement = speed * dt;

    FpVel threshold;
    if (e.collider.shape_type == ShapeType::SPHERE) {
        // Convert FpPos radius to FpVel: Q24.8 -> Q15.16 (shift left 8)
        threshold = FpVel{e.collider.sphere.radius.raw << 8} * FpVel::from_float(0.5f);
    } else {
        threshold = FpVel{e.collider.capsule.radius.raw << 8} * FpVel::from_float(0.5f);
    }

    return displacement > threshold;
}

void ContinuousCollisionDetection::flag_ccd_entities(
    std::array<Entity, MAX_ENTITIES>& entities, FpVel dt) {
    for (EntityId id = 0; id < MAX_ENTITIES; ++id) {
        if (needs_ccd(entities[id], dt)) {
            entities[id].add(COMP_CCD);
        } else {
            entities[id].remove(COMP_CCD);
        }
    }
    // Ball always CCD
    entities[BALL_ENTITY].add(COMP_CCD);
}

// ============================================================================
// Sweep a single entity against all static colliders
// ============================================================================
CCDHit ContinuousCollisionDetection::sweep_entity(
    const Entity& mover,
    const std::array<Entity, MAX_ENTITIES>& entities,
    EntityId skip_id,
    FpVel substep_dt) const {

    CCDHit best;
    best.toi = FpVel::one();

    if (!mover.has(COMP_RIGIDBODY | COMP_COLLIDER)) return best;

    FpVel sphere_r;
    Vec3Vel sphere_start = pv(mover.rigidbody.position);

    if (mover.collider.shape_type == ShapeType::SPHERE) {
        sphere_r = FpVel{mover.collider.sphere.radius.raw << 8};
    } else {
        sphere_r = FpVel{mover.collider.capsule.radius.raw << 8};
    }

    // Displacement over substep
    Vec3Vel sphere_vel = mover.rigidbody.velocity * substep_dt;

    for (EntityId id = 0; id < MAX_ENTITIES; ++id) {
        if (id == skip_id) continue;
        const Entity& other = entities[id];
        if (!other.has(COMP_COLLIDER)) continue;
        if (!other.has(COMP_STATIC) && !other.has(COMP_RIGIDBODY)) continue;

        Vec3Vel hit_normal{};
        FpVel   toi{-1};

        if (other.collider.shape_type == ShapeType::CAPSULE) {
            Vec3Vel cap_centre = pv(other.rigidbody.position);
            Vec3Vel base_l = pv(other.collider.capsule.local_base);
            Vec3Vel tip_l  = pv(other.collider.capsule.local_tip);
            Vec3Vel base_w = cap_centre + other.rigidbody.orientation.rotate(base_l);
            Vec3Vel tip_w  = cap_centre + other.rigidbody.orientation.rotate(tip_l);
            FpVel cap_r = FpVel{other.collider.capsule.radius.raw << 8};

            toi = swept_sphere_capsule(sphere_start, sphere_vel, sphere_r,
                                        base_w, tip_w, cap_r, hit_normal);

        } else if (other.collider.shape_type == ShapeType::BOX) {
            toi = swept_sphere_box(sphere_start, sphere_vel, sphere_r,
                                    other.collider.world_aabb, hit_normal);

        } else if (other.collider.shape_type == ShapeType::SPHERE) {
            // Sphere vs sphere — use analytical approach
            Vec3Vel other_centre = pv(other.rigidbody.position);
            FpVel other_r = FpVel{other.collider.sphere.radius.raw << 8};
            Vec3Vel m2 = sphere_start - other_centre;
            FpVel sum_r = sphere_r + other_r;
            FpVel a2 = sphere_vel.dot(sphere_vel);
            FpVel b2 = m2.dot(sphere_vel) * FpVel::from_int(2);
            FpVel c2 = m2.dot(m2) - sum_r * sum_r;
            toi = solve_quadratic_min_positive(a2, b2, c2);
            if (toi.raw >= 0) {
                Vec3Vel hit_pt = sphere_start + sphere_vel * toi;
                hit_normal = (hit_pt - other_centre).normalized();
            }
        }

        if (toi.raw >= 0 && toi < best.toi) {
            best.toi           = toi;
            best.normal        = hit_normal;
            best.contact_point = sphere_start + sphere_vel * toi;
            best.other_entity  = id;
            best.valid         = true;
        }
    }

    return best;
}

// ============================================================================
// Test all CCD entities
// ============================================================================
void ContinuousCollisionDetection::test_all(
    std::array<Entity, MAX_ENTITIES>& entities,
    const BroadPhase& broad_phase,
    FpVel substep_dt,
    std::vector<std::pair<EntityId, CCDHit>>& hits_out) {

    // Process in deterministic entity ID order
    for (EntityId id = 0; id < MAX_ENTITIES; ++id) {
        Entity& e = entities[id];
        if (!e.has(COMP_CCD | COMP_RIGIDBODY)) continue;
        if (e.has(COMP_SLEEP | COMP_STATIC)) continue;

        CCDHit hit = sweep_entity(e, entities, id, substep_dt);
        if (hit.valid) {
            hits_out.push_back({id, hit});
        }
    }

    // Sort by TOI for processing earliest hits first
    std::sort(hits_out.begin(), hits_out.end(),
              [](const auto& a, const auto& b) {
                  return a.second.toi < b.second.toi;
              });
}

} // namespace dspe